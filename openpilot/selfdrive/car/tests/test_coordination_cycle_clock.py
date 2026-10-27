"""Producer pose inputs must be frozen before the common control clock is sampled."""

from types import SimpleNamespace as NS
import pytest
from opendbc.car import structs
from openpilot.cereal import log
from openpilot.selfdrive.car import longitudinal_context as lc
from openpilot.selfdrive.locationd.helpers import PoseCalibrator


class Inputs(dict):
  def __init__(self, source, *, healthy=True):
    dm = log.DeviceMotion.new_message(timestamp=source, inputsOK=True, sensorsOK=healthy, posenetOK=True)
    dm.orientationNED.valid = True
    super().__init__(deviceMotion=dm, extrinsicsCalibration=log.ExtrinsicsCalibration.new_message(calStatus='calibrated', rpyCalib=[0.0, 0.0, 0.0]))
    self.seen = dict.fromkeys(self, True)
    self.valid = dict(self.seen)
    self.logMonoTime = dict.fromkeys(self, source)
    self.updates = 0

  def update(self, *args):
    self.updates += 1


def test_drain_happens_before_clock_sample():
  seen = []
  sm = NS(update=lambda timeout: seen.append('drain'))

  def clock():
    seen.append('clock')
    return 20

  assert lc.begin_coordination_cycle(sm, clock_ns=clock) == 20
  assert seen == ['drain', 'clock']


@pytest.mark.parametrize('early,source,after', [(240272541591912, 240272541685661, 240272542000000), (240272491671916, 240272491990037, 240272492200000)])
def test_recorded_healthy_pose_race_uses_post_drain_clock(early, source, after):
  sm = Inputs(source)
  now = lc.begin_coordination_cycle(sm, clock_ns=lambda: after)
  cc = structs.CarControl(enabled=True, longActive=True)
  q = cc.navigatorLongitudinal
  q.enabled = True
  q.producerMonoTime = now
  q.poseValid = True
  lc.refresh_coordination_pose(cc, sm, PoseCalibrator(), drain=False)
  assert sm.updates == 1 and early < source <= q.producerMonoTime and q.poseValid
  assert q.poseMonoTime == source


@pytest.mark.parametrize('case', ['future', 'unhealthy', 'producer_invalid'])
def test_post_drain_snapshot_does_not_rescue_bad_pose(case):
  now = 1_000_000_000
  sm = Inputs(now + 1 if case == 'future' else now - 1_000_000, healthy=case != 'unhealthy')
  stamp = lc.begin_coordination_cycle(sm, clock_ns=lambda: now)
  cc = structs.CarControl(enabled=True, longActive=True)
  q = cc.navigatorLongitudinal
  q.enabled = True
  q.producerMonoTime = stamp
  q.poseValid = case != 'producer_invalid'
  lc.refresh_coordination_pose(cc, sm, PoseCalibrator(), drain=False)
  assert not q.poseValid and q.poseMonoTime == sm['deviceMotion'].timestamp


def test_replay_clock_is_not_advanced_to_pose_source():
  sm = Inputs(1_000_000_001)
  assert lc.begin_coordination_cycle(sm, replay_ns=1_000_000_000, clock_ns=lambda: 1_500_000_000) == 1_000_000_000
  assert sm.updates == 1


@pytest.mark.parametrize('case', ['healthy', 'future', 'unhealthy', 'producer_invalid', 'stale', 'replay'])
def test_real_producer_cycle_freezes_pose_before_timestamp(case):
  import subprocess
  import sys

  code = f'''import pytest
from openpilot.selfdrive.car.tests.test_coordination_cycle_clock import _exercise_producer_clock
with pytest.MonkeyPatch.context() as patch:
  _exercise_producer_clock(patch, {case!r})
'''
  subprocess.run([sys.executable, '-c', code], check=True, timeout=30)


def _exercise_producer_clock(monkeypatch, case):
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.ford.values import CAR
  from opendbc.car.vehicle_model import VehicleModel
  from openpilot.selfdrive.car.tests.test_high_speed_pipeline import SM
  from openpilot.selfdrive.controls import controlsd
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
  from opendbc.car.ford.longitudinal_coordination import GradeLease

  monkeypatch.delenv('REPLAY', raising=False)
  early, source = 240272541591912, 240272541685661
  after = source + 1_000_000
  selected_source = after + 1 if case == 'future' else early - 110_000_000 if case == 'stale' else source
  if case == 'replay':
    monkeypatch.setenv('REPLAY', '1')
  clock = [early]
  monkeypatch.setattr(controlsd, 'time', NS(monotonic_ns=lambda: clock[0]))
  secondary = Inputs(selected_source, healthy=case != 'unhealthy')
  secondary.logMonoTime = dict.fromkeys(secondary, early + 100_000)

  def drain(_):
    secondary.updates += 1
    assert secondary.updates == 1, 'Publisher drained after producer clock was sampled'
    clock[0] = after

  secondary.update = drain
  primary = Inputs(early - (200_000_000 if case == 'stale' else 20_000_000))
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.openpilotLongitudinalControl = True
  state = structs.CarState(vEgo=9.12, vEgoRaw=9.12, canValid=True)
  params = log.VehicleParameters.new_message(steerRatio=22.5, stiffnessFactor=1.0)
  sm = SM(
    primary,
    carState=state,
    vehicleParameters=params,
    modelV2=log.ModelDataV2.new_message(),
    selfdriveState=log.SelfdriveState.new_message(enabled=True, state='enabled'),
    driverMonitoringState=log.DriverMonitoringState.new_message(),
    onroadEvents=[],
    longitudinalPlan=log.LongitudinalPlan.new_message(aTarget=-0.209495, shouldStop=False),
    lateralDelay=NS(lateralDelay=0.2),
    carOutput=NS(actuatorsOutput=structs.CarControl.Actuators()),
  )
  sm.valid = dict.fromkeys(sm, True) | {'lateralManeuverPlan': False, 'driverAssistance': False}
  sm.seen = dict.fromkeys(sm, True)
  sm.logMonoTime = dict.fromkeys(sm, early - 10_000_000)
  c = controlsd.Controls.__new__(controlsd.Controls)
  c.CP, c.sm, c.VM = cp, sm, VehicleModel(cp)
  c.CI = NS(get_pid_accel_limits=lambda *args: (-3.5, 2.0))
  c.LaC = LatControlAngle(cp, c.CI, 0.01)
  c.LoC = LongControl(cp)
  c.LoC.long_control_state = structs.CarControl.Actuators.LongControlState.pid
  c.longitudinal_context = lc.LongitudinalContext(True)
  c.coordination_pose_sm = secondary
  c.coordination_pose_calibrator = PoseCalibrator()
  c.coordination_pose_valid = case != 'producer_invalid'
  c.calibrated_pose = None
  c.curvature = c.desired_curvature = 0.0
  c.steer_limited_by_safety = False
  captured = {}
  c.pm = NS(send=lambda topic, event: captured.update({topic: event.to_bytes()}))
  cc, lac = c.state_control()
  c.publish(cc, lac)
  with log.Event.from_bytes(captured['carControl']) as event:
    q = event.carControl.navigatorLongitudinal
    assert q.poseMonoTime == selected_source
    assert q.producerMonoTime == (sm.logMonoTime['carState'] if case == 'replay' else after)
    result = GradeLease(True).update(q.producerMonoTime / 1e9, 0.0, q.poseMonoTime / 1e9, q.poseValid, True, state.vEgo, 'pid')
    if case == 'healthy':
      assert q.poseValid and early < source <= q.producerMonoTime
      assert result.reason != 'invalid_pose_disengage'
    else:
      assert result.reason == 'invalid_pose_disengage'
      if case != 'stale':
        assert not q.poseValid
  assert secondary.updates == 1
