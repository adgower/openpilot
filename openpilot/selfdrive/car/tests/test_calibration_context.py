import math
from types import SimpleNamespace as NS
import pytest
from opendbc.car import structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, FordFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.ford.lateral_angle_roll_mapping import RollAwareMapping
from openpilot.selfdrive.car.calibration_context import stamp_calibration, feed_calibration_context


def setup():
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.flags |= int(FordFlags.PINION_CURVATURE | FordFlags.LOW_SPEED_GAIN)
  c = RollAwareMapping(cp, enabled=True)
  vm = VehicleModel(cp)
  vm.update_params(1.0, 22.5)
  return c, vm


def packet(vm, now, k=0.003, roll=0.04, speed=60 * 0.44704):
  cc = structs.CarControl.new_message()
  cc.latActive = True
  cc.actuators.curvature = k
  cc.actuators.steeringAngleDeg = math.degrees(vm.get_steer_from_curvature(-cc.actuators.curvature, speed, roll)) - 1.5
  lp = NS(
    valid=True,
    sensorValid=True,
    angleOffsetValid=True,
    steerRatioValid=True,
    stiffnessFactorValid=True,
    roll=roll,
    steerRatio=22.5,
    stiffnessFactor=1.0,
    angleOffsetDeg=-1.5,
  )
  stamp_calibration(cc, lp, now_ns=now - 2_000_000, source_ns=now - 15_000_000, state_ns=now - 10_000_000, speed=speed, valid=True)
  return cc


def apply(c, cc, now, control_ns=None, **kw):
  control_ns = now - 1_000_000 if control_ns is None else control_ns
  c.set_context(now_ns=now, state_ns=now, control_ns=control_ns, model_ns=now, model_valid=True, lane_change=False)
  feed_calibration_context(c, cc, now_ns=now, control_ns=control_ns, control_valid=True, **kw)
  state = NS(
    vEgoRaw=60 * 0.44704,
    vEgo=59 * 0.44704,
    yawRate=0.0,
    steeringAngleDeg=0.0,
    steeringPressed=False,
    canValid=True,
    steerFaultTemporary=False,
    steerFaultPermanent=False,
    leftBlinker=False,
    rightBlinker=False,
  )
  return c.update(cc, NS(out=state), cc.actuators)


def test_atomic_wire_snapshot_and_producer_speed():
  c, vm = setup()
  for i in range(50):
    now = 2_000_000_000 + i * 50_000_000
    cc = packet(vm, now)
    with structs.CarControl.from_bytes(cc.to_bytes()) as decoded:
      apply(c, decoded, now)
  assert c.map_reason == 'mapped' and c.map_delta != 0
  assert c.target_speed == pytest.approx(60 * 0.44704, abs=1e-5)


@pytest.mark.parametrize(
  'case',
  [
    'legacy',
    'source_future',
    'compute_future',
    'state_future',
    'source_stale',
    'control_stale',
    'computed_stale',
    'invalid',
    'mismatched_curvature',
    'mismatched_angle',
    'nan_speed',
  ],
)
def test_bad_snapshot_falls_back(case):
  c, vm = setup()
  now = 2_000_000_000
  cc = packet(vm, now)
  p = cc.navigatorCalibration
  control_ns = now - 1_000_000
  if case == 'legacy':
    cc = structs.CarControl.new_message()
    cc.latActive = True
    cc.actuators.curvature = 0.003
  if case == 'source_future':
    p.sourceMonoTime = now + 1
  if case == 'compute_future':
    p.computedMonoTime = now + 1
  if case == 'state_future':
    p.stateMonoTime = now + 1
  if case == 'source_stale':
    p.sourceMonoTime = now - 200_000_000
  if case == 'control_stale':
    control_ns = now - 150_000_000
  if case == 'computed_stale':
    p.computedMonoTime = now - 150_000_000
    p.sourceMonoTime = now - 160_000_000
    p.stateMonoTime = now - 155_000_000
  if case == 'invalid':
    p.valid = False
  if case == 'mismatched_curvature':
    cc.actuators.curvature = -0.003
  if case == 'mismatched_angle':
    cc.actuators.steeringAngleDeg += 2
  if case == 'nan_speed':
    p.targetSpeed = float('nan')
  apply(c, cc, now, control_ns)
  assert c.map_delta == 0


def test_reordered_control_is_rejected_without_poisoning_next_packet():
  c, vm = setup()
  now = 2_000_000_000
  for i in range(30):
    apply(c, packet(vm, now + i * 50_000_000), now + i * 50_000_000)
  apply(c, packet(vm, now + 30 * 50_000_000), now + 30 * 50_000_000, control_ns=now + 28 * 50_000_000)
  assert c.map_delta == 0
  apply(c, packet(vm, now + 31 * 50_000_000), now + 31 * 50_000_000)
  assert c.calibration[2]


def test_release_blocks_rearm_until_final_baseline_rejoins():
  c, vm = setup()
  for i in range(50):
    now = 2_000_000_000 + i * 50_000_000
    apply(c, packet(vm, now), now)
  assert c.map_delta != 0
  now += 50_000_000
  cc = packet(vm, now)
  cc.navigatorCalibration.valid = False
  out = apply(c, cc, now)
  assert c.map_delta == 0
  assert abs(out.path_angle - c.baseline.path_angle_last) <= 0.01 + 1e-12
  for _i in range(10):
    now += 50_000_000
    out = apply(c, packet(vm, now), now)
    if c.release_pending:
      assert c.map_delta == 0
  assert not c.release_pending


def test_old_compiled_registry_preserves_default_off():
  from openpilot.selfdrive.car.calibration_context import roll_mapping_opt_in
  from openpilot.common.params import UnknownKeyName

  class OldRegistry:
    def get_bool(self, key):
      raise UnknownKeyName(key)

  assert not roll_mapping_opt_in(OldRegistry())
  assert roll_mapping_opt_in(NS(get_bool=lambda key: True))
  assert not roll_mapping_opt_in(NS(get_bool=lambda key: False))


def test_repeated_control_expires_and_new_control_recovers():
  c, vm = setup()
  start = 2_000_000_000
  cc = packet(vm, start)
  apply(c, cc, start)
  for i in range(1, 10):
    apply(c, cc, start + i * 50_000_000, control_ns=start - 1_000_000)
  assert not c.calibration[2] and c.map_delta == 0
  now = start + 500_000_000
  apply(c, packet(vm, now), now)
  assert c.calibration[2]


def test_future_control_does_not_poison_valid_highwater():
  c, vm = setup()
  now = 2_000_000_000
  apply(c, packet(vm, now), now, control_ns=now + 10_000_000_000)
  assert not c.calibration[2]
  now += 50_000_000
  apply(c, packet(vm, now), now)
  assert c.calibration[2]


def test_release_debt_blocks_rearm_while_baseline_moves_at_rate_limit():
  c, vm = setup()
  now = 2_000_000_000
  for _i in range(50):
    now += 50_000_000
    apply(c, packet(vm, now), now)
  assert c.path_angle_last < c.baseline.path_angle_last
  from opendbc.car.ford.lateral_angle import AngleLateralResult

  def moving_baseline(*args):
    c.baseline.path_angle_last += 0.009
    return AngleLateralResult(path_angle=c.baseline.path_angle_last, shadow_curvature=0.003)

  c.baseline.update = moving_baseline
  for i in range(12):
    now += 50_000_000
    cc = packet(vm, now)
    if i == 0:
      cc.navigatorCalibration.valid = False
    out = apply(c, cc, now)
    assert c.release_pending and c.map_delta == 0
    assert abs(out.path_angle - c.baseline.path_angle_last) <= 0.01 + 1e-12
  c.baseline.update = lambda *args: AngleLateralResult(path_angle=c.baseline.path_angle_last, shadow_curvature=0.003)
  for _i in range(2):
    now += 50_000_000
    out = apply(c, packet(vm, now), now)
  assert not c.release_pending


def test_rebuilt_params_registry_with_real_temporary_store(tmp_path):
  from openpilot.common.params import Params
  from openpilot.selfdrive.car.calibration_context import roll_mapping_opt_in

  params = Params(str(tmp_path))
  assert not roll_mapping_opt_in(params)
  params.put_bool('FordPrefRollAwareMapping', True, block=True)
  assert roll_mapping_opt_in(params)
  params.put_bool('FordPrefRollAwareMapping', False, block=True)
  assert not roll_mapping_opt_in(params)
