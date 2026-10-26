"""Actual producer/card method integration; capture apply/send cannot touch a bus."""

import math
import subprocess
import sys
from types import SimpleNamespace as NS
from opendbc.car import structs
from opendbc.car.vehicle_model import VehicleModel
from openpilot.cereal import log
from openpilot.selfdrive.car.tracking_context import create_tracking_trial
from openpilot.selfdrive.car.motion_context import MotionContext
from openpilot.selfdrive.car.tests.test_high_speed_context import params
from openpilot.selfdrive.car.tests.test_motion_context import calibration, motion
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle


class SM(dict):
  def all_checks(self, topics):
    return all(self.valid.get(topic, False) for topic in topics)

  def all_alive(self, topics):
    return True


def test_actual_producer_card_motion_and_invalid_calibration_release():
  # Process modules capture REPLAY at import. Keep their live-mode imports out
  # of the suite process, whose later replay tests fork and reuse module caches.
  code = '''import pytest
from openpilot.selfdrive.car.tests.test_high_speed_pipeline import _run_pipeline
with pytest.MonkeyPatch.context() as patch:
  _run_pipeline(patch)
'''
  subprocess.run([sys.executable, '-c', code], check=True, timeout=30)


def _run_pipeline(monkeypatch):
  from openpilot.selfdrive.car.card import Car
  from openpilot.selfdrive.controls.controlsd import Controls
  from openpilot.selfdrive.car import card as card_module
  from openpilot.selfdrive.controls import controlsd as controls_module

  cp = params()
  trial = create_tracking_trial(cp, enabled=True, roll_mapping_enabled=True, high_speed_enabled=True)
  v = 83 * 0.44704
  k = 0.00065
  clock = [100_000_000_000]
  monkeypatch.setattr(card_module, 'time', NS(monotonic=lambda: clock[0] / 1e9))
  monkeypatch.setattr(controls_module, 'time', NS(monotonic_ns=lambda: clock[0] - 2_000_000))
  cs = structs.CarState.new_message()
  cs.vEgo = v
  cs.vEgoRaw = v
  cs.canValid = True
  lp = log.VehicleParameters.new_message()
  lp.valid = True
  lp.sensorValid = True
  lp.angleOffsetValid = True
  lp.steerRatioValid = True
  lp.stiffnessFactorValid = True
  lp.steerRatio = 22.5
  lp.stiffnessFactor = 1.0
  lp.roll = 0.0
  lp.angleOffsetDeg = -1.5
  model = log.ModelDataV2.new_message()
  model.action.desiredCurvature = k
  producer = SM(
    carState=cs,
    vehicleParameters=lp,
    longitudinalPlan=log.LongitudinalPlan.new_message(aTarget=0.0, shouldStop=False),
    modelV2=model,
    selfdriveState=log.SelfdriveState.new_message(enabled=True, active=True, state='enabled'),
    driverMonitoringState=log.DriverMonitoringState.new_message(),
    onroadEvents=[],
    lateralDelay=NS(lateralDelay=0.2),
  )
  producer.valid = {'lateralManeuverPlan': False, 'vehicleParameters': True, 'carState': True}
  controls = Controls.__new__(Controls)
  from openpilot.selfdrive.car.longitudinal_context import LongitudinalContext
  controls.longitudinal_context = LongitudinalContext(False)
  controls.CP = cp
  controls.sm = producer
  controls.VM = VehicleModel(cp)
  controls.desired_curvature = k
  controls.steer_limited_by_safety = False
  controls.CI = NS(get_pid_accel_limits=lambda *args: (-1.0, 1.0))
  controls.LaC = LatControlAngle(cp, controls.CI, 0.01)
  controls.LoC = NS(long_control_state=structs.CarControl.Actuators.LongControlState.off, reset=lambda: None, update=lambda *args, **kwargs: 0.0)
  card = Car.__new__(Car)
  card.initialized_prev = True
  card.longitudinal_coordination_enabled = False
  card.tracking_trial = trial
  card.tracking_motion = MotionContext()
  consumer = SM()
  card.sm = consumer
  sent = []
  card.pm = NS(send=lambda topic, data: sent.append(topic))

  def apply(control, now):
    trial.update(control, NS(out=cs), control.actuators)
    return control.actuators, []

  card.CI = NS(apply=apply)
  for _i in range(90):
    clock[0] += 50_000_000
    now = clock[0]
    producer.logMonoTime = {'vehicleParameters': now - 15_000_000, 'carState': now - 10_000_000}
    cc, _ = controls.state_control()
    cs.steeringAngleDeg = cc.actuators.steeringAngleDeg + math.copysign(2.4, cc.actuators.steeringAngleDeg + 1.5)
    ce = calibration(t=now - 40_000_000)
    dm = motion(t=now - 5_000_000, source=now - 15_000_000)
    dm.deviceMotion.angularVelocityDevice.z = k * v * 1.35
    consumer.update(modelV2=model, deviceMotion=dm.deviceMotion, extrinsicsCalibration=ce.extrinsicsCalibration)
    consumer.seen = consumer.updated = {'deviceMotion': True, 'extrinsicsCalibration': True}
    consumer.valid = {'carControl': True, 'modelV2': True, 'deviceMotion': True, 'extrinsicsCalibration': True}
    consumer.logMonoTime = {
      'carControl': now - 1_000_000,
      'modelV2': now - 4_000_000,
      'deviceMotion': int(dm.logMonoTime),
      'extrinsicsCalibration': int(ce.logMonoTime),
    }
    consumer.recv_time = {'deviceMotion': (now - 3_000_000) / 1e9, 'extrinsicsCalibration': (now - 3_000_000) / 1e9}
    card.tracking_state_ns = now - 1_000_000
    card.controls_update(cs, cc)
  assert trial.reduction > 0 and trial.body_reason == 'confirmed'
  assert trial.motion[2] > consumer.logMonoTime['deviceMotion']
  assert set(sent) == {'sendcan'}  # capture function only, never an actual endpoint
  # No new motion message: invalidate calibration and immediately withdraw.
  clock[0] += 50_000_000
  now = clock[0]
  producer.logMonoTime = {'vehicleParameters': now - 15_000_000, 'carState': now - 10_000_000}
  cc, _ = controls.state_control()
  ce = calibration(t=now - 5_000_000, status='invalid')
  consumer['extrinsicsCalibration'] = ce.extrinsicsCalibration
  consumer.updated = {'extrinsicsCalibration': True, 'deviceMotion': False}
  consumer.logMonoTime.update(carControl=now - 1_000_000, modelV2=now - 4_000_000, extrinsicsCalibration=int(ce.logMonoTime))
  consumer.recv_time['extrinsicsCalibration'] = (now - 3_000_000) / 1e9
  card.tracking_state_ns = now - 1_000_000
  card.controls_update(cs, cc)
  assert trial.reduction == 0 and trial.final_residual == 0
