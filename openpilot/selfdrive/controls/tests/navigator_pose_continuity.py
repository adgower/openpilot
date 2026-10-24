"""Healthy calibration publications must not break moving longitudinal ownership."""
from collections import defaultdict
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import log, messaging
from openpilot.selfdrive.car.longitudinal_context import LongitudinalContext, coordination_event
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.locationd.helpers import PoseCalibrator
from opendbc.car import structs
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.longitudinal_coordination import Torque
from opendbc.car.ford.values import CAR, DBC


class Inputs(dict):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.logMonoTime = {'deviceMotion': 990_000_000, 'extrinsicsCalibration': 990_000_000}

  def update(self, *_args):
    pass  # IPC alone is replaced: real update, calibration, publish and controller run.


class PosePipeline:
  def __init__(self, enabled=True, seen_motion=True):
    cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
    cp.openpilotLongitudinalControl = True
    self.cp = cp
    self.now = 1_000_000_000
    motion = log.DeviceMotion.new_message(timestamp=950_000_000, sensorsOK=True, inputsOK=True, posenetOK=True)
    motion.orientationNED.valid = True
    motion.orientationNED.y = .02
    calibration = log.ExtrinsicsCalibration.new_message(calStatus='calibrated', rpyCalib=[0., .01, 0.])
    self.sm = Inputs(deviceMotion=motion, extrinsicsCalibration=calibration,
                     carState=structs.CarState(vEgo=8., vEgoRaw=8., canValid=True),
                     longitudinalPlan=NS(shouldStop=False, hasLead=False),
                     selfdriveState=log.SelfdriveState.new_message(),
                     driverMonitoringState=log.DriverMonitoringState.new_message(),
                     carOutput=NS(actuatorsOutput=structs.CarControl.Actuators()))
    self.sm.updated = {'deviceMotion': seen_motion, 'extrinsicsCalibration': True}
    self.sm.seen = {'deviceMotion': seen_motion, 'extrinsicsCalibration': True}
    self.sm.valid = {'deviceMotion': True, 'extrinsicsCalibration': True, 'driverAssistance': False}
    self.sm.logMonoTime = {'longitudinalPlan': self.now, 'modelV2': self.now, 'deviceMotion': 900_000_000, 'extrinsicsCalibration': 900_000_000}
    self.messages = {}
    c = self.controls = Controls.__new__(Controls)
    c.CP, c.sm = cp, self.sm
    c.pose_calibrator, c.calibrated_pose = PoseCalibrator(), None
    c.coordination_pose_sm = self.sm if enabled else None
    c.coordination_pose_calibrator = PoseCalibrator()
    c.coordination_pose_valid = False
    c.longitudinal_context = LongitudinalContext(enabled)
    c.curvature = c.desired_curvature = 0.
    c.LoC = LongControl(cp)
    c.pm = NS(send=lambda name, message: self.messages.__setitem__(name, message.to_bytes()))
    self.controller = CarController(DBC[CAR.FORD_EXPEDITION_MK4], cp)
    self.controller.longitudinal_coordination_enabled = enabled
    self.controller.coordination_control_valid = True
    self.state = NS(out=self.sm['carState'], buttons_stock_values=defaultdict(int),
                    lkas_status_stock_values=defaultdict(int), acc_tja_status_stock_values=defaultdict(int))

  def command(self, owner=True):
    # Use actual published schema bytes, without sharing a mutable builder.
    self.controls.update()
    cc = structs.CarControl(enabled=owner, longActive=owner)
    cc.actuators.accel = .2 if owner else 0.
    cc.actuators.longControlState = 'pid' if owner else 'off'
    self.controls.longitudinal_context.prepare(cc, self.sm['carOutput'].actuatorsOutput, self.now, output_valid=True)
    self.controls.publish(cc, log.ControlsState.LateralAngleState.new_message())
    return messaging.log_from_bytes(self.messages['carControl']).carControl

  def consume(self, command):
    self.state.longitudinal_brake_torque = Torque(self.now / 1e9, 0.)
    self.controller.frame += self.controller.frame % 2
    output, sends = self.controller.update(command, self.state, self.now)
    self.sm['carOutput'].actuatorsOutput = output
    return output, sends


def test_calibration_only_keeps_healthy_pose_valid_during_engagement():
  h = PosePipeline()
  first = h.command()
  assert first.navigatorLongitudinal.poseValid
  h.sm.updated['deviceMotion'] = False
  h.now += 20_000_000
  cc = h.command()
  assert cc.navigatorLongitudinal.poseValid
  assert cc.navigatorLongitudinal.poseMonoTime == 950_000_000
  output, _ = h.consume(cc)
  assert not output.navigatorLongitudinal.fault
  assert coordination_event(h.cp, output) is None


def test_changed_calibration_updates_orientation_and_validity_together():
  h = PosePipeline()
  first = h.command()
  assert first.orientationNED[1] == pytest.approx(.03)
  h.sm['extrinsicsCalibration'].rpyCalib = [0., .03, 0.]
  h.sm.updated['deviceMotion'] = False
  h.now += 20_000_000
  cc = h.command()
  assert cc.orientationNED[1] == pytest.approx(.05)
  assert cc.navigatorLongitudinal.poseValid
  assert cc.navigatorLongitudinal.poseMonoTime == 950_000_000


def test_calibration_without_motion_does_not_manufacture_pose():
  h = PosePipeline(seen_motion=False)
  cc = h.command()
  assert h.controls.calibrated_pose is None
  assert not cc.navigatorLongitudinal.poseValid


@pytest.mark.parametrize('field', ['sensorsOK', 'inputsOK', 'posenetOK', 'orientationValid', 'motionTopic', 'calibrationTopic', 'uncalibrated'])
def test_real_invalid_input_still_disengages(field):
  h = PosePipeline()
  h.command()
  h.sm.updated['deviceMotion'] = False
  if field in ['sensorsOK', 'inputsOK', 'posenetOK']:
    setattr(h.sm['deviceMotion'], field, False)
  elif field == 'orientationValid':
    h.sm['deviceMotion'].orientationNED.valid = False
  elif field == 'motionTopic':
    h.sm.valid['deviceMotion'] = False
  elif field == 'calibrationTopic':
    h.sm.valid['extrinsicsCalibration'] = False
  else:
    h.sm['extrinsicsCalibration'].calStatus = 'uncalibrated'
  cc = h.command()
  assert not cc.navigatorLongitudinal.poseValid
  output, _ = h.consume(cc)
  assert output.navigatorLongitudinal.fault
  assert coordination_event(h.cp, output) == log.OnroadEvent.EventName.controlsMismatch


@pytest.mark.parametrize('sample_ns', [899_000_000, 1_001_000_000])
def test_calibration_refresh_does_not_restamp_stale_or_future_motion(sample_ns):
  h = PosePipeline()
  h.command()
  h.sm['deviceMotion'].timestamp = sample_ns
  h.sm.updated['deviceMotion'] = False
  cc = h.command()
  assert cc.navigatorLongitudinal.poseMonoTime == sample_ns
  output, _ = h.consume(cc)
  assert output.navigatorLongitudinal.fault


def test_disabled_coordination_preserves_existing_orientation_update_cadence():
  h = PosePipeline(enabled=False)
  first = h.command()
  h.sm['extrinsicsCalibration'].rpyCalib = [0., .03, 0.]
  h.sm.updated['deviceMotion'] = False
  second = h.command()
  assert list(second.orientationNED) == list(first.orientationNED)
  assert not second.navigatorLongitudinal.enabled
  h.sm.updated = {'deviceMotion': True, 'extrinsicsCalibration': False}
  assert h.command().orientationNED[1] == pytest.approx(.05)


def test_reversed_motion_still_faults_and_cannot_rearm_while_owner_enabled():
  h = PosePipeline()
  output, _ = h.consume(h.command())
  assert not output.navigatorLongitudinal.fault
  h.now += 20_000_000
  h.sm['deviceMotion'].timestamp = 940_000_000
  h.sm.updated['deviceMotion'] = False
  output, _ = h.consume(h.command())
  assert output.navigatorLongitudinal.fault
  h.now += 20_000_000
  h.sm['deviceMotion'].timestamp = h.now - 20_000_000
  h.sm.updated['deviceMotion'] = True
  output, _ = h.consume(h.command())
  assert output.navigatorLongitudinal.fault
  assert coordination_event(h.cp, output) == log.OnroadEvent.EventName.controlsMismatch


@pytest.mark.parametrize('calibration_first', [True, False])
def test_independent_calibration_motion_cadence_during_moving_engagement(calibration_first):
  h = PosePipeline()
  for i in range(200):
    h.now = 1_000_000_000 + i * 10_000_000
    motion_update = i % 5 == (1 if calibration_first else 0)
    calibration_update = i % 25 == 0
    if motion_update:
      h.sm['deviceMotion'].timestamp = h.now - 20_000_000
    h.sm.updated = {'deviceMotion': motion_update or i == 0, 'extrinsicsCalibration': calibration_update}
    cc = h.command(owner=i >= 2)
    assert cc.navigatorLongitudinal.poseValid
    assert cc.navigatorLongitudinal.poseMonoTime == h.sm['deviceMotion'].timestamp
    output, _ = h.consume(cc)
    assert not output.navigatorLongitudinal.fault
    assert coordination_event(h.cp, output) is None


@pytest.mark.parametrize('source', ['motion', 'calibration'])
def test_nonfinite_pose_still_faults(source):
  h = PosePipeline()
  h.command()
  if source == 'motion':
    h.sm['deviceMotion'].orientationNED.y = float('nan')
  else:
    h.sm['extrinsicsCalibration'].rpyCalib = [0., float('nan'), 0.]
  cc = h.command()
  output, _ = h.consume(cc)
  assert output.navigatorLongitudinal.fault
  assert coordination_event(h.cp, output) == log.OnroadEvent.EventName.controlsMismatch


def test_calibration_only_engagement_through_real_card_and_parent_fault_latch(monkeypatch):
  from openpilot.selfdrive.car import card
  h = PosePipeline()
  monkeypatch.setattr(card, 'time', NS(monotonic=lambda: h.now / 1e9))
  interface = CarInterface(h.cp)
  interface.CC = h.controller
  interface.CS.out = h.sm['carState']
  for name in ['buttons_stock_values', 'lkas_status_stock_values', 'acc_tja_status_stock_values']:
    setattr(interface.CS, name, defaultdict(int))
  consumer = card.Car.__new__(card.Car)
  consumer.CI, consumer.CP = interface, h.cp
  consumer.sm = NS(all_alive=lambda _topics: True, all_checks=lambda _topics: True)
  consumer.initialized_prev = True
  consumer.tracking_trial = None
  consumer.longitudinal_coordination_enabled = True
  sent = []
  consumer.pm = NS(send=lambda topic, message: sent.append((topic, message)))

  def step(owner=True):
    cc = h.command(owner=owner)
    interface.CS.longitudinal_brake_torque = Torque(h.now / 1e9, 0.)
    consumer.controls_update(h.sm['carState'], cc)
    h.sm['carOutput'].actuatorsOutput = consumer.last_actuators_output
    return cc, consumer.last_actuators_output

  for i in range(20):
    h.now = 1_000_000_000 + i * 10_000_000
    if i % 5 == 0:
      h.sm['deviceMotion'].timestamp = h.now - 20_000_000
    h.sm.updated = {'deviceMotion': i % 5 == 0, 'extrinsicsCalibration': i in [0, 2, 12]}
    cc, output = step()
    assert cc.navigatorLongitudinal.poseValid
    assert not output.navigatorLongitudinal.fault
    assert coordination_event(h.cp, output) is None
  assert sent and {topic for topic, _ in sent} == {'sendcan'}
  h.now += 20_000_000
  h.sm['deviceMotion'].sensorsOK = False
  h.sm.updated = {'deviceMotion': True, 'extrinsicsCalibration': False}
  _, output = step()
  assert output.navigatorLongitudinal.fault
  h.now += 20_000_000
  h.sm['deviceMotion'].sensorsOK = True
  h.sm['deviceMotion'].timestamp = h.now - 20_000_000
  cc, output = step()
  assert h.controls.longitudinal_context.fault
  assert not cc.longActive and output.navigatorLongitudinal.fault
  h.now += 20_000_000
  step(owner=False)
  h.now += 20_000_000
  h.sm['deviceMotion'].timestamp = h.now - 20_000_000
  cc, output = step()
  assert cc.longActive and not h.controls.longitudinal_context.fault
  assert not output.navigatorLongitudinal.fault
  h.now += 20_000_000
  _, output = step()
  assert output.navigatorLongitudinal.valid and not output.navigatorLongitudinal.fault


def test_late_motion_poll_repairs_exact_consumer_timing_without_restamping(monkeypatch):
  h = PosePipeline()
  h.now = 1_031_521_100
  newer = log.DeviceMotion.new_message(timestamp=1_009_643_926, sensorsOK=True, inputsOK=True, posenetOK=True)
  newer.orientationNED.valid = True
  newer.orientationNED.y = .021
  late = Inputs(deviceMotion=newer, extrinsicsCalibration=h.sm['extrinsicsCalibration'])
  late.seen = {'deviceMotion': True, 'extrinsicsCalibration': True}
  late.updated = {'deviceMotion': True, 'extrinsicsCalibration': True}
  late.valid = {'deviceMotion': True, 'extrinsicsCalibration': True}
  h.controls.coordination_pose_sm = late
  h.controls.coordination_pose_calibrator = PoseCalibrator()
  cc = h.command()
  assert cc.navigatorLongitudinal.poseMonoTime == newer.timestamp
  assert cc.navigatorLongitudinal.producerMonoTime == h.now
  assert cc.orientationNED[1] == pytest.approx(.031)
  h.now = 1_054_368_056
  from openpilot.selfdrive.car import card
  monkeypatch.setattr(card, 'time', NS(monotonic=lambda: h.now / 1e9))
  interface = CarInterface(h.cp)
  interface.CC = h.controller
  interface.CS.out = h.sm['carState']
  interface.CS.longitudinal_brake_torque = Torque(h.now / 1e9, 0.)
  for name in ['buttons_stock_values', 'lkas_status_stock_values', 'acc_tja_status_stock_values']:
    setattr(interface.CS, name, defaultdict(int))
  consumer = card.Car.__new__(card.Car)
  consumer.CI, consumer.CP = interface, h.cp
  consumer.sm = NS(all_alive=lambda _topics: True, all_checks=lambda _topics: True)
  consumer.initialized_prev = True
  consumer.tracking_trial = None
  consumer.longitudinal_coordination_enabled = True
  sent = []
  consumer.pm = NS(send=lambda topic, message: sent.append((topic, message)))
  consumer.controls_update(h.sm['carState'], cc)
  assert not consumer.last_actuators_output.navigatorLongitudinal.fault
  assert sent and sent[0][0] == 'sendcan'



@pytest.mark.parametrize('field', ['sensorsOK', 'inputsOK', 'posenetOK', 'orientationValid', 'motionTopic', 'calibrationTopic',
                                 'uncalibrated', 'unseen', 'future', 'stale', 'reversed'])
def test_late_poll_does_not_hide_unhealthy_or_uncausal_input(field):
  h = PosePipeline()
  h.consume(h.command())
  h.now += 20_000_000
  late = Inputs(deviceMotion=log.DeviceMotion.new_message(timestamp=h.now - 20_000_000, sensorsOK=True, inputsOK=True, posenetOK=True),
                extrinsicsCalibration=log.ExtrinsicsCalibration.new_message(calStatus='calibrated', rpyCalib=[0., .01, 0.]))
  late['deviceMotion'].orientationNED.valid = True
  late.seen = {'deviceMotion': True, 'extrinsicsCalibration': True}
  late.updated = {'deviceMotion': True, 'extrinsicsCalibration': True}
  late.valid = {'deviceMotion': True, 'extrinsicsCalibration': True}
  if field in ['sensorsOK', 'inputsOK', 'posenetOK']:
    setattr(late['deviceMotion'], field, False)
  elif field == 'orientationValid':
    late['deviceMotion'].orientationNED.valid = False
  elif field.endswith('Topic'):
    late.valid['deviceMotion' if field == 'motionTopic' else 'extrinsicsCalibration'] = False
  elif field == 'uncalibrated':
    late['extrinsicsCalibration'].calStatus = 'uncalibrated'
  elif field == 'unseen':
    late.seen['deviceMotion'] = False
  else:
    late['deviceMotion'].timestamp = h.now + 1 if field == 'future' else h.now - 110_000_000 if field == 'stale' else 940_000_000
  h.controls.coordination_pose_sm = late
  h.controls.coordination_pose_calibrator = PoseCalibrator()
  cc = h.command()
  if field != 'unseen':
    assert cc.navigatorLongitudinal.poseMonoTime == late['deviceMotion'].timestamp
  output, _ = h.consume(cc)
  assert bool(output.navigatorLongitudinal.fault) == (field != 'unseen')


def test_late_poll_does_not_affect_installed_stop_or_creep_path():
  h = PosePipeline()
  h.sm['carState'].vEgo = 2.0
  class MustNotPoll:
    def update(self, *_args):
      raise AssertionError('stop/creep must retain installed source')
  h.controls.coordination_pose_sm = MustNotPoll()
  assert h.command().navigatorLongitudinal.poseMonoTime == 950_000_000


def test_late_subscriber_is_drained_nonblocking_before_snapshot_is_built():
  h = PosePipeline()
  h.now = 1_031_521_100
  late = Inputs(h.sm)
  late.seen, late.valid = dict(h.sm.seen), dict(h.sm.valid)
  polls = []
  def poll(timeout):
    assert timeout == 0
    polls.append(timeout)
    newer = log.DeviceMotion.new_message(timestamp=1_009_643_926, sensorsOK=True, inputsOK=True, posenetOK=True)
    newer.orientationNED.valid = True
    late['deviceMotion'] = newer
  late.update = poll
  h.controls.coordination_pose_sm = late
  h.controls.coordination_pose_calibrator = PoseCalibrator()
  cc = h.command()
  assert polls == [0]
  assert cc.navigatorLongitudinal.poseMonoTime == 1_009_643_926
  assert h.sm['deviceMotion'].timestamp == 950_000_000


def test_late_healthy_snapshot_cannot_clear_invalidity_already_seen_by_main_producer():
  h = PosePipeline()
  h.sm['deviceMotion'].sensorsOK = False
  late = Inputs(deviceMotion=log.DeviceMotion.new_message(timestamp=980_000_000, sensorsOK=True, inputsOK=True, posenetOK=True),
                extrinsicsCalibration=h.sm['extrinsicsCalibration'])
  late['deviceMotion'].orientationNED.valid = True
  late.seen = {'deviceMotion': True, 'extrinsicsCalibration': True}
  late.valid = {'deviceMotion': True, 'extrinsicsCalibration': True}
  h.controls.coordination_pose_sm = late
  h.controls.coordination_pose_calibrator = PoseCalibrator()
  cc = h.command()
  assert not cc.navigatorLongitudinal.poseValid
  output, _ = h.consume(cc)
  assert output.navigatorLongitudinal.fault


def test_late_refresh_preserves_reference_withdrawal_and_command_ownership():
  from openpilot.selfdrive.car.longitudinal_context import refresh_coordination_pose
  h = PosePipeline()
  cc = h.command().as_builder()
  cc.navigatorLongitudinal.referenceValid = False
  before = cc.to_dict()
  refresh_coordination_pose(cc, h.sm, h.controls.coordination_pose_calibrator)
  assert not cc.navigatorLongitudinal.referenceValid
  for key in ['enabled', 'epoch', 'sequence', 'producerMonoTime']:
    assert getattr(cc.navigatorLongitudinal, key) == before['navigatorLongitudinal'][key]
  assert cc.actuators.to_dict() == before['actuators']


@pytest.mark.parametrize('topic', ['deviceMotion', 'extrinsicsCalibration'])
def test_lagging_dedicated_cache_cannot_roll_back_newer_primary_publication(topic):
  h = PosePipeline()
  h.sm['deviceMotion'].timestamp = 990_000_000
  h.consume(h.command())
  h.now = 1_030_000_000
  late = Inputs(deviceMotion=log.DeviceMotion.new_message(timestamp=980_000_000, sensorsOK=True, inputsOK=True, posenetOK=True),
                extrinsicsCalibration=log.ExtrinsicsCalibration.new_message(calStatus='calibrated', rpyCalib=[0., -.04, 0.]))
  late['deviceMotion'].orientationNED.valid = True
  late.seen = {'deviceMotion': True, 'extrinsicsCalibration': True}
  late.valid = {'deviceMotion': True, 'extrinsicsCalibration': True}
  late.logMonoTime = {'deviceMotion': 995_000_000, 'extrinsicsCalibration': 995_000_000}
  h.sm['deviceMotion'].timestamp = 1_010_000_000
  h.sm.logMonoTime.update(deviceMotion=1_020_000_000, extrinsicsCalibration=1_020_000_000)
  if topic == 'extrinsicsCalibration':
    late['deviceMotion'].timestamp = 1_015_000_000
    late.logMonoTime['deviceMotion'] = 1_025_000_000
  h.controls.coordination_pose_sm = late
  cc = h.command()
  assert cc.navigatorLongitudinal.poseMonoTime == (1_015_000_000 if topic == 'extrinsicsCalibration' else 1_010_000_000)
  assert cc.orientationNED[1] == pytest.approx(.01 if topic == 'extrinsicsCalibration' else .03)
  output, _ = h.consume(cc)
  assert not output.navigatorLongitudinal.fault
