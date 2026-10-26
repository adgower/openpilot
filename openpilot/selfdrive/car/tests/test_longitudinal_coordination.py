import unittest

from opendbc.car import structs


class TestTransport(unittest.TestCase):
  def test_request_and_feedback_wire_roundtrip(self):
    request = structs.CarControl()
    request.navigatorLongitudinal.enabled = True
    request.navigatorLongitudinal.epoch = 123456789123
    request.navigatorLongitudinal.sequence = 22
    request.navigatorLongitudinal.poseMonoTime = 987654321123
    with structs.CarControl.from_bytes(request.to_bytes()) as copy:
      self.assertEqual(copy.navigatorLongitudinal.epoch, 123456789123)
      self.assertEqual(copy.navigatorLongitudinal.poseMonoTime, 987654321123)
    output = structs.CarControl.Actuators()
    output.navigatorLongitudinal.fault = True
    output.navigatorLongitudinal.reason = 'release_timeout_disengage'
    with structs.CarControl.Actuators.from_bytes(output.to_bytes()) as copy:
      self.assertTrue(copy.navigatorLongitudinal.fault)
      self.assertEqual(copy.navigatorLongitudinal.reason, 'release_timeout_disengage')


if __name__ == '__main__':
  unittest.main()

class TestRuntime(unittest.TestCase):
  def test_fault_maps_to_immediate_disable_and_no_entry(self):
    from openpilot.selfdrive.car.longitudinal_context import coordination_event
    from openpilot.selfdrive.selfdrived.events import EVENTS, ET
    from opendbc.car.ford.interface import CarInterface
    from opendbc.car.ford.values import CAR
    cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
    cp.openpilotLongitudinalControl = True
    output = structs.CarControl.Actuators()
    self.assertIsNone(coordination_event(cp, output))
    output.navigatorLongitudinal.fault = True
    event = coordination_event(cp, output)
    self.assertIn(ET.IMMEDIATE_DISABLE, EVENTS[event])
    self.assertIn(ET.NO_ENTRY, EVENTS[event])
    cp.openpilotLongitudinalControl = False
    self.assertIsNone(coordination_event(cp, output))

  def test_owner_epochs_and_feedback_do_not_rearm_while_enabled(self):
    from openpilot.selfdrive.car.longitudinal_context import LongitudinalContext
    context = LongitudinalContext(True)
    request = structs.CarControl(enabled=True, longActive=True)
    output = structs.CarControl.Actuators()
    context.prepare(request, output, 1_000_000_000, output_valid=True)
    epoch = request.navigatorLongitudinal.epoch
    output.navigatorLongitudinal.epoch = epoch
    output.navigatorLongitudinal.sequence = 1
    output.navigatorLongitudinal.fault = True
    output.navigatorLongitudinal.producerMonoTime = 1_000_000_000
    context.prepare(request, output, 1_010_000_000, output_valid=True)
    self.assertFalse(request.longActive)
    self.assertEqual(context.epoch, epoch)
    output.navigatorLongitudinal.fault = False
    request.longActive = True
    context.prepare(request, output, 1_020_000_000, output_valid=True)
    self.assertFalse(request.longActive)
    request.enabled = False
    context.prepare(request, output, 1_030_000_000, output_valid=True)
    request.enabled = request.longActive = True
    context.prepare(request, output, 1_040_000_000, output_valid=True)
    self.assertTrue(request.longActive)
    self.assertNotEqual(context.epoch, epoch)


def test_production_parent_card_pipeline():
  # card captures REPLAY at import. Keep this live-mode exercise out of the
  # suite process, whose later process-replay tests fork and reuse modules.
  import subprocess
  import sys
  code = """import pytest
from openpilot.selfdrive.car.tests.test_longitudinal_coordination import _run_longitudinal_pipeline
with pytest.MonkeyPatch.context() as patch:
  _run_longitudinal_pipeline(patch)
"""
  subprocess.run([sys.executable, '-c', code], check=True, timeout=30)


def _run_longitudinal_pipeline(monkeypatch):
  """Exercise producer, real schema, card and Ford controller; capture sends only."""
  from collections import defaultdict
  from types import SimpleNamespace as NS
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.ford.values import CAR
  from opendbc.car.ford.longitudinal_coordination import Torque
  from opendbc.car.vehicle_model import VehicleModel
  from openpilot.cereal import log
  from openpilot.selfdrive.car.card import Car
  from openpilot.selfdrive.car.longitudinal_context import LongitudinalContext, stamp_pose
  from openpilot.selfdrive.controls.controlsd import Controls
  from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from openpilot.selfdrive.controls import controlsd
  from openpilot.selfdrive.car import card

  class SM(dict):
    def all_checks(self, topics):
      return all(self.valid.get(x, False) for x in topics)

    def all_alive(self, topics):
      return True

  clock = [10_000_000_000]
  monkeypatch.setattr(controlsd, 'time', NS(monotonic_ns=lambda: clock[0]))
  monkeypatch.setattr(card, 'time', NS(monotonic=lambda: clock[0] / 1e9))
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.openpilotLongitudinalControl = True
  ci = CarInterface(cp)
  ci.CC.longitudinal_coordination_enabled = True
  cs = structs.CarState(vEgo=15., vEgoRaw=15., canValid=True)
  ci.CS.out = cs
  ci.CS.buttons_stock_values = defaultdict(int)
  ci.CS.lkas_status_stock_values = defaultdict(int)
  ci.CS.acc_tja_status_stock_values = defaultdict(int)
  lp = log.VehicleParameters.new_message(steerRatio=22.5, stiffnessFactor=1.)
  model = log.ModelDataV2.new_message()
  sm = SM(carState=cs, vehicleParameters=lp, modelV2=model,
          selfdriveState=log.SelfdriveState.new_message(enabled=True, active=True, state='enabled'),
    driverMonitoringState=log.DriverMonitoringState.new_message(), onroadEvents=[],
          longitudinalPlan=log.LongitudinalPlan.new_message(aTarget=.3, shouldStop=False), lateralDelay=NS(lateralDelay=.2),
          carOutput=NS(actuatorsOutput=structs.CarControl.Actuators()))
  sm.valid = {'lateralManeuverPlan': False, 'carOutput': True, 'vehicleParameters': True, 'carState': True, 'longitudinalPlan': True}
  sm.logMonoTime = {'carState': clock[0], 'vehicleParameters': clock[0], 'longitudinalPlan': clock[0]}
  controls = Controls.__new__(Controls)
  controls.CP, controls.CI, controls.sm = cp, ci, sm
  controls.VM = VehicleModel(cp)
  controls.LoC = LongControl(cp)
  controls.LaC = LatControlAngle(cp, ci, .01)
  controls.longitudinal_context = LongitudinalContext(True)
  controls.desired_curvature = controls.curvature = 0.
  controls.steer_limited_by_safety = False
  consumer = Car.__new__(Car)
  consumer.CI = ci
  consumer.CP = cp
  consumer.sm = SM()
  consumer.sm.valid = {'carControl': True}
  consumer.initialized_prev = True
  consumer.tracking_trial = None
  consumer.longitudinal_coordination_enabled = True
  from openpilot.selfdrive.car.longitudinal_pose import ConsumerPoseRefresh
  consumer.coordination_pose_refresher = ConsumerPoseRefresh()
  consumer.coordination_pose_sm = NS(update=lambda *_: None, seen={}, valid={}, logMonoTime={})
  sent = []
  consumer.pm = NS(send=lambda topic, message: sent.append((topic, message)))
  for _ in range(20):
    clock[0] += 10_000_000
    sm.logMonoTime = {'carState': clock[0], 'vehicleParameters': clock[0], 'longitudinalPlan': clock[0]}
    ci.CS.longitudinal_brake_torque = Torque(clock[0] / 1e9, 0.)
    CC, _ = controls.state_control()
    CC.orientationNED = [0., 0., 0.]
    stamp_pose(CC, sample_ns=clock[0], valid=True)
    # Real wire boundary, not sharing a mutable builder across processes.
    with structs.CarControl.from_bytes(CC.to_bytes()) as wire:
      consumer.controls_update(cs, wire)
    sm['carOutput'].actuatorsOutput = consumer.last_actuators_output
  f = consumer.last_actuators_output.navigatorLongitudinal
  assert f.valid and f.epoch == controls.longitudinal_context.epoch
  assert f.sequence > 0 and f.producerMonoTime > 0
  assert sent and {topic for topic, _ in sent} == {'sendcan'}
  # Release with missing physical feedback produces local neutral + output fault.
  ci.CS.longitudinal_brake_torque = Torque(clock[0] / 1e9, 160.)
  sm['longitudinalPlan'].aTarget = -1.
  for _ in range(50):
    clock[0] += 10_000_000
    CC, _ = controls.state_control()
    CC.orientationNED = [0., 0., 0.]
    stamp_pose(CC, sample_ns=clock[0], valid=True)
    consumer.controls_update(cs, CC.as_reader())
  sm['longitudinalPlan'].aTarget = .7
  ci.CS.longitudinal_brake_torque = None
  for _ in range(70):
    clock[0] += 10_000_000
    CC, _ = controls.state_control()
    CC.orientationNED = [0., 0., 0.]
    stamp_pose(CC, sample_ns=clock[0], valid=True)
    consumer.controls_update(cs, CC.as_reader())
    sm['carOutput'].actuatorsOutput = consumer.last_actuators_output
  assert consumer.last_actuators_output.navigatorLongitudinal.fault
  assert controls.longitudinal_context.fault
  assert not CC.longActive
  assert controls.LoC.pid.i == 0.


def test_parent_freezes_only_growth_with_serialized_current_feedback():
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.ford.values import CAR
  from opendbc.car.ford.longitudinal_coordination import feedback_from_wire
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  parent = LongControl(cp)
  parent.long_control_state = structs.CarControl.Actuators.LongControlState.pid
  parent.pid.i = .3
  cs = structs.CarState(vEgo=15., aEgo=-.2)
  output = structs.CarControl.Actuators()
  f = output.navigatorLongitudinal
  f.producerMonoTime, f.epoch, f.sequence = 980_000_000, 1, 1
  f.requested, f.gas, f.limited, f.valid = .3, .1, True, True
  with structs.CarControl.Actuators.from_bytes(output.to_bytes()) as wire:
    feedback = feedback_from_wire(wire.navigatorLongitudinal)
  parent.update(True, cs, 0., False, (-3.5, 2.), feedback=feedback, now=1., epoch=1, sequence=2, coordinated=True)
  assert parent.pid.i == .3 and parent.last_integral_frozen
  parent.update(True, cs, -.5, False, (-3.5, 2.), feedback=feedback, now=1.01, epoch=1, sequence=3, coordinated=True)
  assert parent.pid.i < .3 and not parent.last_integral_frozen


def test_can_brake_source_timestamp_is_not_refreshed_by_cached_state():
  from opendbc.can import CANPacker
  from opendbc.car import Bus
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.ford.values import CAR, DBC
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  interface = CarInterface(cp)
  assert 125 not in interface.can_parsers[Bus.pt].addresses
  interface.CS.enable_longitudinal_coordination(interface.can_parsers[Bus.pt])
  assert 125 in interface.can_parsers[Bus.pt].addresses
  packer = CANPacker(DBC[cp.carFingerprint][Bus.pt])
  message = packer.make_can_msg('BrakeSnData_4', 0, {'BrkTot_Tq_Actl': 4.})
  interface.update([(1_000_000_000, [message])])
  first = interface.CS.longitudinal_brake_torque
  interface.update([(1_020_000_000, [])])
  second = interface.CS.longitudinal_brake_torque
  assert first.t == second.t == 1. and second.value == 4.
  unknown = packer.make_can_msg('BrakeSnData_4', 0, {'BrkTot_Tq_Actl': 32760.})
  interface.update([(1_040_000_000, [unknown])])
  assert not interface.CS.longitudinal_brake_torque.valid
