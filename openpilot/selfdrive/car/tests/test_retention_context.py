import pytest
from types import SimpleNamespace as NS
import json
from opendbc.car import structs
from opendbc.car.ford.longitudinal_coordination import Feedback, feedback_from_wire
from opendbc.car.ford.longitudinal_retention import RetentionReceipt, REQUEST_VERSION, make_retention_owner, make_receipt_tracker, speed_sample_ns
from openpilot.selfdrive.car.longitudinal_context import retention_startup_enabled, stamp_allocator_request, feed_controller_context


class TestTransport:
  def test_old_request_default_and_explicit_roundtrip(self):
    cc = structs.CarControl.new_message()
    assert cc.navigatorLongitudinal.retentionVersion == 0
    cc.navigatorLongitudinal.retentionVersion = REQUEST_VERSION
    with structs.CarControl.from_bytes(cc.to_bytes()) as r:
      assert r.navigatorLongitudinal.retentionVersion == 1

  def test_old_serialized_schema_request_defaults_capability_zero(self):
    with structs.CarControl.from_bytes(
      bytes.fromhex(
        '0000000014000000000000000400080000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000700000000000000000000002a0000000000000007000000000000000000000000000000000000000000000000000000000000000000000000000000'
      )
    ) as cc:
      assert cc.navigatorLongitudinal.retentionVersion == 0
      assert cc.navigatorLongitudinal.epoch == 42
      assert cc.navigatorLongitudinal.sequence == 7

  def test_actual_actuator_wire_to_typed_feedback(self):
    a = structs.CarControl.Actuators.new_message()
    f = a.navigatorLongitudinal
    f.reason = RetentionReceipt(1100000000).encode()
    f.producerMonoTime = 1050000000
    f.epoch = 1
    f.sequence = 2
    f.valid = True
    f.brake = True
    f.precharge = True
    f.referenceBrake = False
    f.gas = -5
    f.requested = 0.2
    f.allocatorVersion = 3
    f.allocatorPhase = 'brake'
    with structs.CarControl.Actuators.from_bytes(a.to_bytes()) as r:
      decoded = feedback_from_wire(r.navigatorLongitudinal)
    assert decoded.retention_receipt == RetentionReceipt(1100000000)
    assert decoded.reason == f.reason
    assert make_receipt_tracker(enabled=True).freeze(1.1, 1, 2, 0.1, 0.1, 0.2, 27.0, 'pid', decoded, True, True)

  def test_old_feedback_json_and_positional(self):
    f = Feedback(1.0, 1, 1, 0.1, -5.0, True, False)
    assert f.reason == ''
    assert f.retention_receipt is None
    d = json.loads(f.encode())
    d.pop('reason')
    assert Feedback.decode(json.dumps(d).encode()).reason == ''

  def test_unknown_malformed_receipt(self):
    for x in ['', None, 'unknown_v2|until_ns=1100000000', 'navigator_retention_v1|until_ns=x', 'navigator_retention_v1|until_ns=' + '1' * 1000]:
      assert RetentionReceipt.parse(x) is None

  def test_explicit_factories_default_off(self):
    assert make_retention_owner() is None
    assert make_receipt_tracker() is None
    assert make_retention_owner(enabled=True) is not None

  def test_startup_exact_supported_gate(self):
    for env in [{}, {'NAVIGATOR_RECEIPT_RETENTION': '0'}, {'NAVIGATOR_RECEIPT_RETENTION': 'true'}, {'NAVIGATOR_RECEIPT_RETENTION': '1 '}]:
      assert not retention_startup_enabled(supported=True, environ=env)
    assert not retention_startup_enabled(supported=False, environ={'NAVIGATOR_RECEIPT_RETENTION': '1'})
    assert retention_startup_enabled(supported=True, environ={'NAVIGATOR_RECEIPT_RETENTION': '1'})

  def test_request_stamp_default_off_and_explicit_capability(self):
    cc = structs.CarControl.new_message()
    stamp_allocator_request(cc, 100, target=0.1, target_ns=90, target_valid=True)
    assert cc.navigatorLongitudinal.retentionVersion == 0
    stamp_allocator_request(cc, 100, target=0.1, target_ns=90, target_valid=True, retention_enabled=True)
    assert cc.navigatorLongitudinal.retentionVersion == 1

  def test_unrelated_can_does_not_advance_speed_sample(self):
    parser = NS(ts_nanos={'BrakeSysFeatures': {'Veh_V_ActlBrk': 1000000000}})
    controller = NS()
    feed_controller_context(controller, control_valid=True, state_ns=1010000000, state_valid=True, speed_ns=speed_sample_ns(parser))
    first = controller.coordination_speed_ns
    feed_controller_context(controller, control_valid=True, state_ns=1070000000, state_valid=True, speed_ns=speed_sample_ns(parser))
    assert controller.coordination_speed_ns == first
    assert controller.coordination_state_ns == 1070000000

  def test_unknown_speed_timestamp_is_zero(self):
    assert speed_sample_ns(NS()) == 0
    assert speed_sample_ns(NS(ts_nanos={'BrakeSysFeatures': {'Veh_V_ActlBrk': float('nan')}})) == 0


  def test_nonfinite_source_forwarding_fails_closed(self):
    for value in [None, float('nan'), float('inf'), -1, '100']:
      controller = NS()
      feed_controller_context(controller, control_valid=True, state_ns=value, state_valid=True, speed_ns=value)
      assert controller.coordination_state_ns == 0
      assert controller.coordination_speed_ns == 0
      assert not controller.coordination_state_valid

  def test_real_parser_speed_sample_survives_unrelated_traffic(self):
    from opendbc.can import CANParser, CANPacker
    parser = CANParser('ford_lincoln_base_pt', [('BrakeSysFeatures', 0), ('SteeringPinion_Data', 0)], 0)
    packer = CANPacker('ford_lincoln_base_pt')
    parser.update([(1_000_000_000, [packer.make_can_msg('BrakeSysFeatures', 0, {'Veh_V_ActlBrk': 50.})])])
    assert speed_sample_ns(parser) == 1_000_000_000
    parser.update([(1_080_000_000, [packer.make_can_msg('SteeringPinion_Data', 0, {'StePinComp_An_Est': 1.})])])
    assert speed_sample_ns(parser) == 1_000_000_000
    parser.update([(1_100_000_000, [packer.make_can_msg('BrakeSysFeatures', 0, {'Veh_V_ActlBrk': 50.})])])
    assert speed_sample_ns(parser) == 1_100_000_000

class TestRealPath:
  def run_controller(self, producer=False, consumer=False):
    from collections import defaultdict
    from opendbc.car.ford.carcontroller import CarController
    from opendbc.car.ford.interface import CarInterface
    from opendbc.car.ford.values import CAR, DBC
    from opendbc.car.ford.longitudinal_coordination import Torque

    cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
    cp.openpilotLongitudinalControl = True
    ctrl = CarController(DBC[CAR.FORD_EXPEDITION_MK4], cp)
    ctrl.longitudinal_coordination_enabled = True
    ctrl.configure_retention(enabled=consumer)
    ctrl.brake_request = True
    ctrl.reference_brake = True
    cs = NS(
      out=structs.CarState(vEgo=27.0, vEgoRaw=27.0, aEgo=0.2, canValid=True),
      buttons_stock_values=defaultdict(int),
      lkas_status_stock_values=defaultdict(int),
      acc_tja_status_stock_values=defaultdict(int),
    )
    cc = structs.CarControl(enabled=True, longActive=True)
    cc.orientationNED = [0.0, 0.0, 0.0]
    cc.actuators.accel = 0.2
    cc.actuators.longControlState = 'pid'
    q = cc.navigatorLongitudinal
    wires = []
    outputs = []
    for i in range(12):
      now = 1000000000 + i * 10000000
      q.enabled = True
      q.epoch = 1
      q.sequence = i + 1
      q.producerMonoTime = now
      q.poseMonoTime = now
      q.poseValid = True
      q.referenceValid = True
      stamp_allocator_request(cc, now, target=0.1, target_ns=now, target_valid=True, retention_enabled=producer)
      feed_controller_context(ctrl, control_valid=True, state_ns=now, state_valid=True, speed_ns=now)
      cs.longitudinal_brake_torque = Torque(now / 1000000000.0, 10.0)
      out, msgs = ctrl.update(cc.as_reader(), cs, now)
      outputs.append(out)
      wires.extend(bytes(x[1]) for x in msgs if x[0] == 390)
    return (ctrl, outputs, wires)

  def test_mismatched_peer_actual_controller_wires_nominal(self):
    _, _, base = self.run_controller()
    for producer, consumer in [(False, True), (True, False)]:
      _, outputs, wires = self.run_controller(producer, consumer)
      assert wires == base
      assert all(feedback_from_wire(o.navigatorLongitudinal).retention_receipt is None for o in outputs)

  def test_both_enabled_actual_controller_tag_and_nominal_reference(self):
    ctrl, outputs, wires = self.run_controller(True, True)
    assert any(feedback_from_wire(o.navigatorLongitudinal).retention_receipt is not None for o in outputs)
    for o in outputs:
      f = feedback_from_wire(o.navigatorLongitudinal)
      if f.retention_receipt is not None:
        assert f.brake
        assert not f.reference_brake
        assert f.allocator_phase == 'brake'
    assert ctrl.last_feedback.reason == outputs[-1].navigatorLongitudinal.reason
    with pytest.raises(RuntimeError):
      ctrl.configure_retention(enabled=False)

  def test_real_context_wire_feedback_reaches_longcontrol(self):
    from openpilot.selfdrive.car.longitudinal_context import LongitudinalContext
    from openpilot.selfdrive.controls.lib.longcontrol import LongControl
    from opendbc.car.ford.interface import CarInterface
    from opendbc.car.ford.values import CAR

    cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
    cp.openpilotLongitudinalControl = True
    ctx = LongitudinalContext(True, retention_enabled=True)
    ctx.epoch = 1
    ctx.owner = True
    ctx.sequence = 1
    a = structs.CarControl.Actuators.new_message()
    f = a.navigatorLongitudinal
    f.reason = RetentionReceipt(1100000000).encode()
    f.producerMonoTime = 1050000000
    f.epoch = 1
    f.sequence = 1
    f.valid = True
    f.brake = True
    f.precharge = True
    f.referenceBrake = False
    f.gas = -5
    f.requested = 0.2
    f.allocatorVersion = 3
    f.allocatorPhase = 'brake'
    with structs.CarControl.Actuators.from_bytes(a.to_bytes()) as r:
      wire = r.as_builder()
    cc = structs.CarControl(enabled=True, longActive=True)
    cs = structs.CarState(vEgo=27.0, aEgo=0.0, canValid=True)
    loc = LongControl(cp, retention_enabled=True)
    for now in [1090000000, 1100000000]:
      ctx.prepare(cc, wire, now, output_valid=True, target=0.1, target_ns=now, target_valid=True)
      assert cc.navigatorLongitudinal.retentionVersion == 1
      loc.update(
        True,
        cs,
        0.1,
        False,
        (-3.5, 2.0),
        feedback=ctx.feedback,
        now=now / 1000000000.0,
        epoch=ctx.epoch,
        sequence=ctx.sequence,
        coordinated=True,
        source_valid=True,
      )
    assert loc.last_integral_frozen
