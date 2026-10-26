"""Real paired precharge schema and producer/consumer contract."""

from opendbc.car import structs
from openpilot.selfdrive.car.longitudinal_context import LongitudinalContext


def test_consumed_target_is_stamped_and_roundtrips_without_latest_topic_join():
  context = LongitudinalContext(True)
  cc = structs.CarControl(enabled=True, longActive=True)
  context.prepare(cc, structs.CarControl.Actuators(), 1_000_000_000, output_valid=False, target=0.125, target_ns=980_000_000, target_valid=True)
  with structs.CarControl.from_bytes(cc.to_bytes()) as wire:
    assert wire.navigatorLongitudinal.allocatorVersion == 1
    assert wire.navigatorLongitudinal.targetValid
    assert wire.navigatorLongitudinal.targetAcceleration == 0.125
    assert wire.navigatorLongitudinal.targetMonoTime == 980_000_000


def test_feedback_precharge_and_nominal_brake_are_distinct_roundtrip_fields():
  from opendbc.car.ford.longitudinal_coordination import feedback_from_wire

  output = structs.CarControl.Actuators()
  f = output.navigatorLongitudinal
  f.allocatorVersion = 1
  f.precharge = True
  f.referenceBrake = True
  f.brake = False
  f.allocatorPhase = 'precharge'
  f.prechargeAge = 0.04
  f.budgetRemaining = 0.004
  with structs.CarControl.Actuators.from_bytes(output.to_bytes()) as wire:
    result = feedback_from_wire(wire.navigatorLongitudinal)
    assert result.allocator_version == 1 and result.precharge and result.reference_brake and not result.brake
    assert result.allocator_phase == 'precharge' and result.precharge_age == 0.04 and result.budget_remaining == 0.004


def test_missing_or_stale_target_cannot_opt_into_precharge():
  for target, source, valid in [(None, 0, False), (0.1, 800_000_000, True), (float('nan'), 980_000_000, True), (0.1, 1_010_000_000, True)]:
    context = LongitudinalContext(True)
    cc = structs.CarControl(enabled=True, longActive=True)
    context.prepare(cc, structs.CarControl.Actuators(), 1_000_000_000, output_valid=False, target=target, target_ns=source, target_valid=valid)
    assert not cc.navigatorLongitudinal.targetValid


def test_older_future_or_malformed_peer_metadata_selects_nominal_without_fault():
  for version, phase, age in [(0, 'inactive', 0.0), (2, 'inactive', 0.0), (1, 99, 0.0), (1, 'precharge', float('nan'))]:
    context = LongitudinalContext(True)
    cc = structs.CarControl(enabled=True, longActive=True)
    output = structs.CarControl.Actuators()
    context.prepare(cc, output, 1_000_000_000, output_valid=False)
    f = output.navigatorLongitudinal
    f.epoch = cc.navigatorLongitudinal.epoch
    f.sequence = 1
    f.producerMonoTime = 1_000_000_000
    f.valid = True
    f.allocatorVersion = version
    f.allocatorPhase = phase
    f.prechargeAge = age
    context.prepare(cc, output, 1_020_000_000, output_valid=True, target=0.1, target_ns=1_000_000_000, target_valid=True)
    assert cc.navigatorLongitudinal.allocatorVersion == 0 and cc.longActive and not context.fault
