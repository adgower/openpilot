"""Production signed-coast metadata survives transport without loosening bounds."""
from dataclasses import replace

from opendbc.car import structs
from opendbc.car.ford.longitudinal_coordination import Feedback, allocator_feedback_valid, feedback_from_wire
from opendbc.car.ford.longitudinal_precharge import allocator_request_valid
from openpilot.selfdrive.car.longitudinal_context import stamp_allocator_request


def coast_message():
  cc = structs.CarControl()
  f = cc.actuators.navigatorLongitudinal
  f.producerMonoTime = 1_000_000_000
  f.epoch, f.sequence = 7, 3
  f.requested, f.gas = .1, -5.
  f.valid, f.referenceBrake, f.limited = True, True, True
  f.brake, f.precharge = False, False
  f.allocatorVersion, f.allocatorPhase = 3, 'release'
  f.prechargeAge, f.budgetRemaining = .04, .009
  return cc


def test_signed_coast_schema_roundtrip_preserves_truthful_command_and_lease():
  cc = coast_message()
  with structs.CarControl.from_bytes(cc.to_bytes()) as wire:
    feedback = feedback_from_wire(wire.actuators.navigatorLongitudinal)
  assert allocator_feedback_valid(feedback)
  assert not feedback.brake and not feedback.precharge and feedback.gas == -5.
  assert Feedback.decode(feedback.encode()) == feedback
  assert feedback.precharge_age == .04 and feedback.budget_remaining == .009


def test_supported_pair_admits_trial_but_older_peer_uses_nominal_fallback():
  cc = coast_message()
  feedback = feedback_from_wire(cc.actuators.navigatorLongitudinal)
  for version, expected in [(3, 3), (1, 0), (2, 0), (99, 0)]:
    cc.navigatorLongitudinal.producerMonoTime = 1_020_000_000
    stamp_allocator_request(cc, 1_020_000_000, target=.1, target_ns=1_010_000_000,
                            target_valid=True, feedback=replace(feedback, allocator_version=version))
    assert cc.navigatorLongitudinal.allocatorVersion == expected
    assert allocator_request_valid(cc.navigatorLongitudinal, 1_020_000_000) == (version == 3)


def test_expired_or_unbounded_coast_cannot_opt_into_trial():
  cc = coast_message()
  feedback = feedback_from_wire(cc.actuators.navigatorLongitudinal)
  for changes in [{'precharge_age': .31}, {'budget_remaining': 0.}, {'gas': .1}, {'limited': False}]:
    cc.navigatorLongitudinal.producerMonoTime = 1_020_000_000
    stamp_allocator_request(cc, 1_020_000_000, target=.1, target_ns=1_010_000_000,
                            target_valid=True, feedback=replace(feedback, **changes))
    assert cc.navigatorLongitudinal.allocatorVersion == 0
