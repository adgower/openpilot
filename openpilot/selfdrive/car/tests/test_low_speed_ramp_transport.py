"""Selected guard consumes real serialized versioned feedback."""

import pytest
from opendbc.car import structs
from opendbc.car.ford.longitudinal_coordination import feedback_from_wire, freeze_integral


def received(version=1, phase='engineRamp', **changes):
  output = structs.CarControl.Actuators()
  f = output.navigatorLongitudinal
  f.producerMonoTime = 10_000_000_000
  f.epoch = 7
  f.sequence = 10
  f.requested = 1.74
  f.gas = 0.64
  f.limited = True
  f.valid = True
  f.allocatorVersion = version
  f.allocatorPhase = phase
  for key, value in changes.items():
    setattr(f, key, value)
  with structs.CarControl.Actuators.from_bytes(output.to_bytes()) as wire:
    return feedback_from_wire(wire.navigatorLongitudinal)


def held(f, **change):
  d = {'now': 10.0, 'epoch': 7, 'sequence': 11, 'error': 0.65, 'target': 1.17, 'wanted': 1.74, 'speed': 1.65, 'phase': 'pid', 'feedback': f, 'enabled': True}
  d.update(change)
  return freeze_integral(**d)


def test_real_engine_ramp_feedback_holds_only_positive_low_speed_error():
  f = received()
  assert held(f)
  assert not held(f, error=-0.1)
  assert not held(f, target=-0.6)
  assert not held(f, now=10.2)


@pytest.mark.parametrize('version,phase', [(0, 'engineRamp'), (2, 'engineRamp'), (1, 'drive'), (1, 'release'), (1, 99)])
def test_other_peer_versions_or_modes_keep_nominal_low_speed_behavior(version, phase):
  assert not held(received(version, phase))


def test_real_brake_and_precharge_feedback_do_not_gain_low_speed_hold():
  assert not held(received(brake=True, precharge=True, referenceBrake=True))
  assert not held(received(phase='precharge', precharge=True, referenceBrake=True, gas=-5.0, prechargeAge=0.1, budgetRemaining=0.004))
