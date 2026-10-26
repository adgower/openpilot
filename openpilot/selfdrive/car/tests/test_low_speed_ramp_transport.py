"""Qualified command recovery consumes real serialized versioned feedback."""

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


def test_unqualified_real_engine_ramp_does_not_broadly_freeze_integral():
  f = received()
  assert not held(f)
  assert not held(f, error=-0.1)
  assert not held(f, target=-0.6)
  assert not held(f, now=10.2)


@pytest.mark.parametrize('version,phase', [(0, 'engineRamp'), (2, 'engineRamp'), (1, 'drive'), (1, 'release'), (1, 99)])
def test_other_peer_versions_or_modes_keep_nominal_low_speed_behavior(version, phase):
  assert not held(received(version, phase))


def test_real_brake_and_precharge_feedback_do_not_gain_low_speed_hold():
  assert not held(received(brake=True, precharge=True, referenceBrake=True))
  assert not held(received(phase='precharge', precharge=True, referenceBrake=True, gas=-5.0, prechargeAge=0.1, budgetRemaining=0.004))


def test_qualified_recovery_reduces_command_from_serialized_feedback():
  from dataclasses import replace
  from types import SimpleNamespace as NS
  from opendbc.car.ford.longitudinal_recovery import CommandRecovery
  recovery = CommandRecovery()
  cs = NS(vEgo=5., aEgo=0., canValid=True, gasPressed=False, brakePressed=False, standstill=False,
          cruiseState=NS(standstill=False))
  f = received()
  recovery.prepare(True, cs, 1.17, False, replace(f, t=9.975), 9.98, 7, 11, True, True, False)
  cs.vEgo = 2.
  braking = replace(f, t=9.985, brake=True, precharge=True, reference_brake=True, gas=-5., allocator_phase='brake')
  recovery.prepare(True, cs, 1.17, False, braking, 9.99, 7, 12, True, True, False)
  cs.vEgo = 1.65
  recovery.prepare(True, cs, 1.17, False, f, 10., 7, 13, True, True, False)
  assert recovery.apply(1.74, .01, 1.74, 'pid') == pytest.approx(1.73)
  assert recovery.reduction == pytest.approx(.01)
