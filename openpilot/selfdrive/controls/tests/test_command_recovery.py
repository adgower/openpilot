"""Production-PID wrapper checks with truthful synthetic software feedback."""

import copy
from types import SimpleNamespace as NS
import pytest
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from opendbc.car.ford.longitudinal_coordination import Feedback


def controller(monkeypatch):
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.ford.values import CAR

  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  loc = LongControl(cp)
  return loc


class Stream:
  def __init__(self, loc):
    self.loc = loc
    self.t = 1.0
    self.seq = 0
    self.out = 0.6

  def step(self, speed, phase='drive', *, target=0.8, actual=0.0, active=True, epoch=1, valid=True, pedals=False, age=0.005, limits=(-3.5, 2.0)):
    self.t += 0.01
    self.seq += 1
    brake = phase == 'brake'
    limited = phase != 'drive'
    fb = Feedback(
      self.t - age,
      epoch,
      self.seq,
      self.out,
      -5.0 if brake else 0.2 if limited else self.out,
      brake,
      limited,
      True,
      False,
      allocator_version=1,
      precharge=brake,
      reference_brake=brake,
      allocator_phase=phase,
    )
    cs = NS(vEgo=speed, aEgo=actual, canValid=valid, gasPressed=pedals, brakePressed=False, standstill=False, cruiseState=NS(standstill=False))
    self.out = self.loc.update(
      active,
      cs,
      target,
      False,
      limits,
      feedback=fb,
      now=self.t,
      source_valid=True,
      urgent=getattr(self.loc, "test_urgent", False),
      epoch=epoch,
      sequence=self.seq,
      coordinated=True,
    )
    return self.out

  def hold(self):
    self.step(5.0)
    self.step(2.0, 'brake', actual=-0.3)
    for _ in range(40):
      self.step(1.5, 'engineRamp')
    assert self.loc.command_recovery.reduction > 0


def test_native_integral_not_repaid_during_command_recovery(monkeypatch):
  loc = controller(monkeypatch)
  s = Stream(loc)
  s.step(5.0)
  s.step(2.0, 'brake', actual=-0.3)
  s.step(1.5, 'engineRamp')
  for _ in range(35):
    native = copy.copy(loc.pid)
    native.set_limits(2.0, -3.5)
    expected = native.update(0.8, speed=1.5, feedforward=0.8, freeze_integrator=False)
    s.step(1.5, 'engineRamp')
    assert loc.pid.i == pytest.approx(native.i)
    assert s.out <= expected
  assert 0 < loc.command_recovery.reduction <= 0.25
  for _ in range(400):
    before = copy.copy(loc.pid)
    before.set_limits(2.0, -3.5)
    expected = before.update(-0.3, speed=4.0, feedforward=0.8, freeze_integrator=False)
    reduction = loc.command_recovery.reduction
    s.step(4.0, actual=1.1)
    assert loc.pid.i == pytest.approx(before.i)
    assert loc.command_recovery.reduction <= reduction
    assert reduction - loc.command_recovery.reduction <= 0.001 + 1e-12
    assert s.out <= expected + 1e-12
  assert loc.command_recovery.reduction == 0


@pytest.mark.parametrize('kw', [{'valid': False}, {'pedals': True}, {'age': 0.2}, {'epoch': 2}])
def test_invalid_overlay_never_rewrites_native_output(monkeypatch, kw):
  loc = controller(monkeypatch)
  s = Stream(loc)
  s.step(5.0)
  s.step(2.0, 'brake', actual=-0.3)
  for _ in range(30):
    s.step(1.5, 'engineRamp')
  assert loc.command_recovery.reduction > 0
  s.step(4.0, actual=0.5, **kw)
  assert loc.command_recovery.last_command_reduction == 0
  assert s.out == loc.last_output_accel


def test_braking_and_reset_do_not_apply_propulsion_reduction(monkeypatch):
  loc = controller(monkeypatch)
  s = Stream(loc)
  s.step(5.0)
  s.step(2.0, 'brake', actual=-0.3)
  for _ in range(30):
    s.step(1.5, 'engineRamp')
  assert loc.command_recovery.reduction > 0
  s.step(2.0, 'brake', target=-1.0, actual=-0.5)
  assert loc.command_recovery.last_command_reduction == 0
  s.step(2.0, active=False)
  assert loc.command_recovery.reduction == 0 and loc.pid.i == 0


@pytest.mark.parametrize('target,phase,speed', [(0.0, 'drive', 4.0), (-0.5, 'drive', 4.0), (0.8, 'brake', 4.0), (0.8, 'engineRamp', 14.0)])
def test_overlay_cannot_reduce_braking_or_operate_at_highway_speed(monkeypatch, target, phase, speed):
  loc = controller(monkeypatch)
  s = Stream(loc)
  s.step(5.0)
  s.step(2.0, 'brake', actual=-0.3)
  for _ in range(30):
    s.step(1.5, 'engineRamp')
  s.step(speed, phase, target=target)
  assert loc.command_recovery.last_command_reduction == 0


def test_urgent_demand_suppresses_overlay_without_positive_integral_step(monkeypatch):
  loc = controller(monkeypatch)
  s = Stream(loc)
  s.step(5.0)
  s.step(2.0, 'brake', actual=-0.3)
  for _ in range(30):
    s.step(1.5, 'engineRamp')
  loc.test_urgent = True
  native = copy.copy(loc.pid)
  native.set_limits(2.0, -3.5)
  expected = native.update(-0.3, speed=4.0, feedforward=0.8, freeze_integrator=False)
  s.step(4.0, actual=1.1)
  assert s.out == pytest.approx(expected) and loc.pid.i == pytest.approx(native.i)


def test_sequence_rollback_cancels_overlay_without_resetting_native_integral(monkeypatch):
  loc = controller(monkeypatch)
  s = Stream(loc)
  s.step(5.0)
  s.step(2.0, 'brake', actual=-0.3)
  for _ in range(30):
    s.step(1.5, 'engineRamp')
  assert loc.command_recovery.reduction > 0.1
  s.seq = 1
  native = copy.copy(loc.pid)
  native.set_limits(2.0, -3.5)
  expected = native.update(0.8, speed=1.5, feedforward=0.8, freeze_integrator=False)
  s.step(1.5, 'engineRamp')
  assert loc.command_recovery.last_command_reduction == 0 and loc.command_recovery.reduction == 0
  assert loc.pid.i == pytest.approx(native.i) and s.out == pytest.approx(expected)
  s.step(1.5, 'engineRamp')
  assert loc.command_recovery.last_command_reduction == 0


def test_own_output_constraint_prevents_positive_windup_after_early_drive(monkeypatch):
  loc = controller(monkeypatch)
  s = Stream(loc)
  s.step(5.0)
  s.step(2.0, 'brake', actual=-0.3)
  for _ in range(30):
    s.step(1.5, 'engineRamp')
  prior = loc.pid.i
  s.step(4.0, 'drive', actual=0.5)
  assert loc.last_integral_frozen
  assert loc.pid.i == pytest.approx(prior)
  assert loc.command_recovery.last_command_reduction > 0
  # Negative error remains free to recover even under the local constraint.
  prior = loc.pid.i
  s.step(4.0, 'drive', actual=1.1)
  assert not loc.last_integral_frozen and loc.pid.i < prior
