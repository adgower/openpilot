import pytest

from openpilot.selfdrive.car.tests.test_geometry_following_return import setup_geometry, tick
from openpilot.selfdrive.car.tests.test_vision_curve_recovery import Rig
from openpilot.selfdrive.car.vision_curve_speed import MPH
from openpilot.selfdrive.car.tests.test_vision_curve_session_reset import interrupt, signal_resume


def test_fresh_resume_clears_debt_free_display_fault_at_original_ceiling():
  r = Rig().run(1.0, clear=False)
  r.observed = 69
  r.run(0.1, clear=False)
  assert r.c.fault and r.c.transport.pending == 0
  r.observed = 70
  interrupt(r, 'cancel')
  signal_resume(r)
  r.run(1.0, forecast=60, clear=False)
  assert r.c.ceiling == 70 and not r.c.fault and r.pulses


@pytest.mark.parametrize('block', ['pending_at_fault', 'monotonic_fault', 'wrong_ceiling', 'resume_expired', 'no_resume', 'onroad_button_only'])
def test_only_verified_debt_free_display_fault_rearms(block):
  if block == 'pending_at_fault':
    r = Rig().run(2.0, forecast=69, clear=False).run(1.0, clear=True, ack=False)
    assert r.c.transport.up_debt == 1
    r.observed = 67
  else:
    r = Rig().run(1.0, clear=False)
    r.observed = 69
    if block == 'monotonic_fault':
      r.c.transport.last_now = r.t + 10.0
  r.run(0.1, clear=False)
  assert r.c.fault
  origin = len(r.pulses)
  r.observed = 70
  interrupt(r, 'cancel')
  if block == 'wrong_ceiling':
    r.observed = 69
  if block == 'no_resume':
    r.run(0.1, cruise_engaged=False, engaged=False)
  elif block == 'onroad_button_only':
    r.run(0.1, user_action='resume', manual=True)
  else:
    r.run(0.1, user_action='resume', manual=True, cruise_engaged=False, engaged=False)
    if block == 'resume_expired':
      r.run(1.1, cruise_engaged=False, engaged=False)
  r.run(1.0, forecast=60, clear=False)
  assert r.c.fault and not r.pulses[origin:]


def test_resume_issued_before_debt_free_fault_cannot_clear_later_disagreement():
  r = Rig().run(1.0, clear=False)
  interrupt(r, 'cancel')
  r.run(0.1, cruise_engaged=False, engaged=False, user_action='resume', manual=True)
  assert r.c.resume_armed
  r.observed = 69
  r.run(0.1, cruise_engaged=False, engaged=False, clear=False)
  assert r.c.fault
  r.observed = 70
  r.run(1.0, forecast=60, clear=False)
  assert r.c.fault and not r.pulses


@pytest.mark.parametrize('phase', ['off_edge', 'on_confirm'])
def test_stale_native_cruise_cannot_release_debt_free_display_fault(phase):
  r = Rig().run(1.0, clear=False)
  r.observed = 69
  r.run(0.1, clear=False)
  assert r.c.fault and r.c.transport.pending == 0
  r.observed = 70
  interrupt(r, 'cancel')
  r.run(0.1, user_action='resume', manual=True, cruise_engaged=False, engaged=False, cruise_state_valid=phase != 'off_edge')
  if phase == 'on_confirm':
    r.run(1.1, cruise_state_valid=False, valid=False, clear=False)
  r.run(1.0, forecast=60, clear=False)
  assert r.c.fault and not r.pulses


def test_verified_reengage_waits_for_valid_recovery_inputs_and_new_curve():
  r = Rig().run(1.0, clear=False)
  r.observed = 69
  r.run(0.1, clear=False)
  assert r.c.fault
  r.observed = 70
  interrupt(r, 'cancel')
  r.run(0.1, user_action='resume', manual=True, cruise_engaged=False, engaged=False)
  r.run(0.65, model=None, clear=False, valid=False, cruise_state_valid=True, forecast_available=False)
  assert not r.c.fault and not r.pulses
  r.run(0.3, bend=True, clear=False, forecast_authoritative=True)
  assert not r.pulses
  r.run(0.8, bend=True, clear=False, forecast_authoritative=True)
  assert any(b == 'SET-' for _, b in r.pulses)


def test_ford_physical_cancel_and_resume_rearms_bend_control():
  fixture, model, controller = setup_geometry()
  switches = fixture.parser.vl['Steering_Data_FD1']
  stock = fixture.state.cruiseState
  model.orientationRate.z = [0.0] * 6
  for i in range(10):
    tick(fixture, i)
  stock.speedCluster = 69 * MPH
  tick(fixture, 10)
  assert controller.fault and controller.transport.pending == 0
  switches['CcAslButtnCnclResPress'] = 1
  tick(fixture, 11)
  stock.enabled = False
  stock.speedCluster = 70 * MPH
  tick(fixture, 12)
  switches.clear()
  tick(fixture, 13)
  switches['CcAslButtnCnclResPress'] = 1
  tick(fixture, 14)
  assert controller.suspended and controller.resume_armed
  stock.enabled = True
  tick(fixture, 15)
  switches.clear()
  tick(fixture, 16)
  assert not controller.suspended and not controller.fault
  model.orientationRate.z = [0.12] * 6
  assert any(tick(fixture, i) == 'SET-' for i in range(17, 65))
