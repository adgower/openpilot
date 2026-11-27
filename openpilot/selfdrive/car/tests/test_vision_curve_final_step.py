import pytest

from openpilot.selfdrive.car.vision_curve_button_transport import BoundedButtonTransport
from openpilot.selfdrive.car.vision_curve_recovery import AcknowledgedCurveSpeed


def final_step():
  return BoundedButtonTransport(69, ceiling_mph=70, press_s=.13, minus_interval=.18, plus_interval=.18, burst=3)


def issued(out):
  return out.reason in ("new-pulse", "repeat-pulse")


def test_final_step_requires_continuous_permission_before_issue():
  transport = final_step()
  sends = []
  seen = 69
  for tick in range(150):
    # A neutral native button arrives once per ten ticks. Two brief model
    # permission windows end before the simulated receiver can sample the level.
    allowed = 12 <= tick < 17 or 44 <= tick < 47 or tick >= 80
    out = transport.update(tick / 100, seen, 70, allowed=allowed, can_assert=tick % 10 == 5)
    if issued(out):
      sends.append(tick)
    if sends and tick == sends[-1] + 3 and out.phase == "press":
      seen += 1
    assert seen <= 70 and transport.observed + transport.up_debt <= 70
  assert sends == [95]
  assert (seen, transport.up_debt) == (70, 0)


@pytest.mark.parametrize("receipt_tick", [None, 280, 1900])
def test_missing_or_late_final_step_across_driver_pause(receipt_tick):
  transport = final_step()
  sends = []
  seen = 69
  for tick in range(2500):
    if tick == receipt_tick:
      seen += 1
    # Model eligibility can return after a manual override. Neither event
    # is an ECU receipt for a previous press.
    allowed = not 260 <= tick < 310
    out = transport.update(tick / 100, seen, 70, allowed=allowed)
    if issued(out):
      assert allowed
      sends.append(tick)
    assert seen + transport.up_debt <= 70
  assert len(sends) == 1
  assert (seen, transport.up_debt) == ((69, 1) if receipt_tick is None else (70, 0))


def test_settling_restarts_after_missing_callbacks():
  transport = final_step()
  for t in (0, .05, .10, 1.0, 1.05, 1.1):
    assert not issued(transport.update(t, 69, 70))
  assert issued(transport.update(1.14, 69, 70))


@pytest.mark.parametrize("target,observed,button", [(66,69,"SET-"),(70,67,"SET+")])
def test_interior_or_decrement_has_no_extra_dwell(target, observed, button):
  transport = BoundedButtonTransport(observed, ceiling_mph=70)
  assert transport.update(0, observed, target).button == button


def test_driver_set_preserves_inflight_credit():
  cruise = AcknowledgedCurveSpeed()
  cruise._start(70)
  cruise.transport = final_step()
  cruise.target = 70
  for tick in range(17):
    cruise._send(tick / 100, 69, True)
  assert cruise.transport.up_debt == 1
  args = dict(engaged=True, valid=True, manual=False, override=False, speed=69, accel=0, model=None, cruise_engaged=True)
  assert cruise.update(.20, user_action="set", set_speed=69, **args) == ""
  cruise.update(.65, user_action=None, set_speed=70, **args)
  assert cruise.ceiling == 70 and cruise.transport.up_debt == 1
  assert all(cruise._send(tick / 100,70,True) != "SET+" for tick in range(66,120))


@pytest.mark.parametrize("first_press_missed", [False, True])
def test_restored_cruise_keeps_one_decrement_available_for_the_next_bend(first_press_missed):
  transport = BoundedButtonTransport(70)
  observed = 70
  tick = 0

  def run(target, duration, *, receive=True, allowed=True):
    nonlocal observed, tick
    commands = []
    for _ in range(duration):
      tick += 1
      out = transport.update(tick / 100, observed, target, allowed=allowed)
      assert not transport.fault
      assert transport.floor <= observed - transport.down_debt
      assert observed + transport.up_debt <= transport.ceiling
      assert transport.down_debt <= transport.burst
      if issued(out):
        commands.append(out.button)
        if receive:
          observed += 1 if out.button == "SET+" else -1
    return commands

  # Ford followed the reduction, then ignored two presses. Preserve both in
  # case they execute later; leave headroom for a subsequent acknowledged bend.
  assert run(65, 250) == ["SET-"] * 5
  assert run(55, 300, receive=False) == ["SET-"] * 2
  assert run(70, 400) == ["SET+"] * 5
  assert observed == 70 and transport.down_debt == 2
  assert not run(60, 100, allowed=False)
  if first_press_missed:
    assert run(60, 500, receive=False) == ["SET-"]
    assert transport.down_debt == 3
    assert not run(55, 1000)
    observed -= 1  # One real, late receipt opens one slot.
  run(60, 500)
  assert observed == 62 and transport.down_debt == 2
  # After reversal, all old decrements can still arrive as one display jump.
  # Their receipt is not a new driver selection and cannot create extra SET+.
  assert run(70, 1000) == ["SET+"] * 8
  observed -= 2
  assert run(70, 500) == ["SET+"] * 2
  assert observed == 70 and transport.pending == 0


def test_manual_selection_does_not_release_reserved_decrement_slot():
  cruise = AcknowledgedCurveSpeed()
  cruise._start(70)
  transport = cruise.transport
  assert issued(transport.update(0., 70, 65))
  transport.update(.05, 69, 65)
  for tick in range(6, 100):
    transport.update(tick / 100, 69, 60)
  assert transport.down_debt == 2 and transport.reserve_down_slot

  args = dict(engaged=True, valid=True, manual=False, override=False, speed=69.,
              accel=0., model=None, cruise_engaged=True)
  cruise.update(1., set_speed=68., user_action="set", **args)
  cruise.update(1.5, set_speed=68., **args)
  assert cruise.ceiling == 68
  assert cruise.transport.down_debt == 2 and cruise.transport.reserve_down_slot
  assert not issued(cruise.transport.update(2., 68, 60))


def test_brief_permission_cannot_resume_a_blocked_following_return():
  cruise = AcknowledgedCurveSpeed()
  cruise._start(65)
  cruise.transport.observed = cruise.observed = 55
  cruise.target = cruise.geometry_target = cruise.current_demand = 65.
  cruise.geometry_trimmed = cruise.model_ok = True
  cruise.geometry_clear_since = cruise.following_clear_since = 0.
  inputs = dict(engaged=True, valid=True, manual=False, override=False, speed=55., accel=0.,
                set_speed=55., model=None, cruise_engaged=True, early_hint_inhibited=True,
                forecast_authoritative=True, forecast_available=True)
  issued = []
  for tick in range(400):
    # The 2.5 s geometry dwell completes just before following becomes unsafe.
    # Only one native neutral frame is available in this first opening.
    following_clear = tick < 255 or tick >= 260
    if tick == 260:
      cruise.following_clear_since = 2.6  # New model evidence starts the following dwell.
    if cruise.update(tick / 100, **inputs, following_clear=following_clear, can_assert=tick in (254, 390)) == 'SET+':
      issued.append(tick)
  assert issued == [390]
  assert cruise.transport.up_debt == 1


def test_new_ceiling_target_after_interior_step_keeps_its_first_opportunity():
  cruise = AcknowledgedCurveSpeed()
  cruise._start(70)
  cruise.transport.observed = cruise.observed = 48
  cruise.target = 48
  assert cruise._send(1., 48, False) != 'SET+'
  cruise.target = 70
  assert cruise._send(1.01, 48, True, settle_following_return=True) == 'SET+'
  # Once this full return is interrupted, the next interior step must settle.
  assert cruise._send(1.5, 49, False) != 'SET+'
  assert cruise._send(1.51, 49, True, settle_following_return=True) != 'SET+'
  assert cruise._send(1.60, 49, False) != 'SET+'
  for tick in range(161, 174):
    assert cruise._send(tick / 100, 49, True, settle_following_return=True) != 'SET+'
  assert cruise._send(1.75, 49, True, settle_following_return=True) == 'SET+'
