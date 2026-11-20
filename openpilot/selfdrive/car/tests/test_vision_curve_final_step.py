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
