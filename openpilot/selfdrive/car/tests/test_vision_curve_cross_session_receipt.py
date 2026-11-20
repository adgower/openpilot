from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.vision_curve_button_transport import BoundedButtonTransport
from openpilot.selfdrive.car.vision_curve_context import FinalRestoreLeadCheck
from openpilot.selfdrive.car.tests.test_vision_curve_recovery import Rig
from openpilot.selfdrive.car.tests.test_vision_curve_session_reset import interrupt


def test_closing_lead_requires_fresh_clear_samples_before_final_increase():
  check = FinalRestoreLeadCheck()
  lead = SimpleNamespace(present=True, dRel=60., vRel=-.9)
  radar = SimpleNamespace(leadOne=SimpleNamespace(present=False), leadTwo=lead)

  def allowed(ms, fresh=True):
    return check.allowed(radar, ms * 1_000_000, fresh, speed=26., set_speed=69.)

  assert not allowed(1000)
  lead.vRel = .001
  assert not allowed(1050)
  assert not allowed(1050)  # Re-reading the same radar sample adds no evidence.
  lead.vRel = -.01
  assert not allowed(1100)
  lead.present = False
  assert not allowed(1150)
  assert not allowed(1200, fresh=False)
  assert not allowed(1300)
  assert not allowed(1600)  # A gap requires a new clearance window.
  assert not allowed(1650)
  assert allowed(1700)


def test_deferred_final_increase_keeps_receipts_and_allows_later_bends():
  rig = Rig().run(2., forecast=69, clear=False)
  mark = len(rig.pulses)
  rig.run(3., clear=True, allow_final_restore=False)
  assert rig.observed == 69 and not rig.pulses[mark:] and not rig.c.transport.pending
  rig.run(2., forecast=65, clear=False, allow_final_restore=False)
  assert rig.observed == 65
  rig.run(3., clear=True, allow_final_restore=False)
  assert rig.observed == 69 and not rig.c.transport.pending
  mark = len(rig.pulses)
  rig.run(1., clear=True)
  assert rig.observed == 70 and [button for _, button in rig.pulses[mark:]] == ["SET+"]


def test_traffic_defers_last_credit_with_delayed_display_for_new_and_repeated_presses():
  transport = BoundedButtonTransport(68, ceiling_mph=70, press_s=.13, plus_interval=.18)
  for tick in range(100):
    out = transport.update(tick / 100., 68, 70, allow_final_restore=False)
    if tick == 0:
      assert out.reason == 'new-pulse'
    else:
      assert out.reason not in ('new-pulse', 'repeat-pulse')
    assert transport.up_debt == 1
  transport.update(1., 69, 70, allow_final_restore=False)
  assert transport.up_debt == 0
  assert transport.update(1.01, 69, 70).reason == 'new-pulse'
  transport.update(1.06, 70, 70)
  assert not transport.pending and not transport.fault


@pytest.mark.parametrize("transition", ["cancel", "off", "resume", "resume_after_blank", "held_resume"])
def test_native_reengagement_cannot_acknowledge_an_old_final_increment(transition):
  rig = Rig().run(2., forecast=69, clear=False).run(1., clear=True, ack=False)
  assert rig.observed == 69 and rig.c.transport.up_debt == 1
  if transition == "cancel":
    rig.observed = 70
    rig.run(.1, user_action="cancel", manual=True)
  interrupt(rig, "cancel")
  if transition == "off":
    rig.observed = 70
    rig.run(.1, engaged=False, cruise_engaged=False)
  rig.run(.1, user_action="resume", manual=True, engaged=False, cruise_engaged=False)
  if transition == "resume_after_blank":
    rig.observed = 0
    rig.run(.15, valid=False, cruise_state_valid=True)
  if transition == "held_resume":
    rig.run(.15, user_action="resume", manual=True)
    rig.observed = 70
    rig.run(.1, user_action="resume", manual=True)
  if transition in ("resume", "resume_after_blank"):
    rig.observed = 70
  rig.run(.1, clear=False)

  # Ford restored its own display while changing modes. The old SET+ can
  # still arrive during the next bend. A second final SET+ would overshoot.
  rig.run(2., forecast=69, clear=False)
  assert rig.observed == 69
  mark = len(rig.pulses)
  rig.run(2., clear=True)
  assert not rig.pulses[mark:]
  assert rig.c.transport.up_debt == 1
  rig.observed += 1
  rig.run(.5, clear=True)
  assert rig.observed == rig.c.ceiling == 70 and not rig.c.transport.pending
  assert not rig.c.fault


def test_retained_final_increment_after_driver_resets_ceiling_inhibits_all_output():
  rig = Rig().run(2., forecast=69, clear=False).run(1., clear=True, ack=False)
  assert rig.observed == 69 and rig.c.transport.up_debt == 1
  mark = len(rig.pulses)
  interrupt(rig, "cancel")
  rig.observed = 58
  rig.run(.1, user_action="set", user_set_direction=1, manual=True)
  for display in (60, 65, 70):
    rig.observed = display
    rig.run(.1, user_action="set", user_set_direction=1, manual=True)
  rig.run(.6, clear=True)
  assert rig.observed == rig.c.ceiling == 70 and rig.c.transport.up_debt == 1

  # The receiver, outside the controller, applies the single pre-cancel SET+.
  rig.observed += 1
  rig.run(.7, forecast=60, clear=False).run(.7, clear=True)
  assert rig.observed == 71 and rig.c.ceiling == 70
  assert (rig.c.fault or rig.c.suspended) and not rig.pulses[mark:]


@pytest.mark.parametrize("direction", [1, -1])
@pytest.mark.parametrize(("during_hold", "step"), [(False, 1), (True, 1), (True, 5)])
def test_matching_old_command_during_physical_set_requires_a_new_driver_selection(direction, during_hold, step):
  if direction > 0:
    rig = Rig().run(2., forecast=69, clear=False).run(1., clear=True, ack=False)
  else:
    rig = Rig().run(.03, forecast=65, clear=False, ack=False)
  assert rig.c.transport.debt[direction] == 1
  mark = len(rig.pulses)
  if during_hold:
    rig.run(.1, user_action="set", user_set_direction=direction, manual=True)
  rig.observed += direction * step
  rig.run(.1, user_action="set", user_set_direction=direction, manual=True)
  rig.run(.6, clear=True)
  assert rig.c.suspended and rig.c.ceiling == 70
  assert rig.c.transport.debt[direction] == 1 and not rig.pulses[mark:]

  # After release, a separate deliberate SET confirms a stable higher setting.
  rig.observed = 75
  rig.run(.1, clear=False)
  rig.run(.1, user_action="set", user_set_direction=1, manual=True)
  rig.run(.6, clear=False)
  assert not rig.c.suspended and rig.c.ceiling == 75
  assert rig.c.transport.debt[direction] == 1 and not rig.pulses[mark:]


def test_prompt_final_restore_and_later_deliberate_higher_set():
  rig = Rig().run(2., forecast=69, clear=False)
  mark = len(rig.pulses)
  rig.run(1., clear=True)
  assert [button for _, button in rig.pulses[mark:]] == ["SET+"]
  assert rig.observed == rig.c.ceiling == 70 and not rig.c.transport.pending
  rig.run(.1, user_action="set", user_set_direction=1, manual=True)
  rig.observed = 75
  rig.run(.1, user_action="set", user_set_direction=1, manual=True)
  rig.run(.6, clear=False)
  assert rig.c.ceiling == 75 and not rig.c.suspended and not rig.c.fault
