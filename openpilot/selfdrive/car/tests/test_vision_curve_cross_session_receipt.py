import pytest

from openpilot.selfdrive.car.tests.test_vision_curve_recovery import Rig
from openpilot.selfdrive.car.tests.test_vision_curve_session_reset import interrupt


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
