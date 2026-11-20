import pytest

from openpilot.selfdrive.car.tests.test_vision_curve_recovery import Rig


def interrupt(rig, reason):
  signals = {'brake': {'override': True, 'override_kind': 'brake'},
             'cancel': {'user_action': 'cancel', 'manual': True},
             'stock_off': {}}[reason]
  rig.run(.2, engaged=False, cruise_engaged=False, **signals)


def signal_resume(rig):
  rig.run(.1, user_action='resume', manual=True, engaged=False, cruise_engaged=False)
  rig.run(.05, model=None, clear=False)


@pytest.mark.parametrize('reason', ['brake', 'cancel', 'stock_off'])
def test_interrupted_forecast_requires_fresh_evidence_before_next_set_minus(reason):
  r = Rig().run(1., forecast=55, clear=False, can_assert=False, forecast_authoritative=True)
  assert r.c.target == 55 and r.observed == 70
  interrupt(r, reason)
  mark = len(r.pulses)
  signal_resume(r)
  r.run(.65, forecast=None, clear=False, forecast_authoritative=True)
  assert not r.c.suspended
  assert not r.pulses[mark:]
  r.run(1., forecast=65, clear=False, forecast_authoritative=True)
  assert r.observed == 65 and r.c.ceiling == 70


def test_interrupted_curve_requires_fresh_bend_admission_after_resume():
  r = Rig().run(1.6, bend=True, clear=False, can_assert=False, forecast_authoritative=True)
  assert r.c.geometry_hold is not None
  interrupt(r, 'brake')
  signal_resume(r)
  mark = len(r.pulses)
  r.run(.4, bend=True, clear=False, forecast_authoritative=True)
  assert not r.pulses[mark:]
  r.run(1., bend=True, clear=False, forecast_authoritative=True)
  assert any(b=='SET-' for _, b in r.pulses[mark:])


def test_pending_restore_is_dropped_until_new_clear_evidence():
  r = Rig().run(2., forecast=64, clear=False, forecast_authoritative=True)
  assert r.observed == 64
  r.run(.3, clear=True, can_assert=False, forecast_authoritative=True)
  assert r.c.target == 70
  interrupt(r, 'cancel')
  mark = len(r.pulses)
  signal_resume(r)
  r.run(.6, clear=False, forecast_authoritative=True)
  assert not r.pulses[mark:]
  r.run(2., clear=True, forecast_authoritative=True)
  assert r.observed == r.c.ceiling == 70


def test_unaccepted_old_resume_tap_does_not_rearm_unrelated_engagement():
  r = Rig().run(1.)
  interrupt(r, 'brake')
  r.run(.1, engaged=False, cruise_engaged=False, user_action='resume', manual=True)
  r.run(4., engaged=False, cruise_engaged=False)
  mark = len(r.pulses)
  r.run(1.5, forecast=50, clear=False)
  assert r.c.suspended and not r.pulses[mark:]
  r.run(.1, user_action='resume', manual=True)
  r.run(1., forecast=60, clear=False)
  assert not r.c.suspended and any(b=='SET-' for _, b in r.pulses[mark:])


@pytest.mark.parametrize('second_input',['brake','cancel','stock_off'])
def test_interrupted_manual_selection_cannot_rearm_a_later_engagement(second_input):
  r = Rig().run(1.)
  r.observed = 65
  r.run(.1, user_action='set', manual=True)
  interrupt(r, second_input)
  r.run(1., engaged=False, cruise_engaged=False)
  r.run(1., forecast=55, clear=False)
  assert r.c.suspended and r.c.ceiling == 70
  r.run(.1, user_action='set', manual=True)
  r.run(.6, clear=False)
  assert not r.c.suspended and r.c.ceiling == 65


def test_set_while_off_must_be_acknowledged_by_a_timely_oem_engagement():
  r = Rig().run(1.)
  interrupt(r, 'brake')
  r.run(.1, user_action='set', manual=True, engaged=False, cruise_engaged=False)
  r.run(4., engaged=False, cruise_engaged=False)
  r.observed = 63
  r.run(1., forecast=50, clear=False)
  assert r.c.ceiling == 70 and r.c.suspended and not r.pulses
  r.run(.1, user_action='set', manual=True)
  r.run(.5, clear=False)
  assert r.c.ceiling == 63 and not r.c.suspended


def test_late_decrement_after_resume_is_not_a_new_positive_speed_request():
  r = Rig().run(.03, forecast=60, clear=False, ack=False, forecast_authoritative=True)
  assert r.c.transport.down_debt == 1
  interrupt(r, 'brake')
  signal_resume(r)
  mark = len(r.pulses)
  r.observed = 69
  r.run(1., clear=False, forecast_authoritative=True)
  assert not r.pulses[mark:]
  assert r.c.transport.down_debt == 0
  r.run(1., clear=True, forecast_authoritative=True)
  assert r.observed == 70


def test_unaccepted_set_cannot_claim_engagement_after_a_sample_gap():
  r=Rig().run(1.)
  interrupt(r,'brake')
  r.run(.01, user_action='set', manual=True, engaged=False, cruise_engaged=False)
  r.t += 4.
  r.observed = 65
  r.run(1.,forecast=50,clear=False)
  assert r.c.ceiling == 70 and r.c.suspended
  assert not r.pulses


@pytest.mark.parametrize('condition', ['invalid','lateral_inactive','gas'])
def test_prompt_native_resume_survives_a_delayed_model_lateral_or_gas(condition):
  r=Rig().run(1.)
  interrupt(r,'brake')
  r.run(.1, engaged=False, cruise_engaged=False, user_action='resume', manual=True)
  inhibitors={'invalid':dict(valid=False),'lateral_inactive':dict(engaged=False),'gas':dict(override=True,override_kind='gas')}
  r.run(2., cruise_state_valid=True, **inhibitors[condition])
  assert not r.pulses
  r.run(1.5,forecast=60,clear=False)
  assert any(b=='SET-' for _,b in r.pulses)


def test_old_resume_cannot_claim_unrelated_engagement_after_a_sample_gap():
  r=Rig().run(1.)
  interrupt(r,'brake')
  r.run(.1, engaged=False, cruise_engaged=False, user_action='resume', manual=True)
  r.t += 4.
  r.run(1., forecast=55, clear=False, cruise_state_valid=True)
  assert r.c.suspended and not r.pulses


def test_invalid_native_status_does_not_confirm_old_resume_then_restart():
  r=Rig().run(1.)
  interrupt(r,'brake')
  r.run(.1, engaged=False, cruise_engaged=False, user_action='resume', manual=True)
  r.run(2.,valid=False,cruise_state_valid=False)
  r.run(1.,forecast=50,clear=False,cruise_state_valid=True)
  assert not r.pulses and r.c.suspended


def test_subsequent_native_off_requires_another_resume_while_model_recovers():
  r=Rig().run(1.)
  interrupt(r,'brake')
  r.run(.1, engaged=False, cruise_engaged=False, user_action='resume', manual=True)
  r.run(.2, valid=False, cruise_state_valid=True)
  r.run(.2, valid=False, engaged=False, cruise_engaged=False, cruise_state_valid=True)
  r.run(1., forecast=50, clear=False)
  assert not r.pulses and r.c.suspended


def test_stale_ford_display_during_suspension_never_retires_a_credit():
  r=Rig().run(.03,forecast=60,clear=False,ack=False)
  assert r.c.transport.down_debt == 1
  interrupt(r, 'brake')
  r.observed=69
  r.run(.1, clear=False, engaged=False, cruise_engaged=False, valid=False, cruise_state_valid=False)
  assert r.c.transport.down_debt == 1 and r.c.transport.observed == 70
  r.observed=70
  r.run(.1, clear=False, engaged=False, cruise_engaged=False, valid=False, cruise_state_valid=True)
  assert not r.c.fault and r.c.transport.down_debt == 1


@pytest.mark.parametrize('old_button', ['SET+', 'SET-'])
def test_unanswered_auto_move_during_manual_set_settling_cannot_replace_driver_intent(old_button):
  if old_button == 'SET+':
    r = Rig().run(2., forecast=69, clear=False).run(1., clear=True, ack=False)
    sign = 1
  else:
    r = Rig().run(.03, forecast=65, clear=False, ack=False)
    sign = -1
  assert r.c.transport.debt[sign] == 1
  mark = len(r.pulses)
  r.observed = 68
  r.run(.1, user_action='set', manual=True, clear=False, ack=False)
  r.run(.15, clear=False, ack=False)
  r.observed += sign
  r.run(.4, clear=False, ack=False)
  assert r.c.ceiling != r.observed
  r.run(.1, user_action='resume', manual=True, ack=False)
  r.run(1., forecast=55, clear=False, ack=False)
  assert len(r.pulses) == mark
  assert r.c.transport.debt[sign] == 1
  r.observed = 63
  r.run(.1, user_action='set', manual=True, ack=False)
  r.run(.6, clear=False, ack=False)
  assert r.observed == r.c.ceiling == 63 and len(r.pulses) == mark
  r.run(.6, forecast=52, clear=False)
  assert any(b=='SET-' for _,b in r.pulses[mark:])


@pytest.mark.parametrize('old_button', ['SET+', 'SET-'])
def test_opposite_move_during_a_held_physical_button_requires_a_separate_selection(old_button):
  if old_button == 'SET+':
    r=Rig().run(2.,forecast=69,clear=False).run(1.,clear=True,ack=False)
    delta=1
  else:
    r=Rig().run(.03,forecast=65,clear=False,ack=False)
    delta=-1
  assert r.c.transport.debt[delta] == 1
  r.observed=68
  r.run(.1,user_action='set',manual=True,user_set_direction=-delta,clear=False,ack=False)
  r.observed += delta
  r.run(.4,user_action='set',manual=True,user_set_direction=-delta,clear=False,ack=False)
  mark=len(r.pulses)
  r.run(.7,forecast=55,clear=False,ack=False)
  assert r.c.suspended and r.c.ceiling != r.observed
  assert not r.pulses[mark:] and r.c.transport.debt[delta] == 1
  r.observed=64
  r.run(.1,user_action='set',manual=True,user_set_direction=-1,clear=False,ack=False)
  r.run(.6,clear=False,ack=False)
  assert not r.c.suspended and r.c.ceiling == 64


def test_valid_same_direction_button_hold_can_change_selected_speed():
  r=Rig().run(1.)
  r.observed=68
  r.run(.1,user_action='set',manual=True,user_set_direction=-1,clear=False)
  r.observed=67
  r.run(.1,user_action='set',manual=True,user_set_direction=-1,clear=False)
  r.run(.6,clear=False)
  assert r.c.ceiling == 67 and not r.c.suspended


@pytest.mark.parametrize('direction', [-1, 1])
def test_debt_free_transient_opposite_read_can_settle_during_same_hold(direction):
  r=Rig().run(1.)
  r.observed=67
  r.run(.1, user_action='set', manual=True, user_set_direction=direction)
  r.observed -= 4 * direction
  r.run(.1, user_action='set', manual=True, user_set_direction=direction)
  r.observed = 67 + direction
  r.run(.15,user_action='set',manual=True,user_set_direction=direction)
  r.run(.6,clear=False,ack=False)
  assert not r.c.suspended and not r.c.selection_ambiguous
  assert r.c.ceiling == 67 + direction


@pytest.mark.parametrize('on_display,resumes', [(70,True),(69,False)])
def test_debt_free_unresolved_selection_requires_verified_previous_driver_ceiling(on_display,resumes):
  r=Rig().run(1.)
  r.observed=65
  r.run(.1,user_action='set',manual=True,user_set_direction=-1)
  r.observed=68
  r.run(.1,user_action='set',manual=True,user_set_direction=-1)
  r.run(.6,clear=False,ack=False)
  assert not r.pulses and r.c.ceiling == 70 and r.c.suspended
  r.run(.1,user_action='resume',manual=True)
  r.run(.3,clear=False)
  assert r.c.suspended
  interrupt(r,'brake')
  r.run(.1,user_action='resume',manual=True,cruise_engaged=False,engaged=False)
  r.observed=on_display
  r.run(.65,clear=False)
  assert not r.pulses and r.c.ceiling == 70
  r.run(.7,forecast=60,clear=False)
  assert bool(r.pulses) == resumes
  assert r.c.suspended != resumes


def test_confirmed_old_ceiling_cannot_flush_unresolved_auto_traffic():
  r=Rig().run(2.,forecast=69,clear=False).run(1.,clear=True,ack=False)
  r.observed=68
  r.run(.1,user_action='set',manual=True,user_set_direction=-1,ack=False)
  r.observed=69
  r.run(.1,user_action='set',manual=True,user_set_direction=-1,ack=False)
  r.run(.6,clear=False,ack=False)
  interrupt(r,'brake')
  r.run(.1,user_action='resume',manual=True,cruise_engaged=False,engaged=False)
  mark=len(r.pulses)
  r.observed=70
  r.run(1.,forecast=50,clear=False,ack=False)
  assert r.c.suspended and r.c.transport.up_debt == 1 and len(r.pulses) == mark
