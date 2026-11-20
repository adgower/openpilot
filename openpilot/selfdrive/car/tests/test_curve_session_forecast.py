import pytest

from openpilot.selfdrive.car.tests.test_vision_forecast_context import ContextFixture
from openpilot.selfdrive.car.vision_curve_speed import MPH


@pytest.mark.parametrize('interruption', ['brake', 'cancel', 'off', 'set'])
def test_fresh_session_does_not_republish_prior_alpha_slowdown(interruption):
  f = ContextFixture()
  assert f.qualify()['forecast_mph'] == pytest.approx(63)
  if interruption == 'brake':
    f.state.brakePressed = True
  elif interruption == 'cancel':
    f.control.cruiseControl.cancel = True
  elif interruption == 'off':
    f.state.cruiseState.enabled = False
  else:
    f.parser.vl['Steering_Data_FD1']['CcAslButtnSetDecPress'] = 1
  f.update(.76, new_model=False)
  f.state.brakePressed = f.control.cruiseControl.cancel = False
  f.state.cruiseState.enabled = True
  f.parser.vl['Steering_Data_FD1'].clear()
  # Flat, neutral model below the ceiling would retain a previously owned hint.
  f.sm['modelV2'].velocity.x = [64 * MPH] * 6
  f.sm['modelV2'].action.desiredAcceleration = 0.
  value = f.update(.80)
  assert value['forecast_mph'] is None


@pytest.mark.parametrize('interruption', ['brake','cancel','set'])
def test_curve_anticipation_requires_fresh_deceleration_after_intervention(interruption):
  f = ContextFixture()
  for i in range(15):
    sample = f.update(i * .05)
  assert sample['comfort_hint_s'] > 0.
  f.state.brakePressed = interruption == 'brake'
  f.control.cruiseControl.cancel = interruption == 'cancel'
  if interruption == 'set':
    f.parser.vl['Steering_Data_FD1']['CcAslButtnSetDecPress'] = 1
  for i in range(1, 8):
    f.update(.70 + i * .05)
  f.state.brakePressed = f.control.cruiseControl.cancel = False
  f.parser.vl['Steering_Data_FD1'].clear()
  assert f.update(1.10)['comfort_hint_s'] == 0.
  assert f.update(1.15)['comfort_hint_s'] == 0.
  for i in range(1,11):
    sample = f.update(1.15 + i * .05)
  assert sample['comfort_hint_s'] > 0.


def test_oem_engagement_feedback_survives_model_outage_only_with_valid_ford_can():
  f = ContextFixture()
  f.sm.valid['modelV2'] = False
  first = f.update(.1)
  assert not first['valid']
  assert first['cruise_state_valid']
  f.state.canValid = False
  assert not f.update(.2)['cruise_state_valid']


def test_explicit_cancel_has_priority_over_a_set_in_the_same_sample():
  f = ContextFixture()
  f.parser.vl['Steering_Data_FD1']['CcAslButtnSetIncPress'] = 1
  f.parser.vl['Steering_Data_FD1']['CcAslButtnCnclPress'] = 1
  assert f.update(.1)['user_action'] == 'cancel'


@pytest.mark.parametrize('switch', ['CcAslButtnSetIncPress', 'CcAsllButtnResPress', 'CcAslButtnCnclResPress'])
def test_stale_pressed_switch_cannot_authorize_an_engagement(switch):
  f = ContextFixture()
  f.state.cruiseState.enabled = False
  f.update(0.)
  f.parser.vl['Steering_Data_FD1'][switch] = 1
  now = 14_000_000_000
  f.sm.logMonoTime['carControl'] = now
  f.parser.ts_nanos['EngBrakeData']['Veh_V_DsplyCcSet'] = now
  f.context.update(now, f.sm, f.parser, f.state, f.control, False)
  assert f.context.cruise.calls[-1][1]['user_action'] is None
  assert f.context.cruise.calls[-1][1]['manual']
  # A held switch needs a witnessed release before it can supply a fresh tap.
  assert f.update(4.1)['user_action'] is None
  f.parser.vl['Steering_Data_FD1'].clear()
  assert f.update(4.2)['user_action'] is None
  f.parser.vl['Steering_Data_FD1'][switch] = 1
  expected = 'set' if switch == 'CcAslButtnSetIncPress' else 'resume'
  assert f.update(4.3)['user_action'] == expected


def test_ambiguous_shared_switch_with_invalid_native_cruise_cannot_authorize_resume():
  f = ContextFixture()
  f.state.cruiseState.enabled = False
  f.update(0.)
  f.state.canValid = False
  f.parser.vl['Steering_Data_FD1']['CcAslButtnCnclResPress'] = 1
  assert f.update(.1)['user_action'] is None
  f.state.canValid = True
  assert f.update(.2)['user_action'] is None


@pytest.mark.parametrize(('keys', 'direction'), [
  (['CcAslButtnSetDecPress'], -1), (['CcAslButtnSetIncPress'], 1), (['CcAslButtnSetPress'], 0),
  (['CcAslButtnSetDecPress', 'CcAslButtnSetIncPress'], 0),
])
def test_ford_reports_manual_set_direction_for_selection_reconciliation(keys, direction):
  f = ContextFixture()
  f.parser.vl['Steering_Data_FD1'].update(dict.fromkeys(keys, 1))
  values = f.update(.1)
  assert values['user_action'] == 'set'
  assert values['user_set_direction'] == direction
