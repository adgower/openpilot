import pytest
from openpilot.selfdrive.car.tests.test_vision_curve_opening_lead import receding_exit, tick

from openpilot.selfdrive.car.tests.test_geometry_following_return import following


@pytest.mark.parametrize(
  'veto',
  [
    'none',
    'near_lead',
    'lead_closes',
    'second_lead',
    'steering',
    'brake',
    'physical_set',
    'lane_change',
    'cancel',
    'stale_radar',
    'stale_yaw',
    'tightens',
    'alpha_brakes',
    'stale_comp',
    'missing_comp',
    'high_comp',
    'lead_vanish',
  ],
)
def test_crossing_curve_clear_requires_continuous_permission(veto):
  f, c, floor = receding_exit()
  old = f.context.can_assert
  f.context.can_assert = lambda *_: False
  for i in range(110, 123):
    assert tick(f, i, ack=False) != 'SET+'
  assert c.opening_since is not None
  assert c.opening_step is None
  m = f.sm['modelV2']
  m.orientationRate.z = [0.0] * len(m.orientationRate.z)
  for i in range(123, 141):
    assert tick(f, i, ack=False) != 'SET+'
    if c.opening_step is not None:
      break
  assert c.opening_step == floor + 1
  f.context.can_assert = old
  if veto == 'lead_vanish':
    f.sm['radarState'].leadOne.present = False
  if veto == 'near_lead':
    f.sm['radarState'].leadOne.dRel = 20
  if veto == 'lead_closes':
    f.sm['radarState'].leadOne.vRel = -2.0
  if veto == 'second_lead':
    f.sm['radarState'].leadTwo = following()
    f.sm['radarState'].leadTwo.dRel = 18.0
  if veto == 'steering':
    f.state.steeringPressed = True
  if veto == 'brake':
    f.state.brakePressed = True
  if veto == 'physical_set':
    f.parser.vl['Steering_Data_FD1']['CcAslButtnSetIncPress'] = 1
  if veto == 'lane_change':
    f.state.rightBlinker = True
  if veto == 'cancel':
    f.control.cruiseControl.cancel = True
  if veto == 'stale_radar':
    f.sm.valid['radarState'] = False
  if veto == 'tightens':
    m.orientationRate.z = [0.35] * len(m.orientationRate.z)
  if veto == 'alpha_brakes':
    m.action.desiredAcceleration = -1.0
  actions = []
  for j in range(1, 5 if veto == 'lead_vanish' else 15):
    actions.append(
      tick(
        f,
        i + j / 5,
        ack=False,
        new_model=(j % 5 == 0 or veto in ('tightens', 'alpha_brakes', 'stale_comp', 'missing_comp', 'high_comp')),
        yaw_age=120_000_000 if veto == 'stale_yaw' else 0,
        comp_age=120_000_000 if veto == 'stale_comp' else 0,
        comp_dps=(None if veto == 'missing_comp' else 45.0 if veto == 'high_comp' else 0.6),
      )
    )
    if j == 1 and veto == 'lead_vanish':
      assert c.opening_since is None and c.opening_ack is None and c.opening_step is None
  if veto == 'none':
    assert actions.count('SET+') == 1 and c.transport.up_debt == 1
  else:
    assert 'SET+' not in actions and c.transport.up_debt == 0


def test_transition_step_expires_if_no_neutral_slot_is_available():
  f, c, floor = receding_exit()
  original = f.context.can_assert
  f.context.can_assert = lambda *_: False
  for i in range(110, 123):
    tick(f, i, ack=False)
  assert c.opening_since is not None
  model = f.sm['modelV2']
  model.orientationRate.z = [0.0] * len(model.orientationRate.z)
  earned = False
  for i in range(123, 150):
    assert tick(f, i, ack=False) != 'SET+'
    earned |= c.opening_step == floor + 1
  assert earned and c.geometry_hold is None and c.opening_step is None and c.opening_since is None
  f.context.can_assert = original
  assert all(tick(f, i, ack=False) != 'SET+' for i in range(150, 157))
  assert c.transport.up_debt == 0
