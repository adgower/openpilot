from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.tests.test_vision_forecast_context import CaptureCruise, ContextFixture
from openpilot.selfdrive.car.vision_curve_recovery import AcknowledgedCurveSpeed
from openpilot.selfdrive.car.vision_curve_speed import MPH


def following():
  return SimpleNamespace(present=True, modelProb=.95, dRel=95., vRel=-.1)


def setup_geometry():
  f = ContextFixture()
  c = AcknowledgedCurveSpeed()
  f.context.cruise = CaptureCruise(c)
  f.state.cruiseState.speedCluster = f.state.vEgo = 70 * MPH
  model = f.sm['modelV2']
  model.velocity.x = [70 * MPH] * 6
  model.action.desiredAcceleration = .15
  model.orientationRate.z = [.12] * 6
  model.laneLineProbs = [.1, .9, .9, .1]
  return f, model, c


def tick(f, i, *, ack=True):
  f.update(i * .05)
  button = f.context.cruise.last_action
  if ack and button in ('SET+', 'SET-'):
    f.state.cruiseState.speedCluster += MPH if button == 'SET+' else -MPH
  return button


@pytest.mark.parametrize('inhibit', [None, 'close', 'uncertain', 'second_lead', 'radar_fault', 'stop', 'closing_forecast', 'steering', 'brake', 'set'])
def test_only_clear_healthy_following_restores_acknowledged_geometry(inhibit):
  f, m, c = setup_geometry()
  for i in range(35):
    tick(f, i)
  assert c.geometry_trimmed
  assert f.state.cruiseState.speedCluster < 69 * MPH
  m.orientationRate.z = [0.] * 6
  radar = f.sm['radarState']
  radar.leadOne = following()
  if inhibit == 'close':
    radar.leadOne.vRel = -10.
  elif inhibit == 'uncertain':
    radar.leadOne.modelProb = .2
  elif inhibit == 'second_lead':
    radar.leadTwo = following()
    radar.leadTwo.dRel = 10.
  elif inhibit == 'radar_fault':
    radar.radarErrors.radarFault = True
  elif inhibit == 'stop':
    m.action.shouldStop = True
  elif inhibit == 'closing_forecast':
    m.velocity.x = [60 * MPH] * 6
    m.action.desiredAcceleration = -.3
  elif inhibit == 'steering':
    f.state.steeringPressed = True
  elif inhibit == 'brake':
    f.state.brakePressed = True
  elif inhibit == 'set':
    f.parser.vl['Steering_Data_FD1']['CcAslButtnSetIncPress'] = 1
  actions = []
  for i in range(35,180):
    if inhibit == 'set' and i == 43:
      f.parser.vl['Steering_Data_FD1'].clear()
    b = tick(f, i)
    if i < 85:
      assert b != 'SET+'
    if b:
      actions.append(b)
  if inhibit:
    assert 'SET+' not in actions
  else:
    assert 'SET+' in actions
    assert round(f.state.cruiseState.speedCluster / MPH) == 70


def test_brief_clear_between_two_bends_does_not_restore_while_following():
  f, m, c = setup_geometry()
  for i in range(35):
    tick(f, i)
  assert c.geometry_trimmed
  f.sm['radarState'].leadOne = following()
  for i in range(35,100):
    m.orientationRate.z = [0.] * 6 if i < 74 else [-.12] * 6
    assert tick(f, i) != 'SET+'


def test_delayed_or_mixed_credit_never_turns_into_geometry_by_arrival_time():
  c = AcknowledgedCurveSpeed()
  c._start(70)
  c.target = c.forecast_target = 64
  assert c._send(0.,70,True) == 'SET-'
  c.geometry_hold = 54
  c.target, c.forecast_target = 54, None
  assert c._send(.07,70,True) == 'SET-'
  assert c.transport.down_debt == 2
  c._send(.08,69,True,can_assert=False)
  assert not c.geometry_trimmed
  c._send(.09,68,True,can_assert=False)
  assert not c.geometry_trimmed and not c.transport.down_debt
  c._send(.14,68,True,can_assert=False)
  assert c._send(.21,68,True) == 'SET-'
  c._send(.22,67,True,can_assert=False)
  assert c.geometry_trimmed


def test_unacknowledged_geometry_or_inherited_manual_session_cannot_grant_return():
  c = AcknowledgedCurveSpeed()
  c._start(70)
  c.target = c.geometry_hold = 50
  assert c._send(0.,70,True) == 'SET-'
  assert not c.geometry_trimmed
  kw=dict(engaged=True, manual=True, override=False, set_speed=70, speed=69, accel=0., model=None, cruise_engaged=True, valid=True)
  c.update(.1, user_action='set', **kw)
  c.update(.55, user_action=None, **dict(kw, manual=False))
  assert c.transport.down_debt == 1
  c.target = c.geometry_hold = 50
  c._send(.6,69,True,can_assert=False)
  assert not c.geometry_trimmed
