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


def tick(f, i, *, ack=True, new_model=True, yaw_quality=3, yaw_age=0):
  if hasattr(f.state, 'yawRate'):
    f.parser.vl['Yaw_Data_FD1'] = {'VehYaw_W_Actl': f.state.yawRate, 'VehYawWActl_D_Qf': yaw_quality}
    f.parser.ts_nanos['Yaw_Data_FD1'] = {'VehYaw_W_Actl': round((10 + i * .05) * 1e9) - yaw_age}
  f.update(i * .05, new_model=new_model)
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


@pytest.mark.parametrize('block', [None, 'speed', 'lateral_load', 'next_bend', 'lead', 'brake', 'steering', 'radar_gap', 'model_stop'])
def test_opening_bend_earns_one_step_after_settling(block):
  f, m, c = setup_geometry()
  f.state.yawRate = .01
  for i in range(110):
    tick(f, i)
  floor = round(f.state.cruiseState.speedCluster / MPH)
  assert floor < 65
  f.state.vEgo = (floor - .5) * MPH
  m.velocity.x = [f.state.vEgo] * 6
  m.orientationRate.z = [f.state.vEgo * 1.5 / ((floor + 5) * MPH) ** 2] * 6
  if block == 'speed':
    f.state.vEgo = (floor + 12) * MPH
  elif block == 'lateral_load':
    f.state.yawRate = .15
  elif block == 'next_bend':
    m.orientationRate.z[1] = -.16
  elif block == 'lead':
    f.sm['radarState'].leadTwo = following()
  elif block == 'brake':
    f.state.brakePressed = True
  elif block == 'steering':
    f.state.steeringPressed = True
  elif block == 'model_stop':
    m.action.shouldStop = True
  steps = []
  for i in range(110, 190):
    if block == 'radar_gap':
      f.sm.valid['radarState'] = i % 12 != 0
    action = tick(f, i)
    if action == 'SET+':
      steps.append(i)
  if block:
    assert not steps
  else:
    assert steps and steps[0] >= 129
    assert all(b - a >= 19 for a, b in zip(steps, steps[1:]))
    assert floor < round(f.state.cruiseState.speedCluster / MPH) <= floor + 4
    m.orientationRate.z = [.19] * 6
    assert all(tick(f, i) != 'SET+' for i in range(190, 215))


def test_deferred_bend_allows_one_step_but_near_bend_withholds_it():
  _, m, c = setup_geometry()
  c._start(70)
  c.geometry_hold, c.geometry_target = 43., 50.
  velocity = 43. * MPH
  m.velocity.x = [velocity] * 6
  m.orientationRate.z = [.055] * 6
  m.orientationRate.z[-1] = -.09
  assert c._opening_step(0., 43., 43., 1., m) is None
  assert c._opening_step(1., 43., 43., 1., m) == 44
  m.orientationRate.z[1] = -.09
  assert c._opening_step(1.05, 43., 43., 1., m) is None


def test_only_acknowledgement_can_raise_working_curve_floor():
  c = AcknowledgedCurveSpeed()
  c._start(70)
  c.transport.observed = c.observed = 46
  c.geometry_hold, c.geometry_target = 43., 50.
  c.opening_step = c.target = 47
  assert c._send(0., 46, True) == 'SET+'
  c._send(.2, 46, False)
  assert c.geometry_hold == 43 and c.opening_ack == 47
  c._send(.25, 47, False)
  assert c.geometry_hold == 47 and c.opening_ack is None


@pytest.mark.parametrize('interruption', ['yaw', 'speed', 'quality', 'stale', 'future'])
def test_motion_change_between_model_and_button_revokes_opening(interruption):
  f, model, c = setup_geometry()
  f.state.yawRate = .01
  for i in range(110):
    tick(f, i)
  floor = round(f.state.cruiseState.speedCluster / MPH)
  f.state.vEgo = (floor - .5) * MPH
  model.velocity.x = [f.state.vEgo] * 6
  model.orientationRate.z = [f.state.vEgo * 1.5 / ((floor + 5) * MPH) ** 2] * 6
  f.context.can_assert = lambda *_: False
  for i in range(110, 150):
    tick(f, i)
    if c.opening_step is not None:
      break
  assert c.opening_step == floor + 1
  if interruption == 'yaw':
    f.state.yawRate = .2
  elif interruption == 'speed':
    f.state.vEgo = (floor + 10) * MPH
  age = {'stale': 101_000_000, 'future': -1}.get(interruption, 0)
  f.context.can_assert = lambda *_: True
  assert tick(f, i + .2, new_model=False, yaw_quality=1 if interruption == 'quality' else 3, yaw_age=age) != 'SET+'
  assert c.opening_step is None and c.opening_since is None
  f.state.vEgo, f.state.yawRate = floor * MPH, .01
  assert all(tick(f, fresh) != 'SET+' for fresh in range(i+1, i+20))
