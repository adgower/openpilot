from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.vision_curve_context import VisionCurveContext
from openpilot.selfdrive.car.tests.test_vision_forecast_context import ContextFixture, CaptureCruise

MPH = .44704


def fixture():
  f = ContextFixture()
  f.context = VisionCurveContext(comfort=True)
  f.context.cruise = CaptureCruise(f.context.cruise)
  f.state.cruiseState.speedCluster = 70. * MPH
  f.state.vEgo = 70. * MPH
  f.sm['modelV2'].velocity.x = [70. * MPH] + [40. * MPH] * 5
  f.sm['modelV2'].laneLineProbs = [.1, .9, .9, .1]
  return f


def tick(f, i, *, ack=True, **kwargs):
  call = f.update(i * .05, **kwargs)
  action = f.context.cruise.last_action
  if ack and action in ('SET-', 'SET+'):
    f.state.cruiseState.speedCluster += (-1 if action == 'SET-' else 1) * MPH
  return call, action


@pytest.mark.parametrize(('accel', 'max_steps'), [(-.11, 1), (-.3, 2), (-1.5, 2)])
def test_single_anchored_early_allowance(accel, max_steps):
  f = fixture()
  f.sm['modelV2'].action.desiredAcceleration = accel
  actions = []
  for i in range(300):
    if i > 60:
      f.state.vEgo = max(33., 70. - .45 * (i - 60)) * MPH
      f.sm['modelV2'].velocity.x = [max(10., (f.state.vEgo / MPH) - 10.) * MPH] * 6
    call, action = tick(f, i)
    if i < 12:
      assert action != 'SET-'
    if i > 53:
      assert call['forecast_reduce_mph'] == 70
    if action:
      actions.append(action)
  assert actions == ['SET-'] * max_steps
  assert round(f.state.cruiseState.speedCluster / MPH) == 70 - max_steps


@pytest.mark.parametrize('demand', [0., .4])
def test_low_future_velocity_without_current_deceleration_has_no_authority(demand):
  f = fixture()
  f.sm['modelV2'].action.desiredAcceleration = demand
  assert all(tick(f, i)[1] not in ('SET-', 'SET+') for i in range(120))


def test_positive_request_blocks_new_decrements_then_releases_without_ceiling_forecast():
  f = fixture()
  actions = []
  for i in range(45):
    if i == 15:
      f.sm['modelV2'].action.desiredAcceleration = .2
    call, action = tick(f, i)
    if action:
      actions.append(action)
    if i >= 15:
      assert call['forecast_reduce_mph'] == 70
      assert action != 'SET-'
  assert actions == ['SET-', 'SET+']
  assert round(f.state.cruiseState.speedCluster / MPH) == 70


def test_lead_alone_and_already_slow_oem_do_not_lower_cruise():
  for mode in ('lead', 'lower_speed'):
    f = fixture()
    if mode == 'lead':
      f.sm['radarState'].leadOne = SimpleNamespace(present=True, dRel=50., vRel=-10.)
    else:
      f.state.vEgo = 60. * MPH
    assert all(tick(f, i)[1] not in ('SET-', 'SET+') for i in range(60))


def test_geometry_can_own_a_larger_drop_and_yield_on_exit():
  f = fixture()
  f.sm['modelV2'].velocity.x = [70. * MPH] * 6
  actions = []
  for i in range(160):
    model = f.sm['modelV2']
    model.orientationRate.z = [.12] * 6 if 6 <= i < 80 else [0.] * 6
    model.action.desiredAcceleration = .3
    _, action = tick(f, i)
    if action:
      actions.append(action)
    if i < 80:
      assert action != 'SET+'
  assert actions.count('SET-') > 2
  assert actions.count('SET-') == actions.count('SET+')
  assert round(f.state.cruiseState.speedCluster / MPH) == 70


def test_pedal_inhibits_new_requests():
  f = fixture()
  for i in range(30):
    f.state.gasPressed = i >= 11
    assert tick(f, i)[1] not in ('SET-', 'SET+')


def test_stale_or_stopped_model_inhibits_new_requests():
  for mode in ('stop', 'stale', 'short'):
    f = fixture()
    if mode == 'stop':
      f.sm['modelV2'].action.shouldStop = True
    elif mode == 'short':
      f.sm['modelV2'].velocity.t[-1] = 9.
    for i in range(80):
      _, action = tick(f, i, new_model=mode != 'stale' or i == 0)
      assert action not in ('SET-', 'SET+')


def test_new_session_at_driver_selected_value():
  f = fixture()
  for i in range(44):
    tick(f, i)
  assert round(f.state.cruiseState.speedCluster / MPH) == 68
  f.parser.vl['Steering_Data_FD1']['CcAslButtnSetIncPress'] = 1
  f.state.cruiseState.speedCluster = 73 * MPH
  tick(f, 44)
  f.parser.vl['Steering_Data_FD1'].clear()
  f.sm['modelV2'].action.desiredAcceleration = .4
  for i in range(45, 76):
    tick(f, i)
  assert f.context.cruise.ceiling == 73


@pytest.mark.parametrize('bad_input', ['misaligned_yaw_times', 'nan_yaw', 'empty_velocity', 'nan_time', 'nan_action'])
def test_invalid_preview_cannot_spend_or_qualify_bend_or_hint(bad_input):
  f = fixture()
  m = f.sm['modelV2']
  m.orientationRate.z = [.14] * 6
  if bad_input == 'misaligned_yaw_times':
    m.orientationRate.t[-2] = 7.2
  elif bad_input == 'nan_yaw':
    m.orientationRate.z[0] = float('nan')
  elif bad_input == 'empty_velocity':
    m.velocity.t.clear()
    m.velocity.x.clear()
  elif bad_input == 'nan_time':
    m.velocity.t[-1] = float('nan')
  elif bad_input == 'nan_action':
    m.action.desiredAcceleration = float('nan')
  for i in range(40):
    assert tick(f, i)[1] not in ('SET-', 'SET+')
  m.orientationRate.t = m.velocity.t = [0.,1.,3.,5.,7.,10.]
  m.orientationRate.z = [.14] * 6
  m.action.desiredAcceleration = -.3
  m.velocity.x = [40 * MPH] * 6
  for i in range(40,50):
    assert tick(f,i)[1] not in ('SET-', 'SET+')


def test_a_subframe_driver_override_cannot_count_toward_hint_dwell():
  f = fixture()
  for i in range(11):
    assert tick(f, i)[1] != 'SET-'
  f.state.steeringPressed = True
  tick(f, 11, new_model=False)
  f.state.steeringPressed = False
  for i in range(12, 23):
    assert tick(f, i)[1] != 'SET-'


def test_radar_fault_blocks_cached_command_until_new_model():
  f = fixture()
  for i in range(13):
    tick(f, i)
  radar = f.sm['radarState']
  radar.radarErrors.radarFault = True
  call, action = tick(f, 13, new_model=False)
  assert not call['valid'] and not call['forecast_clear']
  assert action not in ('SET-', 'SET+')


def test_radar_model_time_and_fault_are_independent_of_envelope():
  from openpilot.selfdrive.car.lead_speed import radar_is_aligned

  f = fixture()
  tick(f, 0)
  radar = f.sm['radarState']
  now = f.sm.logMonoTime['modelV2']
  assert radar_is_aligned(now, radar)
  for radar_time in (0, now + 1, now - 200_000_001):
    radar.mdMonoTime = radar_time
    assert not radar_is_aligned(now, radar)
  radar.mdMonoTime = now
  for flag in ('canError', 'radarFault', 'wrongConfig', 'radarUnavailableTemporary'):
    setattr(radar.radarErrors, flag, True)
    assert not radar_is_aligned(now, radar)
    setattr(radar.radarErrors, flag, False)


def test_new_radar_lead_blocks_cached_hint_presses_between_model_frames():
  f = fixture()
  for i in range(16):
    tick(f, i)
  f.sm['radarState'].leadTwo.present = True
  for i in range(16, 38):
    fresh = i >= 18
    _, action = tick(f, i, new_model=fresh)
    assert action not in ('SET-', 'SET+')
