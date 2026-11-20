from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.tests.test_geometry_following_return import following, setup_geometry, tick
from openpilot.selfdrive.car.vision_curve_recovery import AcknowledgedCurveSpeed
from openpilot.selfdrive.car.vision_curve_speed import MPH


def opening_pulse():
  f, m, c = setup_geometry()
  f.state.yawRate = .01
  for i in range(110):
    tick(f, i)
  floor = round(f.state.cruiseState.speedCluster / MPH)
  f.state.vEgo = (floor - .5) * MPH
  m.velocity.x = [f.state.vEgo] * 6
  m.orientationRate.z = [f.state.vEgo * 1.5 / ((floor + 5) * MPH) ** 2] * 6
  for i in range(110, 165):
    if tick(f, i, ack=False) == 'SET+':
      assert c.transport.up_debt == 1
      return f, m, c, i, floor
  pytest.fail('no opening pulse')


@pytest.mark.parametrize('delivery', [4, 12, 40, None])
def test_bend_opening_press_lasts_until_receipt_or_timeout(delivery):
  f, m, c, start, floor = opening_pulse()
  first_clear = None
  for n in range(1, 75):
    if n == delivery:
      f.state.cruiseState.speedCluster = (floor + 1) * MPH
    button = tick(f, start + n / 5, ack=False, new_model=n % 5 == 0)
    assert button != 'SET+'
    if button == '' and first_clear is None:
      first_clear = n
    assert round(f.state.cruiseState.speedCluster / MPH) + c.transport.up_debt <= floor + 1
  expected_release_tick = delivery if delivery is not None and delivery < 13 else 13
  assert first_clear == pytest.approx(expected_release_tick, abs=1 if expected_release_tick == 13 else 0)
  assert c.transport.up_debt == int(delivery is None)


@pytest.mark.parametrize('block', ['brake', 'direction', 'closing', 'yaw', 'speed', 'lead', 'stop', 'model_invalid'])
def test_unacknowledged_bend_step_releases_on_new_inhibition(block):
  f, m, c, start, floor = opening_pulse()
  if block == 'brake':
    f.state.brakePressed = True
  if block == 'direction':
    f.state.leftBlinker = True
  if block == 'closing':
    m.orientationRate.z[1] = -.19
  if block == 'yaw':
    f.state.yawRate = .3
  if block == 'speed':
    f.state.vEgo = (floor + 8) * MPH
  if block == 'lead':
    f.sm['radarState'].leadOne = following()
  if block == 'stop':
    m.action.shouldStop = True
  if block == 'model_invalid':
    f.sm.valid['modelV2'] = False
  assert tick(f, start + .2, ack=False, new_model=block in ('closing', 'stop')) == ''
  assert c.transport.up_debt == 1


def test_receipt_uses_the_headroom_for_the_step_ford_accepted():
  f, m, c, start, floor = opening_pulse()
  # This bend supports the issued step, while a second step would not fit.
  c.geometry_target = floor + 3.25
  f.state.cruiseState.speedCluster = (floor + 1) * MPH
  assert tick(f, start + .2, ack=False, new_model=False) != 'SET+'
  assert c.geometry_hold == floor + 1
  assert c.transport.up_debt == 0


@pytest.fixture
def opening_curve():
  recovery = AcknowledgedCurveSpeed()
  recovery._start(70)
  recovery.geometry_hold = 44.
  model = SimpleNamespace(
    action=SimpleNamespace(shouldStop=False, desiredAcceleration=.3),
    orientationRate=SimpleNamespace(t=[0., 4., 10.], z=[.01, .01, .01]),
    velocity=SimpleNamespace(x=[20.] * 3),
  )
  return recovery, model


def test_shallow_curvature_variation_preserves_an_earned_opening(opening_curve):
  recovery, model = opening_curve
  recovery.geometry_target = 47.95
  assert recovery._opening_step(0., 45., 45., .8, model) is None
  assert recovery.opening_since is None
  recovery.geometry_target = 48.2
  assert recovery._opening_step(.05, 45., 45., .8, model) is None
  recovery.geometry_target = 47.95
  assert recovery._opening_step(.9, 45., 45., .8, model) is None
  assert recovery._opening_step(1.06, 45., 45., .8, model) == 46
  recovery.geometry_target = 46.95
  assert recovery._opening_step(1.11, 45., 45., .8, model) is None
  assert recovery.opening_since is None


@pytest.mark.parametrize('hazard', ['opposite_bend', 'measured_load'])
def test_new_load_resets_opening_time(opening_curve, hazard):
  recovery, model = opening_curve
  recovery.geometry_target = 49.
  assert recovery._opening_step(0., 45., 45., .8, model) is None
  recovery.geometry_target = 47.95
  lateral_accel = .8
  if hazard == 'opposite_bend':
    model.orientationRate.z[1] = -.2
  else:
    lateral_accel = 1.55
  assert recovery._opening_step(1.1, 45., 45., lateral_accel, model) is None
  assert recovery.opening_since is None


def test_opening_margin_applies_only_to_current_press(opening_curve):
  recovery, model = opening_curve
  recovery.model_ok = True
  recovery.transport.observed = recovery.observed = 45.
  recovery.geometry_target = 48.2
  assert recovery._opening_step(0., 45., 45., .8, model) is None
  recovery.geometry_target = 47.5
  recovery.target = recovery.opening_step = recovery._opening_step(1.01, 45., 45., .8, model)
  assert recovery.opening_step == 46
  assert recovery._send(1.01, 45., True) == 'SET+'
  assert recovery.opening_since is None
  assert recovery.update(1.02, engaged=True, valid=True, manual=False, override=False, set_speed=45.,
                         speed=45., accel=.1, model=None, cruise_engaged=True, forecast_authoritative=True,
                         forecast_available=True, lateral_accel=.8) is None
  assert recovery.transport.pressed
  assert recovery._opening_step(1.06, 45., 45., .8, model) == 46
  recovery._send(1.16, 45., False)
  assert not recovery.transport.pressed
  assert recovery._opening_step(1.17, 45., 45., .8, model) is None
  assert recovery.opening_since is None


@pytest.mark.parametrize('lost_evidence', ['model_brakes', 'opposite_bend', 'yaw_after_release'])
@pytest.mark.parametrize('same_frame', [False, True])
def test_invalidation_revokes_floor_credit_but_still_accounts_for_late_receipt(lost_evidence, same_frame):
  f, m, c, start, floor = opening_pulse()
  historic_floor = c.geometry_hold
  n = 1
  if lost_evidence == 'yaw_after_release':
    for n in range(1, 18):
      tick(f, start + n / 5, ack=False, new_model=n % 5 == 0)
    assert not c.transport.pressed
    f.state.yawRate = .3
  elif lost_evidence == 'opposite_bend':
    m.orientationRate.z[1] = -.16
  else:
    m.action.desiredAcceleration = -.06
  if same_frame:
    f.state.cruiseState.speedCluster = (floor + 1) * MPH
  tick(f, start + (n + 1) / 5, ack=False, new_model=lost_evidence != 'yaw_after_release')
  assert c.opening_ack is None
  if not same_frame:
    assert c.transport.up_debt == 1
    f.state.cruiseState.speedCluster = (floor + 1) * MPH
    tick(f, start + (n + 2) / 5, ack=False, new_model=False)
  assert c.transport.observed == floor + 1 and c.transport.up_debt == 0
  assert c.geometry_hold <= historic_floor


def test_new_model_and_receipt_share_a_tick_without_rating_the_next_step():
  f, m, c, start, floor = opening_pulse()
  f.state.cruiseState.speedCluster = (floor + 1) * MPH
  tick(f, start + 1, ack=False, new_model=True)
  assert c.transport.observed == floor + 1 and c.transport.up_debt == 0
  assert c.geometry_hold == floor + 1
