"""Radar-backed first step during geometry-owned recovery."""

import pytest

from openpilot.selfdrive.car.tests.test_geometry_following_return import following, setup_geometry, tick as geometry_tick
from openpilot.selfdrive.car.vision_curve_speed import MPH


def tick(f, i, *, comp_dps=0.6, comp_age=0, **kwargs):
  if comp_dps is None:
    f.parser.vl.pop('BrakeSnData_3', None)
    f.parser.ts_nanos.pop('BrakeSnData_3', None)
  else:
    f.parser.vl['BrakeSnData_3'] = {'VehYawComp_W_Actl': comp_dps}
    f.parser.ts_nanos['BrakeSnData_3'] = {'VehYawComp_W_Actl': round((10 + i * 0.05) * 1e9) - comp_age}
  return geometry_tick(f, i, **kwargs)


def receding_exit():
  f, model, controller = setup_geometry()
  f.state.yawRate = 0.01
  for i in range(110):
    tick(f, i)
  floor = round(f.state.cruiseState.speedCluster / MPH)
  f.state.vEgo = (floor - 0.5) * MPH
  model.velocity.x = [f.state.vEgo] * 6
  model.orientationRate.z = [f.state.vEgo * 1.5 / ((floor + 5) * MPH) ** 2] * 6
  lead = following()
  lead.vRel = 3.0
  f.sm['radarState'].leadOne = lead
  return f, controller, floor


@pytest.mark.parametrize('delivery', [None, 4, 24])
def test_receding_lead_permits_single_earned_step_and_preserves_late_credit(delivery):
  f, c, floor = receding_exit()
  start = None
  for i in range(110, 175):
    if tick(f, i, ack=False) == 'SET+':
      start = i
      break
  assert start is not None and start >= 129
  assert c.transport.up_debt == 1
  for j in range(1, 90):
    if delivery == j:
      f.state.cruiseState.speedCluster = (floor + 1) * MPH
    assert tick(f, start + j / 5, ack=False, new_model=j % 5 == 0) != 'SET+'
  assert c.transport.up_debt == (1 if delivery is None else 0)
  assert c.transport.observed == (floor if delivery is None else floor + 1)
  assert not c.transport.fault


@pytest.mark.parametrize('block', ['closing', 'near', 'second', 'poor_quality', 'manual', 'cancel', 'brake', 'stale_yaw', 'stale_radar', 'unreceived_minus'])
def test_receding_lead_grant_ends_before_next_switch_slot(block):
  f, c, floor = receding_exit()
  original = f.context.can_assert
  f.context.can_assert = lambda *_: False
  for i in range(110, 170):
    tick(f, i, ack=False)
    if c.opening_step is not None:
      break
  assert c.opening_step == floor + 1
  f.context.can_assert = original
  if block == 'closing':
    f.sm['radarState'].leadOne.vRel = -3.0
  elif block == 'near':
    f.sm['radarState'].leadOne.dRel = 40.0
  elif block == 'second':
    f.sm['radarState'].leadTwo = following()
  elif block == 'poor_quality':
    f.sm['radarState'].leadOne.modelProb = 0.25
  elif block == 'manual':
    f.state.steeringPressed = True
  elif block == 'brake':
    f.state.brakePressed = True
  elif block == 'cancel':
    f.control.cruiseControl.cancel = True
  elif block == 'stale_radar':
    f.sm.valid['radarState'] = False
  elif block == 'unreceived_minus':
    c.transport.debt[-1] = 1
  assert tick(f, i + 0.2, new_model=False, ack=False, yaw_age=120_000_000 if block == 'stale_yaw' else 0) != 'SET+'
  assert c.opening_step is None and c.transport.up_debt == 0


@pytest.mark.parametrize(('value', 'age'), [
  (None, 0), (float('nan'), 0), (float('inf'), 0), (-76., 0), (4094 * 0.03663 - 75., 0),
  (4095 * 0.03663 - 75., 0), (4.5, 0), (-4.5, 0), (0.6, 100_000_001), (0.6, -1),
])
def test_compensated_yaw_can_veto_only_the_new_opening_step(value, age):
  f, c, floor = receding_exit()
  original = f.context.can_assert
  f.context.can_assert = lambda *_: False
  for i in range(110, 170):
    tick(f, i, ack=False)
    if c.opening_step is not None:
      break
  assert c.opening_step == floor + 1
  f.context.can_assert = original
  assert tick(f, i + 0.2, ack=False, new_model=False, comp_dps=value, comp_age=age) != 'SET+'
  assert c.opening_step is None and c.transport.up_debt == 0
