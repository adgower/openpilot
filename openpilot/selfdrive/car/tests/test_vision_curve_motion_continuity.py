import pytest
from types import SimpleNamespace

from openpilot.selfdrive.car.vision_curve_recovery import AcknowledgedCurveSpeed


def step(cruise, now, *, generation, raw, can_assert=True, observed=44):
  """Advance one 100 Hz callback between model frames."""
  return cruise.update(
    now,
    engaged=True,
    valid=True,
    manual=False,
    override=False,
    cruise_engaged=True,
    set_speed=observed,
    speed=44.0,
    accel=0.2,
    model=None,
    can_assert=can_assert,
    forecast_authoritative=True,
    forecast_available=True,
    lateral_accel=1.2,
    opening_motion_generation=generation,
    opening_raw_lateral_accel=raw,
  )


@pytest.fixture
def matured():
  c = AcknowledgedCurveSpeed()
  c._start(70)
  c.transport.observed = c.observed = 44
  c.geometry_hold, c.geometry_target = 43.0, 52.0
  c.geometry_trimmed = c.model_ok = True
  c.motion_generation = 7
  c.motion_since = 0.4
  c.opening_since = 0.5
  c.target = c.opening_step = 45
  return c


@pytest.mark.parametrize('new_generation', [None, 8])
def test_changed_source_revokes_a_dwell_earned_with_additional_authority(matured, new_generation):
  c = matured
  assert step(c, 1.50, generation=7, raw=1.7, can_assert=False) is None
  assert c.opening_step == 45
  assert step(c, 1.51, generation=new_generation, raw=0.7) != 'SET+'
  assert c.opening_since is c.opening_step is c.opening_ack is None
  assert not c.transport.pending


def test_receipt_after_loss_counts_debt_without_ratifying_held_floor(matured):
  c = matured
  assert step(c, 1.50, generation=7, raw=1.7) == 'SET+'
  assert c.transport.up_debt == 1
  assert step(c, 1.51, generation=None, raw=0.7) == ''
  assert c.opening_ack is None and c.transport.up_debt == 1
  assert step(c, 1.52, generation=None, raw=0.7, observed=45) != 'SET+'
  assert c.transport.up_debt == 0 and c.geometry_hold == 43.0


def test_dwell_earned_by_existing_evidence_survives_a_gyro_drop(matured):
  c = matured
  assert step(c, 1.50, generation=7, raw=0.7, can_assert=False) is None
  assert step(c, 1.51, generation=None, raw=0.7) == 'SET+'


@pytest.mark.parametrize(
  'interrupt',
  [
    None,
    'opposite bend',
    'tighter bend',
    'low lane confidence',
    'driver steering',
    'closing lead',
    'high lateral load',
    'model brakes',
    'gyro loss before reacquisition',
    'gyro loss on reacquisition',
    'gyro loss after reacquisition',
    'gyro generation change',
  ],
)
def test_near_ceiling_reentry_preserves_only_qualified_opening_evidence(monkeypatch, interrupt):
  import openpilot.selfdrive.car.vision_curve_recovery as logic
  from openpilot.selfdrive.car.tests.test_vision_forecast_context import ContextFixture

  c = logic.AcknowledgedCurveSpeed()
  c._start(70)
  c.transport.observed = c.observed = 48
  c.geometry_hold, c.geometry_target = 56.0, 65.0
  c.geometry_trimmed = c.model_ok = c.opening_had_lead = True
  c.curve_direction = 1
  c.detect_since = 0.0
  model = ContextFixture().sm['modelV2']
  model.laneLineProbs = [0.1, 0.9, 0.9, 0.1]
  model.action.desiredAcceleration = 0.3
  model.orientationRate.z = [0.003] * 6
  curve = SimpleNamespace(curvature=0.0012, direction=1)
  target = 65.0
  monkeypatch.setattr(logic, 'comfort_approach_mph', lambda *_: target)
  monkeypatch.setattr(c.branch, 'update', lambda *_: False)
  emitted = []
  targets = [65.0] * 8 + [69.0] * 2 + [67.8] * 4 + [69.0] * 14
  for i, target in enumerate(targets):
    t = 1.0 + i * 0.05
    # 65 qualifies. 69 then clears demand; 67.8 re-enters before the
    # ordinary 600 ms admission interval. All other evidence remains fresh.
    block = i == 11
    curve.direction = -1 if block and interrupt == 'opposite bend' else 1
    if block and interrupt == 'tighter bend':
      target = 66.0
    model.laneLineProbs[1] = 0.2 if block and interrupt == 'low lane confidence' else 0.9
    model.action.desiredAcceleration = -0.1 if block and interrupt == 'model brakes' else 0.3
    gen = 7
    if (
      i == 5
      and interrupt == 'gyro loss before reacquisition'
      or i == 10
      and interrupt == 'gyro loss on reacquisition'
      or i == 15
      and interrupt == 'gyro loss after reacquisition'
    ):
      gen = None
    if i >= 15 and interrupt == 'gyro generation change':
      gen = 8
    b = c.update(
      t,
      engaged=True,
      valid=True,
      manual=False,
      override=False,
      cruise_engaged=True,
      set_speed=48,
      speed=50,
      accel=0.2,
      model=model,
      forecast_authoritative=True,
      forecast_available=True,
      preview=(curve, True),
      early_hint_inhibited=True,
      following_clear=True,
      steering_only=block and interrupt == 'driver steering',
      opening_lead_clear=not (block and interrupt == 'closing lead'),
      lateral_accel=1.8 if block and interrupt == 'high lateral load' else 0.8,
      opening_lead_lateral_accel=0.8,
      opening_motion_generation=gen,
      opening_raw_lateral_accel=0.8,
      forecast_clear=False,
    )
    if b == 'SET+':
      emitted.append(t)
  if interrupt:
    assert emitted == []
  else:
    assert len(emitted) == 1 and emitted[0] >= 2.0
