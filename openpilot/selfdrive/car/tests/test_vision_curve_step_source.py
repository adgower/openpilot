import pytest
from openpilot.selfdrive.car.vision_curve_recovery import AcknowledgedCurveSpeed


@pytest.mark.parametrize(('forecast','geo','age','prior_mixed','expected'), [
  (69, 50, 1, False, True), # Ford reads 65, Alpha wants 69, geometry qualifies.
  (65, 50, 1, False, True),
  (64.99, 50, 1, False, False), # Strictly lower forecast may have commanded a decrement.
  (68, 62.01, 1, False, False), # Old plus issued credits could cross fresh geometry.
  (69, 50, .1, False, False), # New geometry has not yet persisted.
  (69, 50, 1, True, False), # Prior mixed-source credit cannot be relabelled.
])
def test_strict_attribution(forecast,geo,age,prior_mixed,expected):
  c=AcknowledgedCurveSpeed()
  c._start(70)
  c.transport.observed=c.observed=65
  c.transport.debt[-1]=2
  c.carried_decrements=True
  c.geometry_hold=c.target=50.
  c.geometry_target=geo
  c.model_ok=True
  c.detect_since=10.-age
  c.all_decrements_geometry=not prior_mixed
  c.forecast_target=forecast
  expected_withheld = forecast >= 65 and (age < .6 or geo > 62)
  result = c._send(10.,65,True)
  if expected_withheld:
    assert result != 'SET-'
    assert c.all_decrements_geometry is (not prior_mixed)
    assert c.transport.debt[-1]==2
  else:
    assert result == 'SET-'
    assert c.all_decrements_geometry is expected
    assert c.transport.debt[-1]==3
  assert c.transport.debt[1]==0


def test_unknown_old_credit_prevents_following_return_even_after_new_proven_geometry():
  c=AcknowledgedCurveSpeed()
  c._start(70)
  c.transport.observed=c.observed=65
  c.transport.debt[-1]=2
  c.carried_decrements=True
  c.geometry_hold=c.target=c.geometry_target=50.
  c.model_ok=True
  c.detect_since=2.
  c.forecast_target=69.
  c.all_decrements_geometry=False
  assert c._send(5.,65,True)=='SET-'
  c._send(5.06,64,False)
  c.geometry_clear_since=c.following_clear_since=5.
  c.forecast_owned=False
  c.forecast_target=c.geometry_hold=None
  c.geometry_trimmed=True
  assert not c._can_return_while_following(10.,True)


def test_fresh_geometry_can_admit_after_temporary_loss_without_blind_retry():
  c=AcknowledgedCurveSpeed()
  c._start(70)
  c.transport.observed=c.observed=65
  c.transport.debt[-1]=2
  c.carried_decrements=True
  c.geometry_hold=c.target=c.geometry_target=50.
  c.forecast_target=69.
  c.all_decrements_geometry=True
  c.model_ok=True
  c.detect_since=None
  assert c._send(2.,65,True)!='SET-'
  assert c._send(2.10,64,True)!='SET-'
  assert c.transport.down_debt==1 and c.transport.observed==64
  c.detect_since=2.2
  assert c._send(2.22,64,True)!='SET-'
  assert c._send(2.82,64,True)=='SET-'
  assert c.transport.down_debt==2 and c.all_decrements_geometry
  assert c._send(2.83,64,False)==''
  assert c.transport.down_debt==2


def test_carry_requires_a_confirmed_full_restore_with_old_downward_credit():
  c=AcknowledgedCurveSpeed()
  c._start(70)
  c.transport.observed=c.observed=69
  c.transport.debt[-1]=2
  c.transport.debt[1]=1
  assert not c.carried_decrements
  assert c._send(5.,69,False)==''
  assert not c.carried_decrements
  c._send(5.05,70,False)
  assert c.carried_decrements
  c._send(5.1,69,False)
  assert c.carried_decrements
  c._send(5.15,68,False)
  assert not c.carried_decrements


def test_fresh_current_curve_without_cross_bend_debt_keeps_existing_behavior():
  c=AcknowledgedCurveSpeed()
  c._start(70)
  c.transport.observed=c.observed=65
  c.geometry_hold=c.target=c.geometry_target=50.
  c.forecast_target=69.
  c.model_ok=True
  c.detect_since=None
  assert not c.carried_decrements
  assert c._send(2.,65,True)=='SET-'
  assert not c.all_decrements_geometry
