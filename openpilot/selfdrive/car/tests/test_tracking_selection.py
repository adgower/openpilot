import pytest
from openpilot.selfdrive.car.tracking_context import create_tracking_trial
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, FordFlags
from opendbc.car.ford.lateral_angle_tracking_budget import BudgetedTrackingPrototype


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('pinion,low_speed', [(False, False), (True, False), (False, True), (True, True)])
def test_existing_opt_in_selects_budgeted_trial_only_with_required_flags(enabled, pinion, low_speed):
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.flags &= ~int(FordFlags.PINION_CURVATURE | FordFlags.LOW_SPEED_GAIN)
  cp.flags |= int(FordFlags.PINION_CURVATURE) if pinion else 0
  cp.flags |= int(FordFlags.LOW_SPEED_GAIN) if low_speed else 0
  trial = create_tracking_trial(cp, enabled=enabled)
  if enabled and pinion and low_speed:
    assert isinstance(trial, BudgetedTrackingPrototype)
    assert trial.tracking_enabled and trial.very_low_speed_enabled
  else:
    assert trial is None


def test_other_platform_not_selected():
  cp = CarInterface.get_non_essential_params(CAR.FORD_F_150_MK14)
  cp.flags |= int(FordFlags.PINION_CURVATURE | FordFlags.LOW_SPEED_GAIN)
  assert create_tracking_trial(cp, enabled=True) is None
