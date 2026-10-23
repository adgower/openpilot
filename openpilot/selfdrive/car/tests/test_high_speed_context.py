from itertools import product
from types import SimpleNamespace as NS
import pytest
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, FordFlags
from openpilot.selfdrive.car import tracking_context as tc


def params():
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.flags |= int(FordFlags.PINION_CURVATURE | FordFlags.LOW_SPEED_GAIN)
  return cp


@pytest.mark.parametrize('tracking,roll,high', list(product([False, True], repeat=3)))
def test_high_speed_factory_requires_all_existing_prerequisites(tracking, roll, high):
  c = tc.create_tracking_trial(params(), enabled=tracking, roll_mapping_enabled=roll, high_speed_enabled=high)
  expected = (
    'ObserverContinuity' if tracking and roll and high else 'RollAwareMapping' if tracking and roll else 'BudgetedTrackingPrototype' if tracking else None
  )
  assert (type(c).__name__ if c is not None else None) == expected


def test_missing_or_old_registry_preserves_disabled_preference():
  from openpilot.common.params import UnknownKeyName

  class OldRegistry:
    def get_bool(self, key):
      raise UnknownKeyName(key)

  assert not tc.high_speed_opt_in(OldRegistry())
  assert not tc.high_speed_opt_in(NS(get_bool=lambda key: False))
  assert tc.high_speed_opt_in(NS(get_bool=lambda key: True))


@pytest.mark.parametrize('flag', [FordFlags.PINION_CURVATURE, FordFlags.LOW_SPEED_GAIN])
def test_no_trial_without_vehicle_flags(flag):
  cp = params()
  cp.flags &= ~int(flag)
  assert tc.create_tracking_trial(cp, enabled=True, roll_mapping_enabled=True, high_speed_enabled=True) is None


def test_real_compiled_registry_defaults_off_and_preserves_other_settings(tmp_path):
  from openpilot.common.params import Params

  p = Params(str(tmp_path))
  p.put_bool('FordPrefLowSpeedGain', True, block=True)
  assert not tc.high_speed_opt_in(p)
  p.put_bool('FordPrefHighSpeedTracking', True, block=True)
  assert tc.high_speed_opt_in(p) and p.get_bool('FordPrefLowSpeedGain')
  p.put_bool('FordPrefHighSpeedTracking', False, block=True)
  assert not tc.high_speed_opt_in(p) and p.get_bool('FordPrefLowSpeedGain')
