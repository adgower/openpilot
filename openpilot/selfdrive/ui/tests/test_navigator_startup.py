"""Real Params-to-Ford-constructor integration with fingerprint I/O stubbed out."""
import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from openpilot.common.params import Params
from opendbc.car import car_helpers, gen_empty_fingerprint
from opendbc.car.ford.lateral_angle import LateralAngle
from opendbc.car.ford.lateral_angle_very_low_speed_prototype import VeryLowSpeedGainPrototype
from opendbc.car.ford.lateral_angle_tracking_budget import BudgetedTrackingPrototype
from opendbc.car.ford.values import CAR, FordFlags


@pytest.mark.parametrize('persistent', [False, True])
@pytest.mark.parametrize('pinion,trial', [(False, False), (False, True), (True, False), (True, True)])
def test_saved_options_select_paired_controller_on_startup(tmp_path, pinion, trial, persistent):
  params = Params(str(tmp_path))
  assert not params.get_bool('FordPrefSteerAngleCurvature')
  assert not params.get_bool('FordPrefLowSpeedGain')
  params.put_bool('FordPrefSteerAngleCurvature', pinion, block=True)
  params.put_bool('FordPrefLowSpeedGain', trial, block=True)
  params.put_bool("FordPrefPersistentTracking", persistent, block=True)
  source = Path(__file__).parents[2] / 'car/card.py'
  tree = ast.parse(source.read_text())
  assignment = next(node for node in ast.walk(tree) if isinstance(node, ast.Assign)
                    and isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Name)
                    and node.value.func.id == 'get_car')
  obj = SimpleNamespace(params=params, can_callbacks=(None, None))
  env = {'self': obj, 'get_car': car_helpers.get_car, 'obd_callback': lambda _: None,
         'alpha_long_allowed': False, 'is_release': False, 'cached_params': None}
  fingerprint = (CAR.FORD_EXPEDITION_MK4, gen_empty_fingerprint(), 'TESTVIN', [], 'fixed', True)
  with patch.object(car_helpers, 'fingerprint', return_value=fingerprint):
    exec(compile(ast.Module(body=[assignment], type_ignores=[]), str(source), 'exec'), env)
  assert bool(obj.CI.CP.flags & FordFlags.PINION_CURVATURE) == pinion
  assert obj.CI.CP.safetyConfigs[-1].safetyParam == (10 if pinion else 2)
  assert type(obj.CI.CC.lateral_angle) is (VeryLowSpeedGainPrototype if pinion and trial else LateralAngle)
  assert obj.CI.CC.lateral_angle.get_curvature_factor(0., .01) == pytest.approx(1.56 * (1.05 * 1.10 if pinion and trial else 1.))

  selector = next(node for node in ast.walk(tree) if isinstance(node, ast.If)
                  and any(isinstance(child, ast.ImportFrom) and child.module == 'openpilot.selfdrive.car.tracking_context'
                          for child in node.body))
  obj.CP = obj.CI.CP
  obj.tracking_trial = None
  exec(compile(ast.Module(body=[selector], type_ignores=[]), str(source), 'exec'), env)
  if persistent and pinion and trial:
    assert isinstance(obj.CI.CC.lateral_angle, BudgetedTrackingPrototype)
    assert obj.CI.CC.lateral_angle is obj.tracking_trial
  else:
    assert obj.tracking_trial is None
    assert type(obj.CI.CC.lateral_angle) is (VeryLowSpeedGainPrototype if pinion and trial else LateralAngle)
  assert params.get_bool('FordPrefPersistentTracking') == persistent
