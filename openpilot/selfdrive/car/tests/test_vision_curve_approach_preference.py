import math
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.vision_curve_speed import COMFORT_LATERAL_ACCEL, Curve, MPH, comfort_approach_mph


@pytest.mark.parametrize(("base_mph", "expected_accel"), [(30., 1.65), (40., 1.65), (47.5, 1.575), (55., 1.5), (70., 1.5)])
@pytest.mark.parametrize("direction", [-1, 1])
def test_tighter_bend_preference_only_changes_the_approach(base_mph, expected_accel, direction):
  curvature = COMFORT_LATERAL_ACCEL / (base_mph * MPH) ** 2
  speed = base_mph * MPH
  curve = Curve(curvature, direction, .6, .99)
  model = SimpleNamespace(orientationRate=SimpleNamespace(t=[.6, 1.], z=[direction * curvature * speed] * 2),
                          velocity=SimpleNamespace(x=[speed] * 2))
  target = comfort_approach_mph(model, curve, base_mph, 0.)
  assert target == pytest.approx(math.sqrt(expected_accel / curvature) / MPH)
  assert COMFORT_LATERAL_ACCEL == 1.5
