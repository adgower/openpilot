import math
import pytest
from openpilot.selfdrive.car.vision_curve_context import opening_load

NOW = 1_000_000_000


def assess(raw, ford, gyro, std=0.12, requested=1.0, **changes):
  speed = 20.0
  motion = {'valid': True, 'sample_ns': NOW - 40_000_000, 'received_ns': NOW - 10_000_000, 'yaw_rate': gyro / speed, 'yaw_std': std / speed}
  motion.update(changes)
  return opening_load(NOW, speed, requested / speed**2, raw, ford, motion)


@pytest.mark.parametrize('sign', (-1.0, 1.0))
def test_exit_requires_independent_signed_measurements(sign):
  load, confirmed = assess(sign * 1.58, sign * 1.24, -sign * 1.19)
  assert confirmed and load == pytest.approx(1.31)
  load, confirmed = assess(sign * 1.18, sign * 1.54, -sign * 1.53)
  assert confirmed and load > 1.5
  load, confirmed = assess(sign * 1.58, sign * 1.24, sign * 1.19)
  assert not confirmed and load >= 1.58


@pytest.mark.parametrize(
  'changes', [{'valid': False}, {'sample_ns': 0}, {'received_ns': NOW + 1}, {'sample_ns': NOW - 100_000_001}, {'yaw_std': math.nan}, {'yaw_std': -0.01}]
)
def test_bad_motion_cannot_overrule_raw_yaw(changes):
  load, confirmed = assess(1.58, 1.24, -1.19, **changes)
  assert not confirmed and load >= 1.58


def test_demand_and_measurement_uncertainty_gate_next_mph():
  assert assess(1.58, 1.24, -1.19, requested=-1.62)[0] >= 1.62
  assert assess(1.58, 1.4, -1.38, std=0.2)[0] == pytest.approx(1.58)
