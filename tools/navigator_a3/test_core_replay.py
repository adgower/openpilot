import csv
from pathlib import Path
from tools.navigator_a3.core_replay import replay


def test_asof_validity_and_common_input(tmp_path: Path):
  path = tmp_path / 'signals.csv'
  rows = []
  for t in (1_000_000_000, 1_060_000_000, 1_300_000_000):
    rows.append(('carControl.actuators.curvature', t, .001, 'True'))
  for source, value in [('carState.vEgoRaw', 20), ('carControl.latActive', 1), ('carState.steeringPressed', 0),
                        ('carState.vehicleSensorsInvalid', 0), ('carState.yawRate', 0), ('modelV2.action.desiredCurvature', -.02)]:
    rows.append((source, 999_000_000, value, 'True'))
  with path.open('w') as f:
    w = csv.writer(f)
    w.writerow(('source', 't_ns', 'value', 'valid'))
    w.writerows(rows)
  result = replay(path)
  assert len(result) == 12
  assert all(r['input_curvature_inv_m'] == .001 for r in result)
  assert all(r['path_angle_rad'] > 0 for r in result[:8])  # model prediction cannot reverse prescribed input
  assert all(r['mode'] == 0 and not r['source_valid'] for r in result[8:])
  assert all(not r['transmission_allowed'] for r in result)


def test_missing_measurement_does_not_become_zero(tmp_path: Path):
  path = tmp_path / 'signals.csv'
  with path.open('w') as f:
    w = csv.writer(f)
    w.writerow(('source', 't_ns', 'value', 'valid'))
    for s, v in [('carControl.actuators.curvature', .01), ('carState.vEgoRaw', 20), ('carControl.latActive', 1),
                 ('carState.steeringPressed', 0), ('carState.vehicleSensorsInvalid', 0)]:
      w.writerow((s, 1_000_000_000, v, 'True'))
  assert all(row['mode'] == 0 and row['reason'] == 'measurement_missing' for row in replay(path))


def test_measurement_uses_native_raw_speed_and_yaw(tmp_path: Path):
  path = tmp_path / 'signals.csv'
  with path.open('w') as f:
    w = csv.writer(f)
    w.writerow(('source', 't_ns', 'value', 'valid'))
    for s, v in [('carControl.actuators.curvature', .001), ('carState.vEgoRaw', 20), ('carControl.latActive', 1),
                 ('carState.steeringPressed', 0), ('carState.vehicleSensorsInvalid', 0), ('carState.yawRate', .08),
                 ('derived.yawCurvature', .008)]:
      w.writerow((s, 1_000_000_000, v, 'True'))
  for row in replay(path):
    assert row['measured_curvature_inv_m'] == -.004
    assert row['measurement_limited_curvature_inv_m'] == -.002
    assert row['path_angle_rad'] < 0
