import csv
from pathlib import Path
from tools.navigator_a3.core_replay import replay


def test_asof_validity_and_common_input(tmp_path: Path):
  path = tmp_path / 'signals.csv'
  rows = []
  for t in (1_000_000_000, 1_060_000_000, 1_300_000_000):
    rows.append(('carControl.actuators.curvature', t, .001, 'True'))
  for source, value in [('carState.vEgoRaw', 20), ('carControl.latActive', 1), ('carState.steeringPressed', 0),
                        ('carState.vehicleSensorsInvalid', 0), ('modelV2.action.desiredCurvature', -.02)]:
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
