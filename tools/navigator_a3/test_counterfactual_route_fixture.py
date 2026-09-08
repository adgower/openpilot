"""Real route regression: post-fault driver override must remain neutral."""
import copy
import json
from pathlib import Path

from tools.navigator_a3.counterfactual_replay import extract_events, replay_rows


FIXTURE = Path(__file__).with_name('fixtures') / 'counterfactual_driver_release_route.json'


def test_recorded_postfault_driver_release_keeps_fault_and_limits():
  assert FIXTURE.exists(), 'actual route driver press/release fixture missing'
  fixture = json.loads(FIXTURE.read_text())
  original = copy.deepcopy(fixture)
  rows = list(extract_events(fixture['events'], fixture['route_id']))
  assert len(rows) == 2
  result = list(replay_rows(rows))
  assert fixture == original
  assert all(r['source_evidence']['exact_source_event'] and r['source_evidence']['cadence_supported'] for r in rows)
  assert all(r['diagnostic']['controller']['input']['speed_mps'] > 9 for r in rows)
  assert all(r['diagnostic']['controller']['input']['measurement_valid'] for r in rows)
  assert result[0]['synthetic']['reason'] == 'driver_override'
  assert result[0]['synthetic']['path_angle_rad'] == 0
  release = result[1]['synthetic']
  assert release['mode'] == 1
  assert 0 < release['path_angle_rad'] <= .0425
  assert abs(release['measurement_limited_curvature_inv_m'] - release['measured_curvature_inv_m']) <= .002
  assert all(r['historical']['fault_reason'] == 'direct_steering_rejection' for r in result)
  assert all(r['historical']['controller']['output']['reason'] == 'invalid' for r in result)
  assert all(r['synthetic_transport_acceptance'] == 'unknown' and not r['physical_activation_allowed'] for r in result)
  assert all(not r['synthetic']['transmission_allowed'] for r in result)
