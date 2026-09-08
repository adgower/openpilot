"""Behavior tests: disabling evidence/driver/measurement gates must fail these."""
import copy
import importlib.util
from pathlib import Path
import unittest

MODULE = Path(__file__).with_name('counterfactual_replay.py')


def row(t=1_000_000_000):
  return {'route_id': 'test', 't_ns': t + 1, 'car_output_event_valid': True, 'diagnostic': {'fault_reason': 'direct_steering_rejection', 'controller': {
    'config': {'mode': 'shadow', 'profile': 'expedition-provisional-v1'},
    'input': {'now_ns': t, 'source_ns': t - 1_000_000, 'curvature_inv_m': .001, 'speed_mps': 20.,
              'active': True, 'driver_pressed': False, 'valid': False, 'measured_curvature_inv_m': 0.,
              'measurement_ns': t - 2_000_000, 'measurement_valid': True},
    'output': {'mode': 0, 'reason': 'invalid'}, 'evidence_fault_reason': 'direct_steering_rejection',
    'actual_frame': {'address': 982, 'data': '0000000000000000', 'bus': 0},
    'proposed_frame': {'address': 982, 'data': '0000000000000000', 'bus': 0}}},
    'counter': 0, 'source_evidence': {'exact_source_event': True, 'event_valid': True, 'values_match': True,
                                    'cadence_supported': True}}


class CounterfactualTests(unittest.TestCase):
  def replay(self, rows):
    self.assertTrue(MODULE.exists(), 'offline counterfactual replay is not implemented')
    spec = importlib.util.spec_from_file_location('counterfactual_replay', MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return list(module.replay_rows(rows))

  def test_fault_is_retained_but_never_used_as_synthetic_feedback_or_permission(self):
    rows = [row(), row(1_050_000_000)]
    before = copy.deepcopy(rows)
    out = self.replay(rows)
    self.assertEqual(rows, before)
    self.assertTrue(all(r['synthetic']['mode'] == 1 for r in out))
    self.assertTrue(all(r['historical']['fault_reason'] == 'direct_steering_rejection' for r in out))
    self.assertTrue(all(r['synthetic_transport_acceptance'] == 'unknown' and not r['physical_activation_allowed'] for r in out))
    self.assertTrue(all(not r['synthetic']['transmission_allowed'] for r in out))

  def test_driver_press_neutralizes_and_release_reenters_with_bounded_history(self):
    rows = [row(1_000_000_000 + n * 50_000_000) for n in range(3)]
    rows[1]['diagnostic']['controller']['input']['driver_pressed'] = True
    out = self.replay(rows)
    self.assertEqual(out[1]['synthetic']['reason'], 'driver_override')
    self.assertEqual(out[1]['synthetic']['path_angle_rad'], 0)
    self.assertEqual(out[2]['synthetic']['path_angle_rad'], out[0]['synthetic']['path_angle_rad'])

  def test_missing_matching_or_cadence_evidence_is_unsupported(self):
    for key in ('exact_source_event', 'values_match', 'cadence_supported'):
      r = row(); r['source_evidence'][key] = False
      out = self.replay([r])[0]
      self.assertIsNone(out['synthetic'])
      self.assertIn(key, out['unsupported_reason'])

  def test_invalid_event_and_stale_inputs_stay_inactive(self):
    r = row(); r['source_evidence']['event_valid'] = False
    self.assertEqual(self.replay([r])[0]['synthetic']['reason'], 'invalid')
    r = row(); r['diagnostic']['controller']['input']['source_ns'] -= 200_000_000
    self.assertEqual(self.replay([r])[0]['synthetic']['reason'], 'stale')

  def test_measurement_missing_invalid_stale_and_clipping_above_nine(self):
    for field, value, reason in [('measurement_ns', None, 'measurement_missing'),
                                 ('measurement_valid', False, 'measurement_invalid'),
                                 ('measurement_ns', 1, 'measurement_stale')]:
      r = row(); r['diagnostic']['controller']['input'][field] = value
      self.assertEqual(self.replay([r])[0]['synthetic']['reason'], reason)
    r = row(); r['diagnostic']['controller']['input']['curvature_inv_m'] = .01
    out = self.replay([r])[0]['synthetic']
    self.assertEqual(out['measurement_limited_curvature_inv_m'], .002)
    self.assertLess(out['equivalent_curvature_inv_m'], .002)

  def test_repeated_publication_does_not_reset_strategy_history(self):
    a = row(); b = row(1_050_000_000)
    baseline = self.replay([a, b])[-1]['synthetic']
    repeated = self.replay([a, copy.deepcopy(a), b])
    self.assertEqual(repeated[-1]['synthetic'], baseline)

  def test_native_extraction_matches_source_and_never_uses_future_control(self):
    self.replay([])
    spec = importlib.util.spec_from_file_location('counterfactual_replay', MODULE)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    r = row(); source = r['diagnostic']['controller']['input']['source_ns']
    events = [{'kind': 'carControl', 't_ns': source - n * 10_000_000, 'valid': True,
               'data': {'actuators': {'curvature': .001}, 'latActive': True}} for n in range(10, -1, -1)]
    events += [{'kind': 'carOutput', 't_ns': r['t_ns'], 'valid': True, 'data': r['diagnostic']}]
    extracted = list(module.extract_events(events, 'test'))[0]
    self.assertTrue(extracted['source_evidence']['cadence_supported'])
    self.assertEqual(self.replay([extracted])[0]['synthetic']['mode'], 1)
    events[10]['t_ns'] = source + 1
    extracted = list(module.extract_events(events, 'test'))[0]
    self.assertFalse(extracted['source_evidence']['exact_source_event'])

  def test_segment_overlap_preserves_native_order_and_every_event(self):
    self.replay([])
    spec = importlib.util.spec_from_file_location('counterfactual_replay', MODULE)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    self.assertTrue(hasattr(module, 'merge_segments'), 'native segment overlap merger missing')
    segments = [[{'t_ns': 1}, {'t_ns': 11}], [{'t_ns': 10}, {'t_ns': 20}], [{'t_ns': 19}, {'t_ns': 30}]]
    self.assertEqual([e['t_ns'] for e in module.merge_segments(segments)], [1, 10, 11, 19, 20, 30])

  def test_invalid_car_output_is_unsupported_and_resets_history(self):
    first = row(); invalid = row(1_050_000_000); last = row(1_100_000_000)
    invalid['car_output_event_valid'] = False
    results = self.replay([first, invalid, last])
    self.assertIsNone(results[1]['synthetic'])
    self.assertEqual(results[1]['unsupported_reason'], 'invalid_car_output_event')
    self.assertEqual(results[2]['synthetic']['path_angle_rad'], results[0]['synthetic']['path_angle_rad'])

  def test_unknown_original_invalidity_is_not_cleared(self):
    r = row(); r['diagnostic']['controller']['evidence_fault_reason'] = None
    self.assertEqual(self.replay([r])[0]['synthetic']['reason'], 'invalid')


if __name__ == '__main__':
  unittest.main()


def test_v2_calculation_fault_cannot_be_removed_by_legacy_rejection_inference():
  from tools.navigator_a3.counterfactual_replay import replay_rows
  r = row()
  r['diagnostic']['schema_version'] = 2
  c = r['diagnostic']['controller']
  c['schema_version'] = 2
  c['calculation_fault_reason'] = 'permission_unavailable'
  c['calculation_eligible'] = False
  result = list(replay_rows([r]))[0]
  assert result['synthetic']['mode'] == 0
  assert not result['synthetic_input']['valid']
