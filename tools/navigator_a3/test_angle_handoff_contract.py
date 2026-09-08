"""Executable contract cases fail if admission, history or provenance is misreported."""
import importlib.util
from pathlib import Path

import pytest

MODULE = Path(__file__).with_name('angle_handoff_contract.py')


@pytest.fixture(scope='module')
def report(tmp_path_factory):
  assert MODULE.exists(), 'Missing compiled Angle handoff contract implementation'
  spec = importlib.util.spec_from_file_location('angle_handoff_contract', MODULE)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module.audit(Path('/private/tmp/navigator-a3-opendbc'), tmp_path_factory.mktemp('handoff'))


def txs(report, case, variant='candidate'):
  return [r for r in report['cases'][case][variant] if r['event']['tx']]


def test_valid_normal_both_signs_profiles_and_production_block(report):
  for profile in range(4):
    for sign in (-1, 1):
      name = f'normal-p{profile}-s{sign}'
      assert all(r['accepted'] for r in txs(report, name))
      assert all(not r['accepted'] for r in txs(report, name, 'production'))
      assert all(r['failed_checks'] for r in txs(report, name, 'production'))


def test_jitter_is_not_silently_accepted_or_fixed(report):
  rows = txs(report, 'jitter-49-51ms')
  assert [r['accepted'] for r in rows] == [True, False, False, False, False]
  assert rows[2]['reason_names'] == ['A3_PATH_RATE']
  assert rows[1]['before']['compiled_candidate']['angle_last'] == rows[1]['encoded']['previous_host_proposal_angle_signed_counts']
  assert rows[2]['before']['compiled_candidate']['angle_last'] != rows[2]['encoded']['previous_host_proposal_angle_signed_counts']
  small = txs(report, 'jitter-small-demand')
  assert [r['accepted'] for r in small] == [True, False, True, False, True]
  assert rows[1]['reason_names'] == ['A3_CADENCE']
  assert abs(rows[1]['encoded']['delta_from_candidate_last_rad']) <= .009 + 1e-9
  assert abs(rows[2]['encoded']['delta_from_candidate_last_rad']) > .009
  assert abs(rows[2]['encoded']['delta_from_previous_host_proposal_rad']) <= .009 + 1e-9
  assert rows[1]['before']['compiled_candidate']['angle_last'] == rows[1]['after']['compiled_candidate']['angle_last']
  assert rows[1]['proposal']['output']['reason'] != 'cadence'


def test_driver_handoff_and_inactive_contract(report):
  rows = txs(report, 'driver-handoff')
  assert all(r['accepted'] for r in rows)
  assert rows[2]['proposal']['output']['mode'] == 0
  assert rows[3]['proposal']['output']['mode'] == 1
  invalid = txs(report, 'inactive-nonzero')[0]
  assert not invalid['accepted'] and 'A3_INACTIVE' in invalid['reason_names']


def test_permission_rejection_changes_stock_history_not_candidate_history(report):
  rows = txs(report, 'permission-loss-neutral-return')
  failure = rows[2]
  assert not failure['accepted'] and 'A3_STOCK_ENVELOPE' in failure['reason_names']
  assert failure['before']['compiled_stock_desired_last'] != failure['after']['compiled_stock_desired_last']
  assert failure['before']['compiled_candidate']['angle_last'] == failure['after']['compiled_candidate']['angle_last']
  assert rows[3]['accepted']  # valid neutral; permission remains off
  assert rows[4]['accepted']  # explicit synthetic cruise RX re-engagement
  assert failure['failed_checks']


def test_a2_neutral_and_raw_excessive_contract(report):
  assert all(r['accepted'] for r in txs(report, 'a2-neutral', 'production'))
  assert not txs(report, 'excessive-then-neutral')[1]['accepted']
  assert txs(report, 'excessive-then-neutral')[2]['accepted']


def test_sequences_are_identical_without_oracle_feedback_and_checks_are_attributed(report):
  assert report['instrumentation_equivalent']
  assert report['production_matches_current_source']
  for case in report['cases'].values():
    reference = case['candidate']
    for variant in ('stock-reference', 'production'):
      assert [r['event'] for r in reference] == [r['event'] for r in case[variant]]
      assert [r['proposal'] for r in reference] == [r['proposal'] for r in case[variant]]
    for row in reference:
      assert row['event'].get('controls_fixture') is None
      assert row['physical_eps_response'] == 'unknown'
      if row['reason_names']:
        assert row['candidate_failed_checks']
  assert report['physical_mapping_validated'] is False
