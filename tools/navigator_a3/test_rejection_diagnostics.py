"""Offline fault controls prove unmodified proposal history and continuous evaluator state."""
from pathlib import Path
import pytest

from tools.navigator_a3.rejection_diagnostics import paired_audit, classify_src


@pytest.fixture(scope='module')
def audit(tmp_path_factory):
  return paired_audit(Path('/Users/alex/Apps/bluepilot'), Path('/private/tmp/navigator-a3-opendbc'),
                      tmp_path_factory.mktemp('rejection-audit'))


def test_positive_baseline(audit):
  for pair in audit['cases'].values():
    assert all(r['accepted'] for r in pair['control'])
    assert all(r['proposal']['output']['mode'] == 1 for r in pair['control'] if r['proposal'])


@pytest.mark.parametrize(('fault', 'bit'), [('stale_rx', 2), ('corrupt_rx', 2), ('controls_off', 32), ('excessive_tx', 512)])
def test_one_injection_and_specific_rejection(audit, fault, bit):
  rows = audit['cases'][fault]['injected']
  assert sum(r['injected'] for r in rows) == 1
  assert any(r['accepted'] is False and (r['reason_bits'] or 0) & bit for r in rows if r['event']['tx'])
  if fault in ('corrupt_rx', 'controls_off'):
    assert audit['cases'][fault]['unmodified_suffix_rejections']


@pytest.mark.parametrize('fault', ['stale_rx', 'corrupt_rx', 'controls_off', 'excessive_tx'])
def test_no_oracle_or_hidden_resets(audit, fault):
  pair = audit['cases'][fault]
  assert [r['proposal'] for r in pair['control']] == [r['proposal'] for r in pair['injected']]
  for previous, current in zip(pair['injected'], pair['injected'][1:], strict=False):
    a, b = dict(previous['after']), dict(current['before'])
    a.pop('time_us')
    b.pop('time_us')
    assert a == b
  injection = next(r for r in pair['injected'] if r['injected'])
  assert injection['before']['compiled_candidate']['tx_seen'] == 1
  assert injection['after']['compiled_candidate']['tx_seen'] == 1
  assert injection['before']['controller_proposal_state']['last_ns'] is not None


def test_rejection_holds_candidate_history_without_reset(audit):
  row = next(r for r in audit['cases']['excessive_tx']['injected'] if r['injected'])
  for key in ('angle_last', 'tx_time_us', 'tx_seen'):
    assert row['before']['compiled_candidate'][key] == row['after']['compiled_candidate'][key]
  assert row['after']['controller_proposal_state']['last_ns'] > row['before']['controller_proposal_state']['last_ns']


def test_historical_tags_never_acknowledge_counterfactual():
  assert classify_src(0)['kind'] == 'rx'
  assert classify_src(0x80)['kind'] == 'returned'
  assert classify_src(0xC0)['kind'] == 'rejected'
  assert classify_src(0x140)['kind'] == 'unknown'
  assert all(not classify_src(i)['acknowledges_synthetic_a3'] for i in (0, 0x80, 0xC0, 0x140))
