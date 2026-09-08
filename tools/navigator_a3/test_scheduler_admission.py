"""Scheduler timing is exercised against unchanged compiled admission checks."""
from pathlib import Path

import pytest


@pytest.fixture(scope='module')
def report(tmp_path_factory):
  from tools.navigator_a3.scheduler_admission import audit
  return audit(Path('/private/tmp/navigator-a3-opendbc'), tmp_path_factory.mktemp('scheduler-admission'))


def txs(report, name, variant='candidate'):
  return [r for r in report['cases'][name][variant] if r['event']['tx']]


def test_jitter_waits_without_neutral_or_counter_advance(report):
  for p in range(4):
    for s in (-1, 1):
      name = f'jitter-p{p}-s{s}'
      calls = report['scheduler_calls'][name]
      waiting = [r for r in calls if r['time_us'] == 149000]
      assert len(waiting) == 1 and not waiting[0]['emitted']
      assert waiting[0]['before'] == waiting[0]['after']
      emitted = [r for r in calls if r['emitted']]
      assert all(b['time_us'] - a['time_us'] >= 50000 for a, b in zip(emitted, emitted[1:]))
      assert [r['counter'] for r in emitted] == list(range(len(emitted)))
      assert all(r['accepted'] for r in txs(report, name))
      assert all(not r['accepted'] for r in txs(report, name, 'production'))


def test_transport_compression_is_not_assumed_safe(report):
  rows = txs(report, 'compressed-arrival')
  assert rows[0]['accepted']
  assert not rows[1]['accepted'] and 'A3_CADENCE' in rows[1]['reason_names']
  assert all(r['physical_eps_response'] == 'unknown' for r in rows)
  assert report['scheduler_calls']['compressed-arrival'] == report['scheduler_calls']['jitter-p0-s1']


def test_independent_rejection_does_not_change_host_schedule(report):
  rows = txs(report, 'excessive-injection')
  assert not rows[1]['accepted']
  assert report['scheduler_calls']['excessive-injection'] == report['scheduler_calls']['jitter-p0-s1']
  assert rows[1]['proposal']['wire_mutation']
  assert report['no_oracle_feedback'] and report['instrumentation_equivalent']


def test_recorded_times_are_not_claimed_mcu_times(report):
  assert not report['physical_mapping_validated']
  assert report['timing_basis'] == 'synthetic host and separately assumed MCU arrival times'
  for case in report['cases'].values():
    assert [r['event'] for r in case['candidate']] == [r['event'] for r in case['production']]
    assert all(r['event'].get('controls_fixture') is None for r in case['candidate'])


def test_inactive_transitions_and_gap_reentry_stay_accepted(report):
  for name in ('driver-handoff', 'timing-gap'):
    rows = txs(report, name)
    assert all(r['accepted'] for r in rows)
    neutral = [r for r in rows if r['proposal']['output']['mode'] == 0]
    assert len(neutral) == 1
    index = rows.index(neutral[0])
    assert rows[index+1]['event']['time_us'] - neutral[0]['event']['time_us'] >= 50000
    assert rows[index+1]['proposal']['output']['mode'] == 1
  assert all(r['accepted'] for r in txs(report, 'original-call-times'))
  assert [r['event']['time_us'] for r in txs(report, 'original-call-times')] == [100000, 200000, 300000]
