import importlib
from pathlib import Path

import pytest


@pytest.fixture(scope='module')
def result(tmp_path_factory):
  assert importlib.util.find_spec('tools.navigator_a3.command_path_audit') is not None
  audit=importlib.import_module('tools.navigator_a3.command_path_audit').audit
  return audit(Path('/private/tmp/navigator-a3-opendbc'),tmp_path_factory.mktemp('command-path'))


def test_full_card_proposals_reach_only_simulated_transport(result):
  assert result['instrumentation_equivalent']
  for name, case in result['cases'].items():
    for row in case['calls']:
      if row['publication'] is not None:
        assert row['publication']['receiver_acceptance']=='unknown'
        assert row['publication']['provenance']=='synthetic'
      assert row['actual_can_equal_a2']
      assert row['serialized_selection_matches']
    if name.startswith('normal') or name in ('driver', 'measurement', 'gap'):
      assert all(r['accepted'] for r in case['aggregate'] if r['event']['tx'])
    assert all(not r['accepted'] for r in case['production'] if r['event']['tx'] and
               (((bytes.fromhex(r['event']['data_hex'])[3]&31)<<6 | bytes.fromhex(r['event']['data_hex'])[4]>>2)!=1000))


def test_faulted_cases_do_not_resume_on_returned_frames(result):
  for name in ('rejected','permission','configuration'):
    calls=result['cases'][name]['calls']
    assert any(r['fault'] for r in calls)
    assert all(r['fault'] for r in calls if r['step']>=17)
    assert not any(r['publication'] and r['diagnostic']['output']['mode']==1 for r in calls if r['step']>=17)


def test_no_catchup_no_duplicate_publication_and_bounded_handoff(result):
  for case in result['cases'].values():
    counters=[r['diagnostic']['scheduler']['proposal_counter'] for r in case['calls'] if r['publication']]
    assert all(b==(a+1)%16 for a,b in zip(counters,counters[1:]))
    assert any(r['diagnostic']['scheduler']['waiting'] and r['publication'] is None for r in case['calls'])
  for name in ('driver','measurement','gap'):
    calls=[r for r in result['cases'][name]['calls'] if r['publication']]
    for a,b in zip(calls,calls[1:]):
      if a['diagnostic']['output']['mode']==0 and b['diagnostic']['output']['mode']==1:
        assert b['now_ns']-a['now_ns']>=50_000_000


def test_compiled_rejection_does_not_feed_back_into_host(result):
  cases=result['cases']
  assert cases['wire-excessive']['calls']==cases['normal-p0-s1']['calls']
  assert any(not r['accepted'] for r in cases['wire-excessive']['aggregate'] if r['event']['tx'])
  assert cases['compressed']['calls']==cases['normal-p0-s1']['calls']
  assert result['physical_response']=='unknown'
