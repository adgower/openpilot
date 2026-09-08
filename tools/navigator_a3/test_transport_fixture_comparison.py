import json

import pytest

from tools.navigator_a3.transport_fixture_comparison import compare_fixtures, generate_transport
from tools.navigator_a3.transport_observer import ORACLE_FIELDS, Observer


def fixture_row(accepted, tx=True, time_us=10):
  return {'event': {'tx': tx, 'time_us': time_us, 'address': 0x3D6, 'bus': 0,
                    'data_hex': '1000000000000000', 'controls_fixture': False},
          'accepted': accepted, 'reason_bits': 123, 'before': {'compiled_secret': True}}


@pytest.mark.parametrize('fault', ['stale_rx', 'corrupt_rx', 'controls_off', 'excessive_tx'])
def test_fault_control_and_missing_feedback(tmp_path, fault):
  source = tmp_path / 'fixtures.json'
  source.write_text(json.dumps({'cases': {fault: {
    'control': [fixture_row(True)],
    'injected': [fixture_row(False, tx=False), fixture_row(True), fixture_row(False, time_us=20)]}}}))
  result = compare_fixtures(source, tmp_path / 'out')
  rows = {(r['variant'], r['feedback']): r for r in result['comparisons']}
  control = rows['control', 'complete']
  assert control['observable_rejection_count'] == control['oracle_rejection_count'] == 0
  complete = rows['injected', 'complete']
  assert complete['oracle_rejection_count'] == complete['observable_rejection_count'] == 1
  assert complete['oracle_first_rejected_source_event_index'] == complete['observable_first_rejected_source_event_index'] == 2
  assert complete['count_matches'] and complete['first_rejection_matches']
  missing = rows['injected', 'no_rejected_echoes']
  assert missing['observable_rejection_count'] == 0
  assert missing['observable_first_rejected_source_event_index'] is None
  assert missing['missing_feedback_interpretation'] == 'unknown'
  assert not missing['count_matches']
  assert (tmp_path / 'out' / 'summary.json').exists()
  source_events = [json.loads(line) for line in (tmp_path / 'out' / 'source-events.jsonl').read_text().splitlines()]
  replay = Observer()
  for event in source_events:
    assert not ORACLE_FIELDS.intersection(event)
    assert replay.consume(event)['receiver_acceptance'] == 'unknown'


def test_observer_input_is_allowlisted_and_synthetic():
  events = generate_transport([fixture_row(False)], 'fault', 'injected', 'complete', 'fixture.json')
  assert [e['kind'] for e in events] == ['published', 'rejected']
  observer = Observer()
  for event in events:
    assert not ORACLE_FIELDS.intersection(event)
    assert 'before' not in event and 'controls_fixture' not in event
    assert event['provenance'] == 'synthetic'
    assert event['route_id'] == 'synthetic/fault/injected/complete'
    assert observer.consume(event)['receiver_acceptance'] == 'unknown'
  assert events[1]['t_ns'] - events[0]['t_ns'] == 1
  assert events[1]['timing_basis'] == 'fixture publication time; emulated echo offset +1 ns, not measured'


def test_no_echo_does_not_infer_acceptance():
  events = generate_transport([fixture_row(False)], 'fault', 'injected', 'no_rejected_echoes', 'fixture.json')
  assert len(events) == 1
  observed = Observer().consume(events[0])
  assert observed['receiver_acceptance'] == 'unknown'
  assert not observed['direct_steering_rejection']


def test_generator_requires_boolean_oracle():
  with pytest.raises(ValueError, match='boolean'):
    generate_transport([fixture_row(None)], 'fault', 'injected', 'complete', 'fixture.json')
