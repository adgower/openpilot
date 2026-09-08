"""Transport evidence never becomes an invented actuator acknowledgment."""
import pytest
from tools.navigator_a3.transport_observer import Observer, resolve_candidates


def packet(kind, order, t=None, **kwargs):
  return dict(route_id='route-A', provenance='recorded', order=order, t_ns=order * 100 if t is None else t,
              valid=True, kind=kind, bus=0, address=982, dlc=8, data_hex='14237d0fa2008000', **kwargs)


def health(order, counter=0, allowed=True, valid=True, **kwargs):
  return dict(route_id='route-A', provenance='recorded', order=order, t_ns=order * 100,
              valid=valid, kind='health', panda_index=0, safetyTxBlocked=counter,
              controlsAllowed=allowed, safetyRxChecksInvalid=False, safetyModel='ford', safetyParam=4,
              alternativeExperience=0, **kwargs)


def run(events):
  observer = Observer()
  rows = [observer.consume(e) for e in events]
  return observer, rows


def test_unique_rejection_and_transport_delay():
  observer, rows = run([packet('published', 0), packet('rejected', 1)])
  r = rows[-1]
  assert r['direct_steering_rejection'] is True
  assert r['attribution'] == 'unique'
  assert r['publication_to_observation_ns'] == 100
  assert r['receiver_acceptance'] == 'unknown'
  assert [e['order'] for e in resolve_candidates(observer.groups, r)] == [0]


def test_all_repeated_publications_retained_no_arbitrary_window():
  observer, rows = run([packet('published', 0), packet('published', 1), packet('rejected', 2, 10**12)])
  assert rows[-1]['candidate_count'] == 2
  assert rows[-1]['attribution'] == 'ambiguous'
  assert rows[-1]['publication_to_observation_ns'] is None
  assert [x['order'] for x in resolve_candidates(observer.groups, rows[-1])] == [0, 1]


def test_returned_is_not_acknowledged_or_consumed():
  observer, rows = run([packet('published', 0), packet('returned', 1), packet('rejected', 2)])
  assert rows[1]['receiver_acceptance'] == 'unknown'
  assert rows[1]['direct_steering_rejection'] is False
  assert rows[2]['candidate_count'] == 1
  assert len(resolve_candidates(observer.groups, rows[2])) == 1


def test_duplicate_echoes_stay_observations_not_unique_command_count():
  _, rows = run([packet('published', 0), packet('rejected', 1), packet('rejected', 2)])
  assert [r['candidate_count'] for r in rows[1:]] == [1, 1]
  assert rows[2]['possible_duplicate_echo'] is True


def test_missing_publication_or_echo_never_success():
  _, rows = run([packet('rejected', 0), packet('published', 1)])
  assert rows[0]['attribution'] == 'unmatched'
  assert rows[0]['publication_to_observation_ns'] is None
  assert rows[1]['receiver_acceptance'] == 'unknown'


def test_native_time_and_observation_order_both_required():
  observer, rows = run([packet('published', 0, 500), packet('published', 1, 100), packet('rejected', 2, 300),
                       packet('published', 3, 200)])
  assert rows[2]['candidate_count'] == 1
  assert [x['order'] for x in resolve_candidates(observer.groups, rows[2])] == [1]
  assert rows[2]['publication_to_observation_ns'] == 200


@pytest.mark.parametrize(('field', 'value'), [('route_id', 'route-B'), ('provenance', 'synthetic'),
                                              ('bus', 1), ('address', 983), ('dlc', 7),
                                              ('data_hex', '14237d0fa2008001')])
def test_correlation_identity_boundaries(field, value):
  changed = packet('rejected', 1)
  changed[field] = value
  _, rows = run([packet('published', 0), changed])
  assert rows[-1]['candidate_count'] == 0


def test_invalid_publication_not_evidence_and_invalid_echo_not_rejection():
  p = packet('published', 0)
  p['valid'] = False
  r = packet('rejected', 2)
  r['valid'] = False
  _, rows = run([p, packet('rejected', 1), r])
  assert rows[1]['candidate_count'] == 0
  assert rows[2]['direct_steering_rejection'] is False
  assert rows[2]['attribution'] == 'invalid_evidence'


def test_unrelated_rejection_separate_from_steering():
  r = packet('rejected', 0)
  r['address'] = 0x186
  _, rows = run([r, health(1), health(2, 3)])
  assert rows[0]['event_type'] == 'other_tx_rejection'
  assert not rows[0]['direct_steering_rejection']
  assert rows[-1]['counter_delta'] == 3
  assert rows[-1]['counter_interpretation'] == 'aggregate_increase'
  assert not rows[-1]['direct_steering_rejection']


def test_counter_decrease_never_inferred_wrap():
  _, rows = run([health(0, 2**32-1), health(1, 1)])
  assert rows[-1]['counter_interpretation'] == 'discontinuity_reset_or_wrap_unknown'
  assert rows[-1]['counter_delta'] is None


def test_health_invalid_gap_and_config_change():
  _, rows = run([health(0, 2), health(1, 100, valid=False), health(2, 4),
                dict(health(3, 4), safetyParam=5)])
  assert rows[1]['counter_delta'] is None
  assert rows[2]['counter_delta'] == 2
  assert rows[2]['health_coverage_gap'] is True
  assert rows[3]['configuration_changed'] is True


def test_permission_loss_and_rx_fault_are_distinct():
  _, rows = run([health(0), dict(health(1, allowed=False), safetyRxChecksInvalid=True)])
  assert rows[1]['permission_lost'] is True
  assert rows[1]['rx_checks_invalid'] is True
  assert rows[1]['direct_steering_rejection'] is False


def test_out_of_order_health_does_not_rebase_counter():
  _, rows = run([dict(health(0, 10), t_ns=500), dict(health(1, 2), t_ns=100), dict(health(2, 11), t_ns=600)])
  assert rows[1]['health_out_of_order'] is True
  assert rows[1]['counter_delta'] is None
  assert rows[2]['counter_delta'] == 1


def test_oracle_fields_are_not_inputs():
  event = packet('returned', 0)
  event['accepted'] = True
  with pytest.raises(ValueError, match='oracle'):
    Observer().consume(event)


def test_equal_timestamps_allow_preceding_observation_only():
  observer, rows = run([packet('rejected', 0, 100), packet('published', 1, 100), packet('rejected', 2, 100)])
  assert rows[0]['candidate_count'] == 0
  assert rows[2]['candidate_count'] == 1
  assert rows[2]['publication_to_observation_ns'] == 0
  assert resolve_candidates(observer.groups, rows[0]) == []


def test_missing_health_configuration_cannot_establish_continuity():
  a, b = health(0), health(1, counter=1)
  for e in (a, b):
    del e['safetyModel']
  _, rows = run([a, b])
  assert all(r['event_type'] == 'invalid_health' and r['counter_delta'] is None for r in rows)


@pytest.mark.parametrize('field', ['route_id', 'provenance'])
def test_null_identity_cannot_correlate(field):
  e = packet('published', 0)
  e[field] = None
  with pytest.raises(ValueError, match='identity'):
    Observer().consume(e)
