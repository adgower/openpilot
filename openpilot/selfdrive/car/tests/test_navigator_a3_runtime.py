import pytest
from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeObserver, RuntimeBridge


def packet(kind, order, payload='14237d0fa2008000', **changes):
  e = {'route_id': 'r', 'provenance': 'live', 'kind': kind, 'order': order, 't_ns': order * 10,
           'valid': True, 'bus': 0, 'address': 982, 'dlc': 8, 'data_hex': payload}
  e.update(changes)
  return e


def health(order, counter=0, **changes):
  e = {'route_id': 'r', 'provenance': 'live', 'kind': 'health', 'order': order, 't_ns': order * 10, 'valid': True,
           'panda_index': 0, 'safetyTxBlocked': counter, 'controlsAllowed': True, 'safetyRxChecksInvalid': False,
           'safetyModel': 'ford', 'safetyParam': 4, 'alternativeExperience': 0}
  e.update(changes)
  return e


def test_evicted_history_never_manufactures_unique_attribution():
  o = RuntimeObserver('r', 'live', capacity=2)
  o.consume(packet('published', 0))
  o.consume(packet('published', 1, '04237d0fa2008000'))
  o.consume(packet('published', 2))
  r = o.consume(packet('rejected', 3))
  assert len(o.publications) == 2
  assert r['candidate_orders'] == [2]
  assert r['coverage_incomplete'] and r['attribution'] == 'incomplete'
  assert r['publication_to_observation_ns'] is None
  assert r['receiver_acceptance'] == 'unknown'


def test_all_retained_candidates_and_missing_echo_remain_unknown():
  o = RuntimeObserver('r', 'live', capacity=4)
  o.consume(packet('published', 0))
  o.consume(packet('published', 1))
  r = o.consume(packet('returned', 2))
  assert r['candidate_orders'] == [0, 1] and r['attribution'] == 'ambiguous'
  assert r['receiver_acceptance'] == 'unknown'
  assert o.consume(packet('rejected', 3))['candidate_orders'] == [0, 1]


@pytest.mark.parametrize('change', [{'route_id':'other'}, {'provenance':'synthetic'}, {'valid':False}, {'bus':1},
                                  {'data_hex':'0000000000000000'}, {'t_ns':-1}])
def test_identity_validity_and_native_time_prevent_match(change):
  o = RuntimeObserver('r', 'live')
  o.consume(packet('published', 0))
  assert o.consume(packet('rejected', 1, **change))['candidate_orders'] == []


def test_health_is_distinct_and_invalid_does_not_rebase():
  o = RuntimeObserver('r', 'live')
  o.consume(health(0, 10))
  assert o.consume(health(1, 20, valid=False))['event_type'] == 'invalid_health'
  r = o.consume(health(2, 11, controlsAllowed=False))
  assert r['counter_delta'] == 1 and r['permission_lost'] and r['health_coverage_gap']
  r = o.consume(health(3, 0))
  assert r['counter_interpretation'] == 'discontinuity_reset_or_wrap_unknown'
  assert r['counter_delta'] is None


def test_health_configuration_unknown_and_out_of_order_fail_closed():
  o = RuntimeObserver('r', 'live')
  o.consume(health(0))
  assert o.consume(health(1, t_ns=0))['event_type'] == 'out_of_order_health'
  assert o.consume(health(2, safetyModel=None))['event_type'] == 'invalid_health'
  with pytest.raises(ValueError):
    o.consume(dict(health(3), accepted=True))


def test_bridge_latches_fault_and_never_acknowledges_synthetic_command():
  b = RuntimeBridge('shadow', [('ford', 4, 0)], 'r', 'live')
  b.observe(packet('published', 0))
  b.observe(packet('rejected', 1))
  assert b.fault_reason == 'direct_steering_rejection'
  b.observe(packet('returned', 2))
  b.observe(health(3))
  assert b.fault_reason == 'direct_steering_rejection'
  assert not b.disengagement_requested
  assert RuntimeBridge('requested', [('ford',4,0)], 'r', 'live').disengagement_requested


def test_bridge_bounds_pending_diagnostics_without_looking_like_complete_history():
  b = RuntimeBridge('shadow', [('ford',4,0)], 'r', 'live', capacity=2)
  for n in range(50):
    b.observe(packet('rejected', n))
  d = b.diagnostics()
  assert len(d['transport']) <= 16 and d['dropped_diagnostics'] > 0
  assert b.diagnostics()['transport'] == []


def test_inactive_permission_is_normal_but_active_permission_loss_latches():
  b = RuntimeBridge('shadow', [('ford',4,0)], 'r', 'live')
  b.controls_ready = True
  b.observe(health(0, controlsAllowed=False))
  assert b.fault_reason is None
  b.active_request = True
  b.observe(health(1, controlsAllowed=False))
  assert b.fault_reason == 'permission_unavailable'


def test_startup_safety_transition_does_not_latch_but_later_mismatch_does():
  b = RuntimeBridge('shadow', [('ford',4,0)], 'r','live')
  b.observe(health(0, safetyModel='noOutput', safetyParam=0, controlsAllowed=False))
  assert b.fault_reason is None
  b.controls_ready = True
  b.observe(health(1, safetyModel='noOutput', safetyParam=0, controlsAllowed=False))
  assert b.fault_reason is None
  b.observe(health(2, controlsAllowed=False))
  assert b.configuration_armed
  b.observe(health(3, safetyParam=0))
  assert b.fault_reason == 'configuration_mismatch'
  b.observe(health(4))
  assert b.fault_reason == 'configuration_mismatch'


def test_invalid_startup_health_waits_for_evidence_then_faults_when_armed():
  b = RuntimeBridge('shadow', [('ford',4,0)], 'r','live')
  b.observe(health(0, valid=False))
  assert b.fault_reason is None
  b.controls_ready = True
  b.observe(health(1))
  b.observe(health(2, valid=False))
  assert b.fault_reason == 'invalid_health'
