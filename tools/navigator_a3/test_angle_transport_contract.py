"""Executable host-knowledge requirements, separate from any MCU admission oracle."""
import pytest

from tools.navigator_a3.angle_transport_contract import cases, observe_case


@pytest.mark.parametrize('delay_ms', [5, 50, 100, 250])
def test_first_direct_evidence_arrives_only_at_observed_echo(delay_ms):
  result = observe_case(cases(delay_ms)['delayed_rejection'])
  assert result['first_direct_rejection_ns'] == 1_000_000_000 + delay_ms * 1_000_000
  assert all(r['fault_reason'] is None for r in result['rows'] if r['input']['t_ns'] < result['first_direct_rejection_ns'])
  assert result['first_direct_publication_latency_ns'] == delay_ms * 1_000_000
  assert result['rows'][-1]['fault_reason'] == 'direct_steering_rejection'
  assert result['rows'][-1]['calculation_fault_reason'] is None
  assert result['physical_response'] == 'unknown'
  assert result['guaranteed_detection_bound_ns'] is None


@pytest.mark.parametrize('case', ['missing_echo', 'returned_only', 'aggregate_only', 'unrelated_rejection'])
def test_non_direct_evidence_does_not_invent_acknowledgment_or_direct_fault(case):
  result = observe_case(cases()[case])
  assert result['first_direct_rejection_ns'] is None
  assert all(r['observation']['receiver_acceptance'] == 'unknown' for r in result['rows'])
  assert result['rows'][-1]['fault_reason'] is None
  assert result['guaranteed_detection_bound_ns'] is None


def test_repeated_payload_keeps_ambiguous_attribution():
  result = observe_case(cases()['ambiguous_echo'])
  assert result['first_direct_rejection_ns'] is not None
  assert result['first_direct_publication_latency_ns'] is None
  assert result['rows'][-1]['observation']['attribution'] == 'ambiguous'
  assert len(result['rows'][-1]['observation']['candidate_orders']) == 2


@pytest.mark.parametrize('case,reason', [('permission_loss', 'permission_unavailable'),
                                       ('invalid_health', 'invalid_health'), ('config_change', 'configuration_mismatch')])
def test_later_nonexempt_fault_persists_after_returned_and_restored_health(case, reason):
  result = observe_case(cases()[case])
  assert result['rows'][-1]['fault_reason'] == 'direct_steering_rejection'
  assert result['rows'][-1]['calculation_fault_reason'] == reason


def test_eviction_does_not_manufacture_unique_match():
  result = observe_case(cases()['ambiguous_echo'], capacity=1)
  assert result['rows'][-1]['observation']['attribution'] == 'incomplete'
  assert result['first_direct_publication_latency_ns'] is None


@pytest.mark.parametrize('name', list(cases()))
def test_requested_mode_is_blocked_before_and_after_every_event(name):
  result = observe_case(cases()[name], mode='requested')
  assert all(r['disengagement_requested'] for r in result['rows'])
  assert all(r['fault_reason'] == 'physical_enforcement_unvalidated' for r in result['rows'])
  assert result['active_lifecycle_validated'] is False
