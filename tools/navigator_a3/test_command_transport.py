"""The virtual transport never imports or connects to Panda hardware."""
import importlib
from types import SimpleNamespace as NS

import pytest

from opendbc.car.ford.navigator_a3_command import select_command
from tools.navigator_a3.safety_audit import lmc


def transport():
  assert importlib.util.find_spec('tools.navigator_a3.command_transport') is not None
  result = importlib.import_module('tools.navigator_a3.command_transport').SimulatedTransport('fixture')
  result.control_input(True, False, True, True)
  return result


def health(t, **changes):
  values = dict(safetyTxBlocked=0, controlsAllowed=True, safetyRxChecksInvalid=False,
                safetyModel='ford', safetyParam=2, alternativeExperience=0)
  values.update(changes)
  return t, NS(**values)


def test_publication_never_implies_acceptance_and_repeated_echo_is_ambiguous():
  t = transport()
  t.health(*health(1))
  frame = (982, bytes(lmc(angle=1)), 0)
  choice = select_command('shadow', None, frame)
  assert t.publish(10, choice)['receiver_acceptance'] == 'unknown'
  t.publish(60_000_010, choice)
  row = t.feedback('returned', 60_000_011, frame)
  assert row['attribution'] == 'ambiguous' and len(row['candidate_orders']) == 2
  assert row['receiver_acceptance'] == 'unknown'
  assert not t.fault_reason


@pytest.mark.parametrize('fault', ['rejected', 'permission', 'configuration', 'invalid_health'])
def test_fault_is_persistent_and_neutral_does_not_clear_it(fault):
  t = transport()
  t.health(*health(1))
  active = (982, bytes(lmc(angle=1)), 0)
  t.publish(10, select_command('shadow', None, active))
  if fault == 'rejected':
    t.feedback('rejected', 11, active)
  else:
    t.health(*health(11, **({'controlsAllowed':False} if fault=='permission' else
                          {'safetyParam':3} if fault=='configuration' else {})), valid=fault!='invalid_health')
  reason = t.fault_reason
  assert reason
  assert t.publish(12, select_command('shadow', None, active)) is None
  neutral = (982, bytes(lmc(active=False)), 0)
  assert t.publish(13, select_command('shadow', None, neutral)) is not None
  t.feedback('returned', 14, neutral)
  t.health(*health(15))
  assert t.fault_reason == reason and t.calculation_fault_reason == reason


def test_absent_proposal_does_not_publish_or_advance_observer():
  t = transport()
  before = t.bridge.order
  assert t.publish(10, select_command('requested', None, None)) is None
  assert t.bridge.order == before


def test_mixed_provenance_is_not_correlated():
  t = transport()
  t.health(*health(1))
  frame=(982,bytes(lmc(angle=1)),0)
  t.publish(10,select_command('shadow',None,frame))
  row=t.feedback('returned',11,frame,provenance='recorded')
  assert row['event_type']=='identity_mismatch'
  assert row['candidate_orders']==[]
  assert t.fault_reason=='identity_mismatch'


@pytest.mark.parametrize('change', [{'controlsAllowed':False}, {'safetyRxChecksInvalid':True}])
def test_permission_loss_during_driver_pause_remains_latched(change):
  t = transport()
  t.health(*health(1))
  active = (982, bytes(lmc(angle=1)), 0)
  t.publish(10, select_command('shadow', None, active))
  t.publish(11, select_command('shadow', None, (982, bytes(lmc(active=False)), 0)))
  t.health(*health(12, **change))
  t.health(*health(13))
  assert t.fault_reason == 'permission_unavailable'
  assert t.publish(14, select_command('shadow', None, active)) is None


def test_serialized_synthetic_bridge_cannot_label_publications_live():
  import json
  from openpilot.selfdrive.car.navigator_a3_runtime import diagnostic_json
  t = transport()
  c = NS(navigator_a3=NS(diagnostic={}))
  d = json.loads(diagnostic_json(c, t.bridge))
  assert d['published_frames_provenance'] == 'synthetic'

@pytest.mark.parametrize('inputs', [(True, True, True, True), (True, False, False, True),
                                  (True, False, True, False), (False, False, True, True)])
def test_explicit_ineligible_input_prevents_active_simulated_publication(inputs):
  t = transport()
  t.health(*health(1))
  t.control_input(*inputs)
  assert t.publish(10, select_command('shadow', None, (982, bytes(lmc(angle=1)), 0))) is None
