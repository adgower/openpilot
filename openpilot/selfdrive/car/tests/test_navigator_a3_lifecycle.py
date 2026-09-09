import importlib
from dataclasses import FrozenInstanceError

import pytest


def lifecycle(role='a3'):
  assert importlib.util.find_spec('openpilot.selfdrive.car.navigator_a3_lifecycle') is not None
  return importlib.import_module('openpilot.selfdrive.car.navigator_a3_lifecycle').Lifecycle(role)


def test_driver_pause_is_not_session_termination_or_permission_reset():
  s=lifecycle()
  s.control_input(True, False, True, True)
  assert s.evaluate(True,True).calculation_eligible
  s.control_input(True, True, True, True)
  d=s.evaluate(True,True)
  assert d.phase=='driver_pause' and d.neutral_reset and not d.disengagement_intent
  s.observe_fault('permission_unavailable')
  s.control_input(False,False,True,True)
  assert s.session_requested
  s.control_input(True,False,True,True)
  d=s.evaluate(True,True)
  assert d.fault_reason=='permission_unavailable'
  assert not d.calculation_eligible and d.disengagement_intent
  assert not d.production_activation_allowed


@pytest.mark.parametrize('role,eligible',[('a2',True),('a3',False)])
def test_rejection_effect_is_bound_to_monitored_command(role,eligible):
  s=lifecycle(role)
  s.control_input(True,False,True,True)
  s.observe_fault('direct_steering_rejection')
  d=s.evaluate(True,True)
  assert d.fault_reason=='direct_steering_rejection'
  assert d.calculation_eligible==eligible
  s.observe_fault('configuration_mismatch')
  assert s.evaluate(True,True).calculation_fault_reason == ('configuration_mismatch' if role=='a2' else 'direct_steering_rejection')


@pytest.mark.parametrize('source,measurement',[(False,True),(True,False),(False,False)])
def test_freshness_neutralizes_without_fabricating_a_persistent_fault(source,measurement):
  s=lifecycle()
  s.control_input(True,False,source,measurement)
  assert s.evaluate(True,True).neutral_reset
  assert not s.fault_reason
  s.control_input(True,False,True,True)
  assert s.evaluate(True,True).calculation_eligible


def test_decision_cannot_grant_production_permission():
  s=lifecycle()
  s.control_input(True,False,True,True)
  d=s.evaluate(True,True)
  with pytest.raises(FrozenInstanceError):
    d.production_activation_allowed=True


def test_live_bridge_cannot_select_simulated_a3_monitoring():
  from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeBridge
  with pytest.raises(ValueError):
    RuntimeBridge('shadow',[('ford',2,0)],'r','live',command_role='a3')


@pytest.mark.parametrize('fault', ['rejected', 'permission', 'configuration', 'ambiguous'])
def test_real_card_and_simulator_share_pause_fault_lifecycle(monkeypatch, fault):
  import json
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.car.tests.test_navigator_a3_card import harness, frames
  from opendbc.car.ford.tests.test_navigator_a3_runtime import sample
  from tools.navigator_a3.command_transport import SimulatedTransport
  from tools.navigator_a3.safety_audit import lmc
  from opendbc.car.ford.navigator_a3_command import select_command

  instance, cc, _ = harness(monkeypatch, 'shadow')
  baseline, _, _ = harness(monkeypatch, 'a2')
  transport = SimulatedTransport('card-session')
  instance.navigator_a3_bridge = transport.bridge
  monkeypatch.setattr('openpilot.selfdrive.car.card.observe_publication', lambda *a, **k: None)
  health = dict(safetyTxBlocked=0, controlsAllowed=True, safetyRxChecksInvalid=False,
                safetyModel='ford', safetyParam=2, alternativeExperience=0)
  transport.health(1_000_000_001, NS(**health))
  _, cs = sample(25)
  cs.out.canValid = True
  for c in (instance, baseline):
    c.CI.apply = lambda control, now, c=c: c.CI.CC.update(control, cs, now)
  active = (982, bytes(lmc(angle=1)), 0)
  transport.control_input(True, False, True, True)
  transport.publish(1_000_000_002, select_command('shadow', None, active))
  transport.publish(1_000_000_003, select_command('shadow', None, active))
  for step in range(1, 16):
    now = 1_000_000_000 + step * 10_000_000
    cs.out.steeringPressed = step < 6
    if step == 2:
      if fault in ('rejected', 'ambiguous'):
        row = transport.feedback('rejected' if fault == 'rejected' else 'returned', now, active)
        assert row['attribution'] == 'ambiguous'
        assert row['receiver_acceptance'] == 'unknown'
      else:
        transport.health(now, NS(**dict(health, **({'controlsAllowed':False} if fault == 'permission' else {'safetyParam':3}))))
    if step == 6:
      transport.health(now, NS(**health))
      transport.feedback('returned', now + 1, active)
    monkeypatch.setattr('openpilot.selfdrive.car.card.time.monotonic', lambda now=now: now/1e9)
    for c in (instance, baseline):
      c.sm.logMonoTime['carControl'] = now
      for signals in c.CI.can_parsers['pt'].ts_nanos.values():
        for name in signals:
          signals[name] = now - 100_000_001 if step == 7 else now
      c.controls_update(cs.out, cc)
      c.state_publish(cs.out, None)
    assert frames(instance) == frames(baseline)
    diagnostic = json.loads(instance.pm.read('carOutput').carOutput.navigatorA3.diagnosticsJson)
    decision = transport.bridge.decision()
    assert diagnostic['lifecycle']['calculation_eligible'] == decision.calculation_eligible
    assert not decision.production_activation_allowed
    if fault != 'ambiguous' and step >= 2:
      assert decision.disengagement_intent and not decision.calculation_eligible
      assert transport.publish(now, select_command('shadow', None, active)) is None
    elif step < 6:
      assert decision.phase == 'driver_pause'
      output = diagnostic['controller']['output']
      assert output is None or output['mode'] == 0
    elif step == 7:
      assert decision.phase == 'invalid_input'
    else:
      assert decision.calculation_eligible


@pytest.mark.parametrize('raw,filtered,eligible', [(9.,10.,True), (10.,9.,False)])
def test_lifecycle_uses_controller_raw_speed_for_measurement_boundary(monkeypatch, raw, filtered, eligible):
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.car.tests.test_navigator_a3_card import harness
  from opendbc.car.ford.tests.test_navigator_a3_runtime import sample
  instance, command, _ = harness(monkeypatch, 'shadow')
  bridge = instance.navigator_a3_bridge
  bridge.panda(1_000_000_001, True, 0, NS(safetyTxBlocked=0, controlsAllowed=True,
    safetyRxChecksInvalid=False, safetyModel='ford', safetyParam=2, alternativeExperience=0))
  _, cs = sample(25)
  cs.out.canValid = True
  cs.out.vEgoRaw, cs.out.vEgo = raw, filtered
  instance.CI.apply = lambda control, now: instance.CI.CC.update(control, cs, now)
  monkeypatch.setattr('openpilot.selfdrive.car.card.time.monotonic', lambda: 1.1)
  instance.sm.logMonoTime['carControl'] = 1_100_000_000
  instance.controls_update(cs.out, command)
  assert bridge.decision().calculation_eligible == eligible
