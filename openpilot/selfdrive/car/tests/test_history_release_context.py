from types import SimpleNamespace as NS

import pytest

from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import longitudinal_context
import openpilot.selfdrive.car.card as card
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR


@pytest.mark.parametrize('value', [None, '', '0', 'true', 'True', 'yes', '01', '1 ', ' 1', '1\n'])
def test_history_release_startup_requires_exact_one(value):
  environ = {} if value is None else {'NAVIGATOR_HISTORY_RELEASE': value}
  assert not longitudinal_context.history_release_startup_enabled(supported=True, environ=environ)


def test_history_release_startup_supported_and_independent():
  env = {'NAVIGATOR_HISTORY_RELEASE': '1'}
  assert longitudinal_context.history_release_startup_enabled(supported=True, environ=env)
  assert not longitudinal_context.history_release_startup_enabled(environ=env)
  assert not longitudinal_context.retention_startup_enabled(supported=True, environ=env)
  assert not longitudinal_context.history_release_startup_enabled(
    supported=True, environ={'NAVIGATOR_RECEIPT_RETENTION': '1'})


def construct_car(monkeypatch, *, coordination=True, alpha=True, old_child=False, reference_hook=True):
  monkeypatch.setattr(card, 'REPLAY', True)
  monkeypatch.setattr(card.messaging, 'SubMaster', lambda topics: NS(services=topics))
  monkeypatch.setattr(card.messaging, 'sub_sock', lambda *args, **kwargs: None)
  monkeypatch.setattr(card.messaging, 'PubMaster', lambda *args, **kwargs: NS(sock={'sendcan': None}))
  params = Params()
  # Startup reads a separate Params handle; finish fixture persistence first.
  params.put_bool('FordPrefLongitudinalCoordination', coordination, block=True)
  params.put_bool('OpenpilotEnabledToggle', True, block=True)
  assert Params().get_bool('FordPrefLongitudinalCoordination') == coordination
  assert Params().get_bool('OpenpilotEnabledToggle')
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.openpilotLongitudinalControl = alpha
  ci = CarInterface(cp)
  if not reference_hook:
    monkeypatch.delattr(type(ci.CC), "configure_coast_reference")
  calls = []
  ci.CC.configure_history_release = None if old_child else lambda *, enabled=False: calls.append(enabled)
  return card.Car(CI=ci, RI=NS()), calls


@pytest.mark.parametrize('coordination,alpha,flag,expected', [
  (True, True, None, [False]), (True, True, '1', [True]),
  (False, True, '1', []), (True, False, '1', []), (True, True, 'true', [False]),
])
def test_actual_card_startup_gate(monkeypatch, coordination, alpha, flag, expected):
  if flag is None:
    monkeypatch.delenv('NAVIGATOR_HISTORY_RELEASE', raising=False)
  else:
    monkeypatch.setenv('NAVIGATOR_HISTORY_RELEASE', flag)
  with OpenpilotPrefix():
    c, calls = construct_car(monkeypatch, coordination=coordination, alpha=alpha)
    assert calls == expected
    monkeypatch.setenv('NAVIGATOR_HISTORY_RELEASE', '0' if flag == '1' else '1')
    assert calls == expected
    assert c.longitudinal_coordination_enabled == (coordination and alpha)


@pytest.mark.parametrize('history,receipt', [('1', '0'), ('0', '1'), ('1', '1')])
def test_actual_card_flags_independent(monkeypatch, history, receipt):
  monkeypatch.setenv('NAVIGATOR_HISTORY_RELEASE', history)
  monkeypatch.setenv('NAVIGATOR_RECEIPT_RETENTION', receipt)
  with OpenpilotPrefix():
    c, calls = construct_car(monkeypatch)
    assert calls == [history == '1']
    assert (c.CI.CC.retention_owner is not None) == (receipt == '1')


def test_old_child_without_history_api_stays_compatible(monkeypatch):
  monkeypatch.setenv('NAVIGATOR_HISTORY_RELEASE', '1')
  with OpenpilotPrefix():
    c, calls = construct_car(monkeypatch, old_child=True)
    assert calls == []
    assert c.CI.CC.configure_history_release is None


@pytest.mark.parametrize('reference_enabled', [False, True])
def test_old_child_without_reference_hook(monkeypatch, reference_enabled):
  monkeypatch.setenv('NAVIGATOR_COAST_REFERENCE', '1' if reference_enabled else '0')
  monkeypatch.delenv('NAVIGATOR_RECEIPT_RETENTION', raising=False)
  with OpenpilotPrefix():
    if reference_enabled:
      with pytest.raises(ValueError, match="paired consumer"):
        construct_car(monkeypatch, reference_hook=False)
    else:
      c, calls = construct_car(monkeypatch, reference_hook=False)
      assert c.longitudinal_coordination_enabled
      assert calls == [False]
