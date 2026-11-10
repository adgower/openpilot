"""Optional test availability must not add work or authority to ordinary Alpha."""
from types import SimpleNamespace as NS

import pytest
from openpilot.common.params import Params, ParamKeyFlag, ParamKeyType
from openpilot.selfdrive.car import prediction_pilot as pilot
from openpilot.tools.navigator.prediction_pilot import command

KEY = 'NavigatorPredictionPilotEnabled'


@pytest.mark.parametrize('stale', [False, True])
def test_default_factory_never_constructs_optional_worker(monkeypatch, stale):
  def unexpected(*args, **kwargs):
    pytest.fail('default Alpha constructed optional worker')
  monkeypatch.setattr(pilot, 'PilotWorker', unexpected)
  params = NS(get_bool=lambda key: False, get=lambda key: 'stale-session' if stale else None)
  assert pilot.create_prediction_pilot(params, NS()) is None


def test_explicit_opt_in_constructs_disabled_runner(monkeypatch):
  started = []
  worker = NS(session_id='s', source_id='x', cancel_generation=0, health_generation=0, start=lambda: started.append(True))
  monkeypatch.setattr(pilot, 'PilotWorker', lambda *args: worker)
  cp = NS(carFingerprint='FORD EXPEDITION 2018', openpilotLongitudinalControl=True, safetyConfigs=[])
  runtime = pilot.create_prediction_pilot(NS(get_bool=lambda key: key == KEY), cp)
  assert runtime.worker is worker and started == [True]
  assert runtime.runner.state == 'disabled'


def test_native_key_defaults_off_and_is_limited_to_one_session(tmp_path):
  p = Params(str(tmp_path))
  assert KEY.encode() in p.all_keys(), 'native opt-in key is missing'
  assert p.get_type(KEY) == ParamKeyType.BOOL
  assert p.get_default_value(KEY) is False and not p.get_bool(KEY)
  p.put_bool('FordPrefLongitudinalCoordination', True, block=True)
  p.put_bool(KEY, True, block=True)
  p.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON | ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  assert p.get_bool(KEY), 'offroad opt-in must survive the intended startup'
  p.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
  assert not p.get_bool(KEY)
  p.put_bool(KEY, True, block=True)
  p.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
  assert not p.get_bool(KEY) and p.get_bool('FordPrefLongitudinalCoordination')


def test_offroad_enable_and_disable_clear_stale_mailboxes_without_arming(tmp_path):
  p = Params(str(tmp_path))
  p.put_bool('IsOffroad', True, block=True)
  for name in [pilot.PERMIT, pilot.CANCEL, pilot.STATUS]:
    p.put(name, 'stale', block=True)
  result = command(p, 'worker-enable')
  assert result['worker_enabled_for_next_session'] is True and result['test_armed'] is False
  assert p.get_bool(KEY) and all(p.get(k) is None for k in [pilot.PERMIT, pilot.CANCEL, pilot.STATUS])
  p.put(pilot.PERMIT, 'stale-permit', block=True)
  result = command(p, 'worker-disable')
  assert result['worker_enabled_for_next_session'] is False and result['test_armed'] is False
  assert not p.get_bool(KEY) and p.get(pilot.PERMIT) is None


@pytest.mark.parametrize('action', ['worker-enable', 'worker-disable'])
def test_onroad_availability_change_is_rejected_before_any_write(action):
  writes = []
  p = NS(get_bool=lambda key: False, get=lambda key: None,
         put_bool=lambda *a, **kw: writes.append(a), remove=lambda *a: writes.append(a))
  with pytest.raises(ValueError, match='offroad'):
    command(p, action)
  assert not writes


def test_partial_enable_failure_leaves_flag_false():
  values = {KEY: True}
  def remove(key):
    raise OSError('private fixture storage failure')
  p = NS(get_bool=lambda key: True if key == 'IsOffroad' else values[key],
         put_bool=lambda key, value, **kw: values.update({key: value}), remove=remove, get=lambda _: None)
  with pytest.raises(OSError):
    command(p, 'worker-enable')
  assert values[KEY] is False


def test_enable_losing_offroad_state_fails_closed():
  values = {KEY: False}
  states = iter([True, True, False])
  p = NS(get_bool=lambda key: next(states) if key == 'IsOffroad' else values[key],
         put_bool=lambda key, value, **kw: values.update({key: value}), remove=lambda _: None,
         get=lambda _: None)
  with pytest.raises(ValueError, match='offroad'):
    command(p, 'worker-enable')
  assert values[KEY] is False


@pytest.mark.parametrize('failure', ['disable-write', 'clear-permit', 'enable-write', 'enable-exception'])
def test_storage_failures_cannot_report_success(tmp_path, failure):
  p = Params(str(tmp_path))
  p.put_bool('IsOffroad', True, block=True)
  p.put_bool(KEY, failure == 'disable-write', block=True)
  p.put(pilot.PERMIT, 'stale', block=True)

  def write(key, value, **kwargs):
    if failure == 'disable-write' or (failure == 'enable-write' and value):
      return  # The native Params wrapper can silently discard a failed write.
    p.put_bool(key, value, **kwargs)
    if failure == 'enable-exception' and value:
      raise OSError('failure after value became visible')

  proxy = NS(get_bool=p.get_bool, get=p.get, put_bool=write,
             remove=lambda key: None if failure == 'clear-permit' else p.remove(key))
  with pytest.raises((ValueError, OSError)):
    command(proxy, 'worker-enable')
  # If disabling itself fails, report failure; never claim the flag was cleared.
  assert p.get_bool(KEY) is (failure == 'disable-write')


@pytest.mark.parametrize('opted_in', [False, True])
def test_actual_card_startup_keeps_normal_alpha_with_optional_worker(monkeypatch, opted_in):
  from openpilot.selfdrive.car import card
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.ford.values import CAR

  cp = CarInterface.get_non_essential_params(CAR.LINCOLN_NAVIGATOR_MK4)
  cp.openpilotLongitudinalControl = True
  ci = CarInterface(cp)
  writes, starts, attempts = [], [], []
  class PrivateParams:
    def get_bool(self, key):
      return opted_in if key == KEY else key in ('FordPrefLongitudinalCoordination', 'AlphaLongitudinalEnabled', 'OpenpilotEnabledToggle')
    def get(self, key, **kwargs):
      return 40 if key == 'InitialCruiseSpeedKph' else None
    def put(self, key, value, **kwargs):
      writes.append(key)
    put_bool = put

  def worker(*args):
    attempts.append(True)
    assert opted_in, 'ordinary Alpha created a test worker'
    return NS(session_id='s', source_id='x', cancel_generation=0, health_generation=0, start=lambda: starts.append(True))

  monkeypatch.setattr(pilot, 'PilotWorker', worker)
  monkeypatch.setattr(card, 'Params', PrivateParams)
  monkeypatch.setattr(card, 'REPLAY', False)
  monkeypatch.setattr(card, 'get_car', lambda *a, **kw: ci)
  monkeypatch.setattr(card, 'interfaces', {cp.carFingerprint: NS(RadarInterface=lambda _: NS())})
  monkeypatch.setattr(card.messaging, 'sub_sock', lambda *a, **kw: None)
  monkeypatch.setattr(card.messaging, 'recv_one_retry', lambda *a: NS(can=[None]))
  monkeypatch.setattr(card.messaging, 'SubMaster', lambda *a, **kw: NS())
  monkeypatch.setattr(card.messaging, 'PubMaster', lambda *a: NS(sock={'sendcan': None}))
  monkeypatch.setattr(card, 'open_stall_recorder', lambda _: None, raising=False)
  c = card.Car()
  assert c.CP.openpilotLongitudinalControl and c.longitudinal_coordination_enabled
  assert ci.CC.longitudinal_coordination_enabled
  assert (c.prediction_pilot is not None) is opted_in
  assert ci.CC.prediction_pilot is c.prediction_pilot
  assert starts == ([True] if opted_in else [])
  assert attempts == starts
  assert not set(writes) & {KEY, pilot.PERMIT, pilot.CANCEL, pilot.STATUS}
