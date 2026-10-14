"""Exercise the toggle callback and supported-vehicle predicate without a GPU window."""
import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

SOURCE = Path(__file__).parents[1] / 'layouts/settings/toggles.py'


def load_methods():
  tree = ast.parse(SOURCE.read_text())
  klass = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TogglesLayout')
  methods = [n for n in klass.body if isinstance(n, ast.FunctionDef) and n.name in ('_pinion_option_supported', '_toggle_callback')]
  module = ast.Module(body=methods, type_ignores=[])
  state = SimpleNamespace(started=False, CP=SimpleNamespace(brand='ford', carFingerprint='FORD_EXPEDITION_MK4', flags=1))
  env = {'ui_state': state}
  exec(compile(ast.fix_missing_locations(module), str(SOURCE), 'exec'), env)
  return env, state


def test_supported_only_offroad_expedition():
  env, state = load_methods()
  assert env['_pinion_option_supported']()
  state.started = True
  assert not env['_pinion_option_supported']()
  state.started = False
  state.CP.carFingerprint = 'FORD_EXPLORER_MK6'
  assert not env['_pinion_option_supported']()
  state.CP = None
  assert not env['_pinion_option_supported']()


def test_onroad_callback_cannot_write_option():
  env, state = load_methods()
  state.started = True
  obj = SimpleNamespace(_params=Mock(), _pinion_option_supported=env['_pinion_option_supported'])
  env['_toggle_callback'](obj, True, 'FordPrefSteerAngleCurvature')
  obj._params.put_bool.assert_not_called()


def test_offroad_callback_saves_and_requests_reinitialization():
  env, _ = load_methods()
  obj = SimpleNamespace(_params=Mock(), _pinion_option_supported=env['_pinion_option_supported'],
                        _toggle_defs={'FordPrefSteerAngleCurvature': (None, None, None, True)})
  env['_toggle_callback'](obj, True, 'FordPrefSteerAngleCurvature')
  assert obj._params.put_bool.call_args_list == [
    ((('FordPrefSteerAngleCurvature', True)), {'block': True}),
    ((('OnroadCycleRequested', True)), {'block': True}),
  ]
