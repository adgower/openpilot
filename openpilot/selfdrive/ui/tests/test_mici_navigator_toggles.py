import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

SOURCE = Path(__file__).parents[1] / 'mici/layouts/settings/toggles.py'
KEYS = ('FordPrefSteerAngleCurvature', 'FordPrefLowSpeedGain')


class Widget:
  def __init__(self, *args, **kwargs):
    self.checked = kwargs.get('initial_state', False)
    self.callback = kwargs.get('toggle_callback')
    self._options = args[2] if len(args) > 2 else []
    self.refresh_count = 0

  def refresh(self):
    self.refresh_count += 1

  def set_checked(self, checked):
    self.checked = checked

  def set_enabled(self, enabled):
    self.enabled = enabled

  def set_visible(self, visible):
    self.visible = visible


class Nav:
  def __init__(self):
    self.widgets = []
    self._scroller = SimpleNamespace(add_widgets=self.widgets.extend)


def layout():
  tree = ast.parse(SOURCE.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TogglesLayoutMici')
  params = Mock()
  params.get_bool.side_effect = lambda key: key in KEYS
  state = SimpleNamespace(params=params, started=False, engaged=False,
                          CP=SimpleNamespace(brand='ford', carFingerprint='FORD_EXPEDITION_MK4', flags=1),
                          add_engaged_transition_callback=Mock(), update_params=Mock(), has_longitudinal_control=True)
  env = {'NavScroller': Nav, 'ui_state': state, 'gui_app': Mock(), 'restart_needed_callback': Mock(),
         'BigMultiParamToggle': Widget, 'BigToggle': Widget, 'BigParamControl': Widget,
         'InitialCruiseSpeedButton': Widget}
  exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE), 'exec'), env)
  return env['TogglesLayoutMici'](), state


def test_controls_present_and_refresh_enabled_values():
  obj, state = layout()
  registered = dict(obj._refresh_toggles)
  for key in KEYS:
    assert key in registered
    assert registered[key] in obj.widgets
  obj._update_toggles()
  assert all(registered[key].checked for key in KEYS)
  assert obj._initial_cruise_speed in obj.widgets
  assert obj._initial_cruise_speed.refresh_count == 1


def test_callbacks_do_not_write_onroad_or_without_vehicle():
  obj, state = layout()
  for unsupported in ('onroad', 'no_vehicle', 'wrong_vehicle'):
    state.started = unsupported == 'onroad'
    state.CP = None if unsupported == 'no_vehicle' else SimpleNamespace(
      brand='ford', carFingerprint='OTHER' if unsupported == 'wrong_vehicle' else 'FORD_EXPEDITION_MK4', flags=1)
    for key in KEYS:
      dict(obj._refresh_toggles)[key].callback(False)
    state.params.put_bool.assert_not_called()


def test_offroad_callbacks_save_and_request_next_start():
  obj, state = layout()
  for key in KEYS:
    dict(obj._refresh_toggles)[key].callback(False)
  assert state.params.put_bool.call_args_list == [
    ((KEYS[0], False), {'block': True}), (('OnroadCycleRequested', True), {'block': True}),
    ((KEYS[1], False), {'block': True}), (('OnroadCycleRequested', True), {'block': True}),
  ]
