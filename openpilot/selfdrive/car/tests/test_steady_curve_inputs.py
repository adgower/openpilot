import math
from types import SimpleNamespace as NS

import pytest
from openpilot.cereal import log
from openpilot.selfdrive.car.tracking_context import create_tracking_trial, steady_curve_inputs
from openpilot.selfdrive.car.tests.test_high_speed_context import params


@pytest.mark.parametrize('which,active,saturated,valid,expected', [
  ('angleState', True, False, True, True), ('angleState', False, False, True, False),
  ('angleState', True, True, True, False), ('angleState', True, False, False, False), ('torqueState', True, False, True, False)])
def test_saturation_and_ford_yaw_preserve_source_timestamps(which, active, saturated, valid, expected):
  cs = log.ControlsState.new_message()
  state = cs.lateralControlState.init(which)
  state.active = active
  state.saturated = saturated
  class SM(dict):
    pass
  sm = SM(controlsState=cs)
  sm.valid, sm.logMonoTime = {'controlsState': valid}, {'controlsState': 500}
  parser = NS(vl={'BrakeSnData_3': {'VehYawComp_W_Actl': 2.5}}, ts_nanos={'BrakeSnData_3': {'VehYawComp_W_Actl': 350}})
  assert steady_curve_inputs(sm, parser) == (500, expected, 350, 2.5)
  for missing in ({}, {'BrakeSnData_3': {}}):
    sat_ns, available, yaw_ns, yaw = steady_curve_inputs(sm, NS(vl=missing, ts_nanos=missing))
    assert (sat_ns, available, yaw_ns) == (500, expected, 0) and math.isnan(yaw)


def test_existing_opt_in_requires_oem_acc_for_steady_curve_relief():
  cp = params()
  for oem in (True, False):
    cp.openpilotLongitudinalControl = not oem
    selected = create_tracking_trial(cp, enabled=True, high_speed_enabled=True, roll_mapping_enabled=True)
    assert (selected.steady_curve_relief is not None) == oem


@pytest.mark.parametrize('oem,enabled,expected', [(True, True, True), (True, False, False), (False, True, False)])
def test_card_constructor_wires_only_selected_ford_trial(monkeypatch, oem, enabled, expected):
  from opendbc.car.ford.interface import CarInterface
  from openpilot.selfdrive.car import card

  class SM(dict):
    def __init__(self, services):
      super().__init__(controlsState=log.ControlsState.new_message())
      self.services = services
      self.valid = {'controlsState': False}
      self.logMonoTime = {'controlsState': 0}

  flags = {'FordPrefPersistentTracking', 'FordPrefRollAwareMapping', 'FordPrefHighSpeedTracking', 'OpenpilotEnabledToggle'}
  if not enabled:
    flags.remove('FordPrefHighSpeedTracking')
  monkeypatch.setattr(card, 'Params', lambda: NS(get_bool=lambda key: key in flags, get=lambda *a, **kw: None, put=lambda *a, **kw: None))
  monkeypatch.setattr(card.messaging, 'SubMaster', SM)
  monkeypatch.setattr(card.messaging, 'sub_sock', lambda *a, **kw: None)
  monkeypatch.setattr(card.messaging, 'PubMaster', lambda *a, **kw: NS(sock={'sendcan': None}))
  cp = params()
  cp.openpilotLongitudinalControl = not oem
  ci = CarInterface(cp)
  instance = card.Car(CI=ci, RI=NS())
  assert ('controlsState' in instance.sm.services) == expected
  relief = getattr(instance.CI.CC.lateral_angle, 'steady_curve_relief', None)
  assert (relief is not None) == expected
  if expected:
    from opendbc.car.ford.lateral_angle_centered_hold import CenteredCommandHold
    assert isinstance(relief, CenteredCommandHold)
    # At boot the genuine Ford parser has registered yaw but has no sample.
    assert relief.read_inputs()[:3] == (0, False, 0)
