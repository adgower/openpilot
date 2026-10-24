from types import SimpleNamespace as NS
from collections import defaultdict
from openpilot.selfdrive.car.tests.test_longitudinal_pose import command, Inputs
from openpilot.selfdrive.car.longitudinal_pose import ConsumerPoseRefresh
import openpilot.selfdrive.car.card as card
from opendbc.car import structs
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, DBC
from opendbc.car.ford.longitudinal_coordination import Torque


def test_actual_card_apply_boundary_uses_fresh_copy_and_actual_controller(monkeypatch):
  monkeypatch.setattr(card, "REPLAY", True)
  now = 1_000_000_000
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.openpilotLongitudinalControl = True
  c = CarController(DBC[cp.carFingerprint], cp)
  c.longitudinal_coordination_enabled = True
  cs = structs.CarState(vEgo=8.0, vEgoRaw=8.0, canValid=True)
  state = NS(
    out=cs,
    buttons_stock_values=defaultdict(int),
    lkas_status_stock_values=defaultdict(int),
    acc_tja_status_stock_values=defaultdict(int),
    acc_tja_status_stock_values_deprecated=defaultdict(int),
  )
  captured = []
  wire = []

  def apply(cc, ns):
    captured.append(cc)
    state.longitudinal_brake_torque = Torque(ns / 1e9, 0.0)
    return c.update(cc, state, ns)

  sm = NS(all_alive=lambda *_: True, all_checks=lambda *_: True)
  h = NS(
    initialized_prev=True,
    sm=sm,
    tracking_trial=None,
    longitudinal_coordination_enabled=True,
    CI=NS(CC=c, apply=apply),
    can_log_mono_time=now,
    coordination_pose_refresher=ConsumerPoseRefresh(),
    coordination_pose_sm=Inputs(now, now - 30_552_118),
    pm=NS(send=lambda topic, msg: wire.append(topic)),
  )
  original = command()
  original.navigatorCalibration.valid = True
  original.navigatorCalibration.sourceMonoTime = now - 20_000_000
  original.navigatorCalibration.targetAngleDeg = 0.7
  before = original.to_dict()
  card.Car.controls_update(h, cs, original.as_reader())
  assert not h.last_actuators_output.navigatorLongitudinal.fault
  assert captured[0].navigatorLongitudinal.poseMonoTime == now - 30_552_118
  assert captured[0].navigatorCalibration.to_dict() == original.navigatorCalibration.to_dict()
  assert captured[0].actuators.to_dict() == original.actuators.to_dict()
  assert original.to_dict() == before and wire == ['sendcan']


import pytest


@pytest.mark.parametrize('opt_in,alpha,expected', [(True, True, True), (False, True, False), (True, False, False)])
def test_actual_constructor_only_creates_subscriber_for_enabled_navigator(monkeypatch, opt_in, alpha, expected):
  class Params:
    def get_bool(self, key):
      return opt_in if key == 'FordPrefLongitudinalCoordination' else key == 'OpenpilotEnabledToggle'

    def get(self, key, return_default=False):
      return 40 if key == 'InitialCruiseSpeedKph' and return_default else None

    def put(self, *args, **kwargs):
      pass

  services = []

  def sub(topics):
    services.append(topics)
    return NS(services=topics)

  monkeypatch.setattr(card, 'Params', Params)
  monkeypatch.setattr(card.messaging, 'SubMaster', sub)
  monkeypatch.setattr(card.messaging, 'sub_sock', lambda *a, **kw: None)
  monkeypatch.setattr(card.messaging, 'PubMaster', lambda *a, **kw: NS(sock={'sendcan': None}))
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.openpilotLongitudinalControl = alpha
  ci = CarInterface(cp)
  c = card.Car(CI=ci, RI=NS())
  assert (c.coordination_pose_refresher is not None) == expected
  assert (['deviceMotion', 'extrinsicsCalibration'] in services) == expected


def test_live_apply_clock_is_shared_by_tracking_and_controller(monkeypatch):
  from openpilot.selfdrive.car import tracking_context

  monkeypatch.setattr(card, 'REPLAY', False)
  monkeypatch.setattr(card, 'time', NS(monotonic=lambda: 1.0))
  tracking_times = []
  monkeypatch.setattr(tracking_context, 'feed_tracking_context', lambda *a, **kw: tracking_times.append(kw['now_ns']))
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.openpilotLongitudinalControl = True
  c = CarController(DBC[cp.carFingerprint], cp)
  c.longitudinal_coordination_enabled = True
  cs = structs.CarState(vEgo=8.0, vEgoRaw=8.0, canValid=True)
  state = NS(out=cs, buttons_stock_values=defaultdict(int), lkas_status_stock_values=defaultdict(int), acc_tja_status_stock_values=defaultdict(int))
  apply_times = []

  def apply(cc, ns):
    apply_times.append(ns)
    state.longitudinal_brake_torque = Torque(ns / 1e9, 0.0)
    return c.update(cc, state, ns)

  class SM(dict):
    valid = {'carControl': True, 'modelV2': True}
    logMonoTime = {'carControl': 995_000_000, 'modelV2': 990_000_000}

    def all_alive(self, *_):
      return True

    def all_checks(self, *_):
      return True

  times = iter([1_001_000_000, 1_002_000_000])
  h = NS(
    initialized_prev=True,
    sm=SM(modelV2=None),
    tracking_trial=object(),
    tracking_state_ns=990_000_000,
    longitudinal_coordination_enabled=True,
    CI=NS(CC=c, apply=apply),
    coordination_pose_refresher=ConsumerPoseRefresh(clock_ns=lambda: next(times)),
    coordination_pose_sm=Inputs(1_000_000_000, 970_000_000),
    pm=NS(send=lambda *_: None),
  )
  card.Car.controls_update(h, cs, command().as_reader())
  assert tracking_times == apply_times == [1_002_000_000]
  assert not h.last_actuators_output.navigatorLongitudinal.fault
