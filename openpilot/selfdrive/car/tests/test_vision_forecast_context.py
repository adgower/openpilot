from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.vision_curve_context import VisionCurveContext
from openpilot.selfdrive.car.vision_curve_recovery import AcknowledgedCurveSpeed
from openpilot.selfdrive.car.vision_curve_speed import MPH


class Messages(dict):
  def __init__(self):
    super().__init__()
    self.valid = dict.fromkeys(("modelV2", "carControl", "radarState"), True)
    self.logMonoTime = dict.fromkeys(self.valid, 0)


class CaptureCruise:
  def __init__(self, controller=None):
    self.controller = controller
    self.calls = []
    self.last_action = None

  @property
  def ceiling(self):
    return self.controller.ceiling if self.controller is not None else None

  @property
  def geometry_clear_since(self):
    return self.controller.geometry_clear_since if self.controller is not None else None

  def update(self, now, **kwargs):
    self.calls.append((now, kwargs))
    self.last_action = self.controller.update(now, **kwargs) if self.controller is not None else None
    return self.last_action


class ContextFixture:
  def __init__(self, *, comfort=True):
    self.context = VisionCurveContext(comfort=comfort)
    self.context.cruise = CaptureCruise()
    self.sm = Messages()
    self.sm["modelV2"] = SimpleNamespace(
      velocity=SimpleNamespace(t=[0.0, 1.0, 3.0, 5.0, 7.0, 10.0], x=[55.0 * MPH] * 6),
      action=SimpleNamespace(desiredAcceleration=-0.3, shouldStop=False),
      meta=SimpleNamespace(laneChangeState="off"),
      laneLineProbs=[0.1] * 4,
      orientationRate=SimpleNamespace(t=[0.0, 1.0, 3.0, 5.0, 7.0, 10.0], z=[0.0] * 6),
      position=SimpleNamespace(x=[0.0, 40.0, 80.0, 120.0, 160.0, 200.0], y=[0.0] * 6),
      laneLines=[SimpleNamespace(x=[0.0, 40.0, 80.0, 120.0, 160.0, 200.0], y=[offset] * 6) for offset in (-5.0, -1.8, 1.8, 5.0)],
      roadEdgeStds=[0.5, 0.5],
    )
    self.sm["radarState"] = SimpleNamespace(
      leadOne=SimpleNamespace(present=False), leadTwo=SimpleNamespace(present=False),
      mdMonoTime=0, radarErrors=SimpleNamespace(canError=False, radarFault=False, wrongConfig=False, radarUnavailableTemporary=False),
    )
    self.state = SimpleNamespace(
      canValid=True,
      vEgo=64.0 * MPH,
      aEgo=0.0,
      gasPressed=False,
      brakePressed=False,
      steeringPressed=False,
      leftBlinker=False,
      rightBlinker=False,
      gearShifter="drive",
      cruiseState=SimpleNamespace(speedCluster=65.0 * MPH, enabled=True, nonAdaptive=False),
    )
    self.control = SimpleNamespace(enabled=True, latActive=True, longActive=False, cruiseControl=SimpleNamespace(cancel=False, resume=False))
    self.parser = SimpleNamespace(vl={"Steering_Data_FD1": {}}, vl_all={"Steering_Data_FD1": {}}, ts_nanos={"Steering_Data_FD1": {}, "EngBrakeData": {}})

  def update(self, seconds, *, new_model=True, radar_fresh=True, metric=False):
    now = round((10.0 + seconds) * 1e9)
    if new_model:
      self.sm.logMonoTime["modelV2"] = now
    self.sm.logMonoTime["carControl"] = now
    if radar_fresh:
      self.sm.logMonoTime["radarState"] = now
      self.sm["radarState"].mdMonoTime = self.sm.logMonoTime["modelV2"]
    # Ford's native cruise switch updates at 10 Hz, independently of the model callback.
    switch_ns = (now + 10_000_000) // 100_000_000 * 100_000_000 - 10_000_000
    self.parser.ts_nanos["Steering_Data_FD1"]["CcAslButtnSetIncPress"] = switch_ns
    self.parser.ts_nanos["EngBrakeData"]["Veh_V_DsplyCcSet"] = now
    self.context.update(now, self.sm, self.parser, self.state, self.control, metric)
    return self.context.cruise.calls[-1][1]

  def qualify(self):
    for index in range(16):
      values = self.update(index * 0.05)
    return values


def test_forecast_qualifies_before_curve_with_low_lane_confidence():
  fixture = ContextFixture()
  values = fixture.qualify()
  assert values["forecast_mph"] == pytest.approx(63.0)  # Two-mph hint from the original 65 mph setting.
  assert not values["forecast_clear"]


@pytest.mark.parametrize("block", [None, "bend", "weak_forecast", "radar_error", "missing_yaw", "nan_yaw", "bad_time"])
def test_recover_original_setpoint_when_the_reduced_cruise_limits_the_forecast(block):
  f = ContextFixture()
  c = AcknowledgedCurveSpeed()
  f.context.cruise = CaptureCruise(c)
  radar = f.sm["radarState"]
  radar.radarErrors = SimpleNamespace(canError=False, radarFault=False, wrongConfig=False, radarUnavailableTemporary=False)
  model = f.sm["modelV2"]
  model.laneLineProbs = [.1, .9, .9, .1]
  f.state.cruiseState.speedCluster = 70 * MPH
  f.state.vEgo = 69 * MPH

  def frame(i):
    radar.mdMonoTime = round((10 + i * .05) * 1e9)
    f.update(i * .05)
    button = f.context.cruise.last_action
    if button in ("SET-", "SET+"):
      f.state.cruiseState.speedCluster += MPH if button == "SET+" else -MPH
    return button

  for i in range(35):
    frame(i)
  lower = round(f.state.cruiseState.speedCluster / MPH)
  assert 55 <= lower < 69
  f.state.vEgo = (lower - 1) * MPH
  model.velocity.x = [lower * MPH] * len(model.velocity.t)
  model.velocity.x[-1] = (lower + .5) * MPH
  model.action.desiredAcceleration = .15
  if block == "bend":
    model.orientationRate.z = [.12] * len(model.velocity.t)
  elif block == "weak_forecast":
    model.action.desiredAcceleration = 0.
  elif block == "radar_error":
    radar.radarErrors.radarFault = True
  elif block == "missing_yaw":
    model.orientationRate.z = []
  elif block == "nan_yaw":
    model.orientationRate.z[3] = float('nan')
  elif block == "bad_time":
    model.orientationRate.t[3] = model.orientationRate.t[2]
  buttons = [frame(i) for i in range(35, 140)]
  if block:
    assert "SET+" not in buttons
  else:
    assert round(f.state.cruiseState.speedCluster / MPH) == 70
    assert c.transport.pending == 0


def test_native_switch_needs_an_observed_neutral_between_automatic_presses():
  ctx = VisionCurveContext(comfort=True)
  start = 1_000_000_000
  assert ctx.can_assert(start + 10_000_000, start, False)
  ctx.used_switch_ns = start
  # A skipped native packet cannot count as the intervening neutral.
  assert not ctx.can_assert(start + 210_000_000, start + 200_000_000, False)
  assert ctx.can_assert(start + 310_000_000, start + 300_000_000, False)
  # A physical switch press takes priority, including across repeated callbacks.
  assert not ctx.can_assert(start + 310_000_000, start + 300_000_000, True)
  assert not ctx.can_assert(start + 315_000_000, start + 300_000_000, False)


def test_repeated_control_callbacks_do_not_reobserve_same_model():
  fixture = ContextFixture()
  fixture.qualify()
  for index in range(1, 5):
    values = fixture.update(0.75 + index * 0.01, new_model=False)
    assert values["forecast_mph"] == pytest.approx(63.0)  # Two-mph hint from the original 65 mph setting.


@pytest.mark.parametrize("attribute", ["gasPressed", "brakePressed", "steeringPressed", "leftBlinker", "rightBlinker"])
def test_driver_input_clears_cached_forecast_between_model_frames(attribute):
  fixture = ContextFixture()
  fixture.qualify()
  setattr(fixture.state, attribute, True)
  values = fixture.update(0.76, new_model=False)
  assert values["forecast_mph"] is None
  assert not values["forecast_clear"]


def test_cruise_switch_sample_clears_forecast_between_model_frames():
  fixture = ContextFixture()
  fixture.qualify()
  fixture.parser.vl_all["Steering_Data_FD1"]["CcAslButtnSetIncPress"] = [1]
  values = fixture.update(0.76, new_model=False)
  assert values["manual"]
  assert values["forecast_mph"] is None
  assert not values["forecast_clear"]


@pytest.mark.parametrize("source", ["modelV2", "radarState"])
def test_stale_channel_revokes_cached_forecast_between_model_frames(source):
  fixture = ContextFixture()
  fixture.qualify()
  fixture.sm.valid[source] = False
  values = fixture.update(0.76, new_model=False)
  assert values["forecast_mph"] is None
  assert not values["forecast_clear"]
  assert not values["valid"]


def test_fresh_lead_arrival_preserves_ego_forecast():
  fixture = ContextFixture()
  fixture.qualify()
  fixture.sm["radarState"].leadTwo.present = True
  values = fixture.update(0.76, new_model=False)
  assert values["forecast_mph"] == pytest.approx(63.0)  # Two-mph hint from the original 65 mph setting.
  assert not values["forecast_clear"]
  assert values["valid"]


def test_invalid_forecast_cannot_be_clear_evidence():
  fixture = ContextFixture()
  fixture.qualify()
  fixture.sm["modelV2"].velocity.x[3] = float("nan")
  values = fixture.update(0.8)
  assert values["forecast_mph"] is None
  assert not values["forecast_clear"]
  assert not values["valid"]


def test_model_gap_requires_new_forecast_persistence():
  fixture = ContextFixture()
  fixture.qualify()
  values = fixture.update(1.1)
  assert values["forecast_mph"] is None
  assert not values["forecast_clear"]
  for index in range(23, 36):
    values = fixture.update(index * 0.05)
  assert values["forecast_mph"] == pytest.approx(63.0)  # Two-mph hint from the original 65 mph setting.


def test_stop_intent_suspends_forecast_without_invalidating_data():
  fixture = ContextFixture()
  fixture.qualify()
  fixture.sm["modelV2"].action.shouldStop = True
  values = fixture.update(0.8)
  assert values["forecast_mph"] is None
  assert not values["forecast_clear"]
  assert values["valid"]


def test_forecast_owned_reduction_holds_during_stop_and_restores_after_valid_clear():
  fixture = ContextFixture()
  controller = AcknowledgedCurveSpeed()
  fixture.context.cruise = CaptureCruise(controller)
  presses = []
  for index in range(15):
    fixture.update(index * 0.05)
    if fixture.context.cruise.last_action:
      presses.append(fixture.context.cruise.last_action)
      fixture.state.cruiseState.speedCluster -= MPH
  assert presses == ["SET-"]
  assert fixture.state.cruiseState.speedCluster == pytest.approx(64.0 * MPH)
  assert controller.forecast_owned
  fixture.sm["modelV2"].action.shouldStop = True
  for index in range(15, 41):
    values = fixture.update(index * 0.05)
    assert values["valid"]
    assert fixture.context.cruise.last_action not in ("SET-", "SET+")
    assert not controller.fault
  fixture.sm["modelV2"].action.shouldStop = False
  fixture.sm["modelV2"].velocity.x = [65.0 * MPH] * 6
  for index in range(41, 80):
    fixture.update(index * 0.05)
    if fixture.context.cruise.last_action:
      presses.append(fixture.context.cruise.last_action)
      fixture.state.cruiseState.speedCluster += MPH
  assert presses == ["SET-", "SET+"]
  assert fixture.state.cruiseState.speedCluster == pytest.approx(65.0 * MPH)
  assert not controller.forecast_owned
  assert not controller.fault


@pytest.mark.parametrize("pedal", ["brakePressed", "gasPressed"])
def test_physical_pedal_pauses_without_fault_and_brake_requires_resume(pedal):
  fixture = ContextFixture()
  controller = AcknowledgedCurveSpeed()
  fixture.context.cruise = CaptureCruise(controller)
  for index in range(15):
    fixture.update(index * 0.05)
    if fixture.context.cruise.last_action == "SET-":
      fixture.state.cruiseState.speedCluster -= MPH
  setattr(fixture.state, pedal, True)
  if pedal == "brakePressed":
    fixture.state.cruiseState.enabled = False
  for index in range(15, 20):
    fixture.update(index * .05)
    assert fixture.context.cruise.last_action not in ("SET-", "SET+")
  assert not controller.fault
  assert controller.ceiling == 65
  assert controller.suspended == (pedal == "brakePressed")
  setattr(fixture.state, pedal, False)
  fixture.sm["modelV2"].velocity.x = [65.0 * MPH] * 6
  for index in range(20, 50):
    fixture.update(index * .05)
    if pedal == "brakePressed":
      assert fixture.context.cruise.last_action not in ("SET-", "SET+")
    elif fixture.context.cruise.last_action == "SET+":
      fixture.state.cruiseState.speedCluster += MPH
  if pedal == "brakePressed":
    fixture.parser.vl["Steering_Data_FD1"]["CcAslButtnCnclResPress"] = 1
    fixture.update(2.5)
    fixture.state.cruiseState.enabled = True
    fixture.update(2.51, new_model=False)
    fixture.parser.vl["Steering_Data_FD1"]["CcAslButtnCnclResPress"] = 0
    for index in range(51, 85):
      fixture.update(index * .05)
      if fixture.context.cruise.last_action == "SET+":
        fixture.state.cruiseState.speedCluster += MPH
  assert fixture.state.cruiseState.speedCluster == pytest.approx(65 * MPH)
  assert not controller.fault


def test_forecast_clear_requires_release_dwell_and_no_pending_admission():
  fixture = ContextFixture()
  fixture.qualify()
  fixture.sm["modelV2"].velocity.x = [65.0 * MPH] * 6
  assert not fixture.update(0.8)["forecast_clear"]
  for index in range(17, 28):
    values = fixture.update(index * 0.05)
  assert values["forecast_clear"]
  assert values["forecast_mph"] is None


@pytest.mark.parametrize("reason", ["comfort_off", "alpha", "lateral_off", "cruise_off", "metric", "low_speed"])
def test_forecast_requires_supported_enabled_oem_context(reason):
  fixture = ContextFixture(comfort=reason != "comfort_off")
  if reason == "alpha":
    fixture.control.longActive = True
  if reason == "lateral_off":
    fixture.control.latActive = False
  if reason == "cruise_off":
    fixture.state.cruiseState.enabled = False
  if reason == "low_speed":
    fixture.state.vEgo = 35.0 * MPH
  for index in range(20):
    values = fixture.update(index * 0.05, metric=reason == "metric")
    assert values.get("forecast_mph") is None
    if reason != "low_speed":
      assert not values.get("forecast_clear", False)
