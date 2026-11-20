import math

from openpilot.selfdrive.car.lead_speed import radar_is_aligned, stable_following
from openpilot.selfdrive.car.predicted_speed import AlphaEarlyHint
from openpilot.selfdrive.car.vision_curve_speed import COMFORT_APPROACH_DECEL, MPH, measure_curve_preview
from openpilot.selfdrive.car.vision_curve_recovery import AcknowledgedCurveSpeed

class ReleaseEdgeFilter:
  """Pass active commands unchanged, but send only one neutral per assertion."""
  def __init__(self):
    self.asserted = False

  def update(self, button):
    if button in ('SET-', 'SET+'):
      self.asserted = True
      return button
    if button == '' and self.asserted:
      self.asserted = False
      return ''
    return None


BUTTON_MASKS = (
  "CcAslButtnCnclPress", "CcAsllButtnResPress", "CcAslButtnCnclResPress", "CcAslButtnIndxDecPress", "CcAslButtnIndxIncPress",
  "CcAslButtnOffCnclPress", "CcAslButtnOnOffCncl", "CcAslButtnOnPress", "CcAslButtnResDecPress", "CcAslButtnResIncPress",
  "CcAslButtnSetDecPress", "CcAslButtnSetIncPress", "CcAslButtnSetPress", "CcButtnOffPress", "CcButtnOnOffCnclPress",
  "CcButtnOnOffPress", "CcButtnOnPress", "AslButtnOnOffCnclPress", "AslButtnOnOffPress",
)


def supported(cp):
  return cp.brand == "ford" and cp.carFingerprint == "LINCOLN_NAVIGATOR_MK4" and bool(cp.flags & 1) and not cp.openpilotLongitudinalControl


class VisionCurveContext:
  def __init__(self, *, comfort=False):
    self.cruise = AcknowledgedCurveSpeed()
    self.model_ns = 0
    self.lane_changing = True
    self.comfort = comfort
    self.decel_since = None
    self.prediction = AlphaEarlyHint() if comfort else None
    self.forecast_mph = None
    self.forecast_clear = False
    self.forecast_data_valid = False
    self.shared_button_action = None
    self.require_button_release = False
    self.release_edge = ReleaseEdgeFilter()
    self.used_switch_ns = 0
    self.latest_neutral_ns = 0
    self.previous_neutral_ns = 0

  def can_assert(self, now_ns, switch_ns, manual):
    if manual and 0 < switch_ns <= now_ns:
      self.used_switch_ns = max(self.used_switch_ns, switch_ns)
    elif self.latest_neutral_ns < switch_ns <= now_ns and now_ns - switch_ns <= 25_000_000:
      self.previous_neutral_ns, self.latest_neutral_ns = self.latest_neutral_ns, switch_ns
    # The native switch reported neutral about every 100 ms on these drives.
    # Select one tick early in that period. Timing is a delivery hypothesis.
    return (not manual and switch_ns > self.used_switch_ns and
            (not self.used_switch_ns or
             (self.previous_neutral_ns > self.used_switch_ns and
              75_000_000 <= switch_ns - self.previous_neutral_ns <= 125_000_000 and
              switch_ns - self.used_switch_ns >= 180_000_000)) and
            5_000_000 <= now_ns - switch_ns <= 25_000_000)

  def update(self, now_ns, sm, parser, state, control, metric):
    model_ns = sm.logMonoTime["modelV2"]
    fresh_model = sm.valid["modelV2"] and 0 <= now_ns - model_ns <= 200_000_000
    fresh_control = sm.valid["carControl"] and 0 <= now_ns - sm.logMonoTime["carControl"] <= 200_000_000
    switches = parser.vl["Steering_Data_FD1"]
    switch_samples = parser.vl_all["Steering_Data_FD1"]
    switch_ns = parser.ts_nanos["Steering_Data_FD1"]["CcAslButtnSetIncPress"]
    set_ns = parser.ts_nanos["EngBrakeData"]["Veh_V_DsplyCcSet"]
    stock = state.cruiseState
    set_speed = stock.speedCluster / MPH
    can_valid = state.canValid
    stock_enabled = stock.enabled
    speed = state.vEgo
    accel = state.aEgo
    gas_pressed = state.gasPressed
    brake_pressed = state.brakePressed
    steering_pressed = state.steeringPressed
    cruise_state_valid = can_valid and 0 < set_ns <= now_ns and now_ns - set_ns <= 250_000_000
    switch_fresh = 0 < switch_ns <= now_ns and now_ns - switch_ns <= 250_000_000
    valid = cruise_state_valid and switch_fresh and fresh_control and not metric
    engaged = control.enabled and control.latActive and stock_enabled and not control.longActive
    # Each switch can have more than one sample in a parser update. Check those
    # samples once, including a tap already released in the most recent frame.
    active_buttons = {name for name in BUTTON_MASKS if switches.get(name) or
                      (switch_samples.get(name) and any(switch_samples[name]))}
    physical = bool(active_buttons)
    cruise_control = control.cruiseControl
    cancel = cruise_control.cancel
    manual = cancel or cruise_control.resume or physical
    shared_pressed = 'CcAslButtnCnclResPress' in active_buttons
    # A held level across missing switch data cannot establish a new driver tap.
    if not (switch_fresh and can_valid) or (shared_pressed and not cruise_state_valid):
      self.require_button_release = True
    elif not physical:
      self.require_button_release = False
    if not shared_pressed or self.require_button_release:
      self.shared_button_action = None
    elif self.shared_button_action is None:
      self.shared_button_action = 'cancel' if stock_enabled else 'resume'
    # Latch physical cancel/resume at its press edge so the ensuing cruise-state
    # transition cannot invert it. Cancel always wins over another button.
    user_action = self.shared_button_action
    user_set_direction = 0
    if cancel or user_action == 'cancel' or 'CcAslButtnCnclPress' in active_buttons or 'CcAslButtnOffCnclPress' in active_buttons:
      user_action = 'cancel'
    elif self.require_button_release:
      user_action = None
    elif physical:
      inc = 'CcAslButtnSetIncPress' in active_buttons or 'CcAslButtnIndxIncPress' in active_buttons
      dec = 'CcAslButtnSetDecPress' in active_buttons or 'CcAslButtnIndxDecPress' in active_buttons
      if inc or dec or 'CcAslButtnSetPress' in active_buttons:
        user_action = 'set'
        user_set_direction = int(inc) - int(dec)
      elif 'CcAsllButtnResPress' in active_buttons:
        user_action = 'resume'
    override = gas_pressed or brake_pressed
    valid &= not engaged or (fresh_model and not stock.nonAdaptive and str(state.gearShifter) == "drive" and 25 <= set_speed <= 95)
    radar_fresh = False
    lateral_accel = math.inf
    if self.comfort:
      yaw_ns = parser.ts_nanos.get("Yaw_Data_FD1", {}).get("VehYaw_W_Actl", 0)
      if 0 < yaw_ns <= now_ns and now_ns - yaw_ns <= 100_000_000:
        yaw = parser.vl["Yaw_Data_FD1"]
        if yaw.get("VehYawWActl_D_Qf") == 3:
          lateral_accel = speed * yaw["VehYaw_W_Actl"]
      radar_ns = sm.logMonoTime["radarState"]
      radar_fresh = (sm.valid["radarState"] and 0 <= now_ns - radar_ns <= 200_000_000 and
                     radar_is_aligned(model_ns, sm["radarState"]))
    lead_present = radar_fresh and (sm["radarState"].leadOne.present or sm["radarState"].leadTwo.present)
    following_clear = False
    new_model = preview = None
    hint_s = 0. if self.comfort else None
    end_demand = brake_pressed or not stock_enabled or user_action in ('cancel', 'set')
    if end_demand or not (engaged and fresh_model):
      self.decel_since = None
    if model_ns != self.model_ns:
      if model_ns - self.model_ns > 200_000_000:
        self.decel_since = None
      self.model_ns = model_ns
      model = sm["modelV2"]
      self.lane_changing = str(model.meta.laneChangeState) != "off"
      if fresh_model:
        new_model = model
        if self.comfort and engaged and not end_demand:
          lead = sm["radarState"].leadOne if radar_fresh else None
          clear = lead is not None and (not lead.present or (
            lead.dRel >= 3.5 * speed and lead.dRel >= 20. * max(0., -lead.vRel)))
          action = model.action
          forecast = 0.
          if clear and not action.shouldStop:
            velocity = model.velocity
            forecast = max(((speed - v) / t for t, v in zip(velocity.t, velocity.x, strict=False)
                            if 3. <= t <= 10. and 0. <= v <= speed - 2 * MPH), default=0.)
          if action.desiredAcceleration > -.1 or forecast < .25:
            self.decel_since = None
          else:
            if self.decel_since is None:
              self.decel_since = model_ns
            if model_ns - self.decel_since >= 400_000_000:
              hint_s = min(2., forecast / COMFORT_APPROACH_DECEL)
    lane_change = state.leftBlinker or state.rightBlinker or self.lane_changing
    if self.comfort:
      # Permission does not include the admission speed: owned caps must be
      # able to clear below 40 mph. The observer gates new reductions itself.
      allowed = (valid and engaged and fresh_model and radar_fresh and
                 not (manual or override or steering_pressed or lane_change) and
                 0 <= speed <= 95 * MPH)
      if new_model is not None:
        preview = measure_curve_preview(new_model)
        allowed &= preview[1]
      if new_model is not None:
        radar = sm["radarState"] if radar_fresh else None
        self.forecast_mph = self.prediction.observe(model_ns / 1e9, speed / MPH,
                                                  self.cruise.ceiling or set_speed,
                                                  new_model, radar, allowed, accel)
        self.forecast_data_valid = bool(preview and preview[1] and self.prediction.data_valid)
        self.forecast_clear = self.prediction.clear
      if lead_present:
        # A new lead can arrive between model frames. Ford owns following.
        self.prediction.reduction_target = self.cruise.ceiling or set_speed
        self.forecast_clear = False
        # Following is still Ford's job when the bend that we slowed for has
        # ended. Only qualify a geometry-owned return with healthy fresh leads.
        if allowed and self.prediction.available and self.prediction.target is None:
          action = sm["modelV2"].action
          following_clear = (not action.shouldStop and action.desiredAcceleration > -.1
                             and self.prediction.ego.detect_since is None and not self.prediction.ego.future_slowdown
                             and stable_following(sm["radarState"], speed))
      if not allowed:
        # A brake, cancellation or new selection ends the old absolute speed hint.
        # Ordinary transient overrides may still retain its recovery target.
        if end_demand and self.prediction.target is not None:
          self.prediction.reset()
        else:
          self.prediction.inhibit()
        self.forecast_mph = None
        self.forecast_clear = False
      if engaged:
        valid &= radar_fresh and self.forecast_data_valid
    # Drop pending authority on model loss while engaged. A clean
    # disengagement clears the session even if the model is unavailable.
    opening_lead_clear = (following_clear and all(not lead.present or (lead.dRel >= max(55., 3. * speed) and lead.vRel >= 1.5)
                                  for lead in (sm["radarState"].leadOne, sm["radarState"].leadTwo)))
    opening_lead_lateral_accel = math.inf
    if opening_lead_clear:
      comp_ns = parser.ts_nanos.get("BrakeSnData_3", {}).get("VehYawComp_W_Actl", 0)
      comp_dps = parser.vl.get("BrakeSnData_3", {}).get("VehYawComp_W_Actl", math.nan)
      # Ford marks the two highest unsigned words as no data and fault.
      if 0 < comp_ns <= now_ns and now_ns - comp_ns <= 100_000_000 and -75. <= comp_dps <= 4093 * .03663 - 75.:
        opening_lead_lateral_accel = speed * math.radians(comp_dps)
    button = self.cruise.update(now_ns / 1e9, engaged=engaged, valid=valid, manual=manual, override=override, set_speed=set_speed,
                              speed=speed / MPH, accel=accel, steering_only=steering_pressed, lane_change=lane_change,
                              model=new_model if (25 * MPH <= speed <= 95 * MPH or
                                                  (self.comfort and self.forecast_clear)) else None,
                              lateral_accel=lateral_accel, opening_lead_lateral_accel=opening_lead_lateral_accel,
                              preview=preview, early_hint_inhibited=lead_present, opening_lead_clear=opening_lead_clear, following_clear=following_clear, comfort_hint_s=hint_s, cruise_engaged=stock_enabled, cruise_state_valid=cruise_state_valid,
                              forecast_mph=self.forecast_mph, forecast_clear=self.forecast_clear, user_action=user_action, user_set_direction=user_set_direction,
                              override_kind='brake' if brake_pressed else 'gas' if gas_pressed else None,
                              can_assert=self.can_assert(now_ns, switch_ns, manual),
                              forecast_authoritative=self.comfort,
                              forecast_reduce_mph=self.prediction.reduction_target if self.comfort else None,
                              forecast_available=(self.prediction.available and speed > .1 and
                                                  not getattr(state, 'standstill', speed <= .1)) if self.comfort else None)

    if button in ('SET-', 'SET+'):
      self.used_switch_ns = switch_ns
    return self.release_edge.update(button)
