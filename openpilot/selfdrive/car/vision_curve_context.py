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

SET_MINUS = "CcAslButtnSetDecPress"
OTHER_DRIVER_ASSIST_SWITCHES = ("AccButtnGapDecPress", "AccButtnGapIncPress", "AccButtnGapTogglePress", "TjaButtnOnOffPress")


def lower_switch_level(switches, samples, active_buttons, manual, action, direction):
  """Return an exclusive held or neutral level; a mixed batch proves neither."""
  level = switches.get(SET_MINUS)
  if level is None or any(sample != level for sample in samples.get(SET_MINUS, ())):
    return None
  if any(switches.get(key) or any(samples.get(key, ())) for key in OTHER_DRIVER_ASSIST_SWITCHES):
    return None
  if not level and not manual:
    return False
  if level and action == 'set' and direction == -1 and len(active_buttons) == 1 and SET_MINUS in active_buttons:
    return True
  return None


def supported(cp):
  return cp.brand == "ford" and cp.carFingerprint == "LINCOLN_NAVIGATOR_MK4" and bool(cp.flags & 1) and not cp.openpilotLongitudinalControl


def opening_load(now_ns, speed, requested_curvature, raw_load, compensated_load, motion):
  """Reconcile Ford yaw with independently calibrated gyro for one bend-exit step."""
  fallback = abs(raw_load)
  if (not motion['valid'] or not 0 < motion['sample_ns'] <= motion['received_ns'] <= now_ns or
      now_ns - motion['sample_ns'] > 100_000_000 or not math.isfinite(raw_load)):
    return fallback, False

  # Ford and the calibrated openpilot gyro use opposite turn directions.
  gyro_load = motion['yaw_rate'] * speed
  uncertainty = motion['yaw_std'] * speed
  requested = requested_curvature * speed * speed
  if not all(map(math.isfinite, (gyro_load, uncertainty, requested))) or not 0 <= motion['yaw_std'] <= .03:
    return fallback, False
  measured = max(abs(compensated_load), abs(gyro_load) + uncertainty, abs(requested))
  if abs(gyro_load + compensated_load) > .15:
    return max(fallback, measured), False
  return measured, True


class FinalRestoreLeadCheck:
  """After a closing lead, confirm clearance across distinct radar samples."""
  def __init__(self):
    self.waiting = False
    self.clear_since = 0
    self.last_sample = 0

  def allowed(self, radar, sample_ns, fresh, speed, set_speed):
    if not fresh:
      self.waiting = True
      self.clear_since = self.last_sample = 0
      return False
    closing = (speed + MPH < set_speed * MPH and any(lead.present and
               (not math.isfinite(lead.dRel) or not math.isfinite(lead.vRel) or
                (lead.dRel < 3.5 * speed and lead.vRel < 0.)) for lead in (radar.leadOne, radar.leadTwo)))
    if closing:
      self.waiting = True
      self.clear_since = 0
    elif self.waiting:
      if not self.clear_since or sample_ns < self.last_sample or sample_ns - self.last_sample > 200_000_000:
        self.clear_since = sample_ns
      elif sample_ns - self.clear_since >= 100_000_000:
        self.waiting = False
    self.last_sample = sample_ns
    return not self.waiting


class VisionCurveContext:
  def __init__(self, *, comfort=False, motion=None):
    self.motion = motion
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
    self.final_restore_lead = FinalRestoreLeadCheck()
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
    raw_lateral_accel = lateral_accel
    opening_motion_generation = None
    motion = self.motion.latest if self.motion is not None else None
    check_load = (motion is not None and self.cruise.geometry_trimmed and self.cruise.geometry_hold is not None and
                  engaged and fresh_control and valid and not (manual or override or steering_pressed or lane_change))
    if opening_lead_clear or check_load:
      comp_ns = parser.ts_nanos.get("BrakeSnData_3", {}).get("VehYawComp_W_Actl", 0)
      comp_dps = parser.vl.get("BrakeSnData_3", {}).get("VehYawComp_W_Actl", math.nan)
      # Ford reserves the two highest encoded yaw values for missing data and sensor faults.
      if 0 < comp_ns <= now_ns and now_ns - comp_ns <= 100_000_000 and -75. <= comp_dps <= 4093 * .03663 - 75.:
        opening_lead_lateral_accel = speed * math.radians(comp_dps)
        if check_load and motion['generation'] == self.motion.generation:
          lateral_accel, corroborated = opening_load(now_ns, speed, control.actuators.curvature, lateral_accel,
                                                    opening_lead_lateral_accel, motion)
          if corroborated:
            opening_motion_generation = motion['generation']
    allow_final_restore = True
    if self.comfort and self.cruise.ceiling is not None and 25 <= set_speed <= 95 and set_speed < self.cruise.ceiling:
      # The last increase has no immediate benefit while Ford is following a
      # slower lead below the current setting. Recheck on every fresh sample.
      allow_final_restore = self.final_restore_lead.allowed(sm["radarState"] if radar_fresh else None, radar_ns, radar_fresh, speed, set_speed)
    lower_switch = None
    set_samples = ()
    if not (cancel or cruise_control.resume) and (user_set_direction == -1 or self.cruise.lower_hold is not None):
      lower_switch = lower_switch_level(switches, switch_samples, active_buttons, manual, user_action, user_set_direction)
      set_samples = parser.vl_all.get("EngBrakeData", {}).get("Veh_V_DsplyCcSet", ())
    button = self.cruise.update(now_ns / 1e9, engaged=engaged, valid=valid, manual=manual, override=override, set_speed=set_speed,
                              speed=speed / MPH, accel=accel, steering_only=steering_pressed, lane_change=lane_change,
                              model=new_model if (25 * MPH <= speed <= 95 * MPH or
                                                  (self.comfort and self.forecast_clear)) else None,
                              lateral_accel=lateral_accel, opening_lead_lateral_accel=opening_lead_lateral_accel,
                              preview=preview, early_hint_inhibited=lead_present, opening_lead_clear=opening_lead_clear,
                              opening_motion_generation=opening_motion_generation,
                              opening_raw_lateral_accel=raw_lateral_accel, following_clear=following_clear, comfort_hint_s=hint_s,
                              cruise_engaged=stock_enabled, cruise_state_valid=cruise_state_valid,
                              forecast_mph=self.forecast_mph, forecast_clear=self.forecast_clear,
                              user_action=user_action, user_set_direction=user_set_direction, allow_final_restore=allow_final_restore,
                              switch_ns=switch_ns, lower_switch=lower_switch, set_ns=set_ns, set_samples=set_samples,
                              override_kind='brake' if brake_pressed else 'gas' if gas_pressed else None,
                              can_assert=self.can_assert(now_ns, switch_ns, manual),
                              forecast_authoritative=self.comfort,
                              forecast_reduce_mph=self.prediction.reduction_target if self.comfort else None,
                              forecast_available=(self.prediction.available and speed > .1 and
                                                  not getattr(state, 'standstill', speed <= .1)) if self.comfort else None)

    if button in ('SET-', 'SET+'):
      self.used_switch_ns = switch_ns
    return self.release_edge.update(button)
