from openpilot.selfdrive.car.multi_signal_speed import MultiSignalDemand
from openpilot.selfdrive.car.vision_curve_speed import COMFORT_APPROACH_DECEL, MPH
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
    self.prediction = MultiSignalDemand() if comfort else None
    self.forecast_mph = None
    self.forecast_clear = False
    self.forecast_data_valid = False
    self.shared_button_action = None
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
    valid = (state.canValid and fresh_control and not metric and
             0 < switch_ns <= now_ns and now_ns - switch_ns <= 250_000_000 and
             0 < set_ns <= now_ns and now_ns - set_ns <= 250_000_000)
    engaged = control.enabled and control.latActive and stock.enabled and not control.longActive
    manual = (control.cruiseControl.cancel or control.cruiseControl.resume or
              any(switches.get(name) or any(switch_samples.get(name, ())) for name in BUTTON_MASKS))
    def pressed(names):
      return any(switches.get(name) or any(switch_samples.get(name, ())) for name in names)
    shared_pressed = pressed(('CcAslButtnCnclResPress',))
    if not shared_pressed:
      self.shared_button_action = None
    elif self.shared_button_action is None:
      self.shared_button_action = 'cancel' if stock.enabled else 'resume'
    # Navigator's physical cancel/resume is the shared switch. Latch the action
    # at its press edge so the ensuing cruise-state transition cannot invert it.
    user_action = ('set' if pressed(('CcAslButtnSetDecPress', 'CcAslButtnSetIncPress', 'CcAslButtnSetPress',
                                    'CcAslButtnIndxDecPress', 'CcAslButtnIndxIncPress')) else
                   'cancel' if control.cruiseControl.cancel or pressed(('CcAslButtnCnclPress', 'CcAslButtnOffCnclPress')) else
                   'resume' if pressed(('CcAsllButtnResPress',)) else self.shared_button_action)
    override = state.gasPressed or state.brakePressed
    valid &= not engaged or (fresh_model and not stock.nonAdaptive and str(state.gearShifter) == "drive" and 25 <= set_speed <= 95)
    radar_fresh = False
    if self.comfort:
      radar_ns = sm.logMonoTime["radarState"]
      radar_fresh = sm.valid["radarState"] and 0 <= now_ns - radar_ns <= 200_000_000
    new_model = None
    hint_s = 0. if self.comfort else None
    if not (engaged and fresh_model):
      self.decel_since = None
    if model_ns != self.model_ns:
      if model_ns - self.model_ns > 200_000_000:
        self.decel_since = None
      self.model_ns = model_ns
      model = sm["modelV2"]
      self.lane_changing = str(model.meta.laneChangeState) != "off"
      if fresh_model:
        new_model = model
        if self.comfort and engaged:
          lead = sm["radarState"].leadOne if radar_fresh else None
          clear = lead is not None and (not lead.present or (
            lead.dRel >= 3.5 * state.vEgo and lead.dRel >= 20. * max(0., -lead.vRel)))
          forecast = max(((state.vEgo - v) / t for t, v in zip(model.velocity.t, model.velocity.x, strict=False)
                          if 3. <= t <= 10. and 0. <= v <= state.vEgo - 2 * MPH), default=0.) if clear and not model.action.shouldStop else 0.
          if forecast < .25:
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
                 not (manual or override or state.steeringPressed or lane_change) and
                 str(state.gearShifter) == "drive" and not stock.nonAdaptive and
                 25 <= set_speed <= 95 and 0 <= state.vEgo <= 95 * MPH)
      if user_action == 'set':
        self.prediction.reset()
      if new_model is not None:
        radar = sm["radarState"] if radar_fresh else None
        self.forecast_mph = self.prediction.observe(model_ns / 1e9, state.vEgo / MPH,
                                                  self.cruise.ceiling or set_speed,
                                                  new_model, radar, allowed, state.aEgo)
        self.forecast_data_valid = self.prediction.data_valid
        self.forecast_clear = self.prediction.clear
      if not allowed:
        self.forecast_mph = None
        self.forecast_clear = False
      if engaged:
        valid &= radar_fresh and self.forecast_data_valid
    # Drop pending authority on model loss while engaged. A clean
    # disengagement clears the session even if the model is unavailable.
    button = self.cruise.update(now_ns / 1e9, engaged=engaged, valid=valid, manual=manual, override=override, set_speed=set_speed,
                              speed=state.vEgo / MPH, accel=state.aEgo, steering_only=state.steeringPressed, lane_change=lane_change,
                              model=new_model if (25 * MPH <= state.vEgo <= 95 * MPH or
                                                  (self.comfort and self.forecast_clear)) else None,
                              comfort_hint_s=hint_s, cruise_engaged=stock.enabled,
                              forecast_mph=self.forecast_mph, forecast_clear=self.forecast_clear, user_action=user_action,
                              override_kind='brake' if state.brakePressed else 'gas' if state.gasPressed else None,
                              can_assert=self.can_assert(now_ns, switch_ns, manual),
                              forecast_authoritative=self.comfort,
                              forecast_reduce_mph=self.prediction.reduction_target if self.comfort else None,
                              forecast_available=(self.prediction.available and state.vEgo > .1 and
                                                  not getattr(state, 'standstill', state.vEgo <= .1)) if self.comfort else None)

    if button in ('SET-', 'SET+'):
      self.used_switch_ns = switch_ns
    if self.comfort:
      if not allowed:
        self.prediction.exit_since = None
      elif new_model is not None and self.cruise.ceiling is not None:
        self.prediction.release_ego_after_exit(now_ns / 1e9, state.vEgo / MPH, set_speed,
                                               self.cruise.ceiling, self.cruise.geometry_clear_since)
    return self.release_edge.update(button)
