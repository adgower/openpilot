from openpilot.selfdrive.car.vision_curve_speed import AcknowledgedCurveSpeed, COMFORT_APPROACH_DECEL, MPH

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
          radar_ns = sm.logMonoTime["radarState"]
          radar_fresh = sm.valid["radarState"] and 0 <= now_ns - radar_ns <= 200_000_000
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
    override = state.gasPressed or state.brakePressed
    lane_change = state.leftBlinker or state.rightBlinker or self.lane_changing
    # Drop pending authority on model loss while engaged. A clean
    # disengagement clears the session even if the model is unavailable.
    valid &= not engaged or (fresh_model and not stock.nonAdaptive and str(state.gearShifter) == "drive" and 25 <= set_speed <= 95)
    return self.cruise.update(now_ns / 1e9, engaged=engaged, valid=valid, manual=manual, override=override, set_speed=set_speed,
                              speed=state.vEgo / MPH, accel=state.aEgo, steering_only=state.steeringPressed, lane_change=lane_change,
                              model=new_model if 25 * MPH <= state.vEgo <= 95 * MPH else None, comfort_hint_s=hint_s, cruise_engaged=stock.enabled)
