"""Pure card-to-strategy seam; no sockets, vehicle access or transmission."""


def feed_tracking_context(strategy, *, now_ns, state_ns, control_ns, control_valid, model_ns, model_valid, model, state):
  # Missing/unknown context disables the *extra* correction only.
  lane_change = model is None or str(model.meta.laneChangeState) != 'off' or state.leftBlinker or state.rightBlinker
  strategy.set_context(
    now_ns=now_ns, state_ns=state_ns, control_ns=control_ns, model_ns=model_ns, model_valid=bool(control_valid and model_valid), lane_change=lane_change
  )


def create_tracking_trial(CP, *, enabled, roll_mapping_enabled=False, high_speed_enabled=False):
  from opendbc.car.ford.values import CAR, FordFlags

  if not enabled or CP.carFingerprint != CAR.FORD_EXPEDITION_MK4 or not CP.flags & FordFlags.PINION_CURVATURE or not CP.flags & FordFlags.LOW_SPEED_GAIN:
    return None
  if roll_mapping_enabled and high_speed_enabled:
    from opendbc.car.ford.lateral_angle_wheel_target import CalibratedWheelTargetReduction

    return CalibratedWheelTargetReduction(CP, enabled=True)
  if roll_mapping_enabled:
    from opendbc.car.ford.lateral_angle_roll_mapping import RollAwareMapping

    return RollAwareMapping(CP, enabled=True)
  from opendbc.car.ford.lateral_angle_tracking_budget import BudgetedTrackingPrototype

  return BudgetedTrackingPrototype(CP, enabled=True)


def high_speed_opt_in(params):
  # A previous compiled registry must preserve startup/default-off compatibility.
  from openpilot.common.params import UnknownKeyName

  try:
    return params.get_bool("FordPrefHighSpeedTracking")
  except UnknownKeyName:
    return False
