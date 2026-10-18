"""Atomic control/calibration provenance. No sockets, params, or vehicle I/O."""
import math


def stamp_calibration(control, params, *, now_ns, source_ns, state_ns, speed, valid):
  packet = control.navigatorCalibration
  packet.valid = bool(valid and params.valid and params.sensorValid and params.angleOffsetValid
                      and params.steerRatioValid and params.stiffnessFactorValid)
  packet.sourceMonoTime = source_ns
  packet.stateMonoTime = state_ns
  packet.computedMonoTime = now_ns
  packet.targetSpeed = speed
  packet.roll = params.roll
  packet.steerRatio = params.steerRatio
  packet.stiffnessFactor = params.stiffnessFactor
  packet.angleOffsetDeg = params.angleOffsetDeg
  packet.requestedCurvature = control.actuators.curvature
  packet.targetAngleDeg = control.actuators.steeringAngleDeg


def feed_calibration_context(strategy, control, *, now_ns, control_ns, control_valid):
  packet = control.navigatorCalibration
  highwater = getattr(strategy, '_calibration_control_highwater', 0)
  ordered = 0 < control_ns <= now_ns and control_ns >= highwater
  if 0 < control_ns <= now_ns:
    strategy._calibration_control_highwater = max(highwater, control_ns)
  created = packet.computedMonoTime
  good = bool(control_valid and packet.valid and ordered
              and 0 < packet.sourceMonoTime <= created <= control_ns <= now_ns
              and 0 < packet.stateMonoTime <= created
              and 0 <= now_ns-control_ns <= 100_000_000
              and 0 <= now_ns-created <= 100_000_000
              and 0 <= created-packet.stateMonoTime <= 80_000_000
              and packet.requestedCurvature == control.actuators.curvature
              and packet.targetAngleDeg == control.actuators.steeringAngleDeg
              and math.isfinite(packet.targetSpeed) and packet.targetSpeed > .1)
  strategy.target_speed = float(packet.targetSpeed) if good else None
  strategy.set_calibration(now_ns=now_ns, sample_ns=packet.sourceMonoTime, valid=good,
                           roll=packet.roll, ratio=packet.steerRatio,
                           stiffness=packet.stiffnessFactor, offset_deg=packet.angleOffsetDeg)


def roll_mapping_opt_in(params):
  # Older compiled Params extensions do not know additive keys. Preserve startup
  # baseline until the registry is rebuilt; never make default-off a crash.
  from openpilot.common.params import UnknownKeyName
  try:
    return params.get_bool("FordPrefRollAwareMapping")
  except UnknownKeyName:
    return False
