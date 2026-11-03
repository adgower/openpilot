"""Timestamped Navigator coordination transport and normal disengagement events."""
import math
import time
import os

from opendbc.car.ford.longitudinal_retention import REQUEST_VERSION, speed_sample_ns as speed_sample_ns

from opendbc.car.ford.longitudinal_coordination import coordination_available, feedback_from_wire, allocator_feedback_valid

from opendbc.car.ford.longitudinal_precharge import PRECHARGE_PROTOCOL_VERSION


class LongitudinalContext:
  def __init__(self, enabled=False, *, retention_enabled=False):
    self.enabled = enabled
    self.retention_enabled = bool(enabled and retention_enabled)
    self.epoch = time.monotonic_ns()
    self.sequence = 0
    self.owner = False
    self.fault = False
    self.feedback = None

  def prepare(self, CC, actuators_output, now_ns, *, output_valid, target=None, target_ns=0, target_valid=False):
    if not self.enabled:
      return
    if not CC.enabled:
      self.fault = False
    elif not self.owner:
      self.epoch += 1
    self.owner = CC.enabled
    self.sequence += 1
    request = CC.navigatorLongitudinal
    request.enabled = True
    request.epoch = self.epoch
    request.sequence = self.sequence
    request.producerMonoTime = now_ns
    f = feedback_from_wire(actuators_output.navigatorLongitudinal)
    fresh = output_valid and f.epoch == self.epoch and 0 <= now_ns / 1e9 - f.t <= .100000001 and 0 < f.sequence <= self.sequence
    self.feedback = f if fresh else None
    stamp_allocator_request(CC, now_ns, target=target, target_ns=target_ns, target_valid=target_valid,
                            feedback=self.feedback, retention_enabled=self.retention_enabled)
    if CC.enabled and fresh and f.fault:
      self.fault = True
    if self.fault:
      CC.longActive = False


def stamp_allocator_request(CC, now_ns, *, target=None, target_ns=0, target_valid=False, feedback=None, retention_enabled=False):
  """Opt in with this control's consumed target; unknown peer feedback stays nominal."""
  request = CC.navigatorLongitudinal
  request.allocatorVersion = PRECHARGE_PROTOCOL_VERSION
  request.retentionVersion = REQUEST_VERSION if retention_enabled else 0
  target_finite = target is not None and math.isfinite(target)
  request.targetAcceleration = float(target) if target_finite else 0.
  request.targetMonoTime = max(0, target_ns)
  request.targetValid = bool(target_valid and target_finite and 0 < target_ns <= now_ns and now_ns - target_ns <= 100_000_000)
  if feedback is not None and feedback.valid and not allocator_feedback_valid(feedback):
    request.allocatorVersion = 0


def stamp_pose(CC, *, sample_ns, valid, reference_valid=True):
  request = CC.navigatorLongitudinal
  if request.enabled:
    request.poseMonoTime = sample_ns
    request.poseValid = bool(valid)
    request.referenceValid = bool(reference_valid)


def begin_coordination_cycle(pose_sm, *, replay_ns=None, clock_ns=None):
  """Freeze pose subscriptions before sampling this control's common clock."""
  if pose_sm is not None:
    pose_sm.update(0)
  return replay_ns if replay_ns is not None else (clock_ns or time.monotonic_ns)()


def refresh_coordination_pose(CC, sm, calibrator, primary_sm=None, *, drain=True):
  """Drain only pose inputs; retain source time and fail closed on unhealthy data."""
  import math
  from openpilot.selfdrive.locationd.helpers import Pose

  if drain:
    sm.update(0)
  # Independent subscribers can lag each other. Select by publication time,
  # not source time: a newly published reversed source must still be rejected.
  sources = {}
  for topic in ['deviceMotion', 'extrinsicsCalibration']:
    sources[topic] = max((source for source in (primary_sm, sm) if source is not None),
                         key=lambda source: source.logMonoTime[topic] if source.seen[topic] else -1)
  motion_sm, calibration_sm = sources['deviceMotion'], sources['extrinsicsCalibration']
  if calibration_sm.seen['extrinsicsCalibration']:
    calibrator.feed_extrinsics_calibration(calibration_sm['extrinsicsCalibration'])
  if not (motion_sm.seen['deviceMotion'] and calibration_sm.seen['extrinsicsCalibration']):
    stamp_pose(CC, sample_ns=0, valid=False, reference_valid=CC.navigatorLongitudinal.referenceValid)
    return

  dm = motion_sm['deviceMotion']
  pose = calibrator.build_calibrated_pose(Pose.from_device_motion(dm))
  orientation = pose.orientation.xyz.tolist()
  angular_velocity = pose.angular_velocity.xyz.tolist()
  CC.orientationNED = orientation
  CC.angularVelocity = angular_velocity
  # A late topic must never clear invalidity already seen by the primary producer.
  valid = bool(CC.navigatorLongitudinal.poseValid and motion_sm.valid['deviceMotion'] and calibration_sm.valid['extrinsicsCalibration']
    and calibrator.calib_valid and dm.orientationNED.valid and dm.sensorsOK and dm.inputsOK and dm.posenetOK
    and 0 < dm.timestamp <= CC.navigatorLongitudinal.producerMonoTime
    and all(math.isfinite(v) for v in orientation + angular_velocity))
  stamp_pose(CC, sample_ns=int(dm.timestamp), valid=valid, reference_valid=CC.navigatorLongitudinal.referenceValid)


def retention_startup_enabled(*, supported=False, environ=None):
  """Explicit developer launch selection, read once by each process; unset is OFF."""
  return bool(supported and (os.environ if environ is None else environ).get('NAVIGATOR_RECEIPT_RETENTION') == '1')


def history_release_startup_enabled(*, supported=False, environ=None):
  """Explicit developer launch selection, read once by card; unset is OFF."""
  return bool(supported and (os.environ if environ is None else environ).get('NAVIGATOR_HISTORY_RELEASE') == '1')


def feed_controller_context(controller, *, control_valid, state_ns=0, state_valid=False, speed_ns=0):
  controller.coordination_control_valid = bool(control_valid)
  controller.coordination_state_ns = state_ns if isinstance(state_ns, int) and 0 < state_ns < 2**64 else 0
  controller.coordination_speed_ns = speed_ns if isinstance(speed_ns, int) and 0 < speed_ns < 2**64 else 0
  controller.coordination_state_valid = bool(state_valid and controller.coordination_state_ns and controller.coordination_speed_ns)


def coordination_event(CP, actuators_output):
  if coordination_available(CP) and actuators_output.navigatorLongitudinal.fault:
    from openpilot.cereal import log
    # Existing immediate takeover + no-entry presentation; detailed cause is
    # recorded in carOutput, without inventing an ECU-fault diagnosis.
    return log.OnroadEvent.EventName.controlsMismatch
  return None
