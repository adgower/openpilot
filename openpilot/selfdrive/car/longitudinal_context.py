"""Timestamped Navigator coordination transport and normal disengagement events."""
import time

from opendbc.car.ford.longitudinal_coordination import coordination_available, feedback_from_wire


class LongitudinalContext:
  def __init__(self, enabled=False):
    self.enabled = enabled
    self.epoch = time.monotonic_ns()
    self.sequence = 0
    self.owner = False
    self.fault = False
    self.feedback = None

  def prepare(self, CC, actuators_output, now_ns, *, output_valid):
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
    if CC.enabled and fresh and f.fault:
      self.fault = True
    if self.fault:
      CC.longActive = False


def stamp_pose(CC, *, sample_ns, valid, reference_valid=True):
  request = CC.navigatorLongitudinal
  if request.enabled:
    request.poseMonoTime = sample_ns
    request.poseValid = bool(valid)
    request.referenceValid = bool(reference_valid)


def feed_controller_context(controller, *, control_valid):
  controller.coordination_control_valid = bool(control_valid)


def coordination_event(CP, actuators_output):
  if coordination_available(CP) and actuators_output.navigatorLongitudinal.fault:
    from openpilot.cereal import log
    # Existing immediate takeover + no-entry presentation; detailed cause is
    # recorded in carOutput, without inventing an ECU-fault diagnosis.
    return log.OnroadEvent.EventName.controlsMismatch
  return None
