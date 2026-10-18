"""Read-only event-to-controller motion context; no cache or vehicle interfaces.

Pose source time, event arrival time and validity remain distinct. Calibration
is the existing installed PoseCalibrator, not raw CAN yaw or a fitted offset.
"""

import math
from types import SimpleNamespace

from openpilot.selfdrive.locationd.helpers import Pose, PoseCalibrator


class MotionContext:
  def __init__(self):
    self.calibrator = PoseCalibrator()
    self.calibration_event_valid = False
    self.latest = None
    self.generation = 0

  def on_event(self, e, *, received_ns=None):
    kind = e.which()
    published_ns = int(e.logMonoTime)
    received_ns = published_ns if received_ns is None else received_ns
    envelope_valid = bool(e.valid and 0 < published_ns <= received_ns)
    if kind == 'extrinsicsCalibration':
      previous = self.calibration_event_valid and self.calibrator.calib_valid
      self.calibrator.feed_extrinsics_calibration(e.extrinsicsCalibration)
      self.calibration_event_valid = envelope_valid
      current = self.calibration_event_valid and self.calibrator.calib_valid
      # Invalidate, never revive a previously transformed cached sample.
      # History generation also prevents bridging invalid/recovered calibration.
      if not current:
        self.generation += 1
        if self.latest is not None:
          self.latest['valid'] = False
      elif not previous:
        self.generation += 1
    elif kind == 'deviceMotion':
      dm = e.deviceMotion
      pose = self.calibrator.build_calibrated_pose(Pose.from_device_motion(dm))
      valid = bool(
        envelope_valid
        and self.calibration_event_valid
        and self.calibrator.calib_valid
        and dm.angularVelocityDevice.valid
        and dm.sensorsOK
        and dm.inputsOK
        and dm.posenetOK
      )
      self.latest = {
        "sample_ns": int(dm.timestamp),
        "received_ns": received_ns,
        "valid": valid,
        "yaw_rate": float(pose.angular_velocity.z),
        "yaw_std": float(pose.angular_velocity.z_std),
        "generation": self.generation,
      }

  def feed(self, strategy, *, now_ns):
    values = dict(self.latest or {"sample_ns": 0, "received_ns": 0, "valid": False, "yaw_rate": 0.0, "yaw_std": 0.0, "generation": self.generation})
    values['valid'] = bool(values['valid'] and self.calibration_event_valid and self.calibrator.calib_valid and values['generation'] == self.generation)
    strategy.set_motion(now_ns=now_ns, **values)

  def update_from_sm(self, sm, *, replay=False):
    # SubMaster conflates per topic. Source publication order determines which
    # transform applies; actual receive time remains a separate freshness input.
    topics = [s for s in ('extrinsicsCalibration', 'deviceMotion') if sm.updated.get(s, False) and sm.seen.get(s, False)]
    for service in sorted(topics, key=lambda s: sm.logMonoTime[s]):
      published = int(sm.logMonoTime[service])
      receipt = sm.recv_time.get(service, 0.0)
      received = published if replay else int(receipt * 1e9) if math.isfinite(receipt) and receipt > 0 else 0
      event = SimpleNamespace(logMonoTime=published, valid=sm.valid[service], which=lambda service=service: service, **{service: sm[service]})
      self.on_event(event, received_ns=received)


def feed_motion_context(strategy, context, sm, *, now_ns, state_valid, replay=False):
  if context is None:
    strategy.input_state_valid = False
    return
  context.update_from_sm(sm, replay=replay)
  strategy.input_state_valid = bool(state_valid)
  context.feed(strategy, now_ns=now_ns)
