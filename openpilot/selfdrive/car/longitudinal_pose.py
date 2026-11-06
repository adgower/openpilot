"""Navigator expiry-boundary consumer pose recovery."""

import math
from dataclasses import dataclass
from openpilot.selfdrive.car.longitudinal_context import refresh_coordination_pose
from openpilot.selfdrive.locationd.helpers import PoseCalibrator


class FrozenInputs(dict):
  def __init__(self, sm):
    super().__init__({k: sm[k] for k in ['deviceMotion', 'extrinsicsCalibration'] if sm.seen.get(k, False)})
    self.seen = dict(sm.seen)
    self.valid = dict(sm.valid)
    self.logMonoTime = dict(sm.logMonoTime)

  def update(self, *args):
    pass


@dataclass(frozen=True)
class PoseRefreshFailure:
  initial_clock_ns: int
  selection_clock_ns: int
  final_clock_ns: int
  original_source_ns: int
  producer_ns: int
  selected_source_ns: int
  stage: str
  reason: str


class ConsumerPoseRefresh:
  def __init__(self, clock_ns=None):
    self.clock_ns = clock_ns
    self.apply_time_ns = 0
    self.calibrator = PoseCalibrator()
    self.last_producer_source = None
    self.last_granted_source = None
    self.epoch = None
    self.cached = None
    self.reason = 'baseline'
    self.failure_diagnostic = None

  def refresh(self, cc, sm, now_ns, *, enabled=True, speed=8.0):
    self.failure_diagnostic = None
    q = cc.navigatorLongitudinal
    active = enabled and q.enabled and cc.enabled and cc.longActive and speed >= 3 and str(cc.actuators.longControlState) == 'pid'
    if not active or self.clock_ns is None:
      return self._refresh(cc, sm, now_ns, enabled=enabled, speed=speed)
    selection_ns = self.clock_ns()
    self.apply_time_ns = selection_ns
    stage = 'selection_clock'
    if selection_ns < now_ns:
      result = cc
      reason = 'not_run'
    else:
      result = self._refresh(cc, sm, selection_ns, enabled=enabled, speed=speed)
      reason = self.reason
      previous_ns = self.apply_time_ns
      final_ns = self.clock_ns()
      self.apply_time_ns = final_ns
      rq = result.navigatorLongitudinal
      stage = 'canonical'
      if final_ns < previous_ns or not (0 < rq.poseMonoTime <= rq.producerMonoTime <= final_ns
                                       and final_ns - rq.producerMonoTime <= 100_000_000
                                       and final_ns - rq.poseMonoTime <= 100_000_000):
        stage = 'final_clock'
      elif rq.poseValid:
        return result
    if stage != 'canonical':
      r = result.as_builder()
      r.navigatorLongitudinal.poseValid = False
      result = r.as_reader()
      self.reason = 'clock_guard'
    self.failure_diagnostic = PoseRefreshFailure(
      now_ns, selection_ns, self.apply_time_ns, int(q.poseMonoTime), int(q.producerMonoTime),
      int(result.navigatorLongitudinal.poseMonoTime), stage, reason)
    return result

  def _refresh(self, cc, sm, now_ns, *, enabled=True, speed=8.0):
    self.apply_time_ns = now_ns
    self.reason = 'baseline'
    q = cc.navigatorLongitudinal
    if not enabled or not q.enabled or not cc.enabled or not cc.longActive or speed < 3 or str(cc.actuators.longControlState) != 'pid':
      if not cc.enabled:
        self.last_producer_source = None
        self.last_granted_source = None
        self.epoch = None
        self.cached = None
      return cc
    original_valid = (
      q.poseValid
      and len(cc.orientationNED) == 3
      and all(math.isfinite(x) for x in cc.orientationNED)
      and 0 < q.poseMonoTime <= q.producerMonoTime <= now_ns
      and now_ns - q.producerMonoTime <= 100_000_000
      and q.producerMonoTime - q.poseMonoTime <= 100_000_000
    )
    if not original_valid:
      return cc
    if self.epoch != q.epoch:
      self.epoch = q.epoch
      self.last_producer_source = None
      self.last_granted_source = None
      self.cached = None

    def invalid():
      r = cc.as_builder()
      r.navigatorLongitudinal.poseValid = False
      self.reason = 'invalid'
      return r.as_reader()

    if self.last_producer_source is not None and q.poseMonoTime < self.last_producer_source:
      return invalid()
    self.last_producer_source = q.poseMonoTime
    need = now_ns - q.poseMonoTime > 100_000_000 or (self.last_granted_source is not None and q.poseMonoTime < self.last_granted_source)
    if not need:
      self.last_granted_source = q.poseMonoTime
      return cc
    sm.update(0)
    sm = FrozenInputs(sm)  # One drain, one coherent immutable subscriber snapshot.
    if self.clock_ns is not None:
      applied_ns = self.clock_ns()
      self.apply_time_ns = applied_ns
      if applied_ns < now_ns or applied_ns - q.producerMonoTime > 100_000_000:
        return invalid()
      now_ns = applied_ns
    result = cc
    if all(sm.seen.get(k, False) for k in ['deviceMotion', 'extrinsicsCalibration']):
      pub = sm.logMonoTime['deviceMotion']
      source = sm['deviceMotion'].timestamp
      # An independent subscriber behind the producer must not invalidate it.
      if pub > q.producerMonoTime and (source > now_ns or source < q.poseMonoTime or not sm.valid['deviceMotion']):
        return invalid()
      if 0 < pub <= now_ns and now_ns - pub <= 100_000_000 and q.poseMonoTime <= source <= q.producerMonoTime:
        r = cc.as_builder()
        refresh_coordination_pose(r, sm, self.calibrator)
        sq = r.navigatorLongitudinal
        valid = (
          sq.poseValid
          and 0 < sm.logMonoTime['extrinsicsCalibration'] <= now_ns
          and q.poseMonoTime <= sq.poseMonoTime <= q.producerMonoTime
          and 0 <= now_ns - sq.poseMonoTime <= 100_000_000
          and (self.last_granted_source is None or sq.poseMonoTime >= self.last_granted_source)
        )
        if not valid:
          return invalid()
        result = r.as_reader()
        self.cached = (sq.poseMonoTime, list(r.orientationNED), list(r.angularVelocity))
        self.reason = 'refreshed'
      elif pub > now_ns:
        return invalid()
    # Repeated producer snapshots must not reverse a pose already refreshed by
    # this consumer. Reuse only the same measured source while still fresh.
    if self.cached is not None and result.navigatorLongitudinal.poseMonoTime < self.cached[0]:
      source, orientation, angular = self.cached
      if 0 <= now_ns - source <= 100_000_000 and source <= q.producerMonoTime:
        r = cc.as_builder()
        r.orientationNED = orientation
        r.angularVelocity = angular
        r.navigatorLongitudinal.poseMonoTime = source
        result = r.as_reader()
        self.reason = 'same_measured_source'
    if result.navigatorLongitudinal.poseValid and 0 <= now_ns - result.navigatorLongitudinal.poseMonoTime <= 100_000_000:
      self.last_granted_source = result.navigatorLongitudinal.poseMonoTime
    if self.clock_ns is not None:
      applied_ns = self.clock_ns()
      self.apply_time_ns = applied_ns
      if applied_ns < now_ns or applied_ns - q.producerMonoTime > 100_000_000 or not 0 <= applied_ns - result.navigatorLongitudinal.poseMonoTime <= 100_000_000:
        return invalid()
    return result
