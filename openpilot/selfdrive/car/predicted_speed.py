"""Model speed demand evaluated independently of cruise button readiness."""

import math

MPH = 0.44704


class PredictedSpeedDemand:
  """Qualify a model speed decrease without requiring a geometric curve.

  The caller supplies current model/driver validity and fresh no-lead radar.
  Lane probabilities are deliberately outside this observer. The robust forecast
  ignores isolated low future samples and uses modest immediate model deceleration
  as corroboration. Both criteria are configurable for replay comparisons.
  A missing cue has a bounded release dwell; invalid data clears immediately.
  ``data_valid`` covers model and numerical validity independently of permission.
  ``available`` additionally requires caller permission, no lead and no stop intent.
  """

  def __init__(self, *, raw_threshold=-0.1, robust=True, persistence=0.6, release_dwell=0.5):
    self.raw_threshold = raw_threshold
    self.robust = robust
    self.persistence = persistence
    self.release_dwell = release_dwell
    self.reset()

  def reset(self):
    self.data_valid = False
    self.available = False
    self.last_time = None
    self.detect_since = None
    self.release_since = None
    self.target = None
    self.qualified = False
    # Future-speed evidence is distinct from immediate acceleration. Crossing
    # the acceleration admission threshold does not prove the approach cleared.
    self.future_slowdown = False

  def _future_speed(self, model, speed_mph, *, require_loss=True):
    times, speeds = list(model.velocity.t), list(model.velocity.x)
    if len(times) != len(speeds) or len(times) < 3:
      return None
    previous = -math.inf
    future = []
    for horizon, speed in zip(times, speeds, strict=True):
      if not (math.isfinite(horizon) and math.isfinite(speed)) or speed < 0.0 or horizon <= previous:
        return None
      previous = horizon
      if 3.0 <= horizon <= 10.0:
        future.append((horizon, speed / MPH))
    if len(future) < 3:
      return None
    if not self.robust:
      return min(speed for _, speed in future)
    low = [(horizon, speed) for horizon, speed in future if speed <= speed_mph - 4.0]
    if require_loss and (len(low) < 3 or low[-1][0] - low[0][0] < 1.0):
      return math.inf
    ordered = sorted(speed for _, speed in future)
    rank = 0.2 * (len(ordered) - 1)
    index = int(rank)
    fraction = rank - index
    return ordered[index] + fraction * (ordered[index + 1] - ordered[index])

  def observe(self, now, speed_mph, ceiling, model, allowed, radar_clear):
    if not (model is not None and all(math.isfinite(value) for value in (now, speed_mph, ceiling)) and speed_mph >= 0.0 and ceiling >= 25.0):
      self.reset()
      return None
    if self.last_time is not None and now <= self.last_time:
      self.reset()
      return None
    if self.last_time is not None and now - self.last_time > 0.2:
      self.reset()
    self.last_time = now
    accel = model.action.desiredAcceleration
    future_speed = self._future_speed(model, speed_mph)
    if not math.isfinite(accel) or future_speed is None:
      self.reset()
      self.last_time = now
      return None
    if not allowed or not radar_clear or model.action.shouldStop:
      self.reset()
      self.last_time = now
      self.data_valid = True
      return None
    self.data_valid = True
    self.available = True
    self.future_slowdown = math.isfinite(future_speed) and speed_mph - future_speed >= 4.0
    cue = speed_mph - future_speed >= 4.0 and (self.raw_threshold is None or accel <= self.raw_threshold)
    if cue:
      self.release_since = None
      if self.detect_since is None:
        self.detect_since = now
      if self.qualified or now - self.detect_since >= self.persistence:
        self.qualified = True
        self.target = max(25.0, min(ceiling, future_speed))
    else:
      self.detect_since = None
      if self.qualified:
        if self.release_since is None:
          self.release_since = now
        if now - self.release_since >= self.release_dwell:
          self.qualified = False
          self.target = None
          self.release_since = None
    if self.target is not None:
      self.target = max(25.0, min(ceiling, self.target))
    return self.target
