"""Offline, solver-free lead cap hypothesis; never longitudinal actuation.

Policy assumptions: 1.5 s headway, 6 m stationary gap. Two slots are alternative
observations, never independent votes. Horizon is exactly the six model samples.
"""
import math
from bisect import bisect_left

MPH = .44704


class LeadSpeedDemand:
  def __init__(self, *, persistence=.6, release_dwell=.5, headway=1.5, standstill_gap=6.):
    self.persistence, self.release_dwell = persistence, release_dwell
    self.headway, self.standstill_gap = headway, standstill_gap
    self.reset()

  def reset(self):
    self.state, self.reason, self.target, self.data_valid = 'unavailable', 'reset', None, False
    self.last_time = self.detect_since = self.release_since = None
    self.identity = None
    self.last_distance = None

  def _unavailable(self, reason, now=None):
    self.reset()
    self.reason, self.last_time = reason, now
    return None

  @staticmethod
  def _payload(value, field):
    return getattr(value, field, value)

  @staticmethod
  def _interp(x, xs, ys):
    i = bisect_left(xs, x)
    if i == 0:
      return ys[0]
    if i == len(xs):
      return ys[-1]
    f = (x-xs[i-1])/(xs[i]-xs[i-1])
    return ys[i-1] + f*(ys[i]-ys[i-1])

  def observe(self, now, speed_mph, ceiling, model, radar, allowed, ego_accel=0.):
    try:
      if not all(math.isfinite(v) for v in (now, speed_mph, ceiling, ego_accel)) or speed_mph < 0 or ceiling < 25:
        return self._unavailable('invalid_vehicle')
      if self.last_time is not None and now <= self.last_time:
        return self._unavailable('nonmonotonic_time', now)
      if self.last_time is not None and now-self.last_time > .2+1e-6:
        self.reset()
      self.last_time = now
      if model is None or radar is None:
        return self._unavailable('missing_source', now)
      m, r = self._payload(model, 'modelV2'), self._payload(radar, 'radarState')
      # Caller guarantees fresh valid envelopes; now is model logMonoTime.
      # mdMonoTime independently checks radar/model association freshness.
      mn = int(now*1e9)
      if not math.isfinite(r.mdMonoTime) or r.mdMonoTime <= 0 or not 0 <= mn-r.mdMonoTime <= 200_000_000 or any((r.radarErrors.canError, r.radarErrors.radarFault,
                                                                                                 r.radarErrors.wrongConfig, r.radarErrors.radarUnavailableTemporary)):
        return self._unavailable('source_alignment', now)
      present = r.leadOne.present or r.leadTwo.present
      if present:
        px, py = list(m.position.x), list(m.position.y)
        if not 2 <= len(px) <= 33 or len(px) != len(py) or not all(math.isfinite(v) for v in px+py) or any(b <= a for a,b in zip(px, px[1:])):
          return self._unavailable('invalid_path', now)
        if len(m.leadsV3) < 2:
          return self._unavailable('missing_slots', now)
      candidates = []
      for slot, lead in enumerate((r.leadOne, r.leadTwo)):
        if not lead.present:
          continue
        ml = m.leadsV3[slot]
        ts, xs, ys, vs = [list(getattr(ml, name)) for name in ('t','x','y','v')]
        if any(len(a) != 6 for a in (ts,xs,ys,vs)) or not all(math.isfinite(v) for v in ts+xs+ys+vs+[lead.dRel, lead.vLead, lead.modelProb, ml.prob]):
          return self._unavailable('invalid_lead', now)
        if any(abs(t-2*i) > .01 for i,t in enumerate(ts)) or lead.dRel <= 0 or lead.vLead < 0:
          return self._unavailable('invalid_trajectory', now)
        if ml.prob < .7 or lead.modelProb < .7:
          return self._unavailable('low_confidence', now)
        if abs((xs[0]-1.52)-lead.dRel) > max(5., .25*lead.dRel) or abs(vs[0]-lead.vLead) > 8.:
          return self._unavailable('association_mismatch', now)
        # Strong departure from the predicted ego corridor is positive clear
        # evidence, including camera-only leads taking another fork. Do not
        # extrapolate the path laterally beyond its measured x support.
        supported = [i for i,x in enumerate(xs) if px[0] <= x <= px[-1]]
        offsets = [abs(ys[i]-self._interp(xs[i],px,py)) for i in supported]
        if not offsets:
          return self._unavailable('path_unobserved', now)
        if offsets[0] > 2.2 or (len(offsets) >= 2 and any(v > 3. for v in offsets[1:])):
          continue
        if len(offsets) < 3 or 0 not in supported:
          return self._unavailable('path_horizon_short', now)
        anchored_x = [lead.dRel+x-xs[0] for x in xs]
        anchored_v = [lead.vLead+v-vs[0] for v in vs]
        if any(b < a for a,b in zip(anchored_x, anchored_x[1:])) or any(v < 0 for v in anchored_v):
          return self._unavailable('implausible_trajectory', now)
        # Constant-ego scenario: solve xlead - u*t >= gap + headway*u.
        # This is a research cap, not a braking-distance safety certificate.
        cap = min((anchored_x[i]-self.standstill_gap)/(ts[i]+self.headway) for i in supported if i > 0)
        cap = max(25., min(ceiling, cap/MPH))
        needed = max(0., (speed_mph-cap)*MPH/4.)
        identity = (slot, bool(lead.radar), getattr(lead,'radarTrackId',-1))
        candidates.append((cap, needed, identity, lead.dRel))
      self.data_valid = True
      if not allowed:
        return self._unavailable('not_allowed', now)
      best = min(candidates, default=None)
      cue = best is not None and best[0] <= ceiling-2.
      # A cap above actual speed can still prevent an unwanted acceleration
      # toward the driver's higher ceiling when following traffic.
      # Suppress only an additional slowing request already covered by OEM.
      # Existing observed OEM deceleration already satisfies this scenario.
      if cue and self.target is None and best[1] > .15 and ego_accel <= -best[1]:
        cue = False
      if cue:
        cap, _, identity, distance = best
        if identity != self.identity or (self.last_distance is not None and abs(distance-self.last_distance) > 8.):
          self.detect_since = now
          self.target = None
        self.identity, self.last_distance = identity, distance
        self.release_since = None
        if self.detect_since is None:
          self.detect_since = now
        if now-self.detect_since >= self.persistence-1e-6:
          self.target, self.state, self.reason = cap, 'active', 'lead_gap_forecast'
        else:
          self.state, self.reason = 'unavailable', 'qualifying'
      else:
        self.detect_since = None
        self.identity = self.last_distance = None
        if self.release_since is None:
          self.release_since = now
        if now-self.release_since >= self.release_dwell-1e-6:
          self.target, self.state, self.reason = None, 'clear', 'fresh_no_additional_demand'
        elif self.target is not None:
          self.state, self.reason = 'active', 'release_dwell'
        else:
          self.state, self.reason = 'unavailable', 'clear_dwell'
      return self.target
    except (AttributeError, TypeError, ValueError, OverflowError):
      return self._unavailable('malformed_source', now)
