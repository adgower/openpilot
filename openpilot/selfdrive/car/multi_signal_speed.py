"""Local multi-signal speed-cap experiment; no actuator or button authority."""
import math
from openpilot.selfdrive.car.predicted_speed import PredictedSpeedDemand
from openpilot.selfdrive.car.lead_speed import LeadSpeedDemand


class MultiSignalDemand:
  def __init__(self, *, use_lead=True, admission_mph=40., clear_dwell=.5):
    self.ego = PredictedSpeedDemand(raw_threshold=None)
    self.lead = LeadSpeedDemand()
    self.use_lead = use_lead
    self.admission_mph = admission_mph
    self.clear_dwell = clear_dwell
    self.reset()

  def reset(self):
    self.ego.reset()
    self.lead.reset()
    self.owners = set()
    self.sources = {}
    self.target = None
    self.reduction_target = None
    self.state, self.reason = 'unavailable', 'reset'
    self.data_valid = self.available = self.clear = False
    self.detect_since = None
    self.future_slowdown = False
    self.last_time = self.ego_clear_since = self.ceiling_clear_since = None
    self.lead_hold_target = None
    self.lead_raise_since = self.lead_raise_min = None
    self.ego_hold_target = None
    self.raise_since = self.raise_min = None

  def observe(self, now, speed_mph, ceiling, model, radar, allowed, ego_accel=0.):
    # Numerical validity and permission are distinct. Ownership survives a
    # dropout; missing evidence is not permission to restore the set speed.
    gap = self.last_time is not None and (now <= self.last_time or now-self.last_time > .2+1e-6)
    if gap:
      self.ego.reset()
      self.lead.reset()
      self.ego_clear_since = self.ceiling_clear_since = None
      self.raise_since = self.raise_min = None
    self.last_time = now
    ego_target = self.ego.observe(now, speed_mph, ceiling, model, allowed, True)
    self.data_valid = self.ego.data_valid
    self.detect_since = self.ego.detect_since
    self.future_slowdown = self.ego.future_slowdown
    if not (allowed and self.ego.available):
      ego_state, ego_reason = 'unavailable', 'invalid_or_inhibited'
      self.ego_clear_since = self.ceiling_clear_since = None
      self.raise_since = self.raise_min = None
    elif ego_target is not None:
      ego_state, ego_reason = 'active', 'future_ego_speed'
      if self.ego.future_slowdown:
        self.ego_clear_since = None
      elif self.ego_clear_since is None:
        self.ego_clear_since = now
    elif self.ego.future_slowdown:
      ego_state, ego_reason = 'unavailable', 'qualifying'
      self.ego_clear_since = None
    else:
      if self.ego_clear_since is None:
        self.ego_clear_since = now
      ego_state = 'clear' if now-self.ego_clear_since >= self.clear_dwell-1e-6 else 'unavailable'
      ego_reason = 'fresh_future_clear' if ego_state == 'clear' else 'clear_dwell'
    # Admission asks whether more deceleration is needed relative to actual
    # speed. Once owned, keep the absolute forecast cap until it supports the
    # user's original ceiling; becoming already slow is not release evidence.
    if ('ego' in self.owners and allowed and self.ego.available and
        not (ego_target is None and self.ego.future_slowdown)):
      absolute = self.ego._future_speed(model, speed_mph, require_loss=False)
      if absolute is not None and math.isfinite(absolute):
        held = self.ego_hold_target if self.ego_hold_target is not None else ceiling
        if absolute >= ceiling-1e-6:
          if self.ceiling_clear_since is None:
            self.ceiling_clear_since = now
          clear = now-self.ceiling_clear_since >= self.clear_dwell-1e-6
          ego_state = 'clear' if clear else 'active'
          ego_reason = 'ceiling_supported' if clear else 'ceiling_support_dwell'
          ego_target = None if clear else held
          self.raise_since = self.raise_min = None
        else:
          self.ceiling_clear_since = None
          # Strengthening still requires the original supported deceleration
          # cue. Removing that gate for recovery must not admit isolated lows.
          if self.ego.future_slowdown and ego_target is not None and ego_target < held:
            held = ego_target
            self.raise_since = self.raise_min = None
          elif absolute >= held+1. and absolute >= speed_mph:
            if self.raise_since is None:
              self.raise_since, self.raise_min = now, absolute
            self.raise_min = min(self.raise_min, absolute)
            if now-self.raise_since >= self.clear_dwell-1e-6:
              held = max(held, min(ceiling, self.raise_min))
              self.raise_since = self.raise_min = None
          else:
            self.raise_since = self.raise_min = None
          ego_target = max(25., min(ceiling, held))
          ego_state, ego_reason = 'active', 'owned_absolute_forecast'
    if ego_state == 'active':
      self.ego_hold_target = ego_target
    elif ego_state == 'clear':
      self.ego_hold_target = None
    lead_target = self.lead.observe(now, speed_mph, ceiling, model, radar, allowed, ego_accel) if self.use_lead else None
    lead_state = self.lead.state if self.use_lead else 'clear'
    lead_reason = self.lead.reason if self.use_lead else 'disabled'
    # Qualify relaxation in the same whole-mph units as cruise requests.
    if gap or lead_state != 'active':
      self.lead_raise_since = self.lead_raise_min = None
    if lead_state == 'clear':
      self.lead_hold_target = None
    elif lead_state == 'active' and lead_target is not None:
      requested = math.floor(max(25., min(ceiling, lead_target)))
      held = self.lead_hold_target
      if held is None or requested <= held:
        self.lead_hold_target = requested
        self.lead_raise_since = self.lead_raise_min = None
      else:
        if self.lead_raise_since is None:
          self.lead_raise_since = now
          self.lead_raise_min = requested
        self.lead_raise_min = min(self.lead_raise_min, requested)
        if now - self.lead_raise_since >= self.clear_dwell - 1e-6:
          self.lead_hold_target = self.lead_raise_min
          self.lead_raise_since = self.lead_raise_min = None
      lead_target = self.lead_hold_target
    self.sources = {
      'ego': {'state': ego_state, 'target': ego_target, 'reason': ego_reason},
      'lead': {'state': lead_state, 'target': lead_target, 'reason': lead_reason},
    }
    active = []
    pending = False
    for name, source in self.sources.items():
      if source['state'] == 'active' and name not in self.owners and speed_mph < self.admission_mph:
        source.update(state='unavailable', target=None, reason='below_admission')
        if name == 'lead':
          self.lead_hold_target = None
          self.lead_raise_since = self.lead_raise_min = None
      if source['state'] == 'active':
        self.owners.add(name)
        if source['target'] is not None and math.isfinite(source['target']):
          active.append(max(25., min(ceiling, source['target'])))
      elif source['state'] == 'clear':
        self.owners.discard(name)
      pending |= source['reason'] == 'qualifying'
    unavailable_owner = any(self.sources[name]['state'] == 'unavailable' for name in self.owners)
    self.available = bool(allowed and self.data_valid and self.ego.available and not unavailable_owner and not pending)
    self.clear = bool(self.available and not active and not self.owners and ego_state == 'clear')
    self.target = min(active) if active and self.available else None
    # Holding an old restriction is not permission to spend more SET-.
    # Keep fresh reduction evidence separate from the retained recovery cap.
    self.reduction_target = None
    if self.target is not None:
      ego_floor = ceiling
      if self.sources['ego']['state'] == 'active':
        absolute = self.ego._future_speed(model, speed_mph, require_loss=False)
        ego_floor = max(self.sources['ego']['target'], absolute) if absolute is not None else self.sources['ego']['target']
      lead_floor = lead_target if self.sources['lead']['state'] == 'active' else ceiling
      self.reduction_target = max(25., min(ceiling, ego_floor, lead_floor))
    self.state = 'active' if self.target is not None else 'clear' if self.clear else 'unavailable'
    self.reason = 'minimum_qualified_cap' if self.target is not None else 'fresh_sources_clear' if self.clear else 'awaiting_source_evidence'
    return self.target
