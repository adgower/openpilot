"""Acknowledged OEM cruise requests with independent user intent and pulse credit."""
import math

from openpilot.selfdrive.car.vision_curve_button_transport import BoundedButtonTransport
from openpilot.selfdrive.car.vision_curve_speed import (
  BranchContext, MPH, _corridor_traces, comfort_approach_mph, corridor_veto, measure_curve_preview,
)


class AcknowledgedCurveSpeed:
  def __init__(self):
    self.ceiling = self.observed = self.pending = None
    self.transport = self.last_transport = None
    self.fault = False
    self.recovering = False
    self.retries = set()
    self.forecast_owned = False
    self.forecast_restore_active = False
    self.branch = BranchContext()
    self.override_until = -math.inf
    self.suspended = False
    self.resume_armed = False
    self.previous_user_action = None
    self.selection_at = None
    self.manual_at = -math.inf
    self.target = None
    self.forecast_target = None
    self.geometry_target = None
    self.geometry_hold = self.geometry_clear_since = None
    self.current_demand = None
    self.detect_since = self.clear_since = self.accel_since = None
    self.curve_direction = None
    self.model_ok = False
    self.geometry_restore_blocked = False
    self.last_accel_model_at = None
    self.clear_pending = False
    self.reason = 'uninitialized'

  def _start(self, speed):
    self.ceiling = self.observed = round(speed)
    self.target = float(self.ceiling)
    self.transport = BoundedButtonTransport(self.observed, ceiling_mph=self.ceiling,
                                           minus_interval=.18, plus_interval=.18, press_s=.13, burst=3)
    self.fault = False
    self.suspended = False
    self.resume_armed = False
    self.forecast_owned = self.forecast_restore_active = False
    self.forecast_target = self.geometry_target = None
    self.geometry_hold = self.geometry_clear_since = None
    self.current_demand = float(self.ceiling)
    self.detect_since = self.clear_since = self.accel_since = None
    self.geometry_restore_blocked = False
    self.model_ok = False
    self.clear_pending = False

  def _send(self, now, observed, allowed, *, can_assert=True):
    if self.transport is None:
      return None
    # Off-road/invalid samples carry no new authoritative setpoint evidence.
    if not allowed and (not math.isfinite(observed) or not 25 <= observed <= 95):
      observed = self.transport.observed
    out = self.transport.update(now, round(observed), self.target or self.ceiling, allowed=allowed, can_assert=can_assert)
    self.last_transport = out
    self.observed = self.transport.observed
    self.pending = None
    self.fault |= self.transport.fault
    if self.observed >= self.ceiling and not self.transport.pending:
      self.forecast_restore_active = False
    self.recovering = out.phase == 'wait'
    self.reason = out.reason
    # One emitted CAN button packet per ledger credit. The transport's
    # internal hold phase gates timing; 100 Hz callbacks must not turn a
    # single pulse credit into five independent button transmissions.
    if out.button and out.reason in ('new-pulse', 'repeat-pulse'):
      return out.button
    return '' if out.phase == 'release' else None

  def update(self, now, *, engaged, valid, manual, override, set_speed, speed, accel, model,
             steering_only=False, lane_change=False, comfort_hint_s=None, cruise_engaged=None,
             forecast_mph=None, forecast_clear=False, user_action=None, override_kind=None, forecast_authoritative=False, forecast_available=None, forecast_reduce_mph=None, can_assert=True):
    # No inhibited interval may contribute to geometric release evidence.
    if (not valid or not engaged or not cruise_engaged or manual or override or
        steering_only or lane_change or forecast_available is False or self.suspended):
      self.geometry_clear_since = None
    resume_edge = user_action == 'resume' and self.previous_user_action != 'resume'
    self.previous_user_action = user_action
    # A physical SET selection replaces intent only after button release and
    # without treating driver movement as an automatic acknowledgement.
    if user_action == 'set':
      self.selection_at = self.manual_at = now
      # A physical SET explains this movement. Release the old scheduler
      # without submitting the driver change as an automatic ACK.
      observed = self.transport.observed if self.transport is not None else set_speed
      return self._send(now, observed, False)
    if self.selection_at is not None:
      if now - self.manual_at < .4 or not valid or not 25 <= set_speed <= 95:
        observed = self.transport.observed if self.transport is not None else set_speed
        return self._send(now, observed, False)
      self.selection_at = None
      previous_transport = self.transport
      self._start(set_speed)
      if previous_transport is not None:
        # A new driver selection cannot acknowledge an earlier synthetic press.
        self.transport.debt = previous_transport.debt.copy()
        self.transport.next_press = max(self.transport.next_press, previous_transport.next_press)
    if resume_edge and (self.suspended or not cruise_engaged):
      # A RES tap while already active must not authorize a later unrelated
      # suspension. Retain a genuine RES edge preceding OEM engagement.
      self.resume_armed = True
    if override and override_kind == 'gas' and cruise_engaged and user_action != 'cancel':
      # Native cruise may stay engaged through accelerator override. Pause
      # button writes without creating a brake/cancel suspension requiring
      # the shared RES/CANCEL switch (which would cancel active cruise).
      self.override_until = max(self.override_until, now + .25)
      self.accel_since = None
      self.model_ok = False
      return self._send(now, set_speed, False)
    if override or user_action == 'cancel':
      self.suspended = True
      # A genuine RES press can precede OEM engagement while the driver is
      # accelerating. Gas pauses output but must not erase that permission.
      # Brake, cancel and other overrides still revoke it.
      if override_kind != 'gas' or user_action == 'cancel':
        self.resume_armed = False
      return self._send(now, set_speed, False)
    if not cruise_engaged:
      if self.ceiling is not None:
        self.suspended = True
      return self._send(now, set_speed, False)
    if self.suspended:
      if not (self.resume_armed and engaged and valid):
        return self._send(now, set_speed, False)
      self.suspended = False
      self.resume_armed = False
    if manual and user_action is None:
      self.override_until = max(self.override_until, now + .4)
    if self.ceiling is None:
      if not (valid and engaged and 25 <= set_speed <= 95):
        return None
      self._start(set_speed)

    if steering_only or lane_change:
      self.override_until = max(self.override_until, now + .25)
    permitted = (valid and engaged and not manual and not steering_only and not lane_change and not self.fault
                 and forecast_available is not False)
    if not permitted:
      self.accel_since = None
      self.model_ok = False
      self.geometry_clear_since = None
      return self._send(now, set_speed, False)

    if model is not None:
      if self.last_accel_model_at is not None and now-self.last_accel_model_at > .2:
        self.geometry_clear_since = None
      curve, preview_valid = measure_curve_preview(model)
      traces = _corridor_traces(model)
      path_blocked = bool(corridor_veto(model, traces))
      branch_blocked = self.branch.update(now, self.ceiling, model, curve, traces)
      lane_ok = len(model.laneLineProbs) >= 3 and min(list(model.laneLineProbs)[1:3]) >= .55
      qualified_geometry = curve is not None and lane_ok and not (path_blocked or branch_blocked)
      raw = (comfort_approach_mph(model, curve, speed, comfort_hint_s or 0.)
             if qualified_geometry else self.ceiling)
      self.geometry_target = max(25., min(self.ceiling, raw))
      geometric_demand = qualified_geometry and self.ceiling - self.geometry_target >= 2.
      if geometric_demand:
        if self.detect_since is None or curve.direction != self.curve_direction:
          self.detect_since = now
        self.curve_direction = curve.direction
      else:
        self.detect_since = None
        self.curve_direction = None
      geometry_active = geometric_demand and now - self.detect_since >= .6
      # Admission dwell delays reduction, never permits restoration into a
      # newly observed bend. Also release a currently held SET+ immediately.
      raw_geometric_demand = curve is not None and self.ceiling - math.sqrt(1.8 / curve.curvature) / MPH >= 2.
      self.geometry_restore_blocked = raw_geometric_demand or not preview_valid

      if forecast_authoritative:
        # Preserve the bend's own minimum independently of a forecast cap.
        # A forecast recovering upward must not erase a still-owned bend.
        if geometry_active:
          self.geometry_hold = min(self.geometry_hold if self.geometry_hold is not None else self.ceiling,
                                   self.geometry_target)
          self.geometry_clear_since = None
        elif not geometric_demand and not self.geometry_restore_blocked and lane_ok and not (path_blocked or branch_blocked):
          if self.geometry_clear_since is None:
            self.geometry_clear_since = now
          if now-self.geometry_clear_since >= .4:
            self.geometry_hold = None
        else:
          self.geometry_clear_since = None

      forecast = forecast_mph if forecast_mph is not None and math.isfinite(forecast_mph) else None
      if forecast is not None:
        self.forecast_target = min(self.ceiling, max(25., forecast))
        self.forecast_owned = True
        self.forecast_restore_active = False
      elif forecast_clear:
        self.forecast_restore_active |= self.forecast_owned
        self.forecast_target = None
        self.forecast_owned = False

      # Missing model frames and intervening driver/data inhibition cannot
      # count toward Alpha acceleration persistence.
      if self.last_accel_model_at is not None and now - self.last_accel_model_at > .2:
        self.accel_since = None
      self.last_accel_model_at = now
      positive_accel = (not model.action.shouldStop and math.isfinite(model.action.desiredAcceleration)
                        and model.action.desiredAcceleration >= .1)
      self.accel_since = (now if self.accel_since is None else self.accel_since) if positive_accel else None
      acceleration_clear = self.accel_since is not None and now - self.accel_since >= .2
      # Fresh positive acceleration can release an old forecast, not a current
      # curve target. Missing/stale inputs alone are never a restoration cue.
      if not forecast_authoritative and acceleration_clear and forecast is None and not geometric_demand and lane_ok and not path_blocked:
        self.forecast_target = None
        self.forecast_owned = False

      forecast_restore = self.forecast_restore_active and forecast_clear and forecast is None and not raw_geometric_demand
      self.model_ok = not (path_blocked or branch_blocked) and (lane_ok or forecast is not None or forecast_restore)
      reduction = (forecast_reduce_mph if forecast_authoritative and forecast_reduce_mph is not None
                   and math.isfinite(forecast_reduce_mph) else self.forecast_target)
      self.current_demand = min(self.ceiling, self.geometry_target if qualified_geometry else self.ceiling,
                                reduction if reduction is not None else self.ceiling)
      active = (self.geometry_hold is not None if forecast_authoritative else geometry_active) or self.forecast_target is not None
      self.clear_pending = False
      if active and self.model_ok:
        demand = min(self.ceiling, self.geometry_target if geometry_active else self.ceiling,
                     self.forecast_target if self.forecast_target is not None else self.ceiling)
        # An active slowdown episode may strengthen but never restore just
        # because its estimate weakens. Only the qualified clear path below
        # releases this minimum back to the driver's ceiling.
        if forecast_authoritative:
          # Owned future-speed recovery supplies a qualified cap, not an
          # unconditional jump to the ceiling. Geometry has its own hold.
          self.target = min(self.ceiling,
                            self.geometry_hold if self.geometry_hold is not None else self.ceiling,
                            self.forecast_target if self.forecast_target is not None else self.ceiling)
        else:
          self.target = min(self.target if self.target is not None else self.ceiling, demand)
        self.clear_since = None
      elif self.model_ok and not geometric_demand and (forecast_clear or (acceleration_clear and not forecast_authoritative)):
        if self.clear_since is None:
          self.clear_since = now
        if forecast_authoritative or acceleration_clear or now - self.clear_since >= .4:
          self.target = float(self.ceiling)
        else:
          self.clear_pending = True
      else:
        self.clear_since = None

    allowed = self.model_ok and now >= self.override_until
    if self.clear_pending and self.target < set_speed:
      allowed = False
    if self.geometry_restore_blocked and (self.target > set_speed or
        (self.transport.pressed and self.transport.direction > 0)):
      allowed = False
    if (forecast_authoritative and self.target is not None and self.target < set_speed and
        self.current_demand is not None and set_speed <= self.current_demand):
      # The acknowledged setpoint already satisfies fresh evidence. Don't
      # spend another decrement solely toward a historical episode minimum.
      allowed = False
    # OEM following/coasting remains authoritative; avoid a new SET- while
    # actual speed is already below the requested comfort target.
    if self.target is not None and self.target < set_speed and speed <= max(self.target, self.current_demand or self.target):
      allowed = False
    return self._send(now, set_speed, allowed, can_assert=can_assert)

