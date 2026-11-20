"""Acknowledged OEM cruise requests with independent user intent and pulse credit."""
import math

from openpilot.selfdrive.car.vision_curve_button_transport import BoundedButtonTransport
from openpilot.selfdrive.car.vision_curve_speed import (
  BranchContext, COMFORT_APPROACH_DECEL, COMFORT_LATERAL_ACCEL, COMFORT_RESPONSE_TIME, MPH, _corridor_traces, comfort_approach_mph, corridor_veto, measure_curve_preview,
)


# A driver tap while cruise is off must be followed by a fresh OEM engagement.
# This window applies only to intent, never to a transmitted button credit.
DRIVER_REENGAGE_WINDOW = 1.


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
    self.resume_until = -math.inf
    self.previous_user_action = None
    self.selection_at = None
    self.selection_engaged = False
    self.selection_display = None
    self.selection_direction = 0
    self.selection_ambiguous = False
    self.manual_at = -math.inf
    self.target = None
    self.forecast_target = None
    self.geometry_target = None
    self.geometry_hold = self.geometry_clear_since = None
    self.geometry_trimmed = False
    self.opening_since = self.opening_step = self.opening_ack = None
    self.all_decrements_geometry = False
    self.carried_decrements = False
    self.following_clear_since = None
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
    self.transport = BoundedButtonTransport(self.observed, ceiling_mph=self.ceiling,
                                           minus_interval=.18, plus_interval=.18, press_s=.13, burst=3)
    self.fault = False
    self.suspended = False
    self.resume_armed = False
    self.all_decrements_geometry = False
    self.carried_decrements = False
    self._clear_demand()

  def _clear_demand(self):
    # The driver still owns the ceiling. An acknowledged reduction may need to
    # return there, but fresh evidence must supply the next command.
    reduced = self.observed < self.ceiling
    self.forecast_restore_active = reduced and (self.forecast_restore_active or self.forecast_owned)
    self.geometry_trimmed = reduced and self.geometry_trimmed
    self.target = self.current_demand = None
    self.forecast_owned = False
    self.forecast_target = self.geometry_target = None
    self.geometry_hold = self.geometry_clear_since = None
    self.opening_since = self.opening_step = self.opening_ack = None
    self.following_clear_since = None
    self.curve_direction = self.last_accel_model_at = None
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
    before = self.transport.observed
    outstanding = self.transport.down_debt
    # A forecast at or above the reported setpoint cannot request a decrement.
    # In that overlap, emit only a step justified by the freshly qualified bend
    # and the positions already reserved for possibly delayed decrements.
    display = round(observed)
    down_receipt = max(0, before - display)
    lowest_next = display - max(0, outstanding - down_receipt) - 1
    forecast_cannot_reduce = self.forecast_target is not None and self.forecast_target >= display
    geometry_alone = (self.model_ok and self.detect_since is not None and now - self.detect_since >= .6
                      and self.geometry_target is not None and lowest_next >= self.geometry_target
                      and self.geometry_hold is not None and lowest_next >= self.geometry_hold)
    if (allowed and self.carried_decrements and self.geometry_hold is not None and forecast_cannot_reduce
        and self.target is not None and self.target < display and not geometry_alone):
      can_assert = False
    out = self.transport.update(now, round(observed), self.target or self.ceiling, allowed=allowed, can_assert=can_assert)
    acknowledged = max(0, before - self.transport.observed)
    if not self.transport.down_debt:
      self.carried_decrements = False
    elif self.transport.observed > before and self.transport.observed == self.ceiling:
      self.carried_decrements = True
    if acknowledged and allowed and self.all_decrements_geometry:
      self.geometry_trimmed = True
    # Source is attached at issuance. In a mixed group we cannot know which
    # press Ford accepted; wait for a wholly geometric group before granting.
    if out.button == 'SET-' and out.reason in ('new-pulse', 'repeat-pulse'):
      geometry_only = self.geometry_hold is not None and (self.forecast_target is None or (forecast_cannot_reduce and geometry_alone))
      self.all_decrements_geometry = geometry_only and (outstanding == acknowledged or self.all_decrements_geometry)
    elif not self.transport.down_debt:
      self.all_decrements_geometry = False
    if self.opening_ack is not None and self.transport.observed > before:
      # Only a reported setpoint increase can replace the old minimum. A
      # fresh tighter curve still takes priority on the next model frame.
      if self.transport.observed == self.opening_ack and self.geometry_hold is not None:
        self.geometry_hold = max(self.geometry_hold, min(self.opening_ack, self.geometry_target))
      self.opening_ack = self.opening_step = self.opening_since = None
    if self.opening_step is not None and out.button == 'SET+' and out.reason in ('new-pulse', 'repeat-pulse'):
      self.opening_ack = self.opening_step
      self.opening_since = None
    self.last_transport = out
    self.observed = self.transport.observed
    self.pending = None
    self.fault |= self.transport.fault
    if self.observed >= self.ceiling and not self.transport.pending:
      self.forecast_restore_active = False
      self.geometry_trimmed = False
    self.recovering = out.phase == 'wait'
    self.reason = out.reason
    # One emitted CAN button packet per ledger credit. The transport's
    # internal hold phase gates timing; 100 Hz callbacks must not turn a
    # single pulse credit into five independent button transmissions.
    if out.button and out.reason in ('new-pulse', 'repeat-pulse'):
      return out.button
    return '' if out.phase == 'release' else None

  def _can_return_while_following(self, now, following_clear):
    return (following_clear and self.geometry_trimmed and self.geometry_hold is None
            and self.geometry_clear_since is not None and now - self.geometry_clear_since >= 2.5
            and self.following_clear_since is not None and now - self.following_clear_since >= 1.
            and not self.forecast_owned and self.forecast_target is None
            and (not self.transport.down_debt or self.all_decrements_geometry))

  def _motion_allows_step(self, display, speed, lateral_accel):
    next_mph = round(display) + 1
    # Keep a full mph above the trial speed while a qualified easing persists.
    # The buffered 10-second path and measured load still gate every sample.
    in_progress = self.opening_since is not None or (self.opening_ack == next_mph and self.transport.pressed)
    reserve = 1. if in_progress else 2.
    return (self.geometry_target is not None and self.geometry_target >= max(next_mph + reserve, speed + 1.) and
            speed > 0. and abs(speed - display) <= 3. and
            abs(lateral_accel) * max(1., (next_mph / speed) ** 2) <= COMFORT_LATERAL_ACCEL)

  def _opening_step(self, now, display, speed, lateral_accel, model):
    # A partial return must fit both the currently measured turn and every
    # forecast turn, including an opposite-direction bend farther ahead.
    # A fresh preview may arrive on the same tick as Ford's receipt. Check
    # the issued speed before its receipt can raise the bend minimum.
    next_mph = self.opening_ack if self.opening_ack is not None else round(display) + 1
    buffered_speed_sq = ((next_mph + 1) * MPH) ** 2
    approach = 2. * COMFORT_APPROACH_DECEL * speed * MPH
    supported = (self.geometry_hold is not None and next_mph < self.ceiling and
                 round(display) in (next_mph - 1, next_mph) and
                 self._motion_allows_step(next_mph - 1, speed, lateral_accel) and
                 not model.action.shouldStop and model.action.desiredAcceleration >= .1 and
                 all(velocity >= 5. and
                     abs(rate) * (buffered_speed_sq - approach * max(0., t - COMFORT_RESPONSE_TIME)) <= COMFORT_LATERAL_ACCEL * velocity
                     for t, rate, velocity in zip(model.orientationRate.t, model.orientationRate.z, model.velocity.x, strict=True)))
    if not supported:
      self.opening_since = self.opening_ack = None
      return None
    if self.opening_ack is not None:
      return next_mph if self.transport.pressed and round(display) < next_mph else None
    if self.opening_since is None:
      self.opening_since = now
    return next_mph if now - self.opening_since >= 1. else None

  def update(self, now, *, engaged, valid, manual, override, set_speed, speed, accel, model,
             steering_only=False, lane_change=False, comfort_hint_s=None, cruise_engaged=None,
             forecast_mph=None, forecast_clear=False, user_action=None, override_kind=None, forecast_authoritative=False, forecast_available=None, forecast_reduce_mph=None, can_assert=True, lateral_accel=math.inf, preview=None, early_hint_inhibited=False, following_clear=False, cruise_state_valid=None, user_set_direction=0):
    # No inhibited interval may contribute to geometric release evidence.
    if (not valid or not engaged or not cruise_engaged or manual or override or
        steering_only or lane_change or forecast_available is False or self.suspended):
      self.geometry_clear_since = None
      self.opening_since = self.opening_step = self.opening_ack = None
    # A Ford yaw or speed sample can change after the last model frame but
    # before a neutral button frame arrives. Revoke the earned step at once.
    motion_display = self.opening_ack - 1 if self.opening_ack is not None else set_speed
    if early_hint_inhibited or ((self.opening_since is not None or self.opening_ack is not None) and
                                not self._motion_allows_step(motion_display, speed, lateral_accel)):
      self.opening_since = self.opening_step = self.opening_ack = None
    if not following_clear or self.geometry_clear_since is None:
      self.following_clear_since = None
    resume_edge = user_action == 'resume' and self.previous_user_action != 'resume'
    set_edge = user_action == 'set' and self.previous_user_action != 'set'
    self.previous_user_action = user_action
    native_valid = valid if cruise_state_valid is None else cruise_state_valid
    native_on = native_valid and cruise_engaged
    native_off = native_valid and not cruise_engaged
    stopped = user_action == 'cancel' or (override and override_kind != 'gas')
    entering_suspend = self.ceiling is not None and not self.suspended and (stopped or native_off)
    if entering_suspend or stopped:
      self.selection_ambiguous |= self.selection_at is not None or (stopped and user_action == 'set')
      self.suspended = True
      self.resume_armed = False
      self.selection_at = None
    if self.resume_armed and now > self.resume_until:
      self.resume_armed = False
    if resume_edge and self.suspended and not stopped:
      if not self.selection_ambiguous or (native_off and self.transport is not None and not self.transport.pending):
        self.resume_armed = True
        self.resume_until = now + DRIVER_REENGAGE_WINDOW
    if self.resume_armed and native_on:
      # A deliberate OEM restart can re-establish the previously verified
      # ceiling after a contradictory driver reading only with no unpaid moves.
      if (self.selection_ambiguous and self.transport is not None and not self.transport.pending
          and 25 <= set_speed <= 95 and round(set_speed) == self.ceiling):
        self.selection_ambiguous = False
        self._start(self.ceiling)
        self.override_until = max(self.override_until, now + .4)
      if not self.selection_ambiguous:
        # Ford can accept RES before radar, the model or lateral control recover.
        self.suspended = self.resume_armed = False

    if self.selection_at is not None:
      if (self.selection_engaged and native_off) or (not self.selection_engaged and now - self.manual_at > DRIVER_REENGAGE_WINDOW):
        self.selection_at = None
      elif native_on:
        self.selection_engaged = True
    if user_action == 'set' and not stopped:
      if set_edge:
        self.selection_ambiguous = False
        self.selection_display = None
        self.selection_direction = user_set_direction
        self.selection_engaged = bool(native_on)
      if set_edge or self.selection_at is not None:
        self.selection_at = self.manual_at = now
        if native_on and 25 <= set_speed <= 95:
          display = round(set_speed)
          if self.selection_display is None or user_set_direction * (display - self.selection_display) >= 0:
            self.selection_display = display
          elif self.transport is not None and abs(display - self.selection_display) <= self.transport.debt[-user_set_direction]:
            # An outstanding move could explain this wrong-direction change.
            self.selection_ambiguous = True
          # Otherwise keep the last consistent reading and let the physical
          # hold finish. A transient contradiction cannot become the ceiling.
    elif self.selection_at is not None and native_on and self.transport is not None and 25 <= set_speed <= 95:
      display = round(set_speed)
      if self.selection_display is None:
        self.selection_ambiguous = bool(self.transport.pending)
      else:
        delta = display - self.selection_display
        # After release, either a wrong-direction driver value or a change that
        # might be an old automatic press leaves the new ceiling unconfirmed.
        if (self.selection_direction * delta < 0 or (delta > 0 and self.transport.up_debt)
            or (delta < 0 and self.transport.down_debt)):
          self.selection_ambiguous = True
    if self.selection_ambiguous and self.selection_at is not None:
      self.suspended = True
      self.selection_at = None
      if self.ceiling is not None:
        self._clear_demand()

    selecting = user_action == 'set' or self.selection_at is not None or self.selection_ambiguous
    if stopped or user_action == 'set' or self.selection_ambiguous or not native_on or (self.suspended and not selecting):
      # A driver SET owns the display delta; it cannot acknowledge an older
      # synthetic press. Ordinary brake/off can coincide with a genuine receipt.
      display = self.transport.observed if self.transport is not None and (selecting or not native_valid) else set_speed
      release = self._send(now, display, False)
      if entering_suspend:
        self._clear_demand()
      return release
    if self.selection_at is not None:
      if now - self.manual_at < .4 or not self.selection_engaged or not valid or not 25 <= set_speed <= 95:
        observed = self.transport.observed if self.transport is not None else set_speed
        return self._send(now, observed, False)
      self.selection_at = None
      previous_transport = self.transport
      self._start(set_speed)
      if previous_transport is not None:
        # A new driver selection cannot acknowledge an earlier synthetic press.
        self.transport.debt = previous_transport.debt.copy()
        self.transport.reserve_down_slot = previous_transport.reserve_down_slot
        self.transport.next_press = max(self.transport.next_press, previous_transport.next_press)
    if override and override_kind == 'gas':
      self.override_until = max(self.override_until, now + .25)
      self.accel_since = None
      self.model_ok = False
      return self._send(now, set_speed, False)
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
      self.detect_since = None
      self.accel_since = None
      self.model_ok = False
      self.geometry_clear_since = None
      return self._send(now, set_speed, False)

    if model is not None:
      self.opening_step = None
      if self.last_accel_model_at is not None and now-self.last_accel_model_at > .2:
        self.geometry_clear_since = self.opening_since = self.opening_ack = None
      curve, preview_valid = preview if preview is not None else measure_curve_preview(model)
      if forecast_authoritative and not preview_valid:
        self.geometry_restore_blocked = True
        self.opening_since = self.opening_ack = None
        self.detect_since = self.accel_since = self.geometry_clear_since = None
        self.model_ok = False
        return self._send(now, set_speed, False)
      traces = _corridor_traces(model)
      path_blocked = bool(corridor_veto(model, traces))
      branch_blocked = self.branch.update(now, self.ceiling, model, curve, traces)
      lane_ok = len(model.laneLineProbs) >= 3 and min(list(model.laneLineProbs)[1:3]) >= .55
      qualified_geometry = curve is not None and lane_ok and not (path_blocked or branch_blocked)
      raw = (comfort_approach_mph(model, curve, speed, comfort_hint_s or 0.)
             if qualified_geometry else self.ceiling)
      self.geometry_target = max(25., min(self.ceiling, raw))
      if self.opening_ack is not None and (not qualified_geometry or self.geometry_target < self.opening_ack):
        self.opening_ack = None
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

      if following_clear and self.geometry_trimmed and self.geometry_clear_since is not None:
        if self.following_clear_since is None:
          self.following_clear_since = now
      else:
        self.following_clear_since = None
      forecast_restore = self.forecast_restore_active and forecast_clear and forecast is None and not raw_geometric_demand
      self.model_ok = not (path_blocked or branch_blocked) and (lane_ok or forecast is not None or forecast_restore)
      reduction = (forecast_reduce_mph if forecast_authoritative and forecast_reduce_mph is not None
                   and math.isfinite(forecast_reduce_mph) else self.forecast_target)
      self.current_demand = min(self.ceiling, self.geometry_target if qualified_geometry else self.ceiling,
                                reduction if reduction is not None else self.ceiling)
      active = (self.geometry_hold is not None if forecast_authoritative else geometry_active) or self.forecast_target is not None
      self.clear_pending = False
      opening_allowed = (active and self.model_ok and geometry_active and not early_hint_inhibited
                         and forecast_available and not self.forecast_owned and now >= self.override_until)
      if not opening_allowed:
        self.opening_since = self.opening_ack = None
      if active and self.model_ok:
        demand = min(self.ceiling, self.geometry_target if geometry_active else self.ceiling,
                     self.forecast_target if self.forecast_target is not None else self.ceiling)
        # A change in the forecast alone cannot release the minimum. While a
        # bend remains active, earn one step at a time from fresh headroom.
        if forecast_authoritative:
          # Owned future-speed recovery supplies a qualified cap, not an
          # unconditional jump to the ceiling. Geometry has its own hold.
          if opening_allowed:
            self.opening_step = self._opening_step(now, set_speed, speed, lateral_accel, model)
          curve_cap = self.geometry_hold if self.geometry_hold is not None else self.ceiling
          if self.opening_step is not None:
            curve_cap = max(curve_cap, self.opening_step)
          self.target = min(self.ceiling, curve_cap, self.forecast_target if self.forecast_target is not None else self.ceiling)
        else:
          self.target = min(self.target if self.target is not None else self.ceiling, demand)
        self.clear_since = None
      elif self.model_ok and not geometric_demand and (forecast_clear or self._can_return_while_following(now, following_clear) or (acceleration_clear and not forecast_authoritative)):
        if self.clear_since is None:
          self.clear_since = now
        if forecast_authoritative or acceleration_clear or now - self.clear_since >= .4:
          self.target = float(self.ceiling)
        else:
          self.clear_pending = True
      else:
        self.clear_since = None

    if self.target is None:
      return self._send(now, set_speed, False)
    allowed = self.model_ok and now >= self.override_until
    if early_hint_inhibited and ((self.target > set_speed and not self._can_return_while_following(now, following_clear)) or
        (self.target < set_speed and (self.geometry_hold is None or self.geometry_target is None
                                      or set_speed <= self.geometry_target or speed <= self.geometry_target))):
      # A fresh radar lead can arrive between model frames. Wait for Ford to
      # handle following; only an independently qualified bend can spend SET-.
      allowed = False
    if self.clear_pending and self.target < set_speed:
      allowed = False
    if (self.geometry_restore_blocked or self.geometry_hold is not None) and self.opening_step is None and (self.target > set_speed or
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

