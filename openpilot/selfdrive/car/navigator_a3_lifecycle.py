"""Shared calculation lifecycle; never grants production Angle permission."""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Decision:
  calculation_eligible: bool
  neutral_reset: bool
  phase: str
  fault_reason: str | None
  calculation_fault_reason: str | None
  disengagement_intent: bool
  production_activation_allowed: bool = field(default=False, init=False)


class Lifecycle:
  def __init__(self, command_role='a2'):
    if command_role not in ('a2', 'a3'):
      raise ValueError('Unknown monitored command role')
    self._command_role = command_role
    self.session_requested = False
    self.active = False
    self.driver_pressed = False
    self.source_fresh = False
    self.measurement_fresh = False
    self.fault_reason = None
    self.calculation_fault_reason = None

  def control_input(self, active, driver_pressed, source_fresh, measurement_fresh):
    self.active = bool(active)
    self.session_requested |= self.active
    self.driver_pressed = bool(driver_pressed)
    self.source_fresh = bool(source_fresh)
    self.measurement_fresh = bool(measurement_fresh)

  def observe_fault(self, reason):
    if not reason:
      return
    if self.fault_reason is None:
      self.fault_reason = reason
    # A2 evidence cannot reject an untransmitted shadow proposal.
    if self.calculation_fault_reason is None and not (self._command_role == 'a2' and reason == 'direct_steering_rejection'):
      self.calculation_fault_reason = reason

  def evaluate(self, configuration_valid, permission_available):
    if self.calculation_fault_reason:
      phase = 'faulted'
    elif not configuration_valid:
      phase = 'configuration_pending'
    elif not permission_available:
      phase = 'permission_unavailable'
    elif self.driver_pressed:
      phase = 'driver_pause'
    elif not self.active:
      phase = 'inactive'
    elif not self.source_fresh or not self.measurement_fresh:
      phase = 'invalid_input'
    else:
      phase = 'eligible'
    return Decision(phase == 'eligible', phase != 'eligible', phase, self.fault_reason,
                    self.calculation_fault_reason, self._command_role == 'a3' and self.calculation_fault_reason is not None)
