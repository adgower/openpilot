import numpy as np
from opendbc.car.structs import car
from opendbc.car.ford.longitudinal_recovery import CommandRecovery
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(active, long_control_state, should_stop, brake_pressed, cruise_standstill):
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if should_stop:
        long_control_state = LongCtrlState.stopping

  return long_control_state

class LongControl:
  def __init__(self, CP, *, retention_enabled=False, reference_enabled=False):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController(0.0, (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.reference_integral = None
    if reference_enabled:
      if retention_enabled:
        raise ValueError("Navigator coast reference cannot be combined with receipt retention")
      from opendbc.car.ford.longitudinal_reference_feedback import ReferenceIntegral
      self.reference_integral = ReferenceIntegral()
    self.command_recovery = CommandRecovery(retention_enabled=retention_enabled,
      nominal_policy=self._reference_nominal if reference_enabled else None)

  def _reference_nominal(self, *args, **kwargs):
    return self.reference_integral.nominal(self.command_recovery, *args, **kwargs)

  def reset(self):
    self.pid.reset()
    self.command_recovery.reset()
    if self.reference_integral is not None:
      self.reference_integral.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, *, feedback=None, now=0., epoch=0, sequence=0,
             coordinated=False, source_valid=True, urgent=False, reference_context=None, target_ns=0):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    if self.reference_integral is not None:
      from opendbc.car.ford.longitudinal_reference_feedback import ReferenceContext
      self.reference_integral.begin(reference_context if reference_context is not None else ReferenceContext())
    self.command_recovery.prepare(active, CS, a_target, should_stop, feedback, now, epoch, sequence, coordinated, source_valid, urgent)
    before_i = float(self.pid.i)
    self.last_integral_frozen = False
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(active, self.long_control_state, should_stop,
                                                       CS.brakePressed, CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        # TODO: can we just go straight to stopAccel?
        output_accel -= 1.0 * DT_CTRL  # m/s^2/s while trying to stop
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      self.last_integral_frozen = self.command_recovery.freeze(now, epoch, sequence, error, a_target,
        self.pid.i + a_target, CS.vEgo, 'pid', feedback, enabled=coordinated)
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target, freeze_integrator=self.last_integral_frozen)

    output_accel = self.command_recovery.apply(output_accel, float(self.pid.i) - before_i, before_i + a_target,
      'pid' if self.long_control_state == LongCtrlState.pid else 'non_pid')
    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    if self.reference_integral is not None and self.long_control_state == LongCtrlState.pid:
      self.reference_integral.issue(epoch, sequence, now, a_target,
        source_valid=bool(coordinated and source_valid and not urgent and self.reference_integral.current.eligible), target_ns=target_ns)
    return self.last_output_accel
