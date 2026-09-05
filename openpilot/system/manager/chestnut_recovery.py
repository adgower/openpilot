"""Bounded modeld restarts, interlocked with selfdrived while disengaged."""
import logging
import signal
from dataclasses import dataclass
from enum import Enum, auto

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class RecoveryInput:
  started: bool = False
  allowed: bool = False
  safe_to_restart: bool = False
  power_ready: bool = False
  model_valid: bool = False
  model_big: bool = False
  model_frame: int = 0
  model_time: float = 0.


class Phase(Enum):
  IDLE = auto()
  REQUESTED = auto()
  STOPPING = auto()
  LOADING = auto()
  CANCELLING = auto()
  FAILED = auto()


class ChestnutRecovery:
  MAX_ATTEMPTS = 2
  RETRY_DELAY = 30.
  STABLE_TIME = 10.
  ACK_TIMEOUT = 5.
  STOP_TIMEOUT = 5.
  LOAD_TIMEOUT = 90.
  VALIDATION_TIME = 2.
  VALIDATION_FRAMES = 30

  def __init__(self, params, process, logger=log):
    self.params, self.process = params, process
    self.log = logger
    self.generation = 0
    self.phase = Phase.IDLE
    self.attempts = 0
    self.saw_big = False
    self.failed_since = self.safe_since = self.valid_since = None
    self.phase_start = 0.
    self.validation_frame = 0
    self.small_only = False

  def _release(self):
    self.params.put('ChestnutRecoveryReady', 0, block=True)
    self.params.put('ChestnutRecoveryRequest', 0, block=True)
    self.params.put('ChestnutRecoveryAck', 0, block=True)
    self.phase = Phase.IDLE
    self.failed_since = self.safe_since = self.valid_since = None

  def _stop(self, now, small_only):
    self.small_only = small_only
    self.params.put_bool('ChestnutRecoverySkipBig', small_only, block=True)
    self.params.put('ChestnutRecoveryReady', 0, block=True)
    self.phase, self.phase_start = Phase.STOPPING, now
    self.valid_since = None
    self.process.stop(block=False)
    self.log.warning('Chestnut recovery stopping modeld (small_only=%s)', small_only)

  def _reaped(self, now):
    if self.process.proc is not None and self.process.proc.exitcode is None:
      if now-self.phase_start >= self.STOP_TIMEOUT:
        self.process.signal(signal.SIGKILL)
      return False
    self.process.stop(block=False)
    return True

  def update(self, state: RecoveryInput, now: float) -> bool:
    """Return True while modeld must be excluded from ensure_running."""
    if not state.started or not state.allowed:
      if self.phase != Phase.CANCELLING and (self.phase != Phase.IDLE or self.saw_big):
        stopping = self.phase == Phase.STOPPING
        self._release()
        self.params.put_bool('ChestnutRecoverySkipBig', False, block=True)
        if stopping:
          self.phase = Phase.CANCELLING
      self.saw_big, self.attempts = False, 0
      if self.phase != Phase.CANCELLING:
        return False

    if self.phase == Phase.CANCELLING:
      if not self._reaped(now):
        return True
      self.phase = Phase.IDLE
      return False

    active = self.params.get('ChestnutActive')
    self.saw_big |= active is True
    safe = state.safe_to_restart and state.power_ready

    if self.phase == Phase.IDLE:
      if not self.saw_big or active is not False or self.attempts >= self.MAX_ATTEMPTS:
        self.failed_since = self.safe_since = None
        return False
      if self.failed_since is None:
        self.failed_since = now
      self.safe_since = (now if self.safe_since is None else self.safe_since) if safe else None
      if self.safe_since is not None and now-self.safe_since >= self.STABLE_TIME and now-self.failed_since >= self.RETRY_DELAY:
        self.generation += 1
        self.params.put('ChestnutRecoveryAck', 0, block=True)
        self.params.put('ChestnutRecoveryRequest', self.generation, block=True)
        self.phase, self.phase_start = Phase.REQUESTED, now
        self.log.warning('Chestnut recovery requested (%d)', self.generation)

    elif self.phase == Phase.REQUESTED:
      if not safe or now-self.phase_start >= self.ACK_TIMEOUT:
        self._release()
      elif self.params.get('ChestnutRecoveryAck') == self.generation:
        self.attempts += 1
        self._stop(now, small_only=False)
        return True

    elif self.phase == Phase.STOPPING:
      # Reap before allowing ensure_running to launch a replacement. Never
      # overlap the old process's GPU context or a timed-out loader thread.
      if not self._reaped(now):
        return True
      self.params.remove('ChestnutActive')
      self.params.put_bool('ChestnutLoading', not self.small_only, block=True)
      self.phase, self.phase_start = Phase.LOADING, now

    elif self.phase == Phase.LOADING:
      loading = self.params.get_bool('ChestnutLoading')
      if not self.small_only and (now-self.phase_start >= self.LOAD_TIMEOUT or (active is False and not loading)):
        self._stop(now, small_only=True)
        return True
      if self.small_only and now-self.phase_start >= self.LOAD_TIMEOUT:
        # Leave the no-entry interlock set if even the clean small-model
        # process cannot publish. Do not restart forever or permit engagement.
        self.phase = Phase.FAILED
        self.log.error('Chestnut recovery failed: small model unavailable')
      elif (self.params.get('ChestnutRecoveryReady') == self.generation and
            state.model_valid and self.phase_start < state.model_time <= now and now-state.model_time < 1. and
            not loading and active is (not self.small_only) and state.model_big == (not self.small_only)):
        if self.valid_since is None or state.model_frame < self.validation_frame:
          self.valid_since, self.validation_frame = now, state.model_frame
        if now-self.valid_since >= self.VALIDATION_TIME and state.model_frame-self.validation_frame >= self.VALIDATION_FRAMES:
          self.log.warning('Chestnut recovery complete (small_only=%s)', self.small_only)
          self._release()
      else:
        self.valid_since = None

    return False
