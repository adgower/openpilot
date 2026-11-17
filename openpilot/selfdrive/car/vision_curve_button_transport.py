"""Bounded cruise-button pulse scheduler; ECU pulse response needs road validation."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Output:
    button: str | None
    phase: str
    reason: str
    outstanding: int
    ceiling_mph: int


class BoundedButtonTransport:
    """Repeated pulse train with conserved unacknowledged movement credits.

    Call every <= press_s seconds. A press output denotes a level to hold until
    an explicit release output. `None` MUST clear the synthetic button bit.
    Default 50 ms is a scheduling experiment, not validated ECU timing. A
    100 ms polling receiver may miss the entire pulse depending on phase;
    a per-frame receiver may instead count repeated held frames as multiple
    presses. Neither receiver model is assumed to match the vehicle.
    Session reset is allowed only after independently verified cruise disengage.
    """
    def __init__(self, observed_mph: int, *, floor_mph=25, burst=3, press_s=.05,
                 minus_interval=.25, plus_interval=.35, ceiling_mph=None):
        if not all(math.isfinite(v) for v in (observed_mph, floor_mph, press_s, minus_interval, plus_interval)):
            raise ValueError('finite configuration required')
        if observed_mph != int(observed_mph) or floor_mph != int(floor_mph) or not 0 <= floor_mph <= observed_mph <= 95:
            raise ValueError('whole mph floor <= initial ceiling <= 95 required')
        if not isinstance(burst, int) or burst < 1 or not 0 < press_s < min(minus_interval, plus_interval):
            raise ValueError('positive finite pulse and release intervals required')
        self.ceiling = observed_mph if ceiling_mph is None else ceiling_mph
        if not isinstance(self.ceiling, int) or not observed_mph <= self.ceiling <= 95:
            raise ValueError("whole mph observed <= ceiling <= 95 required")
        self.floor = floor_mph
        self.observed = observed_mph
        self.burst = burst
        self.press_s = press_s
        self.intervals = {-1: minus_interval, 1: plus_interval}
        self.debt = {-1: 0, 1: 0}
        self.direction = 0
        self.press_until = -math.inf
        self.next_press = -math.inf
        self.last_now = -math.inf
        self.pressed = False
        self.fault = False

    @property
    def pending(self):
        return sum(self.debt.values())

    @property
    def up_debt(self):
        return self.debt[1]

    @property
    def down_debt(self):
        return self.debt[-1]

    def update(self, now, observed_mph, target_mph, *, allowed=True, manual=False):
        def result(button, phase, reason):
            return Output(button, phase, reason, self.pending, self.ceiling)
        valid = all(math.isfinite(v) for v in (now, observed_mph, target_mph))
        if not valid or now < self.last_now:
            self.fault = True
        if valid:
            self.last_now = now
        if manual:
            self.fault = True  # Caller must end/reconcile ownership; never recapture a lowered ceiling.
        if self.fault:
            self.pressed = False
            return result(None, 'fault', 'invalid-input-or-manual-intervention')
        delta = observed_mph - self.observed
        if observed_mph != int(observed_mph) or not self.floor <= observed_mph <= self.ceiling:
            self.fault = True
        elif delta:
            ack_direction = 1 if delta > 0 else -1
            if abs(delta) > self.debt[ack_direction]:
                self.fault = True
            else:
                self.debt[ack_direction] -= int(abs(delta))
                self.observed = observed_mph
        if self.fault:
            self.pressed = False
            return result(None, 'fault', 'unexplained-setpoint-change')
        if not allowed:
            if self.pressed:
                self.next_press = max(self.next_press, now + self.intervals[self.direction] - self.press_s)
            self.pressed = False
            # Keep credits and rate limits through pauses/dropouts.
            return result(None, 'release', 'inhibited')
        target = min(self.ceiling, max(self.floor, int(target_mph)))
        wanted = (target > self.observed) - (target < self.observed)
        if self.pressed:
            # A changed demand (or an ACK reaching it) must clear the old
            # level immediately; already-issued movement credit stays owed.
            if wanted != self.direction:
                self.pressed = False
                self.next_press = max(self.next_press, now + self.intervals[self.direction] - self.press_s)
                return result(None, 'release', 'demand-changed')
            if now < self.press_until:
                return result('SET+' if self.direction > 0 else 'SET-', 'press', 'holding-pulse')
            self.pressed = False
            self.next_press = max(self.next_press, now + self.intervals[self.direction] - self.press_s)
            return result(None, 'release', 'pulse-complete')
        if not wanted:
            return result(None, 'idle', 'target-observed')
        if now < self.next_press:
            return result(None, 'release', 'release-dwell')
        if self.debt[wanted] >= self.burst:
            return result(None, 'wait', 'burst-awaiting-progress')
        # Every emitted pulse remains a potential movement until observed. No
        # timeout may forgive its credit: delayed pulses could all still apply.
        projected = self.observed + wanted * (self.debt[wanted] + 1)
        if not self.floor <= projected <= self.ceiling or (projected - target) * wanted > 0:
            return result(None, 'wait', 'reserved-target-or-boundary')
        self.direction = wanted
        self.debt[wanted] += 1
        self.pressed = True
        self.press_until = now + self.press_s
        self.next_press = now + self.intervals[wanted]
        return result('SET+' if wanted > 0 else 'SET-', 'press', 'new-pulse')
