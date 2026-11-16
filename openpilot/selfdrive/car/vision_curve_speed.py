"""Vision curve geometry and acknowledged OEM cruise setpoint decisions."""
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
import math

MPH = 0.44704
MAX_REDUCTION_MPH = 5.


@dataclass(frozen=True)
class Curve:
    curvature: float
    direction: int
    first_horizon_s: float
    lane_confidence: float


def measure_curve(model):
    """Look for a same-direction bend in the next five seconds."""
    orientation = model.orientationRate
    times = list(orientation.t)
    yaw = list(orientation.z)
    velocity = list(model.velocity.x)
    if not (len(times) == len(yaw) == len(velocity)) or len(times) < 6:
        return None

    best = None
    probs = model.laneLineProbs
    confidence = min(probs[1], probs[2]) if len(probs) >= 3 else 0.
    run = []
    direction = 0
    first = previous = 0.
    # A neutral sentinel closes a run at the end of the input.
    times.append(math.inf)
    yaw.append(0.)
    velocity.append(0.)
    for t, rate, speed in zip(times, yaw, velocity, strict=True):
        curvature = rate / speed if .6 <= t <= 5. and speed >= 5. and math.isfinite(rate) and math.isfinite(speed) else 0.
        sign = (1 if curvature > 0. else -1) if abs(curvature) >= .0009 else 0
        if sign != direction:
            if len(run) >= 4 and previous - first >= .65:
                run.sort()
                index = .8 * (len(run) - 1)
                lower = int(index)
                fraction = index - lower
                lo, hi = run[lower], run[lower + 1]
                spread = hi - lo
                magnitude = lo + spread * fraction if fraction < .5 else hi - spread * (1. - fraction)
                if best is None or magnitude > best.curvature:
                    best = Curve(magnitude, direction, first, confidence)
            run.clear()
            first = t
            direction = sign
        if sign:
            run.append(abs(curvature))
        previous = t
    return best


def _corridor_traces(model):
    lines = model.laneLines
    if len(lines) < 3:
        return ()
    traces = []
    for line in (model.position, lines[1], lines[2]):
        xs, ys = list(line.x), list(line.y)
        if len(xs) < 4 or len(xs) != len(ys):
            return ()
        previous = -math.inf
        for x, y in zip(xs, ys, strict=True):
            if x < previous or not (math.isfinite(x) and math.isfinite(y)):
                return ()
            previous = x
        traces.append((xs, ys))
    return traces


def _interp(at, trace):
    xs, ys = trace
    right = bisect_right(xs, at)
    if right == 0:
        return ys[0]
    if right == len(xs) or xs[right - 1] == at:
        return ys[right - 1]
    left = right - 1
    return (ys[right] - ys[left]) / (xs[right] - xs[left]) * (at - xs[left]) + ys[left]


def corridor_veto(model, traces=None):
    """Report predicted path/lane disagreement, a proxy for uncertain routing."""
    traces = _corridor_traces(model) if traces is None else traces
    if not traces:
        return 'corridor_unavailable'
    near = max(15., *(x[0] for x, _ in traces))
    far = min(85., *(x[-1] for x, _ in traces))
    if far - near < 40.:
        return 'corridor_unavailable'

    spacing = (far - near) / 6.
    departed = previous = False
    for i in range(7):
        at = far if i == 6 else near + i * spacing
        path, left, right = (_interp(at, trace) for trace in traces)
        width = right - left
        if not 2.1 <= width <= 5.2:
            return 'corridor_width'
        outside = abs(path - (left + right) / 2.) > max(1., .38 * width)
        departed |= previous and outside
        previous = outside
    return 'corridor_departure' if departed else None


class BranchContext:
    """Causal proxy for a branch approach using logged set speed and lane geometry."""

    def __init__(self):
        self.set_history = deque()
        self.widening_since = None
        self.last_wide_at = -math.inf
        self.widened_at = -math.inf
        self.ambiguous = False
        self.clear_since = None

    def update(self, now, ceiling, model, curve, traces):
        self.set_history.append((now, ceiling))
        while now - self.set_history[0][0] > 15.:
            self.set_history.popleft()

        probs = model.laneLineProbs
        reliable = traces and len(probs) >= 3 and min(probs[1], probs[2]) >= .55
        if reliable and max(x[0] for x, _ in traces) <= 55. and min(x[-1] for x, _ in traces) >= 85.:
            _, left, right = traces
            if max(_interp(at, right) - _interp(at, left) for at in (55., 85.)) >= 3.75:
                if now - self.last_wide_at > .15:
                    self.widening_since = now
                self.last_wide_at = now
                if now - self.widening_since >= .35:
                    self.widened_at = now
            if (curve is not None and now - self.widened_at <= 6. and min(x[-1] for x, _ in traces) >= 120.
                    and max(value for _, value in self.set_history) - ceiling >= 5.):
                path, left, right = (_interp(120., trace) for trace in traces)
                offset = path - (left + right) / 2.
                edges = model.roadEdgeStds
                if len(edges) >= 2:
                    inner = 1 if curve.direction > 0 else 0
                    if offset * curve.direction >= right - left and edges[inner] < .6 and edges[1 - inner] > 1.2:
                        self.ambiguous = True
        if curve is not None and ceiling - math.sqrt(1.8 / curve.curvature) / MPH >= 2.:
            self.clear_since = None
        elif self.ambiguous:
            self.clear_since = now if self.clear_since is None else self.clear_since
            if now - self.clear_since >= 1.5:
                self.ambiguous = False
        return self.ambiguous


class VirtualSetSpeed:
    """Shadow cluster with a one-press-per-0.75s, one unit per press assumption."""

    def __init__(self, metric=False, min_set_mph=0.):
        self.min_set_mph = min_set_mph
        self.step_mph = 1/1.609344 if metric else 1.
        self.reset()

    def reset(self, *, preserve_holds=False):
        self.value = None
        self.previous_set = None
        self.last_step = -math.inf
        if not preserve_holds:
            self.manual_until = self.override_until = -math.inf
        self.detect_since = None
        self.clear_since = None
        self.curve_active = False
        self.previous_direction = None
        self.motion = deque()

    def _decel_reaches_target(self, now, speed_mph, accel_mps2, target, seconds_ahead):
        if not math.isfinite(accel_mps2) or accel_mps2 > -.25:
            self.motion.clear()
            return False
        self.motion.append((now, speed_mph, accel_mps2))
        while now - self.motion[0][0] > .9:
            self.motion.popleft()
        elapsed = now - self.motion[0][0]
        if elapsed < .6:
            return False
        measured_slope = (self.motion[0][1] - speed_mph) * MPH / elapsed
        weakest_braking = min(-sample[2] for sample in self.motion)
        # Predict only half the weaker observed deceleration, over at most two seconds.
        predicted_decrease = .5 * min(measured_slope, weakest_braking, 1.) * min(seconds_ahead, 2.)
        return predicted_decrease > 0. and speed_mph - predicted_decrease / MPH <= target

    def update(self, now, ceiling, curve, eligible, inhibit, manual, *, speed_mph, accel_mps2,
               manual_observed_at=-math.inf, override_observed_at=-math.inf):
        # Consume observations before checking eligibility. A release or brief
        # driver input during missing model samples persists into re-admission.
        self.manual_until = max(self.manual_until, manual_observed_at + 4.)
        self.override_until = max(self.override_until, override_observed_at + 1.5)
        if not eligible:
            self.reset(preserve_holds=True)
            return None, None, None, 'ineligible'
        if self.value is None:
            self.value = ceiling
        manual = manual or (self.previous_set is not None and abs(ceiling - self.previous_set) >= self.step_mph * .75)
        self.previous_set = ceiling
        if manual:
            self.manual_until = now + 4.
            self.value = ceiling
            self.last_step = now
            self.detect_since = self.clear_since = None
            self.curve_active = False
        self.value = min(self.value, ceiling)
        raw = ceiling if curve is None else min(ceiling, math.sqrt(1.8 / curve.curvature) / MPH)
        # Ignore tiny reductions, bound the experiment below the recorded set speed.
        proposal = min(ceiling, max(self.min_set_mph, ceiling - MAX_REDUCTION_MPH, raw))
        if inhibit or now < self.manual_until or now < self.override_until:
            self.motion.clear()
            self.detect_since = self.clear_since = None
            self.curve_active = False
            self.previous_direction = None
            mode = 'manual-hold' if now < self.manual_until else 'inhibited' if inhibit else 'override-hold'
            return raw, self.value, None, mode
        already_slowing_enough = self._decel_reaches_target(
            now, speed_mph, accel_mps2, proposal, curve.first_horizon_s if curve is not None else 0.)
        trigger = curve is not None and ceiling - proposal >= 2.
        if trigger:
            if self.detect_since is None or curve.direction != self.previous_direction:
                self.detect_since = now
                self.curve_active = False
            self.previous_direction = curve.direction
            self.clear_since = None
            self.curve_active |= now - self.detect_since >= .6
        else:
            self.detect_since = None
            self.clear_since = now if self.clear_since is None else self.clear_since
            if now - self.clear_since >= 1.5:
                self.curve_active = False
        # No restores before the release dwell. Never increase toward a new proposal inside a curve.
        target = min(proposal, self.value) if self.curve_active and trigger else self.value
        if not self.curve_active and not trigger and now - self.clear_since >= 1.5:
            target = ceiling
        step = None
        if now - self.last_step >= .75:
            if self.value - target >= self.step_mph and self.value - self.step_mph >= ceiling - MAX_REDUCTION_MPH:
                if speed_mph <= target:
                    return raw, self.value, None, 'already-at-target'
                if already_slowing_enough:
                    return raw, self.value, None, 'recorded-deceleration'
                self.value -= self.step_mph
                step = 'SET-'
            elif not self.curve_active and target - self.value >= self.step_mph * .9:
                self.value = min(ceiling, self.value + self.step_mph)
                step = 'SET+'
            if step:
                self.last_step = now
        return raw, self.value, step, 'curve' if self.curve_active else 'hold'


# A recorded setpoint is external input in replay. Live setpoints may also
# acknowledge our last request, so keep the driver ceiling independent.
class AcknowledgedCurveSpeed:
    ACK_TIMEOUT = 1.0
    MIN_PRESS_INTERVAL = 1.0
    SPEED_TOLERANCE = 0.2

    def __init__(self):
        self.virtual = VirtualSetSpeed(min_set_mph=25.)
        self.branch = BranchContext()
        self.ceiling = None
        self.observed = None
        self.pending = None
        self.pending_at = -math.inf
        self.last_press = -math.inf
        self.override_until = -math.inf
        self.fault = False

    def reset(self, *, preserve_branch=False):
        self.virtual.reset()
        if not preserve_branch:
            self.branch = BranchContext()
        self.ceiling = self.observed = self.pending = None

    def update(self, now, *, engaged, valid, manual, override, set_speed, speed, accel, model):
        # Release the fault only on confirmed disengagement, not on stale data.
        if valid and not engaged:
            self.fault = False
            self.reset()
            return None
        if not valid:
            self.fault |= self.pending is not None or (self.ceiling is not None and self.observed < self.ceiling)
            self.reset()
            return None
        branch_blocked = False
        curve = None
        traces = ()
        if model is not None:
            curve = measure_curve(model)
            traces = _corridor_traces(model)
            external_change = self.pending is None and self.observed is not None and abs(set_speed - self.observed) > self.SPEED_TOLERANCE
            branch_ceiling = set_speed if self.ceiling is None or external_change or manual else self.ceiling
            branch_blocked = self.branch.update(now, branch_ceiling, model, curve, traces)
        if manual or override:
            self.override_until = max(self.override_until, now + (4.0 if manual else 1.5))
            self.fault |= self.pending is not None
            self.reset(preserve_branch=True)
            return None
        if self.fault or now < self.override_until:
            return None
        if self.pending is not None:
            if now - self.pending_at >= self.ACK_TIMEOUT:
                self.fault = True
                self.reset()
                return None
            if abs(set_speed - self.pending) <= self.SPEED_TOLERANCE:
                self.observed = set_speed
                self.pending = None
            elif abs(set_speed - self.observed) > self.SPEED_TOLERANCE:
                self.fault = True
                self.reset()
                return None
            else:
                return None
        if self.ceiling is None or abs(set_speed - self.observed) > self.SPEED_TOLERANCE:
            if self.ceiling is not None:
                self.override_until = now + 4.0
            self.reset(preserve_branch=True)
            # The Ford CAN-FD setpoint is reported in whole mph.
            self.ceiling = self.observed = round(set_speed)
            return None
        if model is None or now - self.last_press < self.MIN_PRESS_INTERVAL:
            return None
        blocked = branch_blocked or corridor_veto(model, traces) or len(model.laneLineProbs) < 3 or min(model.laneLineProbs[1], model.laneLineProbs[2]) < .55
        _, target, step, _ = self.virtual.update(now, self.ceiling, curve, True, blocked, False,
                                                speed_mph=speed, accel_mps2=accel)
        if step is not None:
            self.pending = target
            self.pending_at = self.last_press = now
        return step
