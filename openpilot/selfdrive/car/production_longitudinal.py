"""Bounded producer ownership for the optional Navigator V5 wire protocol.

The actual PID is never assigned here. The nominal controller has its own
integral, recovery policy and feedback history, fed only by serialized consumer
facts. Source IDs are protocol revisions, not authentication of local IPC.
"""
import math
import struct

from opendbc.car.ford.longitudinal_coordination import freeze_integral
from opendbc.car.ford.longitudinal_production_contract import (
  CONTRACT_ID, SOURCE_ID, REQUEST_VERSION, PREFIX, FRESH_NS,
  ProductionIssuedRegistry, nominal_from_wire,
)
from opendbc.car.ford.longitudinal_reference_feedback import ReferenceIntegral


def _f32(value):
  return struct.unpack('f', struct.pack('f', value))[0]


class ProductionIntegral(ReferenceIntegral):
  def __init__(self, producer):
    self.producer = producer
    super().__init__()

  def nominal(self, rc, t, owner, seq, error, target, wanted, speed, phase, fb, enabled=False):
    if not str(getattr(fb, 'reason', '')).startswith(PREFIX):
      return super().nominal(rc, t, owner, seq, error, target, wanted, speed, phase, fb, enabled=enabled)

    actual = freeze_integral(t, owner, seq, error, target, wanted, speed, phase, fb, enabled=enabled)
    p, current = self.producer, self.current
    now_ns = round(t * 1e9)
    scoped = bool(
      enabled and phase == 'pid' and rc.valid and rc.sample is not None
      and rc.sample[:3] == (t, owner, seq)
      and current.now_ns == now_ns and current.epoch == owner and current.sequence == seq
      and current.eligible and current.pose_valid and p.eligible
      and p.identity == (now_ns, owner, seq) and p.target == target
      and all(math.isfinite(v) for v in (t, error, target, wanted, speed, current.pitch))
      and 20 * .44704 <= speed <= 30 * .44704 and 0 < target <= .2 and .02 < wanted <= .3)
    witness = p.registry.verify(fb, now_ns=now_ns, cutoff_ns=p.cutoff_ns,
                                epoch=owner, sequence=seq) if scoped else None
    reference = None
    reason = p.registry.last_reason if scoped else 'current_production_scope'
    if witness is not None:
      offset = witness.grade_offset
      age = (now_ns - witness.selected_ns) / 1e9
      # The published grade is Float32. Subtract a full ULP before the
      # canonical decay/fade rather than inventing favorable rounding.
      lower = max(0., offset - (2. ** (math.frexp(offset)[1] - 24) if offset else 0.))
      lower = max(0., lower - .005 * age) * max(0., min(1., (speed - 3) / 2))
      net = wanted + 9.81 * math.sin(current.pitch) + lower
      if (offset == p.wire_grade and math.isfinite(net) and net >= -.14
          and not fb.brake and not fb.precharge and not fb.fault and fb.valid):
        reference = freeze_integral(t, owner, seq, error, target, wanted, speed, phase,
                                    witness.nominal_feedback(), enabled=enabled)
      else:
        reason = 'production_grade_or_actual_scope'
    owned = reference is not None
    self.counts['owned' if owned else 'declined'] += 1
    self.diagnostic = {'owned': owned, 'nominal_frozen': bool(actual), 'reference_frozen': reference,
                       'receipt_reason': 'production_history' if owned else reason,
                       'selected_ns': witness.selected_ns if witness is not None else None}
    return bool(reference if owned else actual)


class ProductionProducer:
  def __init__(self, actual, CP, *, run_id, source_id=SOURCE_ID):
    if type(run_id) is not int or not 0 < run_id < 2**64:
      raise ValueError('production run identity must be a positive UInt64')
    from openpilot.selfdrive.controls.lib.longcontrol import LongControl
    self.nominal = LongControl(CP)
    self.registry = ProductionIssuedRegistry(source_id)
    self.source_id, self.run_id = source_id, run_id
    self.identity = None
    self.target = 0.
    self.cutoff_ns = 0
    self.wire_grade = float('nan')
    self.eligible = False
    self.actual = actual

  def prepare(self, CC, CS, target, should_stop, limits, feedback_wire, *, now_ns, cutoff_ns,
              source_valid, urgent, no_lead, radar_ns, radar_valid):
    q = CC.navigatorLongitudinal
    nominal_feedback = nominal_from_wire(feedback_wire, source_id=self.source_id, run_id=self.run_id,
                                         now_ns=now_ns, cutoff_ns=cutoff_ns,
                                         epoch=int(q.epoch), sequence=int(q.sequence))
    before_phase = self.nominal.long_control_state
    nominal_raw = float(self.nominal.update(
      CC.longActive, CS, target, should_stop, limits, feedback=nominal_feedback,
      now=now_ns / 1e9, epoch=int(q.epoch), sequence=int(q.sequence),
      coordinated=True, source_valid=source_valid, urgent=urgent, target_ns=int(q.targetMonoTime)))
    q.productionContractId = CONTRACT_ID
    q.productionSourceId = self.source_id
    q.productionRunId = self.run_id
    # Mirror the canonical CarControl.Actuators Float32 boundary exactly;
    # retain the separate nominal PID's original full-precision state.
    q.nominalRawAcceleration = _f32(nominal_raw) if math.isfinite(nominal_raw) else 0.
    q.nominalLongControlState = before_phase
    q.nominalValid = math.isfinite(nominal_raw)
    q.productionSourceValid = bool(source_valid)
    q.productionUrgent = bool(urgent)
    q.noLead = bool(no_lead and radar_valid)
    q.radarMonoTime = radar_ns if type(radar_ns) is int and 0 < radar_ns <= now_ns else 0
    q.feedbackCutoffMonoTime = cutoff_ns if type(cutoff_ns) is int and 0 < cutoff_ns <= now_ns else 0
    q.productionPoseMonoTime = int(q.poseMonoTime)
    q.allocatorVersion = REQUEST_VERSION
    self.identity = (now_ns, int(q.epoch), int(q.sequence))
    self.target = target
    self.cutoff_ns = int(q.feedbackCutoffMonoTime)
    self.wire_grade = float(feedback_wire.gradeOffset)
    self.eligible = bool(source_valid and not urgent and q.targetValid
                         and 0 < q.targetMonoTime <= now_ns
                         and now_ns - q.targetMonoTime <= FRESH_NS
                         and q.noLead and 0 < q.radarMonoTime <= now_ns
                         and now_ns - q.radarMonoTime <= FRESH_NS)

  def finalize_control(self, CC):
    if self.identity != (int(CC.navigatorLongitudinal.producerMonoTime),
                         int(CC.navigatorLongitudinal.epoch), int(CC.navigatorLongitudinal.sequence)):
      return False
    return self.registry.issue(CC)
