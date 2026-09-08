"""Bounded transport evidence for the inhibited core path-angle experiment.

No safety oracle or missing-echo acceptance inference. Runtime data uses a unique
process stream identity, which the enclosing route logger preserves alongside it.
"""
from collections import deque
import json
import math

TARGET = 0x3D6


class RuntimeObserver:
  def __init__(self, route_id: str, provenance: str, capacity: int = 2048):
    if not route_id or provenance not in ('live', 'recorded', 'synthetic') or not 1 <= capacity <= 8192:
      raise ValueError('Invalid observer identity or capacity')
    self.route_id, self.provenance = route_id, provenance
    self.publications: deque = deque(maxlen=capacity)
    self.coverage_incomplete = False
    self.health: dict = {}
    self.health_gaps: set = set()
    self.last_order = -1

  def consume(self, e):
    if any(k in e for k in ('accepted', 'failed_checks', 'reason_bits')) or any(k.startswith(('compiled', 'oracle')) for k in e):
      raise ValueError('Safety oracle cannot enter runtime observation')
    if type(e['order']) is not int or e['order'] <= self.last_order:
      raise ValueError('Observation order must increase')
    self.last_order = e['order']
    r = dict(e, event_type=e['kind'], receiver_acceptance='unknown', candidate_orders=[], attribution='not_applicable',
             publication_to_observation_ns=None, coverage_incomplete=self.coverage_incomplete,
             direct_steering_rejection=False, counter_delta=None, counter_interpretation='unknown',
             permission_lost=False, configuration_changed=False, health_coverage_gap=False)
    if e.get('route_id') != self.route_id or e.get('provenance') != self.provenance:
      r['event_type'] = 'identity_mismatch'
      return r
    if e['kind'] == 'health':
      self._health(e, r)
      return r
    if e.get('valid') is not True or type(e.get('t_ns')) is not int or e['t_ns'] < 0:
      r['event_type'] = 'invalid_packet'
      return r
    if e['kind'] not in ('published', 'returned', 'rejected'):
      return r
    try:
      data = bytes.fromhex(e['data_hex'])
    except (ValueError, TypeError):
      r['event_type'] = 'invalid_packet'
      return r
    if type(e.get('bus')) is not int or not 0 <= e['bus'] <= 7 or len(data) != e.get('dlc'):
      r['event_type'] = 'invalid_packet'
      return r
    if e['address'] != TARGET:
      r['event_type'] = 'other_rejection' if e['kind'] == 'rejected' else 'other_traffic'
      return r
    key = (e['bus'], e['address'], e['dlc'], data)
    if e['kind'] == 'published':
      if len(self.publications) == self.publications.maxlen:
        self.coverage_incomplete = True
      self.publications.append((key, e['order'], e['t_ns']))
      r['coverage_incomplete'] = self.coverage_incomplete
      r['attribution'] = 'publication_only'
      return r
    matches = [p for p in self.publications if p[0] == key and p[2] <= e['t_ns']]
    r['candidate_orders'] = [p[1] for p in matches]
    r['attribution'] = 'incomplete' if self.coverage_incomplete else 'ambiguous' if len(matches) > 1 else 'unique' if matches else 'unmatched'
    if r['attribution'] == 'unique':
      r['publication_to_observation_ns'] = e['t_ns'] - matches[0][2]
    r['direct_steering_rejection'] = e['kind'] == 'rejected'
    r['wire_mode'] = (data[0] >> 4) & 7 if len(data) == 8 else None
    return r

  def _health(self, e, r):
    idx = e.get('panda_index')
    if type(idx) is not int or not 0 <= idx < 8:
      r['event_type'] = 'invalid_health'
      return
    required = ('safetyTxBlocked', 'safetyParam', 'alternativeExperience', 't_ns')
    if (e.get('valid') is not True or any(type(e.get(k)) is not int or e[k] < 0 for k in required)
        or e['safetyTxBlocked'] > 0xFFFFFFFF or not isinstance(e.get('safetyModel'), str) or not e['safetyModel']
        or any(type(e.get(k)) is not bool for k in ('controlsAllowed', 'safetyRxChecksInvalid'))):
      r['event_type'] = 'invalid_health'
      self.health_gaps.add(idx)
      return
    prev = self.health.get(idx)
    if prev and e['t_ns'] <= prev['t_ns']:
      r['event_type'] = 'out_of_order_health'
      self.health_gaps.add(idx)
      return
    r['health_coverage_gap'] = idx in self.health_gaps
    self.health_gaps.discard(idx)
    r['counter_interpretation'] = 'initial_observation'
    if prev:
      r['permission_lost'] = prev['controlsAllowed'] and not e['controlsAllowed']
      r['configuration_changed'] = any(e[k] != prev[k] for k in ('safetyModel', 'safetyParam', 'alternativeExperience'))
      delta = e['safetyTxBlocked'] - prev['safetyTxBlocked']
      r['health_observation_interval_ns'] = e['t_ns'] - prev['t_ns']
      if delta < 0:
        r['counter_interpretation'] = 'discontinuity_reset_or_wrap_unknown'
      elif r['configuration_changed']:
        r['counter_interpretation'] = 'configuration_changed_continuity_unknown'
      else:
        r['counter_delta'] = delta
        r['counter_interpretation'] = 'aggregate_increase' if delta else 'no_observed_increase'
    self.health[idx] = e.copy()


class RuntimeBridge:
  def __init__(self, mode, expected_pandas, route_id, provenance, capacity=2048):
    if mode not in ('shadow', 'requested'):
      raise ValueError('Runtime bridge requires explicit experiment mode')
    self.mode = mode
    self.active_request = False
    self.controls_ready = False
    self.configuration_armed = False
    self.observer = RuntimeObserver(route_id, provenance, capacity)
    self.expected_pandas = tuple(expected_pandas)
    self.pending: deque = deque(maxlen=16)
    self.dropped_diagnostics = 0
    self.order = 0
    self.fault_reason = 'physical_enforcement_unvalidated' if mode == 'requested' else None

  @property
  def disengagement_requested(self):
    return self.mode == 'requested'

  def observe(self, event):
    row = self.observer.consume(event)
    self.order = event['order'] + 1
    reason = None
    if row['direct_steering_rejection'] and row.get('wire_mode') != 0:
      reason = 'direct_steering_rejection'
    elif row['event_type'] in ('invalid_health', 'out_of_order_health', 'identity_mismatch'):
      if self.configuration_armed or row['event_type'] == 'identity_mismatch':
        reason = row['event_type']
    elif event['kind'] == 'health':
      actual = (event['safetyModel'], event['safetyParam'], event['alternativeExperience'])
      idx = event['panda_index']
      mismatch = idx >= len(self.expected_pandas) or actual != self.expected_pandas[idx]
      if self.controls_ready and not mismatch:
        matching = [self.observer.health.get(i) for i in range(len(self.expected_pandas))]
        self.configuration_armed |= all(h is not None and (h['safetyModel'], h['safetyParam'], h['alternativeExperience']) == expected
                                        for h, expected in zip(matching, self.expected_pandas, strict=True))
      if mismatch and self.configuration_armed:
        reason = 'configuration_mismatch'
      elif self.configuration_armed and self.active_request and (not event['controlsAllowed'] or event['safetyRxChecksInvalid']):
        reason = 'permission_unavailable'
    if reason and self.fault_reason is None:
      self.fault_reason = reason
    # Keep bounded pending evidence; the original can/sendcan/pandaStates remain
    # logged by normal services. Never turn diagnostic overflow into attribution.
    if event['kind'] == 'health' or event.get('address') == TARGET or event['kind'] == 'rejected':
      if len(self.pending) == self.pending.maxlen:
        self.dropped_diagnostics += 1
      self.pending.append(row)
    return row

  def packet(self, kind, t_ns, valid, address, data, bus, src=None):
    return self.observe({'route_id': self.observer.route_id, 'provenance': self.observer.provenance,
                             'order': self.order, 'kind': kind, 't_ns': t_ns, 'valid': valid, 'address': address,
                             'data_hex': bytes(data).hex(), 'dlc': len(data), 'bus': bus, 'src': src})

  def panda(self, t_ns, valid, index, state):
    return self.observe({'route_id': self.observer.route_id, 'provenance': self.observer.provenance,
                             'order': self.order, 'kind': 'health', 't_ns': t_ns, 'valid': valid, 'panda_index': index,
                             'safetyTxBlocked': int(state.safetyTxBlocked), 'controlsAllowed': bool(state.controlsAllowed),
                             'safetyRxChecksInvalid': bool(state.safetyRxChecksInvalid), 'safetyModel': str(state.safetyModel),
                             'safetyParam': int(state.safetyParam), 'alternativeExperience': int(state.alternativeExperience)})

  def diagnostics(self):
    result = {'schema_version': 1, 'stream_id': self.observer.route_id, 'provenance': self.observer.provenance,
              'fault_reason': self.fault_reason, 'inhibited': self.disengagement_requested,
              'transport': list(self.pending), 'dropped_diagnostics': self.dropped_diagnostics,
              'coverage_incomplete': self.observer.coverage_incomplete, 'configuration_armed': self.configuration_armed}
    self.pending.clear()
    return result


def diagnostic_json(controller, bridge):
  d = bridge.diagnostics()
  d['controller'] = controller.navigator_a3.diagnostic
  d['controller_proposal_provenance'] = 'synthetic'
  d['published_frames_provenance'] = 'synthetic' if bridge.observer.provenance == 'recorded' else 'live'
  # Exact candidates can be reconstructed from normal CAN logs when long lists
  # would make real-time diagnostic serialization unbounded in practice.
  for row in d['transport']:
    row['retained_candidate_count'] = len(row['candidate_orders'])
    if len(row['candidate_orders']) > 16:
      row['candidate_orders'] = row['candidate_orders'][:16]
      row['candidate_list_truncated'] = True
  nonfinite = False
  def sanitize(value):
    nonlocal nonfinite
    if isinstance(value, float) and not math.isfinite(value):
      nonfinite = True
      return None
    if isinstance(value, dict):
      return {k: sanitize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
      return [sanitize(v) for v in value]
    return value
  d = sanitize(d)
  d['nonfinite_values_present'] = nonfinite
  return json.dumps(d, separators=(',', ':'), allow_nan=False)


def add_inhibition_event(car_output, add_event, steering_unavailable):
  """Existing immediate-disable/no-entry alert truthfully represents blocked steering."""
  if car_output.navigatorA3.mode == 'requested' and car_output.navigatorA3.inhibited:
    add_event(steering_unavailable)


def prepare_controller(ci, cs, sm, bridge):
  # Parser timestamps advance only for successfully parsed message samples.
  parser = ci.can_parsers.get('pt')
  timestamps = []
  if parser is not None:
    timestamps = [parser.ts_nanos.get(msg, {}).get(signal, 0) for msg, signal in
                  (('Yaw_Data_FD1', 'VehYaw_W_Actl'), ('BrakeSysFeatures', 'Veh_V_ActlBrk'))]
  measurement_ns = min(timestamps) if timestamps and all(t > 0 for t in timestamps) else None
  ci.CC.set_navigator_a3_evidence(source_ns=int(sm.logMonoTime['carControl']),
                                source_valid=bool(sm.all_checks(['carControl'])),
                                measurement_ns=measurement_ns,
                                measurement_valid=bool(cs.canValid and not cs.vehicleSensorsInvalid and measurement_ns is not None),
                                fault_reason=bridge.fault_reason or (None if bridge.configuration_armed else 'configuration_pending'))


def control_for_apply(cc, bridge):
  if not bridge.disengagement_requested:
    return cc
  from opendbc.car.structs import car
  blocked = car.CarControl.new_message(**cc.to_dict())
  blocked.enabled = False
  blocked.latActive = False
  blocked.longActive = False
  return blocked.as_reader()


def observe_publication(bridge, publication, replay=False):
  # REPLAY produces new commands on a host clock, not original recorded sendcan.
  # Their bytes remain in controller diagnostics; never correlate old echoes to them.
  if replay:
    return
  for packet in publication.sendcan:
    bridge.packet('published', int(publication.logMonoTime), bool(publication.valid),
                  packet.address, packet.dat, int(packet.src), int(packet.src))
