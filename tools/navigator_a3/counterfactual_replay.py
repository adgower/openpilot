"""Offline full-rate counterfactual, isolated from the recorded faulted lifecycle.

Only direct recorded steering-rejection inhibition is separated. Source validity
is independently inferred from exact native carControl events and ten preceding
intervals <=100ms; this is NOT a reproduction of SubMaster receive-time/frequency
checks. All original measurement evidence and other Inputs fields are unchanged.
No transport event can acknowledge/reject synthetic bytes or permit activation.
"""
import argparse
from collections import Counter, deque
import copy
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path

from opendbc.can import CANPacker
from opendbc.car.ford.navigator_a3 import Inputs, State, PROFILES, update, encode_offline


def replay_rows(rows):
  states = {}
  schedulers = {}
  previous = {}
  packer = CANPacker('ford_lincoln_base_pt')
  for row in rows:
    historical = copy.deepcopy(row['diagnostic'])
    result = {'route_id': row['route_id'], 't_ns': row['t_ns'], 'historical': historical,
              'car_output_event_valid': row.get('car_output_event_valid'),
              'source_evidence': copy.deepcopy(row.get('source_evidence')), 'synthetic': None,
              'synthetic_frame': None, 'unsupported_reason': None,
              'synthetic_transport_acceptance': 'unknown', 'physical_activation_allowed': False,
              'label': 'offline counterfactual; inferred source checks; no vehicle response prediction'}
    c = historical.get('controller') or {}
    identity = row['route_id']
    try:
      if row.get('car_output_event_valid') is not True:
        raise ValueError('invalid_car_output_event')
      sample = Inputs(**c['input'])
      profile = c['config']['profile']
      if profile not in PROFILES:
        raise ValueError('unsupported_profile')
      ev = row.get('source_evidence') or {}
      for field in ('exact_source_event', 'values_match', 'cadence_supported'):
        if ev.get(field) is not True:
          raise ValueError(field)
      if type(ev.get('event_valid')) is not bool:
        raise ValueError('unknown_event_validity')
      if sample.now_ns <= previous.get(identity, -1):
        raise ValueError('duplicate_or_regressing_controller_time')
      if identity in states and states[identity][0] != profile:
        raise ValueError('profile_changed')
      scheduler_version = c.get('schema_version', 1) >= 3
      counter = row.get('counter')
      if not scheduler_version and (type(counter) is not int or not 0 <= counter <= 15):
        raise ValueError('missing_recorded_counter')
      # Other original invalidity stays invalid. Never modify the historical dict.
      if historical.get('schema_version', 1) >= 2 or c.get('schema_version', 1) >= 2:
        # Version 2 already separates transport evidence from calculation gates.
        # Missing explicit eligibility is unknown; never apply the legacy inference.
        inferred_valid = (ev['event_valid'] and sample.valid and c.get('calculation_eligible') is True
                          and 'calculation_fault_reason' in c and c['calculation_fault_reason'] is None)
        result['validity_policy'] = 'version2_explicit_calculation_gates'
      else:
        inferred_valid = ev['event_valid'] and (sample.valid or c.get('evidence_fault_reason') == 'direct_steering_rejection')
        result['validity_policy'] = 'legacy_version1_inferred_source_validity'
      synthetic_input = replace(sample, valid=inferred_valid)
      if scheduler_version:
        from opendbc.car.ford.navigator_a3_scheduler import ShadowScheduler
        scheduler = schedulers.setdefault(identity, ShadowScheduler())
        decision = scheduler.step(PROFILES[profile], synthetic_input)
        proposal, counter = decision.output, decision.proposal_counter
        result['synthetic_scheduler'] = {k: v for k, v in asdict(decision).items() if k != 'output'}
        states[identity] = (profile, scheduler.state)
      else:
        state = states.get(identity, (profile, State()))[1]
        proposal = update(PROFILES[profile], state, synthetic_input)
        states[identity] = (profile, proposal.state)
      previous[identity] = sample.now_ns
      result['synthetic_input'] = asdict(synthetic_input)
      if proposal is not None:
        address, data, bus = encode_offline(packer, proposal, counter)
        result.update(synthetic=asdict(proposal), synthetic_frame={'address': address, 'data': data.hex(), 'bus': bus})
    except (KeyError, TypeError, ValueError) as exc:
      result['unsupported_reason'] = str(exc)
      # A hole cannot inherit unverified history. Next supported sample begins
      # neutral and retains its native timestamp (never compress the gap).
      if str(exc) != 'duplicate_or_regressing_controller_time':
        states.pop(identity, None)
        schedulers.pop(identity, None)
    yield result


def extract_events(events, route_id):
  """Events must already be in native timestamp order; never select future data.

  Reader emits dictionaries {kind,t_ns,valid,data}. carOutput data is full
  navigatorA3 diagnostics JSON. Repeated diagnostic publications are marked as
  such by replay, not counted as additional strategy steps.
  """
  controls = {}
  control_times = deque(maxlen=11)
  last_t = -1
  for event in events:
    t = event['t_ns']
    if t < last_t:
      raise ValueError('events_not_native_time_order')
    last_t = t
    if event['kind'] == 'carControl':
      control_times.append(t)
      controls[t] = (event, tuple(control_times))
      # bounded lookup; source >1s old cannot satisfy strategy freshness anyway
      while controls and next(iter(controls)) < t - 1_000_000_000:
        del controls[next(iter(controls))]
    elif event['kind'] == 'carOutput':
      d = event['data']; c = d.get('controller') or {}; sample = c.get('input') or {}
      source = controls.get(sample.get('source_ns'))
      ev = {'exact_source_event': source is not None, 'values_match': False, 'cadence_supported': False,
            'event_valid': None, 'method': 'native event validity plus prior ten intervals; not exact SubMaster checks'}
      if source:
        control, stamps = source
        cc = control['data']
        ev.update(event_valid=control['valid'], source_t_ns=control['t_ns'],
                  values_match=cc['actuators']['curvature'] == sample.get('curvature_inv_m') and cc['latActive'] == sample.get('active'),
                  cadence_supported=len(stamps) == 11 and all(0 < b-a <= 100_000_000 for a, b in zip(stamps, stamps[1:])))
      frame = c.get('actual_frame') or {}
      counter = None
      if frame.get('address') == 982:
        try:
          data = bytes.fromhex(frame['data'])
          if len(data) == 8:
            # Frozen ford_lincoln_base_pt.dbc LatCtlPath_No_Cnt 60|4@0+.
            counter = (data[7] >> 1) & 15
        except (KeyError, ValueError):
          pass
      yield {'route_id': route_id, 't_ns': t, 'diagnostic': d, 'source_evidence': ev, 'counter': counter,
             'car_output_event_valid': event['valid']}


def merge_segments(segments):
  """Keep one segment plus boundary overlap; downstream rejects deeper regressions."""
  pending = []
  for events in segments:
    if not events:
      continue
    events = sorted(events, key=lambda e: e['t_ns'])
    boundary = events[0]['t_ns']
    yield from (e for e in pending if e['t_ns'] < boundary)
    pending = sorted([e for e in pending if e['t_ns'] >= boundary] + events, key=lambda e: e['t_ns'])
  yield from pending


def raw_segments(paths):
  import capnp
  import zstandard
  root = Path(__file__).resolve().parents[2]
  log = capnp.load(str(root / 'openpilot/cereal/log.capnp'),
                   imports=['/private/tmp/navigator-a3-opendbc/opendbc/car'])
  for path in paths:
    with path.open('rb') as stream:
      raw = zstandard.ZstdDecompressor().stream_reader(stream).read()
    # Rlog service interleaving can be out of timestamp order; reorder each
    # complete segment, preserving native clocks, then enforce across segments.
    events = []
    for m in log.Event.read_multiple_bytes(raw):
      kind = m.which()
      if kind not in ('carControl', 'carOutput'):
        continue
      data = m.carControl.to_dict() if kind == 'carControl' else json.loads(m.carOutput.navigatorA3.diagnosticsJson)
      events.append({'kind': kind, 't_ns': int(m.logMonoTime), 'valid': bool(m.valid), 'data': data})
    yield events


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--logs', type=Path, nargs='+', required=True, help='Complete rlog segments from route start in segment order')
  parser.add_argument('--route-id', required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  for index, path in enumerate(args.logs):
    if path.parent.name != f'{args.route_id}--{index}':
      parser.error('logs must be contiguous segments starting at route segment 0')
  args.output.mkdir(parents=True, exist_ok=True)
  counts = Counter()
  with (args.output / 'counterfactual-timeline.jsonl').open('w') as target:
    for result in replay_rows(extract_events(merge_segments(raw_segments(args.logs)), args.route_id)):
      counts['rows'] += 1
      if result['unsupported_reason']:
        outcome = 'unsupported:' + result['unsupported_reason']
      elif result['synthetic'] is None:
        outcome = 'scheduler:' + result['synthetic_scheduler']['reason']
      else:
        outcome = 'synthetic:' + result['synthetic']['reason']
      counts[outcome] += 1
      target.write(json.dumps(result, allow_nan=False, separators=(',', ':')) + '\n')
  import opendbc.car.ford.navigator_a3 as strategy
  strategy_path = Path(strategy.__file__).resolve()
  summary = {'strategy_path': str(strategy_path), 'strategy_sha256': hashlib.sha256(strategy_path.read_bytes()).hexdigest(), 'counts': dict(counts), 'inputs': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in args.logs],
             'limitations': [__doc__, 'Starts neutral at supplied route start; no synthetic transport feedback.',
                             'Unsupported diagnostic gaps reset counterfactual history; original lifecycle is retained verbatim.',
                             'Recorded carControl activation is retained; no hypothetical reengagement is invented.'],
             'physical_activation_allowed': False}
  (args.output / 'counterfactual-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
  print(json.dumps(summary))


if __name__ == '__main__':
  main()
