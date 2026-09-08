"""Scheduler replay at supported recorded sample times only; no interpolated input."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

from opendbc.can import CANPacker
from opendbc.car.ford.navigator_a3 import Inputs, PROFILES, encode_offline
from opendbc.car.ford.navigator_a3_scheduler import ShadowScheduler


def replay_supported_rows(rows):
  schedulers, generations = {}, {}
  packer = CANPacker('ford_lincoln_base_pt')
  for row in rows:
    identity = row['route_id']
    unsupported = row.get('unsupported_reason')
    result = {'route_id': identity, 't_ns': row['t_ns'], 'historical_fault': (row.get('historical') or {}).get('fault_reason'),
              'unsupported_reason': unsupported, 'proposal': None, 'frame': None, 'decision': None,
              'transport_acceptance': 'unknown', 'activation_allowed': False,
              'segment_generation': generations.get(identity, 0)}
    if unsupported or row.get('synthetic_input') is None:
      if unsupported != 'duplicate_or_regressing_controller_time':
        schedulers.pop(identity, None)
        generations[identity] = generations.get(identity, 0) + 1
      yield result
      continue
    profile = row['synthetic']['profile']
    scheduler = schedulers.setdefault(identity, ShadowScheduler())
    decision = scheduler.step(PROFILES[profile], Inputs(**row['synthetic_input']))
    result['input'] = row['synthetic_input']
    result['decision'] = {k: v for k, v in asdict(decision).items() if k != 'output'}
    if decision.output is not None:
      address, data, bus = encode_offline(packer, decision.output, decision.proposal_counter)
      result['proposal'] = asdict(decision.output)
      result['frame'] = {'address': address, 'data': data.hex(), 'bus': bus}
    yield result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--input', type=Path, required=True, help='Preserved legacy counterfactual timeline')
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  counts = Counter()
  with args.input.open() as source, (args.output / 'scheduler-timeline.jsonl').open('w') as target:
    for r in replay_supported_rows(json.loads(line) for line in source):
      counts['rows'] += 1
      if r['decision'] is None:
        counts['unsupported:' + str(r['unsupported_reason'])] += 1
      else:
        counts['supported_calls'] += 1
        counts['decision:' + r['decision']['reason']] += 1
        counts['emitted_frames'] += r['frame'] is not None
        counts['active_frames'] += r['proposal'] is not None and r['proposal']['mode'] == 1
        counts['neutral_frames'] += r['proposal'] is not None and r['proposal']['mode'] == 0
        counts['no_frame'] += r['frame'] is None
      target.write(json.dumps(r, separators=(',', ':'), allow_nan=False) + '\n')
  import opendbc.car.ford.navigator_a3_scheduler as scheduler
  paths = [args.input, Path(__file__), Path(scheduler.__file__)]
  result = {'counts': dict(counts), 'source_hashes': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
            'limitations': ['Only supported original roughly 20 Hz diagnostic input times are replayed; 100 Hz intermediate calls are not invented.',
                            'Not a full runtime replay or a comparison of measured steering. Host timing results do not guarantee MCU arrival spacing.',
                            'Legacy inferred validity is preserved, not retroactively improved. Unsupported gaps restart a separately numbered hypothetical stream.',
                            'Original lifecycle is unchanged. No transport feedback enters scheduling.'],
            'activation_allowed': False}
  (args.output / 'scheduler-summary.json').write_text(json.dumps(result, indent=2) + '\n')
  print(json.dumps(dict(counts)))


if __name__ == '__main__':
  main()
