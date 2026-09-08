"""Compare frozen and revised hypothetical strategies; never reconstruct live permission."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess

from opendbc.car.ford import navigator_a3 as revised
import sys
import types

FROZEN_CHILD = '01bea343a7f7d4e4f8a1947539abacb59ab8c278'


def load_frozen(child):
  source = subprocess.check_output(['git', '-C', str(child), 'show', FROZEN_CHILD + ':opendbc/car/ford/navigator_a3.py'])
  module = types.ModuleType('_navigator_shadow_frozen')
  sys.modules[module.__name__] = module
  exec(compile(source, 'frozen_navigator_a3.py', 'exec'), module.__dict__)
  module.source_bytes = source
  return module


def compare_rows(rows, frozen):
  states = {}
  for index, row in enumerate(rows):
    identity = row['route_id']
    historical = row.get('historical') or {}
    original = historical.get('controller') or {}
    reason = row.get('unsupported_reason')
    result = {'source_row': index, 'route_id': identity, 't_ns': row['t_ns'],
              'historical_fault': historical.get('fault_reason'),
              'historical_reason': (original.get('output') or {}).get('reason'),
              'unsupported_reason': reason, 'input': row.get('synthetic_input'),
              'frozen': None, 'revised': None, 'frozen_matches_saved': None,
              'transport_acceptance': 'unknown', 'activation_allowed': False}
    if reason or row.get('synthetic') is None:
      if reason != 'duplicate_or_regressing_controller_time':
        states.pop(identity, None)
      yield result
      continue
    profile = row['synthetic']['profile']
    old_state, new_state = states.get(identity, (frozen.State(), revised.State()))
    old = frozen.update(frozen.PROFILES[profile], old_state, frozen.Inputs(**row['synthetic_input']))
    new_input = dict(row['synthetic_input'], scheduled_update=True)
    new = revised.update(revised.PROFILES[profile], new_state, revised.Inputs(**new_input))
    states[identity] = (old.state, new.state)
    result.update(frozen=asdict(old), revised=asdict(new), frozen_matches_saved=asdict(old) == row['synthetic'])
    yield result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--input', required=True, type=Path)
  parser.add_argument('--output', required=True, type=Path)
  parser.add_argument('--child', default='/private/tmp/navigator-a3-opendbc', type=Path)
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  frozen = load_frozen(args.child)
  counts = Counter()
  max_delta = 0.
  intervals = {'frozen': [], 'revised': []}
  opened = {}
  last_ns = None
  with args.input.open() as source, (args.output / 'comparison-timeline.jsonl').open('w') as target:
    for row in compare_rows((json.loads(line) for line in source), frozen):
      counts['rows'] += 1
      if row['frozen'] is not None:
        counts['supported_steps'] += 1
        counts['frozen_saved_mismatches'] += not row['frozen_matches_saved']
        now = row['input']['now_ns']
        for name in intervals:
          output = row[name]
          counts[name + ':' + output['reason']] += 1
          if output['mode'] == 0 and name not in opened:
            opened[name] = now
          elif output['mode'] != 0 and name in opened:
            intervals[name].append({'start_ns': opened.pop(name), 'end_ns': now, 'end_kind': 'active_sample'})
        counts['mode_changes'] += row['frozen']['mode'] != row['revised']['mode']
        max_delta = max(max_delta, abs(row['frozen']['path_angle_rad'] - row['revised']['path_angle_rad']))
        last_ns = now
      else:
        counts['unsupported:' + str(row['unsupported_reason'])] += 1
        if row['unsupported_reason'] != 'duplicate_or_regressing_controller_time':
          for name in list(opened):
            intervals[name].append({'start_ns': opened.pop(name), 'end_ns': last_ns, 'end_kind': 'coverage_gap'})
      target.write(json.dumps(row, separators=(',', ':'), allow_nan=False) + '\n')
  for name in opened:
    intervals[name].append({'start_ns': opened[name], 'end_ns': last_ns, 'end_kind': 'end_of_coverage'})
  paths = [args.input, Path(revised.__file__), Path(__file__)]
  summary = {'counts': dict(counts), 'max_path_angle_difference_rad': max_delta, 'neutral_intervals': intervals,
             'frozen_child': FROZEN_CHILD, 'frozen_strategy_sha256': hashlib.sha256(frozen.source_bytes).hexdigest(),
             'sources': [{'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()} for path in paths],
             'activation_allowed': False, 'transport_acceptance': 'unknown',
             'limitations': ['Both streams are hypothetical; historical lifecycle remains recorded.',
                             'Reuses saved inferred source validity; not exact live SubMaster or bridge reconstruction.',
                             'Only revised stream trusts the recorded scheduled Ford update; no model preview or transport feedback.',
                             'Neutral intervals end at observed next sample or coverage boundary; no interpolation through gaps.']}
  (args.output / 'comparison-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
  print(json.dumps({'counts': dict(counts), 'max_path_angle_difference_rad': max_delta}))


if __name__ == '__main__':
  main()
