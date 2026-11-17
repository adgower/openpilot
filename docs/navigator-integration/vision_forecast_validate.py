"""Validate actual-context offline command replay and print a compact receipt."""

import argparse
from collections import Counter
import json
from pathlib import Path


def summarize(receipt):
  routes = receipt['routes']
  exit_arms = routes['000002c3--9509dc3167']['arms']
  exit_window = (375.0, 386.728)
  summary = {
    'segments': receipt['segments'],
    'routes': len(routes),
    'context_calls': sum(route['frames'] for route in routes.values()),
    'recorded_can': receipt['can_counts'],
    'arms': {},
    'exit': {},
    'controls': {},
  }
  for name in next(iter(routes.values()))['arms']:
    counts = Counter()
    reversals = 0
    for route in routes.values():
      arm = route['arms'][name]
      counts.update(arm['counts'])
      reversals += arm['reversals']
      for event in arm['events']:
        assert 25 <= event['target'] <= event['ceiling'], event
    summary['arms'][name] = {'counts': counts, 'direction_changes': reversals}
  for name, arm in exit_arms.items():
    events = [event for event in arm['events'] if exit_window[0] <= event['t'] < exit_window[1]]
    summary['exit'][name] = {'requests': len(events), 'first': events[:1], 'last': events[-1:]}
    if name == 'baseline':
      assert not events
    else:
      assert len(events) > 3
      assert 381.7 < events[0]['t'] < 381.9
      assert all(event['step'] == 'SET-' for event in events)
      low_confidence_times = {row['t'] for row in arm['observations'] if row['lane_confidence'] < 0.55}
      assert any(event['t'] in low_confidence_times for event in events)
      if name == 'integrated_miss_first_down':
        assert events[0]['dropped']
        assert not events[1]['dropped']
        assert events[0]['target'] == events[1]['target']
        assert events[1]['t'] - events[0]['t'] <= 1.5
  for route, lower, upper in [('000002ac--53ac8baf70', 87.0, 90.0), ('000002ac--53ac8baf70', 715.0, 723.0), ('000002c1--fed23eccc6', 187.0, 192.0)]:
    key = f'{route}:{lower}-{upper}'
    summary['controls'][key] = {}
    for name, arm in routes[route]['arms'].items():
      events = [event for event in arm['events'] if lower <= event['t'] <= upper]
      summary['controls'][key][name] = len(events)
    assert summary['controls'][key]['integrated_normal'] <= summary['controls'][key]['baseline']
  return summary


if __name__ == '__main__':
  cli = argparse.ArgumentParser(description=__doc__)
  cli.add_argument('receipt', type=Path)
  cli.add_argument('--output', type=Path)
  arguments = cli.parse_args()
  result = summarize(json.loads(arguments.receipt.read_text()))
  output = json.dumps(result, indent=2)
  if arguments.output:
    arguments.output.write_text(output + '\n')
  print(output)
