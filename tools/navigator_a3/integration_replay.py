"""Replay recorded transport through the bounded runtime observer, without control feedback."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeObserver


def replay(source, output, capacity=2048):
  output.mkdir(parents=True, exist_ok=True)
  observers = {}
  counts = Counter()
  with source.open() as stream, (output / 'runtime-timeline.jsonl').open('w') as target:
    for line in stream:
      e = json.loads(line)
      if e['kind'] != 'health' and e.get('address') != 982 and e['kind'] != 'rejected':
        continue
      identity = (e['route_id'], e['provenance'])
      if identity not in observers:
        observers[identity] = RuntimeObserver(*identity, capacity=capacity)
      row = observers[identity].consume(e)
      counts[row['event_type']] += 1
      if row['direct_steering_rejection']:
        counts['steering_rejection:' + row['attribution']] += 1
      # All retained candidates remain represented in this offline diagnostic.
      target.write(json.dumps(row, separators=(',', ':')) + '\n')
  with source.open('rb') as f:
    digest = hashlib.file_digest(f, 'sha256').hexdigest()
  result = {'source_sha256': digest, 'capacity': capacity, 'counts': dict(counts),
            'retained_publications': sum(len(o.publications) for o in observers.values()),
            'coverage_incomplete': any(o.coverage_incomplete for o in observers.values()),
            'scope': 'Recorded transport through bounded runtime observer; no generated commands or control feedback',
            'acceptance': 'unknown; history eviction deliberately prevents unique attribution'}
  (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
  return result


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--events', type=Path, required=True)
  p.add_argument('--output', type=Path, required=True)
  args = p.parse_args()
  print(json.dumps(replay(args.events, args.output)))


if __name__ == '__main__':
  main()
