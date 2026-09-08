"""Offline synthetic transport comparison, not independently observed vehicle evidence.

Fixture admission labels drive an explicit emulated echo generator only. The
transport observer receives an allowlisted event stream; oracle comparisons take
place outside it. Agreement tests plumbing, not independent safety correctness.
"""
import argparse
import hashlib
import json
from pathlib import Path

from tools.navigator_a3.transport_observer import Observer, TARGET

TIMING = 'fixture publication time; emulated echo offset +1 ns, not measured'


def generate_transport(rows: list[dict], case: str, variant: str, feedback: str, source_file: str) -> list[dict]:
  """Translate fixture TX into synthetic publications and admission-labelled echoes."""
  if feedback not in ('complete', 'no_rejected_echoes'):
    raise ValueError('Unknown feedback variant')
  events = []
  for index, row in enumerate(rows):
    event = row['event']
    if event.get('tx') is not True:
      continue
    if type(row['accepted']) is not bool:
      raise ValueError('TX fixture accepted must be boolean')
    raw = bytes.fromhex(event['data_hex'])
    publication = {'order': len(events), 'kind': 'published', 't_ns': event['time_us'] * 1000,
                   'route_id': f'synthetic/{case}/{variant}/{feedback}', 'provenance': 'synthetic',
                   'bus': event['bus'], 'address': event['address'], 'data_hex': raw.hex(), 'dlc': len(raw),
                   'valid': True, 'source_file': source_file, 'source_event_index': index, 'packet_index': 0,
                   'timing_basis': TIMING}
    events.append(publication)
    if row['accepted'] or feedback == 'complete':
      events.append(dict(publication, order=len(events), kind='returned' if row['accepted'] else 'rejected',
                         t_ns=publication['t_ns'] + 1))
  return events


def compare_fixtures(fixtures: Path, output: Path) -> dict:
  fixture_bytes = fixtures.read_bytes()
  cases = json.loads(fixture_bytes)['cases']
  output.mkdir(parents=True, exist_ok=True)
  comparisons = []
  source_order = 0
  with (output / 'source-events.jsonl').open('w') as source, (output / 'observer-timeline.jsonl').open('w') as timeline:
    for case, variants in cases.items():
      for variant in ('control', 'injected'):
        rows = variants[variant]
        # Oracle labels are used here solely for post-observation comparison.
        oracle_indices = [i for i, r in enumerate(rows)
                          if r['event'].get('tx') is True and r['event']['address'] == TARGET and r['accepted'] is False]
        for feedback in ('complete', 'no_rejected_echoes'):
          events = generate_transport(rows, case, variant, feedback, str(fixtures))
          observer = Observer()
          observed_indices = []
          for event in events:
            event['order'] = source_order
            source_order += 1
            source.write(json.dumps(event, separators=(',', ':')) + '\n')
            observed = observer.consume(event)
            timeline.write(json.dumps(observed, separators=(',', ':')) + '\n')
            if observed['direct_steering_rejection']:
              observed_indices.append(observed['source_event_index'])
          route_id = f'synthetic/{case}/{variant}/{feedback}'
          comparisons.append({'case': case, 'variant': variant, 'feedback': feedback, 'route_id': route_id,
                              'provenance': 'synthetic', 'source_event_rows': len(events),
                              'oracle_rejection_count': len(oracle_indices),
                              'observable_rejection_count': len(observed_indices),
                              'oracle_first_rejected_source_event_index': next(iter(oracle_indices), None),
                              'observable_first_rejected_source_event_index': next(iter(observed_indices), None),
                              'first_observable_rejection': observer.first_rejections.get((route_id, 'synthetic')),
                              'count_matches': len(oracle_indices) == len(observed_indices),
                              'first_rejection_matches': next(iter(oracle_indices), None) == next(iter(observed_indices), None),
                              'receiver_acceptance': 'unknown', 'missing_feedback_interpretation': 'unknown'})
          # Candidate group ids are local to each isolated route, so persist with route context.
          (output / f'{case}-{variant}-{feedback}-publication-groups.json').write_text(json.dumps(observer.groups) + '\n')
  summary = {'scope': 'Synthetic emulated transport fixtures; not real transport or independent safety validation',
             'fixtures_sha256': hashlib.sha256(fixture_bytes).hexdigest(),
             'generator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             'timing_basis': TIMING, 'comparisons': comparisons,
             'limits': ['Echo kind is generated from the existing admission oracle; agreement is by construction.',
                        'Observer input contains transport fields only; oracle comparison occurs outside the observer.',
                        'Echo count is not a count of independently identified rejected commands.',
                        'Missing rejected echoes remain unknown, even when the oracle records a rejection.',
                        'Returned echoes do not establish EPS acceptance or execution.',
                        'Synthetic +1 ns echo timing cannot establish real latency, deadlines, or on-road performance.']}
  (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
  return summary


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--fixtures', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  print(json.dumps(compare_fixtures(args.fixtures, args.output)))


if __name__ == '__main__':
  main()
