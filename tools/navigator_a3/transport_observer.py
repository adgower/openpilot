"""Standalone transport observer. No controller, safety evaluator or device imports.

A compact candidate-group selector retains every plausible publication without
quadratically copying repeated neutral payloads. It never proves EPS acceptance.
"""
import argparse
from bisect import bisect_right, insort
from collections import Counter
import hashlib
import html
import json
from pathlib import Path

TARGET = 0x3D6
ORACLE_FIELDS = {'accepted', 'compiled_admitted', 'reason_bits', 'failed_checks', 'safety_before', 'safety_after'}


def resolve_candidates(groups: dict, row: dict) -> list[dict]:
  """Resolve ALL possible preceding publications, including duplicate payloads."""
  selector = row.get('candidate_selector')
  if selector is None:
    return []
  return [p for p in groups[selector['group_id']]
          if p['order'] < selector['before_order'] and p['t_ns'] <= selector['not_after_t_ns']]


class Observer:
  def __init__(self):
    self.groups: dict[str, list[dict]] = {}
    self._group_keys: dict[tuple, str] = {}
    self._times: dict[str, list[int]] = {}
    self._health: dict[tuple, dict] = {}
    self._health_gap: set[tuple] = set()
    self._echoes: Counter = Counter()
    self.counts: Counter = Counter()
    self.first_rejections: dict[tuple, dict] = {}
    self._last_order: int | None = None

  @staticmethod
  def _key(e):
    return (e['route_id'], e['provenance'], e['bus'], e['address'], e['dlc'], e['data_hex'].lower())

  def consume(self, e: dict) -> dict:
    if any(not isinstance(e.get(k), str) or not e[k].strip() for k in ('route_id', 'provenance')):
      raise ValueError('Nonempty route and provenance identity are required')
    if ORACLE_FIELDS.intersection(e) or any(k.startswith(('compiled', 'oracle')) for k in e):
      raise ValueError('Safety oracle fields cannot enter the transport observer')
    if self._last_order is not None and e['order'] <= self._last_order:
      raise ValueError('Input order must strictly increase; do not sort away original observation order')
    self._last_order = e['order']
    r = dict(e, event_type=e['kind'], direct_steering_rejection=False, receiver_acceptance='unknown',
             candidate_count=0, candidate_selector=None, attribution='not_applicable',
             publication_to_observation_ns=None, possible_duplicate_echo=False,
             counter_delta=None, counter_interpretation='not_applicable', health_coverage_gap=False,
             configuration_changed=False, permission_lost=False, rx_checks_invalid=None,
             health_out_of_order=False)
    self.counts['input:' + e['kind']] += 1
    if e['kind'] == 'health':
      self._observe_health(e, r)
      return r
    if e.get('valid') is not True:
      r['attribution'] = 'invalid_evidence'
      r['event_type'] = 'invalid_' + e['kind']
      return r
    if e['kind'] not in ('published', 'returned', 'rejected'):
      return r
    try:
      raw = bytes.fromhex(e['data_hex'])
    except (ValueError, TypeError):
      r['attribution'] = 'invalid_evidence'
      return r
    if e.get('bus') is None or len(raw) != e.get('dlc'):
      r['attribution'] = 'invalid_evidence'
      return r
    if e['address'] != TARGET:
      r['event_type'] = 'other_tx_rejection' if e['kind'] == 'rejected' else 'other_' + e['kind']
      return r
    r['wire_mode'] = (raw[0] >> 4) & 7 if len(raw) == 8 else None
    k = self._key(e)
    if e['kind'] == 'published':
      if k not in self._group_keys:
        group = str(len(self.groups))
        self._group_keys[k] = group
        self.groups[group] = []
        self._times[group] = []
      group = self._group_keys[k]
      self.groups[group].append({'order': e['order'], 't_ns': e['t_ns'],
                                 'source_file': e.get('source_file'), 'source_event_index': e.get('source_event_index'),
                                 'packet_index': e.get('packet_index')})
      insort(self._times[group], e['t_ns'])
      r['publication_group'] = group
      r['attribution'] = 'publication_only'
      return r
    r['direct_steering_rejection'] = e['kind'] == 'rejected'
    if r['direct_steering_rejection']:
      self.first_rejections.setdefault((e['route_id'], e['provenance']),
                                       {'order': e['order'], 't_ns': e['t_ns'], 'wire_mode': r['wire_mode']})
    self._echoes[(k, e['kind'])] += 1
    r['possible_duplicate_echo'] = self._echoes[(k, e['kind'])] > 1
    if k in self._group_keys:
      group = self._group_keys[k]
      count = bisect_right(self._times[group], e['t_ns'])
      r['candidate_count'] = count
      r['candidate_selector'] = {'group_id': group, 'before_order': e['order'], 'not_after_t_ns': e['t_ns']}
      if count == 1:
        candidate = resolve_candidates(self.groups, r)[0]
        r['publication_to_observation_ns'] = e['t_ns'] - candidate['t_ns']
    r['attribution'] = 'unique' if r['candidate_count'] == 1 else 'ambiguous' if r['candidate_count'] else 'unmatched'
    self.counts[e['kind'] + ':' + r['attribution']] += 1
    return r

  def _observe_health(self, e, r):
    key = (e['route_id'], e['provenance'], e['panda_index'])
    fields = ('controlsAllowed', 'safetyRxChecksInvalid')
    counter = e.get('safetyTxBlocked')
    if (e.get('valid') is not True or type(counter) is not int or not 0 <= counter <= 0xFFFFFFFF
        or any(type(e.get(k)) is not bool for k in fields)
        or not isinstance(e.get('safetyModel'), str) or not e['safetyModel']
        or any(type(e.get(k)) is not int for k in ('safetyParam', 'alternativeExperience'))):
      r['event_type'] = 'invalid_health'
      self._health_gap.add(key)
      return
    prev = self._health.get(key)
    if prev and e['t_ns'] <= prev['t_ns']:
      r['event_type'] = 'out_of_order_or_same_time_health'
      r['health_out_of_order'] = True
      self._health_gap.add(key)
      return
    r['health_coverage_gap'] = key in self._health_gap
    self._health_gap.discard(key)
    r['rx_checks_invalid'] = e['safetyRxChecksInvalid']
    r['permission_available'] = e['controlsAllowed']
    r['counter_interpretation'] = 'initial_observation'
    if prev:
      r['health_observation_interval_ns'] = e['t_ns'] - prev['t_ns']
      r['permission_lost'] = prev['controlsAllowed'] and not e['controlsAllowed']
      config_fields = ('safetyModel', 'safetyParam', 'alternativeExperience')
      r['configuration_changed'] = any(e.get(k) != prev.get(k) for k in config_fields)
      delta = counter - prev['safetyTxBlocked']
      if delta < 0:
        r['counter_interpretation'] = 'discontinuity_reset_or_wrap_unknown'
      elif r['configuration_changed']:
        r['counter_interpretation'] = 'configuration_changed_counter_continuity_unknown'
      else:
        r['counter_delta'] = delta
        r['counter_interpretation'] = 'aggregate_increase' if delta else 'no_observed_increase'
    self._health[key] = e.copy()
    self.counts['health:' + r['counter_interpretation']] += 1
    if r['permission_lost']:
      self.counts['health:permission_lost'] += 1


def run_file(source: Path, output: Path) -> dict:
  output.mkdir(parents=True, exist_ok=True)
  observer = Observer()
  count = 0
  with source.open() as stream, (output / 'observer-timeline.jsonl').open('w') as dest:
    for line in stream:
      event = json.loads(line)
      row = observer.consume(event)
      # Preserve all steering, health and rejected/unknown observations; do not duplicate
      # the extractor's potentially millions of unrelated normal publications/returns.
      if event['kind'] == 'health' or event.get('address') == TARGET or event['kind'] in ('rejected', 'unknown'):
        dest.write(json.dumps(row, separators=(',', ':')) + '\n')
        count += 1
  (output / 'publication-groups.json').write_text(json.dumps(observer.groups, separators=(',', ':')) + '\n')
  with source.open('rb') as source_stream:
    input_hash = hashlib.file_digest(source_stream, 'sha256').hexdigest()
  summary = {'scope': 'Transport observations only; no EPS acceptance or A3 motion proof',
             'input_sha256': input_hash,
             'observer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             'counts': dict(observer.counts), 'timeline_rows': count,
             'first_observable_steering_rejections': [{'route_id': k[0], 'provenance': k[1], **v}
                                                       for k, v in observer.first_rejections.items()],
             'limits': ['Candidate selectors retain all earlier observed valid identical publications with native time not after echo',
                        'No time window or one-to-one consumption assumed; unique is unique in supplied coverage only',
                        'Repeated echo content can represent another command or a duplicate; never count as unique rejected commands',
                        'Publication-to-observation interval is not safety decision latency, EPS execution latency or a production deadline',
                        'No inferred acceptance from returned/missing echoes or quiet aggregate counters',
                        'No elapsed-time stale-health threshold assumed; intervals, invalid gaps and ordering anomalies remain visible']}
  (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
  table = ''.join(f'<tr><td>{html.escape(k)}</td><td>{v}</td></tr>' for k, v in sorted(observer.counts.items()))
  evidence = render_evidence(output / 'observer-timeline.jsonl')
  page = ('<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
          + '<title>Steering rejection observer</title><style>body{font:17px/1.6 system-ui;max-width:1000px;margin:40px auto;padding:0 24px}'
          + 'td{padding:6px 20px;border-bottom:1px solid #ddd}a{color:#16648f}</style><h1>Steering transport observations</h1>'
          + '<p>These are historical transport observations. Returned frames do not acknowledge EPS execution; missing feedback is unknown.</p>'
          + '<p><a href="summary.json">Summary and first observed rejection</a> · <a href="observer-timeline.jsonl">Native-time evidence timeline</a> · '
          + '<a href="publication-groups.json">All candidate publications</a></p><table>' + table + '</table>' + evidence + '<h2>Interpretation limits</h2><ul>'
          + ''.join('<li>' + html.escape(s) + '</li>' for s in summary['limits']) + '</ul></html>')
  (output / 'report.html').write_text(page)
  return summary


def render_evidence(timeline: Path) -> str:
  """Readable native-time evidence and first rejection approach; full rows stay in JSONL."""
  with timeline.open() as stream:
    rows = [json.loads(line) for line in stream]
  rejected = [r for r in rows if r['direct_steering_rejection']]
  transitions = [r for r in rows if r['event_type'] == 'health' and
                 (r['permission_lost'] or r['configuration_changed'] or r['counter_interpretation'] == 'aggregate_increase'
                  or r['counter_interpretation'] == 'discontinuity_reset_or_wrap_unknown')]
  def table(items):
    body = ''
    for row in items:
      interval = row['publication_to_observation_ns']
      detail = (f"{row['attribution']}; candidates={row['candidate_count']}; "
                + (f"publication interval={interval / 1e6:.3f} ms" if interval is not None else 'interval unknown'))
      if row['kind'] == 'health':
        detail = (f"{row['counter_interpretation']}; delta={row['counter_delta']}; "
                  + f"permission lost={row['permission_lost']}; config changed={row['configuration_changed']}")
      body += ('<tr><td>' + str(row['order']) + '</td><td>' + f"{row['t_ns']/1e9:.9f}" + '</td><td>'
               + html.escape(row['event_type']) + '</td><td>' + html.escape(row.get('data_hex', ''))
               + '</td><td>' + html.escape(detail) + '</td></tr>')
    return ('<table><thead><tr><th>Order</th><th>Native seconds</th><th>Observation</th><th>Full payload</th>'
            + '<th>Attribution / health</th></tr></thead><tbody>' + body + '</tbody></table>')
  result = '<h2>Direct steering rejection observations</h2>' + (table(rejected) if rejected else '<p>None observed in supplied coverage.</p>')
  if rejected:
    first = rejected[0]
    approach = [r for r in rows if r['route_id'] == first['route_id'] and r['provenance'] == first['provenance']
                and first['t_ns'] - 1_000_000_000 <= r['t_ns'] <= first['t_ns'] + 1_000_000_000]
    result += ('<h2>First observable rejection: one second before and after</h2>'
               + '<p>Shown in recorded packet order, on the same native clock.</p>' + table(approach))
  result += '<h2>Aggregate counter and permission/configuration transitions</h2>'
  result += table(transitions) if transitions else '<p>No valid transitions observed.</p>'
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--events', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  print(json.dumps(run_file(args.events, args.output)))


if __name__ == '__main__':
  main()
