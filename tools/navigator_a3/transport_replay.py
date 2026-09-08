"""Reproduce recorded transport analysis from the frozen local replay manifest."""
import argparse
from collections import Counter
import csv
import html
import json
from pathlib import Path

from tools.navigator_a3.transport_extract import extract
from tools.navigator_a3.transport_observer import run_file


def write_report(output: Path, manifest: dict):
  results = {}
  windows = {}
  for name in ('A2', 'curve17', 'curve29-30'):
    root = output / name
    with (root / 'observer-timeline.jsonl').open() as stream:
      rows = [json.loads(line) for line in stream]
    rejections = [r for r in rows if r['direct_steering_rejection']]
    health = [r for r in rows if r['kind'] == 'health']
    results[name] = {'steering_rejection_observations': len(rejections),
                     'attribution': dict(Counter(r['attribution'] for r in rejections)),
                     'mode_counts': dict(Counter(r['wire_mode'] for r in rejections)),
                     'first_observation': rejections[0] if rejections else None,
                     'first_mode_1_observation': next((r for r in rejections if r['wire_mode'] == 1), None),
                     'counter_increases_sum': sum(r['counter_delta'] or 0 for r in health),
                     'counter_discontinuities': sum(r['counter_interpretation'] == 'discontinuity_reset_or_wrap_unknown' for r in health),
                     'permission_loss_observations': sum(r['permission_lost'] for r in health),
                     'other_rejections': dict(Counter(str(r['address']) for r in rows if r['event_type'] == 'other_tx_rejection'))}
    # Window bounds alone come from earlier replay CSV; no synthetic command or
    # evaluator result is ever passed to the observer. Correlation keeps full prefix.
    for case in manifest['cases']:
      if case['name'] not in (name, 'A2-run19' if name == 'A2' else name):
        continue
      cmd = case['raw_admission_command']
      timeline = Path(cmd[cmd.index('--timeline') + 1])
      selected = str(case.get('selected_admission_run') or '')
      with timeline.open() as stream:
        times = [int(r['t_ns']) for r in csv.DictReader(stream) if not selected or str(r['run']) == selected]
      if not times:
        continue
      start, end = min(times), max(times)
      selected_rows = [r for r in rows if start <= r['t_ns'] <= end]
      windows[case['name']] = {'start_ns': start, 'end_ns': end, 'run': selected,
                              'steering_publications': sum(r['kind'] == 'published' for r in selected_rows),
                              'steering_rejections': sum(r['direct_steering_rejection'] for r in selected_rows),
                              'returned_observations': sum(r['kind'] == 'returned' for r in selected_rows),
                              'bound_source': str(timeline), 'correlation': 'full recorded prefix retained'}
  data = {'scope': 'Recorded A2 transport only; no A3 command acknowledgment or physical response validation',
          'recordings': results, 'previous_replay_windows': windows}
  (output / 'results.json').write_text(json.dumps(data, indent=2) + '\n')
  table = ''.join('<tr><td><a href="' + name + '/report.html">' + name + '</a></td><td>'
                  + str(r['steering_rejection_observations']) + '</td><td>' + html.escape(str(r['attribution']))
                  + '</td><td>' + str(r['counter_increases_sum']) + '</td></tr>' for name, r in results.items())
  page = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Navigator: recorded steering rejection observer</title><style>
body{font:17px/1.6 system-ui;max-width:1100px;margin:40px auto;padding:0 24px;color:#18232f;background:#fafbfc}
td,th{padding:10px;border-bottom:1px solid #ccd4da;text-align:left}a{color:#12638a}code{font-size:14px}
</style><h1>What the host could observe</h1>
<p>The standalone rejection observer is implemented. These results use saved A2 recordings; the controller, safety implementation and device are unchanged.</p>
<h2>Recorded findings</h2><table><tr><th>Recording</th><th>Steering rejection observations</th><th>Publication attribution</th><th>Aggregate counter
increases</th></tr>'''
  page += table + '''</table><p>Counts are observations, not a deduplicated count of rejected commands. Counter increases include all traffic and are
not attributable to steering.</p>
<p>Unique means unique in supplied coverage. An inactive message rejection does not establish the cause of a maneuver failure.
Counter decreases are discontinuities with reset/wrap cause unknown.</p>
<p>If a recording contains no rejected frames or counter increase, command acceptance still remains unknown.
A counter already nonzero at the start can reflect rejections outside the supplied coverage.</p>
<h2>Inspect the evidence</h2><ul>
<li>Open a recording above for a native-time table of every direct rejection, its attribution, and the first rejection's one-second approach and aftermath.</li>
<li><a href="results.json">Recording summaries and the previous replay windows</a></li>
<li><a href="fixtures/summary.json">Synthetic fault comparison</a> (emulated transport timing, not measured timing)</li>
<li><a href="verification.json">Frozen-source verification and local commit</a></li>
<li><a href="pytest.txt">Actual test results</a> · <a href="README.md">Reproduction and interpretation</a></li></ul>
<h2>What this proves</h2><p>A valid rejected frame is direct rejection evidence. A matching returned frame is a transport observation, not
confirmation that the steering system executed it. Missing echoes remain unknown. An exact publication-to-observation interval is reported only for
one eligible publication; it is not a safety-decision or steering-response latency.</p>
<p>Repeated exact payloads retain every eligible preceding publication through a compact group selector. No timeout, consumed-match assumption or
missing-rejection rule is used to invent acceptance. Full packet order, source identity, validity and bytes remain in the downloadable JSONL
files.</p>
<h2>Remaining work</h2><p>This checkpoint adds no production observer, disengagement policy, retry, recovery or firmware changes. Physical Angle
command-to-response evidence and a reviewed production safety implementation remain necessary before installation. Historical A2 returned frames
cannot acknowledge synthetic A3 commands.</p></html>'''
  (output / 'index.html').write_text(page)
  return data


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--manifest', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--render-only', action='store_true', help='Render existing observer output; do not re-extract or replay')
  args = parser.parse_args()
  manifest = json.loads(args.manifest.read_text())
  if not args.render_only:
    for case in manifest['cases']:
      if case['name'] == 'A2-run19':
        continue
      cmd = case['raw_admission_command']
      paths = [Path(cmd[i+1]) for i, value in enumerate(cmd) if value == '--rlog']
      root = args.output / case['name']
      extract(paths, root)
      run_file(root / 'transport-events.jsonl', root)
  print(json.dumps(write_report(args.output, manifest), indent=2))


if __name__ == '__main__':
  main()
