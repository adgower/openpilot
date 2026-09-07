"""Offline common-input path-angle counterfactual; consumes existing diagnostic CSVs.

No model prediction is used as controller input. Recorded motion is context only,
not a prediction of A3 performance. No device or CAN transport dependency.
"""
import argparse
from bisect import bisect_right
import csv
from dataclasses import asdict
import hashlib
import html
import json
from pathlib import Path

from opendbc.can import CANPacker
from opendbc.car.ford.navigator_a3 import Inputs, State, PROFILES, update, encode_offline, DONOR_REVISION

INPUT = 'carControl.actuators.curvature'
CONTEXT = ['carState.vEgoRaw', 'carControl.latActive', 'carState.steeringPressed', 'carState.steeringAngleDeg',
           'derived.yawCurvature', 'carOutput.actuatorsOutput.curvature', 'modelV2.action.desiredCurvature',
           'lateralManeuverPlan.desiredCurvature', 'carState.vehicleSensorsInvalid']


def replay(path):
  groups = {}
  with path.open() as f:
    for row in csv.DictReader(f):
      if row['source'] not in [INPUT, *CONTEXT]:
        continue
      key = row.get('run_id', 'drive')
      groups.setdefault(key, {}).setdefault(row['source'], []).append(
        (int(row['t_ns']), float(row['value']) if row['value'] else None, row['valid'] in ('1', 'True')))
  packer = CANPacker('ford_lincoln_base_pt')
  result = []
  for run, series in groups.items():
    for values in series.values():
      values.sort()
    times = {s: [r[0] for r in rows] for s, rows in series.items()}
    states = {name: State() for name in PROFILES}
    last = None
    for t, k, valid in series.get(INPUT, []):
      if last is not None and t - last < 50_000_000:
        continue
      last = t
      context = {}
      stamps = [t]
      good = valid and k is not None
      for s in CONTEXT:
        i = bisect_right(times.get(s, []), t) - 1
        row = series[s][i] if i >= 0 else None
        ok = row is not None and row[2] and row[1] is not None and 0 <= t - row[0] <= 100_000_000
        context[s] = row[1] if ok else None
        if s in CONTEXT[:3] + ['carState.vehicleSensorsInvalid']:
          good &= ok
          if row:
            stamps.append(row[0])
      good &= context['carState.vehicleSensorsInvalid'] == 0
      sample = Inputs(t, min(stamps), k or 0., context['carState.vEgoRaw'] or 0.,
                      bool(context['carControl.latActive']), bool(context['carState.steeringPressed']), bool(good))
      for name, profile in PROFILES.items():
        output = update(profile, states[name], sample)
        states[name] = output.state
        addr, data, bus = encode_offline(packer, output, (len(result) // len(PROFILES)) % 16)
        result.append({'run': run, 't_ns': t, 'input_curvature_inv_m': k, 'source_valid': good,
                       **context, **{k: v for k, v in asdict(output).items() if k != 'state'},
                       'wire_path_angle_rad': -output.path_angle_rad, 'address': addr, 'bus': bus, 'data_hex': data.hex(),
                       'safety_admission': 'not evaluated in this replay; see compiled audit'})
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--input', type=Path, required=True)
  parser.add_argument('--admission', type=Path, help='Compiled raw-RX admission.json from this timeline')
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  rows = replay(args.input)
  if not rows:
    raise ValueError('No bounded general-controls input samples found in diagnostic CSV')
  admissions = {}
  if args.admission:
    admissions = {(r['run'], r['t_ns'], r['profile']): r for r in json.loads(args.admission.read_text())}
  for row in rows:
    admission = admissions.get((row['run'], row['t_ns'], row['profile']))
    row['compiled_admitted'] = int(admission['accepted']) if admission else None
    row['compiled_reason_bits'] = admission['candidate_reasons'] if admission else None
    row['safety_admission'] = 'compiled host-only recorded-RX hypothesis audit' if admission else 'unavailable in selected raw-RX audit coverage'
  args.output.mkdir(parents=True, exist_ok=True)
  with (args.output / 'candidate-timeline.csv').open('w') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)
  counts = {name: {reason: sum(r['profile'] == name and r['reason'] == reason for r in rows)
                   for reason in sorted({r['reason'] for r in rows})} for name in PROFILES}
  metadata = {'label': 'core path-angle experiment', 'source': str(args.input),
              'source_sha256': hashlib.sha256(args.input.read_bytes()).hexdigest(), 'donor': DONOR_REVISION,
              'profiles': {name: asdict(p) for name, p in PROFILES.items()}, 'counts': counts,
              'limits': ['No A3 motion recorded', 'No device access', 'Safety admission not inferred from Python limits',
                         'As-of joins only, maximum age 100 ms; missing validity becomes inactive',
                         'Replayed cadence approximates 20 Hz from logged timestamps; not original frame phase']}
  (args.output / 'metadata.json').write_text(json.dumps(metadata, indent=2))
  sections = []
  panels = [
    ('Compiled admission (0/1) / reason bits; missing means not evaluated', [('compiled_admitted', '#187246'), ('compiled_reason_bits', '#b52b25')]),
    ('Curvature (1/m)', [('input_curvature_inv_m', '#b52b25'), ('modelV2.action.desiredCurvature', '#9962ab'),
                       ('lateralManeuverPlan.desiredCurvature', '#9962ab'), ('carOutput.actuatorsOutput.curvature', '#dd9820'),
                       ('derived.yawCurvature', '#187246')]),
    ('Path angle (rad)', [('raw_path_angle_rad', '#999'), ('path_angle_rad', '#2768c0'), ('wire_path_angle_rad', '#187246')]),
    ('Measured steering angle (deg)', [('carState.steeringAngleDeg', '#187246')]),
    ('Vehicle speed (m/s)', [('carState.vEgoRaw', '#2768c0')]),
    ('Lateral active / driver pressed / candidate mode', [('carControl.latActive', '#2768c0'),
                                                       ('carState.steeringPressed', '#b52b25'), ('mode', '#187246')]),
  ]
  for name in PROFILES:
    selected = [r for r in rows if r['profile'] == name]
    for run in dict.fromkeys(r['run'] for r in selected):
      rs = [r for r in selected if r['run'] == run]
      start, end = rs[0]['t_ns'], rs[-1]['t_ns']
      sections.append(f'<h2>{html.escape(name)} · run {html.escape(run)}</h2><p>Native t={start}–{end} ns; '
                      + 'all panels share the same time axis. Missing values break lines.</p>')
      for title, fields in panels:
        vals = [r[field] for r in rs for field, _ in fields if r[field] is not None]
        lo, hi = (min(vals), max(vals)) if vals else (-1., 1.)
        if hi == lo:
          lo, hi = lo - 1., hi + 1.
        paths = []
        for field, color in fields:
          segments, points = [], []
          previous_t = None
          for r in rs:
            if previous_t is not None and r['t_ns'] - previous_t > 100_000_000 and points:
              segments.append(' '.join(points))
              points = []
            previous_t = r['t_ns']
            if r[field] is None:
              if points:
                segments.append(' '.join(points))
                points = []
            else:
              points.append(f'{50 + 900 * (r["t_ns"] - start) / max(end-start, 1):.2f},'
                            + f'{150 - 120*(r[field]-lo)/(hi-lo):.2f}')
          if points:
            segments.append(' '.join(points))
          paths.extend(f'<polyline points="{seg}" fill="none" stroke="{color}"/>' for seg in segments)
        legend = ' · '.join(f'<span style="color:{color}">{html.escape(field)}</span>' for field, color in fields)
        sections.append(f'<h3>{title}</h3><small>{legend}</small><svg viewBox="0 0 1000 190">'
                        + f'<text x="0" y="30">{hi:.4g}</text><text x="0" y="150">{lo:.4g}</text>'
                        + f'<text x="50" y="182">0 s</text><text x="870" y="182">{(end-start)/1e9:.2f} s</text>'
                        + ''.join(paths) + '</svg>')
  (args.output / 'report.html').write_text('<!doctype html><meta charset="utf-8"><title>Core path-angle offline replay</title>'
    + '<style>body{font:16px system-ui;max-width:1100px;margin:40px auto;padding:20px}'
    + 'svg{width:100%;background:#f6f7fa}pre{white-space:pre-wrap}text{font-size:12px}</style>'
    + '<h1>Core path-angle experiment — offline commands</h1><p><b>No A3 handling result or deployment readiness is established.</b> '
    + 'Every strategy receives the same recorded bounded general-controls curvature. Model and yaw signals are context only. '
    + 'Immediate driver press produces inactive output; no automatic recovery pulse is implemented. '
    + 'Stock safety still blocks path angle. See the independent compiled admission audit.</p>'
    + '<p><a href="candidate-timeline.csv">Full common-clock timeline, context and encoded CAN bytes</a> · '
    + '<a href="metadata.json">Profiles, input hash and limitations</a> · '
    + '<a href="raw-admission/report.html">Raw-RX compiled admission details</a></p><pre>'
    + html.escape(json.dumps(counts, indent=2)) + '</pre>' + ''.join(sections))
  print(json.dumps({'rows': len(rows), 'output': str(args.output), 'counts': counts}))


if __name__ == '__main__':
  main()
