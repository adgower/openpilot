"""Replay actual RX frames interleaved with offline synthetic candidate TX.

No synthetic measurements, controls permission or health seeding. This tests
command admission against recorded motion, not closed-loop A3 performance.
"""
import argparse
from collections import Counter
import csv
import ctypes
import hashlib
import html
import json
from typing import Any
from pathlib import Path

from openpilot.tools.lib.logreader import _LogFileReader
from tools.navigator_a3.safety_audit import build
from opendbc.car.ford.navigator_a3 import PROFILES


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--timeline', type=Path, required=True)
  p.add_argument('--run', help='One diagnostic run; use for overlapping maneuver windows')
  p.add_argument('--rlog', type=Path, action='append', required=True)
  p.add_argument('--output', type=Path, required=True)
  p.add_argument('--donor', type=Path, default=Path('/Users/alex/Apps/bluepilot'))
  p.add_argument('--opendbc', type=Path, default=Path('/private/tmp/navigator-a3-opendbc'))
  args = p.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  lib, meta = build(args.donor, args.opendbc, args.output / 'compiled', 'candidate', True)
  rx = []
  ignored = Counter()
  for path in args.rlog:
    for msg in _LogFileReader(str(path), only_union_types=True):
      if msg.which() != 'can':
        continue
      for packet in msg.can:
        if packet.src in (0, 1, 2) and len(packet.dat) == 8:
          rx.append((int(msg.logMonoTime), int(packet.address), int(packet.src), bytes(packet.dat)))
        else:
          ignored[f'bus{packet.src}_len{len(packet.dat)}'] += 1
  rx.sort(key=lambda x: x[0])
  with args.timeline.open() as f:
    timeline = list(csv.DictReader(f))
  if args.run is not None:
    timeline = [row for row in timeline if row['run'] == args.run]
  results: list[dict[str, Any]] = []
  for profile_id, profile in enumerate(PROFILES):
    lib.audit_init()
    lib.audit_profile(profile_id)
    # audit_init starts controls_allowed=False. Only actual recorded RX can set it.
    index = 0
    for row in sorted((r for r in timeline if r['profile'] == profile), key=lambda r: int(r['t_ns'])):
      t = int(row['t_ns'])
      if not rx or t < rx[0][0] or t > rx[-1][0]:
        continue
      while index < len(rx) and rx[index][0] <= t:
        rt, addr, bus, data = rx[index]
        lib.audit_timer((rt // 1000) % (2**32))
        lib.audit_packet(addr, bus, (ctypes.c_ubyte * 8).from_buffer_copy(data), 0)
        index += 1
      lib.audit_timer((t // 1000) % (2**32))
      accepted = bool(lib.audit_packet(int(row['address']), int(row['bus']),
                                      (ctypes.c_ubyte * 8).from_buffer_copy(bytes.fromhex(row['data_hex'])), 1))
      failures = [meta['checks'][lib.audit_failed(i)] for i in range(lib.audit_nfailed())]
      results.append({'t_ns': t, 'run': row['run'], 'profile': profile, 'mode': int(row['mode']),
                      'data_hex': row['data_hex'], 'accepted': accepted, 'candidate_reasons': lib.audit_reasons(),
                      'controls_allowed': bool(lib.audit_controls_get()), 'failed_checks': failures})
  (args.output / 'admission.json').write_text(json.dumps(results, indent=2))
  summary = {'candidate_header_sha256': hashlib.sha256((args.opendbc / 'opendbc/safety/tests/navigator_a3/core_path_angle.h').read_bytes()).hexdigest(),
             'selected_run': args.run, 'recorded_rx_frames': len(rx), 'ignored_frames': dict(ignored), 'compiled': meta,
             'inputs': {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in args.rlog},
             'counts': dict(Counter(f'{r["profile"]}:mode{r["mode"]}:{r["accepted"]}:reasons{r["candidate_reasons"]}' for r in results)),
             'limitations': ['Actual recorded RX only; segment starts may lack prior engagement state',
                             'Counterfactual TX against old A2 motion is not a closed-loop prediction',
                             'Host-only algebraic mapping evaluator is not independently validated vehicle safety',
                             'No periodic safety_tick is simulated; this is not complete firmware-runtime equivalence',
                             'No production path-angle firmware or device execution']}
  (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
  failures = Counter(check['expression'] for row in results for check in row['failed_checks'])
  report = '<!doctype html><meta charset="utf-8"><title>Recorded-RX admission audit</title>'
  report += '<style>body{font:16px system-ui;max-width:1000px;margin:40px auto}pre{white-space:pre-wrap}</style>'
  report += '<h1>Recorded-RX / synthetic-TX admission audit</h1><p>Host-only evaluator, not firmware approval or measured A3 handling. '
  report += 'No synthetic health or controls permission was seeded.</p>'
  report += '<p>Native timestamped eight-byte CAN RX on buses 0–2; other lengths and returned/echo buses excluded. '
  report += f'Selected diagnostic run: {html.escape(args.run or "whole ordinary-drive segment window")}. '
  report += 'Maneuver windows overlap, so A2 admission is deliberately limited to one selected run.</p>'
  report += '<p><a href="admission.json">Every encoded command, result and executed failed check</a> · '
  report += '<a href="summary.json">Source hashes and compiled artifact provenance</a></p>'
  report += '<h2>Counts</h2><pre>' + html.escape(json.dumps(summary['counts'], indent=2)) + '</pre>'
  report += '<h2>Executed failed stock predicates</h2><pre>' + html.escape(json.dumps(dict(failures), indent=2)) + '</pre>'
  report += '<p>Candidate reason bits are defined in core_path_angle.h; multiple bits identify simultaneous additional failures. '
  report += 'Historical segment starts may lack prior engagement state.</p>'
  (args.output / 'report.html').write_text(report)
  print(json.dumps({'tx_cases': len(results), 'rx_frames': len(rx), 'counts': summary['counts']}))


if __name__ == '__main__':
  main()
