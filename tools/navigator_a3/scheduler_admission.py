"""Synthetic scheduler/compiled-admission audit; no hardware and no oracle feedback."""
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import subprocess

from opendbc.can import CANPacker
from opendbc.car.ford.navigator_a3 import Inputs, PROFILES, encode_offline
from opendbc.car.ford.navigator_a3_scheduler import ShadowScheduler
from tools.navigator_a3.angle_handoff_contract import build_production, execute, sha
from tools.navigator_a3.rejection_diagnostics import REASONS
from tools.navigator_a3.safety_audit import STOCK, build, event, inputs, lmc


def fixtures():
  cases, calls = {}, {}
  rx = [e for e in inputs(25.)['latch-expiry-controls-off'] if not e['tx']]
  # Approximately 100 Hz caller with the original 149/200/249/300 ms jitter points.
  times = [100000, 110000, 120000, 130000, 140000, 149000, 159000,
           169000, 179000, 189000, 200000, 210000, 220000, 230000,
           240000, 249000, 259000, 269000, 279000, 289000, 300000, 310000]
  configurations = [(f'jitter-p{p}-s{sign}', p, sign) for p in range(4) for sign in (-1, 1)]
  configurations += [('driver-handoff', 0, 1), ('timing-gap', 0, 1), ('original-call-times', 0, 1)]
  for name, p, sign in configurations:
    profile = list(PROFILES.values())[p]
    case_times = ([100000, 149000, 200000, 249000, 300000] if name == 'original-call-times' else
                  [100000, 150000, 260000, 270000, 300000, 310000, 360000] if name == 'timing-gap' else times)
    sequence = [dict(e, controls_fixture=None) for e in rx[:18]]
    sequence.append(event([0, 4, 0, 0, 0, 0, 0, 0], address=0x165, tx=False, t=90000))
    scheduler, packer = ShadowScheduler(), CANPacker('ford_lincoln_base_pt')
    previous_wire, rows, emission = bytes(lmc()), [], 0
    for t in case_times:
      driver = name == 'driver-handoff' and 169000 <= t <= 189000
      sample = Inputs(t*1000, t*1000, sign*.001, 25., True, driver,
                      measured_curvature_inv_m=0., measurement_ns=t*1000, measurement_valid=True)
      before = asdict(scheduler.state)
      decision = scheduler.step(profile, sample)
      after = asdict(scheduler.state)
      rows.append({'time_us': t, 'before': before, 'after': after, 'emitted': decision.output is not None,
                   'counter': decision.proposal_counter, 'decision': asdict(decision)})
      if decision.output is None:
        continue
      sequence.extend(dict(e, time_us=t, controls_fixture=None) for e in rx[18+emission*3:21+emission*3])
      output = decision.output
      address, wire, bus = encode_offline(packer, output, decision.proposal_counter)
      sequence.append(dict(event(wire, address=address, t=t), bus=bus, host_publication_us=t, proposal={
        'input': asdict(sample), 'before': before, 'output': asdict(output), 'after': after,
        'unmodified_wire_hex': wire.hex(), 'previous_proposal_wire_hex': previous_wire.hex(), 'wire_mutation': None}))
      previous_wire = wire
      emission += 1
    cases[name] = {'profile': p, 'sign': sign, 'sequence': sequence}
    calls[name] = rows
  # Same host calculation, separate presumed arrival times: hold first TX 10 ms,
  # leave the next one unchanged. 59 ms host spacing becomes 49 ms MCU spacing.
  base = 'jitter-p0-s1'
  for name in ('compressed-arrival', 'excessive-injection'):
    case = deepcopy(cases[base])
    tx = [e for e in case['sequence'] if e['tx']]
    if name == 'compressed-arrival':
      tx[0]['time_us'] += 10000
      tx[0]['transport_assumption'] = 'first frame delayed 10 ms; no host feedback'
      case['sequence'].sort(key=lambda e: e['time_us'])
    else:
      tx[1]['data_hex'] = bytes(lmc(angle=700)).hex()
      tx[1]['proposal']['wire_mutation'] = 'isolated excessive wire; scheduler receives no feedback'
    cases[name], calls[name] = case, deepcopy(calls[base])
  return cases, calls


def audit(repo: Path, output: Path):
  output.mkdir(parents=True, exist_ok=True)
  source = repo / 'opendbc/safety/tests/navigator_a3/core_path_angle.h'
  sources = [{'reason': name, 'file': str(source.relative_to(repo)), 'line': i, 'expression': line.strip()}
             for i, line in enumerate(source.read_text().splitlines(), 1)
             for name in REASONS.values() if f'a3_reasons|={name}' in line]
  cases, calls = fixtures()
  result = {'schema_version': 1, 'cases': {name: {} for name in cases}, 'scheduler_calls': calls,
            'no_oracle_feedback': True, 'instrumentation_equivalent': True, 'physical_mapping_validated': False,
            'timing_basis': 'synthetic host and separately assumed MCU arrival times', 'builds': {},
            'safety_param': 2, 'historical_stock_commit': STOCK, 'compiler': subprocess.check_output(['cc', '--version'], text=True),
            'source_hashes': {str(p): sha(p) for p in (Path(__file__), source,
              repo/'opendbc/car/ford/navigator_a3.py', repo/'opendbc/car/ford/navigator_a3_scheduler.py',
              Path(__file__).with_name('angle_handoff_contract.py'), Path(__file__).with_name('safety_audit.py'),
              Path(__file__).with_name('rejection_diagnostics.py'))},
            'limitations': ['Host wrapper does not reconstruct MCU timing or EPS execution.',
              'Synthetic RX establishes permission, without a controls setter.',
              'Independent rejection remains possible and never feeds host proposal state.',
              'Counterfactual proposals are not transmitted or acknowledged.']}
  for variant in ('candidate', 'production'):
    pair = {}
    for traced in (False, True):
      if variant == 'production':
        lib, meta = build_production(repo, output/'compiled', traced)
      else:
        lib, meta = build(Path('/Users/alex/Apps/bluepilot'), repo, output/'compiled', variant, traced)
      pair[traced] = {name: execute(lib, meta['checks'], case, sources, variant) for name, case in cases.items()}
      result['builds'][f'{variant}-{"traced" if traced else "plain"}'] = meta
    for name in cases:
      for plain, traced in zip(pair[False][name], pair[True][name], strict=True):
        assert {k:v for k,v in plain.items() if k != 'failed_checks'} == {k:v for k,v in traced.items() if k != 'failed_checks'}
      result['cases'][name][variant] = pair[True][name]
  result['summary'] = {name: {variant: {'accepted_tx': sum(r['accepted'] for r in rows if r['event']['tx']),
      'rejected_tx': sum(not r['accepted'] for r in rows if r['event']['tx']),
      'first_rejection_us': next((r['event']['time_us'] for r in rows if r['event']['tx'] and not r['accepted']), None)}
      for variant, rows in variants.items()} for name, variants in result['cases'].items()}
  (output/'scheduler-compiled-cases.json').write_text(json.dumps(result, indent=2))
  (output/'scheduler-compiled-summary.json').write_text(json.dumps(result['summary'], indent=2))
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--stock-repo', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  result = audit(args.stock_repo, args.output)
  print(json.dumps(result['summary'], indent=2))


if __name__ == '__main__':
  main()
