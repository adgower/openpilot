"""Offline compiled admission/handoff contract. Never connects to hardware.

The candidate C evaluator is an unvalidated physical-mapping hypothesis. Current
production rejects nonzero encoded Angle. No oracle result enters proposal state.
"""
from __future__ import annotations

import argparse
import ctypes
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from opendbc.can import CANPacker
from opendbc.car.ford.navigator_a3 import Inputs, State, PROFILES, update, encode_offline
from tools.navigator_a3.rejection_diagnostics import snapshot, REASONS
from tools.navigator_a3.safety_audit import build, inputs, event, instrument, lmc, WRAPPER, STOCK


def sha(path):
  return hashlib.sha256(path.read_bytes()).hexdigest()


def build_production(repo, output, traced):
  """Compile current actual safety.h dispatch, not the historical audit extract."""
  root = output / ('production-traced' if traced else 'production-plain')
  shutil.copytree(repo / 'opendbc/safety', root / 'opendbc/safety', dirs_exist_ok=True)
  before = {str(p.relative_to(root)): sha(p) for p in (root / 'opendbc/safety').rglob('*.h')}
  assert all(sha(repo / rel) == digest for rel, digest in before.items())
  checks = instrument(root) if traced else {}
  source = WRAPPER.replace('AUDIT_RESET', '').replace('AUDIT_LATCH', '0')
  source = source.replace('AUDIT_LAST', 'curvature_state.desired_last').replace('AUDIT_MEAS', 'curvature_state.meas')
  source = source.replace('AUDIT_CANDIDATE_INCLUDE', 'unsigned int audit_reasons(void) {return 0;}\n'
    'void audit_profile(int p) {(void)p;}\nunsigned int audit_candidate_state(int i) {(void)i;return 0;}')
  source = source.replace('AUDIT_CANDIDATE_DISPATCH', '')
  (root / 'wrapper.c').write_text(source)
  command = ['cc', '-shared', '-fPIC', '-std=gnu11', '-O0', '-DALLOW_DEBUG', '-I', str(root),
             str(root / 'wrapper.c'), '-o', str(root / 'audit.so')]
  subprocess.run(command, check=True)
  lib = ctypes.CDLL(str(root / 'audit.so'))
  lib.audit_packet.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int]
  return lib, {'checks': checks, 'compile_command': command, 'library_sha256': sha(root / 'audit.so'),
               'unmodified_header_sha256': before, 'source_commit': subprocess.check_output(
                 ['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()}


def fixtures():
  cases = {f'normal-p{p}-s{s}': {'profile': p, 'sign': s} for p in range(4) for s in (-1, 1)}
  cases.update({name: {'profile': 0, 'sign': 1} for name in (
    'jitter-49-51ms', 'jitter-small-demand', 'driver-handoff', 'inactive-nonzero',
    'permission-loss-neutral-return', 'excessive-then-neutral', 'a2-neutral')})
  result = {}
  rx = [e for e in inputs(25.)['latch-expiry-controls-off'] if not e['tx']]
  for name, config in cases.items():
    sequence = [dict(e, controls_fixture=None) for e in rx[:18]]
    sequence.append(event([0, 4, 0, 0, 0, 0, 0, 0], address=0x165, tx=False, t=90000))
    state = State()
    previous_wire = bytes(lmc())
    packer = CANPacker('ford_lincoln_base_pt')
    times = [100000, 149000, 200000, 249000, 300000] if name.startswith('jitter-') else [100000+i*50000 for i in range(5)]
    for i, t in enumerate(times):
      sequence.extend(dict(e, time_us=t, controls_fixture=None) for e in rx[18+i*3:21+i*3])
      if name == 'permission-loss-neutral-return' and i in (2, 4):
        sequence.append(event([0, 0 if i == 2 else 4, 0, 0, 0, 0, 0, 0], address=0x165, tx=False, t=t))
      driver = name == 'driver-handoff' and i == 2
      active = not (name == 'permission-loss-neutral-return' and i == 3)
      sample = Inputs(t*1000, t*1000, config['sign']*(.00002 if name == 'jitter-small-demand' else .001), 25., active, driver,
                      measured_curvature_inv_m=0., measurement_ns=t*1000, measurement_valid=True, scheduled_update=True)
      before = asdict(state)
      proposal = update(list(PROFILES.values())[config['profile']], state, sample)
      state = proposal.state  # Built once without any compiled admission feedback.
      address, wire, bus = encode_offline(packer, proposal, i)
      mutation = None
      if name == 'excessive-then-neutral' and i in (1, 2):
        wire = bytes(lmc(angle=700) if i == 1 else lmc(active=False))
        mutation = 'injected excessive wire' if i == 1 else 'injected neutral wire; host strategy history unchanged'
      elif name == 'inactive-nonzero':
        wire, mutation = bytes(lmc(angle=2, active=False)), 'malformed inactive nonzero wire'
      elif name == 'a2-neutral':
        wire, mutation = bytes(lmc()), 'neutral mode-1 A2 fixture, not an Angle proposal'
      sequence.append(dict(event(wire, address=address, t=t), bus=bus, proposal={
        'input': asdict(sample), 'before': before, 'output': asdict(proposal), 'after': asdict(state),
        'unmodified_wire_hex': encode_offline(packer, proposal, i)[1].hex(),
        'previous_proposal_wire_hex': previous_wire.hex(), 'wire_mutation': mutation}))
      previous_wire = encode_offline(packer, proposal, i)[1]
    result[name] = dict(config, sequence=sequence)
  return result


def execute(lib, checks, case, candidate_sources, variant):
  lib.audit_init()
  lib.audit_profile(case['profile'])
  rows = []
  state = State()
  for original in case['sequence']:
    e = {k: v for k, v in original.items() if k != 'proposal'}
    t = e['time_us']
    lib.audit_timer(t)
    before = snapshot(lib, state, t)
    proposal = original.get('proposal')
    if proposal:
      state = State(**proposal['after'])
    wire = (ctypes.c_ubyte * 8).from_buffer_copy(bytes.fromhex(e['data_hex']))
    accepted = bool(lib.audit_packet(e['address'], e['bus'], wire, int(e['tx'])))
    bits = lib.audit_reasons() if e['tx'] else 0
    names = [name for bit, name in REASONS.items() if bits & bit]
    encoded = None
    if e['tx']:
      def angle_raw(hex_data):
        data = bytes.fromhex(hex_data)
        return (((data[3] & 31) << 6) | (data[4] >> 2)) - 1000
      wire_angle = angle_raw(e['data_hex'])
      previous_host = angle_raw(proposal['previous_proposal_wire_hex'])
      encoded = {'path_angle_signed_counts': wire_angle, 'path_angle_raw_unsigned': wire_angle + 1000, 'path_angle_rad': wire_angle * .0005,
                 'previous_host_proposal_angle_signed_counts': previous_host,
                 'delta_from_previous_host_proposal_rad': (wire_angle - previous_host) * .0005,
                 'delta_from_candidate_last_rad': (wire_angle - before['compiled_candidate']['angle_last']) * .0005,
                 'candidate_path_step_limit_rad_at_fixture_25_mps': .009,
                 'candidate_history_applicable': variant == 'candidate',
                 'note': 'Candidate history has meaning only in candidate variant; mutated bytes are explicitly marked in proposal.'}
    rows.append({'event': e, 'proposal': proposal, 'encoded': encoded, 'accepted': accepted, 'before': before,
      'after': snapshot(lib, state, t), 'reason_bits': bits, 'reason_names': names,
      'failed_checks': [checks[lib.audit_failed(i)] for i in range(lib.audit_nfailed())],
      'candidate_failed_checks': [s for s in candidate_sources if s['reason'] in names],
      'physical_eps_response': 'unknown',
      'attribution_limit': 'Executed failed predicates only; early returns skip subsequent checks. Host wrapper is not MCU firmware timing.'})
  return rows


def audit(repo: Path, output: Path):
  output.mkdir(parents=True, exist_ok=True)
  source = repo / 'opendbc/safety/tests/navigator_a3/core_path_angle.h'
  candidate_sources = [{'reason': name, 'file': str(source.relative_to(repo)), 'line': i, 'expression': line.strip()}
                       for i, line in enumerate(source.read_text().splitlines(), 1)
                       for name in REASONS.values() if f'a3_reasons|={name}' in line]
  cases = fixtures()
  result = {'schema_version': 1, 'cases': {name: {} for name in cases}, 'builds': {},
    'physical_mapping_validated': False, 'production_matches_current_source': True,
    'instrumentation_equivalent': True, 'safety_param': 2, 'no_oracle_feedback': True,
    'assumptions': ['All input and command sequences are synthetic; no physical EPS execution is observed.',
      'Checksum/counter-valid synthetic 25 m/s straight-motion RX and cruise RX establish permission; no controls setter is used.',
      'Candidate evaluator uses current hypothesis header plus frozen stock safety; stock-reference is historical b4ef5e1c.',
      'Production variant compiles current checked-out safety.h and final-byte gate with DEBUG; no MCU image is built.',
      'Only exported state is observed. Candidate accepted angle and stock desired history may differ after rejection.',
      'Host strategy runs once per scheduled update regardless of the compiled result. This is no recovery/fallback implementation.',
      'Production checks forbid nonneutral path angle; neutral mode-1 can still be valid A2 steering.'],
    'source_hashes': {str(p): sha(p) for p in (Path(__file__), source, repo/'opendbc/car/ford/navigator_a3.py',
      Path(__file__).with_name('safety_audit.py'), Path(__file__).with_name('rejection_diagnostics.py'))},
    'parent_pin_before_checkpoint': subprocess.check_output(['git', '-C', str(Path(__file__).resolve().parents[2]), 'rev-parse', 'HEAD'], text=True).strip(),
    'historical_stock_commit': STOCK,
    'compiler': subprocess.check_output(['cc', '--version'], text=True)}
  for variant in ('candidate', 'stock-reference', 'production'):
    pair = {}
    for traced in (False, True):
      if variant == 'production':
        lib, meta = build_production(repo, output/'compiled', traced)
      else:
        lib, meta = build(Path('/Users/alex/Apps/bluepilot'), repo, output/'compiled',
                          'stock' if variant == 'stock-reference' else variant, traced)
      pair[traced] = {name: execute(lib, meta['checks'], case, candidate_sources, variant) for name, case in cases.items()}
      result['builds'][f'{variant}-{"traced" if traced else "plain"}'] = meta
    for name in cases:
      for plain, traced in zip(pair[False][name], pair[True][name], strict=True):
        assert {k:v for k,v in plain.items() if k != 'failed_checks'} == {k:v for k,v in traced.items() if k != 'failed_checks'}, (variant, name)
      result['cases'][name][variant] = pair[True][name]
  result['summary'] = {name: {variant: {'accepted_tx': sum(r['accepted'] for r in rows if r['event']['tx']),
    'rejected_tx': sum(not r['accepted'] for r in rows if r['event']['tx']),
    'first_rejection_us': next((r['event']['time_us'] for r in rows if r['event']['tx'] and not r['accepted']), None)}
    for variant, rows in pair.items()} for name, pair in result['cases'].items()}
  (output/'compiled-cases.json').write_text(json.dumps(result, indent=2))
  (output/'compiled-summary.json').write_text(json.dumps(result['summary'], indent=2))
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--stock-repo', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  result = audit(args.stock_repo, args.output)
  print(json.dumps({'cases': len(result['cases']), 'instrumentation_equivalent': result['instrumentation_equivalent']}))


if __name__ == '__main__':
  main()
