"""Deterministic offline paired fault audit; admission never feeds proposal state."""
from __future__ import annotations

import argparse
import ctypes
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from typing import Any

from opendbc.can import CANPacker
from opendbc.car.ford.navigator_a3 import Inputs, State, PROFILES, update, encode_offline
from tools.navigator_a3.safety_audit import build, inputs, event, lmc

STATE_FIELDS = ('angle_last', 'tx_time_us', 'tx_seen', 'profile', 'reasons', 'brake_rx_time_us',
                'pcm_rx_time_us', 'yaw_rx_time_us', 'brake_seen', 'pcm_seen', 'yaw_seen',
                'brake_valid', 'pcm_valid', 'yaw_valid')
REASONS = {1: 'A3_MODE', 2: 'A3_FRESHNESS', 4: 'A3_CADENCE', 8: 'A3_PROFILE',
           16: 'A3_INACTIVE', 32: 'A3_STOCK_ENVELOPE', 64: 'A3_SPEED',
           128: 'A3_WIRE_RANGE', 256: 'A3_RELAY', 512: 'A3_PATH_RATE'}
FAULTS = ('stale_rx', 'corrupt_rx', 'controls_off', 'excessive_tx')


def classify_src(src: int) -> dict:
  """Decode normal single-tag pandad src; ambiguous combined tags stay unknown."""
  if 0 <= src < 8:
    return {'kind': 'rx', 'bus': src, 'acknowledges_synthetic_a3': False}
  if 0x80 <= src < 0x88:
    return {'kind': 'returned', 'bus': src - 0x80, 'acknowledges_synthetic_a3': False}
  if 0xC0 <= src < 0xC8:
    return {'kind': 'rejected', 'bus': src - 0xC0, 'acknowledges_synthetic_a3': False}
  return {'kind': 'unknown', 'bus': None, 'acknowledges_synthetic_a3': False}


def snapshot(lib, state, t):
  candidate = [ctypes.c_int32(lib.audit_candidate_state(i)).value if i == 0 else lib.audit_candidate_state(i) & 0xFFFFFFFF
               for i in range(14)]
  return {'time_us': t, 'controller_proposal_state': asdict(state),
          'compiled_stock_desired_last': lib.audit_last(),
          'compiled_controls_allowed': bool(lib.audit_controls_get()),
          'compiled_measurement_range': [lib.audit_meas_min(), lib.audit_meas_max()],
          'compiled_candidate': dict(zip(STATE_FIELDS, candidate, strict=True))}


def sequence(speed=25.):
  """Checksum-valid synthetic RX at 100 ms; proposals at 50 ms, above 9 m/s."""
  source = [e for e in inputs(speed)['latch-expiry-controls-off'] if not e['tx']]
  result = []
  # Six warmup samples fill frozen safety measurement/counter history.
  result.extend(dict(e, controls_fixture=None) for e in source[:18])
  result.append(event([0, 4, 0, 0, 0, 0, 0, 0], address=0x165, tx=False, t=90000))
  for i in range(24):
    t = 100000 + i * 50000
    if i % 2 == 0:
      for e in source[18 + (i // 2) * 3:21 + (i // 2) * 3]:
        result.append(dict(e, time_us=t, controls_fixture=None))
    if t == 500000:
      result.append(event([0, 4, 0, 0, 0, 0, 0, 0], address=0x165, tx=False, t=t))
    result.append({'time_us': t, 'proposal_index': i, 'tx': True, 'address': 0x3D6})
  return result


def execute(lib, checks, fault=None, speed=25.) -> list[dict[str, Any]]:
  if fault is not None and fault not in FAULTS:
    raise ValueError(fault)
  lib.audit_init()  # Exactly once per full independent run, never after rejection.
  lib.audit_profile(0)
  state = State()
  rows: list[dict[str, Any]] = []
  packer = CANPacker('ford_lincoln_base_pt')
  for original in sequence(speed):
    e = dict(original)
    t = e['time_us']
    lib.audit_timer(t)
    before = snapshot(lib, state, t)
    injected, proposal = False, None
    if 'proposal_index' in e:
      # No evaluator state, acceptance, reason or transport feedback is an input.
      sample = Inputs(t * 1000, t * 1000, .0002, speed, True, False, True,
                      measured_curvature_inv_m=0., measurement_ns=t * 1000, measurement_valid=True)
      output = update(next(iter(PROFILES.values())), state, sample)
      state = output.state  # Always advance proposal history, even on rejection.
      address, wire, bus = encode_offline(packer, output, e['proposal_index'] % 16)
      proposal = {'input': asdict(sample), 'output': asdict(output), 'unmodified_wire_hex': wire.hex()}
      if fault == 'excessive_tx' and t == 500000:
        wire, injected = bytes(lmc(angle=700)), True
      e.update(address=address, bus=bus, data_hex=wire.hex())
    if t == 500000 and e['address'] == 0x415 and not e['tx']:
      if fault == 'stale_rx':
        rows.append({'event': e, 'injected': True, 'operation': 'drop_one_rx', 'accepted': None,
                     'before': before, 'after': snapshot(lib, state, t), 'proposal': proposal,
                     'reason_bits': None, 'failed_checks': [], 'reason_names': []})
        continue
      if fault == 'corrupt_rx':
        data = bytearray.fromhex(e['data_hex'])
        data[3] ^= 1  # Corrupt checksum; do not repair flags/checksum or seed health.
        e['data_hex'], injected = data.hex(), True
    if fault == 'controls_off' and t == 500000 and e['address'] == 0x165:
      # Mutate only the cruise state; all monitored RX frames remain intact.
      e['data_hex'], injected = bytes(8).hex(), True
    data = (ctypes.c_ubyte * 8).from_buffer_copy(bytes.fromhex(e['data_hex']))
    accepted = bool(lib.audit_packet(e['address'], e['bus'], data, int(e['tx'])))
    bits = lib.audit_reasons() if e['tx'] else None
    failed = [checks[lib.audit_failed(i)] for i in range(lib.audit_nfailed())]
    rows.append({'event': e, 'injected': injected, 'operation': 'packet', 'accepted': accepted,
                 'before': before, 'after': snapshot(lib, state, t), 'proposal': proposal,
                 'reason_bits': bits, 'reason_names': [name for bit, name in REASONS.items() if bits and bits & bit],
                 'failed_checks': failed,
                 'untraced_rejection': not accepted and not failed and not bits,
                 'trace_limit': 'Only executed failed checks are listed; early return skips later predicates.'})
  return rows


def paired_audit(repo: Path, stock_repo: Path, output: Path) -> dict[str, Any]:
  result: dict[str, Any] = {'scope': 'host-only synthetic fixtures, not recorded vehicle performance', 'cases': {}, 'builds': {},
            'no_oracle_feedback': True, 'resets': 'once at beginning of each full run only',
            'state_limit': 'Exported stock desired_last and measurement range plus complete exported candidate history; not a dump of every stock global'}
  for fault in FAULTS:
    pair: dict[str, Any] = {}
    for label, injection in [('control', None), ('injected', fault)]:
      lib, meta = build(repo, stock_repo, output / fault / label, 'candidate', True)
      pair[label] = execute(lib, meta['checks'], injection)
      result['builds'][fault + '/' + label] = meta
    assert all(a['proposal'] == b['proposal'] for a, b in zip(pair['control'], pair['injected'], strict=True))
    pair['divergences'] = [{'index': i, 'time_us': a['event']['time_us'],
                            'acceptance_changed': a['accepted'] != b['accepted'],
                            'history_changed': a['after'] != b['after'], 'injected_here': b['injected']}
                           for i, (a, b) in enumerate(zip(pair['control'], pair['injected'], strict=True))
                           if a['accepted'] != b['accepted'] or a['after'] != b['after']]
    pair['unmodified_suffix_rejections'] = [i for i, r in enumerate(pair['injected'])
                                            if r['event']['tx'] and not r['injected'] and r['accepted'] is False]
    result['cases'][fault] = pair
  result['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
  result['input_source_sha256'] = {str(p.relative_to(stock_repo)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in (stock_repo / 'opendbc/car/ford/navigator_a3.py',
                                            stock_repo / 'opendbc/safety/tests/navigator_a3/core_path_angle.h')}
  candidate_lines = (stock_repo / 'opendbc/safety/tests/navigator_a3/core_path_angle.h').read_text().splitlines()
  result['candidate_predicate_sources'] = [{'line': n, 'source': line.strip()}
                                           for n, line in enumerate(candidate_lines, 1)
                                           if 'a3_reasons|=' in line]
  output.mkdir(parents=True, exist_ok=True)
  (output / 'rejection-cases.json').write_text(json.dumps(result, indent=2))
  return result


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--donor-repo', type=Path, default=Path('/Users/alex/Apps/bluepilot'))
  p.add_argument('--stock-repo', type=Path, required=True)
  p.add_argument('--output', type=Path, required=True)
  args = p.parse_args()
  result = paired_audit(args.donor_repo, args.stock_repo, args.output)
  print(json.dumps({k: {'divergences': len(v['divergences']), 'suffix_rejections': len(v['unmodified_suffix_rejections'])}
                    for k, v in result['cases'].items()}))


if __name__ == '__main__':
  main()
