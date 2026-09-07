"""Offline compiled Ford admission differential. Never connects to hardware.

Builds exact pinned donor and stock headers, plus a donor with only the four
latch violation-clearing assignments removed. Instrumentation records each
executed OR-check once and preserves its return value and side effects.
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import io
import json
from pathlib import Path
import re
import shutil
import subprocess
import tarfile

DONOR = '3210caa02d9e09b46d08be490f689edb23b22b20'
STOCK = 'b4ef5e1cf406ff143fa67bdbfb154739d43279c9'


def extract(repo: Path, rev: str, prefix: str, destination: Path) -> None:
  blob = subprocess.check_output(['git', '-C', str(repo), 'archive', rev, prefix + 'opendbc/safety'])
  with tarfile.open(fileobj=io.BytesIO(blob)) as archive:
    for member in archive.getmembers():
      if member.isfile():
        path = destination / member.name.removeprefix(prefix)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(archive.extractfile(member).read())


def instrument(root: Path) -> dict:
  checks = {}
  for rel in ['opendbc/safety/modes/ford.h', 'opendbc/safety/lateral.h']:
    p = root / rel
    output = []
    for line_number, line in enumerate(p.read_text().splitlines(), 1):
      match = re.match(r'(\s*)(violation|curvature_violation) \|= (.*?);(.*)', line)
      if match:
        key = len(checks) + 1
        checks[key] = {'file': rel, 'line': line_number, 'expression': match[3]}
        line = f'{match[1]}{match[2]} |= audit_check({key}, ({match[3]}));{match[4]}'
      else:
        assignment = re.match(r'(\s*)(bool )?violation = (.*?);(.*)', line)
        if assignment and assignment[3] not in ('false', 'true'):
          key = len(checks) + 1
          checks[key] = {'file': rel, 'line': line_number, 'expression': assignment[3]}
          line = f'{assignment[1]}{assignment[2] or ""}violation = audit_check({key}, ({assignment[3]}));{assignment[4]}'
      output.append(line)
    p.write_text('\n'.join(output) + '\n')
  return checks


WRAPPER = r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
static uint32_t timer_cnt;
uint32_t microsecond_timer_get(void) { return timer_cnt; }
static int failed[512];
static int nfailed;
static bool audit_check(int id, bool result) { if (result && nfailed < 512) failed[nfailed++] = id; return result; }
#include "opendbc/safety/can.h"
#include "opendbc/safety/safety.h"
AUDIT_CANDIDATE_INCLUDE
int audit_failed(int i) { return i < nfailed ? failed[i] : -1; }
int audit_nfailed(void) { return nfailed; }
void audit_timer(uint32_t t) { timer_cnt = t; }
int audit_packet_len(int address, int bus, unsigned char *data, int tx, int length) {
 CANPacket_t p = {0}; p.addr = address; p.bus = bus; p.data_len_code = length; memcpy(p.data, data, 8);
 nfailed = 0;
 AUDIT_CANDIDATE_DISPATCH
 return tx ? safety_tx_hook(&p) : safety_rx_hook(&p);
}
int audit_packet(int address, int bus, unsigned char *data, int tx) { return audit_packet_len(address,bus,data,tx,8); }
void audit_relay(int value) { relay_malfunction=value; }
void audit_controls(int value) { controls_allowed = value; }
void audit_init(void) {
 timer_cnt = 0; set_safety_hooks(SAFETY_FORD, 2); controls_allowed = false;
 AUDIT_RESET
}
int audit_latch(void) { return AUDIT_LATCH; }
int audit_last(void) { return AUDIT_LAST; }
int audit_controls_get(void) { return controls_allowed; }
int audit_meas_min(void) { return AUDIT_MEAS.min; }
int audit_meas_max(void) { return AUDIT_MEAS.max; }
'''


def build(repo: Path, stock_repo: Path, output: Path, variant: str, traced: bool) -> tuple[ctypes.CDLL, dict]:
  root = output / (variant + ('-traced' if traced else '-plain'))
  root.mkdir(parents=True, exist_ok=True)
  donor = variant.startswith('donor')
  extract(repo if donor else stock_repo, DONOR if donor else STOCK, 'opendbc_repo/' if donor else '', root)
  ford = root / 'opendbc/safety/modes/ford.h'
  if variant == 'donor-no-clear':
    source = ford.read_text()
    # Exact isolated mutation: bookkeeping and all surrounding expressions retained.
    pattern = r'(reset_bypass_latch_counter = RESET_BYPASS_LATCH_DURATION;\s*)violation = false;|(?<=reset_bypass_latch_counter--;\n)      violation = false;'
    source, count = re.subn(pattern, lambda m: (m[1] or '') + '/* audit: retain violation */', source)
    assert count == 4, count
    ford.write_text(source)
  checks = instrument(root) if traced else {}
  source = WRAPPER.replace(
    'AUDIT_RESET',
    'reset_bypass_latch_counter = 0; desired_path_angle_last = 0; desired_path_offset_last = 0; desired_curvature_rate_last = 0; '
    + 'ford_bp_angle_mode_engaged = false; ford_bp_shadow_curvature_raw = 0; controls_allowed_lateral = false;'
    if donor
    else '',
  )
  source = source.replace('AUDIT_LATCH', 'reset_bypass_latch_counter' if donor else '0')
  source = source.replace('AUDIT_LAST', 'desired_path_angle_last' if donor else 'curvature_state.desired_last')
  source = source.replace('AUDIT_MEAS', 'angle_meas' if donor else 'curvature_state.meas')
  if variant == 'candidate':
    target = root / 'opendbc/safety/tests/navigator_a3/core_path_angle.h'
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(stock_repo / 'opendbc/safety/tests/navigator_a3/core_path_angle.h', target)
    source = source.replace(
      'AUDIT_CANDIDATE_INCLUDE',
      '#include "opendbc/safety/tests/navigator_a3/core_path_angle.h"\n'
      + 'unsigned int audit_reasons(void) { return a3_reasons; }\nvoid audit_profile(int p) { a3_profile=p; }\n'
      + 'unsigned int audit_candidate_state(int i) { unsigned int v[]={a3_angle_last,a3_tx_time,a3_tx_seen,a3_profile,a3_reasons,'
      + 'a3_rx_time[0],a3_rx_time[1],a3_rx_time[2],a3_rx_seen[0],a3_rx_seen[1],a3_rx_seen[2],a3_rx_valid[0],a3_rx_valid[1],a3_rx_valid[2]}; return v[i]; }',
    )
    source = source.replace(
      'AUDIT_CANDIDATE_DISPATCH',
      'if (!tx) { bool valid=safety_rx_hook(&p); a3_observe_rx(&p,valid); return valid; } if (address==FORD_LateralMotionControl2) return a3_admit(&p);',
    )
    source = source.replace('controls_allowed = false;\n', 'controls_allowed = false; a3_reset();\n')
  else:
    source = source.replace(
      'AUDIT_CANDIDATE_INCLUDE',
      'unsigned int audit_reasons(void) { return 0; }\nvoid audit_profile(int p) { (void)p; }\n'
      + 'unsigned int audit_candidate_state(int i) { (void)i; return 0; }',
    ).replace('AUDIT_CANDIDATE_DISPATCH', '')
  (root / 'wrapper.c').write_text(source)
  command = ['cc', '-shared', '-fPIC', '-std=gnu11', '-O0', '-DALLOW_DEBUG', '-I', str(root), str(root / 'wrapper.c'), '-o', str(root / 'audit.so')]
  subprocess.run(command, check=True)
  lib = ctypes.CDLL(str(root / 'audit.so'))
  lib.audit_packet.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int]
  return lib, {'checks': checks, 'compile_command': command, 'library_sha256': hashlib.sha256((root / 'audit.so').read_bytes()).hexdigest()}


def lmc(angle=0, curvature=0, offset=0, rate=0, active=True):
  a, c, o, r = angle + 1000, curvature + 1000, offset + 512, rate + 1024
  return [16 if active else 0, 0, c >> 3, ((c & 7) << 5) | (a >> 6), ((a & 63) << 2) | (o >> 8), o & 255, r >> 3, (r & 7) << 5]


def event(data, *, address=0x3D6, tx=True, t=0, controls=None):
  return {'time_us': t, 'address': address, 'bus': 0, 'data_hex': bytes(data).hex(), 'tx': tx, 'controls_fixture': controls}


def inputs(speed_mps=25):
  # Real Ford RX encodings with checksum, counters and valid quality flags.
  def rx_at(i, t):
    speed = round(speed_mps * 360)
    count = i % 16
    brake = [speed >> 8, speed & 255, 0xC0 | (count << 2), 0, 0, 0, 0, 0]
    brake[3] = (255 - brake[0] - brake[1] - 3 - count) & 255
    pcm = [0, 0, 0, 0, 0x60, 0, speed >> 8, speed & 255]
    yaw = [0, 0, 32500 >> 8, 32500 & 255, 0, i % 256, 0x30, 0]
    yaw[4] = (255 - sum(yaw[:4]) - i % 256 - 3) & 255
    return [event(brake, address=0x415, tx=False, t=t), event(pcm, address=0x202, tx=False, t=t), event(yaw, address=0x91, tx=False, t=t)]

  rx = [e for i in range(6) for e in rx_at(i, i * 10000)]
  angle_mode = event([0, 0, 0, 0, 1, 0, 0, 0], address=0x3CA, t=50000)
  common = rx + [angle_mode]

  def seq(*frames):
    return common + [
      e for i, (data, controls) in enumerate(frames) for e in rx_at(i + 6, 100000 + i * 50000) + [event(data, t=100000 + i * 50000, controls=controls)]
    ]

  cases = {
    'normal-small-angle': seq((lmc(angle=2), True)),
    'normal-inactive': seq((lmc(active=False), False)),
    'bounded-reengagement': seq((lmc(active=False), False), (lmc(angle=2), True), (lmc(angle=4), True)),
    'controls-off-reset': seq((lmc(), False)),
    'controls-off-active-after-reset': seq((lmc(active=False), False), (lmc(angle=2), False)),
    'path-rate-after-reset': seq((lmc(), True), (lmc(angle=700), True)),
    'offset-magnitude-on-reset': seq((lmc(offset=200), True)),
    'inactive-nonzero-after-reset': seq((lmc(active=False), False), (lmc(angle=2, active=False), False)),
    'curvature-magnitude-after-reset': seq((lmc(), True), (lmc(curvature=1047), True)),
    'shadow-error-after-reset': common
    + [event(lmc(), t=100000, controls=True), event([0, 0, 0, 0, 1, 0x4E, 0x20, 0], address=0x3CA, t=120000), event(lmc(angle=2), t=150000)],
    'latch-expiry-controls-off': seq((lmc(active=False), False), *[(lmc(angle=2), False) for _ in range(62)]),
    'rearm-keeps-window-open': seq(*[(lmc() if i % 2 == 0 else lmc(angle=700), False) for i in range(65)]),
  }
  cases['stale-rx'] = common + [event(lmc(angle=2), t=200001, controls=True)]
  bad = rx_at(6, 100000)
  corrupt = bytearray.fromhex(bad[0]['data_hex'])
  corrupt[2] &= 63
  bad[0]['data_hex'] = corrupt.hex()
  cases['invalid-rx'] = common + bad + [event(lmc(angle=2), t=100000, controls=True)]
  cases['too-fast-cadence'] = seq((lmc(angle=2), True)) + [event(lmc(angle=4), t=110000, controls=True)]
  cases['rx-cruise-engagement'] = common + [event([0, 4, 0, 0, 0, 0, 0, 0], address=0x165, tx=False, t=90000), event(lmc(angle=2), t=100000)]
  cases['rx-cruise-disengagement'] = cases['rx-cruise-engagement'] + [
    event([0, 0, 0, 0, 0, 0, 0, 0], address=0x165, tx=False, t=110000),
    event(lmc(angle=4), t=150000),
  ]
  return cases


def run(lib, cases, checks):
  output = {}
  for name, sequence in cases.items():
    lib.audit_init()
    rows = []
    for e in sequence:
      lib.audit_timer(e['time_us'])
      if e['controls_fixture'] is not None:
        lib.audit_controls(e['controls_fixture'])
      before = {
        'latch': lib.audit_latch(),
        'last_command': lib.audit_last(),
        'controls_allowed': bool(lib.audit_controls_get()),
        'measurement_min': lib.audit_meas_min(),
        'measurement_max': lib.audit_meas_max(),
        'candidate_state': [lib.audit_candidate_state(j) for j in range(14)],
      }
      data = (ctypes.c_ubyte * 8).from_buffer_copy(bytes.fromhex(e['data_hex']))
      accepted = bool(lib.audit_packet(e['address'], e['bus'], data, e['tx']))
      failures = [checks[lib.audit_failed(i)] for i in range(lib.audit_nfailed())]
      rows.append(
        {
          'input': e,
          'accepted': accepted,
          'before': before,
          'after': {'latch': lib.audit_latch(), 'last_command': lib.audit_last(), 'controls_allowed': bool(lib.audit_controls_get())},
          'failed_checks': failures,
          'candidate_reasons': lib.audit_reasons(),
          'candidate_state_after': [lib.audit_candidate_state(j) for j in range(14)],
        }
      )
    output[name] = rows
  return output


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--donor-repo', type=Path, default=Path('/Users/alex/Apps/bluepilot'))
  parser.add_argument('--stock-repo', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  result = {
    'donor_commit': DONOR,
    'stock_commit': STOCK,
    'fixture_note': 'Synthetic commands and trusted-encoding RX. controls_allowed is explicitly seeded in test fixtures; '
    + 'this does not simulate a complete engagement stack.',
    'variants': {},
    'builds': {},
    'differences': [],
  }
  libraries = {}
  for variant in ['stock', 'donor', 'donor-no-clear', 'candidate']:
    plain, plain_meta = build(args.donor_repo, args.stock_repo, args.output, variant, False)
    traced, traced_meta = build(args.donor_repo, args.stock_repo, args.output, variant, True)
    libraries[variant] = (traced, traced_meta['checks'])
    p = run(plain, inputs(), {})
    t = run(traced, inputs(), traced_meta['checks'])
    for name in p:
      for a, b in zip(p[name], t[name], strict=True):
        assert {k: v for k, v in a.items() if k != 'failed_checks'} == {k: v for k, v in b.items() if k != 'failed_checks'}, (variant, name)
    result['variants'][variant] = t
    result['builds'][variant] = {'plain': plain_meta, 'traced': traced_meta, 'instrumentation_equivalent': True}
  for name, rows in result['variants']['donor'].items():
    for i, row in enumerate(rows):
      other = result['variants']['donor-no-clear'][name][i]
      if row['accepted'] != other['accepted']:
        assert other['failed_checks'], (name, i)
        result['differences'].append(
          {
            'case': name,
            'index': i,
            'first_divergence': not any(d['case'] == name for d in result['differences']),
            'donor': row,
            'without_clear': other,
            'classification': 'rejection must remain for controls-off, excessive, or malformed fixture; no valid-transition difference found',
            'state_diverged_before': row['before'] != other['before'],
            'stock': result['variants']['stock'][name][i],
            'candidate': result['variants']['candidate'][name][i],
          }
        )
  result['isolated_first_divergences'] = []
  for difference in result['differences']:
    if difference['first_divergence']:
      name, index = difference['case'], difference['index']
      prefix = {name: inputs()[name][: index + 1]}
      left = run(libraries['donor'][0], prefix, libraries['donor'][1])[name][-1]
      right = run(libraries['donor-no-clear'][0], prefix, libraries['donor-no-clear'][1])[name][-1]
      assert left == difference['donor'] and right == difference['without_clear']
      result['isolated_first_divergences'].append({'case': name, 'index': index, 'fresh_state_prefix_reproduced': True})
  args.output.mkdir(parents=True, exist_ok=True)
  (args.output / 'admission-results.json').write_text(json.dumps(result, indent=2))
  print(json.dumps({'cases': len(inputs()), 'admission_differences': len(result['differences']), 'instrumentation_equivalent': True}))


if __name__ == '__main__':
  main()
