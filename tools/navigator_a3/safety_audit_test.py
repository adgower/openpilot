"""Positive and negative requirements independent of donor decisions."""

import ctypes
from pathlib import Path

import pytest

from tools.navigator_a3.safety_audit import build, inputs, run, lmc


@pytest.fixture(scope='module')
def variants(tmp_path_factory):
  out = tmp_path_factory.mktemp('a3-safety')
  results = {}
  for name in ['stock', 'donor', 'donor-no-clear', 'candidate']:
    lib, meta = build(Path('/Users/alex/Apps/bluepilot'), Path('/private/tmp/navigator-a3-opendbc'), out, name, True)
    results[name] = (lib, run(lib, inputs(), meta['checks']))
  return results


@pytest.mark.parametrize('case', ['normal-small-angle', 'normal-inactive', 'bounded-reengagement', 'rx-cruise-engagement'])
def test_positive_candidate_acceptance(variants, case):
  assert all(row['accepted'] for row in variants['candidate'][1][case] if row['input']['address'] == 0x3D6)


@pytest.mark.parametrize('case', ['normal-small-angle', 'normal-inactive', 'bounded-reengagement', 'rx-cruise-engagement'])
def test_normal_donor_without_bypass_remains_accepted(variants, case):
  assert all(row['accepted'] for row in variants['donor-no-clear'][1][case] if row['input']['address'] == 0x3D6)


@pytest.mark.parametrize(
  ('case', 'reason'),
  [
    ('stale-rx', 2),
    ('invalid-rx', 2),
    ('too-fast-cadence', 4),
    ('controls-off-reset', 32),
    ('controls-off-active-after-reset', 32),
    ('path-rate-after-reset', 512),
    ('offset-magnitude-on-reset', 1),
    ('inactive-nonzero-after-reset', 16),
    ('curvature-magnitude-after-reset', 1),
    ('rx-cruise-disengagement', 32),
  ],
)
def test_candidate_rejections(variants, case, reason):
  row = variants['candidate'][1][case][-1]
  assert not row['accepted']
  assert row['candidate_reasons'] & reason


def test_stock_production_still_blocks_path_angle(variants):
  assert not variants['stock'][1]['normal-small-angle'][-1]['accepted']
  assert variants['stock'][1]['normal-inactive'][-1]['accepted']


def test_latch_expires_by_message_count_not_time(variants):
  rows = [r for r in variants['donor'][1]['latch-expiry-controls-off'] if r['input']['address'] == 0x3D6]
  assert all(r['accepted'] for r in rows[:61])  # reset plus sixty suppressed messages
  assert not rows[61]['accepted']
  assert not rows[62]['accepted']
  # Repeated neutral messages re-arm indefinitely, including with controls off.
  assert all(r['accepted'] for r in variants['donor'][1]['rearm-keeps-window-open'] if r['input']['address'] == 0x3D6)


@pytest.mark.parametrize('profile', [0, 1, 2, 3])
@pytest.mark.parametrize('angle', [-2, 2])
@pytest.mark.parametrize('speed', [1, 13.5, 26.82, 35])
def test_both_directions_each_supported_profile(variants, profile, angle, speed):
  lib = variants['candidate'][0]
  lib.audit_init()
  for row in inputs(speed)['normal-small-angle'][:-1]:
    lib.audit_timer(row['time_us'])
    d = (ctypes.c_ubyte * 8).from_buffer_copy(bytes.fromhex(row['data_hex']))
    lib.audit_packet(row['address'], 0, d, int(row['tx']))
  lib.audit_profile(profile)
  lib.audit_controls(1)
  lib.audit_timer(100000)
  d = (ctypes.c_ubyte * 8)(*lmc(angle=angle))
  assert lib.audit_packet(0x3D6, 0, d, 1)


def test_unsupported_profile_rejected(variants):
  lib = variants['candidate'][0]
  lib.audit_init()
  lib.audit_profile(4)
  d = (ctypes.c_ubyte * 8)(*lmc(active=False))
  assert not lib.audit_packet(0x3D6, 0, d, 1)
  assert lib.audit_reasons() & 8


def test_relay_fault_blocks_even_neutral(variants):
  lib = variants['candidate'][0]
  lib.audit_init()
  lib.audit_relay(1)
  data = (ctypes.c_ubyte * 8)(*lmc(active=False))
  assert not lib.audit_packet(0x3D6, 0, data, 1)
  assert lib.audit_reasons() & 256


def test_wrong_length_rx_cannot_refresh(variants):
  lib = variants['candidate'][0]
  lib.audit_init()
  for row in inputs()['normal-small-angle'][:-1]:
    lib.audit_timer(row['time_us'])
    data = (ctypes.c_ubyte * 8).from_buffer_copy(bytes.fromhex(row['data_hex']))
    lib.audit_packet_len(row['address'], 0, data, int(row['tx']), 7 if row['address'] == 0x415 else 8)
  lib.audit_timer(100000)
  lib.audit_controls(1)
  data = (ctypes.c_ubyte * 8)(*lmc(angle=2))
  assert not lib.audit_packet(0x3D6, 0, data, 1)
  assert lib.audit_reasons() & 2


@pytest.mark.parametrize('speed', [0, 61])
def test_speed_outside_core_domain(variants, speed):
  lib = variants['candidate'][0]
  result = run(lib, {'normal-small-angle': inputs(speed)['normal-small-angle']}, {})
  assert not result['normal-small-angle'][-1]['accepted']
  assert result['normal-small-angle'][-1]['candidate_reasons'] & 64


@pytest.mark.parametrize('profile_id', range(4))
@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('speed', [8.763889, 13.5, 23.76111, 26.82, 35., 60.])
def test_core_commands_and_driver_reentry_pass_unchanged_compiled_checks(variants, profile_id, sign, speed):
  from opendbc.can import CANPacker
  from opendbc.car.ford.navigator_a3 import PROFILES, Inputs, State, update, encode_offline
  lib = variants['candidate'][0]
  lib.audit_init()
  lib.audit_profile(profile_id)
  profile = list(PROFILES.values())[profile_id]
  state = State()
  packer = CANPacker('ford_lincoln_base_pt')
  frame = 0
  # Reuse checksum-valid, counter-incrementing RX fixture. Permission is explicit
  # test setup, not fabricated evidence in raw-log replay or production.
  for row in inputs(speed)['latch-expiry-controls-off']:
    lib.audit_timer(row['time_us'])
    if not row['tx']:
      data = (ctypes.c_ubyte * 8).from_buffer_copy(bytes.fromhex(row['data_hex']))
      assert lib.audit_packet(row['address'], 0, data, 0)
    elif row['address'] == 0x3D6:
      lib.audit_controls(1)
      t = row['time_us'] * 1000
      output = update(profile, state, Inputs(t, t, sign * .001, speed, True, frame in (20, 21)))
      state = output.state
      addr, wire, bus = encode_offline(packer, output, frame % 16)
      assert output.mode == (0 if frame in (20, 21) else 1)
      assert lib.audit_packet(addr, bus, (ctypes.c_ubyte * 8).from_buffer_copy(wire), 1), (frame, lib.audit_reasons())
      frame += 1
