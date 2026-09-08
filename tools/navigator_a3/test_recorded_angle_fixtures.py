"""Recorded donor evidence drives synthetic A3 proposals, never vehicle-response claims."""
import hashlib
import json
from pathlib import Path

import pytest
from opendbc.car.ford.navigator_a3 import Inputs, PROFILES, State, update
from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeBridge, RuntimeObserver
from tools.navigator_a3.transport_observer import Observer, resolve_candidates

FIXTURE = Path(__file__).with_name('fixtures') / 'recorded_angle_release.json'


def fixture():
  return json.loads(FIXTURE.read_text())


def test_fixture_integrity_and_native_time():
  f = fixture()
  assert f['provenance'] == 'recorded'
  assert f['route_id'] == '00000002--bfd966a552'
  assert f['donor_commit'] == '3210caa02d9e09b46d08be490f689edb23b22b20'
  assert hashlib.sha256(json.dumps(f['data'], sort_keys=True, separators=(',', ':')).encode()).hexdigest() == f['data_sha256']
  for rows in f['data']['warnings'].values():
    assert all(int(a['t_ns']) < int(b['t_ns']) for a, b in zip(rows, rows[1:]))
    assert any(r['driver_pressed'] == '1.0' for r in rows)
    assert all(int(r['carControl_t_ns']) <= int(r['t_ns']) for r in rows)


@pytest.mark.parametrize('name', PROFILES)
@pytest.mark.parametrize('window', ['warning-1', 'warning-2'])
def test_recorded_driver_release_bounded_no_pulse(name, window):
  state = State()
  previous_driver = False
  release_count = active_count = 0
  pending_release = False
  bounded_reentries = 0
  for row in fixture()['data']['warnings'][window]:
    now = int(row['t_ns'])
    # Downsample only to satisfy the strategy cadence; timestamps are never rewritten.
    if state.last_ns is not None and now - state.last_ns < 50_000_000:
      continue
    driver = row['driver_pressed'] == '1.0'
    sample = Inputs(now, int(row['carControl_t_ns']), float(row['general_curvature']), float(row['speed_mps']),
                    row['lat_active'] == '1.0', driver,
                    valid=row['carControl_valid'] == 'True' and row['carState_valid'] == 'True',
                    measured_curvature_inv_m=float(row['yaw_curvature']),
                    measurement_ns=min(int(row['rawYaw_t_ns']), int(row['rawSpeed_t_ns'])),
                    measurement_valid=row['rawYaw_valid'] == row['rawSpeed_valid'] == 'True')
    out = update(PROFILES[name], state, sample)
    assert not out.transmission_allowed
    if driver:
      assert out.mode == 0 and out.path_angle_rad == 0
    if out.mode:
      active_count += 1
      # These highway samples span the 15-to-25 m/s rate interpolation.
      assert sample.speed_mps >= 15
      limit = .009 + max(0., 25 - sample.speed_mps) * (.0425 - .009) / 10
      assert abs(out.path_angle_rad - state.path_angle_rad) <= limit + 1e-12
      assert abs(out.path_angle_rad / .0005 - round(out.path_angle_rad / .0005)) < 1e-9
      assert abs(out.measurement_limited_curvature_inv_m - sample.measured_curvature_inv_m) <= .002 + 1e-12
    if previous_driver and not driver:
      release_count += 1
      pending_release = True
      if now - state.last_ns > 100_000_000:
        assert out.mode == 0 and out.reason == 'cadence'
      else:
        assert out.mode == 1
    if pending_release and not driver and 50_000_000 <= now - state.last_ns <= 100_000_000:
      assert out.mode == 1  # No extra inactive recovery interval after cadence becomes valid.
    if pending_release and out.mode:
      bounded_reentries += 1
      pending_release = False
    state, previous_driver = out.state, driver
  assert release_count >= 1 and active_count > 100
  assert bounded_reentries == release_count


def test_recorded_active_rejection_keeps_all_five_candidates_and_fault():
  f = fixture()
  events = f['data']['active_rejection_events']
  observer = Observer()
  runtime = RuntimeBridge('shadow', [], f['route_id'], 'recorded')
  for e in events:
    observed = observer.consume(e)
    bounded = runtime.observe(e)
    if e['kind'] == 'rejected':
      assert observed['attribution'] == bounded['attribution'] == 'ambiguous'
      assert observed['candidate_count'] == len(bounded['candidate_orders']) == 5
      assert [x['order'] for x in resolve_candidates(observer.groups, observed)] == f['data']['candidate_orders']
      assert observed['publication_to_observation_ns'] is None
      assert runtime.fault_reason == 'direct_steering_rejection'
    assert observed['receiver_acceptance'] == bounded['receiver_acceptance'] == 'unknown'
  # A later returned frame is a synthetic transport variant, not new recorded evidence.
  returned = dict(events[-1], kind='returned', order=events[-1]['order'] + 1, t_ns=events[-1]['t_ns'] + 1)
  runtime.observe(returned)
  assert runtime.fault_reason == 'direct_steering_rejection'
  assert runtime.disengagement_requested is False  # Shadow must leave A2 lifecycle unchanged.


def test_old_echo_cannot_acknowledge_synthetic_publication():
  f = fixture()
  events = f['data']['active_rejection_events']
  observer = RuntimeObserver(f['route_id'], 'synthetic')
  pub = dict(events[0], provenance='synthetic')
  observer.consume(pub)
  result = observer.consume(events[-1])
  assert result['event_type'] == 'identity_mismatch'
  assert result['candidate_orders'] == []
  assert not result['direct_steering_rejection']
  assert result['receiver_acceptance'] == 'unknown'
