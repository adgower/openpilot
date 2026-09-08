"""Host-only timing contract: compiled acceptance, bucket state and history."""
import importlib.util
from pathlib import Path
import pytest

@pytest.fixture(scope='module')
def report(tmp_path_factory):
  path = Path(__file__).with_name('aggregate_timing_contract.py')
  assert path.exists(), 'Missing aggregate timing contract implementation'
  spec = importlib.util.spec_from_file_location('angle_timing_contract', path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module.audit(Path('/private/tmp/navigator-a3-opendbc'), tmp_path_factory.mktemp('timing'))

def tx(report, name, variant='aggregate'):
  return [r for r in report['cases'][name][variant] if r['event']['tx']]

def test_jitter_and_bounded_compression(report):
  for p in range(4):
    for s in (-1, 1):
      for fixture in ('jitter', 'compressed', 'wrap', 'driver'):
        name = f'{fixture}-p{p}-s{s}'
        assert all(r['accepted'] for r in tx(report, name)), name
      assert any(not r['accepted'] for r in tx(report, f'jitter-p{p}-s{s}', 'strict-historical'))

def test_overspeed_and_bursts_are_bounded(report):
  for p in range(4):
    for s in (-1, 1):
      for fixture in ('overspeed', 'burst'):
        rows = tx(report, f'{fixture}-p{p}-s{s}')
        assert any(r['accepted'] for r in rows)
        assert any('A3_CADENCE' in r['reason_names'] for r in rows)

def test_rejection_never_advances_accepted_history(report):
  for case in report['cases'].values():
    for r in case['aggregate']:
      if r['event']['tx'] and not r['accepted']:
        assert r['before']['accepted_reference'] == r['after']['accepted_reference']
  for p in range(4):
    for s in (-1, 1):
      rows = tx(report, f'excessive-p{p}-s{s}')
      assert rows[0]['accepted'] and not rows[1]['accepted'] and rows[2]['accepted']
      assert 'A3_PATH_RATE' in rows[1]['reason_names']
      permission = tx(report, f'permission-p{p}-s{s}')
      assert not permission[1]['accepted']
      assert permission[2]['accepted']
      assert not permission[2]['after']['compiled_controls_allowed']
      assert not permission[3]['accepted']

def test_equivalence_and_sequence_identity(report):
  assert report['instrumentation_equivalent']
  assert report['historical_header_byte_identical']
  assert not report['physical_mapping_validated']
  for case in report['cases'].values():
    assert [r['event'] for r in case['aggregate']] == [r['event'] for r in case['strict-historical']]
    assert all('rate_buckets' in r['before'] and 'rate_buckets' in r['after'] for r in case['aggregate'])


def test_final_encoded_bytes_stay_gated(report):
  for p in range(4):
    for s in (-1, 1):
      for kind in ('inactive-nonzero','wire-curvature','wire-offset','wire-rate','wire-mode','stale-rx','invalid-rx'):
        rows=tx(report,f'{kind}-p{p}-s{s}')
        assert all(not r['accepted'] for r in rows)
        assert all(r['reason_names'] for r in rows)

def test_exact_rate_check_increment_roll_order(report):
  reference=None
  for build in report['builds'].values():
    assert build['pinned_function_byte_identical']
    probes=build['direct_rate_probes']
    if reference is None: reference=probes
    assert probes==reference
    for rows in probes.values():
      current=previous=last=0
      for row in rows:
        assert row['before']==[current,previous,last]
        assert row['rejected']==(current+previous>7)
        current+=1
        if ((row['time_us']-last)&0xffffffff)>=125000:
          previous,current,last=current,0,row['time_us']
        assert row['after']==[current,previous,last]
    # The pinned check deliberately accepts the eighth attempt at startup.
    assert [r['rejected'] for r in probes['burst']]==[False]*8+[True]*2


def test_skipped_rate_check_is_not_attributed_as_cadence(tmp_path):
  spec=importlib.util.spec_from_file_location('aggregate_timing_contract',Path(__file__).with_name('aggregate_timing_contract.py'))
  module=importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  case=module.fixtures()['burst-p0-s1']
  count=0
  for e in case['sequence']:
    if e['tx']: count+=1
    if not e['tx'] and e['address']==0x202 and count>=9:
      wire=bytearray.fromhex(e['data_hex'])
      speed=35*360
      wire[6],wire[7]=speed>>8,speed&255
      e['data_hex']=wire.hex()
  lib,meta=module.compiled(Path('/private/tmp/navigator-a3-opendbc'),tmp_path,'aggregate',True)
  rows=module.execute(lib,meta['checks'],case)
  mismatch=next(r for r in rows if r['event']['tx'] and r['before']['compiled_controls_allowed'] and not r['after']['compiled_controls_allowed'])
  assert not mismatch['accepted']
  assert mismatch['before']['rate_buckets']==mismatch['after']['rate_buckets']
  assert mismatch['reason_names']==['A3_STOCK_ENVELOPE']
  assert mismatch['before']['accepted_reference']==mismatch['after']['accepted_reference']


def test_stock_per_command_limits_execute_inside_path_step_bound(report):
  predicates={
    'stock-jerk':'safety_max_limit_check(desired_curvature, highest_desired_curvature, lowest_desired_curvature)',
    'stock-measured':'safety_max_limit_check(desired_curvature, highest_desired_curvature, lowest_desired_curvature)',
    'stock-absolute':'safety_max_limit_check(desired_curvature, limits.max_curvature, -limits.max_curvature)',
    'stock-accel':'safety_max_limit_check(desired_curvature, max_curvature_can, -max_curvature_can)',
  }
  for p in range(4):
    for sign in (-1,1):
      for kind,predicate in predicates.items():
        rows=tx(report,f'{kind}-p{p}-s{sign}')
        rejected=next(r for r in rows if not r['accepted'])
        assert rejected['reason_names']==['A3_STOCK_ENVELOPE']
        assert predicate in [f['expression'] for f in rejected['failed_checks']]
        assert rejected['before']['accepted_reference']==rejected['after']['accepted_reference']
        assert rejected['before']['rate_buckets']!=rejected['after']['rate_buckets']
        equivalent=rejected['after']['last_evaluated_equivalent_curvature_signed_counts']
        previous=rejected['before']['accepted_reference']['stock_desired_last']
        if kind=='stock-jerk':
          assert abs(equivalent-previous)>3  # 60 m/s frozen stock jerk cap
          assert abs(equivalent)<=101  # measured error band is not exceeded
        if kind=='stock-measured':
          assert rejected['before']['compiled_measured_curvature_range']==[0,0]
          assert abs(equivalent)>101
          assert abs(equivalent-previous)<=16  # 25 m/s jerk cap respected
        if kind=='stock-absolute': assert abs(equivalent)>1000
        if kind=='stock-accel': assert abs(equivalent)>52  # 60 m/s accel cap
        if kind in ('stock-measured','stock-accel'):
          assert rows[0]['accepted']
          assert any(r['accepted'] for r in rows)
