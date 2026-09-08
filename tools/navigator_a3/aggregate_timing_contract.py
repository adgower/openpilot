"""Host-only strict/aggregate timing comparison; no device or physical-model proof."""
from __future__ import annotations
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from tools.navigator_a3.angle_handoff_contract import fixtures as handoff_fixtures
from tools.navigator_a3.safety_audit import build, event, inputs, lmc, STOCK
from tools.navigator_a3.rejection_diagnostics import REASONS

PIN = 'f95f996f5917dcbbf2e32fe51b606a24cf836af6'
PIN_URL = f'https://github.com/sunnypilot/opendbc/blob/{PIN}/opendbc/safety/lateral.h#L177-L196'

PINNED_RATE_FUNCTION = 'static bool rt_curvature_rate_limit_check(CurvatureSteeringLimits limits) {\n  bool violation = false;\n  uint32_t ts = microsecond_timer_get();\n\n  // *** curvature real time rate limit check ***\n  int max_rt_msgs = ((float)limits.frequency * MAX_RT_INTERVAL / 1e6 * 1.2) + 1;  // 1.2x buffer\n  uint32_t rt_msgs = curvature_state.rt_msgs + curvature_state.rt_msgs_prev;\n  if ((int)rt_msgs > max_rt_msgs) {\n    violation = true;\n  }\n  curvature_state.rt_msgs += 1U;\n\n  //roll the window every half interval\n  if (safety_get_ts_elapsed(ts, curvature_state.ts_check_last) >= (MAX_RT_INTERVAL / 2U)) {\n    curvature_state.rt_msgs_prev = curvature_state.rt_msgs;\n    curvature_state.rt_msgs = 0U;\n    curvature_state.ts_check_last = ts;\n  }\n\n  return violation;\n}'


def compiled(repo, output, variant, traced):
  # Preserve old build variants. Its frozen stock helper already is the pinned
  # aggregate algorithm; the historical header adds an independent 50 ms veto.
  lib, meta = build(Path('/Users/alex/Apps/bluepilot'), repo, output/variant, 'candidate', traced)
  root = output/variant/('candidate-traced' if traced else 'candidate-plain')
  lateral = subprocess.check_output(['git', '-C', str(repo), 'show', f'{STOCK}:opendbc/safety/lateral.h'], text=True)
  fn = lateral[lateral.index('static bool rt_curvature_rate_limit_check'):lateral.index('static bool steer_angle_cmd_inactive_check')].strip()
  assert fn == PINNED_RATE_FUNCTION, 'Frozen stock aggregate helper differs from pinned source'
  declarations = (root/'opendbc/safety/declarations.h').read_text()
  assert '#define MAX_RT_INTERVAL 250000U' in declarations
  name = 'core_path_angle_aggregate.h' if variant == 'aggregate' else 'core_path_angle.h'
  header = repo/'opendbc/safety/tests/navigator_a3'/name
  shutil.copyfile(header, root/'opendbc/safety/tests/navigator_a3/core_path_angle.h')
  wrapper = root/'wrapper.c'
  wrapper.write_text(wrapper.read_text()+'''
unsigned int audit_bucket(int i) {
  unsigned int v[]={curvature_state.rt_msgs,curvature_state.rt_msgs_prev,curvature_state.ts_check_last}; return v[i];
}
int audit_power(void) { return curvature_state.steer_power_last; }
int audit_equivalent(void) { return (int)roundf(a3_equivalent_curvature*50000.f); }
int audit_frequency(void) { return FORD_STEERING_LIMITS.frequency; }
int audit_rate_probe(void) { return rt_curvature_rate_limit_check(FORD_STEERING_LIMITS); }
''')
  # Use a different filename: dlopen caches paths already loaded by build().
  command = list(meta['compile_command'])
  command[-1] = str(root/'timing.so')
  subprocess.run(command, check=True)
  lib = ctypes.CDLL(command[-1])
  lib.audit_packet.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int]
  assert lib.audit_frequency() == 20
  meta.update(compile_command=command, library_sha256=hashlib.sha256(Path(command[-1]).read_bytes()).hexdigest(),
              header_sha256=hashlib.sha256(header.read_bytes()).hexdigest(),
              pinned_function_sha256=hashlib.sha256(fn.encode()).hexdigest(), pinned_function_byte_identical=True)
  return lib, meta


def fixtures():
  # Existing proposal-derived cases exercise actual host driver transitions.
  original = handoff_fixtures()
  rx = [e for e in inputs(25.)['latch-expiry-controls-off'] if not e['tx']]
  cases = {}
  for p in range(4):
    for sign in (-1, 1):
      for kind in ('jitter', 'compressed', 'wrap', 'driver', 'overspeed', 'burst', 'excessive', 'permission', 'inactive-nonzero', 'wire-curvature', 'wire-offset', 'wire-rate', 'wire-mode', 'stale-rx', 'invalid-rx', 'stock-jerk', 'stock-measured', 'stock-absolute', 'stock-accel', 'speed-mismatch'):
        speed=60. if kind in ('stock-jerk','stock-accel') else 1. if kind=='stock-absolute' else 25.
        rx=[e for e in inputs(speed)['latch-expiry-controls-off'] if not e['tx']]
        if kind in ('jitter', 'wrap'):
          intervals = [0]+[49000 if i%2 else 51000 for i in range(1,24)]
        elif kind == 'compressed':
          intervals = [0]+[10000 if i%2 else 90000 for i in range(1,24)]
        elif kind in ('overspeed', 'burst', 'speed-mismatch'):
          intervals = [0]+[10000 if kind=='overspeed' else 1000]*39
        elif kind in ('stock-measured','stock-accel'):
          intervals=[0]+[50000]*59
        else:
          intervals = [0]+[50000]*4
        t = 100000
        times=[]
        for dt in intervals:
          t += dt
          times.append(t)
        shift = 0xFFFFFFFF-225000 if kind=='wrap' else 0
        def shifted(e):
          return dict(e, time_us=(e['time_us']+shift)&0xFFFFFFFF, controls_fixture=None)
        sequence = [shifted(e) for e in rx[:18]]
        sequence.append(shifted(event([0,4,0,0,0,0,0,0],address=0x165,tx=False,t=90000)))
        for i,t in enumerate(times):
          if kind!='stale-rx':
            current_rx=[shifted(dict(e,time_us=t)) for e in rx[18+i*3:21+i*3]]
            if kind=='invalid-rx':
              corrupt=bytearray.fromhex(current_rx[0]['data_hex'])
              corrupt[3]^=1
              current_rx[0]['data_hex']=corrupt.hex()
            if kind=='speed-mismatch' and i>=9:
              data=bytearray.fromhex(current_rx[1]['data_hex'])
              data[6],data[7]=(35*360)>>8,(35*360)&255
              current_rx[1]['data_hex']=data.hex()
            sequence.extend(current_rx)
          if kind=='permission' and i==1:
            sequence.append(shifted(event(bytes(8),address=0x165,tx=False,t=t)))
          # Modest fixed encoded demand isolates timing; transitions never
          # consume admission feedback. Driver goes neutral and returns bounded.
          wire = lmc(angle=sign*2)
          if kind=='excessive' and i==1: wire=lmc(angle=sign*700)
          if (kind=='driver' or kind=='permission') and i==2: wire=lmc(active=False)
          if kind=='inactive-nonzero': wire=lmc(angle=sign*2,active=False)
          if kind=='wire-curvature': wire=lmc(angle=sign*2,curvature=sign)
          if kind=='wire-offset': wire=lmc(angle=sign*2,offset=sign)
          if kind=='wire-rate': wire=lmc(angle=sign*2,rate=sign)
          if kind=='wire-mode': wire[0]=32
          if kind=='stock-jerk': wire=lmc(angle=sign*18)
          if kind=='stock-absolute': wire=lmc(angle=sign*110)
          if kind=='stock-measured': wire=lmc(angle=sign*8*(i+1))
          if kind=='stock-accel': wire=lmc(angle=sign*4*(i+1))
          sequence.append(shifted(event(wire,t=t+100001 if kind=='stale-rx' else t)))
        cases[f'{kind}-p{p}-s{sign}'] = {'profile':p,'sign':sign,'sequence':sequence}
  # Preserve proposal-generated 49/51 ms cascade and host driver handoff evidence.
  for name,case in original.items():
    if name in ('jitter-49-51ms', 'driver-handoff'):
      cases['host-'+name]=case
  return cases


def execute(lib, checks, case):
  lib.audit_init()
  lib.audit_profile(case['profile'])
  def state():
    candidate = [lib.audit_candidate_state(i) for i in range(14)]
    return {'accepted_reference': {'angle_signed_counts':ctypes.c_int32(candidate[0]).value,
      'tx_time_us':candidate[1]&0xFFFFFFFF, 'tx_seen':bool(candidate[2]),
      'stock_desired_last':lib.audit_last(), 'stock_steer_power_last':lib.audit_power()},
      'rate_buckets':dict(zip(('current','previous','last_roll_us'),[lib.audit_bucket(i)&0xFFFFFFFF for i in range(3)])),
      'compiled_controls_allowed':bool(lib.audit_controls_get()),
      'last_evaluated_equivalent_curvature_signed_counts':lib.audit_equivalent(),
      'compiled_measured_curvature_range':[lib.audit_meas_min(),lib.audit_meas_max()]}
  rows=[]
  for original in case['sequence']:
    e={k:v for k,v in original.items() if k!='proposal'}
    lib.audit_timer(e['time_us'])
    before=state()
    wire=(ctypes.c_ubyte*8).from_buffer_copy(bytes.fromhex(e['data_hex']))
    accepted=bool(lib.audit_packet(e['address'],e['bus'],wire,int(e['tx'])))
    bits=lib.audit_reasons() if e['tx'] else 0
    rows.append({'event':e,'proposal':original.get('proposal'),'accepted':accepted,'before':before,'after':state(),
      'reason_bits':bits,'reason_names':[name for bit,name in REASONS.items() if bits&bit],
      'failed_checks':[checks[lib.audit_failed(i)] for i in range(lib.audit_nfailed())],
      'physical_eps_response':'unknown', 'attribution_limit':'Executed predicates only; early-return checks are not attributed.'})
  return rows


def audit(repo, output):
  output.mkdir(parents=True,exist_ok=True)
  cases=fixtures()
  report={'schema_version':1,'cases':{k:{} for k in cases},'builds':{},'instrumentation_equivalent':True,
    'physical_mapping_validated':False,'physical_limits_validated':False,'no_oracle_feedback':True,
    'policy':{'strict-historical':'Historical self-imposed 50000 us per-frame veto plus frozen stock aggregate rate algorithm.',
      'aggregate':'Pinned upstream algorithm already in frozen stock, experimental host-only integration removes extra veto; rejected commands preserve accepted desired history.',
      'source_commit':PIN,'source_url':PIN_URL,'max_rt_interval_us':250000,'bucket_roll_us':125000,'nominal_frequency_hz':20,
      'max_rt_msgs':7,'ordering':'Check current+previous >7, increment current, then roll if elapsed>=125000 us. Rejected stock checks retain attempt bucket changes; early-rejected path/wire checks skip aggregate check.',
      'limitations':'Synthetic encoded command sequences and separately included host proposals; no vehicle, firmware or physical-model validation.'}}
  for variant in ('strict-historical','aggregate'):
    pair={}
    for traced in (False,True):
      lib,meta=compiled(repo,output/'compiled',variant,traced)
      pair[traced]={name:execute(lib,meta['checks'],case) for name,case in cases.items()}
      probes={}
      for name,times in {'burst':[100000]*10,'roll':[0,124999,125000,249999,250000,500001],
                         'wrap':[0xffff0000,0xffff1000,0x00010000,0x00030000]}.items():
        lib.audit_init()
        rows=[]
        for t in times:
          lib.audit_timer(t)
          before=[lib.audit_bucket(i)&0xffffffff for i in range(3)]
          rejected=bool(lib.audit_rate_probe())
          rows.append({'time_us':t,'before':before,'rejected':rejected,
                       'after':[lib.audit_bucket(i)&0xffffffff for i in range(3)]})
        probes[name]=rows
      meta['direct_rate_probes']=probes
      report['builds'][f'{variant}-{"traced" if traced else "plain"}']=meta
    for name in cases:
      for a,b in zip(pair[False][name],pair[True][name],strict=True):
        assert {k:v for k,v in a.items() if k!='failed_checks'}=={k:v for k,v in b.items() if k!='failed_checks'},(variant,name)
      report['cases'][name][variant]=pair[True][name]
  report['compiler']=subprocess.check_output(['cc','--version'],text=True)
  source_paths=[Path(__file__),Path(__file__).with_name('safety_audit.py'),
    repo/'opendbc/safety/tests/navigator_a3/core_path_angle.h',
    repo/'opendbc/safety/tests/navigator_a3/core_path_angle_aggregate.h',
    repo/'opendbc/car/ford/navigator_a3.py']
  report['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
  historical=repo/'opendbc/safety/tests/navigator_a3/core_path_angle.h'
  pinned_historical=subprocess.check_output(['git','-C',str(repo),'show',
    '12a4366c866b8b3d8bac46a0e730e337351567a0:opendbc/safety/tests/navigator_a3/core_path_angle.h'])
  assert historical.read_bytes()==pinned_historical
  report['historical_header_byte_identical']=True
  report['differences']={}
  for name,variants in report['cases'].items():
    diffs=[]
    for i,(strict,aggregate) in enumerate(zip(variants['strict-historical'],variants['aggregate'],strict=True)):
      change={'event_index':i,'time_us':strict['event']['time_us'],
        'admission_changed':strict['accepted']!=aggregate['accepted'],
        'accepted_reference_changed':strict['after']['accepted_reference']!=aggregate['after']['accepted_reference'],
        'rate_buckets_changed':strict['after']['rate_buckets']!=aggregate['after']['rate_buckets']}
      if any(change[k] for k in ('admission_changed','accepted_reference_changed','rate_buckets_changed')): diffs.append(change)
    report['differences'][name]={'events':diffs,
      'first_admission_divergence':next((r['event_index'] for r in diffs if r['admission_changed']),None),
      'first_history_divergence':next((r['event_index'] for r in diffs if r['accepted_reference_changed']),None)}
  report['summary']={name:{variant:{'accepted_tx':sum(r['accepted'] for r in rows if r['event']['tx']),
    'rejected_tx':sum(not r['accepted'] for r in rows if r['event']['tx'])} for variant,rows in variants.items()}
    for name,variants in report['cases'].items()}
  (output/'aggregate-timing-cases.json').write_text(json.dumps(report,indent=2))
  (output/'aggregate-timing-summary.json').write_text(json.dumps(report['summary'],indent=2))
  return report

if __name__=='__main__':
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--stock-repo',type=Path,required=True)
  parser.add_argument('--output',type=Path,required=True)
  args=parser.parse_args()
  result=audit(args.stock_repo,args.output)
  print(json.dumps({'cases':len(result['cases']),'instrumentation_equivalent':result['instrumentation_equivalent']}))
