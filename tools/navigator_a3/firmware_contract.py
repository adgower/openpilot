"""Shared production-header checks exercised through a host-only admission wrapper."""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from tools.navigator_a3.safety_audit import WRAPPER, instrument
from tools.navigator_a3.aggregate_timing_contract import fixtures, compiled, execute


def build(repo, output, traced=False, production=False):
  root=output/('production' if production else 'shared')/('traced' if traced else 'plain')
  shutil.copytree(repo/'opendbc/safety',root/'opendbc/safety',dirs_exist_ok=True)
  checks=instrument(root) if traced else {}
  exports='''
#include <math.h>
#include "opendbc/safety/tests/navigator_a3/core_path_angle_aggregate.h"
unsigned int audit_reasons(void) { return ford_a3_reasons; }
void audit_profile(int p) { ford_a3_profile=p; }
unsigned int audit_candidate_state(int i) {
 unsigned int v[]={ford_a3_angle_last,ford_a3_tx_time,ford_a3_tx_seen,ford_a3_profile,ford_a3_reasons,
 ford_a3_rx_time[0],ford_a3_rx_time[1],ford_a3_rx_time[2],ford_a3_rx_seen[0],ford_a3_rx_seen[1],ford_a3_rx_seen[2],ford_a3_rx_valid[0],ford_a3_rx_valid[1],ford_a3_rx_valid[2]}; return v[i];
}
unsigned int audit_bucket(int i) { unsigned int v[]={curvature_state.rt_msgs,curvature_state.rt_msgs_prev,curvature_state.ts_check_last}; return v[i]; }
int audit_power(void) { return curvature_state.steer_power_last; }
int audit_equivalent(void) { return (int)roundf(ford_a3_equivalent_curvature*50000.f); }
int audit_pending(void) { return ford_a3_pending_valid; }
void audit_probe_prepare(void) { ford_a3_reasons=0xffffffffU; }
int audit_math_parity(void) {
 int differences=0;
 for (int profile=0;profile<4;profile++) {
  a3_profile=profile; ford_a3_profile=profile;
  for (int speed=1;speed<=60;speed++) {
   for (int angle=-1000;angle<=1047;angle++) {
    float old=a3_inverse(angle*.0005f,(float)speed);
    float now=ford_a3_inverse(angle*.0005f,(float)speed);
    if (old!=now || (int)roundf(old*50000.f)!=ford_a3_round(now*50000.f)) differences++;
   }
  }
 }
 return differences;
}
'''
  source=WRAPPER.replace('AUDIT_RESET','').replace('AUDIT_LATCH','0').replace('AUDIT_LAST','curvature_state.desired_last').replace('AUDIT_MEAS','curvature_state.meas')
  source=source.replace('AUDIT_CANDIDATE_INCLUDE',exports)
  # Host test wrapper only; no firmware macro or production selector can use it.
  dispatch='' if production else '''if (tx && address==FORD_LateralMotionControl2) {
    bool admitted=ford_a3_check(&p); if (admitted) ford_a3_commit(); return admitted;
  }'''
  source=source.replace('AUDIT_CANDIDATE_DISPATCH',dispatch)
  (root/'wrapper.c').write_text(source)
  command=['cc','-shared','-fPIC','-std=gnu11','-O0','-DALLOW_DEBUG','-I',str(root),str(root/'wrapper.c'),'-o',str(root/'shared.so')]
  subprocess.run(command,check=True)
  lib=ctypes.CDLL(command[-1])
  lib.audit_packet.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_ubyte),ctypes.c_int]
  return lib,dict(checks=checks,compile_command=command,library_sha256=hashlib.sha256(Path(command[-1]).read_bytes()).hexdigest())


def audit(repo,output):
  output.mkdir(parents=True,exist_ok=True)
  cases=fixtures()
  report=dict(differences=[],cases={},builds={},positive_cases=0,rejected_cases=0,physical_limits_validated=False)
  variants={}
  for name in ('historical','shared'):
    pair={}
    for traced in (False,True):
      lib,meta=compiled(repo,output/name,'aggregate',traced) if name=='historical' else build(repo,output,traced)
      pair[traced]={n:execute(lib,meta['checks'],c) for n,c in cases.items()}
      report['builds'][f'{name}-{traced}']=meta
    for n in cases:
      for a,b in zip(pair[False][n],pair[True][n],strict=True):
        assert {k:v for k,v in a.items() if k!='failed_checks'}=={k:v for k,v in b.items() if k!='failed_checks'},(name,n)
    variants[name]=pair[True]
  for n in cases:
    report['cases'][n]={v:variants[v][n] for v in variants}
    for i,(a,b) in enumerate(zip(variants['historical'][n],variants['shared'][n],strict=True)):
      if any(a[k]!=b[k] for k in ('accepted','before','after','reason_bits')):
        report['differences'].append(dict(case=n,event=i,historical=a,shared=b))
      if b['event']['tx']:
        report['positive_cases']+=int(b['accepted'])
        report['rejected_cases']+=int(not b['accepted'])
  lib,meta=build(repo,output,production=True)
  from tools.navigator_a3.safety_audit import lmc
  lib.audit_init()
  def packet(data):
    wire=(ctypes.c_ubyte*8).from_buffer_copy(bytes(data))
    return bool(lib.audit_packet(982,0,wire,1))
  for e in cases['jitter-p0-s1']['sequence']:
    if e['tx']: break
    lib.audit_timer(e['time_us'])
    wire=(ctypes.c_ubyte*8).from_buffer_copy(bytes.fromhex(e['data_hex']))
    lib.audit_packet(e['address'],e['bus'],wire,0)
  lib.audit_timer(100000)
  a2_before=packet(lmc(curvature=2))
  def history():
    return [lib.audit_last(),lib.audit_power(),*[lib.audit_bucket(i) for i in range(3)],lib.audit_meas_min(),lib.audit_meas_max()]
  before=history()
  lib.audit_probe_prepare()
  accepted=packet(lmc(angle=2))
  after=history()
  probe=dict(shared_called=lib.audit_reasons()!=0xffffffff, shared_checks_passed=lib.audit_reasons()==0,
    blocked=not accepted,angle_reference_unchanged=lib.audit_candidate_state(0)==0 and not lib.audit_pending(),
    a2_history_before=before,a2_history_after=after,a2_history_preserved=before==after,
    a2_before_accepted=a2_before)
  lib.audit_timer(150000)
  probe['a2_after_accepted']=packet(lmc(curvature=2))
  report['production_probe']=probe
  report['math_parity_differences']=lib.audit_math_parity()
  report['builds']['production']=meta
  report['source_sha256']={str(p.relative_to(repo)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
    (repo/'opendbc/safety/modes/ford.h',repo/'opendbc/safety/modes/ford_angle.h',repo/'opendbc/safety/safety.h')}
  (output/'firmware-contract.json').write_text(json.dumps(report,indent=2)+'\n')
  return report

if __name__=='__main__':
  p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
  a=p.parse_args();r=audit(a.repo,a.output)
  print(json.dumps({k:r[k] for k in ('positive_cases','rejected_cases','production_probe')}))
  assert not r['differences']
