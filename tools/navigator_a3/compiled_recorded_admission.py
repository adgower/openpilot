"""Bounded stock-header host-order replay; NEVER attributes actual firmware failures.

RX and published TX are consumed in file/message/packet order, without sorting.
The 1 Hz safety tick is phase-parametrized. Actual USB heartbeat and safety mode
transitions are unavailable, so this is explicitly an incomplete runtime model.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict, deque
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
from tools.navigator_a3.safety_audit import extract, instrument
from tools.navigator_a3.rejection_diagnostics import classify_src

REV = '01bea343a7f7d4e4f8a1947539abacb59ab8c278'
C = r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
static uint32_t timer_cnt;
uint32_t microsecond_timer_get(void) { return timer_cnt; }
static int failed[512], nfailed;
static bool audit_check(int id, bool v) { if(v && nfailed<512) failed[nfailed++]=id; return v; }
#include "opendbc/safety/can.h"
#include "opendbc/safety/safety.h"
void init(int param, int alternative) { set_safety_hooks(SAFETY_FORD,param); alternative_experience=alternative; }
void timer(uint32_t t) { timer_cnt=t; }
void tick(void) { safety_mode_cnt++; safety_tick(&current_safety_config); }
int packet(int addr,int bus,unsigned char *data,int length,int tx) {
  CANPacket_t p={0}; p.addr=addr; p.bus=bus;
  for(int i=0;i<16;i++) if(dlc_to_len[i]==length) { p.data_len_code=i; break; }
  memcpy(p.data,data,length); nfailed=0;
  return tx?safety_tx_hook(&p):safety_rx_hook(&p);
}
int failures(int i) { return i<nfailed?failed[i]:-1; }
int state(int i) {
  int v[]={controls_allowed,relay_malfunction,curvature_state.desired_last,
    curvature_state.meas.min,curvature_state.meas.max,vehicle_speed.min,vehicle_speed.max,
    curvature_state.rt_msgs,curvature_state.rt_msgs_prev,curvature_state.ts_check_last,
    safety_mode_cnt,safety_rx_checks_invalid,cruise_engaged_prev,brake_pressed,brake_pressed_prev,
    gas_pressed,gas_pressed_prev,vehicle_moving,curvature_state.steer_power_last};
  return v[i];
}
int *states(void) { static int v[19]; for(int i=0;i<19;i++) v[i]=state(i); return v; }
'''
FIELDS = ['controls_allowed','relay_malfunction','desired_last','measured_min','measured_max',
          'speed_min','speed_max','rt_msgs','rt_msgs_prev','ts_check_last','safety_mode_cnt',
          'rx_checks_invalid','cruise_engaged_prev','brake_pressed','brake_pressed_prev',
          'gas_pressed','gas_pressed_prev','vehicle_moving','steer_power_last']


def build(repo, output, traced):
  root=output/('traced' if traced else 'plain'); root.mkdir(parents=True,exist_ok=True)
  extract(repo,REV,'',root)
  hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*.h')}
  checks=instrument(root) if traced else {}
  source=C
  if traced:
    # Preserve short circuit and execution count of the final admission guard.
    p=root/'opendbc/safety/safety.h'; text=p.read_text()
    terms=[]
    for expr in ['!relay_malfunction','whitelisted','safety_allowed']:
      key=len(checks)+1; checks[key]={'file':'opendbc/safety/safety.h','expression':'!('+expr+')','kind':'final admission guard'}
      terms.append(f'!audit_check({key}, !({expr}))')
    text=text.replace('return !relay_malfunction && whitelisted && safety_allowed;', 'return '+' && '.join(terms)+';')
    p.write_text(text)
  (root/'wrapper.c').write_text(source)
  cmd=['cc','-shared','-fPIC','-std=gnu11','-O0','-DALLOW_DEBUG','-I',str(root),str(root/'wrapper.c'),'-o',str(root/'audit.so')]
  subprocess.run(cmd,check=True)
  lib=ctypes.CDLL(str(root/'audit.so'))
  lib.packet.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_ubyte),ctypes.c_int,ctypes.c_int]
  lib.states.restype=ctypes.POINTER(ctypes.c_int*len(FIELDS))
  return lib,{'revision':REV,'source_sha256':hashes,'checks':checks,'compile_command':cmd}


def snapshot(lib):
  return dict(zip(FIELDS,list(lib.states().contents),strict=True))


def agreement_eligible(observation_valid, matched):
  return bool(matched is not None and observation_valid and matched['provenance']['message_valid'])

def main():
  p=argparse.ArgumentParser(description=__doc__)
  p.add_argument('--opendbc',type=Path,required=True);p.add_argument('--rlog',type=Path,action='append',required=True)
  p.add_argument('--output',type=Path,required=True);p.add_argument('--param',type=int,required=True)
  p.add_argument('--alternative-experience',type=int,required=True)
  p.add_argument('--tick-phase-us',type=int,default=0)
  p.add_argument('--start-ns',type=int,default=0,help='Explicit truncated replay start; state before this boundary remains unknown')
  p.add_argument('--max-match-delay-ms',type=int,default=100,help='Bound for ambiguous published/returned byte matching')
  a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
  assert 0<=a.tick_phase_us<1000000
  plain,pm=build(a.opendbc,a.output/'compiled',False); traced,tm=build(a.opendbc,a.output/'compiled',True)
  assert pm['source_sha256']==tm['source_sha256']
  for lib in (plain,traced):lib.init(a.param,a.alternative_experience)
  from openpilot.tools.lib.logreader import _LogFileReader
  pending=defaultdict(deque);counts=Counter();first_difference=None;clock=None;next_tick=None;ordinal=0
  input_files=[]
  out=(a.output/'observations.jsonl').open('w')
  for path in a.rlog:
    file_hash=hashlib.sha256(path.read_bytes()).hexdigest()
    input_files.append({'path':str(path),'sha256':file_hash,'bytes':path.stat().st_size})
    for mi,msg in enumerate(_LogFileReader(str(path),only_union_types=True)):
      kind=msg.which()
      if kind not in ('can','sendcan'):continue
      t=int(msg.logMonoTime);us=t//1000
      if t<a.start_ns:continue
      if clock is None:next_tick=us-us%1000000+a.tick_phase_us;next_tick+=1000000 if next_tick<us else 0
      if clock is not None and us<clock:counts['host_timestamp_regressions']+=1
      while next_tick<=us:
        for lib in (plain,traced):lib.timer(next_tick%(2**32));lib.tick()
        next_tick+=1000000;counts['ticks']+=1
      clock=us
      for pi,packet in enumerate(getattr(msg,kind)):
        ordinal+=1;src=int(packet.src);addr=int(packet.address);data=bytes(packet.dat)
        provenance={'path':str(path),'sha256':file_hash,'message_index':mi,'packet_index':pi,'message_valid':bool(msg.valid)}
        if not msg.valid:counts['invalid_message_packets_retained_in_model']+=1
        flag=classify_src(src)
        if flag['kind']=='unknown':counts['unknown_src_packets']+=1;continue
        key=(flag['bus'],addr,data)
        if kind=='can' and flag['kind'] in ('returned','rejected'):
          rejected=flag['kind']=='rejected';counts['recorded_rejected' if rejected else 'recorded_returned']+=1
          # FIFO bytes match is evidence of compatibility, never a unique firmware sequence identity.
          while pending[key] and pending[key][0]['t_ns'] < t-a.max_match_delay_ms*1000000:
            pending[key].popleft();counts['expired_unmatched_publications']+=1
          matched=pending[key].popleft() if pending[key] and pending[key][0]['t_ns']<=t else None
          row={'provenance':provenance,'t_ns':t,'ordinal':ordinal,'src':src,'address':addr,'data_hex':data.hex(),'observed':'rejected' if rejected else 'returned',
               'published_match':matched,'actual_failed_checks':'unresolved','prior_observed_divergence':first_difference is not None,'match_method':'bounded FIFO identical bus/address/bytes; duplicates ambiguous'}
          eligible=agreement_eligible(bool(msg.valid),matched)
          row['agreement_eligible']=eligible
          if eligible:
            agrees=matched['accepted'] != rejected;row['model_agrees']=agrees
            if not agrees and first_difference is None:first_difference={'observation':{k:v for k,v in row.items() if k!='published_match'},'published_match':matched}
            counts['matched_observations']+=1;counts['model_disagreements']+=not agrees
          else:counts['unmatched_or_invalid_observations']+=1
          if rejected or (eligible and not row['model_agrees']):out.write(json.dumps(row)+'\n')
          continue
        if src not in (0,1,2) or len(data) not in (0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64):counts['ignored_packets']+=1;continue
        tx=kind=='sendcan';before=snapshot(traced) if tx else None;results=[]
        for lib in (plain,traced):
          lib.timer(us%(2**32));results.append(bool(lib.packet(addr,src,(ctypes.c_ubyte*len(data)).from_buffer_copy(data),len(data),int(tx))))
        assert results[0]==results[1] and bytes(plain.states().contents)==bytes(traced.states().contents), ('instrumentation divergence',path,mi,pi)
        counts['tx' if tx else 'rx']+=1
        if tx:
          failed=[];i=0
          while traced.failures(i)!=-1:failed.append(tm['checks'][traced.failures(i)]);i+=1
          pending[key].append({'provenance':provenance,'t_ns':t,'ordinal':ordinal,'accepted':results[1],'before':before,'after':snapshot(traced),'executed_failed_checks_in_model':failed})
  out.close()
  report={'scope':'Stock compiled safety host-order model, not faithful firmware runtime reconstruction',
          'actual_failed_checks':'unresolved','input_files':input_files,'counts':dict(counts),'first_observed_admission_difference':first_difference,
          'instrumentation_decision_and_selected_state_equivalent':True,'state_fields_compared':FIELDS,
          'parameters':vars(a)|{'opendbc':str(a.opendbc),'rlog':[str(x) for x in a.rlog],'output':str(a.output)},
          'build':{'plain':pm,'traced':tm},
          'limitations':['Host logMonoTime is not MCU timer; RX/sendcan order is not MCU execution order.',
                         'Invalid envelope packets retained as explicit raw-runtime assumption; invalid observations/publications excluded from agreement. No validity inferred from payload.',
                         'Packet order retained within supplied file/message/packet sequence; no sorting or health permission seeding.',
                         'Unknown initial state, mode transitions, USB heartbeat events and tick phase; mode starts Ford at first recorded packet.',
                         '1 Hz safety_tick and safety_mode_cnt modeled; main.c heartbeat timeout/mismatch permission clearing not reconstructed.',
                         'Matching identical bytes FIFO is ambiguous with duplicate commands and dropped or unlogged packets.',
                         'Instrumentation equivalence covers decisions and listed state fields, not full firmware or all internal state.',
                         'Any recorded rejection reason remains unresolved; model predicates describe only this replay.']}
  (a.output/'report.json').write_text(json.dumps(report,indent=2,default=str))
  print(json.dumps({'counts':dict(counts),'actual_failed_checks':'unresolved','output':str(a.output)}))
if __name__=='__main__':main()
