"""Two fresh stock replays bracketing one recorded brake/TX ordering ambiguity.

This deliberately counterfactual experiment is NOT actual firmware attribution.
Only the same logged brake RX preceding the rejected echo within its CAN batch is
moved before the TX. No controls/relay state is seeded or overridden.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
from tools.navigator_a3.compiled_recorded_admission import build,snapshot

START=268449297847
TX=511955516970
ECHO=511961025272
WIRE=bytes.fromhex('141b5c2fa200801e')


def main():
  p=argparse.ArgumentParser(description=__doc__);p.add_argument('--route',type=Path,required=True)
  p.add_argument('--opendbc',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
  a.output.mkdir(parents=True,exist_ok=True)
  from openpilot.tools.lib.logreader import _LogFileReader
  paths=[a.route/f'00000003--371271599e--{i}'/'rlog.zst' for i in range(5)]
  brake=None;echo=None
  for mi,m in enumerate(_LogFileReader(str(paths[4]),only_union_types=True)):
    if m.which()=='can' and int(m.logMonoTime)==ECHO:
      for pi,c in enumerate(m.can):
        if int(c.address)==0x3d6 and int(c.src)==0xc0 and bytes(c.dat)==WIRE:
          assert echo is None and m.valid
          echo={'file':str(paths[4]),'sha256':hashlib.sha256(paths[4].read_bytes()).hexdigest(),'message_index':mi,'packet_index':pi,
                't_ns':ECHO,'src':0xc0,'address':0x3d6,'data_hex':bytes(c.dat).hex(),'valid':bool(m.valid)}
        if int(c.address)==0x165 and int(c.src)==0:
          assert brake is None and m.valid
          brake={'file':str(paths[4]),'sha256':hashlib.sha256(paths[4].read_bytes()).hexdigest(),'message_index':mi,'packet_index':pi,
                 't_ns':ECHO,'src':0,'address':0x165,'data_hex':bytes(c.dat).hex(),'valid':bool(m.valid)}
  assert brake is not None and brake['packet_index']==12 and (bytes.fromhex(brake['data_hex'])[0]>>4)&3==2
  assert echo is not None and echo['packet_index']==72 and echo['message_index']==brake['message_index'] and echo['packet_index']>brake['packet_index']
  variants={};builds={}
  for scenario in ('host_order','same_batch_brake_before_tx'):
    libs=[build(a.opendbc,a.output/'compiled'/scenario,trace) for trace in (False,True)]
    builds[scenario]=[meta for _,meta in libs]
    for lib,_ in libs:lib.init(3,0)
    next_tick=269000000;done=False;counts={'rx':0,'tx':0}
    def apply(addr,src,data,tx,us):
      results=[]
      for lib,_ in libs:
        lib.timer(us%2**32);results.append(bool(lib.packet(addr,src,(ctypes.c_ubyte*len(data)).from_buffer_copy(data),len(data),tx)))
      assert results[0]==results[1] and bytes(libs[0][0].states().contents)==bytes(libs[1][0].states().contents)
      return results[1]
    for path in paths:
      for mi,m in enumerate(_LogFileReader(str(path),only_union_types=True)):
        if m.which() not in ('can','sendcan') or int(m.logMonoTime)<START:continue
        assert m.valid, ('Invalid input envelope makes this isolated bracket unsupported',path,mi)
        us=int(m.logMonoTime)//1000
        while next_tick<=us:
          for lib,_ in libs:lib.timer(next_tick%2**32);lib.tick()
          next_tick+=1000000
        for pi,c in enumerate(getattr(m,m.which())):
          addr=int(c.address);src=int(c.src);data=bytes(c.dat);tx=m.which()=='sendcan'
          if src not in (0,1,2):continue
          target=tx and int(m.logMonoTime)==TX and addr==0x3d6 and data==WIRE
          if target:
            before_brake=snapshot(libs[1][0])
            if scenario=='same_batch_brake_before_tx':apply(0x165,0,bytes.fromhex(brake['data_hex']),False,us)
            before=snapshot(libs[1][0]);accepted=apply(addr,src,data,True,us)
            lib,meta=libs[1];failed=[];i=0
            while lib.failures(i)!=-1:failed.append(meta['checks'][lib.failures(i)]);i+=1
            variants[scenario]={'target':{'file':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'message_index':mi,'packet_index':pi,'t_ns':TX,'data_hex':data.hex(),'valid':bool(m.valid)},
                                'before_brake_intervention':before_brake,'before_tx':before,'accepted':accepted,'after_tx':snapshot(lib),'executed_failed_checks':failed,'prefix_counts':counts}
            done=True;break
          apply(addr,src,data,tx,us);counts['tx' if tx else 'rx']+=1
        if done:break
      if done:break
    assert done
  result={'scope':'Explicit one-packet counterfactual ordering bracket; actual failed check unresolved',
          'brake_packet':brake,'rejected_echo_packet':echo,'variants':variants,'builds':builds,
          'input_files':[{'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()} for path in paths],
          'all_replayed_envelopes_valid':True,
          'limitations':['Mode begins first observed Ford sample; earlier state and precise mode transition unknown.',
                         'Host-order replay does not reproduce actual MCU packet/timer/USB heartbeat order.',
                         'Second scenario moves only actual same-batch brake RX before target TX, assigning target host TX timer; this is an assumed order/time, not firmware telemetry.',
                         'It does not claim all packets within the batch preceded TX, and it does not infer a cause for other rejections.'],
          'instrumentation_decisions_and_selected_state_equivalent':True,'actual_failed_check':'unresolved'}
  (a.output/'first-rejection-order.json').write_text(json.dumps(result,indent=2))
  print(json.dumps({k:{'accepted':v['accepted'],'checks':[c['expression'] for c in v['executed_failed_checks']]} for k,v in variants.items()}))
if __name__=='__main__':main()
