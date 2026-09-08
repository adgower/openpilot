"""Host-only real card/controller -> virtual publication -> compiled audit.

pytest's scoped MonkeyPatch and the existing in-memory card fixture replace
hardware initialization and IPC. No modified production permission is exercised.
"""
import argparse
from copy import deepcopy
import json
import hashlib
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from opendbc.car.ford.navigator_a3 import PROFILES
from opendbc.car.ford.tests.test_navigator_a3_runtime import sample
from openpilot.selfdrive.car.tests.test_navigator_a3_card import harness, frames
from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeBridge
from tools.navigator_a3.command_transport import SimulatedTransport
from tools.navigator_a3.safety_audit import inputs, event, lmc
from tools.navigator_a3.aggregate_timing_contract import compiled, execute
from tools.navigator_a3.angle_handoff_contract import build_production, execute as production_execute


def run_case(name, profile, sign):
  with pytest.MonkeyPatch.context() as mp:
    instance, command, _ = harness(mp, 'shadow')
    mp.setenv('NAVIGATOR_A3_PROFILE', list(PROFILES)[profile])
    # Recreate the actual controller at the selected startup profile. The real
    # card methods remain responsible for apply, serialization and publication.
    from opendbc.car.ford.carcontroller import CarController
    from opendbc.car import Bus
    instance.CI.CC = CarController({Bus.pt:'ford_lincoln_base_pt'},instance.CP)
    instance.navigator_a3_bridge = RuntimeBridge('shadow',[('ford',2,0)],'card-fixture','synthetic')
    instance.navigator_a3_bridge.controls_ready = True
    baseline, _, _ = harness(mp, 'a2')
    baseline.CI.CC = CarController({Bus.pt:'ford_lincoln_base_pt'},baseline.CP)
    t = SimulatedTransport(name)
    h = NS(safetyTxBlocked=0, controlsAllowed=True, safetyRxChecksInvalid=False,
           safetyModel='ford', safetyParam=2, alternativeExperience=0)
    t.health(1_000_000_001,h)
    instance.navigator_a3_bridge.panda(1_000_000_001,True,0,h)
    _, cs = sample(25)
    cs.out.canValid = True
    cs.out.vEgoRaw = cs.out.vEgo = 25.
    cs.out.yawRate = 0.
    command = command.as_builder()
    command.actuators.curvature = sign * .001
    for c in (instance,baseline):
      c.CI.apply = lambda control,now,c=c: c.CI.CC.update(control,cs,now)
    calls=[]
    rx=[e for e in inputs(25.)['latch-expiry-controls-off'] if not e['tx']]
    sequence=[dict(e,controls_fixture=None) for e in rx[:18]]
    sequence.append(event([0,4,0,0,0,0,0,0],address=0x165,tx=False,t=90000))
    emission=0
    last_frame=None
    for step in range(60):
      us=100000+step*10000-(1000 if step%10==5 else 0)
      if name=='gap' and step>=17:
        us+=110000
      now=1_000_000_000+us*1000
      if step==17 and name=='rejected':
        t.feedback('rejected',now-1,last_frame)
      if step==17 and name in ('permission','configuration'):
        t.health(now-1,NS(**dict(vars(h),**({'controlsAllowed':False} if name=='permission' else {'safetyParam':3}))))
      if step==25 and name in ('rejected','permission','configuration'):
        t.health(now-1,h)
        if last_frame:
          t.feedback('returned',now,last_frame)
      cs.out.steeringPressed=name=='driver' and 17<=step<=21
      mp.setattr('openpilot.selfdrive.car.card.time.monotonic',lambda now=now:now/1e9)
      # Explicit synthetic evidence input. This does not clear or change any
      # production fault; this whole card instance is an in-memory fixture.
      instance.navigator_a3_bridge.calculation_fault_reason=t.calculation_fault_reason
      for c in (instance,baseline):
        c.sm.logMonoTime['carControl']=now
        for values in c.CI.can_parsers['pt'].ts_nanos.values():
          for key in values:
            values[key]=now-100_000_001 if name=='measurement' and 17<=step<=21 else now
        c.controls_update(cs.out,command.as_reader())
        c.state_publish(cs.out,None)
      d=json.loads(instance.pm.read('carOutput').carOutput.navigatorA3.diagnosticsJson)['controller']
      selection=instance.CI.CC.navigator_a3.selection
      pub=t.publish(now,selection)
      serialized=d['command_selection']['experimental_frame']
      calls.append({'step':step,'now_ns':now,'diagnostic':d,'publication':pub,'fault':t.fault_reason,
                    'actual_can_equal_a2':frames(instance)==frames(baseline),
                    'serialized_selection_matches':serialized==d['proposed_frame']})
      if pub:
        frame=selection.experimental_frame
        previous_wire=bytes(lmc()) if last_frame is None else last_frame[1]
        last_frame=frame
        sequence.extend(dict(e,time_us=us,controls_fixture=None) for e in rx[18+emission*3:21+emission*3])
        sequence.append(dict(event(frame[1],address=frame[0],t=us),bus=frame[2],
                             proposal={'diagnostic':d,'provenance':'synthetic', 'after':d['output']['state'],
                                       'previous_proposal_wire_hex':previous_wire.hex()}))
        emission+=1
    return {'profile':profile,'sign':sign,'sequence':sequence,'calls':calls}


def audit(repo,output):
  output.mkdir(parents=True,exist_ok=True)
  cases={f'normal-p{p}-s{s}':run_case(f'normal-p{p}-s{s}',p,s) for p in range(4) for s in (-1,1)}
  cases.update({name:run_case(name,0,1) for name in ('driver','measurement','gap','rejected','permission','configuration')})
  for name in ('wire-excessive','compressed'):
    case=deepcopy(cases['normal-p0-s1'])
    tx=[e for e in case['sequence'] if e['tx']]
    if name=='wire-excessive':
      tx[1]['data_hex']=bytes(lmc(angle=700)).hex()
      tx[1]['injected_transport_fault']='excessive bytes; no feedback to host'
    else:
      tx[0]['time_us']+=40000
      tx[0]['injected_transport_fault']='40 ms delay; no feedback to host'
      case['sequence'].sort(key=lambda e:e['time_us'])
    cases[name]=case
  result={'version':1,'physical_response':'unknown','instrumentation_equivalent':True,'cases':cases,'builds':{},
          'timing':'Synthetic host timestamps and separately assumed compiled arrival times',
          'boundary':'Real card methods with in-memory publisher and synthetic inputs; not a device/IPC/EPS test.'}
  parent=Path(__file__).resolve().parents[2]
  paths=[Path(__file__),Path(__file__).with_name('command_transport.py'),
    parent/'openpilot/selfdrive/car/tests/test_navigator_a3_card.py',
    parent/'openpilot/selfdrive/car/card.py',parent/'openpilot/selfdrive/car/navigator_a3_runtime.py']
  paths += [repo/'opendbc/car/ford'/name for name in ('carcontroller.py','navigator_a3_command.py',
    'navigator_a3_runtime.py','navigator_a3_scheduler.py','navigator_a3.py','fordcan.py')]
  result['host_source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
  for variant in ('aggregate','production'):
    pair={}
    for traced in (False,True):
      if variant=='aggregate':
        lib,meta=compiled(repo,output/'compiled',variant,traced)
        pair[traced]={n:execute(lib,meta['checks'],c) for n,c in cases.items()}
      else:
        lib,meta=build_production(repo,output/'compiled',traced)
        pair[traced]={n:production_execute(lib,meta['checks'],c,[],variant) for n,c in cases.items()}
      result['builds'][f'{variant}-{traced}']=meta
    for name in cases:
      for a,b in zip(pair[False][name],pair[True][name],strict=True):
        assert {k:v for k,v in a.items() if k!='failed_checks'}=={k:v for k,v in b.items() if k!='failed_checks'}
      cases[name][variant]=pair[True][name]
  result['summary']={n:{v:{'accepted':sum(r['accepted'] for r in c[v] if r['event']['tx']),
      'rejected':sum(not r['accepted'] for r in c[v] if r['event']['tx'])} for v in ('aggregate','production')} for n,c in cases.items()}
  (output/'command-path.json').write_text(json.dumps(result,indent=2)+'\n')
  return result


if __name__=='__main__':
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--stock-repo',type=Path,required=True)
  parser.add_argument('--output',type=Path,required=True)
  args=parser.parse_args()
  print(json.dumps(audit(args.stock_repo,args.output)['summary'],indent=2))
