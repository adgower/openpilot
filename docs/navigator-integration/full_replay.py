"""Full-rate recorded RX + controller-generated TX replay. Never opens vehicle interfaces."""
import os,sys,json,hashlib,zstandard as z
from pathlib import Path
from types import SimpleNamespace
from collections import Counter
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'opendbc_repo'))
from openpilot.cereal import log
from opendbc.car import structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR,DBC,FordFlags
from opendbc.car.ford.pinion import configure_pinion_curvature
from opendbc.car.ford.carcontroller import CarController
from opendbc.can.parser import CANParser
from opendbc.safety.tests.libsafety import libsafety_py as api
OUT=Path(__file__).resolve().parent
MODE=sys.argv[1]
assert MODE in ('default-off', 'pinion', 'low-speed-prototype'), MODE
api.load(OUT/'safety-debug-macos.so');safety=api.libsafety
rxcheck={0x165,0x204,0x213,0x415,0x202,0x91,0x7e}
roots=[('earlier','000000d5--aece684f24'),('latest-1','000000d6--9443699ad9'),('latest-2','000000d7--0b88479abb')]
results=[]
for label,prefix in roots:
 cp=CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
 configure_pinion_curvature(cp,MODE in ('pinion','low-speed-prototype'))
 if MODE=='low-speed-prototype': cp.flags |= int(FordFlags.LOW_SPEED_GAIN)
 controller=CarController(DBC[CAR.FORD_EXPEDITION_MK4],cp)
 from opendbc.car.ford.lateral_angle import LateralAngle
 baseline=CarController(DBC[CAR.FORD_EXPEDITION_MK4],cp)
 baseline.lateral_angle=LateralAngle(cp)
 comparison_rows=[];high_comparisons=0;high_differences=0;changed_frames=0
 parser0=CANParser('ford_lincoln_base_pt',[('Steering_Data_FD1',0)],0)
 parser2=CANParser('ford_lincoln_base_pt',[('ACCDATA_3',0),('IPMA_Data',0)],2)
 state=None;configured=False;actualparam=None;tick=None;counts=Counter();blocks=[];rxinvalid=Counter();txdigest=hashlib.sha256();recordeddigest=hashlib.sha256();last_t=0;first_t=None;reportedmax=0
 def tx(t,frames,active=False,origin='generated'):
  for addr,dat,bus in frames:
   txdigest.update(int(addr).to_bytes(4,'little')+int(bus).to_bytes(1,'little')+bytes(dat))
   counts['generated_frames' if origin=='generated' else 'recorded_tx_frames']+=1
   if not configured:counts['tx_before_ford_config']+=1;continue
   accepted=bool(safety.safety_tx_hook(api.make_CANPacket(addr,bus,dat)))
   counts['tx_accepted' if accepted else 'tx_rejected']+=1
   if addr==0x3d6 and ((dat[0]>>4)&7):counts['active_angle_frames']+=1
   if not accepted:
    counts['reject_'+hex(addr)]+=1
    if len(blocks)<100:
     blocks.append({'time_s':round((t-first_t)/1e9,6),'addr':hex(addr),'mode':(dat[0]>>4)&7 if addr==0x3d6 else None,'controls_allowed':bool(safety.get_controls_allowed()),'speed_mph':round(state.vEgoRaw/.44704,3) if state else None,'pressed':bool(state.steeringPressed) if state else None,'lat_active':active,'rx_valid':bool(safety.safety_config_valid())})
 files=sorted(Path(os.environ['NAVIGATOR_RECORDINGS']).glob(prefix+'--*/rlog.zst'),key=lambda p:int(p.parent.name.rsplit('--',1)[1]))
 assert len(files)=={'earlier':33,'latest-1':5,'latest-2':7}[label]
 for path in files:
  seg=int(path.parent.name.rsplit('--',1)[1]);print(MODE,label,'segment',seg,flush=True)
  with path.open('rb') as f:data=z.ZstdDecompressor().stream_reader(f).read()
  events=sorted(log.Event.read_multiple_bytes(data),key=lambda e:e.logMonoTime)
  for event in events:
   t=event.logMonoTime;kind=event.which()
   if first_t is None:first_t=t
   last_t=t
   safety.set_timer(int((t-first_t)/1000)&0xffffffff)
   if configured:
    now=int((t-first_t)/1e9)
    if tick is None or now>tick:
     safety.safety_tick();tick=now;counts['invalid_safety_ticks']+=int(not safety.safety_config_valid())
   if kind=='pandaStates':
    states=list(event.pandaStates);ford=[x for x in states if str(x.safetyModel)=='ford']
    reportedmax=max([reportedmax]+[int(x.safetyTxBlocked) for x in states])
    if ford:
     param=int(ford[0].safetyParam)
     if not configured or actualparam!=param:
      replayparam=param|8 if MODE in ('pinion','low-speed-prototype') else param
      assert safety.set_safety_hooks(6,replayparam)==0
      configured=True;actualparam=param;tick=None;counts['ford_initializations']+=1
    else:configured=False;actualparam=None
   elif kind=='can':
    frames=[(int(c.address),bytes(c.dat),int(c.src)) for c in event.can if c.src<3]
    parser0.update([(t,frames)]);parser2.update([(t,frames)])
    if configured:
     for addr,dat,bus in frames:
      valid=bool(safety.safety_rx_hook(api.make_CANPacket(addr,bus,dat)));counts['rx_frames']+=1
      if not valid and addr in rxcheck and bus==0:rxinvalid[hex(addr)]+=1
   elif kind=='carState':state=event.carState.as_builder()
   elif kind=='carControl' and MODE!='recorded' and state is not None:
    cs=SimpleNamespace(out=state,buttons_stock_values=parser0.vl['Steering_Data_FD1'],
                       acc_tja_status_stock_values=parser2.vl['ACCDATA_3'],lkas_status_stock_values=parser2.vl['IPMA_Data'])
    cc=event.carControl
    _,sends=controller.update(cc,cs,t)
    _,before=baseline.update(cc,cs,t)
    if MODE=='low-speed-prototype':
     if state.vEgoRaw >= 45*.44704:
      high_comparisons+=len(sends)
      high_differences+=int(sends!=before)
     changed_frames+=int(sends!=before)
     angle=[bytes(dat) for addr,dat,bus in sends if addr==0x3d6]
     oldangle=[bytes(dat) for addr,dat,bus in before if addr==0x3d6]
     if angle and oldangle:
      decode=lambda dat: -((((dat[3]&31)<<6)|(dat[4]>>2))*.0005-.5)
      comparison_rows.append([(t-first_t)/1e9,state.vEgoRaw,int(cc.latActive),int(state.steeringPressed),cc.actuators.curvature,decode(oldangle[0]),decode(angle[0])])
    counts['controller_updates']+=1
    tx(t,sends,bool(cc.latActive))
   elif kind=='sendcan':
    frames=[(int(c.address),bytes(c.dat),int(c.src)) for c in event.sendcan if c.src<3]
    for addr,dat,bus in frames:recordeddigest.update(addr.to_bytes(4,'little')+bus.to_bytes(1,'little')+dat)
    counts['saved_tx_frames']+=len(frames)
    if MODE=='recorded':tx(t,frames,origin='recorded')
 result={'drive':label,'mode':MODE,'segments':len(files),'duration_s':round((last_t-first_t)/1e9,3),'counts':dict(counts),'rx_invalid':dict(rxinvalid),'reported_blocked_counter_max':reportedmax,'tx_sha256':txdigest.hexdigest(),'saved_tx_sha256':recordeddigest.hexdigest(),'first_rejected_frames':blocks}
 if MODE=='low-speed-prototype':
  result['comparison']={'high_speed_generated_messages':high_comparisons,'high_speed_updates_different':high_differences,'updates_different':changed_frames}
  (OUT/(label+'-command-comparison.json')).write_text(json.dumps({'columns':['t','speed','active','pressed','request','pinion_angle','prototype_angle'],'rows':comparison_rows},separators=(',',':')))
 results.append(result)
 (OUT/(MODE+'-replay.json')).write_text(json.dumps(results,indent=2)+'\n')
 print('RESULT',label,json.dumps({k:v for k,v in result.items() if k not in ['first_rejected_frames']}),flush=True)
