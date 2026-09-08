"""Recorded native-time warning/transport audit; no controller or safety feedback."""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
from tools.navigator_a3.transport_extract import normalize_event
from tools.navigator_a3.transport_observer import Observer, resolve_candidates

def decode_lmc2(data):
  """Pinned Ford DBC 982: wire sign, not host-sign curvature."""
  if len(data)!=8: raise ValueError('LateralMotionControl2 must be eight bytes')
  return {'mode':(data[0]>>4)&7,'curvature_inv_m':((data[2]<<3)|(data[3]>>5))*2e-5-.02,
          'path_angle_rad':(((data[3]&31)<<6)|(data[4]>>2))*.0005-.5,
          'offset_m':(((data[4]&3)<<8)|data[5])*.01-5.12,
          'counter':(data[7]>>1)&15}

def prior_sample(rows,t_ns,max_age_ns):
  idx=bisect_right(rows,t_ns,key=lambda r:r['t_ns'])-1
  if idx<0 or t_ns-rows[idx]['t_ns']>max_age_ns: return None
  return rows[idx]

def merge_windows(windows):
  out=[]
  for a,b in sorted(windows):
    if out and a<=out[-1][1]:out[-1]=(out[-1][0],max(b,out[-1][1]))
    else:out.append((a,b))
  return out

FIELDS={'carState':['vEgo','vEgoRaw','yawRate','steeringAngleDeg','steeringTorque','steeringPressed','brakePressed','gasPressed','steerFaultTemporary','steerFaultPermanent','canValid','standstill'],
        'carControl':['enabled','latActive','actuators.curvature'],
        'carOutput':['actuatorsOutput.curvature','actuatorsOutput.steeringAngleDeg'],
        'modelV2':['action.desiredCurvature','big'],
        'controlsState':['curvature'],
        'selfdriveState':['alertType','alertText1','alertText2','enabled','active']}

def load_schema():
  import capnp
  base=Path(__file__).resolve().parents[2]/'openpilot/cereal'
  return capnp.load(str(base/'log.capnp'),imports=[str(base),'/private/tmp/navigator-a3-opendbc/opendbc/car'])

def run(root,output):
  import zstandard
  log=load_schema();output.mkdir(parents=True,exist_ok=True)
  old=[json.loads(s) for s in (root/'analysis.jsonl').read_text().splitlines()]
  anchors=[t for d in old for t,k,v in d['transitions'] if (k=='rejected' and v['address']==982 and decode_lmc2(bytes.fromhex(v['data']))['mode']!=0) or (k=='alert' and ('steerSaturated' in v or 'steerTempUnavailable' in v))]
  windows=merge_windows([(t-30_000_000_000,t+15_000_000_000) for t in anchors])
  def selected(t):return any(a<=t<=b for a,b in windows)
  obs=Observer();order=0;rejections=[];signals=defaultdict(list);manifest=[]
  with (output/'signals.jsonl').open('w') as sf,(output/'transport.jsonl').open('w') as tf:
    for path in sorted(root.glob('00000003--371271599e--*/rlog.zst'),key=lambda p:int(p.parent.name.rsplit('--',1)[1])):
      seg=int(path.parent.name.rsplit('--',1)[1]);blob=path.read_bytes();raw=zstandard.ZstdDecompressor().stream_reader(blob).read()
      manifest.append({'segment':seg,'sha256':hashlib.sha256(blob).hexdigest()})
      for idx,m in enumerate(log.Event.read_multiple_bytes(raw)):
        t=int(m.logMonoTime);kind=m.which()
        if selected(t) and kind in FIELDS:
          d=getattr(m,kind);r={'t_ns':t,'kind':kind,'valid':bool(m.valid),'segment':seg,'source_index':idx}
          for field in FIELDS[kind]:
            v=d
            for part in field.split('.'):v=getattr(v,part)
            r[field]=v if isinstance(v,(int,float,bool,str)) else str(v)
          if kind=='carOutput':
            j=json.loads(d.navigatorA3.diagnosticsJson);r['diagnostic']=j
          signals[kind].append(r);sf.write(json.dumps(r)+'\n')
        for e in normalize_event(m,'00000003--371271599e',str(path),idx):
          if e['kind']!='health' and e['address']!=982 and e['kind']!='rejected':continue
          e['order']=order;order+=1;r=obs.consume(e)
          if e.get('address')==982:r['decoded']=decode_lmc2(bytes.fromhex(e['data_hex']))
          if r['direct_steering_rejection']:rejections.append(r)
          if selected(t):tf.write(json.dumps(r)+'\n')
          if selected(t) and e['kind']=='health': signals['pandaStates'].append(r)
          if selected(t) and e['kind']=='published' and e.get('address')==982:signals['published'].append(r)
      print('timeline segment',seg,flush=True)
  for r in rejections:r['all_plausible_publications']=resolve_candidates(obs.groups,r)
  (output/'rejections.json').write_text(json.dumps(rejections,indent=2))
  (output/'publication-groups.json').write_text(json.dumps(obs.groups))
  for rows in signals.values():rows.sort(key=lambda r:r['t_ns'])
  joined=[]
  # As-of join uses past timestamps only; age/validity are retained, never interpolated.
  for row in signals['carControl']:
    t=row['t_ns'];r={'t_ns':t,'carControl':row}
    for kind,age in [('carState',100_000_000),('modelV2',150_000_000),('carOutput',100_000_000),('controlsState',100_000_000),('pandaStates',250_000_000),('published',100_000_000),('selfdriveState',100_000_000)]:
      sample=prior_sample(signals[kind],t,age);r[kind]=sample;r[kind+'_age_ns']=t-sample['t_ns'] if sample else None
    joined.append(r)
  with (output/'aligned.jsonl').open('w') as f:
    for r in joined:f.write(json.dumps(r)+'\n')
  summary={'windows_ns':windows,'raw_inputs':manifest,'rejected_steering_frames':len(rejections),'attribution_counts':dict(Counter(r['attribution'] for r in rejections)),
           'scope':'Recorded A2 commands; no A3 acknowledgment or safety oracle input. Past-only timestamp joins retain validity and explicit age.',
           'observer_counts':dict(obs.counts),'joined_rows':len(joined)}
  (output/'summary.json').write_text(json.dumps(summary,indent=2));return summary

if __name__=='__main__':
  p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(json.dumps(run(a.root,a.output)))
