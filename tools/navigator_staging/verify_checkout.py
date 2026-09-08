"""Check pinned checkout and optional target build outputs; no hardware construction."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def sha(p):
  with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def verify(root):
  manifest=json.loads((root/'.staging-package.json').read_text())
  for item in manifest['repos']:
    path=root/item['path']
    actual=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
    if actual!=item['sha']:raise ValueError(f'Wrong pin: {path}')
    subprocess.run(['git','-C',str(path),'diff','--exit-code','HEAD','--'],check=True)
    for entry in item.get('lfs',[]):
      if sha(path/entry['name'])!=entry['oid']:raise ValueError(f'LFS payload mismatch: {entry["name"]}')
  for name,expected in manifest['models'].items():
    if sha(root/name)!=expected['sha256']:raise ValueError(f'Model hash mismatch: {name}')
  if 'export NAVIGATOR_A2_PROFILE="2023-navigator-swb-4wd"' not in (root/'launch_env.sh').read_text():raise ValueError('Missing A2 selector')
  if 'export FINGERPRINT="FORD_EXPEDITION_MK4"' not in (root/'launch_env.sh').read_text():raise ValueError('Missing fixed fingerprint')
  return manifest

if __name__=='__main__':
  p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--artifacts',action='store_true');p.add_argument('--match-result',action='store_true');a=p.parse_args();r=a.root.resolve();verify(r)
  result={'source_verified':True,'target_built':False}
  if a.artifacts:
    required=['openpilot/selfdrive/modeld/models/dmonitoring_model_metadata.pkl','openpilot/system/camerad/camerad','openpilot/system/loggerd/loggerd','openpilot/selfdrive/pandad/pandad','openpilot/cereal/libcereal.a','openpilot/cereal/gen/cpp/car.capnp.h','openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so','panda/board/obj/panda_h7.bin.signed','openpilot/selfdrive/modeld/models/dm_warp_1344x760_tinygrad.pkl','openpilot/selfdrive/modeld/models/dm_warp_1928x1208_tinygrad.pkl','openpilot/selfdrive/modeld/models/tg_input_devices.json']
    # Chunk manifests, not a stale unchunked pickle, are the runtime availability contract.
    from openpilot.common.file_chunker import get_manifest_path,get_existing_chunks
    for stem in ('big_driving_tinygrad.pkl','driving_tinygrad.pkl','dmonitoring_model_tinygrad.pkl'):
      base=r/'openpilot/selfdrive/modeld/models'/stem;mf=Path(get_manifest_path(str(base)))
      if not mf.is_file():raise ValueError(f'Required model manifest missing: {mf}')
      required.append(str(mf.relative_to(r)))
      chunks=get_existing_chunks(str(base))
      if sum(Path(part).stat().st_size for part in chunks if part != str(mf))==0:raise ValueError('Empty compiled model')
      for part in get_existing_chunks(str(base)):
        if not Path(part).is_file():raise ValueError(f'Missing model chunk: {part}')
        required.append(str(Path(part).relative_to(r)))
    # Import generated/native modules only; never instantiate Car, Panda or messaging sockets.
    import opendbc
    from opendbc.car import structs
    import msgq.ipc_pyx
    import openpilot.common.params as params_module
    from openpilot.common.params import Params
    if Path(opendbc.INCLUDE_PATH).resolve()!=(r/'opendbc_repo').resolve():raise ValueError('Wrong independent safety include path')
    for module in (msgq.ipc_pyx,params_module):
      path=Path(module.__file__).resolve()
      if r not in path.parents:raise ValueError(f'Native module outside staging checkout: {path}')
      required.append(str(path.relative_to(r)))
    structs.CarOutput.new_message().navigatorA3.mode='shadow'
    for name in required:
      if not (r/name).is_file() or ((r/name).stat().st_size==0 and '.chunk' not in name):raise ValueError(f'Missing build artifact: {name}')
    result.update(target_built=True,artifacts={name:sha(r/name) for name in sorted(set(required))},signing='repository debug certificate; not installed',safety_include=str(opendbc.INCLUDE_PATH))
  if a.match_result:
    previous=json.loads((r/'.staging-results/build-result.json').read_text())
    if previous.get('target_built') is not True or previous.get('artifacts')!=result.get('artifacts'):raise ValueError('Artifacts differ from successful build record')
  print(json.dumps(result,indent=2))
