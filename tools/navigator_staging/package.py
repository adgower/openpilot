"""Create self-contained pinned Git bundles without pushing or changing source repos."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def git(repo,*args):return subprocess.check_output(['git','-C',str(repo),*args],text=True).strip()

def digest(p):
  with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def package(parent, sources, output):
  output.mkdir(parents=True,exist_ok=False)
  pins={'.':git(parent,'rev-parse','HEAD')}
  for line in git(parent,'ls-tree','HEAD').splitlines():
    mode,kind,rest=line.split(' ',2)
    if mode=='160000':sha,path=rest.split('\t');pins[path]=sha
  repos=[]
  (output/'lfs').mkdir()
  for path,sha in pins.items():
    src=parent if path=='.' else Path(sources[path])
    git(src,'cat-file','-e',sha+'^{commit}')
    name='parent' if path=='.' else path
    bundle=output/(name+'.bundle')
    # A temporary ref advertises the exact pin and includes its complete ancestry.
    with tempfile.TemporaryDirectory(prefix='navigator-bundle-') as temp:
      bare=Path(temp)/'repo.git'
      subprocess.run(['git','clone','--bare','--shared','--quiet',str(src),str(bare)],check=True)
      git(bare,'update-ref','refs/heads/package',sha)
      git(bare,'bundle','create',str(bundle),'refs/heads/package')
      check=Path(temp)/'verify.git';subprocess.run(['git','init','--bare','-q',str(check)],check=True)
      subprocess.run(['git','-C',str(check),'bundle','verify',str(bundle)],check=True)
    lfs = json.loads(git(src,'lfs','ls-files','--long','--json',sha))['files'] or []
    payloads=[]
    for entry in lfs:
      source=src/entry['name']; oid=entry['oid']
      if source.stat().st_size != entry['size'] or digest(source)!=oid:
        raise ValueError(f'Missing or changed frozen LFS payload: {source}')
      # Also verify the pointer belongs to this exact commit, not another checkout.
      pointer=git(src,'show',sha+':'+entry['name'])
      if f'oid sha256:{oid}' not in pointer: raise ValueError('LFS pointer mismatch')
      dest=output/'lfs'/oid
      if not dest.exists(): shutil.copyfile(source,dest)
      payloads.append({'name':entry['name'],'oid':oid,'size':entry['size']})
    repos.append({'lfs':payloads,'path':path,'sha':sha,'bundle':bundle.name,'sha256':digest(bundle),'bytes':bundle.stat().st_size})
  models={}
  for name in git(parent,'ls-files','openpilot/selfdrive/modeld/models/*.onnx').splitlines():
    p=parent/name
    with p.open('rb') as f: prefix=f.read(45)
    if prefix.startswith(b'version https://git-lfs'):raise ValueError('Unmaterialized model LFS pointer')
    models[name]={'sha256':digest(p),'bytes':p.stat().st_size}
  config={'agnos':'19.7','mode':'shadow','profile':'expedition-provisional-v1','wheelbase_m':3.1115,
          'mass':'retain A2 baseline; no new mass override','fingerprint':'FORD_EXPEDITION_MK4',
          'base_parent':'18fd1a6505e072710514e723dae2edc65c0e3355','child':'01bea343a7f7d4e4f8a1947539abacb59ab8c278'}
  manifest={'schema_version':1,'repos':repos,'models':models,'configuration':config,'status':'source package; target build pending'}
  (output/'package.json').write_text(json.dumps(manifest,indent=2)+'\n')
  for name in ('prepare.py','build_only.sh','verify_checkout.py','transfer.sh','README.md','inventory_readonly.sh'):
    shutil.copy2(parent/'tools/navigator_staging'/name,output/name)
  (output/'SHA256SUMS').write_text(''.join(f'{digest(p)}  {p.relative_to(output)}\n' for p in sorted(output.rglob('*')) if p.is_file()))
  print(json.dumps(manifest,indent=2))

if __name__=='__main__':
  p=argparse.ArgumentParser();p.add_argument('--parent',type=Path,required=True);p.add_argument('--sources',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
  a=p.parse_args();package(a.parent,json.loads(a.sources.read_text()),a.output.resolve())
