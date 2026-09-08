"""Materialize verified source bundles only. Never boot, build, flash or update OS."""
import argparse
import hashlib
import json
import os
import re
import shutil
from pathlib import Path
import subprocess


def digest(path):
  with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def validate_destination(target: Path, active: Path):
  target=target.absolute()
  for forbidden in (active.resolve(), Path('/data/openpilot').resolve(), Path('/data/pythonpath').resolve(),
                    Path('/data/params'), Path('/data/safe_staging')):
    if target.resolve()==forbidden or forbidden in target.resolve().parents:
      raise ValueError(f'Refusing active/shared destination: {target}')
  if target.exists() or target.is_symlink():raise ValueError(f'Destination already exists: {target}')


def verify_package(package: Path):
  manifest=json.loads((package/'package.json').read_text())
  entries=manifest['repos']
  if not entries or entries[0]['path']!='.':raise ValueError('First repository must be parent')
  paths=set()
  for item in entries:
    p=Path(item['path']); bundle=Path(item['bundle'])
    if p.is_absolute() or '..' in p.parts or bundle.is_absolute() or len(bundle.parts)!=1:
      raise ValueError('Invalid repository or bundle path')
    if str(p) in paths:raise ValueError('Duplicate repository path')
    paths.add(str(p))
    if digest(package/bundle)!=item['sha256']:raise ValueError(f'Checksum mismatch: {bundle}')
    for entry in item.get('lfs',[]):
      name=Path(entry['name']);oid=entry['oid']
      if name.is_absolute() or '..' in name.parts or not re.fullmatch('[0-9a-f]{64}',oid):
        raise ValueError('Invalid LFS payload path')
      payload=package/'lfs'/oid
      if payload.stat().st_size!=entry['size'] or digest(payload)!=oid:
        raise ValueError(f'LFS checksum mismatch: {name}')
  return manifest


def materialize(package: Path, target: Path, active=Path('/data/openpilot')):
  package=package.resolve();validate_destination(target,active);manifest=verify_package(package)
  for item in manifest['repos']:
    dest=target/item['path']
    # Gitlinks produce empty directories; git clone accepts only those empty destinations.
    subprocess.run(['git','clone','--no-checkout',str(package/item['bundle']),str(dest)],check=True)
    subprocess.run(['git','-C',str(dest),'checkout','--detach',item['sha']],check=True,env={**os.environ,'GIT_LFS_SKIP_SMUDGE':'1'})
    actual=subprocess.check_output(['git','-C',str(dest),'rev-parse','HEAD'],text=True).strip()
    if actual!=item['sha']:raise ValueError('Materialized revision mismatch')
    for entry in item.get('lfs',[]):
      oid=entry['oid'];source=package/'lfs'/oid;target_file=dest/entry['name']
      pointer=target_file.read_text()
      if f'oid sha256:{oid}' not in pointer:raise ValueError('Unexpected materialized LFS pointer')
      shutil.copyfile(source,target_file)  # Preserve executable mode from Git checkout.
      obj=dest/'.git/lfs/objects'/oid[:2]/oid[2:4]/oid
      obj.parent.mkdir(parents=True,exist_ok=True)
      shutil.copyfile(source,obj)
  (target/'.staging-package.json').write_text(json.dumps(manifest,indent=2)+'\n')
  return manifest


if __name__=='__main__':
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--package',type=Path,required=True);parser.add_argument('--destination',type=Path,required=True)
  args=parser.parse_args();materialize(args.package,args.destination)
  print(f'Prepared {args.destination}; boot selection and shared state unchanged.')
