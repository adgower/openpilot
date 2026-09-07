"""Prove the offline experiment leaves frozen A2 runtime source unchanged."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

PARENT = 'e855605fdf1b2ba480185d9473fa544b8ee4b99c'
CHILD = '9fa4a58ee1ea0aff0ea40a9cd820756a0b3f424d'
SELECTOR = b'\n# Owner-authorized A2-WB experiment; mass retained.\nexport NAVIGATOR_A2_PROFILE="2023-navigator-swb-4wd"\n'


def git(root, *args):
  return subprocess.check_output(['git', '-C', str(root), *args])


def audit(parent, child):
  checked = {}
  for root, revision, label in [(parent, PARENT, 'parent'), (child, CHILD, 'opendbc')]:
    records = git(root, 'ls-tree', '-r', revision).decode().splitlines()
    count = 0
    for record in records:
      mode_kind_sha, path = record.split('\t', 1)
      if mode_kind_sha.split()[1] != 'blob':
        continue
      original = git(root, 'show', f'{revision}:{path}')
      actual = os.readlink(root / path).encode() if mode_kind_sha.startswith('120000 ') else (root / path).read_bytes()
      if original.startswith(b'version https://git-lfs.github.com/spec/v1\n') and actual != original:
        fields = dict(line.split(' ', 1) for line in original.decode().splitlines())
        assert hashlib.sha256(actual).hexdigest() == fields['oid'].removeprefix('sha256:'), f'LFS hash changed: {path}'
        assert len(actual) == int(fields['size']), f'LFS size changed: {path}'
      elif label == 'parent' and path == 'launch_env.sh':
        assert actual == original + SELECTOR, 'A2 startup selection or unrelated launcher change'
      else:
        assert actual == original, f'Frozen runtime source changed: {label}/{path}'
      count += 1
    checked[label] = {'base': revision, 'tracked_blobs_verified': count}
  checked['launch_env_sha256'] = hashlib.sha256((parent / 'launch_env.sh').read_bytes()).hexdigest()
  checked['meaning'] = ('All existing A2 child files and frozen parent files are byte-identical, '
                        + 'except the exact previously deployed A2 startup selector. New offline '
                        + 'modules do not change the production controller, safety hooks, schemas, '
                        + 'model, longitudinal behavior, warnings or learned-state handling.')
  checked['limits'] = 'Source equivalence only; no device, native build, firmware or vehicle validation.'
  return checked


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--parent', type=Path, required=True)
  parser.add_argument('--opendbc', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  result = audit(args.parent, args.opendbc)
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, indent=2) + '\n')
  print(json.dumps(result))


if __name__ == '__main__':
  main()
