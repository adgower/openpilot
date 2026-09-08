"""Build pinned host-only dependencies for real card tests. Never starts services.

Run with the isolated diagnostics Python after installing the versions listed
in HOST_DEPENDENCIES. macOS only; this is not a full device or firmware build.
"""
import argparse
import hashlib
import importlib.metadata
import io
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tarfile

HOST_DEPENDENCIES = {'scons': '4.9.1', 'Cython': '3.1.4', 'setuptools': '80.9.0',
                     'comma-deps-json11': '20170411.0.post98', 'comma-deps-zeromq': '4.3.5.post98',
                     'comma-deps-capnproto': '1.0.1.post98'}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--msgq-repo', type=Path, required=True)
  parser.add_argument('--child', type=Path, required=True)
  parser.add_argument('--build-dir', type=Path, required=True)
  args = parser.parse_args()
  if platform.system() != 'Darwin':
    parser.error('This bounded host build recipe is verified for macOS only')
  versions = {name: importlib.metadata.version(name) for name in HOST_DEPENDENCIES}
  if versions != HOST_DEPENDENCIES:
    parser.error(f'Install exact HOST_DEPENDENCIES first; found {versions}')
  import capnproto
  import json11
  import zeromq
  root = Path(__file__).resolve().parents[2]
  revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD:msgq_repo'], text=True).strip()
  args.build_dir.mkdir(parents=True, exist_ok=True)
  msgq = args.build_dir / 'msgq'
  if msgq.exists():
    parser.error('Use a fresh build directory; existing dependency sources are never overwritten')
  msgq.mkdir()
  archive = subprocess.check_output(['git', '-C', str(args.msgq_repo), 'archive', revision])
  with tarfile.open(fileobj=io.BytesIO(archive)) as source:
    source.extractall(msgq, filter='data')
  env = dict(os.environ, PATH=str(Path(sys.executable).parent) + ':' + capnproto.BIN_DIR + ':' + os.environ['PATH'])
  commands = []
  with (args.build_dir / 'build.log').open('w') as log:
    def run(command, cwd=root):
      commands.append({'argv': list(map(str, command)), 'cwd': str(cwd)})
      log.write(json.dumps(commands[-1]) + '\n'); log.flush()
      subprocess.run(command, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    run([sys.executable, '-m', 'SCons', '--minimal', '-j4', 'msgq/ipc_pyx.so'], msgq)
    cereal = root / 'openpilot/cereal'
    gen = cereal / 'gen/cpp'
    gen.mkdir(parents=True, exist_ok=True)
    car = args.child.resolve() / 'opendbc/car'
    run([capnproto.BIN_DIR + '/capnpc', '--src-prefix=' + str(cereal), '--src-prefix=' + str(car),
         '--import-path=' + str(car), *[str(cereal / f) for f in ('log.capnp', 'deprecated.capnp', 'custom.capnp')],
         str(car / 'car.capnp'), '-oc++:' + str(gen)])
    output = root / 'openpilot/common/libparams_c.dylib'
    run(['clang++', '-dynamiclib', '-std=c++17', '-fPIC', '-O2',
         *['-I' + str(p) for p in (root, root / 'openpilot', gen, json11.INCLUDE_DIR, zeromq.INCLUDE_DIR, capnproto.INCLUDE_DIR)],
         *[str(root / 'openpilot/common' / f) for f in ('params_c.cc', 'params.cc', 'util.cc', 'swaglog.cc')],
         '-L' + json11.LIB_DIR, '-L' + zeromq.LIB_DIR, '-ljson11', '-lzmq', '-o', str(output)])
  files = [msgq / 'msgq/ipc_pyx.so', output, *sorted(gen.glob('*.h')), *sorted(gen.glob('*.c++'))]
  result = {'purpose': 'host card tests only; no device build, launch or firmware', 'msgq_revision': revision,
            'dependencies': versions, 'python': sys.version, 'commands': commands,
            'compiler': subprocess.check_output(['clang++', '--version'], text=True),
            'artifacts': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
  (args.build_dir / 'manifest.json').write_text(json.dumps(result, indent=2) + '\n')
  print(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
