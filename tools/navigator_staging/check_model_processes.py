"""Check Linux process argv without matching search strings or shell command text."""
from pathlib import Path
import re

MODULES = {'openpilot.selfdrive.modeld.modeld', 'openpilot.selfdrive.modeld.dmonitoringmodeld',
           'selfdrive.modeld.modeld', 'selfdrive.modeld.dmonitoringmodeld'}
PROGRAMS = {'modeld', 'dmonitoringmodeld', 'modeld.py', 'dmonitoringmodeld.py'}


def is_model_command(argv):
  if not argv:
    return False
  if argv[0] in MODULES or Path(argv[0]).name in PROGRAMS:
    return True
  if not re.fullmatch(r'python(?:\d+(?:\.\d+)*)?', Path(argv[0]).name):
    return False
  for index, arg in enumerate(argv[1:], 1):
    if arg == '-c':
      return False
    if arg == '-m':
      return index + 1 < len(argv) and argv[index + 1] in MODULES
    if not arg.startswith('-'):
      return Path(arg).name in PROGRAMS
  return False


def active_models(proc=Path('/proc')):
  if not proc.is_dir():
    raise RuntimeError('Linux process inventory unavailable')
  found = []
  for entry in proc.iterdir():
    if not entry.name.isdigit():
      continue
    try:
      argv = entry.joinpath('cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
    except (FileNotFoundError, ProcessLookupError):
      continue
    if is_model_command(argv):
      found.append((entry.name, argv))
  return found


if __name__ == '__main__':
  matches = active_models()
  for pid, argv in matches:
    print(f'Active model process {pid}: {argv}')
  raise SystemExit(1 if matches else 0)
