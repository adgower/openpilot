"""Extract recorded transport evidence from local rlogs; never contact a device."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
from collections.abc import Callable, Iterable


def route_segment(path: Path) -> tuple[str, int]:
  """Read segment identity from downloaded names or route--N/rlog layouts."""
  name = path.name
  if name in ('rlog', 'rlog.zst', 'rlog.bz2'):
    name = path.parent.name
  else:
    name = re.sub(r'--rlog(?: \(\d+\))?(?:\.(?:zst|bz2))?$', '', name)
  match = re.fullmatch(r'(.+)--(\d+)', name)
  if match is None:
    raise ValueError(f'Cannot identify route and segment from local rlog: {path}')
  return match[1], int(match[2])


def input_sort_key(path: Path) -> tuple[str, int, str]:
  route, segment = route_segment(path)
  return route, segment, str(path)


def classify_src(src: int) -> tuple[str, int | None]:
  if 0 <= src < 8:
    return 'rx', src
  if 0x80 <= src < 0x88:
    return 'returned', src - 0x80
  if 0xC0 <= src < 0xC8:
    return 'rejected', src - 0xC0
  return 'unknown', None


def normalize_event(msg, route_id: str, source_file: str, source_event_index: int) -> list[dict]:
  """Flatten one event without changing native timing or packet order."""
  which = msg.which()
  if which not in ('sendcan', 'can', 'pandaStates'):
    return []
  common = {'route_id': route_id, 'provenance': 'recorded', 't_ns': int(msg.logMonoTime),
            'valid': bool(msg.valid), 'source_file': source_file, 'source_event_index': source_event_index}
  rows = []
  if which == 'pandaStates':
    for index, state in enumerate(msg.pandaStates):
      rows.append(dict(common, kind='health', panda_index=index, safetyTxBlocked=int(state.safetyTxBlocked),
                       controlsAllowed=bool(state.controlsAllowed), safetyRxChecksInvalid=bool(state.safetyRxChecksInvalid),
                       safetyModel=str(state.safetyModel), safetyParam=int(state.safetyParam),
                       alternativeExperience=int(state.alternativeExperience)))
    return rows
  for index, packet in enumerate(getattr(msg, which)):
    src = int(packet.src)
    if which == 'can' and src < 128:
      continue
    kind, bus = classify_src(src)
    data = bytes(packet.dat)
    rows.append(dict(common, kind='published' if which == 'sendcan' else kind, bus=bus,
                     address=int(packet.address), dlc=len(data), data_hex=data.hex(), src=src, packet_index=index))
  return rows


def file_sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open('rb') as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
      digest.update(chunk)
  return digest.hexdigest()


def extract(paths: Iterable[Path], output: Path, reader_factory: Callable | None = None) -> dict:
  """Write transport-events.jsonl and transport-manifest.json from local files."""
  paths = sorted((Path(p).resolve(strict=True) for p in paths), key=input_sort_key)
  if not paths:
    raise ValueError('At least one local rlog is required')
  inputs, duplicates, seen = [], [], {}
  for path in paths:
    route, segment = route_segment(path)
    digest = file_sha256(path)
    item = {'path': str(path), 'route_id': route, 'segment': segment, 'sha256': digest, 'size_bytes': path.stat().st_size}
    if digest in seen:
      duplicates.append(dict(item, duplicate_of=seen[digest]))
    else:
      inputs.append(item)
      seen[digest] = str(path)
  if reader_factory is None:
    # Keep capnp and the full openpilot reader outside the pure normalization API.
    from openpilot.tools.lib.logreader import _LogFileReader
    def reader_factory(path):
      return _LogFileReader(path, only_union_types=True)
  output.mkdir(parents=True, exist_ok=True)
  event_path = output / 'transport-events.jsonl'
  count = 0
  counts: dict[str, int] = {}
  with event_path.open('w') as stream:
    for item in inputs:
      for event_index, msg in enumerate(reader_factory(item['path'])):
        for row in normalize_event(msg, item['route_id'], item['path'], event_index):
          row['order'] = count
          stream.write(json.dumps(row, separators=(',', ':')) + '\n')
          count += 1
          counts[row['kind']] = counts.get(row['kind'], 0) + 1
  manifest = {'schema_version': 1, 'provenance': 'recorded', 'inputs': inputs, 'duplicates': duplicates,
              'event_count': count, 'kind_counts': counts, 'events_file': event_path.name,
              'events_sha256': file_sha256(event_path), 'extractor_sha256': file_sha256(Path(__file__)),
              'ordering': 'route, numeric segment, source path; native message and packet order; timestamps never sorted',
              'deduplication': 'identical whole-file SHA-256 content only',
              'scope': 'local recorded transport evidence; not controller or safety simulation'}
  (output / 'transport-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
  return manifest


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--rlog', type=Path, action='append', required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  print(json.dumps(extract(args.rlog, args.output), indent=2))


if __name__ == '__main__':
  main()
