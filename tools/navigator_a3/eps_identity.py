"""Offline EPS inventory from carParams.carFw; no device or vehicle access.

These are stored diagnostic payloads, not raw CAN responses. logMonoTime is
carParams publication time, not the original firmware query time. ECU/brand
labels are scanner metadata and do not establish vehicle manufacturer. Firmware
may be cached: repeated publications are not independent fresh ECU queries.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
from pathlib import Path


def decode_identifier(raw: bytes, requests: list[bytes]) -> dict:
  """Interpret only an unambiguous UDS ReadDataByIdentifier request."""
  dids = sorted({r[1:].hex() for r in requests if len(r) == 3 and r[0] == 0x22})
  issues = []
  if len(dids) > 1:
    issues.append('multiple_requested_dids')
  if not dids:
    issues.append('missing_unambiguous_read_did_request')
  did = dids[0] if len(dids) == 1 else None
  value, prefix = raw, None
  # Known F1xx-looking prefixes are retained unless the exact request supports
  # the DID. This is diagnostic payload interpretation, never wire decoding.
  if did and raw.startswith(bytes.fromhex(did)):
    prefix, value = did, raw[2:]
  elif did and len(raw) >= 2 and raw[0] == 0xf1:
    issues.append('payload_request_did_mismatch')
  if not value:
    issues.append('empty_decoded_identifier')
  return {'did': did, 'label': 'diagnostic identifier', 'value_hex': value.hex(),
          'value_printable': ''.join(chr(b) if 32 <= b < 127 else f'\\x{b:02x}' for b in value),
          'prefix_removed_hex': prefix, 'issues': issues}


def _json_safe(value):
  """Preserve rejected record bytes and provenance without inventing a route."""
  if isinstance(value, (bytes, bytearray)):
    return {'bytes_hex': bytes(value).hex()}
  if isinstance(value, dict):
    return {str(k): _json_safe(v) for k, v in value.items()}
  if isinstance(value, (list, tuple)):
    return [_json_safe(v) for v in value]
  if value is None or isinstance(value, (str, int, float, bool)):
    return value
  return {'repr': repr(value)}


def inventory(records, routes=()) -> dict:
  """Normalize mapping records and deduplicate within each route only."""
  grouped = {}
  rejected = []
  for route in routes:
    if isinstance(route, str) and route.strip():
      grouped[route] = {}
    else:
      rejected.append({'reason': 'invalid_declared_route_identity', 'route': _json_safe(route)})
  for record in records:
    if not isinstance(record, dict) or not isinstance(record.get('entry'), dict):
      rejected.append({'reason': 'invalid_record', 'record_repr': repr(record)})
      continue
    route = record.get('route')
    if not isinstance(route, str) or not route.strip():
      rejected.append({'reason': 'invalid_route_identity', 'record': _json_safe(record)})
      continue
    bucket = grouped.setdefault(route, {})
    entry = record['entry']
    if entry.get('address') != 0x730:
      if entry.get('address') is None:
        rejected.append({'reason': 'missing_address', 'route': route,
                         'provenance': {k: v for k, v in record.items() if k != 'entry'}})
      continue
    issues = []
    for field in ('fwVersion', 'request', 'responseAddress', 'bus', 'subAddress', 'ecu', 'brand'):
      if field not in entry:
        issues.append('missing_' + field)
    raw = entry.get('fwVersion')
    if raw is not None and not isinstance(raw, (bytes, bytearray)):
      issues.append('invalid_fwVersion_type')
      raw = None
    if raw == b'':
      issues.append('empty_fwVersion')
    requests = entry.get('request', [])
    if not isinstance(requests, (list, tuple)):
      issues.append('invalid_request_type')
      requests = []
    if any(not isinstance(r, (bytes, bytearray)) for r in requests):
      issues.append('invalid_request_item_type')
    request_bytes = [bytes(r) for r in requests if isinstance(r, (bytes, bytearray))]
    if record.get('valid') is not True:
      issues.append('invalid_carParams_publication')
    for field in ('route', 'file', 'sha256', 'message_index', 'logMonoTime', 'valid', 'entry_index'):
      if field not in record:
        issues.append('missing_' + field)
    item = {k: entry.get(k) for k in ('address', 'responseAddress', 'bus', 'subAddress', 'ecu', 'brand')}
    item.update({k: entry[k] for k in ('logging', 'obdMultiplexing') if k in entry})
    item.update(fwVersion_hex=None if raw is None else bytes(raw).hex(),
                request_hex=[r.hex() for r in request_bytes],
                identifier=None if raw is None else decode_identifier(bytes(raw), request_bytes), issues=issues)
    key = json.dumps(item, sort_keys=True)
    if key not in bucket:
      bucket[key] = dict(item, observation_count=0, observations=[])
    bucket[key]['observation_count'] += 1
    bucket[key]['observations'].append({k: v for k, v in record.items() if k != 'entry'})
  return _summarize(grouped, rejected)


def _summarize(grouped, rejected):
  result = {}
  values = {}
  for route, bucket in grouped.items():
    candidates = list(bucket.values())
    by_did = {}
    usable = 0
    for item in candidates:
      identifier = item['identifier']
      if identifier and identifier['did'] and not identifier['issues'] and not item['issues']:
        usable += 1
        by_did.setdefault(identifier['did'], set()).add(identifier['value_hex'])
    conflicts = [{'did': did, 'values_hex': sorted(v)} for did, v in sorted(by_did.items()) if len(v) > 1]
    result[route] = {'status': 'missing' if not candidates else 'unusable' if not usable else 'conflicting' if conflicts else 'observed',
                     'usable_candidate_count': usable,
                     'candidates': candidates, 'conflicts': conflicts}
    values[route] = by_did
  comparisons = []
  for left, right in itertools.combinations(sorted(values), 2):
    for did in sorted(set(values[left]) | set(values[right])):
      a, b = values[left].get(did, set()), values[right].get(did, set())
      status = ('missing' if not a or not b else 'conflicting' if len(a) > 1 or len(b) > 1
                else 'same_observed_value' if a == b else 'different_observed_values')
      comparisons.append({'routes': [left, right], 'did': did, 'status': status,
                          'values_hex': {left: sorted(a), right: sorted(b)}})
  return {'schema_version': 1, 'scope': __doc__.strip(), 'routes': result,
          'comparisons': comparisons, 'rejected_records': rejected}


def combine_inventories(inputs) -> dict:
  """Combine independently decoded routes without loading multiple schemas.

  Accept parsed JSON inventories. Candidates remain route-specific; repeated
  inputs retain their observations and source manifests explicitly.
  """
  grouped, rejected, manifests = {}, [], []
  for source in inputs:
    manifests.append(copy.deepcopy({k: v for k, v in source.items()
                                    if k not in ('routes', 'comparisons', 'rejected_records')}))
    rejected.extend(copy.deepcopy(source.get('rejected_records', [])))
    for route, data in source['routes'].items():
      bucket = grouped.setdefault(route, {})
      for original in data['candidates']:
        item = copy.deepcopy(original)
        key = json.dumps({k: v for k, v in item.items() if k not in ('observations', 'observation_count')}, sort_keys=True)
        if key not in bucket:
          bucket[key] = item
        else:
          bucket[key]['observations'].extend(item['observations'])
          bucket[key]['observation_count'] += item['observation_count']
  result = _summarize(grouped, rejected)
  result['source_manifests'] = manifests
  return result


def file_sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open('rb') as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
      digest.update(chunk)
  return digest.hexdigest()


def extract(schema_path: Path, paths: list[Path], route: str, import_dirs=()) -> dict:
  """Load exactly one schema per process, recording every carFw publication."""
  import capnp
  import zstandard
  schema_path = schema_path.resolve(strict=True)
  imports = [str(schema_path.parent)] + [str(Path(p).resolve(strict=True)) for p in import_dirs]
  schema = capnp.load(str(schema_path), imports=imports)
  records, inputs = [], []
  for path in sorted(set(p.resolve(strict=True) for p in paths)):
    digest = file_sha256(path)
    source = {'file': str(path), 'sha256': digest, 'route': route}
    count = 0
    with path.open('rb') as compressed:
      # capnp needs a seekable/fileno stream; decompression is local and each
      # file is released before reading the next segment.
      import tempfile
      import shutil
      with tempfile.TemporaryFile() as raw:
        with zstandard.ZstdDecompressor().stream_reader(compressed) as stream:
          shutil.copyfileobj(stream, raw)
        raw.seek(0)
        for index, msg in enumerate(schema.Event.read_multiple(raw)):
          if msg.which() != 'carParams':
            continue
          count += 1
          for entry_index, fw in enumerate(msg.carParams.carFw):
            entry = fw.to_dict()
            records.append(dict(source, message_index=index, entry_index=entry_index,
                                logMonoTime=int(msg.logMonoTime), valid=bool(msg.valid), entry=entry))
    inputs.append(dict(source, carParams_publications=count))
  result = inventory(records, routes=[route])
  result.update(inputs=inputs, schema={'path': str(schema_path), 'sha256': file_sha256(schema_path), 'import_dirs': imports},
                extractor_sha256=file_sha256(Path(__file__)))
  return result


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--schema', type=Path, required=True, help='Exactly one log.capnp schema per process')
  parser.add_argument('--import-dir', type=Path, action='append', default=[], help='Additional explicit capnp import directory; repeatable')
  parser.add_argument('--route', required=True, help='Explicit route identity; invoke separately per route')
  parser.add_argument('--logs', type=Path, nargs='+', required=True, help='Local rlog.zst paths (shell globs allowed)')
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  result = extract(args.schema, args.logs, args.route, args.import_dir)
  args.output.write_text(json.dumps(result, indent=2) + '\n')
  print(json.dumps({'output': str(args.output), 'routes': {k: v['status'] for k, v in result['routes'].items()}}))


if __name__ == '__main__':
  main()
