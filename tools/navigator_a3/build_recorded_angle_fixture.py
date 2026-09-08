"""Extract minimal immutable recorded inputs; no controller or vehicle modifications."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

FIELDS = ('t_ns carControl_t_ns general_curvature speed_mps lat_active driver_pressed '
          'carControl_valid carState_valid yaw_curvature rawYaw_t_ns rawSpeed_t_ns '
          'rawYaw_valid rawSpeed_valid request_mode stall_blip').split()


def build(source):
  warnings = {}
  hashes = {}
  for name in ('warning-1', 'warning-2'):
    path = source / 'events' / f'{name}.csv'
    hashes[str(path.relative_to(source))] = hashlib.sha256(path.read_bytes()).hexdigest()
    warnings[name] = [{key: row[key] for key in FIELDS} for row in csv.DictReader(path.open())]
  rejects_path = source / 'rejections.json'
  hashes[rejects_path.name] = hashlib.sha256(rejects_path.read_bytes()).hexdigest()
  rejection = next(r for r in json.loads(rejects_path.read_text()) if r['wire_mode'] == 1)
  orders = [r['order'] for r in rejection['all_plausible_publications']]
  selected = set(orders + [rejection['order']])
  transport = source / 'transport-events.jsonl'
  hashes[transport.name] = hashlib.sha256(transport.read_bytes()).hexdigest()
  events = [e for line in transport.open() if (e := json.loads(line))['order'] in selected]
  assert len(events) == 6 and events[-1]['kind'] == 'rejected'
  data = {'warnings': warnings, 'active_rejection_events': events, 'candidate_orders': orders}
  return {'schema_version': 1, 'route_id': rejection['route_id'], 'provenance': 'recorded',
          'donor_commit': '3210caa02d9e09b46d08be490f689edb23b22b20', 'source_sha256': hashes,
          'selection': 'Full warning CSV windows and all five exact preceding active-rejection publications; native timestamps unchanged.',
          'limits': 'CSV inputs are recorded donor evidence. A3 proposals are synthetic; historical echoes cannot acknowledge them. Raw sensor checksum/counter validity not independently established.',
          'data_sha256': hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':')).encode()).hexdigest(), 'data': data}


if __name__ == '__main__':
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('analysis_directory', type=Path)
  p.add_argument('--output', type=Path, default=Path(__file__).with_name('fixtures') / 'recorded_angle_release.json')
  args = p.parse_args()
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(build(args.analysis_directory), separators=(',', ':')) + '\n')
