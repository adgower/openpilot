"""Replay production VisionCurveContext on recorded CAN/state/model timing.

The only injected observations are Ford setpoint acknowledgements. Motion,
model output and switch bytes stay recorded. This is an offline command replay.
"""

import argparse
from collections import Counter
import hashlib
import inspect
import json
from pathlib import Path
from types import ModuleType
import sys
import zstandard
from openpilot.cereal import log
from opendbc.can import CANParser
from openpilot.selfdrive.car.vision_curve_context import VisionCurveContext

MPH = 0.44704
TOPICS = {'can', 'carState', 'carControl', 'modelV2', 'radarState'}


def reference_context(source_root):
  module = ModuleType('nav_reference_curve_speed')
  sys.modules[module.__name__] = module
  exec(compile((source_root / 'vision_curve_speed.py').read_text(), str(source_root / 'vision_curve_speed.py'), 'exec'), module.__dict__)
  ctx = ModuleType('nav_reference_curve_context')
  source = (source_root / 'vision_curve_context.py').read_text().replace('openpilot.selfdrive.car.vision_curve_speed', 'nav_reference_curve_speed')
  exec(compile(source, str(source_root / 'vision_curve_context.py'), 'exec'), ctx.__dict__)
  return ctx.VisionCurveContext


class SM:
  def __init__(self):
    self.latest = {}
    self.valid = {}
    self.logMonoTime = {}

  def __getitem__(self, key):
    return self.latest[key]

  def record(self, event):
    key = event.which()
    self.latest[key] = getattr(event, key)
    self.valid[key] = event.valid
    self.logMonoTime[key] = event.logMonoTime


class Arm:
  def __init__(self, name, cls, ack='normal'):
    self.name = name
    self.context = cls(comfort=True)
    self.ack = ack
    self.setpoint = None
    self.recorded = None
    self.queue = []
    self.events = []
    self.observations = []
    self.dropped = False
    self.states = Counter()

  def update(self, now, origin, sm, parser):
    c = sm['carState']
    control = sm['carControl']
    observed = round(c.cruiseState.speedCluster / MPH)
    external = self.recorded is not None and self.recorded != observed
    if self.setpoint is None or external:
      self.setpoint = observed
      self.queue = []
    self.recorded = observed
    for due, delta in self.queue[:]:
      if now >= due:
        self.setpoint += delta
        self.queue.remove((due, delta))
    state = c.as_builder()
    state.cruiseState.speedCluster = self.setpoint * MPH
    button = self.context.update(now, sm, parser, state.as_reader(), control, False)
    self.states['calls'] += 1
    elapsed = (now - origin) / 1000000000.0
    cruise = self.context.cruise
    getattr(self.context, 'forecast', None)
    if 375 <= elapsed <= 387:
      self.observations.append(
        {
          't': round(elapsed, 4),
          'actual': c.vEgo / MPH,
          'setpoint': self.setpoint,
          'pending': cruise.pending,
          'fault': cruise.fault,
          'model_ns': sm.logMonoTime['modelV2'],
          'lane_confidence': min(list(sm['modelV2'].laneLineProbs)[1:3], default=0),
          'button': button,
        }
      )
    if button:
      delta = -1 if button == 'SET-' else 1
      dropped = False
      if self.ack == 'miss_first_down' and delta < 0 and (not self.dropped):
        dropped = self.dropped = True
      if self.ack == 'miss_first_up' and delta > 0 and (not self.dropped):
        dropped = self.dropped = True
      if not dropped:
        self.queue.append((now + int((0.5 if self.ack == 'delayed' else 0.05) * 1000000000.0), delta))
      self.events.append(
        {
          't': round(elapsed, 4),
          'step': button,
          'target': cruise.pending,
          'observed': self.setpoint,
          'actual': c.vEgo / MPH,
          'dropped': dropped,
          'ceiling': cruise.ceiling,
        }
      )
    if cruise.ceiling is not None:
      assert self.setpoint <= cruise.ceiling + 0.2, (self.name, elapsed, self.setpoint, cruise.ceiling)
      assert cruise.pending is None or 25 <= cruise.pending <= cruise.ceiling

  def result(self):
    ds = [e['step'] for e in self.events]
    return {
      'events': self.events,
      'observations': self.observations,
      'counts': dict(Counter(ds)),
      'reversals': sum((a != b for a, b in zip(ds, ds[1:], strict=False))),
      'calls': self.states['calls'],
    }


def main():
  cli = argparse.ArgumentParser()
  cli.add_argument('manifest', type=Path)
  cli.add_argument('--output', type=Path, required=True)
  cli.add_argument('--only-route', default='')
  cli.add_argument('--baseline-source', type=Path, required=True)
  args = cli.parse_args()
  ref = reference_context(args.baseline_source)
  source_root = Path(inspect.getfile(VisionCurveContext)).parent
  source_hashes = {
    n: hashlib.sha256((source_root / n).read_bytes()).hexdigest() for n in ('vision_curve_speed.py', 'vision_curve_context.py', 'predicted_speed.py')
  }
  baseline_hashes = {n: hashlib.sha256((args.baseline_source / n).read_bytes()).hexdigest() for n in ('vision_curve_speed.py', 'vision_curve_context.py')}
  paths = sorted(
    [Path(x) for x in args.manifest.read_text().splitlines() if Path(x).exists()],
    key=lambda p: (p.parent.name.rsplit('--', 1)[0], int(p.parent.name.rsplit('--', 1)[1])),
  )
  prior = None
  origin = None
  sm = None
  parser = None
  arms = []
  routes = {}
  queue = []
  frames = 0
  segments = 0
  can_counts = Counter()
  for f in paths:
    route, segment = f.parent.name.rsplit('--', 1)
    if args.only_route and (not route.startswith(args.only_route)):
      continue
    if route != prior:
      if prior:
        routes[prior] = {'frames': frames, 'arms': {a.name: a.result() for a in arms}}
      prior = route
      origin = None
      sm = SM()
      queue = []
      frames = 0
      parser = CANParser('ford_lincoln_base_pt', [('Steering_Data_FD1', 0), ('EngBrakeData', 0)], 0)
      arms = [Arm('baseline', ref)] + [
        Arm('integrated_' + mode, VisionCurveContext, mode) for mode in ('normal', 'delayed', 'miss_first_down', 'miss_first_up')
      ]
    with f.open('rb') as h, zstandard.ZstdDecompressor().stream_reader(h) as r:
      events = list(log.Event.read_multiple_bytes(r.read()))
    segments += 1
    if origin is None:
      origin = events[0].logMonoTime - int(segment) * 60 * 1000000000
    for e in sorted((e for e in events if e.which() in TOPICS), key=lambda e: e.logMonoTime):
      k = e.which()
      if k == 'can':
        selected = []
        for m in e.can:
          if m.address in (131, 357) and m.src == 0:
            selected.append((m.address, bytes(m.dat), m.src))
            can_counts[hex(m.address)] += 1
        if selected:
          queue.append([e.logMonoTime, selected])
        continue
      sm.record(e)
      if k != 'carState':
        continue
      parser.update(queue)
      queue = []
      needed = ('carControl', 'modelV2', 'radarState')
      if any(k not in sm.latest for k in needed):
        continue
      frames += 1
      for a in arms:
        a.update(e.logMonoTime, origin, sm, parser)
    print(route, 'segment', segment, flush=True)
  if prior:
    routes[prior] = {'frames': frames, 'arms': {a.name: a.result() for a in arms}}
  args.output.write_text(
    json.dumps(
      {'segments': segments, 'routes': routes, 'can_counts': dict(can_counts), 'source_hashes': source_hashes, 'baseline_hashes': baseline_hashes}, indent=2
    )
  )


if __name__ == '__main__':
  main()
