"""Session-bound prediction pilot. All filesystem/Params work is off the control thread."""

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import queue
import threading
import time
import uuid

from opendbc.car.ford.prediction_pilot import Context, Request, Runner
from opendbc.car.ford.longitudinal_precharge import allocator_request_valid
from opendbc.car.ford.values import CAR

PERMIT = 'NavigatorPredictionPilotPermit'
CANCEL = 'NavigatorPredictionPilotCancel'
STATUS = 'NavigatorPredictionPilotStatus'


class PermitBroker:
  def __init__(self, session_id, source_id):
    self.session_id, self.source_id = session_id, source_id
    self.seen = set()
    self.cancel_ns = 0

  def cancel(self, now_ns):
    self.cancel_ns = max(self.cancel_ns, now_ns)

  def accept(self, payload, now_ns, *, parked):
    if not isinstance(payload, dict) or type(payload.get('schema')) is not int or payload.get('schema') != 1 or payload.get('enabled') is not True:
      raise ValueError('explicit versioned enable required')
    if payload.get('session_id') != self.session_id or payload.get('source_id') != self.source_id:
      raise ValueError('session/source mismatch')
    issued = payload.get('issued_ns')
    if type(issued) is not int or not 0 <= now_ns - issued <= 2_000_000_000 or issued <= self.cancel_ns:
      raise ValueError('stale, future or cancelled permit')
    if not parked:
      raise ValueError('fresh stationary Park/disengaged acknowledgment required')
    request = Request(payload.get('trial_id'), payload.get('condition'), self.source_id, payload.get('duration_s', 3.0), payload.get('wait_s', 30.0))
    # Validate the same bounded lease without rebasing its original issue time.
    Runner(enabled=True).arm(request, issued / 1e9)
    if request.trial_id in self.seen:
      raise ValueError('trial ID already consumed')
    self.seen.add(request.trial_id)
    return request, issued / 1e9


class PilotWorker:
  """Single I/O owner; RT only swaps snapshots and uses bounded nonblocking queues."""

  def __init__(self, params, files, log_dir, *, clock=time.monotonic_ns):
    self.params, self.files, self.clock = params, tuple(Path(f) for f in files), clock
    self.session_id = uuid.uuid4().hex
    self.binding = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in self.files}
    self.source_id = hashlib.sha256(json.dumps(self.binding, sort_keys=True).encode()).hexdigest()
    self.log_dir = Path(log_dir)
    self.permits = queue.Queue(maxsize=1)
    self.records = queue.Queue(maxsize=64)
    self.terminal_records = queue.Queue(maxsize=64)
    self.audit_failed = False
    self.cancel_generation = 0
    self.cancel_ns = 0
    self.health_generation = 0
    self.snapshot = {
      'valid_until': 0,
      't': 0,
      'healthy': False,
      'allowed': False,
      'active': False,
      'mismatch': True,
      'fault': True,
      'source': False,
      'audit': False,
    }
    self.status = {}
    self.terminal = None
    self.stop = threading.Event()
    self.thread = None

  def start(self):
    self.thread = threading.Thread(target=self.run, name='prediction-pilot-io', daemon=True)
    self.thread.start()

  def publish(self, record):
    try:
      self.records.put_nowait(record)
      return True
    except queue.Full:
      return False

  def note(self, record):
    self.terminal = record
    try:
      self.terminal_records.put_nowait(record)
    except queue.Full:
      self.audit_failed = True
      self.snapshot = dict(self.snapshot, audit=False, healthy=False)

  def run(self):
    import openpilot.cereal.messaging as messaging

    topics = ['selfdriveState', 'pandaStates', 'onroadEvents']
    sockets = {name: messaging.sub_sock(name) for name in topics}
    latest, seen = {}, {}
    last_raw = {PERMIT: None, CANCEL: None}
    source_ok = True
    last_hash = 0
    last_params = 0
    last_status = 0
    try:
      self.log_dir.mkdir(parents=True, exist_ok=True)
      with (self.log_dir / (self.session_id + '.jsonl')).open('x') as stream:
        stream.write(json.dumps({'event': 'session', 'session_id': self.session_id, 'source_id': self.source_id, 'binding': self.binding}) + '\n')
        stream.flush()
        while not self.stop.is_set():
          now = self.clock()
          if self.audit_failed:
            raise RuntimeError('terminal audit backpressure')
          # Drain every message, preserving brief faults between controller callbacks.
          for topic, sock in sockets.items():
            for event in messaging.drain_sock(sock):
              d = getattr(event, topic)
              if topic == 'selfdriveState':
                value = {'active': bool(d.active), 'mismatch': 'controlsMismatch' in str(d.alertType)}
              elif topic == 'pandaStates':
                boards = [p for p in d if str(p.safetyModel) == 'ford']
                value = {
                  'known': bool(boards and all(p.safetyParam == 11 for p in boards)),
                  'allowed': bool(boards and all(p.controlsAllowed for p in boards)),
                  'fault': bool(not boards or any(p.heartbeatLost or p.safetyRxChecksInvalid or len(p.faults) for p in boards)),
                }
              else:
                value = {'mismatch': any(str(e.name) == 'controlsMismatch' for e in d)}
              previous = latest.get(topic, {})
              withdrawn = any(previous.get(k) is True and value.get(k) is not True for k in ['allowed', 'active', 'known'])
              if withdrawn or value.get('fault') or value.get('mismatch') or not event.valid:
                self.health_generation += 1
              latest[topic] = value
              seen[topic] = (int(event.logMonoTime), bool(event.valid))
          if now - last_params >= 50_000_000:
            # Cancel always precedes permits; reject even delayed pre-cancel messages.
            raw_cancel = self.params.get(CANCEL)
            if raw_cancel and raw_cancel != last_raw[CANCEL]:
              last_raw[CANCEL] = raw_cancel
              self.cancel_ns = now
              self.cancel_generation += 1
            raw = self.params.get(PERMIT)
            if raw and raw != last_raw[PERMIT]:
              last_raw[PERMIT] = raw
              try:
                if len(raw) > 4096:
                  raise ValueError('oversized permit')
                payload = json.loads(raw)
                stream.write(json.dumps({'event': 'permit_received', 't': now, 'payload': payload}) + '\n')
                stream.flush()  # audit acceptance before RT can consume
                self.permits.put_nowait(payload)
              except (ValueError, TypeError, queue.Full):
                self.note({'event': 'request_rejected', 'reason': 'invalid or busy mailbox', 't': now})
            last_params = now
          if now - last_hash >= 100_000_000:
            source_ok = source_ok and all(hashlib.sha256(p.read_bytes()).hexdigest() == self.binding[str(p)] for p in self.files)
            last_hash = now
            if not source_ok:
              self.snapshot = dict(self.snapshot, source=False)
          for _ in range(64):
            try:
              record = self.records.get_nowait()
            except queue.Empty:
              break
            stream.write(json.dumps(record, allow_nan=False) + '\n')
          for _ in range(64):
            try:
              terminal = self.terminal_records.get_nowait()
            except queue.Empty:
              break
            stream.write(json.dumps(terminal, allow_nan=False) + '\n')
          stream.flush()
          ages = {'selfdriveState': 100_000_000, 'pandaStates': 1_000_000_000, 'onroadEvents': 1_500_000_000}
          healthy = all(k in seen and seen[k][1] and 0 <= now - seen[k][0] <= age for k, age in ages.items())
          a, b, e = (latest.get(k, {}) for k in topics)
          self.snapshot = {
            'valid_until': min((seen[k][0] + age for k, age in ages.items() if k in seen), default=0),
            't': now,
            'healthy': bool(healthy and b.get('known')),
            'allowed': b.get('allowed', False),
            'active': a.get('active', False),
            'mismatch': bool(a.get('mismatch', True) or e.get('mismatch', True)),
            'fault': b.get('fault', True),
            'source': source_ok,
            'audit': True,
          }
          if now - last_status >= 100_000_000:
            self.params.put(STATUS, json.dumps(dict(self.status, session_id=self.session_id, source_id=self.source_id, io_ns=now)), block=True)
            last_status = now
          self.stop.wait(0.02)
    except Exception as exc:
      self.snapshot = dict(self.snapshot, audit=False, healthy=False)
      self.terminal = {'event': 'worker_failed', 'reason': type(exc).__name__}


class PredictionPilot:
  def __init__(self, worker):
    self.worker = worker
    self.broker = PermitBroker(worker.session_id, worker.source_id)
    self.runner = Runner()
    self.cancel_generation = worker.cancel_generation
    self.health_generation = worker.health_generation
    self.state_ns = 0
    self.state_valid = False
    self.last_event = 0

  def feed(self, *, state_ns, state_valid):
    self.state_ns, self.state_valid = state_ns, state_valid

  def cancel(self, reason, now_ns):
    self.runner.cancel(reason, now_ns / 1e9)
    self.worker.note({'event': 'cancel', 'reason': reason, 't': now_ns, 'trial_id': self.runner.request.trial_id if self.runner.request else None})

  def apply(self, controller, cc, cs, now_ns, msg):
    w, o = self.worker, cs.out
    health_generation_at_entry = w.health_generation
    h = w.snapshot

    def healthy_at(snapshot):
      return (
        0 <= now_ns - snapshot['t'] <= 250_000_000
        and now_ns <= snapshot.get('valid_until', 0)
        and snapshot['healthy']
        and snapshot['source']
        and snapshot['audit']
        and not getattr(w, 'audit_failed', False)
        and not snapshot['fault']
        and not snapshot['mismatch']
      )

    healthy = healthy_at(h)
    parked = bool(
      healthy
      and self.state_valid
      and 0 < self.state_ns <= now_ns
      and now_ns - self.state_ns <= 100_000_000
      and o.canValid
      and not o.canTimeout
      and str(o.gearShifter) == 'park'
      and abs(o.vEgo) <= 0.1
      and not cc.enabled
      and not cc.longActive
      and not h['active']
      and not o.accFaulted
      and controller.longitudinal_coordination_enabled
      and controller.CP.openpilotLongitudinalControl
      and controller.CP.carFingerprint == CAR.FORD_EXPEDITION_MK4
      and controller.coordination_control_valid
      and not controller.coordination_fault
    )
    if w.cancel_generation != self.cancel_generation:
      self.cancel_generation = w.cancel_generation
      self.broker.cancel(w.cancel_ns)
      self.cancel('operator_cancel', now_ns)
    if self.runner.state in ('waiting', 'active') and (not healthy or w.health_generation != self.health_generation):
      self.cancel('health_source_audit_lost', now_ns)
    try:
      payload = w.permits.get_nowait()
    except queue.Empty:
      payload = None
    if payload is not None:
      try:
        if self.runner.state in ('waiting', 'active'):
          raise ValueError('trial busy')
        request, origin = self.broker.accept(payload, now_ns, parked=parked)
        self.runner = Runner(enabled=True)
        self.runner.arm(request, origin)
        self.last_event = 0
        self.health_generation = health_generation_at_entry
      except (ValueError, TypeError) as exc:
        w.note({'event': 'request_rejected', 't': now_ns, 'reason': str(exc)})
    w.status = {
      't_ns': now_ns,
      'parked': parked,
      'state': self.runner.state,
      'ready': parked,
      'trial_id': self.runner.request.trial_id if self.runner.request else None,
    }
    if self.runner.state not in ('waiting', 'active'):
      return msg
    q = cc.navigatorLongitudinal
    torque = getattr(cs, 'longitudinal_brake_torque', None)
    observer = getattr(controller, 'last_observer', None)
    context = Context(
      source_id=w.source_id,
      alpha=bool(controller.CP.openpilotLongitudinalControl and controller.CP.carFingerprint == CAR.FORD_EXPEDITION_MK4),
      enabled=bool(cc.enabled),
      long_active=bool(cc.longActive),
      phase=str(cc.actuators.longControlState),
      speed_ms=float(o.vEgo),
      can_valid=bool(o.canValid),
      control_valid=bool(controller.coordination_control_valid),
      source_valid=bool(healthy and self.state_valid),
      pose_valid=bool(q.poseValid),
      reference_valid=bool(q.referenceValid),
      request_valid=bool(q.enabled and allocator_request_valid(q, now_ns)),
      epoch=int(q.epoch),
      sequence=int(q.sequence),
      producer_ns=int(q.producerMonoTime),
      state_ns=self.state_ns,
      pose_ns=int(q.poseMonoTime),
      target_ns=int(q.targetMonoTime),
      health_valid=bool(h['healthy']),
      controls_allowed=bool(h['allowed']),
      selfdrive_active=bool(h['active']),
      mismatch=bool(h['mismatch']),
      fault=bool(h['fault'] or controller.coordination_fault or o.accFaulted or o.canTimeout or (observer and observer.fault)),
      in_drive=str(o.gearShifter) == 'drive',
      steering_override=bool(o.steeringPressed),
      gas_pedal=bool(o.gasPressed),
      brake_pedal=bool(o.brakePressed),
      observer_clear=bool(observer and observer.allow),
      torque_valid=bool(torque and torque.valid),
      torque_ns=round(torque.t * 1e9) if torque else 0,
      torque=float(torque.value) if torque else 0.0,
    )
    result = self.runner.process(now_ns / 1e9, context, msg)
    record = {
      't_ns': now_ns,
      'trial_id': self.runner.request.trial_id,
      'context': asdict(context),
      'input_packet': bytes(msg[1]).hex(),
      'proposed_output_packet': bytes(result[1]).hex(),
      'events': self.runner.events[self.last_event :],
    }
    if not w.publish(record):
      self.cancel('audit_backpressure', now_ns)
      return msg
    self.last_event = len(self.runner.events)
    if (
      w.cancel_generation != self.cancel_generation
      or w.health_generation != self.health_generation
      or not healthy_at(w.snapshot)
      or (result != msg and not (w.snapshot['allowed'] and w.snapshot['active']))
    ):
      self.cancel('concurrent_revocation', now_ns)
      return msg
    return result


def create_prediction_pilot(params, cp):
  from openpilot.common.hardware.hw import Paths
  from opendbc.car.ford import carcontroller, prediction_pilot, longitudinal_recovery
  from openpilot.selfdrive.controls.lib import longcontrol
  import openpilot.selfdrive.car.card as card

  files = [Path(module.__file__) for module in (carcontroller, prediction_pilot, longitudinal_recovery, longcontrol, card)]
  files.extend(Path(carcontroller.__file__).parent.glob('*.py'))
  files.extend(Path(__file__).parent.glob('longitudinal*.py'))
  files.append(Path(__file__))
  files = sorted(set(files))
  worker = PilotWorker(params, files, Path(Paths.log_root()).parent / 'navigator-pilot')
  # CP source/config binding is private in the audit; no vehicle identifier is published.
  worker.source_id = hashlib.sha256(
    (
      worker.source_id + str(cp.carFingerprint) + str(cp.openpilotLongitudinalControl) + str([(str(s.safetyModel), s.safetyParam) for s in cp.safetyConfigs])
    ).encode()
  ).hexdigest()
  runtime = PredictionPilot(worker)
  worker.start()
  return runtime
