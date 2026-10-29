import unittest
from openpilot.selfdrive.car.prediction_pilot import PermitBroker


class TestPermit(unittest.TestCase):
  def setUp(self):
    self.b = PermitBroker('session', 'source')
    self.p = {
      'schema': 1,
      'enabled': True,
      'trial_id': 'trial',
      'session_id': 'session',
      'source_id': 'source',
      'issued_ns': 1_000_000_000,
      'condition': 'B',
      'duration_s': 3.0,
      'wait_s': 30.0,
    }

  def test_explicit_session_source_and_age(self):
    for field, value in [('enabled', False), ('session_id', 'old'), ('source_id', 'other'), ('issued_ns', 0), ('condition', 'C'), ('duration_s', 6.0)]:
      with self.subTest(field=field), self.assertRaises(ValueError):
        self.b.accept(dict(self.p, **{field: value}), 1_100_000_000, parked=True)

  def test_park_rechecked_and_duplicate_rejected(self):
    with self.assertRaises(ValueError):
      self.b.accept(self.p, 1_100_000_000, parked=False)
    self.b.accept(self.p, 1_100_000_000, parked=True)
    with self.assertRaises(ValueError):
      self.b.accept(self.p, 1_100_000_000, parked=True)

  def test_cancel_wins_and_is_not_rebased(self):
    self.b.cancel(1_050_000_000)
    with self.assertRaises(ValueError):
      self.b.accept(self.p, 1_100_000_000, parked=True)
    p = dict(self.p, issued_ns=1_060_000_000)
    req, origin = self.b.accept(p, 1_100_000_000, parked=True)
    self.assertEqual(origin, 1.06)
    self.assertEqual(req.condition, 'B')


from types import SimpleNamespace as NS
import queue
from opendbc.car import structs
from opendbc.car.ford.values import CAR
from opendbc.car.ford.longitudinal_coordination import Torque
from opendbc.car.ford.tests.test_prediction_pilot import packet
from openpilot.selfdrive.car.prediction_pilot import PredictionPilot
from openpilot.tools.navigator.prediction_pilot import command


def fixture():
  w = NS(
    session_id='session',
    source_id='source',
    cancel_generation=0,
    cancel_ns=0,
    health_generation=0,
    permits=queue.Queue(maxsize=1),
    status={},
    terminal=None,
    publish=lambda r: True,
    snapshot={
      'valid_until': 1_100_000_000,
      't': 1_000_000_000,
      'healthy': True,
      'source': True,
      'audit': True,
      'fault': False,
      'mismatch': False,
      'allowed': True,
      'active': False,
    },
  )
  w.notes = []

  def note(event):
    w.terminal = event
    w.notes.append(event)

  w.note = note
  r = PredictionPilot(w)
  o = structs.CarState(vEgo=0.0, canValid=True, gearShifter=structs.CarState.GearShifter.park)
  cs = NS(out=o, longitudinal_brake_torque=Torque(1.0, 0.0))
  c = NS(
    CP=NS(openpilotLongitudinalControl=True, carFingerprint=CAR.FORD_EXPEDITION_MK4),
    coordination_fault=None,
    coordination_control_valid=True,
    longitudinal_coordination_enabled=True,
    last_observer=NS(allow=True, fault=False),
  )
  cc = structs.CarControl()
  r.feed(state_ns=1_000_000_000, state_valid=True)
  return w, r, c, cc, cs


def permit(now=1_000_000_000, **kw):
  return dict(
    schema=1, enabled=True, trial_id='trial', session_id='session', source_id='source', issued_ns=now, condition='B', duration_s=3.0, wait_s=30.0, **kw
  )


def frame(r, c, cc, cs, now):
  w = r.worker
  w.snapshot = dict(w.snapshot, t=now, valid_until=now + 100_000_000, active=bool(cc.enabled))
  r.feed(state_ns=now, state_valid=True)
  cs.longitudinal_brake_torque = Torque(now / 1e9, 0.0)
  q = cc.navigatorLongitudinal
  q.enabled = True
  q.allocatorVersion = 1
  from opendbc.car.ford.longitudinal_precharge import PRECHARGE_PROTOCOL_VERSION

  q.allocatorVersion = PRECHARGE_PROTOCOL_VERSION
  q.epoch = 1
  q.sequence = now // 10_000_000
  q.producerMonoTime = now
  q.poseMonoTime = now
  q.poseValid = True
  q.referenceValid = True
  q.targetValid = True
  q.targetMonoTime = now
  q.targetAcceleration = 0.25
  return r.apply(c, cc, cs, now, packet())


class TestRuntime(unittest.TestCase):
  def armed(self):
    w, r, c, cc, cs = fixture()
    w.permits.put_nowait(permit())
    self.assertEqual(r.apply(c, cc, cs, 1_000_000_000, packet()), packet())
    self.assertEqual(r.runner.state, 'waiting')
    return w, r, c, cc, cs

  def active(self):
    w, r, c, cc, cs = self.armed()
    cc.enabled = True
    cc.longActive = True
    cc.actuators.longControlState = structs.CarControl.Actuators.LongControlState.pid
    cs.out.gearShifter = structs.CarState.GearShifter.drive
    cs.out.vEgo = 5.0
    for n in range(1, 9):
      out = frame(r, c, cc, cs, 1_000_000_000 + n * 20_000_000)
    self.assertEqual(r.runner.state, 'active')
    self.assertNotEqual(out, packet())
    return w, r, c, cc, cs

  def test_default_disabled_no_packet_change_or_io(self):
    w, r, c, cc, cs = fixture()
    w.publish = lambda _: self.fail('inert runtime audit I/O')
    self.assertEqual(r.apply(c, cc, cs, 1_000_000_000, packet()), packet())
    self.assertEqual(r.runner.state, 'disabled')

  def test_moving_acceptance_rejected(self):
    for field, value in [('vEgo', 1.0), ('gearShifter', structs.CarState.GearShifter.drive), ('canValid', False)]:
      w, r, c, cc, cs = fixture()
      setattr(cs.out, field, value)
      w.permits.put_nowait(permit())
      r.apply(c, cc, cs, 1_000_000_000, packet())
      self.assertEqual(r.runner.state, 'disabled')

  def test_cancel_priority_and_no_rearm(self):
    w, r, c, cc, cs = self.active()
    w.cancel_ns = 1_180_000_000
    w.cancel_generation += 1
    w.permits.put_nowait(permit())
    self.assertEqual(frame(r, c, cc, cs, 1_180_000_000), packet())
    self.assertEqual(r.runner.state, 'cancelled')
    self.assertEqual(frame(r, c, cc, cs, 1_200_000_000), packet())

  def test_audit_backpressure_cancels_before_output(self):
    w, r, c, cc, cs = self.active()
    w.publish = lambda _: False
    self.assertEqual(frame(r, c, cc, cs, 1_180_000_000), packet())
    self.assertEqual(r.runner.state, 'cancelled')

  def test_brief_fault_latched_even_when_latest_healthy(self):
    w, r, c, cc, cs = self.active()
    w.health_generation += 1
    self.assertEqual(frame(r, c, cc, cs, 1_180_000_000), packet())
    self.assertEqual(r.runner.state, 'cancelled')

  def test_worker_source_and_audit_loss(self):
    for key in ['healthy', 'source', 'audit']:
      w, r, c, cc, cs = self.active()
      w.snapshot = dict(w.snapshot, **{key: False})
      self.assertEqual(frame(r, c, cc, cs, 1_180_000_000), packet())
      self.assertEqual(r.runner.state, 'cancelled')

  def test_stale_worker(self):
    w, r, c, cc, cs = self.active()
    r.feed(state_ns=1_500_000_000, state_valid=True)
    self.assertEqual(r.apply(c, cc, cs, 1_500_000_000, packet()), packet())
    self.assertEqual(r.runner.state, 'cancelled')

  def test_concurrent_cancel_never_releases_changed_frame(self):
    w, r, c, cc, cs = self.active()

    def publish(_):
      w.cancel_generation += 1
      return True

    w.publish = publish
    self.assertEqual(frame(r, c, cc, cs, 1_180_000_000), packet())
    self.assertEqual(r.runner.state, 'cancelled')


class TestOperator(unittest.TestCase):
  def test_arm_needs_explicit_enable_fresh_ready_and_condition(self):
    now = 1_000_000_000
    written = []
    p = NS(
      get=lambda _: __import__('json').dumps(
        {'t_ns': now, 'io_ns': now, 'ready': True, 'parked': True, 'state': 'disabled', 'session_id': 's', 'source_id': 'x'}
      ),
      put=lambda *args, **kw: written.append(args),
    )
    with self.assertRaises(ValueError):
      command(p, 'arm', condition='B', clock=lambda: now)
    command(p, 'arm', enable=True, condition='B', clock=lambda: now)
    self.assertEqual(len(written), 1)
    with self.assertRaises(ValueError):
      command(p, 'arm', enable=True, condition='B', clock=lambda: now + 1_000_000_000)

  def test_cancel_needs_no_status_or_enable(self):
    p = NS(get=lambda _: self.fail('cancel must not need status'), put=lambda *a, **kw: None)
    self.assertTrue(command(p, 'cancel')['cancel_requested'])


import json
from openpilot.selfdrive.car.prediction_pilot import PilotWorker, PERMIT, CANCEL, STATUS


def test_worker_drains_brief_fault_and_audits_before_delivery(tmp_path, monkeypatch):
  import openpilot.cereal.messaging as messaging

  e1 = messaging.new_message('selfdriveState')
  e1.valid = True
  e1.logMonoTime = 1_000_000_000
  e1.selfdriveState.active = False
  e1.selfdriveState.alertType = 'controlsMismatch/immediateDisable'
  e2 = messaging.new_message('selfdriveState')
  e2.valid = True
  e2.logMonoTime = 1_000_000_000
  p = messaging.new_message('pandaStates', 1)
  p.valid = True
  p.logMonoTime = 1_000_000_000
  p.pandaStates[0].safetyModel = 'ford'
  p.pandaStates[0].safetyParam = 11
  e = messaging.new_message('onroadEvents', 0)
  e.valid = True
  e.logMonoTime = 1_000_000_000
  messages = {'selfdriveState': [e1, e2], 'pandaStates': [p], 'onroadEvents': [e]}
  monkeypatch.setattr(messaging, 'sub_sock', lambda topic: topic)
  monkeypatch.setattr(messaging, 'drain_sock', lambda topic: messages.pop(topic, []))
  file = tmp_path / 'source.py'
  file.write_text('source')
  values = {PERMIT: json.dumps(permit()).encode(), CANCEL: b'cancel'}
  written = []
  params = NS(get=lambda key: values.get(key), put=lambda *a, **kw: written.append(a))
  w = PilotWorker(params, [file], tmp_path / 'audit', clock=lambda: 1_000_000_000)

  class Stop:
    stopped = False

    def is_set(self):
      return self.stopped

    def wait(self, _):
      self.stopped = True

  w.stop = Stop()
  w.run()
  assert w.health_generation == 1 and w.snapshot['healthy']
  assert not w.snapshot['mismatch'] and w.cancel_generation == 1
  assert w.permits.get_nowait()['trial_id'] == 'trial'
  assert written[0][0] == STATUS
  journal = next((tmp_path / 'audit').glob('*.jsonl')).read_text()
  assert 'permit_received' in journal and w.snapshot['audit']


def test_worker_audit_failure_stays_unhealthy(tmp_path, monkeypatch):
  import openpilot.cereal.messaging as messaging

  monkeypatch.setattr(messaging, 'sub_sock', lambda topic: topic)
  file = tmp_path / 'source.py'
  file.write_text('source')
  bad = tmp_path / 'not-directory'
  bad.write_text('occupied')
  w = PilotWorker(NS(), [file], bad, clock=lambda: 1_000_000_000)
  w.run()
  assert not w.snapshot['audit'] and not w.snapshot['healthy']
  assert w.terminal['event'] == 'worker_failed'


def test_concurrent_source_audit_and_worker_loss_suppress_output():
  for key in ['source', 'audit', 'healthy', 'allowed', 'active']:
    test = TestRuntime()
    w, r, c, cc, cs = test.active()

    def publish(_, key=key, w=w):
      w.snapshot = dict(w.snapshot, **{key: False})
      return True

    w.publish = publish
    assert frame(r, c, cc, cs, 1_180_000_000) == packet()
    assert r.runner.state == 'cancelled'


def test_health_deadline_is_checked_at_callback_not_worker_time():
  test = TestRuntime()
  w, r, c, cc, cs = test.active()
  w.snapshot = dict(w.snapshot, valid_until=1_170_000_000)
  r.feed(state_ns=1_180_000_000, state_valid=True)
  assert r.apply(c, cc, cs, 1_180_000_000, packet()) == packet()
  assert r.runner.state == 'cancelled'


def test_cancel_plus_stale_permit_retains_both_audit_events():
  test = TestRuntime()
  w, r, c, cc, cs = test.active()
  w.cancel_ns = 1_180_000_000
  w.cancel_generation += 1
  w.permits.put_nowait(permit())
  assert frame(r, c, cc, cs, 1_180_000_000) == packet()
  assert [e['event'] for e in w.notes] == ['cancel', 'request_rejected']
  assert w.notes[0]['reason'] == 'operator_cancel'


def test_terminal_audit_queue_overflow_fails_closed(tmp_path):
  file = tmp_path / 'source.py'
  file.write_text('source')
  w = PilotWorker(NS(), [file], tmp_path / 'audit')
  for i in range(65):
    w.note({'event': 'cancel', 'i': i})
  assert w.audit_failed and not w.snapshot['audit']
  assert w.terminal_records.qsize() == 64


def test_actual_card_feeds_source_timestamp_before_controller(monkeypatch):
  from collections import defaultdict
  import openpilot.selfdrive.car.card as card
  from openpilot.selfdrive.car.tests.test_longitudinal_pose import command as control
  from opendbc.car.ford.carcontroller import CarController
  from opendbc.car.ford.interface import CarInterface
  from opendbc.car.ford.values import DBC

  monkeypatch.setattr(card, 'REPLAY', True)
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.openpilotLongitudinalControl = True
  c = CarController(DBC[cp.carFingerprint], cp)
  c.longitudinal_coordination_enabled = True
  now = 1_000_000_000
  state = NS(
    out=structs.CarState(vEgo=0.0, canValid=True),
    longitudinal_brake_torque=Torque(1.0, 0.0),
    buttons_stock_values=defaultdict(int),
    lkas_status_stock_values=defaultdict(int),
    acc_tja_status_stock_values=defaultdict(int),
  )
  order = []
  runtime = NS(feed=lambda **kw: order.append(('feed', kw)))

  def apply(cc, stamp):
    assert order[0][0] == 'feed'
    order.append(('apply', stamp))
    return c.update(cc, state, stamp)

  h = NS(
    initialized_prev=True,
    sm=NS(all_alive=lambda *_: True, all_checks=lambda *_: True),
    tracking_trial=None,
    longitudinal_coordination_enabled=True,
    coordination_pose_refresher=None,
    prediction_pilot=runtime,
    pilot_state_ns=now - 10_000_000,
    pilot_state_valid=True,
    CI=NS(CC=c, apply=apply),
    can_log_mono_time=now,
    pm=NS(send=lambda *a: None),
  )
  card.Car.controls_update(h, state.out, control().as_reader())
  assert order == [('feed', {'state_ns': now - 10_000_000, 'state_valid': True}), ('apply', now)]
  assert c.prediction_pilot is None


def test_real_params_operator_command_is_written_before_return(tmp_path):
  from openpilot.common.params import Params

  p = Params(str(tmp_path / 'params'))
  now = 1_000_000_000
  p.put(
    STATUS,
    json.dumps({'t_ns': now, 'io_ns': now, 'ready': True, 'parked': True, 'state': 'disabled', 'session_id': 'session', 'source_id': 'source'}),
    block=True,
  )
  command(p, 'arm', enable=True, condition='B', clock=lambda: now)
  assert json.loads(p.get(PERMIT))['condition'] == 'B'
  command(p, 'cancel', clock=lambda: now)
  assert json.loads(p.get(CANCEL))['issued_ns'] == now
