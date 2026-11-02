"""Exercise the actual worker startup and its failure boundary without vehicle I/O."""
import threading
from types import SimpleNamespace as NS
import pytest
from openpilot.common import realtime
from openpilot.selfdrive.car.prediction_pilot import PilotWorker


def worker(tmp_path):
  source = tmp_path / 'source.py'
  source.write_text('source')
  w = PilotWorker(NS(get=lambda _: None, put=lambda *a, **kw: None), [source], tmp_path / 'audit', clock=lambda: 1_000_000_000)

  class Stop:
    stopped = False
    def is_set(self):
      return self.stopped
    def wait(self, _):
      self.stopped = True
  w.stop = Stop()
  return w


def test_worker_isolates_its_own_thread_before_ipc_and_recurring_io(tmp_path, monkeypatch):
  import openpilot.cereal.messaging as messaging
  calls = []
  caller = threading.get_ident()
  monkeypatch.setattr(realtime, 'drop_realtime', lambda: calls.append(('drop', threading.get_ident())))
  monkeypatch.setattr(realtime, 'set_core_affinity', lambda cores: calls.append(('affinity', threading.get_ident(), cores)))
  def sub(topic):
    calls.append(('ipc', threading.get_ident()))
    return topic
  monkeypatch.setattr(messaging, 'sub_sock', sub)
  monkeypatch.setattr(messaging, 'drain_sock', lambda _: [])
  w = worker(tmp_path)
  w.start()
  w.thread.join(2)
  assert not w.thread.is_alive()
  assert [x[0] for x in calls[:2]] == ['drop', 'affinity']
  assert calls[1][2] == [0, 1, 2, 3]
  assert all(x[1] != caller for x in calls)
  assert len([x for x in calls if x[0] == 'ipc']) == 3
  assert (tmp_path / 'audit').is_dir()


@pytest.mark.parametrize('failing', ['drop_realtime', 'set_core_affinity'])
def test_isolation_failure_is_closed_before_ipc_or_audit_io(tmp_path, monkeypatch, failing):
  import openpilot.cereal.messaging as messaging
  monkeypatch.setattr(realtime, 'drop_realtime', lambda: None)
  monkeypatch.setattr(realtime, 'set_core_affinity', lambda _: None)
  def fail(*args):
    raise OSError('scheduler unavailable')
  monkeypatch.setattr(realtime, failing, fail)
  monkeypatch.setattr(messaging, 'sub_sock', lambda _: pytest.fail('IPC before isolation'))
  w = worker(tmp_path)
  w.run()
  assert not w.snapshot['healthy'] and not w.snapshot['audit']
  assert w.terminal == {'event': 'worker_failed', 'reason': 'OSError'}
  assert not (tmp_path / 'audit').exists()
