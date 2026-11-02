"""Exercise the real health-publisher loop with private Params, never hardware I/O."""
import fcntl
import gc
import io
import queue
import threading
import time
from types import SimpleNamespace as NS
from pathlib import Path

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.system.hardware import hardwared as h


class FakeClock:
  now = 1000.
  def monotonic(self):
    return self.now


def publisher(monkeypatch, tmp_path, *, cycles=124, stall=False, transitions=False, lock_seconds=.25, post_stall_interval=0.,
              shutdown=False, thermal_stall=0., thermal_stall_at=0):
  p = Params(str(tmp_path / 'params'))
  p.put('HasAcceptedTerms', h.terms_version, block=True)
  p.put('CompletedTrainingVersion', h.training_version, block=True)
  p.put('UptimeOffroad', 0., block=True)
  p.put('UptimeOnroad', 0., block=True)
  clock = FakeClock()
  records, writes, bool_writes, lock_threads = [], [], [], []
  lock = Path(p.get_param_path()).parent / '.lock'

  class Proxy:
    def get(self, *a, **kw):
      return p.get(*a, **kw)
    def get_bool(self, *a, **kw):
      return p.get_bool(*a, **kw)
    def put(self, key, value, block=False):
      writes.append((key, value, block))
      return p.put(key, value, block=block)
    def put_bool(self, key, value, block=False):
      bool_writes.append((key, value, block))
      return p.put_bool(key, value, block=block)

  ps = log.PandaState.new_message(pandaType='cuatro', harnessStatus='normal', ignitionLine=True)
  peripheral = log.PeripheralState.new_message()
  sd = log.SelfdriveState.new_message(enabled=False)

  class SM(dict):
    frame = 0
    updated = {'pandaStates': True, 'selfdriveState': True}
    alive = {'chestnutState': False, 'gpsLocationExternal': False}
    valid = {'chestnutState': False}
    recv_time = {'pandaStates': clock.now}
    def update(self, _):
      if stall and len(records) >= 121 and post_stall_interval:
        time.sleep(post_stall_interval)
      self.frame += 5
      clock.now += .5
      if transitions:
        ps.ignitionLine = 30 <= self.frame // 5 < 160
        sd.enabled = 40 <= self.frame // 5 < 140
      self.recv_time['pandaStates'] = clock.now
  sm = SM(pandaStates=[ps], peripheralState=peripheral, selfdriveState=sd, chestnutState=None)

  def hold(held):
    with lock.open('a') as f:
      fcntl.flock(f, fcntl.LOCK_EX)
      held.set()
      time.sleep(lock_seconds)
      fcntl.flock(f, fcntl.LOCK_UN)

  class PM:
    def send(self, topic, msg):
      assert topic == 'deviceState'
      records.append((time.perf_counter(), msg.deviceState.to_dict()))
      if stall and len(records) == 121:
        held = threading.Event()
        t = threading.Thread(target=hold, args=(held,))
        lock_threads.append(t)
        t.start()
        assert held.wait(1)

  def thermal_msg():
    if thermal_stall and len(records) == thermal_stall_at:
      time.sleep(thermal_stall)
    return log.DeviceState.new_message(cpuTempC=[30.] * 8, gpuTempC=[30.] * 2, memoryTempC=30., pmicTempC=[30.])
  hw = NS(initialize_hardware=lambda: None, get_device_type=lambda: 'mici', get_thermal_config=lambda: NS(get_msg=thermal_msg),
          get_gpu_usage_percent=lambda: 0, get_screen_brightness=lambda: 50, booted=lambda: True, set_power_save=lambda _: None,
          get_current_power_draw=lambda: 0., get_som_power_draw=lambda: 0.)
  monkeypatch.setattr(h, 'HARDWARE', hw)
  monkeypatch.setattr(h, 'PC', True)
  monkeypatch.setattr(h, 'Params', lambda: Proxy())
  monkeypatch.setattr(h, 'time', clock)
  monkeypatch.setattr(h, 'get_usb_state', list)
  monkeypatch.setattr(h, 'set_usb_state', lambda *a: None)
  monkeypatch.setattr(h, 'get_short_branch', lambda: 'local-fixture')
  monkeypatch.setattr(h, 'get_available_percent', lambda **kw: 100.)
  monkeypatch.setattr(h, 'set_offroad_alert_if_changed', lambda *a, **kw: None)
  monkeypatch.setattr(h, 'open', lambda *a, **kw: io.StringIO(), raising=False)
  monkeypatch.setattr(h, 'LinuxSystemStats', lambda: NS(memory_usage_percent=lambda: 50., cpu_usage_percent=lambda: [0.] * 8))
  monkeypatch.setattr(h, 'PowerMonitoring', lambda: NS(calculate=lambda *a: None, get_power_used=lambda: 0,
                                                     get_car_battery_capacity=lambda: 0, should_shutdown=lambda *a: shutdown))
  monkeypatch.setattr(h, 'FanController', lambda _: NS(update=lambda *a: 0))
  monkeypatch.setattr(h, 'Chestnut', lambda: NS(update=lambda *a: None, failed=False, flashing=False))
  monkeypatch.setattr(h, 'ChestnutStatus', lambda: NS(update=lambda *a: None))
  monkeypatch.setattr(h, 'cloudlog', NS(event=lambda *a, **kw: None, warning=lambda *a: None, error=lambda *a: None, exception=lambda *a: None))
  monkeypatch.setattr(h.messaging, 'SubMaster', lambda *a, **kw: sm)
  monkeypatch.setattr(h.messaging, 'PubMaster', lambda *a, **kw: PM())
  h.hardware_thread(NS(is_set=lambda: len(records) >= cycles), queue.Queue())
  for t in lock_threads:
    t.join(1)
    assert not t.is_alive()
  deadline = time.monotonic() + 3
  expected = {k: v for k, v, _ in writes if k in ['UptimeOffroad', 'UptimeOnroad']}
  while time.monotonic() < deadline and any(p.get(k) != v for k, v in expected.items()):
    time.sleep(.005)
  assert all(p.get(k) == v for k, v in expected.items())
  return records, writes, bool_writes


def test_periodic_uptime_does_not_block_health_publication(monkeypatch, tmp_path):
  records, writes, bool_writes = publisher(monkeypatch, tmp_path, stall=True)
  assert records[121][0] - records[120][0] < .1
  assert all(not block for k, _, block in writes if k in ['UptimeOffroad', 'UptimeOnroad'])
  assert all(block for k, _, block in bool_writes if k == 'IsEngaged')
  assert all(block for k, _, block in writes if k == 'LastOffroadStatusPacket')


def test_uptime_values_and_ignition_transitions_are_retained(monkeypatch, tmp_path):
  records, writes, _ = publisher(monkeypatch, tmp_path, cycles=244, transitions=True)
  assert any(d['started'] for _, d in records)
  assert not records[0][1]['started'] and not records[-1][1]['started']
  for key in ['UptimeOffroad', 'UptimeOnroad']:
    values = [v for k, v, _ in writes if k == key]
    assert len(values) == 3 and values == sorted(values) and values[-1] > 0


def test_normal_params_shutdown_flushes_pending_uptime(tmp_path):
  root = str(tmp_path / 'shutdown')
  writer = Params(root)
  writer.put('UptimeOnroad', 0., block=True)
  observer = Params(root)
  lock = Path(writer.get_param_path()).parent / '.lock'
  held = threading.Event()
  def holder():
    with lock.open('a') as f:
      fcntl.flock(f, fcntl.LOCK_EX)
      held.set()
      time.sleep(.25)
      fcntl.flock(f, fcntl.LOCK_UN)
  t = threading.Thread(target=holder)
  t.start()
  assert held.wait(1)
  writer.put('UptimeOnroad', 42., block=False)
  start = time.perf_counter()
  del writer
  gc.collect()
  assert time.perf_counter() - start >= .15
  assert observer.get('UptimeOnroad') == 42.
  t.join(1)
  assert not t.is_alive()


def test_shutdown_and_engagement_writes_remain_synchronous(monkeypatch, tmp_path):
  _, _, writes = publisher(monkeypatch, tmp_path, cycles=164, transitions=True, shutdown=True)
  engaged = [(v, block) for k, v, block in writes if k == 'IsEngaged']
  shutdown = [(v, block) for k, v, block in writes if k == 'DoShutdown']
  assert (True, True) in engaged and (False, True) in engaged
  assert shutdown and all(v and block for v, block in shutdown)
