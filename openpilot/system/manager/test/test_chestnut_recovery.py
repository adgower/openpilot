import unittest
from dataclasses import replace
from types import SimpleNamespace

from openpilot.system.manager.chestnut_recovery import ChestnutRecovery, RecoveryInput


class Params:
  def __init__(self):
    self.values = {'ChestnutActive': True}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.get(key))

  def put(self, key, value, **kwargs):
    self.values[key] = value

  put_bool = put

  def remove(self, key):
    self.values.pop(key, None)


class Process:
  def __init__(self):
    self.proc = SimpleNamespace(exitcode=None)
    self.stops = 0
    self.signals = []

  def stop(self, block=True):
    assert not block, 'manager must not block on GPU shutdown'
    self.stops += 1
    if self.proc is not None and self.proc.exitcode is not None:
      self.proc = None

  def signal(self, sig):
    self.signals.append(sig)


class TestChestnutRecovery(unittest.TestCase):
  def setUp(self):
    self.params, self.process = Params(), Process()
    self.recovery = ChestnutRecovery(self.params, self.process)
    self.safe = RecoveryInput(started=True, allowed=True, safe_to_restart=True, power_ready=True)
    self.recovery.update(self.safe, 0.)
    self.params.put_bool('ChestnutActive', False)
    self.recovery.update(self.safe, 1.)

  def request(self):
    self.recovery.update(self.safe, 32.)
    request = self.params.get('ChestnutRecoveryRequest')
    self.assertGreater(request, 0)
    self.assertEqual(self.process.stops, 0)
    return request

  def restart(self):
    request = self.request()
    self.params.put('ChestnutRecoveryAck', request)
    self.assertTrue(self.recovery.update(self.safe, 33.))
    self.process.proc.exitcode = 0
    self.assertFalse(self.recovery.update(self.safe, 34.))
    # ensure_running starts the replacement process after suppression is removed.
    self.process.proc = SimpleNamespace(exitcode=None)

  def healthy(self, big, t):
    self.params.put('ChestnutRecoveryReady', self.params.get('ChestnutRecoveryRequest'))
    self.params.put_bool('ChestnutActive', big)
    self.params.put_bool('ChestnutLoading', False)
    self.params.put('ChestnutRecoveryReady', self.params.get('ChestnutRecoveryRequest'))
    self.recovery.update(replace(self.safe, model_valid=True, model_big=big, model_frame=10, model_time=t), t)
    self.recovery.update(replace(self.safe, model_valid=True, model_big=big, model_frame=50, model_time=t+2.), t+2.)

  def test_requires_acknowledged_request_before_stopping(self):
    request = self.request()
    self.params.put('ChestnutRecoveryAck', request-1)
    self.recovery.update(self.safe, 33.)
    self.assertEqual(self.process.stops, 0)
    self.params.put('ChestnutRecoveryAck', request)
    self.assertTrue(self.recovery.update(self.safe, 34.))
    self.assertGreater(self.process.stops, 0)

  def test_engagement_or_stale_inputs_cancel_request(self):
    self.request()
    self.recovery.update(replace(self.safe, safe_to_restart=False), 33.)
    self.assertEqual(self.process.stops, 0)
    self.assertEqual(self.params.get('ChestnutRecoveryRequest'), 0)

  def test_does_not_retry_without_stable_power_or_disabled_vehicle(self):
    for field in ['safe_to_restart', 'power_ready', 'allowed']:
      with self.subTest(field=field):
        self.recovery.update(replace(self.safe, **{field: False}), 31.)
        self.recovery.update(self.safe, 32.)
        self.assertFalse(self.params.get('ChestnutRecoveryRequest'))

  def test_success_keeps_interlock_until_valid_advancing_output(self):
    self.restart()
    self.params.put_bool('ChestnutActive', True)
    self.params.put_bool('ChestnutLoading', False)
    self.params.put('ChestnutRecoveryReady', self.params.get('ChestnutRecoveryRequest'))
    self.recovery.update(self.safe, 35.)
    self.assertGreater(self.params.get('ChestnutRecoveryRequest'), 0)
    self.healthy(True, 36.)
    self.assertEqual(self.params.get('ChestnutRecoveryRequest'), 0)

  def test_stalled_output_does_not_release_interlock(self):
    self.restart()
    self.params.put_bool('ChestnutActive', True)
    self.params.put_bool('ChestnutLoading', False)
    self.params.put('ChestnutRecoveryReady', self.params.get('ChestnutRecoveryRequest'))
    same = replace(self.safe, model_valid=True, model_big=True, model_frame=10, model_time=35.)
    self.recovery.update(same, 35.)
    self.recovery.update(replace(same, model_time=40.), 40.)
    self.assertGreater(self.params.get('ChestnutRecoveryRequest'), 0)

  def test_timeout_reaps_loader_and_starts_small_only(self):
    self.restart()
    self.assertTrue(self.recovery.update(self.safe, 125.))
    self.assertTrue(self.params.get_bool('ChestnutRecoverySkipBig'))
    self.process.proc.exitcode = -9
    self.assertFalse(self.recovery.update(self.safe, 126.))
    self.process.proc = SimpleNamespace(exitcode=None)
    self.healthy(False, 127.)
    self.assertEqual(self.params.get('ChestnutRecoveryRequest'), 0)

  def test_failed_big_load_restarts_without_overlapping_loader(self):
    self.restart()
    self.params.put_bool('ChestnutActive', False)
    self.params.put_bool('ChestnutLoading', False)
    self.params.put('ChestnutRecoveryReady', self.params.get('ChestnutRecoveryRequest'))
    self.assertTrue(self.recovery.update(self.safe, 40.))
    self.assertTrue(self.params.get_bool('ChestnutRecoverySkipBig'))
    self.assertIsNotNone(self.process.proc)
    self.assertTrue(self.recovery.update(self.safe, 46.))
    self.assertTrue(self.process.signals)

  def test_ignition_off_resets_budget_and_interlock(self):
    self.restart()
    self.recovery.update(replace(self.safe, started=False), 36.)
    self.assertEqual(self.params.get('ChestnutRecoveryRequest'), 0)
    self.assertFalse(self.params.get_bool('ChestnutRecoverySkipBig'))
    self.assertEqual(self.recovery.attempts, 0)

  def test_stale_or_pre_restart_output_does_not_release_interlock(self):
    self.restart()
    self.params.put_bool('ChestnutActive', True)
    self.params.put_bool('ChestnutLoading', False)
    self.params.put('ChestnutRecoveryReady', self.params.get('ChestnutRecoveryRequest'))
    for stamp in (33., 34., 100.):
      self.recovery.update(replace(self.safe, model_valid=True, model_big=True, model_frame=10, model_time=stamp), 35.)
      self.recovery.update(replace(self.safe, model_valid=True, model_big=True, model_frame=50, model_time=stamp), 40.)
      self.assertGreater(self.params.get('ChestnutRecoveryRequest'), 0)

  def test_request_timeout_requires_new_acknowledgment(self):
    old_request = self.request()
    self.recovery.update(self.safe, 38.)
    self.assertEqual(self.process.stops, 0)
    self.params.put('ChestnutRecoveryAck', old_request)
    self.recovery.update(self.safe, 39.)
    self.recovery.update(self.safe, 70.)
    self.assertGreater(self.params.get('ChestnutRecoveryRequest'), old_request)
    self.params.put('ChestnutRecoveryAck', old_request)
    self.recovery.update(self.safe, 71.)
    self.assertEqual(self.process.stops, 0)

  def test_power_loss_cancels_acknowledged_request(self):
    request = self.request()
    self.params.put('ChestnutRecoveryAck', request)
    self.recovery.update(replace(self.safe, power_ready=False), 33.)
    self.assertEqual(self.process.stops, 0)
    self.assertEqual(self.params.get('ChestnutRecoveryRequest'), 0)

  def test_failed_small_start_keeps_interlock_without_restart_loop(self):
    self.restart()
    self.recovery.update(self.safe, 125.)
    self.process.proc.exitcode = -9
    self.recovery.update(self.safe, 126.)
    self.process.proc = SimpleNamespace(exitcode=1)
    self.recovery.update(self.safe, 217.)
    stops = self.process.stops
    self.recovery.update(self.safe, 1000.)
    self.assertGreater(self.params.get('ChestnutRecoveryRequest'), 0)
    self.assertEqual(self.process.stops, stops)

  def test_no_automatic_retry_after_initial_load_failure(self):
    recovery = ChestnutRecovery(self.params, self.process)
    recovery.update(self.safe, 0.)
    recovery.update(self.safe, 60.)
    self.assertFalse(self.params.get('ChestnutRecoveryRequest'))

  def test_invalid_output_restarts_validation_window(self):
    self.restart()
    self.params.put_bool('ChestnutActive', True)
    self.params.put_bool('ChestnutLoading', False)
    self.params.put('ChestnutRecoveryReady', self.params.get('ChestnutRecoveryRequest'))
    self.recovery.update(replace(self.safe, model_valid=True, model_big=True, model_frame=10, model_time=35.), 35.)
    self.recovery.update(self.safe, 36.)
    self.recovery.update(replace(self.safe, model_valid=True, model_big=True, model_frame=50, model_time=37.), 37.)
    self.assertGreater(self.params.get('ChestnutRecoveryRequest'), 0)

  def test_sparse_outputs_require_selfdrived_health_validation(self):
    self.restart()
    self.params.put_bool('ChestnutActive', True)
    self.params.put_bool('ChestnutLoading', False)
    self.params.put('ChestnutRecoveryReady', self.params.get('ChestnutRecoveryRequest'))
    self.params.put('ChestnutRecoveryReady', 0)
    # Camera IDs advance even when model output is too infrequent. Only the
    # high-rate subscriber can verify the model publication frequency.
    self.recovery.update(replace(self.safe, model_valid=True, model_big=True, model_frame=10, model_time=35.), 35.)
    self.recovery.update(replace(self.safe, model_valid=True, model_big=True, model_frame=50, model_time=37.), 37.)
    self.assertGreater(self.params.get('ChestnutRecoveryRequest'), 0)
    self.healthy(True, 38.)
    self.assertEqual(self.params.get('ChestnutRecoveryRequest'), 0)

  def test_limits_retries_after_recurrent_failures(self):
    self.restart()
    self.healthy(True, 35.)
    self.params.put_bool('ChestnutActive', False)
    self.recovery.update(self.safe, 40.)
    self.recovery.update(self.safe, 71.)
    self.assertGreater(self.params.get('ChestnutRecoveryRequest'), 0)
    self.params.put('ChestnutRecoveryAck', self.params.get('ChestnutRecoveryRequest'))
    self.recovery.update(self.safe, 72.)
    self.process.proc.exitcode = 0
    self.recovery.update(self.safe, 73.)
    self.process.proc = SimpleNamespace(exitcode=None)
    self.healthy(True, 74.)
    self.params.put_bool('ChestnutActive', False)
    self.recovery.update(self.safe, 80.)
    self.recovery.update(self.safe, 150.)
    self.assertEqual(self.params.get('ChestnutRecoveryRequest'), 0)
    self.assertEqual(self.recovery.attempts, 2)


if __name__ == '__main__':
  unittest.main()
