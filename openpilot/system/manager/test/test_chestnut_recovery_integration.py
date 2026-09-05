import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.events import Events, ET
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.system.manager.chestnut_recovery import ChestnutRecovery, RecoveryInput
from openpilot.system.manager.process import PythonProcess, ensure_running
from openpilot.system.manager.test.test_chestnut_recovery import Params


class TestRecoveryInterlock(unittest.TestCase):
  def setUp(self):
    # Exercise the real event/state/acknowledgment code without sockets or
    # starting the control loop. Unrelated sampling and alert I/O are mocked.
    self.sd = SelfdriveD.__new__(SelfdriveD)
    self.sd.params = Params()
    self.sd.params.put('ChestnutRecoveryRequest', 7)
    self.sd.CP = SimpleNamespace(passive=False)
    self.sd.initialized = True
    self.sd.enabled = self.sd.active = False
    self.sd.big_model_loading = False
    self.sd.big_model_ready_t = 0.
    self.sd.events = Events()
    self.sd.state_machine = StateMachine()
    self.sd.sm = Mock()
    self.sd.sm.all_checks.return_value = True
    self.sd.data_sample = Mock()
    self.sd.update_alerts = Mock()
    self.sd.publish_selfdriveState = Mock()
    self.sd.update_events = self.update_events

  def update_events(self, cs):
    self.sd.events.clear()
    self.sd.update_model_loading()
    self.sd.events.add(log.OnroadEvent.EventName.pcmEnable)

  def test_simultaneous_enable_is_blocked_before_acknowledgment(self):
    self.sd.step()
    self.assertFalse(self.sd.enabled)
    self.assertTrue(self.sd.events.contains(ET.NO_ENTRY))
    self.assertEqual(self.sd.params.get('ChestnutRecoveryAck'), 7)

  def test_no_acknowledgment_from_enabled_states(self):
    for state in ('enabled', 'preEnabled', 'softDisabling', 'overriding'):
      with self.subTest(state=state):
        self.sd.state_machine.state = getattr(log.SelfdriveState.OpenpilotState, state)
        self.sd.state_machine.soft_disable_timer = 100
        self.sd.params.put('ChestnutRecoveryAck', 0)
        self.sd.step()
        self.assertTrue(self.sd.enabled)
        self.assertEqual(self.sd.params.get('ChestnutRecoveryAck'), 0)

  def test_no_acknowledgment_before_initialization_or_in_passive_mode(self):
    for initialized, passive in ((False, False), (True, True)):
      self.sd.initialized, self.sd.CP.passive = initialized, passive
      self.sd.step()
      self.assertIsNone(self.sd.params.get('ChestnutRecoveryAck'))

  def test_recovery_never_enables_startup_fault_suppression(self):
    self.sd.big_model_loading = True
    with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic', return_value=100.):
      for loading, request in ((True, 7), (False, 7), (False, 0)):
        self.sd.params.put_bool('ChestnutLoading', loading)
        self.sd.params.put('ChestnutRecoveryRequest', request)
        self.sd.update_model_loading()
        self.assertFalse(self.sd.big_model_loading)
        self.assertLess(self.sd.big_model_ready_t + 5., 100.)

  def test_initial_startup_retains_loading_grace(self):
    self.sd.params.put('ChestnutRecoveryRequest', 0)
    self.sd.params.put_bool('ChestnutLoading', True)
    self.sd.update_model_loading()
    self.assertTrue(self.sd.big_model_loading)
    self.sd.params.put_bool('ChestnutLoading', False)
    with patch('openpilot.selfdrive.selfdrived.selfdrived.time.monotonic', return_value=100.):
      self.sd.update_model_loading()
    self.assertEqual(self.sd.big_model_ready_t, 100.)

  def test_readiness_requires_model_and_odometry_health(self):
    self.sd.sm.all_checks.return_value = False
    self.sd.step()
    self.assertEqual(self.sd.params.get('ChestnutRecoveryReady'), 0)
    self.sd.sm.all_checks.assert_called_with(['modelV2', 'cameraOdometry'])
    self.sd.sm.all_checks.return_value = True
    self.sd.step()
    self.assertEqual(self.sd.params.get('ChestnutRecoveryReady'), 7)
    self.sd.sm.all_checks.return_value = False
    self.sd.step()
    self.assertEqual(self.sd.params.get('ChestnutRecoveryReady'), 0)


class TestRecoveryProcessOwnership(unittest.TestCase):
  def setUp(self):
    self.params = Params()
    self.process = PythonProcess('modeld', 'unused', lambda started, params, cp: started)
    self.process.proc = SimpleNamespace(exitcode=None)
    self.process.signal = Mock()
    self.process.start = Mock(side_effect=self.start)
    self.recovery = ChestnutRecovery(self.params, self.process)
    self.state = RecoveryInput(started=True, allowed=True, safe_to_restart=True, power_ready=True)
    self.recovery.update(self.state, 0.)
    self.params.put_bool('ChestnutActive', False)
    self.recovery.update(self.state, 1.)
    self.recovery.update(self.state, 32.)
    self.params.put('ChestnutRecoveryAck', self.params.get('ChestnutRecoveryRequest'))

  def start(self):
    self.assertIsNone(self.process.proc, 'replacement must not overlap the previous process')
    self.process.proc = SimpleNamespace(exitcode=None)

  def tick(self, now):
    owned = self.recovery.update(self.state, now)
    procs = {} if owned else {'modeld': self.process}
    ensure_running(procs.values(), self.state.started, self.params, SimpleNamespace())
    return owned

  @patch('openpilot.system.manager.process.join_process', side_effect=AssertionError('recovery must not join in manager'))
  def test_nonblocking_shutdown_reaps_before_start(self, join):
    self.assertTrue(self.tick(33.))
    self.assertTrue(self.tick(34.))
    self.assertTrue(self.tick(39.))
    self.process.start.assert_not_called()
    self.process.proc.exitcode = -9
    self.assertFalse(self.tick(40.))
    self.process.start.assert_called_once()
    self.assertFalse(self.process.shutting_down)

  @patch('openpilot.system.manager.process.join_process', side_effect=AssertionError('offroad cancellation must not join'))
  def test_offroad_during_shutdown_retains_ownership_without_restart(self, join):
    self.assertTrue(self.tick(33.))
    self.state = replace(self.state, started=False)
    self.assertTrue(self.tick(34.))
    self.assertTrue(self.tick(39.))
    self.process.proc.exitcode = -9
    self.assertFalse(self.tick(40.))
    self.process.start.assert_not_called()
    self.assertEqual(self.params.get('ChestnutRecoveryRequest'), 0)


if __name__ == '__main__':
  unittest.main()
