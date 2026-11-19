"""Local-only regressions for session intent and conservative missing-ACK recovery."""
import heapq
from types import SimpleNamespace as NS
import unittest
from openpilot.selfdrive.car.vision_curve_recovery import AcknowledgedCurveSpeed as Controller
from openpilot.selfdrive.car.tests.test_forecast_handoff import road
from openpilot.selfdrive.car.vision_curve_context import VisionCurveContext as Context, ReleaseEdgeFilter as Filter
from openpilot.selfdrive.car.tests.test_vision_forecast_context import ContextFixture, CaptureCruise


class Rig:
  def __init__(self):
    self.c = Controller()
    self.t = 0.
    self.observed = 70
    self.queue = []
    self.pulses = []

  def run(self, duration, *, forecast=None, clear=True, bend=False, acceleration=0., ack=True, lane_confidence=None, **kwargs):
    for _ in range(round(duration / .01)):
      self.t = round(self.t + .01, 6)
      while self.queue and self.queue[0][0] <= self.t:
        _, delta = heapq.heappop(self.queue)
        self.observed += delta
      m = road(bend=bend)
      if lane_confidence is not None:
        m.laneLineProbs = [lane_confidence] * 4
      m.action = NS(shouldStop=False, desiredAcceleration=acceleration)
      args = {'engaged': True, 'valid': True, 'manual': False, 'override': False, 'set_speed': self.observed,
                  'speed': 70., 'accel': 0., 'model': m, 'cruise_engaged': True,
                  'forecast_mph': forecast, 'forecast_clear': clear}
      args.update(kwargs)
      button = self.c.update(self.t, **args)
      if button in ('SET+', 'SET-'):
        self.pulses.append((self.t, button))
        if ack:
          heapq.heappush(self.queue, (self.t + .05, 1 if button == 'SET+' else -1))
    return self


class RecoveryPolicyTest(unittest.TestCase):
  def test_low_lane_forecast_owned_clear_restores_original_ceiling(self):
    r = Rig().run(2., forecast=65., clear=False, lane_confidence=.1)
    self.assertEqual(r.observed, 65)
    r.run(3., clear=True, lane_confidence=.1)
    self.assertEqual(r.observed, 70)
    self.assertFalse(r.c.fault)

  def test_low_lane_raw_curve_still_blocks_forecast_restoration(self):
    r = Rig().run(2., forecast=65., clear=False, lane_confidence=.1)
    mark = len(r.pulses)
    r.run(3., clear=True, bend=True, acceleration=.5, lane_confidence=.1)
    self.assertFalse(any(button == 'SET+' for _, button in r.pulses[mark:]))
    self.assertEqual(r.c.ceiling, 70)

  def test_low_lane_missing_final_restore_ack_has_one_credit(self):
    r = Rig().run(2., forecast=69., clear=False, lane_confidence=.1)
    self.assertEqual(r.observed, 69)
    mark = len(r.pulses)
    r.run(5., clear=True, ack=False, lane_confidence=.1)
    self.assertEqual([button for _, button in r.pulses[mark:]], ['SET+'])
    self.assertEqual(r.c.transport.up_debt, 1)
    self.assertFalse(r.c.fault)

  def test_unknown_forecast_does_not_block_stronger_valid_geometry(self):
    r = Rig().run(2., forecast=65., clear=False)
    self.assertEqual(r.observed, 65)
    r.run(3., forecast=None, clear=False, bend=True)
    self.assertLessEqual(r.observed, 53)
    self.assertFalse(r.c.fault)
    self.assertEqual(r.c.ceiling, 70)

  def test_single_missing_step_does_not_permanently_fault_reduction(self):
    r = Rig().run(.12, forecast=52., clear=False, ack=False)
    r.run(4., forecast=52., clear=False)
    self.assertLessEqual(r.observed, 54)
    self.assertGreaterEqual(r.observed, 52)
    self.assertFalse(r.c.fault)
    self.assertGreater(r.c.transport.down_debt, 0)

  def test_no_ack_has_finite_pulse_budget(self):
    r = Rig().run(10., forecast=40., clear=False, ack=False)
    self.assertLessEqual(len(r.pulses), 3)
    self.assertEqual(r.observed, 70)
    self.assertFalse(r.c.fault)

  def test_brake_resume_retains_ceiling_and_never_sends_resume(self):
    r = Rig().run(2., forecast=65., clear=False)
    before = len(r.pulses)
    r.run(1., override=True, engaged=False, cruise_engaged=False)
    r.run(.5, engaged=True, cruise_engaged=True)
    self.assertEqual(len(r.pulses), before)
    self.assertEqual(r.c.ceiling, 70)
    r.run(.1, user_action='resume', manual=True)
    r.run(3., acceleration=.2)
    self.assertEqual(r.observed, 70)
    self.assertEqual({b for _, b in r.pulses}, {'SET-', 'SET+'})

  def test_held_resume_cannot_arm_a_second_automatic_resume(self):
    r = Rig().run(2., forecast=65., clear=False)
    r.run(.1, override=True, engaged=False, cruise_engaged=False)
    r.run(.1, user_action='resume', manual=True, engaged=False, cruise_engaged=False)
    r.run(.2, user_action='resume', manual=True)
    self.assertFalse(r.c.suspended)
    self.assertFalse(r.c.resume_armed)
    r.run(.1, engaged=False, cruise_engaged=False)
    mark = len(r.pulses)
    r.run(2., acceleration=.2)
    self.assertTrue(r.c.suspended)
    self.assertEqual(len(r.pulses), mark)

  def test_driver_set_replaces_old_ceiling(self):
    r = Rig().run(2., forecast=65., clear=False)
    self.assertEqual(r.c.transport.pending, 0)
    r.observed = 62
    r.run(.1, user_action='set', manual=True)
    r.run(2., acceleration=.2)
    self.assertFalse(r.c.fault)
    self.assertEqual(r.c.ceiling, 62)
    self.assertEqual(r.observed, 62)

  def test_acceleration_never_overrides_active_geometry(self):
    r = Rig().run(3., bend=True, acceleration=.5)
    self.assertLessEqual(r.observed, 53)
    mark = len(r.pulses)
    r.run(2., bend=True, acceleration=.5)
    self.assertFalse(any(b == 'SET+' for _, b in r.pulses[mark:]))

  def test_steering_pause_preserves_user_ceiling(self):
    r = Rig().run(2., forecast=65., clear=False)
    mark = len(r.pulses)
    r.run(1., steering_only=True, clear=False)
    self.assertEqual(len(r.pulses), mark)
    self.assertEqual(r.c.ceiling, 70)
    r.run(2., clear=True, acceleration=.2)
    self.assertEqual(r.observed, 70)

  def test_missing_final_plus_is_reserved_against_late_duplicate(self):
    r = Rig().run(2., forecast=69., clear=False)
    self.assertEqual(r.observed, 69)
    mark = len(r.pulses)
    r.run(5., clear=True, acceleration=.2, ack=False)
    self.assertEqual(sum(b == 'SET+' for _, b in r.pulses[mark:]), 1)
    self.assertEqual(r.c.ceiling, 70)
    self.assertEqual(r.c.transport.up_debt, 1)
    r.observed = 70
    r.run(.2, acceleration=.2)
    self.assertFalse(r.c.fault)


class ReleaseEdgesTest(unittest.TestCase):
  def test_one_neutral_after_each_press(self):
    f = Filter()
    raw = [None, '', '', 'SET-', None, None, '', '', '', 'SET+', '', '']
    actual = [f.update(x) for x in raw]
    self.assertEqual(actual, [None, None, None, 'SET-', None, None, '', None, None, 'SET+', '', None])

  def test_active_frame_identity_for_arbitrary_release_runs(self):
    f = Filter()
    raw = [None] * 100 + (['SET-'] + [''] * 1000 + ['SET+'] + [None] * 7 + [''] * 2000) * 10
    actual = [f.update(x) for x in raw]
    self.assertEqual([x for x in raw if x], [x for x in actual if x])
    self.assertEqual(sum(x == '' for x in actual), 20)

  def test_neutral_waiting_without_prior_assertion_emits_nothing(self):
    f = Filter()
    self.assertTrue(all(f.update('') is None for _ in range(1000)))


class SharedSwitchTest(unittest.TestCase):
  def fixture(self):
    f = ContextFixture()
    f.context = Context(comfort=True)
    f.context.cruise = CaptureCruise()
    return f

  def test_resume_edge_stays_resume_when_stock_engages(self):
    f = self.fixture()
    f.state.cruiseState.enabled = False
    f.parser.vl['Steering_Data_FD1']['CcAslButtnCnclResPress'] = 1
    self.assertEqual(f.update(0.)['user_action'], 'resume')
    f.state.cruiseState.enabled = True
    self.assertEqual(f.update(.01, new_model=False)['user_action'], 'resume')
    f.parser.vl['Steering_Data_FD1']['CcAslButtnCnclResPress'] = 0
    self.assertIsNone(f.update(.02, new_model=False)['user_action'])

  def test_cancel_edge_stays_cancel_when_stock_disengages(self):
    f = self.fixture()
    f.parser.vl['Steering_Data_FD1']['CcAslButtnCnclResPress'] = 1
    self.assertEqual(f.update(0.)['user_action'], 'cancel')
    f.state.cruiseState.enabled = False
    self.assertEqual(f.update(.01, new_model=False)['user_action'], 'cancel')

  def test_observed_shared_resume_releases_suspended_permission(self):
    f = self.fixture()
    c = Context(comfort=True).cruise
    f.context.cruise = CaptureCruise(c)
    # Start one native session at the original user speed, then model an
    # already acknowledged automation reduction before the physical brake.
    c._start(70)
    c.transport.observed = c.observed = 65
    c.target = 65
    f.state.cruiseState.enabled = False
    f.state.brakePressed = True
    f.update(0.)
    self.assertTrue(c.suspended)
    f.state.brakePressed = False
    f.parser.vl['Steering_Data_FD1']['CcAslButtnCnclResPress'] = 1
    f.update(.1)
    self.assertTrue(c.resume_armed)
    f.state.cruiseState.enabled = True
    f.update(.11)
    self.assertFalse(c.suspended)
    self.assertEqual(c.ceiling, 70)


class DriverSelectionRegressionTest(unittest.TestCase):
  def test_driver_selection_does_not_permanently_disable_future_curve_control(self):
    c = Controller()
    c._start(70)
    c.transport.observed = c.observed = 68
    c.transport.debt[-1] = c.transport.debt[1] = 2
    m = road()
    m.action = NS(desiredAcceleration=0.2, shouldStop=False)
    for t, observed, held in ((0.0, 68, True), (0.1, 69, True), (0.4, 70, True), (0.5, 70, False), (1.0, 70, False), (3.0, 70, False)):
      c.update(
        t,
        engaged=True,
        valid=True,
        manual=held,
        override=False,
        set_speed=observed,
        speed=64.0,
        accel=0.0,
        model=m,
        cruise_engaged=True,
        forecast_mph=None,
        forecast_clear=True,
        user_action='set' if held else None,
      )
    self.assertFalse(c.fault)
    self.assertEqual(c.ceiling, 70)
    self.assertEqual(c.transport.up_debt, 2)
    self.assertEqual(c.transport.down_debt, 2)
    output = c.update(
      3.1,
      engaged=True,
      valid=True,
      manual=False,
      override=False,
      set_speed=70.0,
      speed=64.0,
      accel=0.0,
      model=m,
      cruise_engaged=True,
      forecast_mph=50.0,
      forecast_clear=False,
    )
    self.assertEqual(output, 'SET-')
    self.assertEqual(c.transport.down_debt, 3)
    self.assertEqual(c.transport.up_debt, 2)
    self.assertFalse(c.fault)

  def test_manual_lower_selection_keeps_unresolved_upward_credit(self):
    c = Controller()
    c._start(70)
    c.transport.debt[1] = 2
    m = road()
    m.action = NS(desiredAcceleration=0.2, shouldStop=False)
    outputs = []
    for t, held in ((0.0, True), (0.1, True), (0.6, False), (1.0, False)):
      outputs.append(
        c.update(
          t,
          engaged=True,
          valid=True,
          manual=held,
          override=False,
          set_speed=62.0,
          speed=60.0,
          accel=0.0,
          model=m,
          cruise_engaged=True,
          forecast_clear=True,
          user_action='set' if held else None,
        )
      )
    self.assertEqual(c.ceiling, 62)
    self.assertEqual(c.transport.up_debt, 2)
    self.assertNotIn('SET+', outputs)
    self.assertFalse(c.fault)


class CurrentDemandRegressionTest(unittest.TestCase):
  def test_old_episode_minimum_does_not_reduce_when_already_below_current_demand(self):
    c = Controller()
    c._start(70)
    c.target = 38.0
    m = road(bend=True)
    m.action = NS(desiredAcceleration=0.0, shouldStop=False)
    outputs = []
    for i in range(101):
      outputs.append(
        c.update(i * 0.01, engaged=True, valid=True, manual=False, override=False, set_speed=70.0, speed=40.0, accel=0.0, model=m, cruise_engaged=True)
      )
    self.assertGreater(c.geometry_target, 40.0)
    self.assertNotIn('SET-', outputs)
    self.assertEqual(c.target, 38.0, 'Do not authorize restoration into a curve')

  def test_stronger_forecast_still_reduces(self):
    c = Controller()
    c._start(70)
    c.target = 38.0
    m = road(bend=True)
    m.action = NS(desiredAcceleration=0.0, shouldStop=False)
    outputs = [
      c.update(
        i * 0.01, engaged=True, valid=True, manual=False, override=False, set_speed=70.0, speed=40.0, accel=0.0, model=m, cruise_engaged=True, forecast_mph=35.0
      )
      for i in range(101)
    ]
    self.assertIn('SET-', outputs)


class ResumeAcceleratorRegressionTest(unittest.TestCase):
  def call(self, c, t, **kw):
    m = road()
    m.action = NS(desiredAcceleration=0.3, shouldStop=False)
    args = {
      'engaged': True, 'valid': True, 'manual': False, 'override': False, 'set_speed': 65.0,
      'speed': 60.0, 'accel': 0.0, 'model': m, 'cruise_engaged': True, 'forecast_clear': True
    }
    args.update(kw)
    return c.update(t, **args)

  def suspended(self):
    c = Controller()
    c._start(70)
    c.transport.observed = c.observed = 65
    self.call(c, 0.0, override=True, override_kind='brake', engaged=False, cruise_engaged=False)
    return c

  def test_explicit_resume_during_gas_survives_until_oem_engages(self):
    c = self.suspended()
    self.call(c, 1.0, user_action='resume', manual=True, override=True, override_kind='gas', engaged=False, cruise_engaged=False)
    self.assertTrue(c.resume_armed)
    self.call(c, 1.1, user_action='resume', manual=True, override=True, override_kind='gas')
    self.call(c, 1.2, override=True, override_kind='gas')
    self.call(c, 1.5)
    self.assertFalse(c.suspended)
    outputs = [self.call(c, 1.6 + i * 0.05) for i in range(12)]
    self.assertIn('SET+', outputs)
    self.assertNotIn('RESUME', outputs)

  def test_gas_without_resume_does_not_rearm(self):
    c = self.suspended()
    self.call(c, 1.0, override=True, override_kind='gas', engaged=False, cruise_engaged=False)
    self.call(c, 1.5)
    self.assertTrue(c.suspended)

  def test_brake_clears_previous_resume_authorization(self):
    c = self.suspended()
    self.call(c, 1.0, user_action='resume', manual=True, engaged=False, cruise_engaged=False)
    self.call(c, 1.1, override=True, override_kind='brake', engaged=False, cruise_engaged=False)
    self.call(c, 1.5)
    self.assertTrue(c.suspended)
    self.assertFalse(c.resume_armed)

  def test_cancel_with_gas_still_clears_resume(self):
    c = self.suspended()
    self.call(c, 1.0, user_action='resume', manual=True, engaged=False, cruise_engaged=False)
    self.call(c, 1.1, user_action='cancel', manual=True, override=True, override_kind='gas')
    self.call(c, 1.5)
    self.assertTrue(c.suspended)
    self.assertFalse(c.resume_armed)
