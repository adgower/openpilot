"""Local regressions for acknowledged forecast demand on the copied controller."""

import copy
from types import SimpleNamespace as NS
import unittest
from openpilot.selfdrive.car.vision_curve_speed import AcknowledgedCurveSpeed, VirtualSetSpeed, MPH


def road(*, confidence=0.9, departure=False, bend=False):
  times = [0.6, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]
  xs = [0.0, 30.0, 60.0, 90.0, 120.0, 160.0, 200.0, 240.0, 280.0]

  def line(y):
    return NS(x=xs, y=[y] * len(xs))

  return NS(
    laneLineProbs=[0.1, confidence, confidence, 0.1],
    laneLines=[line(-5.0), line(-1.8), line(1.8), line(5.0)],
    position=line(2.0 if departure else 0.0),
    roadEdgeStds=[0.5, 0.5],
    orientationRate=NS(t=times, z=[0.09 if bend else 0.0] * len(times)),
    velocity=NS(x=[60 * MPH] * len(times)),
  )


class ForecastRetryTest(unittest.TestCase):
  def call(self, c, t, set_speed=65, forecast=55.0, model=None, **kw):
    args = {
      "engaged": True,
      "valid": True,
      "manual": False,
      "override": False,
      "set_speed": set_speed,
      "speed": 64.0,
      "accel": 0.0,
      "model": model or road(),
      "cruise_engaged": True,
      "forecast_mph": forecast,
    }
    args.update(kw)
    return c.update(t, **args)

  def initialized(self, **kw):
    c = AcknowledgedCurveSpeed()
    self.assertIsNone(self.call(c, 0, **kw))
    return c

  def test_low_confidence_forecast_requests_and_retries(self):
    c = self.initialized(model=road(confidence=0.1))
    self.assertEqual(self.call(c, 0.1, model=road(confidence=0.1)), 'SET-')
    self.assertEqual(c.pending, 64)
    self.assertIsNone(self.call(c, 1.2, model=road(confidence=0.1)))
    self.assertEqual(self.call(c, 1.46, model=road(confidence=0.1)), 'SET-')
    self.assertEqual(c.pending, 64)
    self.assertIn('SET-', c.retries)

  def test_no_forecast_cannot_bypass_lane_confidence(self):
    c = self.initialized(forecast=None, model=road(confidence=0.1, bend=True))
    for i in range(1, 50):
      self.assertIsNone(self.call(c, i * 0.1, forecast=None, model=road(confidence=0.1, bend=True)))

  def test_corridor_departure_still_blocks(self):
    c = self.initialized(model=road(departure=True))
    for i in range(1, 50):
      self.assertIsNone(self.call(c, i * 0.1, model=road(departure=True)))

  def test_retry_holds_two_press_floor(self):
    c = self.initialized(set_speed=27, forecast=25.0, speed=30.0)
    self.assertEqual(self.call(c, 0.1, set_speed=27, forecast=25.0, speed=30.0), 'SET-')
    self.assertEqual(self.call(c, 1.46, set_speed=27, forecast=25.0, speed=30.0), 'SET-')
    c = self.initialized(set_speed=26, forecast=25.0, speed=30.0)
    self.assertIsNone(self.call(c, 0.1, set_speed=26, forecast=25.0, speed=30.0))

  def test_retry_refused_if_double_press_would_pass_new_target(self):
    c = self.initialized()
    self.assertEqual(self.call(c, 0.1), 'SET-')
    self.assertIsNone(self.call(c, 1.46, forecast=63.5))

  def test_active_forecast_prevents_recovery_raise(self):
    c = self.initialized()
    self.assertEqual(self.call(c, 0.1), 'SET-')
    self.call(c, 0.2, set_speed=64, forecast=None)
    c.recovering = True
    c.pending = None
    for i in range(1, 50):
      self.assertIsNone(self.call(c, 0.2 + i * 0.1, set_speed=64, forecast=55.0))

  def test_restore_respects_original_ceiling_and_retries(self):
    c = self.initialized()
    self.assertEqual(self.call(c, 0.1), 'SET-')
    self.call(c, 0.2, set_speed=64, forecast=None, forecast_clear=True)
    self.call(c, 0.5, set_speed=64, forecast=None, forecast_clear=True)
    self.assertEqual(self.call(c, 2.1, set_speed=64, forecast=None, forecast_clear=True), 'SET+')
    # The original ceiling is only one increment away. Retrying could
    # apply both pulses, so keep this press un-retried.
    self.assertIsNone(self.call(c, 3.46, set_speed=64, forecast=None, forecast_clear=True))
    self.assertLessEqual(c.pending or 65, 65)

  def test_restore_retry_uses_headroom_and_active_demand(self):
    c = self.initialized()
    for t, observed in ((0.1, 65), (0.2, 64), (0.4, 64), (0.5, 63), (0.7, 63), (0.8, 62)):
      self.call(c, t, set_speed=observed, forecast=55.0)
    self.call(c, 1.0, set_speed=62, forecast=None, forecast_clear=True)
    self.assertEqual(self.call(c, 2.6, set_speed=62, forecast=None, forecast_clear=True), 'SET+')
    self.assertEqual(c.pending, 63)
    active = copy.deepcopy(c)
    self.assertIsNone(self.call(active, 3.96, set_speed=62, forecast=55.0))
    # The clear-demand case still retries the same target below the ceiling.
    self.assertEqual(self.call(c, 3.96, set_speed=62, forecast=None, forecast_clear=True), 'SET+')
    self.assertEqual(c.pending, 63)
    self.assertIn('SET+', c.retries)

  def test_low_confidence_forecast_owned_reduction_restores_only_after_explicit_clear(self):
    c = self.initialized(model=road(confidence=0.1))
    self.assertEqual(self.call(c, 0.1, model=road(confidence=0.1)), 'SET-')
    self.call(c, 0.2, set_speed=64, forecast=None, forecast_clear=True, model=road(confidence=0.1))
    self.call(c, 0.5, set_speed=64, forecast=None, forecast_clear=True, model=road(confidence=0.1))
    self.assertEqual(self.call(c, 2.1, set_speed=64, forecast=None, forecast_clear=True, model=road(confidence=0.1)), 'SET+')
    self.assertEqual(c.pending, 65)

  def test_restoring_original_ceiling_ends_forecast_ownership(self):
    c = self.initialized(model=road(confidence=0.1))
    self.call(c, 0.1, model=road(confidence=0.1))
    self.call(c, 0.2, set_speed=64, forecast=None, forecast_clear=True, model=road(confidence=0.1))
    self.call(c, 0.5, set_speed=64, forecast=None, forecast_clear=True, model=road(confidence=0.1))
    self.assertEqual(self.call(c, 2.1, set_speed=64, forecast=None, forecast_clear=True, model=road(confidence=0.1)), 'SET+')
    self.call(c, 2.2, set_speed=65, forecast=None, forecast_clear=True, model=road(confidence=0.1))
    self.assertFalse(c.forecast_owned)
    self.call(c, 2.5, forecast=None, forecast_clear=True, model=road(bend=True))
    self.assertEqual(self.call(c, 3.2, forecast=None, forecast_clear=True, model=road(bend=True)), 'SET-')
    self.assertFalse(c.forecast_owned)
    for i in range(1, 50):
      self.assertIsNone(self.call(c, 3.2 + i * 0.1, set_speed=64, forecast=None, forecast_clear=True, model=road(confidence=0.1)))

  def test_forecast_clear_is_not_authority_without_owned_reduction(self):
    c = self.initialized(forecast=None, model=road(confidence=0.1))
    for i in range(1, 50):
      self.assertIsNone(self.call(c, i * 0.1, forecast=None, forecast_clear=True, model=road(confidence=0.1)))
    self.assertFalse(c.forecast_owned)

  def test_unknown_forecast_and_low_confidence_bend_cannot_restore(self):
    for clear, bend in ((False, False), (True, True)):
      c = self.initialized(model=road(confidence=0.1))
      self.assertEqual(self.call(c, 0.1, model=road(confidence=0.1)), 'SET-')
      for i in range(2, 50):
        self.assertIsNone(self.call(c, i * 0.1, set_speed=64, forecast=None, forecast_clear=clear, model=road(confidence=0.1, bend=bend)))
    c = self.initialized(model=road(confidence=0.1))
    self.call(c, 0.1, model=road(confidence=0.1))
    self.assertIsNone(self.call(c, 0.2, set_speed=64, forecast=None, forecast_clear=True, model=road(confidence=0.1), valid=False))
    self.assertFalse(c.forecast_owned)

  def test_invalid_forecasts_do_not_create_authority(self):
    for forecast in (None, float('nan'), float('inf'), -2.0, 0.0):
      c = self.initialized(forecast=forecast)
      for i in range(1, 30):
        self.assertIsNone(self.call(c, i * 0.1, forecast=forecast))

  def test_forecast_suppresses_steps_when_measured_deceleration_reaches_target(self):
    v = VirtualSetSpeed(min_set_mph=25.0)
    for i in range(7):
      t = i * 0.1
      result = v.update(t, 65.0, None, True, False, False, speed_mph=62.0 - t / MPH, accel_mps2=-1.0, forecast_mph=60.0)
    self.assertIsNone(result[2])
    self.assertEqual(result[3], 'recorded-deceleration')
    v = VirtualSetSpeed(min_set_mph=25.0)
    for i in range(7):
      t = i * 0.1
      result = v.update(t, 65.0, None, True, False, False, speed_mph=62.0 - 0.1 * t / MPH, accel_mps2=-0.1, forecast_mph=60.0)
    self.assertEqual(result[2], 'SET-')

  def test_driver_input_suppresses_forecast(self):
    for name in ('override', 'manual', 'steering_only', 'lane_change'):
      c = self.initialized()
      self.assertIsNone(self.call(c, 0.1, **{name: True}))


if __name__ == '__main__':
  unittest.main()
