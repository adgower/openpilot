"""Regressions for the offline model-speed demand observer."""

from types import SimpleNamespace as NS
import math
import unittest

from openpilot.selfdrive.car.predicted_speed import PredictedSpeedDemand, MPH


def model(speeds=(55.0, 55.0, 55.0, 55.0), times=(3.0, 5.0, 7.0, 10.0), accel=-0.2, stop=False):
  return NS(velocity=NS(t=times, x=[v * MPH for v in speeds]), action=NS(desiredAcceleration=accel, shouldStop=stop))


def observe(observer, now, sample=None, **kwargs):
  values = {"speed_mph": 65.0, "ceiling": 70.0, "model": sample or model(), "allowed": True, "radar_clear": True}
  values.update(kwargs)
  return observer.observe(now, **values)


def qualify(observer, sample=None):
  result = None
  for i in range(15):
    result = observe(observer, i * 0.05, sample)
  return result


class PredictedSpeedTests(unittest.TestCase):
  def test_persistence_and_mph_units(self):
    observer = PredictedSpeedDemand()
    self.assertIsNone(observe(observer, 0.0))
    self.assertIsNone(observe(observer, 0.1))
    self.assertAlmostEqual(qualify(PredictedSpeedDemand()), 55.0)

  def test_single_low_future_point_is_rejected(self):
    self.assertIsNone(qualify(PredictedSpeedDemand(), model((65.0, 65.0, 30.0, 65.0))))
    self.assertEqual(qualify(PredictedSpeedDemand(robust=False, raw_threshold=None), model((65.0, 65.0, 30.0, 65.0))), 30.0)

  def test_weak_acceleration_is_optional(self):
    self.assertIsNone(qualify(PredictedSpeedDemand(), model(accel=0.05)))
    self.assertEqual(qualify(PredictedSpeedDemand(raw_threshold=None), model(accel=0.05)), 55.0)

  def test_driver_override_clears_immediately(self):
    observer = PredictedSpeedDemand()
    qualify(observer)
    self.assertIsNone(observe(observer, 0.75, allowed=False))
    self.assertIsNone(observe(observer, 0.8))

  def test_radar_loss_clears_immediately(self):
    observer = PredictedSpeedDemand()
    qualify(observer)
    self.assertIsNone(observe(observer, 0.75, radar_clear=False))

  def test_model_gap_requires_new_persistence(self):
    observer = PredictedSpeedDemand()
    qualify(observer)
    self.assertIsNone(observe(observer, 1.1))
    self.assertIsNone(observe(observer, 1.15))

  def test_invalid_arrays_rejected(self):
    for sample in (
      model((55.0, math.nan, 55.0, 55.0)),
      model((-1.0, 55.0, 55.0, 55.0)),
      model(times=(3.0, 5.0, 4.0, 10.0)),
      model(times=(3.0, 5.0, 7.0)),
      model(times=(3.0, 3.0, 7.0, 10.0)),
      model(accel=math.nan),
      model(stop=True),
    ):
      with self.subTest(sample=sample):
        self.assertIsNone(qualify(PredictedSpeedDemand(), sample))

  def test_invalid_sample_clears_qualified_target(self):
    observer = PredictedSpeedDemand()
    qualify(observer)
    self.assertIsNone(observe(observer, 0.75, model((55.0, math.nan, 55.0, 55.0))))

  def test_bounded_release_after_forecast_disappears(self):
    observer = PredictedSpeedDemand()
    qualify(observer)
    self.assertEqual(observe(observer, 0.75, model((65.0, 65.0, 65.0, 65.0))), 55.0)
    result = None
    for i in range(16, 27):
      result = observe(observer, i * 0.05, model((65.0, 65.0, 65.0, 65.0)))
    self.assertIsNone(result)

  def test_floor_and_original_ceiling(self):
    self.assertEqual(qualify(PredictedSpeedDemand(), model((0.0, 0.0, 0.0, 0.0))), 25.0)
    observer = PredictedSpeedDemand()
    for i in range(15):
      result = observe(observer, i * 0.05, ceiling=50.0)
    self.assertEqual(result, 50.0)

  def test_reverse_clock_clears(self):
    observer = PredictedSpeedDemand()
    qualify(observer)
    self.assertIsNone(observe(observer, 0.6))

  def test_valid_clear_sample_is_available(self):
    observer = PredictedSpeedDemand()
    self.assertFalse(observer.available)
    self.assertIsNone(observe(observer, 0.0, model((65.0, 65.0, 65.0, 65.0))))
    self.assertTrue(observer.available)

  def test_available_during_persistence(self):
    observer = PredictedSpeedDemand()
    self.assertIsNone(observe(observer, 0.0))
    self.assertTrue(observer.available)
    self.assertFalse(observer.qualified)

  def test_invalid_cues_are_unavailable(self):
    for sample, kwargs in (
      (model((55.0, math.nan, 55.0, 55.0)), {}),
      (model(accel=math.nan), {}),
      (model(stop=True), {}),
      (model(), {'allowed': False}),
      (model(), {'radar_clear': False}),
    ):
      with self.subTest(sample=sample, kwargs=kwargs):
        observer = PredictedSpeedDemand()
        qualify(observer)
        self.assertIsNone(observe(observer, 0.75, sample, **kwargs))
        self.assertFalse(observer.available)

  def test_reset_clears_availability(self):
    observer = PredictedSpeedDemand()
    qualify(observer)
    self.assertTrue(observer.available)
    observer.reset()
    self.assertFalse(observer.available)

  def test_data_valid_distinguishes_clear_and_unavailable(self):
    for sample, kwargs in ((model(), {'allowed': False}), (model(), {'radar_clear': False}), (model(stop=True), {}), (model((65.0, 65.0, 65.0, 65.0)), {})):
      with self.subTest(sample=sample, kwargs=kwargs):
        observer = PredictedSpeedDemand()
        self.assertIsNone(observe(observer, 0.0, sample, **kwargs))
        self.assertTrue(observer.data_valid)
    for sample in (model((55.0, math.nan, 55.0, 55.0)), model(accel=math.nan), model(times=(3.0, 5.0, 4.0, 10.0))):
      observer = PredictedSpeedDemand()
      observe(observer, 0.0, sample)
      self.assertFalse(observer.data_valid)

  def test_data_valid_gap_and_reverse_clock(self):
    observer = PredictedSpeedDemand()
    qualify(observer)
    self.assertIsNone(observe(observer, 1.1))
    self.assertTrue(observer.data_valid)
    self.assertIsNone(observe(observer, 1.0))
    self.assertFalse(observer.data_valid)

  def test_demand_tracks_current_forecast(self):
    observer = PredictedSpeedDemand()
    qualify(observer)
    self.assertEqual(observe(observer, 0.75, model((50.0, 50.0, 50.0, 50.0))), 50.0)


if __name__ == '__main__':
  unittest.main()
