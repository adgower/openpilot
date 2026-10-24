import pytest
from openpilot.selfdrive.car.longitudinal_pose import ConsumerPoseRefresh
from openpilot.cereal import log
from opendbc.car import structs
from opendbc.car.ford.longitudinal_coordination import GradeLease


class Inputs(dict):
  def __init__(self, now, source):
    dm = log.DeviceMotion.new_message(timestamp=source, inputsOK=True, sensorsOK=True, posenetOK=True)
    dm.orientationNED.valid = True
    dm.orientationNED.y = 0.02
    super().__init__(deviceMotion=dm, extrinsicsCalibration=log.ExtrinsicsCalibration.new_message(calStatus='calibrated', rpyCalib=[0.0, 0.01, 0.0]))
    self.seen = dict.fromkeys(self, True)
    self.valid = dict(self.seen)
    self.logMonoTime = dict.fromkeys(self, now - 3000000)

  def update(self, *args):
    pass


def command(now=1_000_000_000):
  c = structs.CarControl(enabled=True, longActive=True)
  c.actuators.longControlState = 'pid'
  c.actuators.accel = 0.2
  c.orientationNED = [0.0, 0.01, 0.0]
  q = c.navigatorLongitudinal
  q.enabled = True
  q.epoch = 1
  q.sequence = 10
  q.producerMonoTime = now - 5_220_595
  q.poseMonoTime = now - 100_120_004
  q.poseValid = True
  q.referenceValid = True
  return c


def test_recorded_100_120ms_race_uses_available_coherent_pose_without_restamping():
  now = 1_000_000_000
  c = command()
  sm = Inputs(now, now - 30_552_118)
  original = c.to_bytes()
  r = ConsumerPoseRefresh().refresh(c.as_reader(), sm, now)
  assert r.navigatorLongitudinal.poseMonoTime == now - 30_552_118
  assert r.navigatorLongitudinal.producerMonoTime == c.navigatorLongitudinal.producerMonoTime
  assert c.to_bytes() == original
  assert abs(r.orientationNED[1] - 0.03) < 1e-6
  assert r.navigatorCalibration.to_dict() == c.navigatorCalibration.to_dict()
  q = r.navigatorLongitudinal
  g = GradeLease(True).update(now / 1e9, 0.0, q.poseMonoTime / 1e9, q.poseValid, True, 8.0, 'pid')
  assert g.reason != 'invalid_pose_disengage'


@pytest.mark.parametrize(
  'case',
  [
    'producer_invalid',
    'sensor_bad',
    'future_source',
    'source_after_producer',
    'stale_source',
    'reversed_source',
    'bad_calibration',
    'unseen',
    'stale_publication',
  ],
)
def test_unhealthy_cannot_rescue_expired_pose(case):
  now = 1_000_000_000
  c = command()
  sm = Inputs(now, now - 30_000_000)
  if case == 'producer_invalid':
    c.navigatorLongitudinal.poseValid = False
  elif case == 'sensor_bad':
    sm['deviceMotion'].sensorsOK = False
  elif case == 'future_source':
    sm['deviceMotion'].timestamp = now + 1
  elif case == 'source_after_producer':
    sm['deviceMotion'].timestamp = now - 1_000_000
  elif case == 'stale_source':
    sm['deviceMotion'].timestamp = now - 110_000_000
  elif case == 'reversed_source':
    sm['deviceMotion'].timestamp = c.navigatorLongitudinal.poseMonoTime - 1
  elif case == 'bad_calibration':
    sm['extrinsicsCalibration'].calStatus = 'uncalibrated'
  elif case == 'unseen':
    sm.seen['deviceMotion'] = False
  elif case == 'stale_publication':
    sm.logMonoTime['deviceMotion'] = now - 200_000_000
  r = ConsumerPoseRefresh().refresh(c.as_reader(), sm, now)
  q = r.navigatorLongitudinal
  g = GradeLease(True).update(now / 1e9, 0.0, q.poseMonoTime / 1e9, q.poseValid, True, 8.0, 'pid')
  assert g.reason == 'invalid_pose_disengage'


@pytest.mark.parametrize('kwargs', [{'enabled': False}, {'speed': 2.8}])
def test_out_of_scope_bytes_unchanged(kwargs):
  c = command()
  r = ConsumerPoseRefresh().refresh(c.as_reader(), Inputs(1_000_000_000, 970_000_000), 1_000_000_000, **kwargs)
  assert r.as_builder().to_bytes() == c.to_bytes()


@pytest.mark.parametrize('fault', ['sensor_bad', 'reversed', 'motion_header_bad', 'future_publication'])
def test_new_unhealthy_sample_cannot_rescue_expired_original(fault):
  now = 1_000_000_000
  c = command()
  c.navigatorLongitudinal.poseMonoTime = now - 100_120_004
  sm = Inputs(now, now - 30_000_000)
  if fault == 'sensor_bad':
    sm['deviceMotion'].inputsOK = False
  elif fault == 'reversed':
    sm['deviceMotion'].timestamp = now - 110_000_000
  elif fault == 'motion_header_bad':
    sm.valid['deviceMotion'] = False
  else:
    sm.logMonoTime['deviceMotion'] = now + 1
  r = ConsumerPoseRefresh().refresh(c.as_reader(), sm, now)
  assert not r.navigatorLongitudinal.poseValid


def test_repeated_old_producer_cannot_reverse_already_refreshed_source():
  f = ConsumerPoseRefresh()
  now = 1_000_000_000
  c = command()
  sm = Inputs(now, now - 30_000_000)
  a = f.refresh(c.as_reader(), sm, now)
  assert a.navigatorLongitudinal.poseMonoTime == 970_000_000
  b = f.refresh(c.as_reader(), sm, now + 10_000_000)
  assert b.navigatorLongitudinal.poseMonoTime == 970_000_000
  assert b.navigatorLongitudinal.producerMonoTime == c.navigatorLongitudinal.producerMonoTime


def test_monotonic_cache_does_not_extend_measured_source_expiry():
  f = ConsumerPoseRefresh()
  now = 1_000_000_000
  c = command()
  sm = Inputs(now, 970_000_000)
  f.refresh(c.as_reader(), sm, now)
  c.navigatorLongitudinal.producerMonoTime = now + 80_000_000
  c.navigatorLongitudinal.poseMonoTime = now
  c.navigatorLongitudinal.sequence += 1
  sm['deviceMotion'].timestamp = 970_000_000
  sm.logMonoTime['deviceMotion'] = now - 3_000_000
  r = f.refresh(c.as_reader(), sm, now + 80_000_000)
  assert r.navigatorLongitudinal.poseMonoTime != 970_000_000 or not r.navigatorLongitudinal.poseValid


def test_true_producer_source_reversal_is_not_hidden_by_fresh_consumer():
  f = ConsumerPoseRefresh()
  now = 1_000_000_000
  c = command()
  sm = Inputs(now, 970_000_000)
  f.refresh(c.as_reader(), sm, now)
  c.navigatorLongitudinal.sequence += 1
  c.navigatorLongitudinal.producerMonoTime += 1_000_000
  c.navigatorLongitudinal.poseMonoTime -= 1
  r = f.refresh(c.as_reader(), sm, now + 1_000_000)
  assert not r.navigatorLongitudinal.poseValid


def test_subscriber_is_drained_once_before_snapshot_validation():
  now = 1_000_000_000
  c = command()

  class RacingInputs(Inputs):
    calls = 0

    def update(self, *args):
      self.calls += 1
      if self.calls >= 2:
        self.logMonoTime['deviceMotion'] = now + 1

  sm = RacingInputs(now, 970_000_000)
  r = ConsumerPoseRefresh().refresh(c.as_reader(), sm, now)
  assert sm.calls == 1 and r.navigatorLongitudinal.poseValid


def test_healthy_pose_is_not_refreshed_or_filtered_before_expiry():
  now = 1_000_000_000
  c = command()
  c.navigatorLongitudinal.poseMonoTime = now - 90_000_000

  class MustNotPoll:
    def update(self, *args):
      raise AssertionError('Healthy source must remain untouched')

  r = ConsumerPoseRefresh().refresh(c.as_reader(), MustNotPoll(), now)
  assert r.navigatorLongitudinal.to_dict() == c.navigatorLongitudinal.to_dict() and list(r.orientationNED) == list(c.orientationNED)


@pytest.mark.parametrize('late_clock', [1_001_000_000, 1_100_000_000, 999_000_000])
def test_post_drain_apply_clock_is_used_and_revalidated(late_clock):
  c = command()
  times = iter([late_clock, late_clock])
  f = ConsumerPoseRefresh(clock_ns=lambda: next(times))
  r = f.refresh(c.as_reader(), Inputs(1_000_000_000, 970_000_000), 1_000_000_000)
  assert f.apply_time_ns == late_clock
  assert bool(r.navigatorLongitudinal.poseValid) == (late_clock == 1_001_000_000)


def test_final_clock_cannot_extend_pose_lifetime_after_calibration_work():
  c = command()
  times = iter([1_001_000_000, 1_080_000_000])
  f = ConsumerPoseRefresh(clock_ns=lambda: next(times))
  r = f.refresh(c.as_reader(), Inputs(1_000_000_000, 970_000_000), 1_000_000_000)
  assert not r.navigatorLongitudinal.poseValid and f.apply_time_ns == 1_080_000_000
