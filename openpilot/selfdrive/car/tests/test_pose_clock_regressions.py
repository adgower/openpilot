"""Live-clock expiry boundaries, independent of recorded/private drive fixtures."""

import pytest
from openpilot.selfdrive.car.longitudinal_pose import ConsumerPoseRefresh
from openpilot.selfdrive.car.tests.test_longitudinal_pose import Inputs, command


NOW = 1_000_000_000


def fresh_command():
  c = command(NOW)
  c.navigatorLongitudinal.poseMonoTime = NOW - 95_000_000
  return c


def test_live_entry_clock_selects_new_measured_pose_before_expiry_decision():
  c = fresh_command()
  before = c.to_dict()
  f = ConsumerPoseRefresh(clock_ns=lambda: NOW + 10_000_000)
  out = f.refresh(c.as_reader(), Inputs(NOW, NOW - 30_000_000), NOW)
  assert out.navigatorLongitudinal.poseValid
  assert out.navigatorLongitudinal.poseMonoTime == NOW - 30_000_000
  assert out.navigatorLongitudinal.producerMonoTime == c.navigatorLongitudinal.producerMonoTime
  assert out.navigatorLongitudinal.sequence == c.navigatorLongitudinal.sequence
  assert out.navigatorLongitudinal.epoch == c.navigatorLongitudinal.epoch
  assert f.apply_time_ns == NOW + 10_000_000
  assert c.to_dict() == before


def test_live_fresh_fast_path_cannot_survive_final_clock_expiry():
  c = fresh_command()
  times = iter([NOW, NOW + 10_000_000])
  f = ConsumerPoseRefresh(clock_ns=lambda: next(times))
  out = f.refresh(c.as_reader(), Inputs(NOW, NOW - 30_000_000), NOW)
  assert not out.navigatorLongitudinal.poseValid
  assert out.navigatorLongitudinal.poseMonoTime == c.navigatorLongitudinal.poseMonoTime
  assert out.navigatorLongitudinal.producerMonoTime == c.navigatorLongitudinal.producerMonoTime
  assert f.apply_time_ns == NOW + 10_000_000


def test_live_selection_clock_reversal_rejects_without_restamping():
  c = fresh_command()
  f = ConsumerPoseRefresh(clock_ns=lambda: NOW - 1)
  out = f.refresh(c.as_reader(), Inputs(NOW, NOW - 30_000_000), NOW)
  assert not out.navigatorLongitudinal.poseValid
  assert out.navigatorLongitudinal.poseMonoTime == c.navigatorLongitudinal.poseMonoTime
  assert out.navigatorLongitudinal.producerMonoTime == c.navigatorLongitudinal.producerMonoTime
  assert f.apply_time_ns == NOW - 1


def test_live_outer_final_clock_reversal_cannot_undo_canonical_guard():
  c = fresh_command()
  times = iter([NOW + 10_000_000, NOW + 11_000_000, NOW + 12_000_000, NOW + 11_500_000])
  f = ConsumerPoseRefresh(clock_ns=lambda: next(times))
  out = f.refresh(c.as_reader(), Inputs(NOW, NOW - 30_000_000), NOW)
  assert not out.navigatorLongitudinal.poseValid
  assert out.navigatorLongitudinal.poseMonoTime == NOW - 30_000_000
  assert out.navigatorLongitudinal.producerMonoTime == c.navigatorLongitudinal.producerMonoTime
  assert f.apply_time_ns == NOW + 11_500_000


def test_live_refreshed_pose_cannot_survive_outer_final_work_expiry():
  c = fresh_command()
  times = iter([NOW + 10_000_000, NOW + 11_000_000, NOW + 12_000_000, NOW + 80_000_000])
  f = ConsumerPoseRefresh(clock_ns=lambda: next(times))
  out = f.refresh(c.as_reader(), Inputs(NOW, NOW - 30_000_000), NOW)
  assert not out.navigatorLongitudinal.poseValid
  assert out.navigatorLongitudinal.poseMonoTime == NOW - 30_000_000
  assert out.navigatorLongitudinal.producerMonoTime == c.navigatorLongitudinal.producerMonoTime
  assert f.apply_time_ns == NOW + 80_000_000


@pytest.mark.parametrize('mode', ['disabled', 'owner_off', 'inactive', 'low_speed', 'non_pid', 'replay'])
def test_live_clock_guards_do_not_run_outside_active_scope(mode):
  c = fresh_command()
  kwargs = {}
  if mode == 'disabled':
    kwargs['enabled'] = False
  elif mode == 'owner_off':
    c.enabled = False
  elif mode == 'inactive':
    c.longActive = False
  elif mode == 'low_speed':
    kwargs['speed'] = 2.8
  elif mode == 'non_pid':
    c.actuators.longControlState = 'off'

  def forbidden_clock():
    raise AssertionError('live clock sampled outside active scope')

  f = ConsumerPoseRefresh(clock_ns=None if mode == 'replay' else forbidden_clock)
  out = f.refresh(c.as_reader(), Inputs(NOW, NOW - 30_000_000), NOW, **kwargs)
  assert out.to_dict() == c.to_dict()
  assert f.apply_time_ns == NOW
