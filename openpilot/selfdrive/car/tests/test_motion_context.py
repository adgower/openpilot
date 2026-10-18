from types import SimpleNamespace as NS
import pytest
from openpilot.selfdrive.car.motion_context import MotionContext
from openpilot.cereal import log


def calibration(*, valid=True, status='calibrated', t=1_000_000_000):
  e = log.Event.new_message()
  e.logMonoTime = t
  e.valid = valid
  c = e.init('extrinsicsCalibration')
  c.rpyCalib = [0.0, 0.0, 0.0]
  c.calStatus = status
  return e


def motion(*, valid=True, t=1_050_000_000, source=1_040_000_000, bad=None):
  e = log.Event.new_message()
  e.logMonoTime = t
  e.valid = valid
  m = e.init('deviceMotion')
  m.timestamp = source
  m.angularVelocityDevice.z = 0.04
  m.angularVelocityDevice.zStd = 0.013
  m.angularVelocityDevice.xStd = 0.013
  m.angularVelocityDevice.yStd = 0.013
  m.angularVelocityDevice.valid = True
  m.inputsOK = True
  m.posenetOK = True
  m.sensorsOK = True
  if bad == 'angular':
    m.angularVelocityDevice.valid = False
  if bad in ['inputsOK', 'posenetOK', 'sensorsOK']:
    setattr(m, bad, False)
  return e


def capture(ctx, now=1_060_000_000):
  values = {}
  ctx.feed(NS(set_motion=lambda **kw: values.update(kw)), now_ns=now)
  return values


def test_valid_actual_adapter_retains_three_distinct_times():
  c = MotionContext()
  c.on_event(calibration())
  c.on_event(motion())
  v = capture(c)
  assert v['valid'] and v['sample_ns'] == 1_040_000_000 and v['received_ns'] == 1_050_000_000 and v['now_ns'] == 1_060_000_000
  assert v['yaw_rate'] == pytest.approx(0.04) and v['yaw_std'] == pytest.approx(0.013)


@pytest.mark.parametrize('event', [{'valid': False}, {'status': 'invalid'}, {'status': 'uncalibrated'}, {'status': 'recalibrating'}])
def test_calibration_invalidation_immediately_revokes_cached_motion(event):
  c = MotionContext()
  c.on_event(calibration())
  c.on_event(motion())
  assert capture(c)['valid']
  c.on_event(calibration(t=1_055_000_000, **event))
  assert not capture(c)['valid']
  c.on_event(calibration(t=1_058_000_000))
  assert not capture(c)['valid'], 'calibration recovery must not revive old cached yaw'
  c.on_event(motion(t=1_059_000_000, source=1_058_500_000))
  assert capture(c)['valid']


@pytest.mark.parametrize('bad', ['angular', 'inputsOK', 'posenetOK', 'sensorsOK', 'envelope'])
def test_motion_validity_flags_preserved(bad):
  c = MotionContext()
  c.on_event(calibration())
  c.on_event(motion(bad=bad, valid=bad != 'envelope'))
  assert not capture(c)['valid']


def test_no_extrinsics_is_invalid():
  c = MotionContext()
  c.on_event(motion())
  assert not capture(c)['valid']


class SM(dict):
  def __init__(self, events, received_ns):
    super().__init__((e.which(), getattr(e, e.which())) for e in events)
    self.seen = {e.which(): True for e in events}
    self.updated = self.seen.copy()
    self.valid = {e.which(): bool(e.valid) for e in events}
    self.logMonoTime = {e.which(): int(e.logMonoTime) for e in events}
    self.recv_time = {e.which(): received_ns / 1e9 for e in events}


def test_submaster_preserves_actual_receipt_instead_of_publication_time():
  c = MotionContext()
  sm = SM([motion(), calibration()], 1_060_000_000)
  c.update_from_sm(sm)
  v = capture(c, 1_070_000_000)
  assert v['valid'] and v['sample_ns'] == 1_040_000_000 and v['received_ns'] == 1_060_000_000


def test_submaster_orders_conflated_topics_before_calibration_invalidation():
  c = MotionContext()
  c.on_event(calibration())
  c.on_event(motion())
  # Dict iteration order is deliberately calibration then older motion.
  sm = SM([calibration(t=1_080_000_000, status='invalid'), motion(t=1_075_000_000)], 1_090_000_000)
  c.update_from_sm(sm)
  assert not capture(c, 1_095_000_000)['valid']


def test_future_publication_cannot_masquerade_as_received_motion():
  c = MotionContext()
  sm = SM([calibration(), motion(t=1_200_000_000)], 1_060_000_000)
  c.update_from_sm(sm)
  assert not capture(c, 1_070_000_000)['valid']


def test_missing_receipt_time_is_fail_closed_in_live_mode():
  c = MotionContext()
  sm = SM([calibration(), motion()], 1_060_000_000)
  sm.recv_time = {}
  c.update_from_sm(sm)
  assert not capture(c, 1_070_000_000)['valid']


def test_replay_receipt_proxy_is_explicit_and_not_wall_clock():
  c = MotionContext()
  sm = SM([calibration(), motion()], 99_000_000_000)
  c.update_from_sm(sm, replay=True)
  assert capture(c)['received_ns'] == 1_050_000_000
