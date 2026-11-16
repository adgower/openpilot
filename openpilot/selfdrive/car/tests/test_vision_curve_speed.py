from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.vision_curve_speed import AcknowledgedCurveSpeed, Curve, MPH, comfort_approach_mph, measure_curve


def model(curved, *, ramp=False):
  x = list(range(0, 161, 5))
  path = [max(0., (d - 85) / 7.) if ramp else 0. for d in x]
  line = lambda y: SimpleNamespace(x=x, y=[y] * len(x))  # noqa: E731
  t = [i * .25 for i in range(21)]
  return SimpleNamespace(
    position=SimpleNamespace(x=x, y=path),
    laneLines=[line(-5), line(-2), line(2), line(5)],
    laneLineProbs=[.9] * 4,
    roadEdgeStds=[1.5, .4],
    orientationRate=SimpleNamespace(t=t, z=[.15 if curved else 0.] * len(t)),
    velocity=SimpleNamespace(x=[60 * MPH] * len(t)),
  )


def drive(controller, *, secs=8, curved_until=4, acknowledge=True):
  ceiling = 65
  presses = []
  for i in range(secs * 20):
    now = i / 20
    action = controller.update(now, engaged=True, valid=True, manual=False, override=False,
                               set_speed=ceiling, speed=63, accel=0, model=model(now < curved_until))
    if action:
      presses.append((now, action))
      if acknowledge:
        ceiling += 1 if action == "SET+" else -1
  return ceiling, presses


def test_slow_and_restore_only_after_exit():
  ceiling, presses = drive(AcknowledgedCurveSpeed(), secs=26, curved_until=12)
  down = [t for t, button in presses if button == "SET-"]
  up = [t for t, button in presses if button == "SET+"]
  assert len(down) > 5
  assert len(up) == len(down)
  assert min(up) >= 13.5
  assert ceiling == 65
  assert all(b[0] - a[0] >= .5 for a, b in zip(presses, presses[1:], strict=False))


def test_no_second_press_without_acknowledgement():
  controller = AcknowledgedCurveSpeed()
  _, presses = drive(controller, curved_until=8, acknowledge=False)
  assert len(presses) == 1 and presses[0][1] == "SET-"
  assert controller.fault


@pytest.mark.parametrize("reason", ["manual", "override", "invalid"])
def test_driver_or_invalid_input_drops_pending_authority(reason):
  controller = AcknowledgedCurveSpeed()
  _, presses = drive(controller, secs=1, curved_until=1, acknowledge=False)
  assert len(presses) == 1
  assert controller.update(1., engaged=True, valid=reason != "invalid", manual=reason == "manual",
                           override=reason == "override", set_speed=64, speed=63, accel=0, model=model(False)) is None
  assert controller.fault
  assert controller.pending is None


def test_ramp_veto_retains_manual_drop_evidence():
  controller = AcknowledgedCurveSpeed()
  for i in range(200):
    t = i / 20
    assert controller.update(t, engaged=True, valid=True, manual=2 <= t < 2.2, override=False,
                             set_speed=65 if t < 2 else 55, speed=54, accel=0,
                             model=model(t >= 3, ramp=t >= 3)) is None
  assert controller.branch.ambiguous


def test_pack_retains_other_fields_and_sets_one_button():
  from opendbc.can.packer import CANPacker
  from opendbc.car.ford.fordcan import create_button_msg
  from opendbc.can.dbc import DBC

  packer = CANPacker("ford_lincoln_base_pt")
  stock = dict.fromkeys(DBC("ford_lincoln_base_pt").name_to_msg["Steering_Data_FD1"].sigs, 0)
  stock.update(WiprFront_D_Stat=3, LghtAmb_D_Sns=2)
  for bus in (0, 2):
    addr, original, output_bus = create_button_msg(packer, bus, stock)
    assert (addr, output_bus) == (0x83, bus)
    for action, mask in (("SET+", 0x08), ("SET-", 0x10)):
      candidate = create_button_msg(packer, bus, stock, curve_button=action)
      expected = bytearray(original)
      expected[3] |= mask
      assert candidate == (addr, bytes(expected), output_bus)


def test_steering_pause_keeps_ceiling_and_reconciles_pending_step():
  controller = AcknowledgedCurveSpeed()
  _, presses = drive(controller, secs=1, curved_until=1, acknowledge=False)
  assert [button for _, button in presses] == ["SET-"]

  def update(now, set_speed, steering_only=False):
    return controller.update(now, engaged=True, valid=True, manual=False, override=False, set_speed=set_speed,
                             speed=58., accel=0., model=model(False), steering_only=steering_only)

  assert update(1., 64, steering_only=True) is None
  assert not controller.fault and controller.pending is None and controller.ceiling == 65
  assert update(1.3, 64, steering_only=True) is None
  assert update(2.79, 64) is None
  assert update(2.8, 64) is None
  assert update(4.31, 64) == "SET+"
  assert update(4.4, 65) is None
  assert update(6., 65) is None


def test_missing_ack_during_steering_pause_still_faults():
  controller = AcknowledgedCurveSpeed()
  _, presses = drive(controller, secs=1, curved_until=1, acknowledge=False)
  sent_at, button = presses[0]
  assert button == "SET-"
  assert controller.update(sent_at + 1.01, engaged=True, valid=True, manual=False, override=False,
                           steering_only=True, set_speed=65, speed=59, accel=0, model=model(False)) is None
  assert controller.fault


def missed_second_reduction():
  controller = AcknowledgedCurveSpeed()
  final, presses = drive(controller, secs=1, curved_until=1)
  assert final == 64
  assert [action for _, action in presses] == ["SET-"]
  assert recovery_update(controller, 1.71, 64, turn=True) == "SET-"
  return controller, 1.71


def recovery_update(controller, now, set_speed, *, turn=False, steering=False, manual=False, valid=True):
  return controller.update(now, engaged=True, valid=valid, manual=manual, override=False, set_speed=set_speed,
                           speed=62., accel=0., model=model(turn), steering_only=steering)


def test_missed_reduction_restores_only_acknowledged_step_after_clear():
  controller, sent_at = missed_second_reduction()
  assert recovery_update(controller, sent_at + 1.01, 64, turn=True) is None
  assert recovery_update(controller, 4., 64, turn=True) is None
  assert not controller.fault
  assert recovery_update(controller, 5., 64, steering=True) is None
  assert recovery_update(controller, 6., 64) is None
  assert recovery_update(controller, 6.6, 64) is None
  assert recovery_update(controller, 8.11, 64) == "SET+"
  assert recovery_update(controller, 8.2, 65) is None
  assert recovery_update(controller, 10., 65) is None


@pytest.mark.parametrize("interrupt", ["late_ack", "manual", "stale", "missed_restore"])
def test_recovery_stops_on_uncertain_set_speed_or_driver_intervention(interrupt):
  controller, sent_at = missed_second_reduction()
  assert recovery_update(controller, sent_at + 1.01, 64, turn=True) is None
  assert recovery_update(controller, 4., 64) is None
  if interrupt == "missed_restore":
    assert recovery_update(controller, 5.51, 64) == "SET+"
    assert recovery_update(controller, 6.52, 64) is None
  else:
    assert recovery_update(controller, 4.1, 63 if interrupt == "late_ack" else 64,
                           manual=interrupt == "manual", valid=interrupt != "stale") is None
  assert recovery_update(controller, 8., 63 if interrupt == "late_ack" else 64) is None
  assert recovery_update(controller, 10., 63 if interrupt == "late_ack" else 64) is None


def test_nonsteering_override_cannot_rebase_below_original_ceiling():
  controller = AcknowledgedCurveSpeed()
  assert drive(controller, secs=1, curved_until=1)[0] == 64
  assert recovery_update(controller, 1., 64, turn=True) is None
  assert controller.update(1.1, engaged=True, valid=True, manual=False, override=True,
                           set_speed=64, speed=63, accel=0, model=model(True)) is None
  for i in range(200):
    assert recovery_update(controller, 3.+i*.05, 64, turn=True) is None
  assert controller.update(14., engaged=False, valid=True, manual=False, override=False,
                           set_speed=64, speed=63, accel=0, model=model(False)) is None
  assert not controller.fault
  assert recovery_update(controller, 15., 64, turn=True) is None
  assert recovery_update(controller, 15.1, 64, turn=True) is None
  assert recovery_update(controller, 15.71, 64, turn=True) == "SET-"


def test_lane_change_pauses_and_restores_known_original_ceiling():
  controller = AcknowledgedCurveSpeed()
  assert drive(controller, secs=1, curved_until=1)[0] == 64
  assert recovery_update(controller, 1., 64, turn=True) is None
  for now in (1.1, 1.8, 2.2):
    assert controller.update(now, engaged=True, valid=True, manual=False, override=False, lane_change=True,
                             set_speed=64, speed=62, accel=0, model=model(False)) is None
    assert controller.ceiling == 65
  assert recovery_update(controller, 3.69, 64) is None
  assert recovery_update(controller, 3.71, 64) is None
  assert recovery_update(controller, 5.22, 64) == "SET+"
  assert recovery_update(controller, 5.3, 65) is None
  assert recovery_update(controller, 7., 65) is None

def test_long_reduction_stops_at_minimum_and_restores_exactly():
  controller = AcknowledgedCurveSpeed()
  set_speed = 65
  settings = [set_speed]
  for i in range(2200):
    now = i / 20
    frame = model(now < 52.)
    if now < 52.:
      frame.orientationRate.z = [.45] * len(frame.orientationRate.t)
    press = controller.update(now, engaged=True, valid=True, manual=False, override=False,
                              set_speed=set_speed, speed=62., accel=0., model=frame)
    if press is not None:
      set_speed += 1 if press == "SET+" else -1
      settings.append(set_speed)
  assert min(settings) == 25
  assert max(settings) == settings[-1] == 65


def test_curve_uses_far_model_samples():
  frame = model(False)
  frame.orientationRate.t = [10. * (i / 32) ** 2 for i in range(33)]
  frame.orientationRate.z = [.15 if t >= 6. else 0. for t in frame.orientationRate.t]
  frame.velocity.x = [60 * MPH] * 33
  curve = measure_curve(frame)
  assert curve is not None and 6. <= curve.first_horizon_s <= 10.


def test_faster_step_still_requires_ack():
  controller = AcknowledgedCurveSpeed()
  _, presses = drive(controller, secs=1, curved_until=1, acknowledge=False)
  sent_at, _ = presses[0]
  assert recovery_update(controller, sent_at + .51, 65, turn=True) is None
  assert recovery_update(controller, sent_at + .52, 64, turn=True) == "SET-"

def test_comfort_preserves_reference_approach_and_lowers_tight_bend_target():
  m = model(True)
  curve = measure_curve(m)
  reference = (1.8 / curve.curvature) ** .5 / MPH
  comfort = comfort_approach_mph(m, curve, 60, 0.)
  assert comfort < reference
  m.orientationRate.t = [t + 6. for t in m.orientationRate.t]
  far_curve = measure_curve(m)
  assert comfort_approach_mph(m, far_curve, 60, 0.) == pytest.approx(reference)


def test_slowdown_forecast_can_advance_comfort_target():
  m = model(True)
  m.orientationRate.z[:8] = [0.] * 8
  curve = measure_curve(m)
  assert comfort_approach_mph(m, curve, 60, 2.) < comfort_approach_mph(m, curve, 60, 0.)

def test_comfort_does_not_start_reduction_without_reference_curve_demand():
  controller = AcknowledgedCurveSpeed().virtual
  curve = Curve(1.8 / (60 * MPH) ** 2, 1, 2., .9)
  for i in range(20):
    _, _, button, _ = controller.update(i * .1, 60., curve, True, False, False, speed_mph=60., accel_mps2=0., comfort_mph=53.)
    assert button is None
