from types import SimpleNamespace

import pytest

from openpilot.selfdrive.car.vision_curve_speed import AcknowledgedCurveSpeed, MPH


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
  ceiling, presses = drive(AcknowledgedCurveSpeed(), secs=12)
  down = [t for t, button in presses if button == "SET-"]
  up = [t for t, button in presses if button == "SET+"]
  assert 0 < len(down) <= 5
  assert len(up) == len(down)
  assert min(up) >= 5.5
  assert ceiling == 65
  assert all(b[0] - a[0] >= 1 for a, b in zip(presses, presses[1:], strict=False))


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
