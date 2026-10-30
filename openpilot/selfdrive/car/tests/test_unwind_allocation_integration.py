"""Runtime factory/context/controller/packer integration with synthetic motion."""

import math
from types import SimpleNamespace as NS
import pytest
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.tests.test_shadow_intent_timing import setup_controller
from opendbc.car.ford.lateral_angle_very_low_speed_prototype import create_tracking_trial as previous_trial
from opendbc.car.ford.lateral_angle_unwind_allocation import create_tracking_trial as child_trial
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, FordFlags
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.car.tracking_context import create_tracking_trial, feed_tracking_context
from openpilot.selfdrive.car.calibration_context import stamp_calibration, feed_calibration_context


def expected_checksum(data):
  # Independently extract the six raw Ford checksum fields from the DBC layout.
  fields = [
    (data[0] >> 4) & 7,
    (data[7] >> 1) & 15,
    (data[2] << 3) | (data[3] >> 5),
    (data[6] << 3) | (data[7] >> 5),
    ((data[3] & 31) << 6) | (data[4] >> 2),
    ((data[4] & 3) << 8) | data[5],
  ]
  return ~sum(value + (value >> 8) for value in fields) & 255


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('condition', ['valid', 'driver', 'stale_motion'])
def test_selected_trial_reaches_real_packer_and_preserves_other_packets(sign, condition):
  current, packet, state = setup_controller(CarController, frame=0)
  previous, _, old_state = setup_controller(CarController, frame=0)
  trial = create_tracking_trial(current.CP, enabled=True, roll_mapping_enabled=True, high_speed_enabled=True)
  assert type(trial).__name__ == 'UnwindAllocation'
  current.lateral_angle = trial
  previous.lateral_angle = previous_trial(previous.CP, enabled=True)
  trial.input_state_valid = previous.lateral_angle.input_state_valid = True
  vm = VehicleModel(current.CP)
  vm.update_params(1.0, 22.5)
  params = NS(
    valid=True,
    sensorValid=True,
    angleOffsetValid=True,
    steerRatioValid=True,
    stiffnessFactorValid=True,
    roll=0.0,
    steerRatio=22.5,
    stiffnessFactor=1.0,
    angleOffsetDeg=-1.5,
  )
  command = packet.as_builder()
  speed = 46 * 0.44704
  changed = 0
  for i in range(26):
    now = 1_000_000_000 + i * 50_000_000
    curvature = sign * 0.003 * 0.97**i
    angle = math.degrees(vm.get_steer_from_curvature(-curvature, speed, 0.0)) - 1.5
    command.actuators.curvature = curvature
    command.actuators.steeringAngleDeg = angle
    stamp_calibration(command, params, now_ns=now - 1_000_000, source_ns=now - 5_000_000, state_ns=now - 10_000_000, speed=speed, valid=True)
    for controller, cs in [(current, state), (previous, old_state)]:
      cs.out.vEgo = cs.out.vEgoRaw = speed
      cs.out.canValid = True
      cs.out.steeringAngleDeg = angle + math.copysign(3.0, angle + 1.5)
      cs.out.steeringPressed = condition == 'driver' and i >= 12
      strategy = controller.lateral_angle
      feed_tracking_context(
        strategy,
        now_ns=now,
        state_ns=now,
        control_ns=now,
        control_valid=True,
        model_ns=now,
        model_valid=True,
        model=NS(meta=NS(laneChangeState='off')),
        state=cs.out,
      )
      feed_calibration_context(strategy, command, now_ns=now, control_ns=now, control_valid=True)
      strategy.set_motion(
        now_ns=now,
        sample_ns=now,
        received_ns=now,
        valid=not (condition == 'stale_motion' and i >= 12),
        yaw_rate=curvature * speed * 1.35,
        yaw_std=0.013,
        generation=0,
      )
      controller.frame = i * 5
    _, sends = current.update(command.as_reader(), state, now)
    _, old_sends = previous.update(command.as_reader(), old_state, now)
    assert [m for m in sends if m[0] != 982] == [m for m in old_sends if m[0] != 982]
    new = next(m[1] for m in sends if m[0] == 982)
    old = next(m[1] for m in old_sends if m[0] == 982)
    mask = [0, 255, 0, 31, 252, 0, 0, 0]
    assert new[1] == expected_checksum(new) and old[1] == expected_checksum(old)
    assert all(((a ^ b) & (~m & 255)) == 0 for a, b, m in zip(new, old, mask, strict=True))
    path = -((((new[3] & 31) << 6) | (new[4] >> 2)) * 0.0005 - 0.5)
    assert abs(path - trial.previous) <= 0.00025001
    changed += new != old
    if i >= 12 and condition != 'valid':
      assert trial.term == 0
    if i >= 12 and condition == 'driver' and trial.human_turn_active:
      assert (new[0] >> 4) & 7 == 0
  assert changed > 0


def test_child_factory_defaults_and_vehicle_prerequisites():
  cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
  cp.flags |= int(FordFlags.PINION_CURVATURE | FordFlags.LOW_SPEED_GAIN)
  assert child_trial(cp) is None
  for flag in [FordFlags.PINION_CURVATURE, FordFlags.LOW_SPEED_GAIN]:
    # CarParams is a Cap'n Proto builder; construct fresh parameters per case.
    missing = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
    missing.flags = cp.flags & ~int(flag)
    assert child_trial(missing, enabled=True) is None
  other = CarInterface.get_non_essential_params(CAR.FORD_F_150_MK14)
  other.flags |= int(FordFlags.PINION_CURVATURE | FordFlags.LOW_SPEED_GAIN)
  assert child_trial(other, enabled=True) is None


def test_checksum_audit_rejects_a_path_bit_change_without_recalculation():
  from opendbc.car.ford import fordcan

  controller, _, _ = setup_controller(CarController, frame=0)
  data = bytearray(fordcan.create_lat_ctl2_msg(controller.packer, controller.CAN, 1, 0.0, 0.02, 0.0, 0.0, 3)[1])
  assert data[1] == expected_checksum(data)
  data[4] ^= 4
  assert data[1] != expected_checksum(data)
