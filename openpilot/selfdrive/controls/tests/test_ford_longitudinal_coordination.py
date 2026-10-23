import unittest

from opendbc.car import structs
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.tests.test_longitudinal_coordination import LongitudinalHarness
from opendbc.car.ford.values import CAR
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState


class TestFordLongitudinalRestart(unittest.TestCase):
  def test_stopping_to_pid_with_capped_positive_demand_releases_brake(self):
    cp = CarInterface.get_non_essential_params(CAR.FORD_EXPEDITION_MK4)
    limits = CarInterface.get_pid_accel_limits(cp, 10., 10.)
    self.assertEqual(float(limits[1]), .2)
    control = LongControl(cp)
    state = structs.CarState(vEgo=0., aEgo=0.)
    state.cruiseState.standstill = True
    control.update(True, state, 0., True, limits)
    self.assertEqual(control.long_control_state, LongCtrlState.stopping)
    h = LongitudinalHarness()
    h.step(-.2, phase='stopping', frames=100)
    state.cruiseState.standstill = False
    for _ in range(50):
      demand = float(control.update(True, state, .2, False, limits))
      self.assertEqual(control.long_control_state, LongCtrlState.pid)
      values = h.step(demand, phase='pid')
    self.assertEqual(values['AccBrkDecel_B_Rq'], 0)
    self.assertGreater(values['AccPrpl_A_Rq'], 0)
