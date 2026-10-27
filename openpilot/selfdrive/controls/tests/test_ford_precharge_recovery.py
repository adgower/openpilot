from opendbc.car.ford.longitudinal_coordination import Feedback


def test_actual_longcontrol_matches_signed_braking_through_release_and_drive():
  from types import SimpleNamespace as NS
  from dataclasses import replace
  from opendbc.car import structs
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl

  cp = structs.CarParams.new_message()
  cp.longitudinalTuning.kiBP = [0.0]
  cp.longitudinalTuning.kiV = [0.5]
  cp.stopAccel = -0.5
  a, b = LongControl(cp), LongControl(cp)
  cs = NS(vEgo=25.0, aEgo=-0.2, canValid=True, gasPressed=False, brakePressed=False, standstill=False, cruiseState=NS(standstill=False))
  for i in range(1, 100):
    t = 1.0 + i * 0.01
    f = Feedback(t - 0.005, 1, i, 0.1, -5.0, True, False, True, False, allocator_version=1, precharge=True, reference_brake=True, allocator_phase='brake')
    g = replace(f, brake=False, limited=True, allocator_phase='precharge', precharge_age=0.02, budget_remaining=0.009)
    if i >= 30:
      f = g = replace(f, brake=False, precharge=False, limited=True, reference_brake=False, allocator_phase='release')
    if i >= 60:
      f = g = replace(f, gas=0.1, limited=False, allocator_phase='drive')
    x = a.update(True, cs, 0.1, False, (-3.5, 2.0), feedback=f, now=t, epoch=1, sequence=i + 1, coordinated=True)
    y = b.update(True, cs, 0.1, False, (-3.5, 2.0), feedback=g, now=t, epoch=1, sequence=i + 1, coordinated=True)
    assert x == y and a.pid.i == b.pid.i and a.last_integral_frozen == b.last_integral_frozen
