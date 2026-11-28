from types import SimpleNamespace


from openpilot.selfdrive.car import vision_curve_context as context
from openpilot.selfdrive.car.vision_curve_recovery import AcknowledgedCurveSpeed


def lead(distance=113., relative=-1., confidence=.94):
  return SimpleNamespace(present=True, dRel=distance, vRel=relative, modelProb=confidence)


def test_wait_for_this_models_radar_when_an_existing_lead_is_fading():
  radar = SimpleNamespace(mdMonoTime=1, leadOne=lead(), leadTwo=lead(confidence=.706))
  model = SimpleNamespace(leadsV3=[SimpleNamespace(prob=.686), SimpleNamespace(prob=.637)])
  assert not context.following_press_ready(2, model, radar)
  radar.mdMonoTime = 2
  assert context.following_press_ready(2, model, radar)
  radar.mdMonoTime = 1
  model.leadsV3[0].prob = model.leadsV3[1].prob = .9
  assert context.following_press_ready(2, model, radar)


def test_missing_or_unusable_camera_slots_wait_for_paired_radar():
  radar = SimpleNamespace(mdMonoTime=1000, leadOne=lead(), leadTwo=lead())
  model = SimpleNamespace(leadsV3=[])
  assert not context.following_press_ready(1100, model, radar)
  model.leadsV3 = [SimpleNamespace(prob=.99), SimpleNamespace(prob=float("nan"))]
  assert not context.following_press_ready(1100, model, radar)
  model.leadsV3[1].prob = float("inf")
  assert not context.following_press_ready(1100, model, radar)
  radar.leadTwo.present = False
  assert context.following_press_ready(1100, model, radar)


def test_unpaired_weak_frame_does_not_spend_a_credit_before_radar_withdraws_permission():
  cruise = AcknowledgedCurveSpeed()
  cruise._start(80)
  cruise.transport.observed = cruise.observed = 72
  cruise.target = cruise.geometry_target = cruise.current_demand = 80.
  cruise.geometry_trimmed = cruise.model_ok = True
  cruise.geometry_clear_since = cruise.following_clear_since = 0.
  args = dict(engaged=True, valid=True, manual=False, override=False, speed=66., accel=0.,
              set_speed=72., model=None, cruise_engaged=True, early_hint_inhibited=True,
              forecast_authoritative=True, forecast_available=True)
  assert cruise.update(3., **args, following_clear=True, following_up_ready=False, can_assert=True) != "SET+"
  assert cruise.update(3.01, **args, following_clear=False, following_up_ready=True, can_assert=True) != "SET+"
  assert cruise.transport.up_debt == 0
  cruise.following_clear_since = 3.02
  for n in range(302, 450):
    button = cruise.update(n / 100., **args, following_clear=True, following_up_ready=True, can_assert=n == 440)
    assert (button == "SET+") == (n == 440)
  assert cruise.transport.up_debt == 1
