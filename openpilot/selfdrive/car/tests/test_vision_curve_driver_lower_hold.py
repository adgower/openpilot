import pytest

from openpilot.selfdrive.car.tests.test_vision_curve_recovery import Rig
from openpilot.selfdrive.car.tests.test_vision_forecast_context import CaptureCruise, ContextFixture
from openpilot.selfdrive.car.vision_curve_speed import MPH


def samples(rig, duration, *, switch_time=None, **inputs):
  inputs.setdefault('lower_switch', False if not inputs.get('manual') else None)
  for _ in range(round(duration / .01)):
    stamp = (round(rig.t * 1e9) // 100_000_000 * 100_000_000 + 10_000_000) if switch_time is None else switch_time
    rig.run(.01, switch_ns=stamp, set_ns=round((rig.t + .01) * 1e9), set_samples=[rig.observed], **inputs)


def hold(rig, display, duration=.15, **values):
  rig.observed = display
  inputs = {'manual': True, 'user_action': 'set', 'user_set_direction': -1, 'lower_switch': True, 'clear': False}
  inputs.update(values)
  samples(rig, duration, **inputs)


def start():
  r = Rig().run(1., forecast=68, ack=False, clear=False)
  assert r.observed == 70 and r.c.transport.down_debt == 2
  return r


def test_confirmed_driver_lowering_preserves_delayed_decrements():
  r = start()
  mark = len(r.pulses)
  for display in (70, 69, 65, 60):
    hold(r, display)
  samples(r, .5, clear=False)
  assert not r.c.suspended and r.c.ceiling == 60 and r.c.transport.debt == {-1: 2, 1: 0}
  assert not r.pulses[mark:]
  r.observed -= 2  # Both older decrements arrive after the new selection.
  r.run(.15, clear=False, forecast=58).run(1., clear=True)
  assert not r.c.fault and not r.c.transport.pending and r.observed == r.c.ceiling == 60


@pytest.mark.parametrize('display', [70, 65])
def test_unchanged_or_unambiguous_initial_tap_keeps_existing_behavior(display):
  r = start()
  hold(r, display, duration=.07)
  samples(r, .5, clear=False)
  assert r.c.ceiling == display and not r.c.suspended and r.c.transport.down_debt == 2


@pytest.mark.parametrize('block', ['small_steps', 'pending_increment', 'cancel', 'increase', 'post_release_drop',
                                    'mixed_button', 'brake', 'invalid', 'cruise_off', 'stale_switch', 'last_sample_neutral',
                                    'gap_held', 'gap_release', 'gap_settle'])
def test_ambiguous_selection_without_a_continuous_physical_witness_stays_suspended(block):
  r = start()
  if block == 'pending_increment':
    r.c.transport.debt[1] = 1
  mark = len(r.pulses)
  hold(r, 70)
  hold(r, 69)
  if block == 'cancel':
    samples(r, .1, manual=True, user_action='cancel', clear=False)
  else:
    if block == 'gap_held':
      r.t += .45
    inputs = {}
    if block in ('mixed_button', 'last_sample_neutral'):
      inputs['lower_switch'] = None
    if block == 'brake':
      inputs.update(override=True, override_kind='brake')
    if block == 'invalid':
      inputs['valid'] = False
    if block == 'cruise_off':
      inputs['cruise_engaged'] = False
    if block == 'stale_switch':
      inputs['switch_time'] = round((r.t - .16) * 1e9)
    hold(r, 70 if block == 'increase' else 68 if block == 'small_steps' else 65, **inputs)
    hold(r, 67 if block == 'small_steps' else 60)
  if block in ('gap_settle', 'post_release_drop'):
    samples(r, .2, clear=False)
  if block in ('gap_release', 'gap_settle'):
    r.t += .45
  if block == 'post_release_drop':
    r.observed -= 1
  samples(r, .6, clear=False)
  assert (r.c.suspended or r.c.fault) and r.c.ceiling == 70 and not r.pulses[mark:]


def test_repeated_cached_set_sample_cannot_supply_two_independent_drops():
  r = start()
  hold(r, 70)
  hold(r, 69)
  single_sample = round((r.t + .01) * 1e9)
  for display in (65, 60):
    hold(r, display, duration=.02, switch_time=single_sample)
  samples(r, .6, clear=False)
  assert r.c.suspended and r.c.ceiling == 70


def test_small_steps_cannot_be_summed_to_make_one_large_drop_per_switch_frame():
  r = start()
  hold(r, 70)
  hold(r, 69)
  for values in ((68, 67, 66), (65, 64, 63)):
    for display in values:
      hold(r, display, duration=.02)
    hold(r, values[-1], duration=.04)
  samples(r, .6, clear=False)
  assert r.c.suspended and r.c.ceiling == 70


@pytest.mark.parametrize('failure', ['cached_release', 'no_fresh_neutral', 'timestamp_regressed', 'future_timestamp'])
def test_confirmation_requires_an_unbroken_stream_of_neutral_after_the_last_held_sample(failure):
  r = start()
  for display in (70, 69, 65, 60):
    hold(r, display)
  last_held = r.c.lower_hold.last_switch_ns
  if failure == 'no_fresh_neutral':
    samples(r, .2, clear=False)
    stamp = r.c.lower_hold.last_switch_ns
  else:
    stamp = last_held
    if failure == 'timestamp_regressed':
      stamp -= 1
    if failure == 'future_timestamp':
      stamp = round((r.t + .05) * 1e9)
  samples(r, .5, clear=False, switch_time=stamp)
  samples(r, .5, clear=False)
  assert r.c.suspended and r.c.ceiling == 70 and r.c.transport.down_debt == 2


@pytest.mark.parametrize('other', ['CcAsllButtnResPress', 'CcAslButtnCnclResPress', 'AccButtnGapTogglePress',
                                   'CcAslButtnSetIncPress', 'TjaButtnOnOffPress', 'final_neutral', 'released_repressed', 'software_resume', None])
def test_context_requires_an_exclusive_final_physical_set_minus(other):
  f = ContextFixture()
  switches = f.parser.vl['Steering_Data_FD1']
  switch_batch = f.parser.vl_all['Steering_Data_FD1']
  key = 'CcAslButtnSetDecPress'
  switches[key] = int(other != 'final_neutral')
  switch_batch[key] = {'final_neutral': [1, 0], 'released_repressed': [1, 0, 1]}.get(other, [1])
  if other == 'software_resume':
    f.control.cruiseControl.resume = True
  elif other not in (None, 'final_neutral', 'released_repressed'):
    switch_batch[other] = [1, 0]
    switches[other] = 0
  values = f.update(.1)
  assert (values['lower_switch'] is True) == (other is None)
  if other == 'CcAslButtnCnclResPress':
    assert values['user_action'] == 'cancel'


def test_released_same_batch_tap_still_follows_existing_driver_selection():
  f = ContextFixture()
  key = 'CcAslButtnSetDecPress'
  f.parser.vl['Steering_Data_FD1'][key] = 0
  f.parser.vl_all['Steering_Data_FD1'][key] = [1, 0]
  values = f.update(.1)
  assert values['manual'] and values['user_action'] == 'set' and values['user_set_direction'] == -1
  assert values['lower_switch'] is None


@pytest.mark.parametrize('case', ['continuous', 'neutral_rebound', 'regressing_timestamp', 'grouped_small_drops',
                                  'single_batch', 'drop_after_last_held_frame'])
def test_native_display_batches_on_ten_hz_context_calls(case):
  f = ContextFixture(comfort=False)
  c = start().c
  f.context.cruise = CaptureCruise(c)
  key = 'CcAslButtnSetDecPress'

  def frame(index, held, displays, lag_ns=0):
    f.parser.vl['Steering_Data_FD1'][key] = int(held)
    f.parser.vl_all['Steering_Data_FD1'][key] = [int(held)]
    f.parser.vl_all['EngBrakeData'] = {'Veh_V_DsplyCcSet': displays}
    f.state.cruiseState.speedCluster = displays[-1] * MPH
    f.update(index * .1, display_lag_ns=lag_ns)
    assert f.context.cruise.last_action not in ('SET-', 'SET+')

  frame(0, False, [70])
  frame(1, True, [70])
  frame(2, True, [69])
  third, fourth = {
    'grouped_small_drops': ([68, 67, 65], [63, 62, 60]),
    'single_batch': ([69], [65, 60]),
  }.get(case, ([65], [60]))
  frame(3, True, third)
  frame(4, True, fourth)
  frame(5, case != 'drop_after_last_held_frame', [60])
  frame(6, False, [60])
  frame(7, False, [55, 60] if case == 'neutral_rebound' else [60], 110_000_000 if case == 'regressing_timestamp' else 0)
  for i in range(8, 12):
    frame(i, False, [60])
  confirmed = case == 'continuous'
  assert c.ceiling == (60 if confirmed else 70)
  assert c.suspended != confirmed
  assert c.transport.debt == {-1: 2, 1: 0}
