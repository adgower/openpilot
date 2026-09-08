from dataclasses import asdict

from tools.navigator_a3.shadow_revision_comparison import compare_rows, load_frozen


def rows():
  frozen = load_frozen('/private/tmp/navigator-a3-opendbc')
  state = frozen.State()
  for n, now in enumerate((1_000_000_000, 1_049_000_000, 1_100_000_000)):
    sample = frozen.Inputs(now, now, .002, 20., True, False, True, 0., now, True)
    proposal = frozen.update(frozen.PROFILES['expedition-provisional-v1'], state, sample)
    state = proposal.state
    yield {'route_id':'r', 't_ns':now, 'synthetic_input':asdict(sample), 'synthetic':asdict(proposal),
           'historical':{'fault_reason':'direct_steering_rejection', 'controller':{'output':{'reason':'invalid'}}},
           'unsupported_reason':None}


def test_same_inputs_frozen_parity_and_independent_jitter_history():
  source = list(rows())
  result = list(compare_rows(source, load_frozen('/private/tmp/navigator-a3-opendbc')))
  assert all(r['frozen_matches_saved'] for r in result)
  assert result[1]['frozen']['reason'] == 'cadence'
  assert result[1]['revised']['mode'] == 1
  assert all(r['historical_fault'] == 'direct_steering_rejection' for r in result)
  assert all(r['transport_acceptance'] == 'unknown' and not r['activation_allowed'] for r in result)
  assert source[1]['synthetic']['mode'] == 0


def test_duplicate_skipped_but_unsupported_gap_resets_both():
  source = list(rows())
  duplicate = dict(source[0], unsupported_reason='duplicate_or_regressing_controller_time', synthetic=None)
  gap = dict(source[0], unsupported_reason='invalid_car_output_event', synthetic=None)
  baseline = list(compare_rows(source, load_frozen('/private/tmp/navigator-a3-opendbc')))
  repeated = list(compare_rows([source[0], duplicate, *source[1:]], load_frozen('/private/tmp/navigator-a3-opendbc')))
  assert repeated[2]['revised'] == baseline[1]['revised']
  reset = list(compare_rows([source[0], gap, source[1]], load_frozen('/private/tmp/navigator-a3-opendbc')))
  assert reset[2]['frozen']['mode'] == 1
  assert reset[2]['revised']['state']['path_angle_rad'] == baseline[0]['revised']['state']['path_angle_rad']
  assert not reset[2]['frozen_matches_saved']  # altered fixture history is detected
