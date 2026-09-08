from tools.navigator_a3.scheduler_replay import replay_supported_rows


def rows():
  return [{'route_id': 'fixture', 't_ns': t, 'unsupported_reason': None,
           'historical': {'fault_reason': 'direct_steering_rejection'},
           'synthetic': {'profile': 'expedition-provisional-v1', 'reason': 'limited'},
           'synthetic_input': {'now_ns': t, 'source_ns': t, 'speed_mps': 25., 'curvature_inv_m': .001,
                               'active': True, 'driver_pressed': False, 'valid': True,
                               'measurement_ns': t, 'measurement_valid': True, 'measured_curvature_inv_m': 0.}}
          for t in [1_000_000_000, 1_049_000_000, 1_100_000_000]]


def test_replay_waits_without_inventing_intermediate_input_or_counter():
  source = rows()
  result = list(replay_supported_rows(source))
  assert [r['t_ns'] for r in result] == [r['t_ns'] for r in source]
  assert result[1]['proposal'] is None and result[1]['frame'] is None
  assert result[2]['decision']['proposal_counter'] == 1
  assert all(r['historical_fault'] == 'direct_steering_rejection' for r in result)
  assert all(r['transport_acceptance'] == 'unknown' for r in result)


def test_coverage_hole_resets_hypothetical_stream_without_synthetic_frame():
  source = rows()
  source[1]['unsupported_reason'] = 'invalid_car_output_event'
  result = list(replay_supported_rows(source))
  assert result[1]['decision'] is None
  assert result[2]['segment_generation'] == 1
  assert result[2]['proposal']['path_angle_rad'] == result[0]['proposal']['path_angle_rad']
