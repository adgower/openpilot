import json
from tools.navigator_a3.transport_observer import run_file
from tools.navigator_a3.transport_replay import write_report


def test_report_uses_actual_observations_not_fixed_results(tmp_path):
  for name in ('A2', 'curve17', 'curve29-30'):
    folder = tmp_path / name
    folder.mkdir()
    (folder / 'transport-events.jsonl').write_text('')
    run_file(folder / 'transport-events.jsonl', folder)
  result = write_report(tmp_path, {'cases': []})
  assert result['recordings']['A2']['steering_rejection_observations'] == 0
  assert '74 steering' not in (tmp_path / 'index.html').read_text()
  assert json.loads((tmp_path / 'results.json').read_text()) == result


def test_unselected_curve_window_uses_all_native_timestamps(tmp_path):
  for name in ('A2', 'curve17', 'curve29-30'):
    folder = tmp_path / name
    folder.mkdir()
    (folder / 'transport-events.jsonl').write_text('')
    run_file(folder / 'transport-events.jsonl', folder)
  source = tmp_path / 'window.csv'
  source.write_text('run,t_ns\ncurve,100\ncurve,200\n')
  result = write_report(tmp_path, {'cases': [{'name': 'curve17', 'selected_admission_run': None,
                                            'raw_admission_command': ['--timeline', str(source)]}]})
  assert result['previous_replay_windows']['curve17']['start_ns'] == 100
  assert result['previous_replay_windows']['curve17']['end_ns'] == 200
