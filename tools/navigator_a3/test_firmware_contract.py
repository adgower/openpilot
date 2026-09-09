from pathlib import Path
import importlib
import pytest

REPO=Path('/private/tmp/navigator-a3-opendbc')

@pytest.fixture(scope='module')
def result(tmp_path_factory):
  assert importlib.util.find_spec('tools.navigator_a3.firmware_contract') is not None
  from tools.navigator_a3.firmware_contract import audit
  return audit(REPO,tmp_path_factory.mktemp('firmware-contract'))


def test_shared_checks_match_historical_aggregate(result):
  assert result['differences'] == []
  assert result['positive_cases'] > 0
  assert result['rejected_cases'] > 0


def test_production_hook_checks_but_cannot_commit_angle(result):
  assert result['production_probe']['shared_called']
  assert result['production_probe']['angle_reference_unchanged']
  assert result['production_probe']['blocked']


def test_final_denial_discards_successful_probe_without_changing_a2(result):
  p=result['production_probe']
  assert p['shared_checks_passed'] and p['a2_history_preserved']
  assert p['a2_before_accepted'] and p['a2_after_accepted']
  assert p['a2_history_before'][0] != 0


def test_firmware_arithmetic_preserves_wire_quantization(result):
  assert result['math_parity_differences'] == 0
