"""Host knowledge contract using the actual runtime observer; never an actuation policy.

Delays are synthetic test points, not physical response or latency limits.
Missing rejection evidence gives no finite detection guarantee.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeBridge

START_NS = 1_000_000_000
PAYLOAD = '141b5c2fa200801e'  # A2 frame from the recorded first-rejection fixture; events below are synthetic


def cases(delay_ms=50):
  if delay_ms <= 0:
    raise ValueError('delay must be positive')
  end = START_NS + delay_ms * 1_000_000

  def packet(kind, t, **changes):
    return dict(kind=kind, t_ns=t, valid=True, bus=0, address=982, dlc=8, data_hex=PAYLOAD, **changes)

  def health(t, **changes):
    value = dict(kind='health', t_ns=t, valid=True, panda_index=0, safetyTxBlocked=0, controlsAllowed=True,
                 safetyRxChecksInvalid=False, safetyModel='ford', safetyParam=3, alternativeExperience=0)
    value.update(changes)
    return value

  initial = [health(START_NS - 1), packet('published', START_NS)]
  rejection = packet('rejected', end)
  result = {
    'delayed_rejection': initial + [health(START_NS + 1), rejection, packet('returned', end + 1), health(end + 2)],
    'missing_echo': initial + [health(end)],
    'returned_only': initial + [packet('returned', end)],
    'aggregate_only': initial + [health(end, safetyTxBlocked=1)],
    'unrelated_rejection': initial + [dict(rejection, address=0x186)],
    'ambiguous_echo': initial + [packet('published', START_NS + 1), rejection],
  }
  for name, change in [('permission_loss', {'controlsAllowed': False}), ('invalid_health', {'valid': False}),
                       ('config_change', {'safetyParam': 2})]:
    result[name] = initial + [rejection, health(end + 1, **change), packet('returned', end + 2), health(end + 3)]
  return {name: [dict(e, order=i, route_id='synthetic-contract', provenance='synthetic') for i, e in enumerate(events)]
          for name, events in result.items()}


def observe_case(events, mode='shadow', capacity=2048):
  bridge = RuntimeBridge(mode, [('ford', 3, 0)], 'synthetic-contract', 'synthetic', capacity=capacity)
  # Explicit synthetic host input: enabled/controls-ready throughout. No MCU
  # state or acceptance is fed into the bridge. This is not a recorded drive.
  bridge.controls_ready = True
  bridge.active_request = True
  rows = []
  first_direct = latency = None
  for e in events:
    observation = bridge.observe(deepcopy(e))
    if observation['direct_steering_rejection'] and first_direct is None:
      first_direct = e['t_ns']
      latency = observation['publication_to_observation_ns']
    rows.append({'input': deepcopy(e), 'observation': observation, 'fault_reason': bridge.fault_reason,
                 'calculation_fault_reason': bridge.calculation_fault_reason,
                 'disengagement_requested': bridge.disengagement_requested})
  return {'mode': mode, 'rows': rows, 'first_direct_rejection_ns': first_direct,
          'first_direct_publication_latency_ns': latency, 'guaranteed_detection_bound_ns': None,
          'physical_response': 'unknown', 'active_lifecycle_validated': False,
          'scope': 'synthetic host transport observations; no compiled oracle input or vehicle actuation'}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', required=True, type=Path)
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  results = {mode: {name: observe_case(events, mode) for name, events in cases().items()} for mode in ('shadow', 'requested')}
  results['delay_sweep'] = {str(delay): observe_case(cases(delay)['delayed_rejection']) for delay in (5, 50, 100, 250)}
  import openpilot.selfdrive.car.navigator_a3_runtime as runtime
  results['source_hashes'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__), Path(runtime.__file__))}
  (args.output / 'transport-contract.json').write_text(json.dumps(results, indent=2) + '\n')
  print('Executed 18 mode/case combinations and four observation-delay examples; physical latency bound remains unknown.')


if __name__ == '__main__':
  main()
