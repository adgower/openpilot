"""Actual route packet fixture exercises the unchanged production fault latch."""
import json
from pathlib import Path
from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeBridge

def test_first_rejection_permission_loss_and_return_do_not_clear_fault():
  f=Path(__file__).with_name('fixtures')/'shadow_drive_first_rejection.json'
  data=json.loads(f.read_text())
  bridge=RuntimeBridge('shadow',[('ford',3,0)],data['route_id'],'recorded')
  bridge.controls_ready=True
  bridge.active_request=True
  saw_rejection=False
  for e in data['events']:
    bridge.observe(e)
    if e['kind']=='rejected':saw_rejection=True
    if saw_rejection:assert bridge.fault_reason=='direct_steering_rejection'
  assert saw_rejection
  assert not bridge.disengagement_requested
  assert data['events'][2]['t_ns']-data['events'][1]['t_ns']==5_508_302
  assert any(e['kind']=='health' and not e['controlsAllowed'] for e in data['events'])
  assert any(e['kind']=='returned' for e in data['events'])
