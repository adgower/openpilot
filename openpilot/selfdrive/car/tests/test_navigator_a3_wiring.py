from opendbc.car.structs import car
from openpilot.selfdrive.car.navigator_a3_runtime import add_inhibition_event


def test_schema_round_trip_reports_requested_inhibition():
  output = car.CarOutput.new_message()
  output.navigatorA3.version = 1
  output.navigatorA3.mode = 'requested'
  output.navigatorA3.inhibited = True
  output.navigatorA3.reason = 'physical_enforcement_unvalidated'
  output.navigatorA3.diagnosticsJson = '{"provenance":"synthetic"}'
  with car.CarOutput.from_bytes(output.to_bytes()) as read:
    assert read.navigatorA3.inhibited
    assert read.navigatorA3.reason == 'physical_enforcement_unvalidated'
    assert read.navigatorA3.version == 1


def test_requested_inhibition_uses_truthful_existing_steering_fault():
  output = car.CarOutput.new_message()
  events = []
  output.navigatorA3.mode = 'requested'
  output.navigatorA3.inhibited = True
  add_inhibition_event(output, events.append, 'steerUnavailable')
  assert events == ['steerUnavailable']
  output.navigatorA3.mode = 'shadow'
  add_inhibition_event(output, events.append, 'steerUnavailable')
  assert events == ['steerUnavailable']


def test_card_helpers_preserve_native_parser_age_and_inhibit_requested_control():
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.car.navigator_a3_runtime import prepare_controller, control_for_apply, RuntimeBridge
  evidence = []
  controller = NS(set_navigator_a3_evidence=lambda **kwargs:evidence.append(kwargs))
  parser = NS(ts_nanos={'Yaw_Data_FD1':{'VehYaw_W_Actl':100}, 'BrakeSysFeatures':{'Veh_V_ActlBrk':90}})
  ci = NS(CC=controller, can_parsers={'pt':parser})
  cs = NS(canValid=True, vehicleSensorsInvalid=False)
  sm = NS(logMonoTime={'carControl':110}, all_checks=lambda services:True)
  bridge = RuntimeBridge('requested', [('ford',4,0)], 'r','live')
  prepare_controller(ci, cs, sm, bridge)
  assert evidence[0]['source_ns'] == 110 and evidence[0]['measurement_ns'] == 90
  assert evidence[0]['measurement_valid']
  cc = car.CarControl.new_message(enabled=True, latActive=True, longActive=True)
  blocked = control_for_apply(cc, bridge)
  assert cc.enabled and cc.latActive and cc.longActive
  assert not blocked.enabled and not blocked.latActive and not blocked.longActive


def test_diagnostics_sanitize_invalid_numeric_evidence_without_crashing():
  from types import SimpleNamespace as NS
  import json
  from openpilot.selfdrive.car.navigator_a3_runtime import diagnostic_json, RuntimeBridge
  cc = NS(navigator_a3=NS(diagnostic={'measurement':float('nan')}))
  data = json.loads(diagnostic_json(cc, RuntimeBridge('shadow', [('ford',4,0)], 'r','live')))
  assert data['controller']['measurement'] is None
  assert data['nonfinite_values_present']


def test_replay_publications_cannot_acknowledge_historical_echoes():
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeBridge, observe_publication
  b = RuntimeBridge('shadow', [('ford',4,0)], 'r','recorded')
  msg = NS(logMonoTime=100, valid=True, sendcan=[NS(address=982, dat=bytes.fromhex('14237d0fa2008000'), src=0)])
  observe_publication(b, msg, replay=True)
  row = b.packet('returned', 101, True, 982, msg.sendcan[0].dat, 0, 128)
  assert row['candidate_orders'] == []
  assert row['receiver_acceptance'] == 'unknown'
  assert len(b.observer.publications) == 0
