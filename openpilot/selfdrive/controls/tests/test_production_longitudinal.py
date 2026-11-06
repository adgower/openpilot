import struct

import pytest

from opendbc.car import structs
from opendbc.car.ford.tests.test_coast_reference_consumer import context
from opendbc.car.ford.tests.test_history_release import controller
from openpilot.selfdrive.controls.lib.longcontrol import LongControl


def test_production_startup_is_explicit_and_retention_is_incompatible():
  from openpilot.selfdrive.car.longitudinal_context import production_reference_startup_enabled
  assert not production_reference_startup_enabled(supported=True, environ={})
  assert not production_reference_startup_enabled(supported=False, environ={'NAVIGATOR_COAST_PRODUCTION': '1'})
  assert production_reference_startup_enabled(supported=True, environ={'NAVIGATOR_COAST_PRODUCTION': '1'})
  with pytest.raises(ValueError):
    production_reference_startup_enabled(supported=True, environ={
      'NAVIGATOR_COAST_PRODUCTION': '1', 'NAVIGATOR_RECEIPT_RETENTION': '1'})


def test_shadow_update_preserves_actual_integral_and_lateral_request():
  c = controller(enabled=True)
  loc = LongControl(c.CP, reference_enabled=True, production_enabled=True, production_run_id=123)
  cc, cs, ns = context(20)
  cc.latActive = True
  cc.actuators.steeringAngleDeg = 4.25
  loc.pid.i = .075
  wire = structs.CarControl.Actuators.new_message().navigatorLongitudinal
  loc.production.prepare(cc, cs.out, .13, False, (-3.5, 2.), wire,
                         now_ns=ns, cutoff_ns=0, source_valid=True, urgent=False,
                         no_lead=True, radar_ns=ns, radar_valid=True)
  assert loc.pid.i == .075
  assert cc.latActive and cc.actuators.steeringAngleDeg == 4.25
  assert loc.production.nominal is not loc and loc.production.nominal.pid is not loc.pid
  assert cc.navigatorLongitudinal.productionRunId == 123
  assert cc.navigatorLongitudinal.nominalRawAcceleration == struct.unpack(
    'f', struct.pack('f', loc.production.nominal.last_output_accel))[0]
  assert cc.navigatorLongitudinal.nominalValid
  assert cc.navigatorLongitudinal.productionPoseMonoTime == cc.navigatorLongitudinal.poseMonoTime


def test_registry_persists_actual_reset_and_requires_final_request():
  c = controller(enabled=True)
  loc = LongControl(c.CP, reference_enabled=True, production_enabled=True, production_run_id=123)
  cc, cs, ns = context(20)
  cc.navigatorLongitudinal.targetAcceleration = .13
  wire = structs.CarControl.Actuators.new_message().navigatorLongitudinal
  loc.production.prepare(cc, cs.out, .13, False, (-3.5, 2.), wire,
                         now_ns=ns, cutoff_ns=ns, source_valid=True, urgent=False,
                         no_lead=True, radar_ns=ns, radar_valid=True)
  cc.actuators.accel = .17
  registry = loc.production.registry
  assert loc.production.finalize_control(cc)
  key = (int(cc.navigatorLongitudinal.epoch), int(cc.navigatorLongitudinal.sequence))
  assert key in registry.issued
  loc.reset()
  assert loc.production.registry is registry and key in registry.issued


def test_default_longcontrol_has_no_shadow_owner():
  c = controller(enabled=True)
  loc = LongControl(c.CP)
  assert loc.production is None
