from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.ford.longitudinal_recovery import CommandRecovery
from opendbc.car.ford.longitudinal_reference_feedback import ReferenceContext
from openpilot.selfdrive.car.longitudinal_context import coast_reference_startup_enabled, stamp_allocator_request
from openpilot.selfdrive.controls.lib.longcontrol import LongControl


def cp():
  return SimpleNamespace(longitudinalTuning=SimpleNamespace(kiBP=[0.], kiV=[.1]), stopAccel=-.5)


def test_default_off_and_reset_preserve_choice():
  assert LongControl(cp()).reference_integral is None
  controller = LongControl(cp(), reference_enabled=True)
  policy = controller.reference_integral
  policy.begin(ReferenceContext(1, 2, 3, True, .01, True))
  policy.issue(2, 3, 1., .1, target_ns=999_000_000)
  controller.reset()
  assert controller.reference_integral is policy
  assert not policy.registry.issued
  assert not policy.current.eligible
  assert controller.command_recovery.nominal_policy is not None


def test_startup_and_versions():
  env = {'NAVIGATOR_COAST_REFERENCE': '1'}
  assert not coast_reference_startup_enabled(environ=env)
  assert coast_reference_startup_enabled(supported=True, environ=env)
  with pytest.raises(ValueError):
    coast_reference_startup_enabled(supported=True, environ={**env, 'NAVIGATOR_RECEIPT_RETENTION': '1'})
  with pytest.raises(ValueError):
    LongControl(cp(), retention_enabled=True, reference_enabled=True)
  cc = structs.CarControl()
  stamp_allocator_request(cc, 1_000_000_000)
  assert cc.navigatorLongitudinal.allocatorVersion == 3
  stamp_allocator_request(cc, 1_000_000_000, reference_enabled=True)
  assert cc.navigatorLongitudinal.allocatorVersion == 4
  stamp_allocator_request(cc, 1_000_000_000, reference_enabled=True,
                          feedback=SimpleNamespace(reason='bounded-release-reference-unknown:bad', valid=False))
  assert cc.navigatorLongitudinal.allocatorVersion == 3


def test_optional_nominal_policy_cannot_remove_local_constraint():
  calls = []
  recovery = CommandRecovery(nominal_policy=lambda *a, **k: calls.append((a, k)) or False)
  recovery.valid = True
  recovery.reduction = .1
  feedback = SimpleNamespace(brake=False, precharge=False, reference_brake=False, gas=0., requested=.1)
  assert recovery.freeze(1., 1, 1, .1, .1, .2, 5., 'pid', feedback, enabled=True)
  assert calls and recovery.counts['local_constraint_freeze'] == 1
  recovery.reset()
  assert recovery.nominal_policy is not None


def test_live_issue_binds_consumed_target_and_pose_context():
  controller = LongControl(cp(), reference_enabled=True)
  cs = SimpleNamespace(canValid=True, gasPressed=False, brakePressed=False, standstill=False,
                       cruiseState=SimpleNamespace(standstill=False), aEgo=0., vEgo=5.)
  context = ReferenceContext(1_000_000_000, 7, 1, True, .01, True)
  controller.update(True, cs, .1, False, (-3., 2.), now=1., epoch=7, sequence=1,
                    coordinated=True, reference_context=context, target_ns=990_000_000)
  issued = controller.reference_integral.registry.issued[(7, 1)]
  assert issued[0] == context.now_ns
  assert issued[1] == .1 and issued[2] is True and issued[4] == 990_000_000
  controller.update(True, cs, .1, False, (-3., 2.), now=1.01, epoch=7, sequence=2,
                    coordinated=True, target_ns=1_000_000_000)
  assert controller.reference_integral.registry.issued[(7, 2)][2] is False
