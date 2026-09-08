"""Real card methods and serialized messages; no Car/device initialization."""
import json
from types import SimpleNamespace as NS

from opendbc.car import get_safety_config
from opendbc.car.structs import car
from opendbc.car.ford.tests.test_navigator_a3_runtime import controller, sample
from openpilot.cereal import messaging
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeBridge, add_inhibition_event


class Publisher:
  def __init__(self):
    self.messages = {}

  def send(self, service, message):
    self.messages[service] = message if isinstance(message, bytes) else message.to_bytes()

  def read(self, service):
    return messaging.log_from_bytes(self.messages[service])


def harness(monkeypatch, mode):
  cc = controller(monkeypatch, mode, safetyConfigs=[get_safety_config(car.CarParams.SafetyModel.ford, 2)])
  command, cs = sample(25)
  cs.out.canValid = True
  ci = NS(CC=cc, can_parsers={'pt': NS(ts_nanos={
    'Yaw_Data_FD1': {'VehYaw_W_Actl': 950_000_000},
    'BrakeSysFeatures': {'Veh_V_ActlBrk': 900_000_000},
  })}, init=lambda *args: None)
  applied = []

  def apply(control, now):
    applied.append(control)
    return cc.update(control, cs, now)

  ci.apply = apply
  instance = Car.__new__(Car)
  instance.CI = ci
  instance.CP = cc.CP
  instance.initialized_prev = False
  instance.can_callbacks = (None, None)
  instance.params = NS(put_bool=lambda *args: None)
  instance.navigator_a3_bridge = RuntimeBridge(mode, [('ford', 2, 0)], 'fixture', 'live') if mode != 'a2' else None
  instance.sm = NS(frame=1, all_alive=lambda services: True, all_checks=lambda services: True,
                   logMonoTime={'carControl': 975_000_000})
  instance.pm = Publisher()
  instance.can_rcv_cum_timeout_counter = 0
  instance.rk = NS(remaining=0.)
  monkeypatch.setattr('openpilot.selfdrive.car.card.time.monotonic', lambda: 1.)
  monkeypatch.setattr('openpilot.selfdrive.car.card.REPLAY', False)
  # Real wire-schema reader exercises control_for_apply rather than a fake builder.
  command = car.CarControl.new_message(**command.to_dict()).as_reader()
  instance.controls_update(cs.out, command)
  instance.state_publish(cs.out, None)
  return instance, command, applied


def frames(instance):
  return [(x.address, bytes(x.dat), x.src) for x in instance.pm.read('sendcan').sendcan]


def test_real_card_default_and_shadow_publish_identical_can(monkeypatch):
  default, _, _ = harness(monkeypatch, 'a2')
  shadow, _, _ = harness(monkeypatch, 'shadow')
  assert frames(default) == frames(shadow)
  status = shadow.pm.read('carOutput').carOutput.navigatorA3
  assert status.mode == 'shadow' and not status.inhibited
  diagnostic = json.loads(status.diagnosticsJson)['controller']
  assert diagnostic['input']['source_ns'] == 975_000_000
  assert diagnostic['output']['measurement_age_ns'] == 100_000_000
  assert len(shadow.navigator_a3_bridge.observer.publications) > 0


def test_real_card_requested_publishes_neutral_and_status(monkeypatch):
  instance, original, applied = harness(monkeypatch, 'requested')
  assert original.enabled and original.latActive and original.longActive
  assert not applied[0].enabled and not applied[0].latActive and not applied[0].longActive
  steering = [data for address, data, bus in frames(instance) if address == 0x3d6]
  assert steering
  for data in steering:
    assert (data[0] >> 4) & 7 == 0
    assert ((data[2] << 3) | (data[3] >> 5)) == 1000
    assert (((data[3] & 31) << 6) | (data[4] >> 2)) == 1000
  accel_frames = [data for address, data, bus in frames(instance) if address == 0x186]
  assert accel_frames
  for data in accel_frames:
    assert (((data[6] & 3) << 8) | data[7]) == 0  # inactive propulsion request
    assert (((data[2] & 3) << 8) | data[3]) == 0  # inactive predicted propulsion
    assert (data[6] & 0xc0) == 0  # no brake precharge/deceleration request
  status = instance.pm.read('carOutput').carOutput.navigatorA3
  assert status.mode == 'requested' and status.inhibited
  assert status.reason == 'physical_enforcement_unvalidated'
  events = []
  add_inhibition_event(instance.pm.read('carOutput').carOutput, events.append, 'steerUnavailable')
  assert events == ['steerUnavailable']
