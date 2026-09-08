"""Host-only, in-memory experimental transport. Never creates a socket or Panda.

Uses the production observer, but treats rejection of a simulated A3 publication
as a persistent calculation fault. Production imports neither this tool nor a
transport factory. Compiled safety results have no interface into this class.
"""
from openpilot.selfdrive.car.navigator_a3_runtime import RuntimeBridge


class SimulatedTransport:
  def __init__(self, route_id, capacity=2048):
    self.bridge = RuntimeBridge('shadow', [('ford', 2, 0)], route_id, 'synthetic', capacity)
    self.bridge.controls_ready = True

  @property
  def fault_reason(self):
    return self.bridge.fault_reason

  @property
  def calculation_fault_reason(self):
    if self.fault_reason:
      return self.fault_reason
    if not self.bridge.configuration_armed:
      return 'configuration_pending'
    health = self.bridge.observer.health.get(0)
    if health is None or not health['controlsAllowed'] or health['safetyRxChecksInvalid']:
      return 'permission_unavailable'
    return self.bridge.calculation_fault_reason

  def health(self, now_ns, state, valid=True):
    return self.bridge.panda(now_ns, valid, 0, state)

  def publish(self, now_ns, selection):
    frame = selection.experimental_frame
    if frame is None:
      return None
    address, data, bus = frame
    if address != 982 or len(data) != 8:
      raise ValueError('Simulation expects a complete Ford CAN-FD lateral frame')
    active = bool((data[0] >> 4) & 7)
    if active and self.calculation_fault_reason:
      return None
    # Once an experimental session has requested actuation, a driver-pause
    # neutral frame cannot disarm persistent permission-loss observation.
    self.bridge.active_request |= active
    return self.bridge.packet('published', now_ns, True, address, data, bus)

  def feedback(self, kind, now_ns, frame, *, provenance='synthetic', valid=True):
    if kind not in ('returned', 'rejected'):
      raise ValueError('Only explicit transport evidence is accepted')
    address, data, bus = frame
    return self.bridge.observe({'route_id': self.bridge.observer.route_id, 'provenance': provenance,
      'order': self.bridge.order, 'kind': kind, 't_ns': now_ns, 'valid': valid,
      'address': address, 'data_hex': bytes(data).hex(), 'dlc': len(data), 'bus': bus})
