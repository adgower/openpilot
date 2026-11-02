"""Offroad pilot availability and bounded session permits; no vehicle transport."""

import argparse
import json
import time
import uuid
from openpilot.selfdrive.car.prediction_pilot import ENABLE, PERMIT, CANCEL, STATUS


def command(params, action, *, enable=False, condition=None, duration=3.0, wait=30.0, clock=time.monotonic_ns):
  if action in ('worker-enable', 'worker-disable'):
    if not params.get_bool('IsOffroad'):
      raise ValueError('worker availability changes require offroad state')
    # Disable first; native storage failures do not necessarily raise.
    params.put_bool(ENABLE, False, block=True)
    if params.get_bool(ENABLE):
      raise ValueError('could not disable worker availability')
    for key in (PERMIT, CANCEL, STATUS):
      params.remove(key)
      if params.get(key) is not None:
        raise ValueError('could not clear stale pilot mailbox')
    wanted = action == 'worker-enable'
    if not params.get_bool('IsOffroad'):
      raise ValueError('offroad state lost while changing worker availability')
    if wanted:
      try:
        params.put_bool(ENABLE, True, block=True)
        if not params.get_bool(ENABLE):
          raise ValueError('could not enable worker availability')
        if not params.get_bool('IsOffroad'):
          raise ValueError('offroad state lost while enabling worker')
      except Exception as exc:
        params.put_bool(ENABLE, False, block=True)
        if params.get_bool(ENABLE):
          raise ValueError('worker enable failed and availability could not be cleared') from exc
        raise
    return {'worker_enabled_for_next_session': wanted, 'test_armed': False,
            'note': 'Applies at next onroad startup; opt-in clears on return offroad or manager restart'}

  now = clock()
  if action == 'cancel':
    params.put(CANCEL, json.dumps({'id': uuid.uuid4().hex, 'issued_ns': now}), block=True)
    return {'cancel_requested': True}
  raw = params.get(STATUS)
  status = json.loads(raw) if raw else {}
  if action == 'status':
    return status
  if action != 'arm' or enable is not True:
    raise ValueError('arm requires --enable-pilot')
  if not status.get('ready') or not status.get('parked') or not 0 <= now - status.get('t_ns', 0) <= 500_000_000:
    raise ValueError('fresh ready stationary-Park status required')
  if not 0 <= now - status.get('io_ns', 0) <= 250_000_000 or status.get('state') in ('waiting', 'active'):
    raise ValueError('worker stale or trial busy')
  payload = {
    'schema': 1,
    'enabled': True,
    'trial_id': uuid.uuid4().hex,
    'condition': condition,
    'session_id': status['session_id'],
    'source_id': status['source_id'],
    'issued_ns': now,
    'duration_s': duration,
    'wait_s': wait,
  }
  from openpilot.selfdrive.car.prediction_pilot import PermitBroker

  PermitBroker(status['session_id'], status['source_id']).accept(payload, now, parked=True)
  params.put(PERMIT, json.dumps(payload), block=True)
  return {'requested': payload['trial_id'], 'condition': condition, 'note': 'Await card acknowledgment; not activation proof'}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('action', choices=['status', 'worker-enable', 'worker-disable', 'arm', 'cancel'])
  parser.add_argument('--enable-pilot', action='store_true')
  parser.add_argument('--condition', choices=['A', 'B'])
  parser.add_argument('--duration', type=float, default=3.0)
  parser.add_argument('--wait', type=float, default=30.0)
  args = parser.parse_args()
  from openpilot.common.params import Params

  try:
    result = command(Params(), args.action, enable=args.enable_pilot, condition=args.condition, duration=args.duration, wait=args.wait)
  except (ValueError, KeyError) as exc:
    parser.error(str(exc))
  print(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
