from capnp import KjException

from opendbc.car.ford.values import CAR
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.structs import CarParams
from opendbc.car.vin import VIN_UNKNOWN


def migrate_previous_navigator_params(previous: bytes, current: CarParams) -> bytes:
  """Keep learned state when the same Navigator switches from its legacy profile to firmware identification."""
  if current.carFingerprint != CAR.LINCOLN_NAVIGATOR_MK4 or current.fingerprintSource != CarParams.FingerprintSource.fw:
    return previous
  if current.fuzzyFingerprint or current.carVin == VIN_UNKNOWN:
    return previous

  try:
    with CarParams.from_bytes(previous) as old:
      if (old.carFingerprint != CAR.FORD_EXPEDITION_MK4 or old.fingerprintSource != CarParams.FingerprintSource.fixed
          or old.carVin != current.carVin or old.wheelbase != current.wheelbase or old.steerRatio != current.steerRatio):
        return previous
      _, matches = match_fw_to_car(old.carFw, old.carVin, allow_fuzzy=False, log=False)
      if matches != {CAR.LINCOLN_NAVIGATOR_MK4}:
        return previous
      migrated = old.as_builder()
      migrated.carFingerprint = current.carFingerprint
      return migrated.to_bytes()
  except (KjException, ValueError):
    return previous
