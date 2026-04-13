#!/usr/bin/env python3
"""apa: bypass registration.

Upstream registration posts to v2/pilotauth/ on api.comma.ai, which requires
a public key, IMEI, serial, and a JWT. This branch runs fully offline, so we
short-circuit to a local-only dongle_id derived from the device serial and
never touch the network. Nothing else in openpilot cares whether the id is
real — it's just used as a tag on cloudlog messages and local filenames.
"""
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"


def is_registered_device() -> bool:
  # Treat every device as registered for the purposes of this branch.
  return True


def register(show_spinner=False) -> str | None:
  params = Params()
  dongle_id = params.get("DongleId")
  if dongle_id:
    return dongle_id

  # Derive a stable local id from the hardware serial so logs and params
  # stay consistent across reboots.
  try:
    serial = HARDWARE.get_serial() or "local"
  except Exception:
    serial = "local"
  dongle_id = f"apa-{serial}"
  params.put("DongleId", dongle_id)
  return dongle_id


if __name__ == "__main__":
  print(register())
