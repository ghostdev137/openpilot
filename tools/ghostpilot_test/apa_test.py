#!/usr/bin/env python3
"""
Direct APA steering test — runs SAPP handshake and commands a steering angle.

Run on the comma 4 while the car is in park with the engine on:
  cd /data/openpilot && PYTHONPATH=/data/openpilot /usr/local/venv/bin/python tools/ghostpilot_test/apa_test.py

USE AT YOUR OWN RISK. Bypasses openpilot entirely.
Research/off-road use only.
"""
import time
import signal
import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp


TARGET_ANGLE_DEG = 20.0   # desired steering angle
APA_RATE_HZ = 50          # ParkAid_Data send rate
TEST_DURATION_S = 30      # how long to run

# SAPP handshake state
class S:
  IDLE = 0
  INIT = 1
  WAIT_OPEN = 2
  ANGLE_REQ = 3
  WAIT_ACTIVE = 4
  PARALLEL = 5
  COMPLETE = 6

STATE_NAMES = {0: "IDLE", 1: "INIT", 2: "WAIT_OPEN", 3: "ANGLE_REQ",
               4: "WAIT_ACTIVE", 5: "PARALLEL", 6: "COMPLETE"}
SAPP_NAMES = {0: "Closed", 1: "Open", 2: "Active", 3: "Fault"}


class APATest:
  def __init__(self):
    self.packer = CANPacker("ford_lincoln_base_pt")
    self.pm = messaging.PubMaster(['sendcan'])
    self.sm = messaging.SubMaster(['can', 'carState'])

    self.state = S.IDLE
    self.frame_counter = 0
    self.sapp_state = 0  # from PSCM
    self.handshake_ts = None

    self.running = True
    signal.signal(signal.SIGINT, lambda *_: setattr(self, 'running', False))

  def read_sapp_state(self):
    """Parse EPAS_INFO (0x082) from live CAN to get SAPPAngleControlStat1."""
    self.sm.update(0)
    # Look at recent CAN messages
    try:
      for msg in messaging.drain_sock(self.sm.sock['can']):
        for c in msg.can:
          if c.address == 0x82 and c.src == 0:  # EPAS_INFO on main bus
            # SAPPAngleControlStat1: byte offset/bit from DBC: 23|2@0+
            # Big endian, starts at bit 23, 2 bits
            # Byte 2, bits 6-7
            self.sapp_state = (c.dat[2] >> 6) & 0x3
    except Exception:
      pass

  def step(self):
    """Update SAPP state machine. Returns (sapp_config, angle_req)."""
    # Fault reset
    if self.sapp_state == 3:
      print(f"  !! PSCM fault (state 3) — resetting")
      self.state = S.IDLE
      self.frame_counter = 0

    if self.state == S.IDLE:
      self.state = S.INIT
      self.frame_counter = 0
      return 70, False

    elif self.state == S.INIT:
      if self.sapp_state == 1:
        self.state = S.WAIT_OPEN
        self.frame_counter = 0
      return 70, False

    elif self.state == S.WAIT_OPEN:
      self.frame_counter += 1
      if self.frame_counter >= 5:
        self.state = S.ANGLE_REQ
        self.frame_counter = 0
      return 86, False

    elif self.state == S.ANGLE_REQ:
      self.frame_counter += 1
      if self.frame_counter >= 20:
        self.state = S.WAIT_ACTIVE
        self.frame_counter = 0
      return 86, True

    elif self.state == S.WAIT_ACTIVE:
      if self.sapp_state == 2:
        self.state = S.PARALLEL
        self.frame_counter = 0
      return 86, True

    elif self.state == S.PARALLEL:
      self.frame_counter += 1
      if self.frame_counter >= 3:
        self.state = S.COMPLETE
        self.frame_counter = 0
      return 224, True

    elif self.state == S.COMPLETE:
      return 16, True

    return 0, False

  def send_apa(self, sapp_config, angle_req, angle_deg):
    """Send ParkAid_Data (0x3A8) on bus 0."""
    values = {
      "ApaSys_D_Stat": 2 if angle_req else 0,
      "SAPPStatusCoding": sapp_config,
      "EPASExtAngleStatReq": 1 if angle_req else 0,
      "ExtSteeringAngleReq2": angle_deg,
    }
    msg = self.packer.make_can_msg("ParkAid_Data", 0, values)
    self.pm.send('sendcan', can_list_to_can_capnp([msg]))

  def run(self):
    print(f"APA Steering Test — target {TARGET_ANGLE_DEG}° for {TEST_DURATION_S}s")
    print(f"Sending ParkAid_Data at {APA_RATE_HZ}Hz")
    print()
    print("Ctrl+C to abort")
    print("-" * 60)

    period = 1.0 / APA_RATE_HZ
    start = time.monotonic()
    last_print = 0
    last_state = -1

    while self.running and (time.monotonic() - start) < TEST_DURATION_S:
      loop_start = time.monotonic()

      # Read PSCM state from CAN
      self.read_sapp_state()

      # Step state machine
      sapp_config, angle_req = self.step()

      # Command angle only once handshake is complete
      if self.state == S.COMPLETE:
        angle = TARGET_ANGLE_DEG
      else:
        angle = 0.0

      # Send it
      self.send_apa(sapp_config, angle_req, angle)

      # Print state changes + periodic status
      if self.state != last_state or (time.monotonic() - last_print) > 1.0:
        t = time.monotonic() - start
        print(f"[{t:5.1f}s] state={STATE_NAMES[self.state]:12s} "
              f"sapp_config={sapp_config:3d} angle_req={int(angle_req)} "
              f"pscm={SAPP_NAMES.get(self.sapp_state,'?'):7s} "
              f"cmd_angle={angle:+6.1f}°")
        last_print = time.monotonic()
        last_state = self.state

      # Sleep to next period
      elapsed = time.monotonic() - loop_start
      if elapsed < period:
        time.sleep(period - elapsed)

    # Zero out on exit
    print("-" * 60)
    print("Zeroing angle and stopping handshake...")
    for _ in range(10):
      self.send_apa(0, False, 0.0)
      time.sleep(0.02)
    print("Done.")


if __name__ == "__main__":
  APATest().run()
