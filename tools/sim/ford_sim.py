#!/usr/bin/env python3
"""
Ford CAN simulator for ghostpilot.

Simulates a Ford vehicle on CAN so openpilot starts all processes
and you can test steering modes without a real car.

Usage:
  cd ~/openpilot
  FINGERPRINT="FORD_BRONCO_SPORT_MK1" python tools/sim/ford_sim.py

  # Or for Transit:
  FINGERPRINT="FORD_TRANSIT_MK5" python tools/sim/ford_sim.py
"""
import os
import sys
import time
import threading

os.environ.setdefault("FINGERPRINT", "FORD_BRONCO_SPORT_MK1")
os.environ["NOBOARD"] = "1"  # no real panda needed
os.environ["SIMULATION"] = "1"

from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.tools.sim.lib.simulated_car_ford import SimulatedCarFord

import cereal.messaging as messaging


def main():
  print(f"ghostpilot Ford simulator")
  print(f"  FINGERPRINT: {os.environ['FINGERPRINT']}")
  print(f"  Steering mode: check Params GhostpilotSteeringMode")
  print()

  # Set up params for simulation
  set_params_enabled()
  params = Params()
  params.put("DongleId", "ghostpilot-sim")

  # Print current steering mode
  mode = params.get("GhostpilotSteeringMode", return_default=True)
  mode_names = {0: "Lane Centering (STOCK)", 1: "APA", 2: "Lane Keep (LKA)"}
  print(f"  Steering mode: {mode_names.get(int(mode), 'unknown')} ({mode})")
  print()

  car = SimulatedCarFord()

  # Simulate driving at 30 mph with cruise engaged
  car.set_speed(48.0)  # 48 kph ~= 30 mph
  car.set_cruise(True, 80.0)

  rk = Ratekeeper(100)  # 100Hz like real CAN
  print("Sending CAN messages at 100Hz... (Ctrl+C to stop)")
  print("Waiting for openpilot to start...")

  started = False
  try:
    while True:
      # Send panda state every 0.5s
      if rk.frame % 50 == 0:
        car.send_panda_state()

      # Send CAN messages every frame
      car.send_can_messages()

      # Read openpilot commands
      car.update()

      # Status output
      if rk.frame % 200 == 0:
        car.sm.update(0)
        if car.sm.valid.get('selfdriveState', False):
          ss = car.sm['selfdriveState']
          if not started:
            print("openpilot started!")
            started = True
          print(f"  state={ss.state} enabled={ss.enabled} "
                f"speed={car.speed_kph:.0f}kph steer={car.steering_angle:.1f}deg "
                f"sapp={car.sapp_state}")

      rk.keep_time()

  except KeyboardInterrupt:
    print("\nStopped.")


if __name__ == "__main__":
  main()
