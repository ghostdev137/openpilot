#!/usr/bin/env python3
"""
Ford CAN simulator for ghostpilot.

Tests all 3 steering modes by running card + controlsd + selfdrived
alongside a simulated Ford CAN bus.

Usage:
  cd ~/openpilot && source .venv/bin/activate
  python tools/sim/ford_sim.py [--mode stock|apa|lka]
"""
import os
import sys
import time
import argparse
import subprocess
import signal

os.environ.setdefault("FINGERPRINT", "FORD_BRONCO_SPORT_MK1")
os.environ["NOBOARD"] = "1"
os.environ["SIMULATION"] = "1"

from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.test.helpers import set_params_enabled
from openpilot.tools.sim.lib.simulated_car_ford import SimulatedCarFord

import cereal.messaging as messaging


PROCS = [
  ("card", [sys.executable, "-m", "openpilot.selfdrive.car.card"]),
  ("controlsd", [sys.executable, "-m", "openpilot.selfdrive.controls.controlsd"]),
  ("selfdrived", [sys.executable, "-m", "openpilot.selfdrive.selfdrived.selfdrived"]),
  ("plannerd", [sys.executable, "-m", "openpilot.selfdrive.controls.plannerd"]),
]


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--mode", choices=["stock", "apa", "lka"], default=None,
                      help="Override steering mode (default: use param)")
  parser.add_argument("--speed", type=float, default=48.0, help="Speed in kph (default 48)")
  parser.add_argument("--duration", type=float, default=30.0, help="Run duration in seconds")
  args = parser.parse_args()

  set_params_enabled()
  params = Params()
  params.put("DongleId", "ghostpilot-sim")

  if args.mode:
    mode_map = {"stock": 0, "apa": 1, "lka": 2}
    params.put("GhostpilotSteeringMode", mode_map[args.mode])

  mode = params.get("GhostpilotSteeringMode", return_default=True)
  mode_names = {0: "STOCK (Lane Centering)", 1: "APA", 2: "LKA (Lane Keep)"}
  print(f"ghostpilot Ford simulator")
  print(f"  FINGERPRINT: {os.environ['FINGERPRINT']}")
  print(f"  Steering mode: {mode_names.get(int(mode), '?')} ({mode})")
  print(f"  Speed: {args.speed} kph")
  print(f"  Duration: {args.duration}s")
  print()

  # Start car sim
  car = SimulatedCarFord()
  car.set_speed(args.speed)
  car.set_cruise(True, 80.0)

  # Pre-seed CAN so card can fingerprint
  print("Seeding CAN bus...")
  rk = Ratekeeper(100)
  for _ in range(300):  # 3 seconds of CAN
    if rk.frame % 50 == 0:
      car.send_panda_state()
    car.send_can_messages()
    rk.keep_time()

  # Start openpilot processes
  print("Starting openpilot processes...")
  procs = {}
  env = os.environ.copy()
  for name, cmd in PROCS:
    p = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    procs[name] = p
    print(f"  {name} pid={p.pid}")

  time.sleep(2)

  # Subscribe to outputs
  sm = messaging.SubMaster(['sendcan', 'selfdriveState', 'controlsState', 'carState', 'carControl'])

  print()
  print(f"Running for {args.duration}s...")
  print("-" * 70)

  start_time = time.time()
  connected = False

  try:
    while time.time() - start_time < args.duration:
      if rk.frame % 50 == 0:
        car.send_panda_state()

      car.send_can_messages()
      sm.update(0)

      # Check for dead processes
      for name, p in procs.items():
        if p.poll() is not None:
          stderr = p.stderr.read().decode()[-300:]
          print(f"\n{name} DIED (code {p.returncode}): {stderr}")

      if sm.updated['carState'] and not connected:
        cs = sm['carState']
        print(f"car connected! speed={cs.vEgo:.1f}m/s fingerprint detected")
        connected = True

      if sm.updated['selfdriveState'] and rk.frame % 200 == 0:
        ss = sm['selfdriveState']
        cc_str = ""
        if sm.valid.get('controlsState', False):
          cs = sm['controlsState']
          cc_str = f"curv={cs.curvatureDesired:.5f}"

        sendcan_n = len(sm['sendcan']) if sm.updated.get('sendcan', False) else 0
        print(f"  [{time.time()-start_time:5.1f}s] state={ss.state:12s} enabled={ss.enabled} "
              f"speed={car.speed_kph:.0f}kph steer={car.steering_angle:.1f}° "
              f"{cc_str} sendcan={sendcan_n}")

      rk.keep_time()

  except KeyboardInterrupt:
    print("\nInterrupted.")

  print("-" * 70)
  print("Stopping processes...")
  for name, p in procs.items():
    p.terminate()
  for name, p in procs.items():
    try:
      p.wait(timeout=3)
    except subprocess.TimeoutExpired:
      p.kill()
  print("Done.")


if __name__ == "__main__":
  main()
