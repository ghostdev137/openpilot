#!/usr/bin/env python3
"""
ghostpilot-bridge — WebSocket JSON-RPC 2.0 server for openpilot on comma 4.

Exposes CAN bus, cereal messaging, params, and process control to network clients.
Bind: 0.0.0.0:8765 (configurable via --port)
"""

import argparse
import asyncio
import json
import subprocess
import sys
import time
import traceback

# Auto-install websockets if missing
try:
  import websockets
except ImportError:
  subprocess.check_call([sys.executable, "-m", "pip", "install", "websockets"])
  import websockets


# ---------------------------------------------------------------------------
# Lazy imports for openpilot libraries (may not exist in dev environments)
# ---------------------------------------------------------------------------

_messaging = None
_Params = None
_can_list_to_can_capnp = None


def _get_messaging():
  global _messaging
  if _messaging is None:
    import cereal.messaging as messaging
    _messaging = messaging
  return _messaging


def _get_params():
  global _Params
  if _Params is None:
    from openpilot.common.params import Params
    _Params = Params
  return _Params


def _get_can_packer():
  global _can_list_to_can_capnp
  if _can_list_to_can_capnp is None:
    from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
    _can_list_to_can_capnp = can_list_to_can_capnp
  return _can_list_to_can_capnp


# ---------------------------------------------------------------------------
# Shared SubMaster / PubMaster (lazy, reused)
# ---------------------------------------------------------------------------

_sub_masters: dict = {}   # service_tuple -> SubMaster
_pub_master = None
_can_sub_master = None


def _get_sub_master(services: tuple):
  """Return a SubMaster subscribed to the given services, creating if needed."""
  key = services
  if key not in _sub_masters:
    messaging = _get_messaging()
    _sub_masters[key] = messaging.SubMaster(list(services))
  return _sub_masters[key]


def _get_pub_master():
  global _pub_master
  if _pub_master is None:
    messaging = _get_messaging()
    _pub_master = messaging.PubMaster(['sendcan'])
  return _pub_master


# ---------------------------------------------------------------------------
# JSON-RPC helpers
# ---------------------------------------------------------------------------

def _ok(result, req_id):
  return {"jsonrpc": "2.0", "result": result, "id": req_id}


def _error(code, message, req_id=None, data=None):
  err = {"code": code, "message": message}
  if data is not None:
    err["data"] = data
  return {"jsonrpc": "2.0", "error": err, "id": req_id}


# ---------------------------------------------------------------------------
# RPC method implementations
# ---------------------------------------------------------------------------

async def rpc_ping(_params):
  return {"pong": True}


async def rpc_can_read(params):
  bus = int(params["bus"])
  duration_ms = int(params.get("duration_ms", 100))
  filter_addrs = params.get("filter_addrs", None)

  messaging = _get_messaging()
  sock = messaging.sub_sock('can')

  collected = []
  deadline = time.monotonic() + duration_ms / 1000.0

  while time.monotonic() < deadline:
    msgs = messaging.drain_sock(sock)
    for msg in msgs:
      for c in msg.can:
        if c.src == bus or bus == -1:
          if filter_addrs is not None and c.address not in filter_addrs:
            continue
          collected.append({
            "addr": c.address,
            "data_hex": bytes(c.dat).hex(),
            "bus": c.src,
          })
    await asyncio.sleep(0.005)

  return collected


async def rpc_can_send(params):
  bus = int(params["bus"])
  addr = int(params["addr"])
  data_hex = params["data_hex"]
  data_bytes = bytes.fromhex(data_hex)

  packer = _get_can_packer()
  pm = _get_pub_master()

  can_bytes = packer([(addr, data_bytes, bus)], msgtype='sendcan')
  pm.send('sendcan', can_bytes)
  return {"ok": True}


async def rpc_can_send_many(params):
  msgs_raw = params["msgs"]
  can_msgs = []
  for m in msgs_raw:
    can_msgs.append((int(m["addr"]), bytes.fromhex(m["data_hex"]), int(m["bus"])))

  packer = _get_can_packer()
  pm = _get_pub_master()

  can_bytes = packer(can_msgs, msgtype='sendcan')
  pm.send('sendcan', can_bytes)
  return {"ok": True, "count": len(can_msgs)}


async def rpc_can_printer(params):
  bus = int(params.get("bus", -1))
  duration_s = float(params.get("duration_s", 2.0))

  messaging = _get_messaging()
  sock = messaging.sub_sock('can')

  stats = {}  # addr -> {count, first_time, last_time, last_data}
  start = time.monotonic()
  deadline = start + duration_s

  while time.monotonic() < deadline:
    msgs = messaging.drain_sock(sock)
    now = time.monotonic()
    for msg in msgs:
      for c in msg.can:
        if bus != -1 and c.src != bus:
          continue
        a = c.address
        data_hex = bytes(c.dat).hex()
        if a not in stats:
          stats[a] = {"count": 0, "first_time": now, "last_time": now, "last_data_hex": data_hex}
        stats[a]["count"] += 1
        stats[a]["last_time"] = now
        stats[a]["last_data_hex"] = data_hex
    await asyncio.sleep(0.005)

  elapsed = time.monotonic() - start
  result = {}
  for addr, s in stats.items():
    hex_addr = f"0x{addr:03X}"
    freq = s["count"] / elapsed if elapsed > 0 else 0
    result[hex_addr] = {
      "addr": addr,
      "count": s["count"],
      "freq_hz": round(freq, 2),
      "last_data_hex": s["last_data_hex"],
    }
  return result


async def rpc_cereal_read(params):
  service = params["service"]
  sm = _get_sub_master((service,))
  sm.update(timeout=500)

  msg = sm[service]
  valid = sm.valid.get(service, False)

  try:
    data = msg.to_dict()
  except Exception:
    data = str(msg)

  return {"service": service, "valid": valid, "data": data}


async def rpc_cereal_read_multi(params):
  services = tuple(params["services"])
  sm = _get_sub_master(services)
  sm.update(timeout=500)

  results = []
  for service in services:
    msg = sm[service]
    valid = sm.valid.get(service, False)
    try:
      data = msg.to_dict()
    except Exception:
      data = str(msg)
    results.append({"service": service, "valid": valid, "data": data})
  return results


async def rpc_param_get(params):
  key = params["key"]
  p = _get_params()()
  val = p.get(key, return_default=True)
  # val is already typed (int, bool, str, etc.) from params_pyx
  if isinstance(val, bytes):
    try:
      val = val.decode("utf-8")
    except Exception:
      val = val.hex()
  return {"key": key, "value": val}


async def rpc_param_put(params):
  key = params["key"]
  value = params["value"]
  p = _get_params()()
  if isinstance(value, (dict, list)):
    value = json.dumps(value)
  p.put_nonblocking(key, str(value))
  return {"ok": True}


async def rpc_panda_health(_params):
  sm = _get_sub_master(("pandaStates",))
  sm.update(timeout=1000)

  states = []
  try:
    for ps in sm["pandaStates"]:
      try:
        d = ps.to_dict()
      except Exception:
        d = str(ps)
      states.append({
        "pandaType": d.get("pandaType", str(ps.pandaType)) if isinstance(d, dict) else str(ps.pandaType),
        "ignitionLine": d.get("ignitionLine", None) if isinstance(d, dict) else None,
        "ignitionCan": d.get("ignitionCan", None) if isinstance(d, dict) else None,
        "controlsAllowed": d.get("controlsAllowed", None) if isinstance(d, dict) else None,
        "safetyModel": d.get("safetyModel", None) if isinstance(d, dict) else None,
        "harnessStatus": d.get("harnessStatus", None) if isinstance(d, dict) else None,
      })
  except Exception:
    pass

  return {"pandas": states}


async def rpc_process_list(_params):
  sm = _get_sub_master(("managerState",))
  sm.update(timeout=1000)

  procs = []
  try:
    for p in sm["managerState"].processes:
      procs.append({
        "name": p.name,
        "running": p.running,
        "pid": p.pid if p.pid != 0 else None,
        "exitCode": p.exitCode,
      })
  except Exception as e:
    return {"error": str(e), "processes": []}

  return procs


async def rpc_process_stop(params):
  name = params["name"]
  # Validate name to prevent injection
  if not name.isalnum() and not all(c in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-." for c in name):
    return _error(-32602, f"Invalid process name: {name}")
  subprocess.run(["pkill", "-f", name], capture_output=True)
  return {"ok": True}


async def rpc_process_restart(params):
  name = params["name"]
  if not all(c in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-." for c in name):
    return _error(-32602, f"Invalid process name: {name}")
  subprocess.run(["pkill", "-f", name], capture_output=True)
  # Manager will auto-restart managed processes
  return {"ok": True}


async def rpc_openpilot_state(_params):
  sm = _get_sub_master(("selfdriveState", "carState"))
  sm.update(timeout=1000)

  params_cls = _get_params()
  p = params_cls()

  state = {}

  # Params
  try:
    fp = p.get("CarFingerprint")
    state["fingerprint"] = fp.decode("utf-8", errors="replace") if isinstance(fp, bytes) else str(fp or "unknown")
  except Exception:
    state["fingerprint"] = "unknown"
  state["dongleId"] = (p.get("DongleId") or b"").decode("utf-8", errors="replace")
  state["gitBranch"] = (p.get("GitBranch") or b"").decode("utf-8", errors="replace")

  # selfdriveState
  try:
    sds = sm["selfdriveState"]
    state["state"] = str(sds.state)
    state["enabled"] = sds.enabled
    state["alertText1"] = sds.alertText1
    state["alertText2"] = sds.alertText2
  except Exception as e:
    state["selfdriveState_error"] = str(e)

  # carState
  try:
    cs = sm["carState"]
    state["speed_mps"] = round(cs.vEgo, 2)
    state["steeringAngleDeg"] = round(cs.steeringAngleDeg, 2)
    state["cruiseState"] = {
      "enabled": cs.cruiseState.enabled,
      "speed_mps": round(cs.cruiseState.speed, 2),
      "available": cs.cruiseState.available,
    }
    state["gasPressed"] = cs.gasPressed
    state["brakePressed"] = cs.brakePressed
    state["steeringPressed"] = cs.steeringPressed
  except Exception as e:
    state["carState_error"] = str(e)

  return state


# ---------------------------------------------------------------------------
# Method dispatch table
# ---------------------------------------------------------------------------

METHODS = {
  "ping": rpc_ping,
  "can_read": rpc_can_read,
  "can_send": rpc_can_send,
  "can_send_many": rpc_can_send_many,
  "can_printer": rpc_can_printer,
  "cereal_read": rpc_cereal_read,
  "cereal_read_multi": rpc_cereal_read_multi,
  "param_get": rpc_param_get,
  "param_put": rpc_param_put,
  "panda_health": rpc_panda_health,
  "process_list": rpc_process_list,
  "process_stop": rpc_process_stop,
  "process_restart": rpc_process_restart,
  "openpilot_state": rpc_openpilot_state,
}


# ---------------------------------------------------------------------------
# JSON-RPC handler
# ---------------------------------------------------------------------------

async def handle_rpc(websocket):
  async for raw in websocket:
    req_id = None
    try:
      request = json.loads(raw)
      req_id = request.get("id")
      method = request.get("method")
      params = request.get("params", {})

      if method not in METHODS:
        resp = _error(-32601, f"Method not found: {method}", req_id)
      else:
        result = await METHODS[method](params)
        resp = _ok(result, req_id)

    except json.JSONDecodeError:
      resp = _error(-32700, "Parse error", req_id)
    except KeyError as e:
      resp = _error(-32602, f"Missing required param: {e}", req_id)
    except Exception as e:
      tb = traceback.format_exc()
      resp = _error(-32000, str(e), req_id, data=tb)

    await websocket.send(json.dumps(resp, default=str))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def main(port: int):
  print(f"ghostpilot-bridge starting on 0.0.0.0:{port}")
  async with websockets.serve(handle_rpc, "0.0.0.0", port):
    print(f"ghostpilot-bridge ready — ws://0.0.0.0:{port}")
    await asyncio.Future()  # run forever


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="ghostpilot-bridge WebSocket JSON-RPC server")
  parser.add_argument("--port", type=int, default=8765, help="Port to bind (default: 8765)")
  args = parser.parse_args()

  asyncio.run(main(args.port))
