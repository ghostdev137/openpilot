# ghostpilot-mcp Design Spec

## Overview

An MCP (Model Context Protocol) server for openpilot that gives AI agents and developer tools direct access to the vehicle's CAN bus, openpilot state, process control, and logging — all over the network.

## Architecture

Two components connected over LAN WebSocket:

```
Claude Code / MCP Client
        |
   ghostpilot-mcp (Mac, MCP protocol)
        |  WebSocket JSON-RPC (LAN, port 8765)
   ghostpilot-bridge (comma 4)
        |
   cereal messaging + panda API
        |
      Ford Transit CAN bus
```

### ghostpilot-bridge (comma 4)

Single Python file. Runs on the comma 4 alongside openpilot. Exposes a WebSocket JSON-RPC server on port 8765. Thin proxy — no business logic, just pipes data between the MCP server and openpilot internals.

**Dependencies:** `websockets`, `panda`, `cereal`, `openpilot.common.params`

**Bridge JSON-RPC methods:**

| Method | Args | Returns |
|--------|------|---------|
| `can_read` | `bus: int`, `duration_ms: int`, `filter_addrs: list[int] \| null` | `[{addr, data_hex, bus, ts_ms}, ...]` |
| `can_send` | `bus: int`, `addr: int`, `data_hex: str` | `{ok: true}` |
| `can_send_many` | `msgs: [{bus, addr, data_hex}, ...]` | `{ok: true, count: int}` |
| `cereal_read` | `service: str` | Latest message as dict |
| `cereal_read_multi` | `services: list[str]` | `{service: message_dict, ...}` |
| `cereal_subscribe` | `services: list[str]`, `hz: int` | Opens streaming channel, pushes updates |
| `param_get` | `key: str` | Value (typed) |
| `param_put` | `key: str`, `value: any` | `{ok: true}` |
| `panda_health` | — | `{voltage, current, safety_mode, ignition, ...}` |
| `panda_set_safety` | `mode: int`, `param: int` | `{ok: true}` |
| `process_list` | — | `[{name, running, pid, enabled}, ...]` |
| `process_start` | `name: str` | `{ok: true, pid: int}` |
| `process_stop` | `name: str` | `{ok: true}` |
| `process_restart` | `name: str` | `{ok: true, pid: int}` |
| `openpilot_state` | — | `{started, engaged, fingerprint, steering_mode, ...}` |
| `log_tail` | `service: str`, `lines: int` | Last N log entries |
| `can_printer` | `bus: int`, `duration_s: float` | `{addr: {count, freq_hz, last_data_hex}, ...}` |
| `fingerprint_ecus` | — | `{ecu: {addr, fw_version}, ...}` |

No auth (LAN only, research use). Stateless except for streaming subscriptions.

### ghostpilot-mcp (Mac)

MCP server connecting to the bridge over WebSocket. Exposes tools to any MCP client. Handles DBC decoding, logging, and session management locally.

**Config:** `~/.ghostpilot/config.json`
```json
{
  "bridge_host": "192.168.4.171",
  "bridge_port": 8765,
  "log_dir": "~/.ghostpilot/logs",
  "dbc": "ford_lincoln_base_pt"
}
```

## MCP Tools

### CAN Tools

**`can_read`** — Read CAN messages from a bus
- Args: `bus: int`, `duration_ms: int` (default 100), `filter_addrs: list[int]` (optional), `decode: bool` (default true)
- Returns: Array of CAN frames. If `decode=true`, includes DBC signal names and values.
- Example return: `[{addr: 0x415, bus: 0, data: "00FA03C0...", signals: {Veh_V_ActlBrk: 48.0, VehVActlBrk_D_Qf: 3}}]`

**`can_send`** — Send a raw CAN frame
- Args: `bus: int`, `addr: int`, `data_hex: str`
- No confirmation, immediate send.

**`can_send_ford`** — Send a named Ford DBC message with signal values
- Args: `message_name: str`, `bus: int`, `signals: dict`
- Example: `can_send_ford(message_name="ParkAid_Data", bus=0, signals={ExtSteeringAngleReq2: 15.0, SAPPStatusCoding: 70})`
- Encodes via DBC locally, sends raw bytes to bridge.

**`can_sniff`** — Capture CAN traffic summary
- Args: `bus: int`, `duration_s: float` (default 2)
- Returns: `{addr: {count, freq_hz, last_data_hex, message_name}, ...}` sorted by frequency.

**`can_printer`** — Live CAN printer (like panda/scripts/can_printer.py)
- Args: `bus: int`, `duration_s: float`, `filter_addrs: list[int]` (optional)
- Returns: Formatted CAN traffic dump with DBC decoding.

### Vehicle State Tools

**`car_state`** — Get decoded carState
- Returns: `{speed_mph, speed_kph, steering_angle_deg, brake_pressed, gas_pressed, cruise_engaged, cruise_speed, gear, doors_open, seatbelt, standstill, yaw_rate, ...}`

**`car_control`** — Get what openpilot is commanding
- Returns: `{lat_active, long_active, curvature, accel, gas, steering_angle_deg, ...}`

**`selfdrive_state`** — Get selfdriveState
- Returns: `{state, enabled, active, events: [...], ...}`

**`panda_state`** — Panda hardware status
- Returns: `{type, firmware_version, safety_mode, ignition, voltage, can_rx_errors, ...}`

### Control Tools

**`set_steering_mode`** — Switch steering mode
- Args: `mode: str` — one of "stock", "apa", "lka"
- Writes `GhostpilotSteeringMode` param.

**`set_param`** — Write any openpilot param
- Args: `key: str`, `value: any`

**`get_param`** — Read any openpilot param
- Args: `key: str`

**`send_steer`** — Send a steering command directly via CAN
- Args: `angle_deg: float` (for APA/LKA modes) or `curvature: float` (for STOCK mode)
- Encodes the appropriate CAN message (ParkAid_Data, Lane_Assist_Data1, or LateralMotionControl) and sends.

**`send_accel`** — Send acceleration command via ACCDATA
- Args: `accel_mss: float`, `gas_mss: float`

### Process Tools

**`process_list`** — List all openpilot processes
- Returns: `[{name, running, pid, enabled}, ...]`

**`process_start`** — Start a named process
- Args: `name: str`

**`process_stop`** — Stop a named process
- Args: `name: str`

**`process_restart`** — Restart a named process
- Args: `name: str`

**`openpilot_status`** — Full system status
- Returns: `{started, engaged, fingerprint, steering_mode, processes: [...], panda: {...}, params: {key: value, ...}}`

### Logging Tools

**`log_start`** — Start a logging session
- Args: `services: list[str]` (default `["can", "carState", "carControl", "sendcan"]`), `include_can: bool` (default true)
- Creates `~/.ghostpilot/logs/YYYY-MM-DD-HH-MM-SS.json`
- Returns: `{session_id, file_path}`

**`log_stop`** — Stop logging
- Args: `session_id: str`
- Returns: `{file_path, duration_s, message_count}`

**`log_list`** — List saved log sessions
- Returns: `[{session_id, file_path, started_at, duration_s, size_bytes}, ...]`

**`log_read`** — Read a log file
- Args: `session_id: str`, `service: str` (optional filter), `limit: int` (default 100)
- Returns: Array of log entries.

### Research Tools

**`dbc_decode`** — Decode raw CAN bytes using Ford DBC
- Args: `addr: int`, `data_hex: str`
- Returns: `{message_name, signals: {name: value, ...}}`

**`dbc_encode`** — Encode signal values to raw CAN bytes
- Args: `message_name: str`, `signals: dict`
- Returns: `{addr, data_hex}`

**`dbc_list_messages`** — List all messages in the Ford DBC
- Returns: `[{name, addr, length, signals: [...]}, ...]`

**`fingerprint_scan`** — Query ECU firmware versions
- Returns: `{eps: {addr, fw}, abs: {addr, fw}, camera: {addr, fw}, radar: {addr, fw}}`

**`sapp_status`** — Get SAPP handshake state
- Returns: `{state: int, state_name: str, handshake_complete: bool}`

## File Structure

```
ghostpilot-mcp/
├── bridge/
│   └── ghostpilot_bridge.py       # Single file, runs on comma 4
├── server/
│   ├── __init__.py
│   ├── mcp_server.py              # MCP server entry point + tool registration
│   ├── bridge_client.py           # WebSocket client to comma 4 bridge
│   ├── tools_can.py               # can_read, can_send, can_send_ford, can_sniff, can_printer
│   ├── tools_vehicle.py           # car_state, car_control, selfdrive_state, panda_state
│   ├── tools_control.py           # set_steering_mode, set/get_param, send_steer, send_accel
│   ├── tools_process.py           # process_list, process_start/stop/restart, openpilot_status
│   ├── tools_logging.py           # log_start/stop/list/read
│   └── tools_research.py          # dbc_decode/encode/list, fingerprint_scan, sapp_status
├── config.json                    # Default config
├── pyproject.toml
└── README.md
```

## Technology

- **Bridge:** Python 3.12, `websockets` library, cereal messaging, panda Python API
- **MCP Server:** Python 3.12, `mcp` SDK (Anthropic), `websockets` client, `opendbc` for DBC encoding/decoding
- **Transport:** WebSocket JSON-RPC between bridge and MCP server. MCP stdio transport for Claude Code.

## Deployment

**Bridge (comma 4):**
```bash
scp bridge/ghostpilot_bridge.py comma@192.168.4.171:/data/
ssh comma@192.168.4.171 "python3 /data/ghostpilot_bridge.py &"
```

**MCP Server (Mac):**
```bash
cd ghostpilot-mcp && pip install -e .
# Add to Claude Code MCP config:
# "ghostpilot": {"command": "ghostpilot-mcp"}
```

## Non-Goals

- No authentication or encryption (LAN-only research tool)
- No web UI (use MCP clients)
- No video/camera streaming (use comma's existing WebRTC)
- No OTA update mechanism
- No safety checks on CAN writes (panda is already passthrough)
