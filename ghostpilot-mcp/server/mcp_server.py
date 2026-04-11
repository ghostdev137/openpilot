import json
import asyncio
import os
import time
from pathlib import Path

from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import Tool, TextContent

try:
  from .bridge_client import BridgeClient
except ImportError:
  from bridge_client import BridgeClient

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

CONFIG_PATH = Path(__file__).parent.parent / "config.json"
DEFAULT_CONFIG = {
    "bridge_host": "192.168.4.171",
    "bridge_port": 8765,
    "log_dir": "~/.ghostpilot/logs",
    "dbc": "ford_lincoln_base_pt",
}


def load_config() -> dict:
    cfg = dict(DEFAULT_CONFIG)
    if CONFIG_PATH.exists():
        with open(CONFIG_PATH) as f:
            cfg.update(json.load(f))
    cfg["log_dir"] = os.path.expanduser(cfg["log_dir"])
    return cfg


CONFIG = load_config()

# ---------------------------------------------------------------------------
# DBC helpers (opendbc)
# ---------------------------------------------------------------------------

from opendbc.can.packer import CANPacker

_packer = CANPacker(CONFIG["dbc"])
_dbc = _packer.dbc


def dbc_encode(message_name: str, signals: dict) -> tuple[int, str, int]:
    """Encode signals into a CAN message. Returns (addr, data_hex, size)."""
    addr, data_bytes, bus = _packer.make_can_msg(message_name, 0, signals)
    return addr, data_bytes.hex(), len(data_bytes)


def dbc_decode(addr: int, data_hex: str) -> dict:
    """Decode a CAN frame by address using DBC signal definitions."""
    msg = _dbc.msgs.get(addr)
    if msg is None:
        # Try addr_to_msg
        msg = _dbc.addr_to_msg.get(addr)
    if msg is None:
        return {"error": f"Unknown message address {addr} (0x{addr:03X})"}

    data = bytes.fromhex(data_hex)
    result = {"message_name": msg.name, "address": addr, "signals": {}}

    for sig_name, sig in msg.sigs.items():
        # Extract signal value from raw bytes (big-endian bit numbering)
        raw = _extract_signal(data, sig)
        value = raw * sig.factor + sig.offset
        result["signals"][sig_name] = {
            "value": value,
            "raw": raw,
            "factor": sig.factor,
            "offset": sig.offset,
        }

    return result


def _extract_signal(data: bytes, sig) -> int:
    """Extract a raw signal value from CAN data bytes."""
    # Convert bytes to a single big-endian integer
    dat = int.from_bytes(data, byteorder="big")
    total_bits = len(data) * 8

    if sig.is_little_endian:
        # Little-endian (Intel byte order)
        start_byte = sig.start_bit // 8
        start_bit_in_byte = sig.start_bit % 8
        bit_pos = start_byte * 8 + start_bit_in_byte
        # Reverse bit position for big-endian int
        shift = bit_pos
        # For little-endian signals, we need to work byte by byte
        result = 0
        bits_read = 0
        byte_idx = sig.start_bit // 8
        bit_idx = sig.start_bit % 8
        while bits_read < sig.size:
            bits_in_byte = min(8 - bit_idx, sig.size - bits_read)
            byte_val = data[byte_idx] if byte_idx < len(data) else 0
            extracted = (byte_val >> bit_idx) & ((1 << bits_in_byte) - 1)
            result |= extracted << bits_read
            bits_read += bits_in_byte
            byte_idx += 1
            bit_idx = 0
        return result
    else:
        # Big-endian (Motorola byte order) — standard for Ford DBC
        # start_bit is the MSB position in big-endian bit numbering
        msb = sig.start_bit
        # Convert MSB to bit position in the big-endian integer
        start_byte = msb // 8
        start_bit_in_byte = msb % 8
        bits_remaining = sig.size
        result = 0
        byte_idx = start_byte
        bit_idx = start_bit_in_byte

        while bits_remaining > 0:
            bits_in_byte = min(bit_idx + 1, bits_remaining)
            byte_val = data[byte_idx] if byte_idx < len(data) else 0
            shift = bit_idx - bits_in_byte + 1
            extracted = (byte_val >> shift) & ((1 << bits_in_byte) - 1)
            result = (result << bits_in_byte) | extracted
            bits_remaining -= bits_in_byte
            byte_idx += 1
            bit_idx = 7

        if sig.is_signed and (result & (1 << (sig.size - 1))):
            result -= 1 << sig.size

        return result


# ---------------------------------------------------------------------------
# Steering mode mapping
# ---------------------------------------------------------------------------

STEERING_MODES = {"stock": "0", "apa": "1", "lka": "2"}

# ---------------------------------------------------------------------------
# Logging state
# ---------------------------------------------------------------------------

_log_task: asyncio.Task | None = None
_log_file_path: str | None = None

# ---------------------------------------------------------------------------
# MCP Server
# ---------------------------------------------------------------------------

server = Server("ghostpilot-mcp")
bridge: BridgeClient | None = None


def _text(obj) -> list[TextContent]:
    """Wrap a result object as MCP TextContent."""
    if isinstance(obj, str):
        return [TextContent(type="text", text=obj)]
    return [TextContent(type="text", text=json.dumps(obj, indent=2, default=str))]


# ---------------------------------------------------------------------------
# Tool definitions
# ---------------------------------------------------------------------------

TOOLS = [
    # CAN Tools
    Tool(
        name="can_read",
        description="Read CAN messages from a bus. Returns raw frames captured over a duration.",
        inputSchema={
            "type": "object",
            "properties": {
                "bus": {"type": "integer", "description": "CAN bus number (0, 1, 2)"},
                "duration_ms": {"type": "integer", "description": "Capture duration in milliseconds (default 100)", "default": 100},
                "filter_addrs": {"type": "array", "items": {"type": "integer"}, "description": "Optional list of CAN addresses to filter"},
            },
            "required": ["bus"],
        },
    ),
    Tool(
        name="can_send",
        description="Send a single CAN message.",
        inputSchema={
            "type": "object",
            "properties": {
                "bus": {"type": "integer", "description": "CAN bus number"},
                "addr": {"type": "integer", "description": "CAN address (arbitration ID)"},
                "data_hex": {"type": "string", "description": "Data payload as hex string (e.g. '0102030405060708')"},
            },
            "required": ["bus", "addr", "data_hex"],
        },
    ),
    Tool(
        name="can_send_many",
        description="Send multiple CAN messages at once.",
        inputSchema={
            "type": "object",
            "properties": {
                "msgs": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "bus": {"type": "integer"},
                            "addr": {"type": "integer"},
                            "data_hex": {"type": "string"},
                        },
                        "required": ["bus", "addr", "data_hex"],
                    },
                    "description": "List of CAN messages to send",
                },
            },
            "required": ["msgs"],
        },
    ),
    Tool(
        name="can_send_ford",
        description="Send a CAN message by Ford DBC message name. Encodes signals locally using the ford_lincoln_base_pt DBC, then sends via CAN.",
        inputSchema={
            "type": "object",
            "properties": {
                "message_name": {"type": "string", "description": "DBC message name (e.g. 'Steering_Data_FD1')"},
                "bus": {"type": "integer", "description": "CAN bus number"},
                "signals": {"type": "object", "description": "Signal name-value pairs to encode"},
            },
            "required": ["message_name", "bus", "signals"],
        },
    ),
    Tool(
        name="can_sniff",
        description="Sniff CAN traffic for a duration. Returns decoded CAN printer output.",
        inputSchema={
            "type": "object",
            "properties": {
                "bus": {"type": "integer", "description": "CAN bus to sniff (optional, all buses if omitted)"},
                "duration_s": {"type": "number", "description": "Duration in seconds (default 2.0)", "default": 2.0},
            },
        },
    ),
    Tool(
        name="can_printer",
        description="Alias for can_sniff. Sniff CAN traffic for a duration.",
        inputSchema={
            "type": "object",
            "properties": {
                "bus": {"type": "integer", "description": "CAN bus to sniff (optional)"},
                "duration_s": {"type": "number", "description": "Duration in seconds (default 2.0)", "default": 2.0},
            },
        },
    ),
    # Vehicle State
    Tool(
        name="car_state",
        description="Read current car state (speed, steering angle, brake, gas, gear, etc.) from openpilot.",
        inputSchema={"type": "object", "properties": {}},
    ),
    Tool(
        name="car_control",
        description="Read current car control state from openpilot.",
        inputSchema={"type": "object", "properties": {}},
    ),
    Tool(
        name="selfdrive_state",
        description="Read current selfdrive state from openpilot (enabled, active, alerts).",
        inputSchema={"type": "object", "properties": {}},
    ),
    Tool(
        name="panda_state",
        description="Read panda hardware state (voltages, power, safety mode, faults).",
        inputSchema={"type": "object", "properties": {}},
    ),
    # Control
    Tool(
        name="set_steering_mode",
        description="Set ghostpilot steering mode: 'stock' (0), 'apa' (1), or 'lka' (2).",
        inputSchema={
            "type": "object",
            "properties": {
                "mode": {"type": "string", "enum": ["stock", "apa", "lka"], "description": "Steering mode"},
            },
            "required": ["mode"],
        },
    ),
    Tool(
        name="set_param",
        description="Set an openpilot parameter.",
        inputSchema={
            "type": "object",
            "properties": {
                "key": {"type": "string", "description": "Parameter name"},
                "value": {"description": "Parameter value (string, number, or boolean)"},
            },
            "required": ["key", "value"],
        },
    ),
    Tool(
        name="get_param",
        description="Get an openpilot parameter value.",
        inputSchema={
            "type": "object",
            "properties": {
                "key": {"type": "string", "description": "Parameter name"},
            },
            "required": ["key"],
        },
    ),
    # Process
    Tool(
        name="process_list",
        description="List all openpilot managed processes and their status.",
        inputSchema={"type": "object", "properties": {}},
    ),
    Tool(
        name="process_stop",
        description="Stop an openpilot managed process.",
        inputSchema={
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Process name (e.g. 'controlsd', 'plannerd')"},
            },
            "required": ["name"],
        },
    ),
    Tool(
        name="process_restart",
        description="Restart an openpilot managed process.",
        inputSchema={
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Process name"},
            },
            "required": ["name"],
        },
    ),
    Tool(
        name="openpilot_status",
        description="Get openpilot system state (started, engaged, car detected, etc.).",
        inputSchema={"type": "object", "properties": {}},
    ),
    # Research
    Tool(
        name="dbc_decode",
        description="Decode a raw CAN frame using the Ford DBC. Returns signal names and values.",
        inputSchema={
            "type": "object",
            "properties": {
                "addr": {"type": "integer", "description": "CAN address (arbitration ID)"},
                "data_hex": {"type": "string", "description": "Data payload as hex string"},
            },
            "required": ["addr", "data_hex"],
        },
    ),
    Tool(
        name="dbc_encode",
        description="Encode signals into a CAN frame using the Ford DBC. Returns address and data hex.",
        inputSchema={
            "type": "object",
            "properties": {
                "message_name": {"type": "string", "description": "DBC message name"},
                "signals": {"type": "object", "description": "Signal name-value pairs"},
            },
            "required": ["message_name", "signals"],
        },
    ),
    Tool(
        name="sapp_status",
        description="Get SAPP (Secondary Active Parallel Parking) status from car state.",
        inputSchema={"type": "object", "properties": {}},
    ),
    # Logging
    Tool(
        name="log_start",
        description="Start background logging of cereal services to a JSON-lines file.",
        inputSchema={
            "type": "object",
            "properties": {
                "services": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Services to log (default: ['carState', 'carControl', 'selfdriveState'])",
                },
            },
        },
    ),
    Tool(
        name="log_stop",
        description="Stop background logging and return the log file path.",
        inputSchema={"type": "object", "properties": {}},
    ),
    Tool(
        name="log_list",
        description="List available log files.",
        inputSchema={"type": "object", "properties": {}},
    ),
]


@server.list_tools()
async def list_tools():
    return TOOLS


@server.call_tool()
async def call_tool(name: str, arguments: dict):
    global _log_task, _log_file_path

    # ------------------------------------------------------------------
    # CAN Tools
    # ------------------------------------------------------------------
    if name == "can_read":
        params = {"bus": arguments["bus"]}
        if "duration_ms" in arguments:
            params["duration_ms"] = arguments["duration_ms"]
        if "filter_addrs" in arguments:
            params["filter_addrs"] = arguments["filter_addrs"]
        result = await bridge.call("can_read", params)
        return _text(result)

    if name == "can_send":
        result = await bridge.call("can_send", {
            "bus": arguments["bus"],
            "addr": arguments["addr"],
            "data_hex": arguments["data_hex"],
        })
        return _text(result)

    if name == "can_send_many":
        result = await bridge.call("can_send_many", {"msgs": arguments["msgs"]})
        return _text(result)

    if name == "can_send_ford":
        addr, data_hex, _ = dbc_encode(arguments["message_name"], arguments["signals"])
        result = await bridge.call("can_send", {
            "bus": arguments["bus"],
            "addr": addr,
            "data_hex": data_hex,
        })
        return _text({"encoded": {"addr": addr, "data_hex": data_hex}, "send_result": result})

    if name in ("can_sniff", "can_printer"):
        params = {}
        if "bus" in arguments:
            params["bus"] = arguments["bus"]
        if "duration_s" in arguments:
            params["duration_s"] = arguments["duration_s"]
        result = await bridge.call("can_printer", params)
        return _text(result)

    # ------------------------------------------------------------------
    # Vehicle State
    # ------------------------------------------------------------------
    if name == "car_state":
        result = await bridge.call("cereal_read", {"service": "carState"})
        return _text(result)

    if name == "car_control":
        result = await bridge.call("cereal_read", {"service": "carControl"})
        return _text(result)

    if name == "selfdrive_state":
        result = await bridge.call("cereal_read", {"service": "selfdriveState"})
        return _text(result)

    if name == "panda_state":
        result = await bridge.call("panda_health")
        return _text(result)

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------
    if name == "set_steering_mode":
        mode = arguments["mode"]
        if mode not in STEERING_MODES:
            return _text({"error": f"Invalid mode '{mode}'. Must be one of: stock, apa, lka"})
        result = await bridge.call("param_put", {
            "key": "GhostpilotSteeringMode",
            "value": STEERING_MODES[mode],
        })
        return _text({"mode": mode, "value": STEERING_MODES[mode], "result": result})

    if name == "set_param":
        result = await bridge.call("param_put", {
            "key": arguments["key"],
            "value": str(arguments["value"]),
        })
        return _text(result)

    if name == "get_param":
        result = await bridge.call("param_get", {"key": arguments["key"]})
        return _text(result)

    # ------------------------------------------------------------------
    # Process
    # ------------------------------------------------------------------
    if name == "process_list":
        result = await bridge.call("process_list")
        return _text(result)

    if name == "process_stop":
        result = await bridge.call("process_stop", {"name": arguments["name"]})
        return _text(result)

    if name == "process_restart":
        result = await bridge.call("process_restart", {"name": arguments["name"]})
        return _text(result)

    if name == "openpilot_status":
        result = await bridge.call("openpilot_state")
        return _text(result)

    # ------------------------------------------------------------------
    # Research
    # ------------------------------------------------------------------
    if name == "dbc_decode":
        result = dbc_decode(arguments["addr"], arguments["data_hex"])
        return _text(result)

    if name == "dbc_encode":
        addr, data_hex, size = dbc_encode(arguments["message_name"], arguments["signals"])
        return _text({"message_name": arguments["message_name"], "addr": addr, "data_hex": data_hex, "size": size})

    if name == "sapp_status":
        result = await bridge.call("cereal_read", {"service": "carState"})
        # Extract SAPP-related fields if present
        sapp_info = {}
        if isinstance(result, dict):
            for key in result:
                if "sapp" in key.lower() or "park" in key.lower():
                    sapp_info[key] = result[key]
        if not sapp_info:
            # Fall back to param
            try:
                sapp_param = await bridge.call("param_get", {"key": "GhostpilotSappState"})
                sapp_info["param"] = sapp_param
            except Exception:
                sapp_info["note"] = "No SAPP data found in carState or params"
        return _text(sapp_info)

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------
    if name == "log_start":
        if _log_task is not None and not _log_task.done():
            return _text({"error": "Logging already active", "file": _log_file_path})

        services = arguments.get("services", ["carState", "carControl", "selfdriveState"])
        log_dir = Path(CONFIG["log_dir"])
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        _log_file_path = str(log_dir / f"log_{timestamp}.jsonl")

        async def _logging_loop(file_path: str, svc_list: list[str]):
            with open(file_path, "w") as f:
                while True:
                    try:
                        result = await bridge.call("cereal_read_multi", {"services": svc_list})
                        entry = {"ts": time.time(), "data": result}
                        f.write(json.dumps(entry, default=str) + "\n")
                        f.flush()
                    except asyncio.CancelledError:
                        break
                    except Exception as e:
                        entry = {"ts": time.time(), "error": str(e)}
                        f.write(json.dumps(entry) + "\n")
                        f.flush()
                    await asyncio.sleep(0.1)

        _log_task = asyncio.create_task(_logging_loop(_log_file_path, services))
        return _text({"status": "logging_started", "file": _log_file_path, "services": services})

    if name == "log_stop":
        if _log_task is None or _log_task.done():
            return _text({"error": "No active logging session"})
        _log_task.cancel()
        try:
            await _log_task
        except asyncio.CancelledError:
            pass
        path = _log_file_path
        _log_task = None
        _log_file_path = None
        return _text({"status": "logging_stopped", "file": path})

    if name == "log_list":
        log_dir = Path(CONFIG["log_dir"])
        if not log_dir.exists():
            return _text({"files": []})
        files = sorted(log_dir.glob("*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
        file_list = []
        for f in files:
            stat = f.stat()
            file_list.append({
                "name": f.name,
                "path": str(f),
                "size_bytes": stat.st_size,
                "modified": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stat.st_mtime)),
            })
        return _text({"files": file_list})

    return _text({"error": f"Unknown tool: {name}"})


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

async def main():
    global bridge
    bridge = BridgeClient(host=CONFIG["bridge_host"], port=CONFIG["bridge_port"])

    async with stdio_server() as (read_stream, write_stream):
        await server.run(
            read_stream,
            write_stream,
            server.create_initialization_options(),
        )


def run():
    """Entry point for console_scripts."""
    asyncio.run(main())


if __name__ == "__main__":
    asyncio.run(main())
