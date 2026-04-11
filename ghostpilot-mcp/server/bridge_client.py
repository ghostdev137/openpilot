import json
import asyncio
import websockets


class BridgeClient:
    def __init__(self, host="192.168.4.171", port=8765):
        self.url = f"ws://{host}:{port}"
        self._ws = None
        self._id = 0
        self._lock = asyncio.Lock()

    async def connect(self):
        self._ws = await websockets.connect(self.url)

    async def close(self):
        if self._ws:
            await self._ws.close()
            self._ws = None

    async def _ensure_connected(self):
        if self._ws is None:
            await self.connect()
            return
        # Check if connection is still alive
        try:
            await self._ws.ping()
        except Exception:
            self._ws = None
            await self.connect()

    async def call(self, method: str, params: dict = None) -> dict:
        """Send JSON-RPC request and return result. Raises on error.
        Automatically reconnects if the connection has dropped."""
        async with self._lock:
            await self._ensure_connected()
            self._id += 1
            req = {
                "jsonrpc": "2.0",
                "method": method,
                "params": params or {},
                "id": self._id,
            }
            try:
                await self._ws.send(json.dumps(req))
                resp = json.loads(await self._ws.recv())
            except (websockets.exceptions.ConnectionClosed, ConnectionError):
                # Reconnect once and retry
                self._ws = None
                await self.connect()
                self._id += 1
                req["id"] = self._id
                await self._ws.send(json.dumps(req))
                resp = json.loads(await self._ws.recv())

            if "error" in resp:
                raise Exception(f"Bridge error: {resp['error']['message']}")
            return resp.get("result", {})
