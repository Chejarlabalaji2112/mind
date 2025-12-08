from mind.ports.act_port import Presenter
from mind.ports.percieve_port import Audition



#!/usr/bin/env python3
"""
ESP32 WebSocket client.

Classes:
  - Connection : Handles discovery + connection + send/receive.
  - sender     : Sends JSON payloads to ESP32 (show method).
  - Listener   : Receives messages from ESP32 (listen method).
"""

import asyncio
import websockets
import json
import socket
import time
from mind.utils.logging_handler import setup_logger

connection_logger = setup_logger("connection")
sender_logger = setup_logger("sender", console=False)
listener_logger = setup_logger("listener", console=False)
import os

ESP32_MDNS = "esp32.local"
WS_PATH = "/ws"
UDP_PORT = 4210
LAST_IP_FILE = "last_esp32_ip.txt"


# ------------------------------
# Connection
# ------------------------------
class Connection:
    """Responsible for establishing and managing the WebSocket connection."""

    def __init__(self):
        self.ws = None

    async def connect(self):
        ws = await self._try_mdns() or await self._try_last_ip() or await self._try_udp()
        if not ws:
            raise RuntimeError("Could not connect to ESP32")
        self.ws = ws

    async def send(self, payload: str):
        if not self.ws:
            raise RuntimeError("Not connected")
        await self.ws.send(payload)

    async def receive(self):
        if not self.ws:
            raise RuntimeError("Not connected")
        async for msg in self.ws:
            yield msg

    async def close(self):
        if self.ws:
            await self.ws.close()
            self.ws = None

    # --- private helpers ---
    async def _try_mdns(self):
        uri = f"ws://{ESP32_MDNS}{WS_PATH}"
        return await self._try_connect(uri, "mDNS")

    async def _try_last_ip(self):
        if not os.path.exists(LAST_IP_FILE):
            return None
        with open(LAST_IP_FILE, "r") as f:
            last_ip = f.read().strip()
        if not last_ip:
            return None
        uri = f"ws://{last_ip}{WS_PATH}"
        return await self._try_connect(uri, "last-known IP")

    async def _try_udp(self):
        ip = self._discover_via_udp(timeout=5)
        if not ip:
            return None
        uri = f"ws://{ip}{WS_PATH}"
        ws = await self._try_connect(uri, "UDP discovery")
        if ws:
            with open(LAST_IP_FILE, "w") as f:
                f.write(ip)
        return ws

    async def _try_connect(self, uri, label):
        try:
            ws = await asyncio.wait_for(websockets.connect(uri), timeout=3)
            connection_logger.info(("="*50))
            connection_logger.info(f"[client] connected via {label}: {uri} ")
            connection_logger.info("="*50)
            return ws
        except Exception:
            return None

    def _discover_via_udp(self, timeout=5):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("", UDP_PORT))
        sock.settimeout(timeout)
        start = time.time()
        try:
            while True:
                remaining = timeout - (time.time() - start)
                if remaining <= 0:
                    break
                try:
                    data, _ = sock.recvfrom(1024)
                except socket.timeout:
                    break
                try:
                    text = data.decode("utf-8", errors="ignore")
                    j = json.loads(text)
                    if "ip" in j:
                        return j["ip"]
                except Exception:
                    if "ip=" in text:
                        return text.split("ip=")[1].split()[0].strip(",}")
        finally:
            sock.close()
        return None


# ------------------------------
# Sender
# ------------------------------
class Sender(Presenter):
    """Sends JSON payloads to ESP32.

    Example payloads:
      {"mode": "text", "text": "Hello", "x": 20, "y": 20, "size": 2}
      {"mode": "eyes", "mood": "happy", "direction": "center"}
      {"mode": "clear"}
      {"mode": "clearRect", "x": 10, "y": 10, "w": 50, "h": 20}
    """

    def __init__(self, connection: Connection):
        self.conn = connection

    async def show(self, obj: dict):
        """Send any JSON object to ESP32."""
        await self.conn.send(json.dumps(obj))


# ------------------------------
# Listener
# ------------------------------
class Listener(Audition):
    """Receives messages from ESP32."""

    def __init__(self, connection: Connection):
        self.conn = connection

    async def listen(self):
        async for msg in self.conn.receive():
            listener_logger.info(f"[ESP32] {msg}")


def esp_output_tuner(title="None", content="", bottom="",/,home=False ):
    if home:
        return {"mode": "eyes"}
    return {"mode": "text",
            "clear": True,
             "items": [
                 {"text":title, "x": 0, "y": 0, "size": 1} ,
                 {"text":content, "x": 10, "y": 20, "size": 2},
                 {"text":bottom, "x": 0, "y": 54, "size": 1}
             ]
             }



# ------------------------------
# Example usage
# ------------------------------
async def main():
    conn = Connection()
    await conn.connect()

    sender = Sender(conn)
    listener = Listener(conn)

    # Run listener in background
    asyncio.create_task(listener.listen())

    # Send some test commands
    await sender.show({"mode": "text", "text": "Hello ESP32"})
    #await sender.show({"mode": "eyes", "mood": "happy", "direction": "center"})
    await asyncio.sleep(5)
    await sender.show({"mode": "eyes", "mood": "happy", "direction": "center"})
    await sender.show({"mode": "clear"})
    connection_logger.info("[client] waiting 3s before closing...")
    await asyncio.sleep(3)
    await conn.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        connection_logger.info("[client] exiting.")
        connection_logger.info("="*40)
