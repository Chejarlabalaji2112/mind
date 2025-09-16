#!/usr/bin/env python3
"""
Robust client for ESP32 WebSocket + UDP discovery.

Usage:
  - Run this script; type commands:
      text Hello world
      eyes happy center
      eyes tired left
      raw {"mode":"text","text":"Hi"}
  - The client will:
      * try mDNS ws://esp32.local/ws
      * try last known IP saved in last_esp32_ip.txt
      * listen for UDP announcements on port 4210 for 5s
      * reconnect automatically on disconnect
"""

import asyncio
import websockets
import json
import socket
import time
import os

ESP32_MDNS = "esp32.local"
WS_PATH = "/ws"
UDP_PORT = 4210
LAST_IP_FILE = "last_esp32_ip.txt"

# Try to connect using hostname (mDNS) or IP string
async def try_connect(uri, timeout=5):
    try:
        ws = await asyncio.wait_for(websockets.connect(uri), timeout=timeout)
        return ws
    except Exception as e:
        return None

def discover_via_udp(timeout=5):
    """Listen for UDP announcements for `timeout` seconds.
       Returns first IP found in JSON announcement or None.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # allow broadcast
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", UDP_PORT))
    sock.settimeout(timeout)
    start = time.time()
    try:
        while True:
            remaining = timeout - (time.time() - start)
            if remaining <= 0:
                break
            sock.settimeout(remaining)
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                break
            try:
                text = data.decode("utf-8", errors="ignore")
                # it's JSON like: {"name":"esp32","ip":"192.168.x.x",...}
                try:
                    j = json.loads(text)
                    ip = j.get("ip")
                    if ip:
                        return ip
                except:
                    # fallback parse key= pairs
                    s = text
                    if "ip=" in s:
                        # ip=192.168.1.42
                        idx = s.find("ip=")
                        tail = s[idx+3:]
                        # take until , or space or }
                        for sep in [",", " ", "}"]:
                            pos = tail.find(sep)
                            if pos != -1:
                                tail = tail[:pos]
                        return tail
            except Exception:
                pass
    finally:
        sock.close()
    return None

async def interactive_send(ws):
    """Read user input in a thread and send JSON to ESP32."""
    def parse_command(line):
        line = line.strip()
        if not line:
            return None
        if line.startswith("{"):
            # raw JSON
            try:
                json.loads(line)
                return line
            except:
                print("[client] invalid JSON")
                return None
        parts = line.split(maxsplit=1)
        cmd = parts[0].lower()
        rest = parts[1] if len(parts) > 1 else ""
        if cmd == "text":
            obj = {"mode":"text", "text":rest, "x":0, "y":20, "size":1}
            return json.dumps(obj)
        if cmd == "eyes":
            # syntax: eyes <mood> <direction>  (direction optional)
            tokens = rest.split()
            mood = tokens[0] if len(tokens) > 0 else "happy"
            direction = tokens[1] if len(tokens) > 1 else "center"
            obj = {"mode":"eyes", "mood": mood, "direction": direction}
            return json.dumps(obj)
        if cmd == "quit" or cmd == "exit":
            return "__QUIT__"
        print("[client] unknown command. Use 'text ...' or 'eyes mood direction' or raw JSON.")
        return None

    loop = asyncio.get_running_loop()
    while True:
        line = await asyncio.to_thread(input, "> ")
        payload = parse_command(line)
        if payload is None:
            continue
        if payload == "__QUIT__":
            await ws.close()
            return
        try:
            await ws.send(payload)
        except Exception as e:
            print("[client] send failed:", e)
            return

async def recv_messages(ws):
    try:
        async for msg in ws:
            print("[ESP32]", msg)
    except Exception as e:
        # connection closed
        # print("[client] receive loop ended:", e)
        return

async def connect_loop():
    backoff = 1
    while True:
        print("[client] Trying to connect...")

        # 1) Try mDNS host first
        uri = f"ws://{ESP32_MDNS}{WS_PATH}"
        ws = await try_connect(uri, timeout=3)
        used_ip = None
        if ws:
            print("[client] connected via mDNS:", uri)
            used_ip = ESP32_MDNS
        else:
            # 2) Try last-known IP
            last_ip = None
            if os.path.exists(LAST_IP_FILE):
                try:
                    with open(LAST_IP_FILE, "r") as f:
                        last_ip = f.read().strip()
                except:
                    last_ip = None
            if last_ip:
                uri2 = f"ws://{last_ip}{WS_PATH}"
                ws = await try_connect(uri2, timeout=3)
                if ws:
                    print("[client] connected via last-known IP:", last_ip)
                    used_ip = last_ip

        if not ws:
            # 3) Listen for UDP announcements for a short time
            print("[client] Listening for UDP announcement for 5s...")
            ip = discover_via_udp(timeout=5)
            if ip:
                print("[client] discovered via UDP:", ip)
                # try to connect
                uri3 = f"ws://{ip}{WS_PATH}"
                ws = await try_connect(uri3, timeout=3)
                if ws:
                    print("[client] connected via UDP discovered IP:", ip)
                    used_ip = ip

        if not ws:
            # nothing worked — wait and retry (with backoff)
            print(f"[client] could not connect; retrying in {backoff}s...")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30)
            continue

        # connected!
        # if used_ip is a real IP (not mdns), save it
        try:
            if used_ip and used_ip != ESP32_MDNS:
                with open(LAST_IP_FILE, "w") as f:
                    f.write(used_ip)
        except:
            pass

        # create tasks: recv + interactive send
        try:
            await asyncio.gather(recv_messages(ws), interactive_send(ws))
        except Exception as e:
            print("[client] connection lost:", e)
        finally:
            try:
                await ws.close()
            except:
                pass

        print("[client] disconnected; will reconnect...")
        # small pause before reconnect
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(connect_loop()) 