import asyncio
import json
from ports.act_port import Presenter
from ports.percieve_port import Audition
import logging

# ------------------------------
# Dummy logger setup
# ------------------------------
def setup_logger(name, console=True):
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    if console and not logger.handlers:
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        formatter = logging.Formatter(f"[{name}] %(message)s")
        ch.setFormatter(formatter)
        logger.addHandler(ch)
    return logger

connection_logger = setup_logger("connection")
sender_logger = setup_logger("sender")
listener_logger = setup_logger("listener")

# ------------------------------
# Dummy Connection
# ------------------------------
class Connection:
    """Dummy connection that does nothing."""

    def __init__(self):
        self.ws = None

    async def connect(self):
        connection_logger.info("[dummy] connect called")

    async def send(self, payload: str):
        sender_logger.info(f"[dummy] send called with payload: {payload}")

    async def receive(self):
        listener_logger.info("[dummy] receive called")
        while True:
            await asyncio.sleep(1)
            yield json.dumps({"dummy": "message"})

    async def close(self):
        connection_logger.info("[dummy] close called")


# ------------------------------
# Dummy Sender
# ------------------------------
class Sender(Presenter):
    def __init__(self, connection: Connection):
        self.conn = connection

    async def show(self, obj: dict):
        sender_logger.info(f"[dummy] show called with: {obj}")
        await self.conn.send(json.dumps(obj))


# ------------------------------
# Dummy Listener
# ------------------------------
class Listener(Audition):
    def __init__(self, connection: Connection):
        self.conn = connection

    async def listen(self):
        async for msg in self.conn.receive():
            pass


# ------------------------------
# Dummy esp_output_tuner
# ------------------------------
def esp_output_tuner(title="None", content="", bottom="", /, home=False):
    if home:
        return {"mode": "eyes"}
    return {
        "mode": "text",
        "clear": True,
        "items": [
            {"text": title, "x": 0, "y": 0, "size": 1},
            {"text": content, "x": 10, "y": 20, "size": 2},
            {"text": bottom, "x": 0, "y": 54, "size": 1},
        ],
    }

# ------------------------------
# Example usage
# ------------------------------
async def main():
    conn = Connection()
    await conn.connect()

    sender = Sender(conn)
    listener = Listener(conn)

    asyncio.create_task(listener.listen())

    await sender.show({"mode": "text", "text": "Hello ESP32 (dummy)"})
    await asyncio.sleep(1)
    await sender.show({"mode": "eyes", "mood": "happy", "direction": "center"})
    await sender.show({"mode": "clear"})
    await asyncio.sleep(1)
    await conn.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        connection_logger.info("[dummy] exiting.")
