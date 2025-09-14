import asyncio
import time
from adapters.esp32_adapter import Connection, Sender, Listener
from tools.time_tools.timer import Timer
from utils.time_conversions import convert_to_seconds

async def main():
    # 1) Connect to ESP32
    conn = Connection()
    await conn.connect()
    sender = Sender(conn)
    listener = Listener(conn)
    asyncio.create_task(listener.listen())

    loop = asyncio.get_running_loop()

    # 2) Timer handlers
    def timer_tick_handler(remaining_time: float, remaining_time_formatted: str):
        loop.call_soon_threadsafe(
            lambda: asyncio.create_task(
                sender.show({
                    "mode": "text",
                    "text": remaining_time_formatted,
                    "x": 20,
                    "y": 20,
                    "size": 2
                })
            )
        )

    def timer_finished_handler():
        print("called finish")
        loop.call_soon_threadsafe(
            lambda: asyncio.create_task(
                sender.show({
                    "mode": "text",
                    "text": "Time's up!",
                    "x": 3,
                    "y": 20,
                    "size": 2
                })
            )
        )

    # 3) Create Timer and pass loop
    timer_duration = convert_to_seconds(minutes=0, seconds=5)
    timer = Timer(loop=loop)
    timer.on_tick.add_listener(timer_tick_handler)
    timer.on_finished.add_listener(timer_finished_handler)

    print("\n--- Demonstrating Timer ---")
    timer.start(timer_duration)
    await asyncio.sleep(7)
    input("continue too home?")
    asyncio.create_task(sender.show({"mode": "eyes"}))
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[client] exiting.")
