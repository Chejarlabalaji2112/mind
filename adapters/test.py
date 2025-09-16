import asyncio
import time
from adapters.esp32_adapter import Connection, Sender, Listener
from tools.time_tools.pomodoro import Pomodoro
from tools.time_tools.stopwatch import Stopwatch
from tools.time_tools.timer import Timer    
from utils.time_conversions import convert_to_seconds
from tools.time_tools.clock import Clock
from utils.logging_handler import setup_logger

logger = setup_logger(__name__)

async def main():
    # 1) Connect to ESP32
    conn = Connection()
    await conn.connect()
    sender = Sender(conn)
    listener = Listener(conn)
    asyncio.create_task(listener.listen())

    loop = asyncio.get_running_loop()

    def screen_handler(title, center, bottom="", x=0, y=0, size=2, clear=True):
            text = f"{title}{center}\n{bottom}"
            loop.call_soon_threadsafe(
            lambda: asyncio.create_task(
                sender.show({
                    "mode": "text",
                    "text": text,
                    "x": x,
                    "y": y,
                    "size": size,
                    "clear": clear
                })
            )
        )

         
    def timer_tick_handler(remaining_time,remaining_time_formatted):
        screen_handler("   Timer", f"\n  {remaining_time_formatted}")

    def timer_finished_handler():
       screen_handler("   Timer", f"\n   finished!")

    def stopwatch_tick_handler(elapsed_time:str, elapsed_time_formatted:str):
        screen_handler(" StopWatch", f"\n {elapsed_time_formatted}")
        
    def pomodoro_tick_handler(remaining_time, phase, remaining_time_formatted):
        bottom = f" {phase[0]}    {pomodoro._current_cycle}/{pomodoro._cycles_before_long_break}" # we should not access private variables like this
        screen_handler(" Pomodoro", f"\n {remaining_time_formatted}", bottom)


    # 3) Create Timer and pass loop
    # timer_duration = convert_to_seconds(minutes=0, seconds=5)
    # timer = Timer(loop=loop)
    # timer.on_tick.add_listener(timer_tick_handler)
    # timer.on_finished.add_listener(timer_finished_handler)
    # timer.start(timer_duration)
    # await asyncio.sleep(timer_duration + 1)
    # asyncio.create_task(sender.show({"mode": "eyes"}))
    # sw = Stopwatch(loop=loop)
    # sw.on_tick.add_listener(stopwatch_tick_handler)
    # sw.start()
    # await asyncio.sleep(10)
    # sw.pause()
    # await asyncio.sleep(3)
    # sw.resume()
    # await asyncio.sleep(5)
    # pomodoro = Pomodoro(work_duration_h=0, work_duration_m=0, work_duration_s=4,
    #              short_break_duration_h=0, short_break_duration_m=0, short_break_duration_s=2,
    #              long_break_duration_h=0, long_break_duration_m=0, long_break_duration_s=3,
    #              cycles_before_long_break=4,loop=loop)
    # pomodoro.on_tick.add_listener(pomodoro_tick_handler)
    # pomodoro.start()
    # await loop.run_in_executor(None, pomodoro._thread.join)
    while True:
        date, time= Clock.get_current_datetime_str(timezone_str="Asia/kolkata").split("|")
        screen_handler(date, "\n\n"+time, size=2) 
        await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("[client] exiting.")
