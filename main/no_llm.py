# core/loop.py
import asyncio
from adapters.esp32_adapter import Connection, Sender, Listener, esp_output_tuner
from utils.logging_handler import setup_logger
import re

logger = setup_logger(__name__)

async def user_input_loop(sender_adapter,loop=None, ):
    loop = asyncio.get_running_loop() if loop is None else loop
    while True:
        user_input = await loop.run_in_executor(None, input, ">>> ")
        await handle_user_command(sender_adapter,user_input)
        

async def handle_user_command(sender: Sender, command: str):
    command = command.lower().strip()
    try:
        # --- Timer ---
        if command.startswith("start timer"):
            m = re.search(r"(\d+)\s*(seconds|second|s|minutes|minute|m|hours|hour|h)", command)
            if not m:
                raise ValueError("No duration found in timer command")
            value = int(m.group(1))
            unit = m.group(2)
            if unit.startswith("s"):
                duration = value
            elif unit.startswith("m"):
                duration = value * 60
            elif unit.startswith("h"):
                duration = value * 3600
            else:
                duration = value
            await sender.show({"mode": "timer", "action": "start", "duration": duration})
            return

        if command == "pause timer":
            await sender.show({"mode": "timer", "action": "pause"})
            return
        if command == "resume timer":
            await sender.show({"mode": "timer", "action": "resume"})
            return
        if command == "stop timer":
            await sender.show({"mode": "timer", "action": "stop"})
            return

        # --- Stopwatch ---
        if command == "start stopwatch":
            await sender.show({"mode": "stopwatch", "action": "start"})
            return
        if command == "pause stopwatch":
            await sender.show({"mode": "stopwatch", "action": "pause"})
            return
        if command == "resume stopwatch":
            await sender.show({"mode": "stopwatch", "action": "resume"})
            return
        if command == "stop stopwatch":
            await sender.show({"mode": "stopwatch", "action": "stop"})
            return

        # --- Pomodoro ---
        if command.startswith("start pomodoro"):
            # default 25/5/15
            work, brk, lbreak = 25, 5, 15
            # allow "start pomodoro 0.1 0.01 0.2"
            numbers = re.findall(r"[\d.]+", command)
            if len(numbers) >= 1: work = float(numbers[0])
            if len(numbers) >= 2: brk = float(numbers[1])
            if len(numbers) >= 3: lbreak = float(numbers[2])
            await sender.show({"mode": "pomodoro", "action": "start", 
                            "work": work, "break": brk, "lBreak": lbreak})
            return

        if command == "pause pomodoro":
            await sender.show({"mode": "pomodoro", "action": "pause"})
            return
        if command == "resume pomodoro":
            await sender.show({"mode": "pomodoro", "action": "resume"})
            return
        if command == "stop pomodoro":
            await sender.show({"mode": "pomodoro", "action": "stop"})
            return

        raise ValueError(f"Unknown command: {command}")
  
    except ValueError:
        logger.info("unknown command is given.....")


        

async def main():
    # ESP32
    conn = Connection()
    await conn.connect()
    sender_adapter = Sender(conn)
    listener = Listener(conn)
    asyncio.create_task(listener.listen())

    loop = asyncio.get_running_loop()
    
    asyncio.create_task(user_input_loop(sender_adapter, loop=loop))

    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("="*50)
        logger.info("Exiting.")
        logger.info("="*50)

