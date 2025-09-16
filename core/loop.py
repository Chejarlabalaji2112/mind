# core/loop.py
import asyncio
from adapters.esp32_adapter import Connection, Sender, Listener
from core.tool_registry import ToolRegistry
from core.agent import Agent
from utils.logging_handler import setup_logger

logger = setup_logger(__name__)

async def user_input_loop(agent):
    loop = asyncio.get_running_loop()
    while True:
        user_input = await loop.run_in_executor(None, input, ">>> ")
        result = await agent.handle_request(user_input)
        print(result)

async def main():
    # 1) Connect to ESP32 (adapter)
    conn = Connection()
    await conn.connect()
    sender_adapter = Sender(conn)  # implements Presenter
    listener = Listener(conn)
    asyncio.create_task(listener.listen())

    # 2) Inject adapter into core
    loop = asyncio.get_running_loop()
    tools = ToolRegistry(presenter=sender_adapter, loop=loop)
    agent = Agent(tools)

    # 3) Start async user input loop
    asyncio.create_task(user_input_loop(agent))

    # 4) Keep main loop alive
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("="*50)
        logger.info("Exiting.")
        logger.info("="*50)
        
