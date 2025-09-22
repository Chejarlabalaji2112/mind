# core/loop.py
import asyncio
import dotenv
from adapters.esp32_adapter import Connection, Sender, Listener, esp_output_tuner
from tools.tool_registry import ToolRegistry
from core.agent import Agent
from adapters.llm_gemini_adapter import GeminiLLMAdapter
from utils.logging_handler import setup_logger

logger = setup_logger(__name__)
dotenv.load_dotenv()

async def user_input_loop(agent: Agent, loop=None):
    loop = asyncio.get_running_loop() if loop is None else loop
    while True:
        user_input = await loop.run_in_executor(None, input, ">>> ")
        response = await agent.handle_input(user_input)
        print("from entry file:", response)

async def main():
    # ESP32
    conn = Connection()
    await conn.connect()
    sender_adapter = Sender(conn)
    listener = Listener(conn)
    asyncio.create_task(listener.listen())

    # Tools
    loop = asyncio.get_running_loop()
    tools = ToolRegistry(presenter=sender_adapter, loop=loop, output_tuner=esp_output_tuner)

    # LLM Agent
    llm_adapter = GeminiLLMAdapter(tools=tools, model="gemini-2.5-flash")
    agent = Agent(decision_maker=llm_adapter, tools=tools)

    asyncio.create_task(user_input_loop(agent, loop=loop))

    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("="*50)
        logger.info("Exiting.")
        logger.info("="*50)
