# core/agent.py
from ports.decision_making import DecisionMaker
from tools.tool_registry import ToolRegistry
from utils.logging_handler import setup_logger

logger = setup_logger(__name__)

class Agent:
    """Orchestrates tools using a DecisionMaker (LLM or other)."""

    def __init__(self, decision_maker: DecisionMaker, tools: ToolRegistry):
        self.decision_maker = decision_maker
        self.tools = tools

    async def handle_input(self, user_input: str):
        """
        Forward user input to the DecisionMaker (LLM) and handle tool execution.
        The LLM adapter decides which tool to call.
        """
        try:

            response = await self.decision_maker.handle_input({'input':user_input})
            logger.info(f"Agent handled input: {user_input} -> {response}")
            return response
        except Exception as e:
            logger.exception(f"Error handling input: {e}")
            return f"Error: {e}"
