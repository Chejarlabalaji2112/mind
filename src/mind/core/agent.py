# core/agent.py
from mind.ports.decision_making import DecisionMaker
from mind.ports.act_port import Manipulator
from mind.utils.logging_handler import setup_logger
from mind.core.base_agent import BaseAgent

logger = setup_logger(__name__)

class Agent(BaseAgent):
    """Orchestrates tools using a DecisionMaker (LLM or other)."""

    def __init__(self, decision_maker: DecisionMaker, agent: BaseAgent ):
        self.decision_maker = decision_maker
        self._agent = agent

    def wake_up(self):
        self._agent.wakeup()

    def sleep(self):
        self._agent.sleep()

    def status(self):
        self._agent.status()

    def run(self):
        self._agent.run()

    def stop(self):
        self._agent.stop()

    def handle_input(self, user_input: str):
        """
        Forward user input to the DecisionMaker (LLM) and handle tool execution.
        The LLM adapter decides which tool to call.
        """
        try:

            response = self.decision_maker.handle_input({'input':user_input})
            logger.info(f"Agent handled input: {user_input} -> {response}")
            return response
        except Exception as e:
            logger.exception(f"Error handling input: {e}")
            return f"Error: {e}"
