# core/agent.py
from mind.ports.decision_making_port import DecisionMaker
from mind.ports.act_port import Manipulator
from mind.utils.logging_handler import setup_logger
from mind.ports.base_robot_controller_port import BaseRobotController

logger = setup_logger(__name__)

class Agent():
    """Orchestrates tools using a DecisionMaker (LLM or other)."""

    def __init__(self, decision_maker: DecisionMaker, robot_controller: BaseRobotController ):
        self.decision_maker = decision_maker
        self.robot_controller = robot_controller

    def wake_up(self):
        self.robot_controller.wake_up()

    def sleep(self):
        self.robot_controller.sleep()

    def status(self):
        self.robot_controller.status()

    def stop(self):
        self.robot_controller.stop()

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
