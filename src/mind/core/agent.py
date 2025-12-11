# core/agent.py
import asyncio
from mind.ports.decision_making_port import DecisionMaker
from mind.core.status import RobotStatus
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

    def shutdown(self):
        self.robot_controller.shutdown()

    def stop(self):
        self.robot_controller.stop()

    def handle_power_command(self, action: str) -> RobotStatus:
        """
        Centralized logic for power state transitions.
        Returns the new status.
        """
        current_status = RobotStatus(self.robot_controller.status())

        if action == "on":
            if current_status in [RobotStatus.SHUTDOWN, RobotStatus.SLEEP]:
                self.wake_up()
        elif action == "shutdown":
            if current_status in [RobotStatus.ACTIVE, RobotStatus.SLEEP]:
                self.shutdown()
        elif action == "sleep":
            if current_status == RobotStatus.ACTIVE:
                self.sleep()
        else:
            # Logic for rejecting invalid states happens here, not in FastAPI
            logger.warning(f"Invalid transition: {current_status} -> {action}")
        
        return RobotStatus(self.robot_controller.status())

    async def handle_input_stream(self, user_input: str):
            """
            Generator that yields chunks of the response.
            This allows the Agent to intercept or modify the stream if needed later.
            """
            logger.info(f"Agent receiving stream request: {user_input}")
            
            # We delegate to the decision maker (LLM), but WE call it, not FastAPI.
            # Assuming decision_maker has a .stream() method. 
            # If your Port defines it differently, adapt here.
            try:
                # Check if decision_maker has stream capability
                if hasattr(self.decision_maker, 'stream'):
                    # Iterate over the LLM's generator
                    async for chunk in self.decision_maker.stream(user_input):
                        # We can add logic here (e.g., check for tool calls in the stream)
                        if hasattr(chunk, 'content'):
                            yield chunk.content
                        else:
                            yield str(chunk)
                            
                else:
                    # Fallback for non-streaming models
                    yield self.decision_maker.handle_input(user_input)
                
            except Exception as e:
                logger.error(f"Error during streaming: {e}")
                yield f"[Error processing request: {e}]"
