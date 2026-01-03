# core/agent.py
from mind.core.ports.decision_making_port import DecisionMaker
from mind.core.status import RobotStatus
from mind.utils.logging_handler import setup_logger
from mind.core.ports.base_robot_controller_port import BaseRobotController
from mind.core.ports.notification_port import NotificationPort



logger = setup_logger(__name__)

class Agent():
    """Orchestrates tools using a DecisionMaker (LLM or other)."""

    def __init__(self, decision_maker: DecisionMaker, robot_controller: BaseRobotController, notifier: NotificationPort, loop=None ):
        self.decision_maker = decision_maker
        self.robot_controller = robot_controller
        self.notifier = notifier

    def wake_up(self):
        self.robot_controller.wake_up()
        self.notifier.notify_status("Activating")

    def sleep(self):
        self.robot_controller.sleep()
        self.notifier.notify_status("To sleep")

    def status(self):
        self.robot_controller.status()

    def shutdown(self):
        self.robot_controller.shutdown()
        self.notifier.notify_status("Shutting down")

    def stop(self):
        self.robot_controller.stop()

    def head_control(self, do=None):
        if do == "yes":
            self.robot_controller.nod_yes()

        elif do == "no":
            self.robot_controller.shake_no()

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
            

    def input_handler(self):
            logger.info(f"Agent receiving request")

            try:
                response = self.decision_maker.input_handler()
                return response
                
            except Exception as e:
                logger.error(f"Error during streaming: {e}")
                return f"[Error processing request: {e}]"
