from mind.core.ports.emotions_handler import EmotionsHandler
from mind.core.ports.base_robot_controller_port import BaseRobotController

class MyEmotionsHandler(EmotionsHandler):
    def __init__(self, robot_controller: BaseRobotController):
        self.robot_controller = robot_controller

    def handle_emotions(self, mood: str = "Default", attend_to_position="center",  body_movement="idle"):
        self.robot_controller.set_eyes_mode(mood.upper())
        self.robot_controller.set_eyes_position(attend_to_position.upper())
        self.robot_controller.set_head_movement(body_movement.upper) #body movement handles say yes, no, or play with hands or antennas of the robot.
