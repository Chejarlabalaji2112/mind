# mind/core/types.py
from enum import Enum

class RobotStatus(Enum):
    SHUTDOWN = "shutdown"
    SLEEP = "sleep"
    ACTIVE = "active"