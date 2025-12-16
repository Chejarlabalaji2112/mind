from toolregistry import ToolRegistry
import time
from mind.tools import Timer, Stopwatch, Pomodoro
class ToolInstances:
    def __init__(self, loop=None):
        self.timer = Timer(loop)
        self.stopwatch = Stopwatch(loop)
        self.pomodoro = Pomodoro(loop)


class Register:
    def __init__(self, loop=None):
        self.registry = ToolRegistry()
        self.tool_instances = ToolInstances(loop)
        self.registry.register_from_class(self.tool_instances.timer, with_namespace=True)
        self.registry.register_from_class(self.tool_instances.stopwatch,with_namespace=True)
        self.registry.register_from_class(self.tool_instances.pomodoro,with_namespace=True)


