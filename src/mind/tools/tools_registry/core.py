from toolregistry import ToolRegistry
import time
from mind.ports.memory_port import MemoryPort
from mind.tools import Timer, Stopwatch, Pomodoro, SkillsTracker, WeatherUpdater
class ToolInstances:
    def __init__(self, loop=None, memory_adapter=None):
        self.timer = Timer(loop)
        self.stopwatch = Stopwatch(loop)
        self.pomodoro = Pomodoro(loop)
        self.skillstracker = SkillsTracker(memory_adapter)


class Register:
    def __init__(self, loop=None, memory_adapter: MemoryPort =None):
        self.registry = ToolRegistry()
        self.tool_instances = ToolInstances(loop, memory_adapter) # are we handling database connect and closing properly..
        self.registry.register_from_class(self.tool_instances.timer, with_namespace=True)
        self.registry.register_from_class(self.tool_instances.stopwatch,with_namespace=True)
        self.registry.register_from_class(self.tool_instances.pomodoro,with_namespace=True)
        self.registry.register_from_class(self.tool_instances.skillstracker, with_namespace=True)


