from toolregistry import ToolRegistry
import time
from mind.core.ports.memory_port import MemoryPort
from mind.tools import Timer, Stopwatch, Pomodoro, SkillsTracker, WeatherUpdater
class ToolsInstances:
    def __init__(self, loop=None, memory_adapter=None):
        self.timer = Timer(loop)
        self.stopwatch = Stopwatch(loop)
        self.pomodoro = Pomodoro(loop)
        self.skillstracker = SkillsTracker(memory_adapter)


class Register:
    def __init__(self, loop=None, memory_adapter: MemoryPort =None):
        self.registry = ToolRegistry()
        self.tools_instances = ToolsInstances(loop, memory_adapter) # are we handling database connect and closing properly..
        self.registry.register_from_class(self.tools_instances.timer, with_namespace=True)
        self.registry.register_from_class(self.tools_instances.stopwatch,with_namespace=True)
        self.registry.register_from_class(self.tools_instances.pomodoro,with_namespace=True)
        self.registry.register_from_class(self.tools_instances.skillstracker, with_namespace=True)


#TODO: implement a easy lookup table. given the agent name then returns a registory of tools for that agent. example utils_agent then only gives tools like timer, stopwatch etc. no other tools like robotcontroller.

