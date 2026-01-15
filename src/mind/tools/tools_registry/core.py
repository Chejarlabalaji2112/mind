from toolregistry import ToolRegistry
import time
import inspect  # Add this import for accurate type checking
from mind.core.ports.memory_port import MemoryPort
from mind.tools import Timer, Stopwatch, Pomodoro, SkillsTracker, WeatherUpdater




class ToolsInstances:
    def __init__(self, loop=None, memory_adapter=None):
        self.timer = Timer(loop)
        self.stopwatch = Stopwatch(loop)
        self.pomodoro = Pomodoro(loop)
        self.skillstracker = SkillsTracker(memory_adapter)

class Register:
    def __init__(self, loop=None, memory_adapter: MemoryPort = None):
        self.registry = ToolRegistry()
        self.tools_instances = ToolsInstances(loop, memory_adapter)
        
        # Central dict for all tools (instances and functions)
        self.all_tools = {
            "timer": self.tools_instances.timer,      # Instance-based
            "stopwatch": self.tools_instances.stopwatch,
            "pomodoro": self.tools_instances.pomodoro,
            "skillstracker": self.tools_instances.skillstracker,   
        }
        
        # Register based on type
        for tool_name, tool in self.all_tools.items():
            if inspect.isfunction(tool):
                self.registry.register(tool)  # For plain functions
            else:  # For classes/instances
                self.registry.register_from_class(tool, with_namespace=True)

    def get_agent_registry(self, agent_name: str) -> ToolRegistry:
        agent_registry = ToolRegistry()
        assigned_tools = agents_tools.get(agent_name, [])
        
        for tool_name in assigned_tools:
            tool = self.all_tools.get(tool_name)
            if tool:
                if inspect.isfunction(tool):
                    agent_registry.register(tool)
                else:
                    agent_registry.register_from_class(tool, with_namespace=True)
        
        return agent_registry

# Update agents_tools to include functions (added 'subtract' for completeness)
agents_tools = {
    "utils_manager": ["timer", "stopwatch", "pomodoro", "add", "subtract"],
    # ...
}

if __name__ == "__main__":
    register = Register()
    from pprint import pprint
    utils_reg = register.get_agent_registry("utils_manager")
    pprint(utils_reg.get_callable("timer-start").__doc__)