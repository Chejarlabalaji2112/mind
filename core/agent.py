# core/agent.py
from core.tool_registry import ToolRegistry
class Agent:
    """LLM or rule-based agent that calls tools via ToolRegistry."""

    def __init__(self, tools: ToolRegistry):
        self.tools = tools

    async def handle_request(self, user_input: str):
        """Rule-based for now; later swap with LLM function-calling."""
        user_input = user_input.lower()
        if "timer" in user_input:
            if "pause" in user_input:
                return self.tools.pause_timer()
            elif "resume" in user_input:
                return self.tools.resume_timer()
            elif "stop" in user_input:
                return self.tools.stop_timer()
            else:
                # extract number of minutes
                numbers = [int(s) for s in user_input.split() if s.isdigit()]
                if numbers:
                    seconds = numbers[0] * 60
                    return self.tools.set_timer(seconds)
                return "Please specify timer duration."
        return "Command not recognized."
