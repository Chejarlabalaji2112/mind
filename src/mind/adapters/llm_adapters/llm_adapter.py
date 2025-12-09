# adapters/llm_adapter.py
import json
from mind.ports.decision_making_port import DecisionMaker
# from mind.core.tool_registry import ToolRegistry
import openai  # or any other LLM provider

class LLMAdapter(DecisionMaker):
    """Concrete LLM adapter using OpenAI (or other LLM)."""

    def __init__(self, tools: ToolRegistry, model="gpt-4o-mini"):
        self.tools = tools
        self.model = model

    async def handle_input(self, user_input: str) -> str:
        # Define functions for LLM to call
        functions = [
            {
                "name": "set_timer",
                "description": "Set a timer in seconds",
                "parameters": {"type": "object", "properties": {"seconds": {"type": "integer"}}}
            },
            {"name": "pause_timer", "description": "Pause the timer"},
            {"name": "resume_timer", "description": "Resume the timer"},
            {"name": "stop_timer", "description": "Stop the timer"},
            # Add more functions for Stopwatch, Pomodoro, Skills, etc.
        ]

        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[{"role": "user", "content": user_input}],
            functions=functions,
            temperature=0,
        )

        message = response.choices[0].message

        # Parse function calls from LLM
        if "function_call" in message:
            fn_name = message.function_call.name
            args = {}
            if message.function_call.arguments:
                try:
                    args = json.loads(message.function_call.arguments)
                except Exception:
                    pass

            if fn_name == "set_timer":
                seconds = int(args.get("seconds", 0))
                return self.tools.set_timer(seconds)
            elif fn_name == "pause_timer":
                return self.tools.pause_timer()
            elif fn_name == "resume_timer":
                return self.tools.resume_timer()
            elif fn_name == "stop_timer":
                return self.tools.stop_timer()
            else:
                return f"Function {fn_name} not implemented."
        return message.content or "LLM did not return a function call"
