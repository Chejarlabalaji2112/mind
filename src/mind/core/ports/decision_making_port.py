# ports/decision_making.py
from abc import ABC, abstractmethod

class DecisionMaker(ABC):
    """Abstract interface for an agent that decides which tools to call."""

    @abstractmethod
    async def handle_input(self, user_input: str) -> str:
        """Take user input and return a response or trigger a tool."""
        pass
