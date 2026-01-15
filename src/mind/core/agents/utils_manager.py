"""
this is an agent which is responsible for managing utils.
THis agent can start, stop tools like timer, stopwatch etc.
"""
from mind.core.ports.decision_making_port import DecisionMaker

class UtilsManager:
    def __init__(self, decision_maker: DecisionMaker):
        self.decision_maker = decision_maker

    async def handle_input(self, input:str):
        response = await self.decision_maker.handle_input(input)
