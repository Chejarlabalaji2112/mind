"""
This agent keeps track of the user health.
sleep monitoring, any recent health issues or cold or fever or etc.
"""

class HealthManager():
    def __init__(self, tools):
        self.tools = tools # specific tools from the central tools registry.
        pass