# ports.py
from abc import ABC, abstractmethod
from mind.tools.weather_tools.core import WeatherData

class WeatherServicePort(ABC):
    def __init__(self):
        self.extra = {}
    @abstractmethod
    def get_current_weather(self, location: str) -> WeatherData:
        pass
