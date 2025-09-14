
import requests
from ports import WeatherServicePort
from core import WeatherData

class OpenMeteoAdapter(WeatherServicePort):
    def __init__(self):
        super().__init__()  # ensures self.extra = {}

    def _geocode(self, location: str):
        url = f"https://geocoding-api.open-meteo.com/v1/search?name={location}&count=1"
        response = requests.get(url).json()
        results = response.get("results")
        if not results:
            raise ValueError(f"Location '{location}' not found")
        return results[0]["latitude"], results[0]["longitude"]

    def get_current_weather(self, location: str) -> WeatherData:
        lat, lon = self._geocode(location)
        url = f"https://api.open-meteo.com/v1/forecast?latitude={lat}&longitude={lon}&current_weather=true"
        data = requests.get(url).json()["current_weather"]

        temperature_c = data["temperature"]
        condition = f"code_{data['weathercode']}"

        # assign extra once
        self.extra = {k: v for k, v in data.items() if k not in ["temperature", "weathercode"]}

        return WeatherData(temperature_c, condition, self.extra)

#TODO: Add error handling for network issues and unexpected API responses or unknown locations.