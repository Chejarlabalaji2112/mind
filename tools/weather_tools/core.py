#weathercore.py 
class WeatherData:
    def __init__(self, temperature_celsius, condition, extra=None):
        self.temperature_celsius = temperature_celsius
        self.condition = condition
        self.extra = extra or {}  # default empty dict

    def __repr__(self):
        if self.extra:
            extras_str = "\n".join(f"{k}={v}" for k, v in self.extra.items())
            return (
                f"WeatherData(\n"
                f"temperature_celsius={self.temperature_celsius}\n"
                f"condition='{self.condition}'\n"
                f"{extras_str}\n"
                f")"
            )
        else:
            return (
                f"WeatherData(\n"
                f"temperature_celsius={self.temperature_celsius}\n"
                f"condition='{self.condition}'\n"
                f")"
            )
