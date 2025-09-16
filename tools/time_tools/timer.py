import time
import threading
from .base_tool import TimeTool, Event
from utils.time_conversions import format_seconds_to_hms

class Timer(TimeTool):
    def __init__(self, loop=None):
        super().__init__()
        self._duration = 0
        self._remaining_time = 0
        self.on_finished = Event(loop=loop)

        # Pass the loop to tick events as well
        self.on_tick.loop = loop

    def start(self, duration):
        if self._is_running:
            print("Timer is already running.")
            return

        if duration <= 0:
            raise ValueError("Duration must be positive")

        self._duration = duration
        self._remaining_time = duration
        self._start_time = time.time()
        self._is_running = True
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True
        self._thread.start()
        self.on_start.emit(duration=duration)
        print(f"Timer started for {duration} seconds.")

    def reset(self):
        super().reset()
        self._duration = 0
        self._remaining_time = 0
        print("Timer reset.")

    def get_status(self):
        if self._is_running:
            elapsed = time.time() - self._start_time
            self._remaining_time = max(0, self._duration - (self._elapsed_at_pause + elapsed))
        return {
            "is_running": self._is_running,
            "duration": self._duration,
            "remaining_time": self._remaining_time,
            "elapsed_time": self._duration - self._remaining_time if self._duration > 0 else 0,
            "remaining_time_formatted": format_seconds_to_hms(self._remaining_time),
            "elapsed_time_formatted": format_seconds_to_hms(self._duration - self._remaining_time if self._duration > 0 else 0)
        }

    def _run(self):
        first_time=True
        last_text = None
        formatted = format_seconds_to_hms(self._remaining_time)
        if formatted != last_text:
            self.on_tick.emit(
                remaining_time=self._remaining_time,
                remaining_time_formatted=formatted
            )
            last_text = formatted
        while self._is_running and self._remaining_time > 0:
            elapsed = time.time() - self._start_time
            self._remaining_time = max(0, self._duration - (self._elapsed_at_pause + elapsed))

            if self._remaining_time <= 0:
                self.on_finished.emit()
                self._is_running = False
                print("Timer finished!")
                break

            formatted = format_seconds_to_hms(self._remaining_time)
            if formatted != last_text:
                if first_time:
                    first_time=False
                    time.sleep(0.3)
                self.on_tick.emit(
                    remaining_time=self._remaining_time,
                    remaining_time_formatted=formatted
                )
                last_text = formatted
                



            time.sleep(0.5)

            
            

        if not self._is_running and self._remaining_time > 0:
            print("Timer stopped/paused before completion.")
