import time
import threading
from time_tools.base_tool import TimeTool, Event

class Stopwatch(TimeTool):
    def __init__(self):
        super().__init__()
        self._elapsed_time = 0
        self._laps = []
        self.on_lap = Event()

    def start(self):
        if self._is_running:
            print("Stopwatch is already running.")
            return

        self._start_time = time.time()
        self._is_running = True
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True
        self._thread.start()
        self.on_start.emit()
        print("Stopwatch started.")

    def reset(self):
        super().reset()
        self._elapsed_time = 0
        self._laps = []
        print("Stopwatch reset.")

    def lap(self):
        if self._is_running:
            current_elapsed = self._elapsed_at_pause + (time.time() - self._start_time)
            self._laps.append(current_elapsed)
            self.on_lap.emit(lap_time=current_elapsed, all_laps=self._laps)
            print(f"Lap recorded: {current_elapsed:.2f} seconds.")
        else:
            print("Stopwatch is not running, cannot record lap.")

    def get_status(self):
        if self._is_running:
            self._elapsed_time = self._elapsed_at_pause + (time.time() - self._start_time)
        return {
            "is_running": self._is_running,
            "elapsed_time": self._elapsed_time,
            "laps": self._laps
        }

    def _run(self):
        while self._is_running:
            current_elapsed = self._elapsed_at_pause + (time.time() - self._start_time)
            self.on_tick.emit(elapsed_time=current_elapsed)
            time.sleep(0.1) # Update every 100ms
