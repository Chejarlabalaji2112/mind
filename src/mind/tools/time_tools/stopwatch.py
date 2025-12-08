import time
import threading
from mind.tools.time_tools.base_tool import TimeTool, Event
from mind.utils.time_conversions import format_seconds_to_hms
from mind.utils.logging_handler import setup_logger

logger = setup_logger(__name__)

class Stopwatch(TimeTool):
    def __init__(self, loop=None):
        super().__init__()
        self._elapsed_time = 0
        self._laps = []
        self.on_tick.loop = loop
        self.on_lap = Event()

    def start(self):
        if self._is_running:
            logger.warning("Stopwatch is already running.")
            return

        self._start_time = time.time()
        self._is_running = True
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True
        self._thread.start()
        self.on_start.emit()
        logger.info("Stopwatch started.")

    def reset(self):
        super().reset()
        self._elapsed_time = 0
        self.on_reset.emit(elapsed_time=0, elapsed_time_formatted=format_seconds_to_hms(0))
        self._laps = []
        logger.info("Stopwatch reset.")

    def lap(self):
        if self._is_running:
            current_elapsed = self._elapsed_at_pause + (time.time() - self._start_time)
            self._laps.append(current_elapsed)
            self.on_lap.emit(lap_time=current_elapsed, lap_time_formatted=format_seconds_to_hms(current_elapsed), all_laps=self._laps)
            logger.info(f"Lap recorded: {format_seconds_to_hms(current_elapsed)}.")
        else:
            logger.warning("Stopwatch is not running, cannot record lap.")

    def get_status(self):
        if self._is_running:
            self._elapsed_time = self._elapsed_at_pause + (time.time() - self._start_time)
        return {
            "is_running": self._is_running,
            "elapsed_time": self._elapsed_time,
            "elapsed_time_formatted": format_seconds_to_hms(self._elapsed_time),
            "laps": self._laps,
            "laps_formatted": [format_seconds_to_hms(lap) for lap in self._laps]
        }

    def _run(self):
        last_text = None
        while self._is_running:
            current_elapsed = self._elapsed_at_pause + (time.time() - self._start_time)
            formatted = format_seconds_to_hms(current_elapsed)
            if formatted != last_text:
                self.on_tick.emit(
                    elapsed_time=current_elapsed,
                      elapsed_time_formatted=formatted
                )
                last_text = formatted       
            
            time.sleep(0.1) # Update every 100ms
