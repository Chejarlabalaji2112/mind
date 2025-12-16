import time
import threading
from enum import Enum, auto
from mind.tools.time_tools.base_tool import TimeTool, Event
from mind.utils.time_conversions import convert_to_seconds, format_seconds_to_hms
from mind.utils.logging_handler import setup_logger

logger = setup_logger(__name__)

class PomodoroPhase(Enum):
    WORK = auto()
    SHORT_BREAK = auto()
    LONG_BREAK = auto()
    IDLE = auto()

class Pomodoro(TimeTool):
    """
    A specialized timer for the Pomodoro Technique.
    Manages cycles of Work, Short Breaks, and Long Breaks, handling automatic 
    or manual transitions between these phases.
    """
    def __init__(self,loop=None):
        """
        Initializes the Pomodoro timer.
        Sets default durations for work (25m), short break (5m), and long break (10m).
        """
        super().__init__()
        self.loop = loop # this is for async
        self.pom_types = {'short': [25,5, 10], "long": [[50, 10, 20]]}
        self._work_duration = convert_to_seconds(25, 0, 0)  # Default 25 minutes
        self._short_break_duration = convert_to_seconds(5, 0, 0)  # Default 5 minutes
        self._long_break_duration = convert_to_seconds(10, 0, 0)  # Default 10 minutes
        self._cycles_before_long_break = 4

        self._current_cycle = 0
        self._current_phase = PomodoroPhase.IDLE
        self._remaining_time = 0

        self.on_work_start = Event()
        self.on_short_break_start = Event()
        self.on_long_break_start = Event()
        self.on_phase_end = Event()
        self.on_cycle_complete = Event()

    def start(self, pom_type: str = "short"):
        """
        Starts the Pomodoro session.

        Args:
            pom_type (str): Configuration type for durations (e.g., 'short', 'long').
                            Defaults to standard Pomodoro settings.
        """
        if self._is_running:
            logger.warning("Pomodoro is already running.")
            return
        if self._current_phase == PomodoroPhase.IDLE:
            self._current_cycle = 1
            if pom_type == 'long':
                self._set_duration(pom_type)
            self._transition_phase(PomodoroPhase.WORK)
        else:
            # If paused, just resume the current phase
            super().resume()
        logger.info("Pomodoro started.")

    def pause(self):
        """
        Pauses the current phase of the Pomodoro.
        """
        super().pause()
        logger.info("Pomodoro paused.")

    def resume(self):
        """
        Resumes the current phase if paused. 
        If the Pomodoro is IDLE, it starts a new session.
        """
        if not self._is_running and self._current_phase != PomodoroPhase.IDLE:
            super().resume()
            logger.info("Pomodoro resumed.")
        elif self._current_phase == PomodoroPhase.IDLE:
            self.start() # If idle, start a new pomodoro
        else:
            logger.warning("Pomodoro is already running or cannot be resumed from current state.")

    def stop(self):
        """Stops the current session and sets the phase to IDLE."""
        super().stop()
        self._current_phase = PomodoroPhase.IDLE
        self._remaining_time = 0
        logger.info("Pomodoro stopped.")

    def reset(self):
        """Resets the cycle count, current phase, and remaining time to defaults."""
        super().reset()
        self._current_cycle = 0
        self._current_phase = PomodoroPhase.IDLE
        self._remaining_time = 0
        logger.info("Pomodoro reset.")

    def get_status(self):
        """
        Returns the current status of the Pomodoro session.

        Returns:
            dict: Contains 'current_phase', 'current_cycle', 'remaining_time', etc.
        """
        if self._is_running:
            elapsed = time.time() - self._start_time
            self._remaining_time = max(0, self._get_current_phase_duration() - (self._elapsed_at_pause + elapsed))
        return {
            "is_running": self._is_running,
            "current_phase": self._current_phase.name,
            "current_cycle": self._current_cycle,
            "remaining_time": self._remaining_time,
            "total_phase_duration": self._get_current_phase_duration(),
            "remaining_time_formatted": format_seconds_to_hms(self._remaining_time)
        }

    def _get_current_phase_duration(self):
        """
        Returns the total duration of the current phase."""
        if self._current_phase == PomodoroPhase.WORK:
            return self._work_duration
        elif self._current_phase == PomodoroPhase.SHORT_BREAK:
            return self._short_break_duration
        elif self._current_phase == PomodoroPhase.LONG_BREAK:
            return self._long_break_duration
        return 0

    def _transition_phase(self, next_phase):
        """
        Internal method to switch from the current phase to the next (e.g., Work -> Short Break).
        Resets internal timers and emits start events for the new phase.
        """
        self.on_phase_end.emit(previous_phase=self._current_phase.name, current_cycle=self._current_cycle)
        self._current_phase = next_phase
        self._elapsed_at_pause = 0 # Reset elapsed for new phase
        self._start_time = time.time()
        self._is_running = True
        self._remaining_time = self._get_current_phase_duration()

        if next_phase == PomodoroPhase.WORK:
            self.on_work_start.emit(cycle=self._current_cycle)
            logger.info(f"Starting Work Phase (Cycle {self._current_cycle}) for {format_seconds_to_hms(self._work_duration)}.")
        elif next_phase == PomodoroPhase.SHORT_BREAK:
            self.on_short_break_start.emit(cycle=self._current_cycle)
            logger.info(f"Starting Short Break for {format_seconds_to_hms(self._short_break_duration)}.")
        elif next_phase == PomodoroPhase.LONG_BREAK:
            self.on_long_break_start.emit(cycle=self._current_cycle)
            logger.info(f"Starting Long Break for {format_seconds_to_hms(self._long_break_duration)}.")
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._run)
            self._thread.daemon = True
            self._thread.start()

    def _set_duration(self, pom_type: str):
        """
        Set the durations based on the selected pomodoro type.
        """
        self._work_duration = convert_to_seconds(self.pom_types[pom_type][0], 0, 0)  # Default 25 minutes
        self._short_break_duration = convert_to_seconds(self.pom_types[pom_type][1], 0, 0)  # Default 5 minutes
        self._long_break_duration = convert_to_seconds(self.pom_types[type][2], 0, 0)  # Default 10 minutes


    def _run(self):
        """
        The internal loop. 
        Manages the countdown for the current phase and automatically triggers 
        phase transitions or stops upon completion.
        """
        first_time=True
        last_text = None
        formatted = format_seconds_to_hms(self._remaining_time)
        self.on_tick.emit(
                    remaining_time=self._remaining_time, 
                    phase=self._current_phase.name, 
                    remaining_time_formatted=formatted
                    )
        last_text = formatted 
        while self._is_running and self._remaining_time > 0:
            current_elapsed = time.time() - self._start_time
            self._remaining_time = max(0, self._get_current_phase_duration() - (self._elapsed_at_pause + current_elapsed))
            formatted = format_seconds_to_hms(self._remaining_time)
            if formatted != last_text:
                if first_time:
                    first_time=False
                    time.sleep(0.3)
                self.on_tick.emit(
                    remaining_time=self._remaining_time, 
                    phase=self._current_phase.name, 
                    remaining_time_formatted=formatted
                    )
                last_text = formatted   
            if self._remaining_time <= 0:
                self._is_running = False
                self.on_phase_end.emit(previous_phase=self._current_phase.name, current_cycle=self._current_cycle)
                logger.info(f"{self._current_phase.name} phase finished.")

                if self._current_phase == PomodoroPhase.WORK:
                    self.on_cycle_complete.emit(cycle=self._current_cycle)
                    if self._current_cycle % self._cycles_before_long_break == 0:
                        self._transition_phase(PomodoroPhase.LONG_BREAK)
                    else:
                        self._transition_phase(PomodoroPhase.SHORT_BREAK)
                elif self._current_phase in [PomodoroPhase.SHORT_BREAK, PomodoroPhase.LONG_BREAK]:
                    self._current_cycle = (self._current_cycle % self._cycles_before_long_break) + 1

                    self._transition_phase(PomodoroPhase.WORK)
                continue
            time.sleep(0.5)
        if not self._is_running and self._remaining_time > 0:
            logger.warning(f"Pomodoro {self._current_phase.name} phase stopped/paused before completion.")
