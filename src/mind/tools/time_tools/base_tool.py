import threading
import time
from abc import ABC, abstractmethod
import asyncio
import inspect
from mind.utils.logging_handler import setup_logger
from mind.utils import Event

logger = setup_logger(__name__)


class TimeTool(ABC):
    """
    An abstract base class for time management tools (e.g., Timer, Stopwatch).
    It handles the fundamental state logic for starting, pausing, resuming,
    and resetting time-based operations, as well as managing event emissions.
    """
    def __init__(self):
        """Initializes the TimeTool with default states and event hooks."""
        self._is_running = False
        self._thread = None
        self._start_time = None
        self._pause_time = None
        self._elapsed_at_pause = 0
        self.paused = False

        self.on_tick = Event()
        self.on_start = Event()
        self.on_pause = Event()
        self.on_resume = Event()
        self.on_stop = Event()
        self.on_reset = Event()

    @property
    def is_running(self):
        """Property that returns True if the tool is currently active and running."""
        return self._is_running

    @abstractmethod
    def start(self, *args, **kwargs):
        """
        Abstract method to start the tool.
        Must be implemented by subclasses to define specific start behavior.
        """
        pass

    def pause(self):
        """
        Pauses the current operation.
        Records the time of the pause to calculate elapsed duration correctly upon resumption.
        """
        if self._is_running:
            self._is_running = False
            self.paused = True
            self._pause_time = time.time()
            if self._start_time is not None:
                self._elapsed_at_pause += (self._pause_time - self._start_time)
            self.on_pause.emit()
            logger.info(f"{self.__class__.__name__} paused.")

    def resume(self):
        """
        Resumes the operation from the paused state.
        Restarts the internal thread and adjusts start time to account for the pause duration.
        """
        if not self._is_running and self._start_time is not None:
            self._is_running = True
            self.paused = False
            self._start_time = time.time()
            self._thread = threading.Thread(target=self._run)
            self._thread.daemon = True
            self._thread.start()
            self.on_resume.emit()
            logger.info(f"{self.__class__.__name__} resumed.")

    def stop(self):
        """
        Stops the tool completely.
        Joins the internal thread and emits the stop event.
        """
        if self._is_running:
            self._is_running = False
            if self._thread:
                self._thread.join(timeout=0.1)
            self._thread = None
            self.on_stop.emit()
            logger.info(f"{self.__class__.__name__} stopped.")

    @abstractmethod
    def reset(self):
        """
        Abstract method to reset the tool.
        Resets internal counters, timers, and flags to their initial state.
        """
        self._is_running = False
        self._thread = None
        self._start_time = None
        self._pause_time = None
        self._elapsed_at_pause = 0
        self.on_reset.emit()
        logger.info(f"{self.__class__.__name__} reset.")

    @abstractmethod
    def get_status(self):
        """
        Abstract method to retrieve the current status of the tool.
        Should return a dictionary containing relevant state data (e.g., elapsed time, remaining time).
        """
        pass

    @abstractmethod
    def _run(self):
        """
        Abstract method containing the main execution loop.
        This method is intended to be run in a separate thread.
        """

        pass
