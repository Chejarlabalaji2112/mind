import threading
import time
from abc import ABC, abstractmethod

class Event:
    def __init__(self):
        self._listeners = []

    def add_listener(self, listener):
        if callable(listener):
            self._listeners.append(listener)
        else:
            raise ValueError("Listener must be a callable function.")

    def remove_listener(self, listener):
        if listener in self._listeners:
            self._listeners.remove(listener)

    def emit(self, *args, **kwargs):
        for listener in self._listeners:
            try:
                listener(*args, **kwargs)
            except Exception as e:
                print(f"Error in event listener: {e}")

class TimeTool(ABC):
    def __init__(self):
        self._is_running = False
        self._thread = None
        self._start_time = None
        self._pause_time = None
        self._elapsed_at_pause = 0

        self.on_tick = Event()
        self.on_start = Event()
        self.on_pause = Event()
        self.on_resume = Event()
        self.on_stop = Event()
        self.on_reset = Event()

    @property
    def is_running(self):
        return self._is_running

    @abstractmethod
    def start(self, *args, **kwargs):
        pass

    def pause(self):
        if self._is_running:
            self._is_running = False
            self._pause_time = time.time()
            if self._start_time is not None:
                self._elapsed_at_pause += (self._pause_time - self._start_time)
            self.on_pause.emit()
            print(f"{self.__class__.__name__} paused.")

    def resume(self):
        if not self._is_running and self._start_time is not None:
            self._is_running = True
            self._start_time = time.time() # Reset start time to calculate new elapsed duration
            self._thread = threading.Thread(target=self._run)
            self._thread.daemon = True
            self._thread.start()
            self.on_resume.emit()
            print(f"{self.__class__.__name__} resumed.")

    def stop(self):
        if self._is_running:
            self._is_running = False
            self._thread.join(timeout=0.1) # Give a small timeout for the thread to finish
            self._thread = None
            self.on_stop.emit()
            print(f"{self.__class__.__name__} stopped.")

    @abstractmethod
    def reset(self):
        self._is_running = False
        self._thread = None
        self._start_time = None
        self._pause_time = None
        self._elapsed_at_pause = 0
        self.on_reset.emit()
        print(f"{self.__class__.__name__} reset.")

    @abstractmethod
    def get_status(self):
        pass

    @abstractmethod
    def _run(self):
        pass
