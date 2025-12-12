import inspect
import asyncio
from mind.utils import setup_logger

logger = setup_logger(__name__)

class Event:
    def __init__(self, loop=None):
        self._listeners = []
        self.loop = loop  # asyncio event loop to schedule async listeners

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
                if inspect.iscoroutinefunction(listener):
                    if self.loop is None:
                        raise RuntimeError("Async listener requires loop to be set")
                    # schedule async listener on main loop thread-safely
                    self.loop.call_soon_threadsafe(
                        lambda l=listener: asyncio.create_task(l(*args, **kwargs))
                    )
                else:
                    listener(*args, **kwargs)
            except Exception as e:
                logger.error(f"Error in event listener: {e}")
