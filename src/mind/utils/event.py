import asyncio
import inspect
from mind.utils import setup_logger

logger = setup_logger(__name__)

class Event:
    def __init__(self, loop: asyncio.AbstractEventLoop | None = None):
        self._listeners = []
        self.loop = loop

    def add_listener(self, listener):
        if not callable(listener):
            raise ValueError("Listener must be callable")
        self._listeners.append(listener)

    def remove_listener(self, listener):
        if listener in self._listeners:
            self._listeners.remove(listener)

    def emit(self, *args, **kwargs):
        for listener in list(self._listeners):
            try:
                result = listener(*args, **kwargs)

                # If listener returned a coroutine, schedule it
                if inspect.iscoroutine(result):
                    if not self.loop:
                        raise RuntimeError("Async listener requires event loop")

                    self.loop.call_soon_threadsafe(
                        asyncio.create_task,
                        self._safe_task(result)
                    )

            except Exception as e:
                logger.exception("Error in event listener")

    async def _safe_task(self, coro):
        try:
            await coro
        except Exception:
            logger.exception("Unhandled exception in async event listener")
