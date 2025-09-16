# core/tool_registry.py
import asyncio
from ports.act_port import Presenter
from tools.time_tools.timer import Timer
from tools.tasks_skills_tools.skilltracker import SkillTracker

class ToolRegistry:
    """Orchestrates all tools. Only depends on ports (Presenter) for output."""

    def __init__(self, presenter: Presenter, loop=None):
        self.presenter = presenter
        self.loop = loop or asyncio.get_running_loop()

        # Tools
        self.timer = Timer(loop=self.loop)
        # self.skills = SkillTracker() no need for connection of db now

        # Hook timer events to presenter
        self.timer.on_tick.add_listener(self._timer_tick_handler)
        self.timer.on_finished.add_listener(self._timer_finished_handler)

    # --- Private Handlers ---
    def _screen_handler(self, title, content, bottom=""):
        self.loop.call_soon_threadsafe(
        lambda:asyncio.create_task(
            self.presenter.show({"mode": "text",
                                 "clear": True,
                                  "items": [
                                      {"text":title, "x": 0, "y": 0, "size": 1} ,
                                      {"text":content, "x": 10, "y": 20, "size": 2},
                                      {"text":bottom, "x": 0, "y": 54, "size": 1}
                                  ]
                                  })
        ))

    def _timer_tick_handler(self, remaining_time, remaining_time_formatted):
        self._screen_handler("Timer", f"{remaining_time_formatted}")

    def _timer_finished_handler(self):
        self._screen_handler("Timer", "Finished!")

    # --- Public API for agent ---
    def set_timer(self, seconds: int):
        self.timer.start(seconds)
        return f"Timer started for {seconds} seconds."

    def pause_timer(self):
        self.timer.pause()
        return "Timer paused."

    def resume_timer(self):
        self.timer.resume()
        return "Timer resumed."

    def stop_timer(self):
        self.timer.stop()
        return "Timer stopped."
