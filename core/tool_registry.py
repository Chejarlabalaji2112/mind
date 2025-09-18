# core/tool_registry.py
import asyncio
from ports.act_port import Presenter
from tools.time_tools.timer import Timer
from tools.tasks_skills_tools.skilltracker import SkillTracker

class ToolRegistry:
    """Orchestrates all tools. Only depends on ports (Presenter) for output."""

    def __init__(self, presenter: Presenter, output_tuner=None,loop=None):
        # TODO before adding new tool, ensure it apprrpriate.
        # what if presenter is not what we expected. this edge case has to be handled.
        # if output_handler is none then some way there should be a default way of handling output.
        # create a custom exception and handle the above cases
        self.presenter = presenter
        self.output_tuner = output_tuner
        self.loop = loop or asyncio.get_running_loop()

        # Tools
        self.timer = Timer(loop=self.loop)
        # self.skills = SkillTracker() no need for connection of db now

        # Hook timer events to presenter
        self.timer.on_tick.add_listener(self._timer_tick_handler)
        self.timer.on_finished.add_listener(self._timer_finished_handler)

    # --- Private Handlers ---
    def _screen_handler(self, tuned_output):
        self.loop.call_soon_threadsafe(
        lambda:asyncio.create_task(
            self.presenter.show(tuned_output)
        )) 

    def _timer_tick_handler(self, remaining_time, remaining_time_formatted):
        tuned_output = self.output_tuner("Timer", f"{remaining_time_formatted}")
        self._screen_handler(tuned_output)

    def _timer_finished_handler(self):
        tuned_output = self.output_tuner("Timer", "Finished!")
        self._screen_handler(tuned_output)

    # --- Public API for agent ---
    def home(self):
        self._screen_handler(self.output_tuner(home=True))
        return "Welcome Home!"

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
