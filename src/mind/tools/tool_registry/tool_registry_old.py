# tools/tool_registry.py
import asyncio
from mind.ports.act_port import Presenter
from mind.tools import Timer, Stopwatch, Pomodoro
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
        self.stopwatch = Stopwatch(loop=self.loop)
        self.pomodoro = Pomodoro(loop=self.loop)
        # self.skills = SkillTracker() no need for connection of db now

        # Hook events
        self.timer.on_tick.add_listener(self._timer_tick_handler)
        self.timer.on_finished.add_listener(self._timer_finished_handler)
        self.stopwatch.on_tick.add_listener(self._stopwatch_tick_handler)  #TODO handle laps and other events   
        self.pomodoro.on_tick.add_listener(self._pomodoro_tick_handler)
        
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

    def _stopwatch_tick_handler(self, elapsed_time, elapsed_time_formatted):
        tuned_output = self.output_tuner("Stopwatch", f"{elapsed_time_formatted}")
        self._screen_handler(tuned_output)  

    def _pomodoro_tick_handler(self, remaining_time, phase, remaining_time_formatted):
        tuned_output = self.output_tuner("Pomodoro", f"{remaining_time_formatted}")
        self._screen_handler(tuned_output)

    # --- Public API for agent ---
    def home(self):
        self._screen_handler(self.output_tuner(home=True))
        return "Welcome Home!"
    
    def timer_tool(self,action: str, duration: int | None = None):
        if action == "start":
            return self.timer.start(duration or 0)
        elif action == "pause":
            return self.timer.pause()
        elif action == "resume":     
            return self.timer.resume()
        elif action == "stop":
            return self.timer.stop()
        elif action == "reset":
            return self.timer.reset()
        elif action == "status":
            return self.timer.get_status()
        else:
            return "Invalid action."
        
    def stopwatch_tool(self, action: str):
        if action == "start":
            return self.stopwatch.start()
        elif action == "pause":
            return self.stopwatch.pause()   
        elif action == "resume":
            return self.stopwatch.resume()
        elif action == "stop":
            return self.stopwatch.stop()
        elif action == "lap":
            return self.stopwatch.lap()
        elif action == "reset":
            return self.stopwatch.reset()
        elif action == "status":
            return self.stopwatch.get_status()
        else:
            return "Invalid action."
    
    def pomodoro_tool(self, action, pom_type: str = 'short'):
        if action == "start":
            return self.pomodoro.start(pom_type)
        elif action == "pause":
            return self.pomodoro.pause()
        elif action == "resume":
            return self.pomodoro.resume()
        elif action == "stop":
            return self.pomodoro.stop()
        elif action == "reset":
            return self.pomodoro.reset()
        elif action == "status":
            return self.pomodoro.get_status()
        else:
            return "Invalid action."
