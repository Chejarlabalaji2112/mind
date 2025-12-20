from mind.core.ports.act_port import Presenter

class ToolsHelpers:
    def __init__(self, presenters: list):
        """
        Initializes the helper with a list of presenters.
        Assumes presenters have .prepare_input() and .show() methods.
        """
        self.presenters = presenters

    def _broadcast(self, title, content, bottom=""):
        """Helper method to update all presenters to avoid code repetition."""
        for presenter in self.presenters:
            prepared_input = presenter.prepare_input(title=title, content=content, botttom=bottom)
            presenter.show(prepared_input)

    # ==========================================
    # TIMER HANDLERS
    # ==========================================

    def timer_on_start_handler(self, duration):
        self._broadcast(
            title="Timer Started", 
            content=f"Countdown started: {duration}s",
            bottom="Running..."
        )

    def timer_on_tick_handler(self, remaining_time, remaining_time_formatted):
        self._broadcast(
            title="Timer", 
            content=remaining_time_formatted, 
            bottom="Counting down..."
        )

    def timer_on_pause_handler(self):
        self._broadcast(
            title="Timer Paused", 
            content="|| Paused", 
            bottom="Waiting to resume"
        )

    def timer_on_resume_handler(self):
        self._broadcast(
            title="Timer Resumed", 
            content="Resuming...", 
            bottom="Counting down..."
        )

    def timer_on_stop_handler(self):
        self._broadcast(
            title="Timer Stopped", 
            content="Stopped by user", 
            bottom=""
        )

    def timer_on_reset_handler(self):
        self._broadcast(
            title="Timer Reset", 
            content="00:00:00", 
            bottom="Ready"
        )

    def timer_on_finished_handler(self):
        self._broadcast(
            title="Timer Finished", 
            content="00:00:00", 
            bottom="Time is up!"
        )

    # ==========================================
    # STOPWATCH HANDLERS
    # ==========================================

    def stopwatch_on_start_handler(self):
        self._broadcast(
            title="Stopwatch Started", 
            content="00:00:00", 
            bottom="Recording..."
        )

    def stopwatch_on_tick_handler(self, elapsed_time, elapsed_time_formatted):
        self._broadcast(
            title="Stopwatch", 
            content=elapsed_time_formatted, 
            bottom="Running..."
        )

    def stopwatch_on_pause_handler(self):
        self._broadcast(
            title="Stopwatch Paused", 
            content="|| Paused", 
            bottom=""
        )

    def stopwatch_on_resume_handler(self):
        self._broadcast(
            title="Stopwatch Resumed", 
            content="Resuming...", 
            bottom="Recording..."
        )

    def stopwatch_on_stop_handler(self):
        self._broadcast(
            title="Stopwatch Stopped", 
            content="Stopped", 
            bottom=""
        )

    def stopwatch_on_reset_handler(self, elapsed_time, elapsed_time_formatted):
        self._broadcast(
            title="Stopwatch Reset", 
            content=elapsed_time_formatted, 
            bottom="Ready"
        )

    def stopwatch_on_lap_handler(self, lap_time, lap_time_formatted, all_laps):
        # Shows the specific lap time in content, and total laps in bottom
        self._broadcast(
            title=f"Lap {len(all_laps)} Recorded", 
            content=lap_time_formatted, 
            bottom=f"Total Laps: {len(all_laps)}"
        )

    # ==========================================
    # POMODORO HANDLERS
    # ==========================================

    def pomodoro_on_tick_handler(self, remaining_time, phase, remaining_time_formatted):
        # Formats the title to look like "Pomodoro - WORK"
        display_phase = phase.replace("_", " ").title()
        self._broadcast(
            title=f"Pomodoro - {display_phase}", 
            content=remaining_time_formatted, 
            bottom="Focus..." if phase == "WORK" else "Relax..."
        )

    def pomodoro_on_work_start_handler(self, cycle):
        self._broadcast(
            title="Pomodoro: Work", 
            content="Focus Time Started", 
            bottom=f"Cycle: {cycle}"
        )

    def pomodoro_on_short_break_start_handler(self, cycle):
        self._broadcast(
            title="Pomodoro: Short Break", 
            content="Take a quick break", 
            bottom=f"Cycle: {cycle}"
        )

    def pomodoro_on_long_break_start_handler(self, cycle):
        self._broadcast(
            title="Pomodoro: Long Break", 
            content="Take a long rest", 
            bottom=f"Cycle: {cycle}"
        )

    def pomodoro_on_phase_end_handler(self, previous_phase, current_cycle):
        display_phase = previous_phase.replace("_", " ").title()
        self._broadcast(
            title=f"{display_phase} Ended", 
            content="Transitioning...", 
            bottom=f"Completed Cycle {current_cycle}"
        )

    def pomodoro_on_cycle_complete_handler(self, cycle):
        self._broadcast(
            title="Cycle Complete", 
            content="Work session finished", 
            bottom=f"Total Cycles: {cycle}"
        )

    def pomodoro_on_pause_handler(self):
        self._broadcast(
            title="Pomodoro Paused", 
            content="|| Paused", 
            bottom=""
        )

    def pomodoro_on_resume_handler(self):
        self._broadcast(
            title="Pomodoro Resumed", 
            content="Resuming...", 
            bottom=""
        )

    def pomodoro_on_stop_handler(self):
        self._broadcast(
            title="Pomodoro Stopped", 
            content="Session Ended", 
            bottom="Idle"
        )

    def pomodoro_on_reset_handler(self):
        self._broadcast(
            title="Pomodoro Reset", 
            content="00:00:00", 
            bottom="Ready"
        )

    