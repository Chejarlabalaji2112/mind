import threading
import time
from datetime import datetime, timedelta

from .alarm_manager import AlarmManager
from .timer_manager import TimerManager
from .pomodoro_manager import PomodoroManager
from .attendance_tracker import AttendanceTracker
from .time_data_model import TimeDataModel
from .time_agent_db import TimeAgentDB

class TimeManagerAgent:
    def __init__(self, db_path="time_manager.db"):
        self.db = TimeAgentDB(db_path)
        self.db.create_tables()

        self.alarm_manager = AlarmManager(self.db)
        self.timer_manager = TimerManager(self.db)
        self.pomodoro_manager = PomodoroManager(self.db)
        self.attendance_tracker = AttendanceTracker(self.db)

        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_time_events)
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def _monitor_time_events(self):
        while self._running:
            now = datetime.now()
            self.alarm_manager.check_and_trigger_alarms(now)
            # Other time-based checks can go here (e.g., Pomodoro phase transitions)
            time.sleep(1) # Check every second

    def get_current_time(self):
        return datetime.now().strftime("%H:%M:%S")

    def get_current_date(self):
        return datetime.now().strftime("%Y-%m-%d")
    
    # TODO : Add timezone support. agent could accept a timezone parameter and convert times accordingly.

    def stop_agent(self):
        self._running = False
        self._monitor_thread.join()
        self.db.close_connection()

    # --- Alarm Management ---
    def set_alarm(self, time_str, days, label="Alarm"):
        return self.alarm_manager.set_alarm(time_str, days, label)

    def get_alarms(self):
        return self.alarm_manager.get_alarms()

    def toggle_alarm(self, alarm_id, is_active):
        return self.alarm_manager.toggle_alarm(alarm_id, is_active)

    def delete_alarm(self, alarm_id):
        return self.alarm_manager.delete_alarm(alarm_id)

    # --- Timer Management ---
    def start_timer(self, duration_seconds, label="Timer"):
        return self.timer_manager.start_timer(duration_seconds, label)

    def get_active_timers(self):
        return self.timer_manager.get_active_timers()

    def stop_timer(self, timer_id):
        return self.timer_manager.stop_timer(timer_id)

    # --- Stopwatch Management ---
    def start_stopwatch(self, label="Stopwatch"):
        return self.timer_manager.start_stopwatch(label)

    def stop_stopwatch(self, stopwatch_id):
        return self.timer_manager.stop_stopwatch(stopwatch_id)

    def get_active_stopwatches(self):
        return self.timer_manager.get_active_stopwatches()

    # --- Pomodoro Management ---
    def start_pomodoro(self, work_duration=25, short_break=5, long_break=15, cycles=4):
        return self.pomodoro_manager.start_pomodoro(work_duration, short_break, long_break, cycles)

    def get_active_pomodoros(self):
        return self.pomodoro_manager.get_active_pomodoros()

    def stop_pomodoro(self, pomodoro_id):
        return self.pomodoro_manager.stop_pomodoro(pomodoro_id)

    # --- Attendance Tracking ---
    def record_attendance(self, event_type="check_in", notes=""):
        return self.attendance_tracker.record_attendance(event_type, notes)

    def get_attendance_logs(self, date=None):
        return self.attendance_tracker.get_attendance_logs(date)

    def get_total_time_allocated(self, date=None):
        # This would likely involve more complex logic, potentially integrating with other agents
        # For now, a placeholder or simple calculation based on attendance/pomodoro
        return self.attendance_tracker.get_total_time_allocated(date)

# Example Usage (for testing purposes)
if __name__ == "__main__":
    agent = TimeManagerAgent()
    print(f"Current Time: {agent.get_current_time()}")
    print(f"Current Date: {agent.get_current_date()}")

    # Example Alarm
    print("\n--- Alarms ---")
    alarm_id = agent.set_alarm("20:00", "Mon,Tue,Wed", "Evening Reminder")
    print(f"Set alarm with ID: {alarm_id}")
    print("Alarms:", agent.get_alarms())

    # Example Timer
    print("\n--- Timers ---")
    timer_id = agent.start_timer(5, "Quick Test Timer")
    print(f"Started timer with ID: {timer_id}")
    print("Active Timers:", agent.get_active_timers())
    time.sleep(6) # Wait for timer to finish
    print("Active Timers after wait:", agent.get_active_timers())

    # Example Stopwatch
    print("\n--- Stopwatches ---")
    sw_id = agent.start_stopwatch("Workout")
    print(f"Started stopwatch with ID: {sw_id}")
    print("Active Stopwatches:", agent.get_active_stopwatches())
    time.sleep(3)
    agent.stop_stopwatch(sw_id)
    print("Active Stopwatches after stop:", agent.get_active_stopwatches())

    # Example Pomodoro
    print("\n--- Pomodoro ---")
    pomo_id = agent.start_pomodoro(work_duration=1, short_break=0.5, cycles=1) # Short cycle for testing
    print(f"Started Pomodoro with ID: {pomo_id}")
    print("Active Pomodoros:", agent.get_active_pomodoros())
    time.sleep(1.1 * 60) # Wait for work phase
    print("Active Pomodoros after work phase:", agent.get_active_pomodoros())
    time.sleep(0.6 * 60) # Wait for short break
    print("Active Pomodoros after break phase:", agent.get_active_pomodoros())

    # Example Attendance
    print("\n--- Attendance ---")
    agent.record_attendance("check_in", "Morning start")
    time.sleep(1)
    agent.record_attendance("check_out", "Lunch break")
    print("Attendance Logs:", agent.get_attendance_logs())

    agent.stop_agent()
    print("\nAgent stopped.")
