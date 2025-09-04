from datetime import datetime, timedelta
import time

# TODO I do not understand why we are saving the timer etc in permament storage. it should be in ram only.
# TODO Add pause/resume functionality to timers and stopwatches.
# TODO Add notifications when timers/alarms go off. 

"""The whole system is saving the timer and details in the disk and retrieving again and again. This is not efficient.
Usually the timer and stopwatch should run directly (may be as a micro-service.)"""

class TimerManager:
    def __init__(self, db):
        self.db = db
        self.active_timers = {} # {timer_id: {"start_time": datetime, "duration": int, "label": str}}
        self.active_stopwatches = {} # {stopwatch_id: {"start_time": datetime, "label": str}}

    # --- Timer Functions ---
    def start_timer(self, duration_seconds, label="Timer"):
        """Starts a new countdown timer."""
        if duration_seconds <= 0:
            print("Error: Timer duration must be positive.")
            return None

        start_time = datetime.now()
        end_time = start_time + timedelta(seconds=duration_seconds)
        timer_id = self.db.insert_timer(duration_seconds, start_time.isoformat(), end_time.isoformat(), label, True)
        if timer_id:
            self.active_timers[timer_id] = {
                "start_time": start_time,
                "duration": duration_seconds,
                "label": label,
                "end_time": end_time
            }
        return timer_id

    def get_active_timers(self):
        """Retrieves all active timers with their remaining time."""
        db_timers = self.db.get_all_timers()
        formatted_timers = []
        for timer in db_timers:
            timer_id, duration, start_time_str, end_time_str, label, is_active = timer
            if is_active:
                start_time = datetime.fromisoformat(start_time_str)
                end_time = datetime.fromisoformat(end_time_str)
                remaining_seconds = (end_time - datetime.now()).total_seconds()
                if remaining_seconds > 0:
                    formatted_timers.append({
                        "id": timer_id,
                        "label": label,
                        "duration_seconds": duration,
                        "remaining_seconds": int(remaining_seconds),
                        "start_time": start_time_str,
                        "end_time": end_time_str
                    })
                else:
                    # Timer has finished, deactivate it in DB
                    self.db.update_timer_status(timer_id, False)
        return formatted_timers

    def stop_timer(self, timer_id):
        """Stops and deactivates a specific timer."""
        if self.db.update_timer_status(timer_id, False):
            if timer_id in self.active_timers:
                del self.active_timers[timer_id]
            return True
        return False

    # --- Stopwatch Functions ---
    def start_stopwatch(self, label="Stopwatch"):
        """Starts a new stopwatch."""
        start_time = datetime.now()
        stopwatch_id = self.db.insert_stopwatch(start_time.isoformat(), label, True)
        if stopwatch_id:
            self.active_stopwatches[stopwatch_id] = {
                "start_time": start_time,
                "label": label
            }
        return stopwatch_id

    def get_active_stopwatches(self):
        """Retrieves all active stopwatches with their elapsed time."""
        db_stopwatches = self.db.get_all_stopwatches()
        formatted_stopwatches = []
        for sw in db_stopwatches:
            sw_id, start_time_str, label, is_active = sw
            if is_active:
                start_time = datetime.fromisoformat(start_time_str)
                elapsed_seconds = (datetime.now() - start_time).total_seconds()
                formatted_stopwatches.append({
                    "id": sw_id,
                    "label": label,
                    "start_time": start_time_str,
                    "elapsed_seconds": int(elapsed_seconds)
                })
        return formatted_stopwatches

    def stop_stopwatch(self, stopwatch_id):
        """Stops and deactivates a specific stopwatch."""
        if self.db.update_stopwatch_status(stopwatch_id, False):
            if stopwatch_id in self.active_stopwatches:
                del self.active_stopwatches[stopwatch_id]
            return True
        return False
