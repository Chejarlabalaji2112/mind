"""
This module is not the final implementation but a prototype for and alarm management system.
Alarm management is a complex task in my opnion. this can be improved in many ways.
"""





import time
import threading
import datetime
import sqlite3
import os

from time_tools.base_tool import Event
from utils.time_conversions import format_seconds_to_hms # Assuming this is useful, will add more if needed

class AlarmManager:
    def __init__(self, db_path="memory/memory.db"):
        self._db_path = db_path
        self._monitor_thread = None
        self._running = False
        self.on_alarm_triggered = Event()

        self._init_db()
        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_alarms_thread, daemon=True)
        self._monitor_thread.start()
        print("AlarmManager initialized and monitoring thread started.")

    def _init_db(self):
        """Initializes the SQLite database and creates the alarms table if it doesn't exist."""
        with sqlite3.connect(self._db_path) as conn:
            cursor = conn.cursor()
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS alarms (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    time_str TEXT NOT NULL,
                    label TEXT NOT NULL,
                    status TEXT NOT NULL,
                    repeat TEXT NOT NULL,
                    next_trigger_timestamp REAL
                )
            """)
            conn.commit()

    def _execute_query(self, query, params=(), fetch_one=False):
        """Helper method to execute SQL queries."""
        with sqlite3.connect(self._db_path) as conn:
            conn.row_factory = sqlite3.Row # This makes rows behave like dictionaries
            cursor = conn.cursor()
            cursor.execute(query, params)
            conn.commit()
            if fetch_one:
                return cursor.fetchone()
            return cursor.fetchall()

    def _calculate_next_trigger_timestamp(self, time_str, repeat, reference_time=None):
        """
        Calculates the next future Unix timestamp for an alarm.
        time_str: "HH:MM"
        repeat: "daily", "weekdays", "once", "Mon,Wed,Fri"
        reference_time: datetime object to calculate from (defaults to current UTC time)
        """
        if reference_time is None:
            reference_time = datetime.datetime.now(datetime.timezone.utc) # Use UTC for consistency

        try:
            alarm_hour, alarm_minute = map(int, time_str.split(':'))
            if not (0 <= alarm_hour < 24 and 0 <= alarm_minute < 60):
                raise ValueError("Invalid time_str format. Must be HH:MM.")
        except ValueError:
            raise ValueError("Invalid time_str format. Must be HH:MM.")

        # Create a datetime object for today at the alarm time
        today_alarm_time = reference_time.replace(
            hour=alarm_hour,
            minute=alarm_minute,
            second=0,
            microsecond=0
        )

        if repeat == "once":
            # If alarm time is in the past, set for next day, otherwise today
            if today_alarm_time <= reference_time:
                return (today_alarm_time + datetime.timedelta(days=1)).timestamp()
            return today_alarm_time.timestamp()
        elif repeat == "daily":
            if today_alarm_time <= reference_time:
                return (today_alarm_time + datetime.timedelta(days=1)).timestamp()
            return today_alarm_time.timestamp()
        elif repeat == "weekdays":
            current_day_of_week = reference_time.weekday() # Monday is 0, Sunday is 6
            target_day_of_week = -1 # Placeholder

            # Find the next weekday (Mon-Fri)
            for i in range(7):
                check_date = reference_time + datetime.timedelta(days=i)
                if 0 <= check_date.weekday() <= 4: # Is a weekday
                    potential_alarm_time = check_date.replace(
                        hour=alarm_hour,
                        minute=alarm_minute,
                        second=0,
                        microsecond=0
                    )
                    if potential_alarm_time > reference_time:
                        return potential_alarm_time.timestamp()
            # Should not happen if there's always a future weekday
            return (reference_time + datetime.timedelta(days=1)).timestamp() # Fallback
        elif ',' in repeat: # Specific days like "Mon,Wed,Fri"
            days_map = {
                "Mon": 0, "Tue": 1, "Wed": 2, "Thu": 3,
                "Fri": 4, "Sat": 5, "Sun": 6
            }
            target_days = [days_map[d.strip()] for d in repeat.split(',') if d.strip() in days_map]
            if not target_days:
                raise ValueError(f"Invalid repeat pattern: {repeat}")

            for i in range(7): # Check up to 7 days in the future
                check_date = reference_time + datetime.timedelta(days=i)
                if check_date.weekday() in target_days:
                    potential_alarm_time = check_date.replace(
                        hour=alarm_hour,
                        minute=alarm_minute,
                        second=0,
                        microsecond=0
                    )
                    if potential_alarm_time > reference_time:
                        return potential_alarm_time.timestamp()
            # Fallback if no future day found within a week (should not happen for valid patterns)
            return (reference_time + datetime.timedelta(days=1)).timestamp()
        else:
            raise ValueError(f"Unsupported repeat pattern: {repeat}")

    def _monitor_alarms_thread(self):
        """Background thread to continuously monitor and trigger alarms."""
        while self._running:
            current_timestamp = time.time()
            active_alarms = self._execute_query("SELECT * FROM alarms WHERE status = 'active'")

            if active_alarms:
                # Sort alarms by their next_trigger_timestamp
                sorted_alarms = sorted(active_alarms, key=lambda x: x['next_trigger_timestamp'])

                earliest_alarm = sorted_alarms[0]

                if earliest_alarm['next_trigger_timestamp'] <= current_timestamp:
                    self._trigger_alarm(earliest_alarm)

            time.sleep(1) # Check every second

    def _trigger_alarm(self, alarm_data):
        """Handles the triggering of an alarm."""
        print(f"Alarm Triggered: {alarm_data['label']} at {datetime.datetime.fromtimestamp(alarm_data['next_trigger_timestamp'])}")
        self.on_alarm_triggered.emit(alarm_id=alarm_data['id'], label=alarm_data['label'])

        if alarm_data['repeat'] == "once":
            # For one-time alarms, set status to inactive
            self._execute_query("UPDATE alarms SET status = 'inactive' WHERE id = ?", (alarm_data['id'],))
        else:
            # For repeating alarms, calculate next trigger time and update
            next_ts = self._calculate_next_trigger_timestamp(
                alarm_data['time_str'],
                alarm_data['repeat'],
                reference_time=datetime.datetime.fromtimestamp(alarm_data['next_trigger_timestamp'], tz=datetime.timezone.utc) # Calculate from the *triggered* time
            )
            self._execute_query("UPDATE alarms SET next_trigger_timestamp = ? WHERE id = ?", (next_ts, alarm_data['id']))

    def create_alarm(self, time_str, label, repeat, status="active"):
        """
        Creates a new alarm.
        time_str: "HH:MM"
        label: A descriptive label for the alarm
        repeat: "daily", "weekdays", "once", "Mon,Wed,Fri"
        status: "active" or "inactive"
        """
        next_trigger_timestamp = self._calculate_next_trigger_timestamp(time_str, repeat)
        query = "INSERT INTO alarms (time_str, label, status, repeat, next_trigger_timestamp) VALUES (?, ?, ?, ?, ?)"
        self._execute_query(query, (time_str, label, status, repeat, next_trigger_timestamp))
        
        # Retrieve the newly created alarm's ID
        last_row_id = self._execute_query("SELECT last_insert_rowid()", fetch_one=True)[0]
        print(f"Alarm '{label}' created with ID: {last_row_id}")
        return last_row_id

    def get_alarms(self, status_filter=None):
        """
        Retrieves alarms based on status filter.
        status_filter: None (all), "active", "inactive"
        Returns a list of alarm dictionaries.
        """
        if status_filter in ["active", "inactive"]:
            query = "SELECT * FROM alarms WHERE status = ?"
            alarms = self._execute_query(query, (status_filter,))
        else:
            query = "SELECT * FROM alarms"
            alarms = self._execute_query(query)
        
        # Convert sqlite3.Row objects to dictionaries for easier handling
        return [dict(alarm) for alarm in alarms]

    def delete_alarm(self, alarm_id):
        """Deletes an alarm by its ID."""
        self._execute_query("DELETE FROM alarms WHERE id = ?", (alarm_id,))
        print(f"Alarm ID {alarm_id} deleted.")

    def toggle_alarm(self, alarm_id):
        """Toggles the status of an alarm (active/inactive)."""
        alarm = self._execute_query("SELECT * FROM alarms WHERE id = ?", (alarm_id,), fetch_one=True)
        if alarm:
            new_status = "inactive" if alarm['status'] == "active" else "active"
            
            # If toggling to active, recalculate next_trigger_timestamp to ensure it's in the future
            if new_status == "active":
                next_ts = self._calculate_next_trigger_timestamp(alarm['time_str'], alarm['repeat'])
                self._execute_query("UPDATE alarms SET status = ?, next_trigger_timestamp = ? WHERE id = ?", (new_status, next_ts, alarm_id))
            else:
                self._execute_query("UPDATE alarms SET status = ? WHERE id = ?", (new_status, alarm_id))
            print(f"Alarm ID {alarm_id} toggled to {new_status}.")
            return True
        print(f"Alarm ID {alarm_id} not found.")
        return False

    def stop(self):
        """Stops the alarm monitoring thread."""
        self._running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2) # Give it a moment to finish
            print("AlarmManager monitoring thread stopped.")

# Example Usage (for testing purposes, can be removed later)
if __name__ == "__main__":
    alarm_manager = AlarmManager()

    # Clear existing alarms for clean testing
    alarm_manager._execute_query("DELETE FROM alarms")

    # Create some alarms
    print("\nCreating alarms:")
    alarm_manager.create_alarm("11:30", "Lunch Break", "daily")
    alarm_manager.create_alarm("11:31", "Quick Check", "once")
    alarm_manager.create_alarm("11:32", "Meeting Prep", "weekdays")
    alarm_manager.create_alarm("11:33", "Weekend Task", "Sat,Sun")

    print("\nAll alarms:")
    for alarm in alarm_manager.get_alarms():
        print(alarm)

    # Simulate waiting for alarms to trigger
    print("\nWaiting for alarms to trigger (will run for 10 seconds)...")
    time.sleep(10)

    print("\nAlarms after some time:")
    for alarm in alarm_manager.get_alarms():
        print(alarm)

    # Toggle an alarm
    print("\nToggling alarm ID 1 to inactive:")
    alarm_manager.toggle_alarm(1)
    print("\nAlarms after toggle:")
    for alarm in alarm_manager.get_alarms():
        print(alarm)

    # Toggle it back to active
    print("\nToggling alarm ID 1 back to active:")
    alarm_manager.toggle_alarm(1)
    print("\nAlarms after re-toggle:")
    for alarm in alarm_manager.get_alarms():
        print(alarm)

    # Delete an alarm
    print("\nDeleting alarm ID 2:")
    alarm_manager.delete_alarm(2)
    print("\nAlarms after deletion:")
    for alarm in alarm_manager.get_alarms():
        print(alarm)

    alarm_manager.stop()
