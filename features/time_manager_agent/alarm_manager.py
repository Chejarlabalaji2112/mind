from datetime import datetime, timedelta
import json

class AlarmManager:
    def __init__(self, db):
        self.db = db
        self.active_alarms = {} # Store active alarm instances if needed for more complex control

    def set_alarm(self, time_str, days, label="Alarm"):
        """
        Sets a new alarm.
        time_str: "HH:MM" format
        days: Comma-separated string of day abbreviations (e.g., "Mon,Tue,Fri") or "Everyday" or "Once"
        """
        try:
            hour, minute = map(int, time_str.split(':'))
            if not (0 <= hour <= 23 and 0 <= minute <= 59):
                raise ValueError("Invalid time format. Use HH:MM.")

            days_list = [d.strip().capitalize() for d in days.split(',')] if days not in ["Everyday", "Once"] else [days]
            days_json = json.dumps(days_list)

            alarm_id = self.db.insert_alarm(hour, minute, days_json, label, True)
            return alarm_id
        except ValueError as e:
            print(f"Error setting alarm: {e}")
            return None

    def get_alarms(self):
        """Retrieves all stored alarms."""
        alarms = self.db.get_all_alarms()
        formatted_alarms = []
        for alarm in alarms:
            alarm_dict = {
                "id": alarm[0],
                "time": f"{alarm[1]:02d}:{alarm[2]:02d}",
                "days": json.loads(alarm[3]),
                "label": alarm[4],
                "is_active": bool(alarm[5])
            }
            formatted_alarms.append(alarm_dict)
        return formatted_alarms

    def toggle_alarm(self, alarm_id, is_active):
        """Activates or deactivates an existing alarm."""
        return self.db.update_alarm_status(alarm_id, is_active)

    def delete_alarm(self, alarm_id):
        """Deletes an alarm."""
        return self.db.delete_alarm(alarm_id)

    def check_and_trigger_alarms(self, current_datetime):
        """
        Checks if any active alarms should be triggered based on the current time.
        In a real application, this would trigger a notification/sound.
        """
        current_hour = current_datetime.hour
        current_minute = current_datetime.minute
        current_day_of_week = current_datetime.strftime("%a") # e.g., "Mon"

        active_alarms = [a for a in self.get_alarms() if a["is_active"]]

        for alarm in active_alarms:
            alarm_time_str = alarm["time"]
            alarm_hour, alarm_minute = map(int, alarm_time_str.split(':'))
            alarm_days = alarm["days"]

            if current_hour == alarm_hour and current_minute == alarm_minute:
                should_trigger = False
                if "Everyday" in alarm_days:
                    should_trigger = True
                elif "Once" in alarm_days and current_datetime.date() == self._get_alarm_set_date(alarm["id"]): # Needs a way to store/check set date for "Once"
                    should_trigger = True
                elif current_day_of_week in alarm_days:
                    should_trigger = True

                if should_trigger:
                    # Prevent multiple triggers within the same minute
                    if not self._is_alarm_triggered_recently(alarm["id"], current_datetime):
                        print(f"ALARM TRIGGERED! ID: {alarm['id']}, Label: {alarm['label']}, Time: {alarm_time_str}")
                        # In a real system, this would send a notification, play a sound, etc.
                        self._mark_alarm_triggered(alarm["id"], current_datetime)
                        if "Once" in alarm_days:
                            self.toggle_alarm(alarm["id"], False) # Deactivate "Once" alarms after triggering

    def _get_alarm_set_date(self, alarm_id):
        # Placeholder: In a real system, "Once" alarms would need their set date stored in DB
        # For now, just return today's date for testing purposes
        return datetime.now().date()

    def _is_alarm_triggered_recently(self, alarm_id, current_datetime):
        # Placeholder: In a real system, store last triggered time in DB or in-memory
        # For simplicity, assume an alarm is triggered if it was triggered in the last minute
        # This needs to be more robust for production
        if hasattr(self, '_last_triggered'):
            if alarm_id in self._last_triggered:
                last_trigger_time = self._last_triggered[alarm_id]
                if (current_datetime - last_trigger_time).total_seconds() < 60:
                    return True
        return False

    def _mark_alarm_triggered(self, alarm_id, current_datetime):
        if not hasattr(self, '_last_triggered'):
            self._last_triggered = {}
        self._last_triggered[alarm_id] = current_datetime
