from datetime import datetime, timedelta

class AttendanceTracker:
    def __init__(self, db):
        self.db = db

    def record_attendance(self, event_type="check_in", notes=""):
        """Records an attendance event."""
        timestamp = datetime.now().isoformat()
        log_id = self.db.insert_attendance_log(timestamp, event_type, notes)
        return log_id

    def get_attendance_logs(self, date=None):
        """
        Retrieves attendance logs for a specific date or all logs if date is None.
        date: "YYYY-MM-DD" string
        """
        logs = self.db.get_all_attendance_logs()
        formatted_logs = []
        for log in logs:
            log_id, timestamp_str, event_type, notes = log
            log_date = datetime.fromisoformat(timestamp_str).strftime("%Y-%m-%d")
            if date is None or log_date == date:
                formatted_logs.append({
                    "id": log_id,
                    "timestamp": timestamp_str,
                    "event_type": event_type,
                    "notes": notes
                })
        return formatted_logs

    def get_total_time_allocated(self, date=None):
        """
        Calculates total time allocated based on check-in/check-out pairs for a given date.
        This is a simplified implementation.
        """
        logs = self.get_attendance_logs(date)
        
        check_ins = sorted([datetime.fromisoformat(log["timestamp"]) for log in logs if log["event_type"] == "check_in"])
        check_outs = sorted([datetime.fromisoformat(log["timestamp"]) for log in logs if log["event_type"] == "check_out"])

        total_allocated_seconds = 0
        
        # Simple pairing: assume first check-in pairs with first check-out, etc.
        # This needs more robust logic for complex scenarios (e.g., missing check-outs, multiple check-ins)
        i, j = 0, 0
        while i < len(check_ins) and j < len(check_outs):
            if check_outs[j] > check_ins[i]:
                total_allocated_seconds += (check_outs[j] - check_ins[i]).total_seconds()
                i += 1
                j += 1
            else: # Check-out happened before or at the same time as check-in, skip check-out
                j += 1
        
        hours = int(total_allocated_seconds // 3600)
        minutes = int((total_allocated_seconds % 3600) // 60)
        seconds = int(total_allocated_seconds % 60)

        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
