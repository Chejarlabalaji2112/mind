import sqlite3
import json

class TimeAgentDB:
    def __init__(self, db_path="time_manager.db"):
        self.db_path = db_path
        self.conn = None
        self._connect()

    def _connect(self):
        try:
            # Using check_same_thread=False to allow access from multiple threads
            self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        except sqlite3.Error as e:
            print(f"Database connection error: {e}")

    def close_connection(self):
        if self.conn:
            self.conn.close()

    def _execute_query(self, query, params=(), fetch_one=False, fetch_all=False):
        if not self.conn:
            print("Database not connected.")
            return None

        try:
            cursor = self.conn.cursor() # Create a new cursor for each operation
            cursor.execute(query, params)
            self.conn.commit()
            if fetch_one:
                return cursor.fetchone()
            if fetch_all:
                return cursor.fetchall()
            return cursor.lastrowid
        except sqlite3.Error as e:
            print(f"Database operation error: {e}")
            return None

    def create_tables(self):
        # Alarms Table
        self._execute_query("""
            CREATE TABLE IF NOT EXISTS alarms (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                hour INTEGER NOT NULL,
                minute INTEGER NOT NULL,
                days TEXT NOT NULL, -- JSON string of days, e.g., '["Mon", "Tue"]' or '["Everyday"]'
                label TEXT,
                is_active BOOLEAN NOT NULL
            );
        """)

        # Timers Table
        self._execute_query("""
            CREATE TABLE IF NOT EXISTS timers (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                duration_seconds INTEGER NOT NULL,
                start_time TEXT NOT NULL, -- ISO format datetime string
                end_time TEXT NOT NULL,   -- ISO format datetime string
                label TEXT,
                is_active BOOLEAN NOT NULL
            );
        """)

        # Stopwatches Table
        self._execute_query("""
            CREATE TABLE IF NOT EXISTS stopwatches (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                start_time TEXT NOT NULL, -- ISO format datetime string
                label TEXT,
                is_active BOOLEAN NOT NULL
            );
        """)

        # Pomodoro Sessions Table
        self._execute_query("""
            CREATE TABLE IF NOT EXISTS pomodoros (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                work_duration_minutes INTEGER NOT NULL,
                short_break_minutes INTEGER NOT NULL,
                long_break_minutes INTEGER NOT NULL,
                total_cycles INTEGER NOT NULL,
                current_cycle INTEGER NOT NULL,
                current_phase TEXT NOT NULL, -- 'work', 'short_break', 'long_break', 'completed'
                start_time TEXT NOT NULL,    -- ISO format datetime string of session start
                phase_end_time TEXT NOT NULL, -- ISO format datetime string of current phase end
                label TEXT,
                is_active BOOLEAN NOT NULL
            );
        """)

        # Attendance Logs Table
        self._execute_query("""
            CREATE TABLE IF NOT EXISTS attendance_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL, -- ISO format datetime string
                event_type TEXT NOT NULL, -- 'check_in', 'check_out', 'manual_log'
                notes TEXT
            );
        """)

    # --- Alarm Operations ---
    def insert_alarm(self, hour, minute, days_json, label, is_active):
        return self._execute_query(
            "INSERT INTO alarms (hour, minute, days, label, is_active) VALUES (?, ?, ?, ?, ?)",
            (hour, minute, days_json, label, is_active)
        )

    def get_all_alarms(self):
        return self._execute_query("SELECT id, hour, minute, days, label, is_active FROM alarms", fetch_all=True)

    def update_alarm_status(self, alarm_id, is_active):
        return self._execute_query("UPDATE alarms SET is_active = ? WHERE id = ?", (is_active, alarm_id))

    def delete_alarm(self, alarm_id):
        return self._execute_query("DELETE FROM alarms WHERE id = ?", (alarm_id,))

    # --- Timer Operations ---
    def insert_timer(self, duration_seconds, start_time, end_time, label, is_active):
        return self._execute_query(
            "INSERT INTO timers (duration_seconds, start_time, end_time, label, is_active) VALUES (?, ?, ?, ?, ?)",
            (duration_seconds, start_time, end_time, label, is_active)
        )

    def get_all_timers(self):
        return self._execute_query("SELECT id, duration_seconds, start_time, end_time, label, is_active FROM timers", fetch_all=True)

    def update_timer_status(self, timer_id, is_active):
        return self._execute_query("UPDATE timers SET is_active = ? WHERE id = ?", (is_active, timer_id))

    # --- Stopwatch Operations ---
    def insert_stopwatch(self, start_time, label, is_active):
        return self._execute_query(
            "INSERT INTO stopwatches (start_time, label, is_active) VALUES (?, ?, ?)",
            (start_time, label, is_active)
        )

    def get_all_stopwatches(self):
        return self._execute_query("SELECT id, start_time, label, is_active FROM stopwatches", fetch_all=True)

    def update_stopwatch_status(self, stopwatch_id, is_active):
        return self._execute_query("UPDATE stopwatches SET is_active = ? WHERE id = ?", (is_active, stopwatch_id))

    # --- Pomodoro Operations ---
    def insert_pomodoro(self, work_duration_minutes, short_break_minutes, long_break_minutes,
                         total_cycles, current_cycle, current_phase, start_time, phase_end_time, label, is_active):
        return self._execute_query(
            """INSERT INTO pomodoros (work_duration_minutes, short_break_minutes, long_break_minutes,
                                   total_cycles, current_cycle, current_phase, start_time, phase_end_time, label, is_active)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (work_duration_minutes, short_break_minutes, long_break_minutes,
             total_cycles, current_cycle, current_phase, start_time, phase_end_time, label, is_active)
        )

    def get_all_pomodoros(self):
        return self._execute_query(
            """SELECT id, work_duration_minutes, short_break_minutes, long_break_minutes,
                      total_cycles, current_cycle, current_phase, start_time, phase_end_time, label, is_active
               FROM pomodoros""", fetch_all=True
        )

    def update_pomodoro(self, pomo_id, current_cycle, current_phase, phase_end_time, is_active):
        return self._execute_query(
            """UPDATE pomodoros SET current_cycle = ?, current_phase = ?, phase_end_time = ?, is_active = ?
               WHERE id = ?""",
            (current_cycle, current_phase, phase_end_time, is_active, pomo_id)
        )

    def update_pomodoro_status(self, pomo_id, is_active):
        return self._execute_query("UPDATE pomodoros SET is_active = ? WHERE id = ?", (is_active, pomo_id))

    # --- Attendance Log Operations ---
    def insert_attendance_log(self, timestamp, event_type, notes):
        return self._execute_query(
            "INSERT INTO attendance_logs (timestamp, event_type, notes) VALUES (?, ?, ?)",
            (timestamp, event_type, notes)
        )

    def get_all_attendance_logs(self):
        return self._execute_query("SELECT id, timestamp, event_type, notes FROM attendance_logs ORDER BY timestamp", fetch_all=True)
