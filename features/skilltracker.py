import sqlite3
from datetime import datetime
from .. import custom_exception as ce

# TODO commits are there for each method we might need to think of those.
class SkillTracker:
    def __init__(self, db_path="../database/mind.db"):
        self.conn = sqlite3.connect(db_path)
        self.cur = self.conn.cursor()
        self.cur.execute("PRAGMA foreign_keys = ON;")
        self._active_sessions = {} # {SkillID: (SessionID, StartTime_datetime_object)}

        # Ensure tables exist
        self.cur.execute("""
        CREATE TABLE IF NOT EXISTS Skills(
            SkillID INTEGER PRIMARY KEY,
            Name TEXT NOT NULL UNIQUE,
            CreatedOn TEXT,
            TotalTimeSpent INTEGER
        );""")
        self.cur.execute("""
        CREATE TABLE IF NOT EXISTS Sessions(
            SessionID INTEGER PRIMARY KEY,
            SkillID INTEGER,
            StartTime TEXT,
            EndTime TEXT,
            Duration INTEGER,
            Notes TEXT,
            FOREIGN KEY (SkillID) REFERENCES Skills(SkillID) ON DELETE CASCADE
        );""")
        self.conn.commit()

    def __del__(self):
        self.conn.close()

    def create_skill(self, skill_name: str) -> bool:
        """Take the name of the skill from the user and inserts in the database."""
        try:
            self.cur.execute("SELECT 1 FROM Skills WHERE Name = ?", (skill_name,))
            exists = self.cur.fetchone()
            if exists:
                raise ce.SkillAlreadyExistsError(f"Skill with name '{skill_name}' already exists.")

            self.cur.execute("""
            INSERT INTO Skills (Name, CreatedOn, TotalTimeSpent) VALUES (?, DATE('now'), 0)""", (skill_name,))
            self.conn.commit()
            return True
        except ce.SkillAlreadyExistsError as sae:
            print(sae)
            print("Please choose a different skill name.")
            return False
        except sqlite3.Error as e:
            print(f"Database error: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return False

    def start_session(self, skill_name: str) -> bool:
        """This function starts a session and returns true if it is successful."""
        try:
            self.cur.execute("SELECT SkillID FROM Skills WHERE Name = ?", (skill_name,))
            skill_id_result = self.cur.fetchone()
            if not skill_id_result:
                raise ce.SkillNotFoundError(f"Skill '{skill_name}' not found.")
            skill_id = skill_id_result[0]

            # Check for existing active session for this skill
            if skill_id in self._active_sessions:
                print(f"Warning: A session for skill '{skill_name}' is already active in this application instance.")
                # Depending on desired behavior, you might raise an error or return False here.
                # For now, we'll allow starting a new one if the database doesn't show it.

            self.cur.execute("SELECT SessionID FROM Sessions WHERE SkillID = ? AND EndTime IS NULL", (skill_id,))
            active_db_session = self.cur.fetchone()
            if active_db_session:
                print(f"Warning: An active session for skill '{skill_name}' already exists in the database.")
                # Again, depending on desired behavior, you might raise an error or return False.
                return False # Prevent starting a new session if one is already active in DB

            start_time_dt = datetime.now()
            start_time_str = start_time_dt.strftime("%Y-%m-%d %H:%M:%S")

            self.cur.execute("""
            INSERT INTO Sessions (SkillID, StartTime, EndTime, Duration, Notes) VALUES (?, ?, NULL, NULL, NULL)""",
            (skill_id, start_time_str))
            self.conn.commit()

            session_id = self.cur.lastrowid
            self._active_sessions[skill_id] = (session_id, start_time_dt)
            print(f"Session started for '{skill_name}' (SessionID: {session_id}) at {start_time_str}")
            return True
        except ce.SkillNotFoundError as snf:
            print(snf)
            return False
        except sqlite3.Error as e:
            print(f"Database error: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return False

    def end_session(self, skill_name: str) -> bool:
        """This function ends a session and saves it in the table and returns true if it is successful."""
        try:
            self.cur.execute("SELECT SkillID FROM Skills WHERE Name = ?", (skill_name,))
            skill_id_result = self.cur.fetchone()
            if not skill_id_result:
                raise ce.SkillNotFoundError(f"Skill '{skill_name}' not found.")
            skill_id = skill_id_result[0]

            session_id = None
            start_time_dt = None

            # Try to get from in-memory active sessions first
            if skill_id in self._active_sessions:
                session_id, start_time_dt = self._active_sessions[skill_id]
                del self._active_sessions[skill_id] # Remove from cache once ending
            else:
                # If not in cache, check the database for an active session
                self.cur.execute("SELECT SessionID, StartTime FROM Sessions WHERE SkillID = ? AND EndTime IS NULL ORDER BY StartTime DESC LIMIT 1", (skill_id,))
                active_db_session = self.cur.fetchone()
                if active_db_session:
                    session_id = active_db_session[0]
                    start_time_str_db = active_db_session[1]
                    start_time_dt = datetime.strptime(start_time_str_db, "%Y-%m-%d %H:%M:%S")

            if not session_id or not start_time_dt:
                raise ce.NoActiveSessionError(f"No active session found for skill '{skill_name}'.")

            end_time_dt = datetime.now()
            end_time_str = end_time_dt.strftime("%Y-%m-%d %H:%M:%S")

            time_difference = end_time_dt - start_time_dt
            duration_minutes = int(time_difference.total_seconds() / 60)

            self.cur.execute("""
                UPDATE Sessions
                SET EndTime = ?, Duration = ?
                WHERE SessionID = ?
            """, (end_time_str, duration_minutes, session_id))

            self.cur.execute("""
                UPDATE Skills
                SET TotalTimeSpent = TotalTimeSpent + ?
                WHERE SkillID = ?
            """, (duration_minutes, skill_id))
            self.conn.commit()
            print(f"Session ended for '{skill_name}' (SessionID: {session_id}). Duration: {duration_minutes} minutes.")
            return True
        except ce.SkillNotFoundError as snf:
            print(snf)
            return False
        except ce.NoActiveSessionError as nase:
            print(nase)
            return False
        except sqlite3.Error as e:
            print(f"Database error: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return False

    def get_all(self) -> list[tuple]:
        """This function returns all the skills and their total time allocated."""
        try:
            self.cur.execute("SELECT Name, TotalTimeSpent FROM Skills ORDER BY Name")
            return self.cur.fetchall()
        except sqlite3.Error as e:
            print(f"Database error: {e}")
            return []
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return []

    def get(self, skill_name: str, include_sessions: bool = False) -> dict:
        """This function takes the skill name as input and returns its details.
        If include_sessions is true, then session details are also included in the output."""
        try:
            self.cur.execute("SELECT SkillID, Name, CreatedOn, TotalTimeSpent FROM Skills WHERE Name = ?", (skill_name,))
            skill_data = self.cur.fetchone()
            if not skill_data:
                raise ce.SkillNotFoundError(f"Skill '{skill_name}' not found.")

            skill_id, name, created_on, total_time_spent = skill_data
            result = {
                "SkillID": skill_id,
                "Name": name,
                "CreatedOn": created_on,
                "TotalTimeSpent": total_time_spent
            }

            if include_sessions:
                self.cur.execute("SELECT SessionID, StartTime, EndTime, Duration, Notes FROM Sessions WHERE SkillID = ? ORDER BY StartTime DESC", (skill_id,))
                sessions_data = self.cur.fetchall()
                session_list = []
                for s_id, s_start, s_end, s_duration, s_notes in sessions_data:
                    session_list.append({
                        "SessionID": s_id,
                        "StartTime": s_start,
                        "EndTime": s_end,
                        "Duration": s_duration,
                        "Notes": s_notes
                    })
                result["Sessions"] = session_list
            return result
        except ce.SkillNotFoundError as snf:
            print(snf)
            return {}
        except sqlite3.Error as e:
            print(f"Database error: {e}")
            return {}
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return {}

    def delete_skill(self, skill_name: str) -> bool:
        """This function, taking the skill name as input, deletes the skill and its data from the table."""
        try:
            self.cur.execute("SELECT SkillID FROM Skills WHERE Name = ?", (skill_name,))
            skill_id_result = self.cur.fetchone()
            if not skill_id_result:
                raise ce.SkillNotFoundError(f"Skill '{skill_name}' not found.")
            skill_id = skill_id_result[0]

            self.cur.execute("DELETE FROM Skills WHERE SkillID = ?", (skill_id,))
            self.conn.commit()
            # Also remove from active sessions if it was being tracked
            if skill_id in self._active_sessions:
                del self._active_sessions[skill_id]
            print(f"Skill '{skill_name}' and all its associated sessions have been deleted.")
            return True
        except ce.SkillNotFoundError as snf:
            print(snf)
            return False
        except sqlite3.Error as e:
            print(f"Database error: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return False

    def reset_skill(self, skill_name: str) -> bool:
        """This function keeps the skill and deletes all the data about the skill."""
        try:
            self.cur.execute("SELECT SkillID FROM Skills WHERE Name = ?", (skill_name,))
            skill_id_result = self.cur.fetchone()
            if not skill_id_result:
                raise ce.SkillNotFoundError(f"Skill '{skill_name}' not found.")
            skill_id = skill_id_result[0]

            # Delete all sessions for this skill
            self.cur.execute("DELETE FROM Sessions WHERE SkillID = ?", (skill_id,))
            # Reset TotalTimeSpent in Skills table
            self.cur.execute("UPDATE Skills SET TotalTimeSpent = 0 WHERE SkillID = ?", (skill_id,))
            self.conn.commit()
            # Clear from active sessions if it was being tracked
            if skill_id in self._active_sessions:
                del self._active_sessions[skill_id]
            print(f"All session data for skill '{skill_name}' has been reset, and TotalTimeSpent set to 0.")
            return True
        except ce.SkillNotFoundError as snf:
            print(snf)
            return False
        except sqlite3.Error as e:
            print(f"Database error: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return False
