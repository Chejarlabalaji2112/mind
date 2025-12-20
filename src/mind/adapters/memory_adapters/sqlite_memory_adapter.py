import sqlite3
from datetime import datetime
from typing import Any, Dict, List, Optional
from mind.utils import custom_exception as ce
from mind.utils.logging_handler import setup_logger
from mind.ports.memory_port import MemoryPort  # Assuming memory_port.py is in the same dir

class SqliteMemoryAdapter(MemoryPort):
    def __init__(self, db_path="/home/badri/mine/hitomi/mind/src/mind/memory/memory.db"):
        self.logger = setup_logger(__name__)
        self.conn = sqlite3.connect(db_path)
        self.cur = self.conn.cursor()
        self.cur.execute("PRAGMA foreign_keys = ON;")
        self._initialize_tables()

    def _initialize_tables(self):
        """Private method to ensure schema exists."""
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

    def encode(self, raw_input: Any):
        """Not used for simple SQL tracking, but required by interface."""
        pass

    def create(self, entity_type: str, data: Dict[str, Any]) -> int:
        """
        Creates a new record.
        entity_type: 'skill' or 'session'
        """
        try:
            if entity_type == 'skill':
                self.cur.execute("""
                INSERT INTO Skills (Name, CreatedOn, TotalTimeSpent) 
                VALUES (?, DATE('now'), 0)""", (data['name'],))
                self.conn.commit()
                return self.cur.lastrowid
            
            elif entity_type == 'session':
                self.cur.execute("""
                INSERT INTO Sessions (SkillID, StartTime, EndTime, Duration, Notes) 
                VALUES (?, ?, NULL, NULL, NULL)""", 
                (data['skill_id'], data['start_time']))
                self.conn.commit()
                return self.cur.lastrowid
                
        except sqlite3.IntegrityError:
            # Re-raise as domain exception if it's a duplicate skill
            if entity_type == 'skill':
                raise ce.SkillAlreadyExistsError(f"Skill '{data.get('name')}' already exists.")
            raise
        except sqlite3.Error as e:
            self.logger.exception(f"Database error in create: {e}")
            raise

    def retrieve(self, query_type: str, filters: Dict[str, Any] = None) -> Any:
        """
        Retrieves data based on query_type.
        query_types: 'skill_id', 'active_session', 'all_skills', 'skill_details', 'skill_sessions'
        """
        filters = filters or {}
        try:
            if query_type == 'skill_id':
                self.cur.execute("SELECT SkillID FROM Skills WHERE Name = ?", (filters['name'],))
                res = self.cur.fetchone()
                return res[0] if res else None

            elif query_type == 'active_session':
                # Returns (SessionID, StartTime)
                self.cur.execute("""
                    SELECT SessionID, StartTime FROM Sessions 
                    WHERE SkillID = ? AND EndTime IS NULL 
                    ORDER BY StartTime DESC LIMIT 1
                """, (filters['skill_id'],))
                return self.cur.fetchone()

            elif query_type == 'all_skills':
                self.cur.execute("SELECT Name, TotalTimeSpent FROM Skills ORDER BY Name")
                return self.cur.fetchall()

            elif query_type == 'skill_details':
                self.cur.execute("SELECT SkillID, Name, CreatedOn, TotalTimeSpent FROM Skills WHERE Name = ?", (filters['name'],))
                return self.cur.fetchone()

            elif query_type == 'skill_sessions':
                self.cur.execute("SELECT SessionID, StartTime, EndTime, Duration, Notes FROM Sessions WHERE SkillID = ? ORDER BY StartTime DESC", (filters['skill_id'],))
                return self.cur.fetchall()
                
        except sqlite3.Error as e:
            self.logger.exception(f"Database error in retrieve: {e}")
            return None

    def update(self, entity_type: str, data: Dict[str, Any], criteria: Dict[str, Any]):
        """
        Updates records.
        entity_type: 'session_end', 'skill_time', 'reset_skill_time'
        """
        try:
            if entity_type == 'session_end':
                self.cur.execute("""
                    UPDATE Sessions SET EndTime = ?, Duration = ? WHERE SessionID = ?
                """, (data['end_time'], data['duration'], criteria['session_id']))
                
            elif entity_type == 'skill_time':
                self.cur.execute("""
                    UPDATE Skills SET TotalTimeSpent = TotalTimeSpent + ? WHERE SkillID = ?
                """, (data['duration'], criteria['skill_id']))
                
            elif entity_type == 'reset_skill_time':
                self.cur.execute("UPDATE Skills SET TotalTimeSpent = 0 WHERE SkillID = ?", (criteria['skill_id'],))

            self.conn.commit()
        except sqlite3.Error as e:
            self.logger.exception(f"Database error in update: {e}")
            raise

    def delete(self, entity_type: str, criteria: Dict[str, Any]):
        """
        Deletes records.
        entity_type: 'skill', 'skill_sessions'
        """
        try:
            if entity_type == 'skill':
                self.cur.execute("DELETE FROM Skills WHERE SkillID = ?", (criteria['skill_id'],))
            elif entity_type == 'skill_sessions':
                self.cur.execute("DELETE FROM Sessions WHERE SkillID = ?", (criteria['skill_id'],))
            
            self.conn.commit()
        except sqlite3.Error as e:
            self.logger.exception(f"Database error in delete: {e}")
            raise

    def store(self): 
        # Optional: Could be used for commits if we managed transactions manually
        pass

    def consolidate(self):
        # Optional: Could be used for database vacuuming or archiving old sessions
        pass