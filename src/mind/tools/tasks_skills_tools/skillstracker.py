from datetime import datetime
from mind.utils import custom_exception as ce
from mind.utils.logging_handler import setup_logger
from mind.core.ports.memory_port import MemoryPort

class SkillsTracker:
    def __init__(self, memory_adapter: MemoryPort):
        """
        Initialize with a memory adapter (Port implementation).
        No DB connection is made here directly.
        """
        self.memory = memory_adapter
        self.logger = setup_logger(__name__)
        # Cache for active sessions: {SkillID: (SessionID, StartTime_datetime_object)}
        self._active_sessions = {} 

    def create_skill(self, skill_name: str) -> bool:
        """Create a new skill via the adapter."""
        try:
            # The adapter handles the 'exists' check internally via IntegrityError or we can check manually
            # Here we let the adapter try to create it.
            self.memory.create('skill', {'name': skill_name})
            self.logger.info(f"Skill '{skill_name}' created successfully.")
            return True
        except ce.SkillAlreadyExistsError as sae:
            self.logger.exception(sae)
            self.logger.info("Please choose a different skill name.")
            return False
        except Exception as e:
            self.logger.exception(f"An unexpected error occurred: {e}")
            return False

    def start_session(self, skill_name: str) -> bool:
        """Start a session using the adapter."""
        try:
            # 1. Get Skill ID
            skill_id = self.memory.retrieve('skill_id', {'name': skill_name})
            if not skill_id:
                raise ce.SkillNotFoundError(f"Skill '{skill_name}' not found.")

            # 2. Check memory cache
            if skill_id in self._active_sessions:
                self.logger.warning(f"Warning: A session for '{skill_name}' is already active (cached).")

            # 3. Check DB for active session
            active_db_session = self.memory.retrieve('active_session', {'skill_id': skill_id})
            if active_db_session:
                self.logger.warning(f"Warning: A session for '{skill_name}' is already active in DB.")
                return False

            # 4. Create Session
            start_time_dt = datetime.now()
            start_time_str = start_time_dt.strftime("%Y-%m-%d %H:%M:%S")
            
            session_id = self.memory.create('session', {
                'skill_id': skill_id, 
                'start_time': start_time_str
            })

            self._active_sessions[skill_id] = (session_id, start_time_dt)
            self.logger.info(f"Session started for '{skill_name}' (SessionID: {session_id}) at {start_time_str}")
            return True

        except ce.SkillNotFoundError as snf:
            self.logger.exception(snf)
            return False
        except Exception as e:
            self.logger.exception(f"An unexpected error occurred: {e}")
            return False

    def end_session(self, skill_name: str) -> bool:
        """End a session using the adapter."""
        try:
            skill_id = self.memory.retrieve('skill_id', {'name': skill_name})
            if not skill_id:
                raise ce.SkillNotFoundError(f"Skill '{skill_name}' not found.")

            session_id = None
            start_time_dt = None

            # 1. Try Cache
            if skill_id in self._active_sessions:
                session_id, start_time_dt = self._active_sessions[skill_id]
                del self._active_sessions[skill_id]
            else:
                # 2. Try DB
                active_db_session = self.memory.retrieve('active_session', {'skill_id': skill_id})
                if active_db_session:
                    session_id = active_db_session[0]
                    start_time_dt = datetime.strptime(active_db_session[1], "%Y-%m-%d %H:%M:%S")

            if not session_id or not start_time_dt:
                raise ce.NoActiveSessionError(f"No active session found for skill '{skill_name}'.")

            # 3. Calculate Duration
            end_time_dt = datetime.now()
            end_time_str = end_time_dt.strftime("%Y-%m-%d %H:%M:%S")
            time_difference = end_time_dt - start_time_dt
            duration_minutes = int(time_difference.total_seconds() / 60)

            # 4. Update Session and Skill via Adapter
            self.memory.update('session_end', 
                               {'end_time': end_time_str, 'duration': duration_minutes}, 
                               {'session_id': session_id})
            
            self.memory.update('skill_time', 
                               {'duration': duration_minutes}, 
                               {'skill_id': skill_id})

            self.logger.info(f"Session ended for '{skill_name}'. Duration: {duration_minutes} minutes.")
            return True

        except (ce.SkillNotFoundError, ce.NoActiveSessionError) as e:
            self.logger.exception(e)
            return False
        except Exception as e:
            self.logger.exception(f"An unexpected error occurred: {e}")
            return False

    def get_all(self) -> list[tuple]:
        """Fetch all skills."""
        try:
            return self.memory.retrieve('all_skills') or []
        except Exception as e:
            self.logger.exception(f"An unexpected error occurred: {e}")
            return []

    def get(self, skill_name: str, include_sessions: bool = False) -> dict:
        """Fetch details for a specific skill."""
        try:
            skill_data = self.memory.retrieve('skill_details', {'name': skill_name})
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
                sessions_data = self.memory.retrieve('skill_sessions', {'skill_id': skill_id})
                session_list = []
                if sessions_data:
                    for s in sessions_data:
                        session_list.append({
                            "SessionID": s[0],
                            "StartTime": s[1],
                            "EndTime": s[2],
                            "Duration": s[3],
                            "Notes": s[4]
                        })
                result["Sessions"] = session_list
            
            return result
        except ce.SkillNotFoundError as snf:
            self.logger.exception(snf)
            return {}
        except Exception as e:
            self.logger.exception(f"An unexpected error occurred: {e}")
            return {}

    def delete_skill(self, skill_name: str) -> bool:
        """Delete a skill."""
        try:
            skill_id = self.memory.retrieve('skill_id', {'name': skill_name})
            if not skill_id:
                raise ce.SkillNotFoundError(f"Skill '{skill_name}' not found.")

            self.memory.delete('skill', {'skill_id': skill_id})
            
            if skill_id in self._active_sessions:
                del self._active_sessions[skill_id]
                
            self.logger.info(f"Skill '{skill_name}' deleted.")
            return True
        except ce.SkillNotFoundError as snf:
            self.logger.exception(snf)
            return False
        except Exception as e:
            self.logger.exception(f"An unexpected error occurred: {e}")
            return False

    def reset_skill(self, skill_name: str) -> bool:
        """Reset skill stats."""
        try:
            skill_id = self.memory.retrieve('skill_id', {'name': skill_name})
            if not skill_id:
                raise ce.SkillNotFoundError(f"Skill '{skill_name}' not found.")

            self.memory.delete('skill_sessions', {'skill_id': skill_id})
            self.memory.update('reset_skill_time', {}, {'skill_id': skill_id})

            if skill_id in self._active_sessions:
                del self._active_sessions[skill_id]
                
            self.logger.info(f"Skill '{skill_name}' reset.")
            return True
        except ce.SkillNotFoundError as snf:
            self.logger.exception(snf)
            return False
        except Exception as e:
            self.logger.exception(f"An unexpected error occurred: {e}")
            return False