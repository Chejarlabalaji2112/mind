from datetime import datetime, timedelta
import time

class PomodoroManager:
    def __init__(self, db):
        self.db = db
        self.active_pomodoros = {} # {pomo_id: {"start_time": datetime, "config": {...}, "current_phase": str, "phase_end_time": datetime}}

    def start_pomodoro(self, work_duration=25, short_break=5, long_break=15, cycles=4, label="Pomodoro"):
        """Starts a new Pomodoro session."""
        if not all(isinstance(arg, (int, float)) and arg > 0 for arg in [work_duration, short_break, long_break, cycles]):
            print("Error: Pomodoro durations and cycles must be positive numbers.")
            return None

        start_time = datetime.now()
        config = {
            "work_duration": work_duration,
            "short_break": short_break,
            "long_break": long_break,
            "cycles": cycles
        }
        current_phase = "work"
        phase_end_time = start_time + timedelta(minutes=work_duration)

        pomo_id = self.db.insert_pomodoro(
            work_duration, short_break, long_break, cycles, 1, current_phase,
            start_time.isoformat(), phase_end_time.isoformat(), label, True
        )
        if pomo_id:
            self.active_pomodoros[pomo_id] = {
                "start_time": start_time,
                "config": config,
                "current_phase": current_phase,
                "phase_end_time": phase_end_time,
                "current_cycle": 1,
                "label": label
            }
        return pomo_id

    def get_active_pomodoros(self):
        """Retrieves all active Pomodoro sessions with their current status."""
        db_pomodoros = self.db.get_all_pomodoros()
        formatted_pomodoros = []
        for pomo in db_pomodoros:
            (pomo_id, work_dur, short_break, long_break, total_cycles,
             current_cycle, current_phase, start_time_str, phase_end_time_str, label, is_active) = pomo

            if is_active:
                phase_end_time = datetime.fromisoformat(phase_end_time_str)
                remaining_seconds = (phase_end_time - datetime.now()).total_seconds()

                if remaining_seconds <= 0:
                    # Phase has ended, transition to next phase
                    current_phase, current_cycle, phase_end_time, is_active = \
                        self._transition_pomodoro_phase(pomo_id, work_dur, short_break, long_break, total_cycles, current_cycle, current_phase)
                    self.db.update_pomodoro(pomo_id, current_cycle, current_phase, phase_end_time.isoformat(), is_active)
                    if not is_active: # Pomodoro session completed
                        continue # Skip adding to active list

                formatted_pomodoros.append({
                    "id": pomo_id,
                    "label": label,
                    "work_duration": work_dur,
                    "short_break": short_break,
                    "long_break": long_break,
                    "total_cycles": total_cycles,
                    "current_cycle": current_cycle,
                    "current_phase": current_phase,
                    "remaining_seconds_in_phase": int(remaining_seconds) if remaining_seconds > 0 else 0,
                    "start_time": start_time_str
                })
        return formatted_pomodoros

    def _transition_pomodoro_phase(self, pomo_id, work_dur, short_break, long_break, total_cycles, current_cycle, current_phase):
        now = datetime.now()
        new_phase_end_time = now
        new_current_cycle = current_cycle
        new_is_active = True

        if current_phase == "work":
            if current_cycle < total_cycles:
                new_phase = "short_break"
                new_phase_end_time += timedelta(minutes=short_break)
                print(f"Pomodoro {pomo_id} - Work phase ended. Starting Short Break ({short_break} min).")
            else:
                new_phase = "long_break"
                new_phase_end_time += timedelta(minutes=long_break)
                print(f"Pomodoro {pomo_id} - Work phase ended. Starting Long Break ({long_break} min).")
        elif current_phase == "short_break":
            new_current_cycle += 1
            new_phase = "work"
            new_phase_end_time += timedelta(minutes=work_dur)
            print(f"Pomodoro {pomo_id} - Short Break ended. Starting Work phase ({work_dur} min).")
        elif current_phase == "long_break":
            new_is_active = False # Session completed
            new_phase = "completed"
            print(f"Pomodoro {pomo_id} - Long Break ended. Session Completed.")
        else: # Should not happen
            new_is_active = False
            new_phase = "error"

        return new_phase, new_current_cycle, new_phase_end_time, new_is_active

    def stop_pomodoro(self, pomodoro_id):
        """Stops and deactivates a specific Pomodoro session."""
        if self.db.update_pomodoro_status(pomodoro_id, False):
            if pomodoro_id in self.active_pomodoros:
                del self.active_pomodoros[pomodoro_id]
            return True
        return False
