import time
import sys
from mind.adapters.memory_adapters.sqlite_memory_adapter import SqliteMemoryAdapter
from mind.tools.tasks_skills_tools.skillstracker import SkillTracker

# Ensure simple logging to console for the demo
import logging
logging.basicConfig(level=logging.INFO, format='%(message)s')

def main():
    print("=== INITIALIZING ARCHITECTURE ===")
    
    # 1. THE ADAPTER (The "Plug")
    # We instantiate the concrete implementation of MemoryPort. 
    # This is the only part of the code that knows "sqlite3" exists.
    db_adapter = SqliteMemoryAdapter(db_path="/home/badri/mine/hitomi/mind/src/mind/memory/memory.db")
    print("✓ Adapter connected to SQLite")

    # 2. THE CORE (The "Logic")
    # We initialize the tracker. Notice we pass the adapter object.
    # SkillTracker does not know it's using SQLite; it just knows it has a 'memory_adapter'.
    tracker = SkillTracker(memory_adapter=db_adapter)
    print("✓ SkillTracker initialized with generic memory adapter")

    # --- DEMONSTRATION WORKFLOW ---

    skill_name = "Python Refactoring"

    # Step 1: Create a Skill
    print(f"\n[1] Creating skill: '{skill_name}'")
    success = tracker.create_skill(skill_name)
    if not success:
        print(f"   (Note: Skill '{skill_name}' might already exist, proceeding...)")

    # Step 2: Start a Session
    print(f"\n[2] Starting session for '{skill_name}'...")
    if tracker.start_session(skill_name):
        print("   >> Session STARTED. Timer is running.")
    else:
        print("   >> Failed to start session (maybe one is already active?)")

    # Step 3: Simulate Work (Time Passing)
    print("\n[3] Simulating work... (Sleeping for 2 seconds)")
    print("   (Note: Real duration logic divides by 60, so <60s will show as 0 mins)")
    for i in range(90):
        print(i, end="")
        time.sleep(1)

    # Step 4: End Session
    print(f"\n[4] Ending session for '{skill_name}'...")
    if tracker.end_session(skill_name):
        print("   >> Session ENDED. Data saved to adapter.")
    
    # Step 5: Retrieve Data (Using the Port)
    print(f"\n[5] Retrieving stats for '{skill_name}'...")
    data = tracker.get(skill_name, include_sessions=True)
    
    # Pretty print the result
    print(f"   Name: {data.get('Name')}")
    print(f"   Total Minutes: {data.get('TotalTimeSpent')}")
    print(f"   Session History:")
    for session in data.get('Sessions', []):
        print(f"     - ID: {session['SessionID']} | Start: {session['StartTime']} | Duration: {session['Duration']} min")

    # Step 6: Get All Skills
    print("\n[6] Listing ALL skills in database:")
    all_skills = tracker.get_all()
    for name, time_spent in all_skills:
        print(f"   - {name}: {time_spent} mins")

    # Optional Cleanup
    # print("\n[7] Cleaning up...")
    # tracker.delete_skill(skill_name)
    # print("   Skill deleted.")

if __name__ == "__main__":
    main()