import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

# Adjust imports based on your project structure
from mind.adapters.memory_adapters.sqlite_memory_adapter import SqliteMemoryAdapter
from mind.tools.tasks_skills_tools.skillstracker import SkillsTracker

app = FastAPI(title="Skills Tracker UI")

# Initialize the adapter and tracker
db_adapter = SqliteMemoryAdapter(db_path="/home/badri/mine/hitomi/mind/src/mind/memory/memory.db")
tracker = SkillsTracker(memory_adapter=db_adapter)

class SkillRequest(BaseModel):
    skill_name: str

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Skills Tracker UI</title>
    <script src="https://cdn.tailwindcss.com"></script>
</head>
<body class="bg-gray-100 text-gray-800 font-sans p-8">

    <div class="max-w-5xl mx-auto">
        <h1 class="text-3xl font-bold mb-8 text-center text-blue-600">Skills Tracker</h1>
        
        <div class="bg-white p-6 rounded-lg shadow-md mb-8 flex gap-4 items-center">
            <input type="text" id="new-skill-name" placeholder="Enter new skill name..." 
                   class="flex-1 border border-gray-300 rounded px-4 py-2 focus:outline-none focus:ring-2 focus:ring-blue-500">
            <button onclick="createSkill()" 
                    class="bg-blue-600 text-white px-6 py-2 rounded hover:bg-blue-700 font-semibold transition">
                Create Skill
            </button>
        </div>

        <div id="skills-container" class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
            </div>
    </div>

    <script>
        async function fetchSkills() {
            const res = await fetch('/api/skills');
            const skills = await res.json();
            const container = document.getElementById('skills-container');
            container.innerHTML = '';
            
            skills.forEach(skill => {
                const card = document.createElement('div');
                card.className = 'bg-white p-6 rounded-lg shadow border border-gray-200 flex flex-col justify-between';
                
                let totalTime = skill.TotalTimeSpent || 0;
                
                card.innerHTML = `
                    <div>
                        <h2 class="text-xl font-bold text-gray-800 mb-1">${skill.Name}</h2>
                        <p class="text-sm text-gray-500 mb-4">Total time spent: <span class="font-bold text-blue-600">${totalTime} mins</span></p>
                    </div>
                    <div class="flex space-x-3 mt-4">
                        <button onclick="actionSkill('${skill.Name}', 'start')" 
                                class="flex-1 bg-green-500 text-white px-4 py-2 rounded hover:bg-green-600 transition font-medium">
                            Start Session
                        </button>
                        <button onclick="actionSkill('${skill.Name}', 'end')" 
                                class="flex-1 bg-red-500 text-white px-4 py-2 rounded hover:bg-red-600 transition font-medium">
                            End Session
                        </button>
                    </div>
                `;
                container.appendChild(card);
            });
        }

        async function createSkill() {
            const input = document.getElementById('new-skill-name');
            const name = input.value.trim();
            if (!name) return alert("Please enter a skill name.");
            
            const res = await fetch('/api/skills', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ skill_name: name })
            });
            
            if (res.ok) {
                input.value = '';
                fetchSkills();
            } else {
                alert("Failed to create skill. It might already exist.");
            }
        }

        async function actionSkill(name, action) {
            const res = await fetch(`/api/skills/${action}`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ skill_name: name })
            });
            
            const data = await res.json();
            alert(data.message);
            fetchSkills(); // Refresh the grid to show updated stats if needed
        }

        // Initial load
        fetchSkills();
    </script>
</body>
</html>
"""

@app.get("/", response_class=HTMLResponse)
def serve_ui():
    """Serves the main HTML interface."""
    return HTMLResponse(content=HTML_TEMPLATE)

@app.get("/api/skills")
def get_all_skills():
    """Fetches all skills and returns their detailed data."""
    skills_data = []
    try:
        raw_skills = tracker.get_all()
        for row in raw_skills:
            # Assuming row[1] contains the skill name based on standard DB structures
            name = row[1] 
            detail = tracker.get(name, include_sessions=False)
            if detail:
                skills_data.append(detail)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    return skills_data

@app.post("/api/skills")
def create_skill(req: SkillRequest):
    """Creates a new skill."""
    success = tracker.create_skill(req.skill_name)
    if success:
        return {"message": f"Skill '{req.skill_name}' created."}
    raise HTTPException(status_code=400, detail="Failed to create skill.")

@app.post("/api/skills/start")
def start_skill_session(req: SkillRequest):
    """Starts a session for a specific skill."""
    success = tracker.start_session(req.skill_name)
    if success:
        return {"message": f"Session started for '{req.skill_name}'."}
    return {"message": f"Failed to start session. One may already be active."}

@app.post("/api/skills/end")
def end_skill_session(req: SkillRequest):
    """Ends the active session for a specific skill."""
    success = tracker.end_session(req.skill_name)
    if success:
        return {"message": f"Session ended for '{req.skill_name}'."}
    return {"message": f"Failed to end session. No active session found."}

if __name__ == "__main__":
    print("Starting Web UI on http://127.0.0.1:8000")
    uvicorn.run(app, host="127.0.0.1", port=8000)
