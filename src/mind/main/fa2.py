import os
import json
import asyncio
import argparse
import threading
import re
from dataclasses import dataclass
from contextlib import asynccontextmanager

import uvicorn
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect

# Architecture Imports
from mind.utils import BASE_DIR
from mind.core.agents.agent import Agent
from mind.core.status import RobotStatus

from mind.utils.logging_handler import setup_logger
from mind.utils.time_conversions import convert_to_seconds

from mind.adapters.llm_adapters.llm_without_agnetv1 import OllamaAdapter
from mind.adapters.robot_controller_adapters.mujoco_robot_adapter import MujocoRobot
from mind.adapters.fastapi_adapters.helper_adapters import ConnectionManager, FNScreenUpdater, Notifier, output_tuner
from mind.tools.tools_registry.core import Register
from mind.tools.tools_registry.tools_helpers import ToolsHelpers

logger = setup_logger(__name__)


# --- SYMBOLIC LOGIC HANDLER ---

class SymbolicToolHandler:
    """
    Handles direct commands starting with '/' to control tools without LLM inference.
    """
    def __init__(self, tool_instances, notifier: Notifier):
        self.tools = tool_instances
        self.notifier = notifier

    def parse_duration(self, duration_str: str) -> int:
        """Parses strings like '1h20m30s' or '10m' into seconds."""
        # Regex to capture hours, minutes, seconds
        h_match = re.search(r'(\d+)h', duration_str)
        m_match = re.search(r'(\d+)m', duration_str)
        s_match = re.search(r'(\d+)s', duration_str)
        
        hours = int(h_match.group(1)) if h_match else 0
        minutes = int(m_match.group(1)) if m_match else 0
        seconds = int(s_match.group(1)) if s_match else 0
        
        # Fallback: if user sent just a number, assume seconds
        if hours == 0 and minutes == 0 and seconds == 0 and duration_str.isdigit():
            seconds = int(duration_str)
            
        return convert_to_seconds(hours, minutes, seconds)

    async def process_command(self, text: str, session: str):
        """Dispatches command to specific tool handlers."""
        parts = text.strip().split()
        if not parts:
            return

        cmd = parts[0].lower()
        
        if cmd == "/timer":
            self._handle_timer(parts[1:])
        elif cmd == "/stopwatch":
            self._handle_stopwatch(parts[1:])
        elif cmd == "/sktracker":
            self._handle_skills(parts[1:])
        elif cmd == "/pomodoro":
            self._handle_pomodoro(parts[1:])
        else:
            self.notifier.notify_content(f"Unknown command: {cmd}")

    def _handle_timer(self, args):
        # Usage: /timer start 10m30s | /timer pause | /timer resume | /timer stop
        if not args: return
        action = args[0].lower()
        timer = self.tools.timer

        if action == "start" and len(args) > 1:
            duration = self.parse_duration(args[1])
            timer.start(duration)
        elif action == "pause":
            timer.pause()
        elif action == "resume":
            timer.resume()
        elif action == "stop":
            timer.stop()
        elif action == "reset":
            timer.reset()

    def _handle_stopwatch(self, args):
        # Usage: /stopwatch start | /stopwatch stop | /stopwatch lap | /stopwatch reset
        if not args: return
        action = args[0].lower()
        stopwatch = self.tools.stopwatch

        if action == "start":
            stopwatch.start()
        elif action == "stop":
            stopwatch.stop()
        elif action == "pause":
            stopwatch.pause()
        elif action == "resume":
            stopwatch.resume()
        elif action == "reset":
            stopwatch.reset()
        elif action == "lap":
            stopwatch.lap()

    def _handle_pomodoro(self, args):
        # Usage: /pomodoro start | /pomodoro stop
        if not args: return
        action = args[0].lower()
        pomodoro = self.tools.pomodoro

        if action == "start":
            pomodoro.start()
        elif action == "stop":
            pomodoro.stop()
        elif action == "pause":
            pomodoro.pause()
        elif action == "resume":
            pomodoro.resume()
        elif action == "reset":
            pomodoro.reset()

    def _handle_skills(self, args):
        # Usage: /sktracker <skill_name> start | /sktracker <skill_name> end | /sktracker create <skill_name>
        # Note: Logic follows the pattern: /tool [arg1] [action]
        if len(args) < 2: return
        
        # Check if first arg is 'create' command or a skill name
        if args[0].lower() == "create":
            skill_name = args[1]
            success = self.tools.skillstracker.create_skill(skill_name)
            msg = f"Skill '{skill_name}' created." if success else f"Failed to create '{skill_name}'."
            self.notifier.notify_content(msg)
            return

        skill_name = args[0]
        action = args[1].lower()

        if action == "start":
            success = self.tools.skillstracker.start_session(skill_name)
            msg = f"Started tracking: {skill_name}" if success else f"Could not start: {skill_name}"
            self.notifier.notify_content(msg)
        elif action == "end" or action == "stop":
            success = self.tools.skillstracker.end_session(skill_name)
            # Duration is logged by the tool, we just notify success
            msg = f"Stopped tracking: {skill_name}" if success else f"Could not stop: {skill_name}"
            self.notifier.notify_content(msg)


# --- APP FACTORY ---

@dataclass
class Args:
    sim: bool = False
    scene: str = "empty"
    wake_up_on_start: bool = False
    fastapi_host: str = "0.0.0.0"
    fastapi_port: int = 8000
    localhost_only: bool = False

def create_app(args):
    @asynccontextmanager
    async def lifespan(app: FastAPI):

        loop = asyncio.get_running_loop()

        manager = ConnectionManager(loop=loop)
        notifier = Notifier(manager)
        app.state.connection_manager = manager
        app.state.screen_updater = FNScreenUpdater(manager)
        
        llm_adapter = OllamaAdapter()
        robot_adapter = None
        
        # Tools Setup
        screen_updater = FNScreenUpdater(manager)
        # 1. Initialize ToolsHelpers with the presenter
        tools_helpers = ToolsHelpers(presenters=[screen_updater])
        
        # 2. Register tools
        # Pass memory_adapter to Register if you have one, assuming generic here or None based on previous code
        from mind.adapters.memory_adapters.sqlite_memory_adapter import SqliteMemoryAdapter
        memory_adapter = SqliteMemoryAdapter() # Or pass intended path
        
        register = Register(loop=loop, memory_adapter=memory_adapter)
        registry = register.registry
        tool_instances = register.tool_instances
        
        app.state.register = register
        app.state.registry = registry
        
        # 3. Wire up Tool Events to Helper Listeners
        # Timer
        t = tool_instances.timer
        t.on_start.add_listener(tools_helpers.timer_on_start_handler)
        t.on_tick.add_listener(tools_helpers.timer_on_tick_handler)
        t.on_pause.add_listener(tools_helpers.timer_on_pause_handler)
        t.on_resume.add_listener(tools_helpers.timer_on_resume_handler)
        t.on_stop.add_listener(tools_helpers.timer_on_stop_handler)
        t.on_reset.add_listener(tools_helpers.timer_on_reset_handler)
        t.on_finished.add_listener(tools_helpers.timer_on_finished_handler)

        # Stopwatch
        s = tool_instances.stopwatch
        s.on_start.add_listener(tools_helpers.stopwatch_on_start_handler)
        s.on_tick.add_listener(tools_helpers.stopwatch_on_tick_handler)
        s.on_pause.add_listener(tools_helpers.stopwatch_on_pause_handler)
        s.on_resume.add_listener(tools_helpers.stopwatch_on_resume_handler)
        s.on_stop.add_listener(tools_helpers.stopwatch_on_stop_handler)
        s.on_reset.add_listener(tools_helpers.stopwatch_on_reset_handler)
        s.on_lap.add_listener(tools_helpers.stopwatch_on_lap_handler)

        # Pomodoro (Assuming Pomodoro events match helper names)
        p = tool_instances.pomodoro
        p.on_tick.add_listener(tools_helpers.pomodoro_on_tick_handler)
        p.on_work_start.add_listener(tools_helpers.pomodoro_on_work_start_handler)
        p.on_short_break_start.add_listener(tools_helpers.pomodoro_on_short_break_start_handler)
        p.on_long_break_start.add_listener(tools_helpers.pomodoro_on_long_break_start_handler)
        p.on_phase_end.add_listener(tools_helpers.pomodoro_on_phase_end_handler)
        p.on_cycle_complete.add_listener(tools_helpers.pomodoro_on_cycle_complete_handler)
        p.on_pause.add_listener(tools_helpers.pomodoro_on_pause_handler)
        p.on_resume.add_listener(tools_helpers.pomodoro_on_resume_handler)
        p.on_stop.add_listener(tools_helpers.pomodoro_on_stop_handler)
        p.on_reset.add_listener(tools_helpers.pomodoro_on_reset_handler)

        # 4. Initialize Symbolic Handler
        symbolic_handler = SymbolicToolHandler(tool_instances, notifier)
        app.state.symbolic_handler = symbolic_handler


        if args.sim:
            robot_adapter = MujocoRobot(loop=loop)
            sim_thread = threading.Thread(target=robot_adapter.run, daemon=True, name="mujoco_sim_thread")
            sim_thread.start()
            app.state.sim_thread = sim_thread
            robot_adapter.wait_until_ready(timeout=10)
            
            robot_adapter.status_update_event.add_listener(manager.broadcast_status)
            robot_adapter.status_update_event.add_listener(notifier.notify_status)
            
        else:
            from mind.core.ports.base_robot_controller_port import BaseRobotController
            class DummyRobot(BaseRobotController):
                def __init__(self, bus=None): self.bus = bus
                def status(self): return "shutdown"
                def wake_up(self): pass
                def sleep(self): pass
                def shutdown(self): pass 
                def run(self): pass
                def stop(self): pass

            robot_adapter = DummyRobot()

        agent = Agent(decision_maker=llm_adapter, robot_controller=robot_adapter, notifier=notifier, loop=loop)
        app.state.agent = agent
        
        if args.wake_up_on_start and robot_adapter:
            agent.wake_up()

        yield

        # Cleanup
        if robot_adapter:
            agent.stop()
            if hasattr(app.state, 'sim_thread'):
                app.state.sim_thread.join(timeout=5.0)
        
        manager.broadcast_status("shutdown")

    app = FastAPI(lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/static")), name="static")
    templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/templates"))

    @app.get("/")
    def root(request: Request):
        return templates.TemplateResponse("index.html", {"request": request})

    @app.get("favicon.ico", include_in_schema=False)
    def favicon():
        return FileResponse(os.path.join(BASE_DIR, "adapters/fastapi_adapters/static/favicon.ico"))

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        manager = app.state.connection_manager
        agent = app.state.agent
        symbolic_handler = app.state.symbolic_handler
        
        await manager.connect(websocket)
        
        try:
            initial_status = RobotStatus.SHUTDOWN.value
            if agent.robot_controller:
                initial_status = agent.robot_controller.status()
            manager.broadcast_status(initial_status)

            response_handler = agent.input_handler()
            current_task = None
            current_session = None

            while True:
                data = await websocket.receive_text()
                try:
                    msg = json.loads(data)
                except json.JSONDecodeError:
                    continue

                msg_type = msg.get("type")

                if msg_type == "text":
                    user_text = msg["data"]  
                    session = msg.get("session", "main")
                    current_session = session
                    
                    # --- NEW: SYMBOLIC LOGIC CHECK ---
                    if user_text.strip().startswith("/"):
                        await symbolic_handler.process_command(user_text, session)
                        # Ensure we notify end of stream to clean up UI state on frontend if needed
                        await websocket.send_text(json.dumps({
                            "type": "stream_end", 
                            "session": session
                        }))
                        continue
                    # ----------------------------------

                    # Cancel previous task if running
                    if current_task and not current_task.done():
                        current_task.cancel()
                        try:
                            await current_task
                        except asyncio.CancelledError:
                            pass
                    
                    async def stream_response():
                        try:
                            if hasattr(response_handler, 'astream'):
                                async for chunk in response_handler.astream(user_text):
                                    await websocket.send_text(json.dumps({
                                        "type": "chunk", 
                                        "text": chunk, 
                                        "session": session
                                    }))
                                    await asyncio.sleep(0.01)
                            else:
                                full = response_handler.get_full_response()
                                await websocket.send_text(json.dumps({
                                    "type": "response", 
                                    "text": full,
                                    "session": session
                                }))   
                        except asyncio.CancelledError:
                            pass
                        finally:
                            await websocket.send_text(json.dumps({
                                "type": "stream_end", 
                                "session": session
                            }))
                    
                    current_task = asyncio.create_task(stream_response())

                elif msg_type == "stop":
                    stop_session = msg.get("session")
                    if stop_session == current_session and current_task and not current_task.done():
                        current_task.cancel()
                        logger.debug("called stop")
                        try:
                            await current_task
                        except asyncio.CancelledError:
                            pass
                        await websocket.send_text(json.dumps({"type": "stream_end", "session": stop_session}))
                        current_task = None
                        current_session = None
                    else:
                        await websocket.send_text(json.dumps({"type": "stream_end", "session": stop_session}))

                elif msg_type == "power":
                    action = msg["data"]["action"]
                    new_status = agent.handle_power_command(action)
                    manager.broadcast_status(new_status)

        except WebSocketDisconnect:
            if current_task and not current_task.done():
                current_task.cancel()
            manager.disconnect(websocket)
        except Exception as e:
            logger.error(f"WS Error: {e}")
            if current_task and not current_task.done():
                current_task.cancel()
            manager.disconnect(websocket)

    return app

def run_app(args: Args) -> None:
    app = create_app(args)
    try:
        uvicorn.run(app, host=args.fastapi_host, port=args.fastapi_port)
    except Exception as e:
        logger.error(f"error in run_app: {e}")

def main() -> None:
    default_args = Args()
    parser = argparse.ArgumentParser(description="Main entry of the mind.")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--scene", type=str, default=default_args.scene)
    parser.add_argument("--wake-up-on-start", action="store_true")
    parser.add_argument("--fastapi-host", type=str, default=default_args.fastapi_host)
    parser.add_argument("--fastapi-port", type=int, default=default_args.fastapi_port)
    parser.add_argument("--localhost-only", action="store_true")
    parsed_args = parser.parse_args()
    run_app(Args(**vars(parsed_args)))

if __name__ == "__main__":
    logger.info("="*50)
    main()