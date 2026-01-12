import os
import json
import asyncio
import argparse
import threading
from dataclasses import dataclass
from contextlib import asynccontextmanager

import uvicorn
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect

# Architecture Imports
from mind.utils import BASE_DIR
from core.agents.hitomi import Hitomi
from mind.core.status import RobotStatus

from mind.utils.logging_handler import setup_logger

from mind.adapters.llm_adapters.ollama_adapter import OllamaAdapter
from mind.adapters.robot_controller_adapters.mujoco_robot_adapter import MujocoRobot
from mind.adapters.fastapi_adapters.helper_adapters import AddEventListeners, ConnectionManager, FNScreenUpdater, Notifier, output_tuner
from mind.tools.tools_registry.core import Register
from mind.tools.tools_registry.tools_helpers import ToolsHelpers
from mind.adapters.fastapi_adapters.symbolic_handler import SymbolicHandler
from mind.adapters.memory_adapters.sqlite_memory_adapter import SqliteMemoryAdapter

logger = setup_logger(__name__)



@dataclass
class Args:
    sim: bool = False
    scene: str = "empty"
    wake_up_on_start: bool = False
    fastapi_host: str = "0.0.0.0"
    fastapi_port: int = 8000
    localhost_only: bool = False

# --- APP FACTORY ---

def create_app(args):
    @asynccontextmanager
    async def lifespan(app: FastAPI):

        loop = asyncio.get_running_loop()

        manager = ConnectionManager(loop=loop)
        notifier = Notifier(manager)
        app.state.connection_manager = manager
        app.state.screen_updater = FNScreenUpdater(manager)
        screen_updater = FNScreenUpdater(manager)
        presenters = [screen_updater]
        
        # Initialize Memory Adapter (using default path or configured path)
        db_path = os.path.join(BASE_DIR, "memory/memory.db")
        memory_adapter = SqliteMemoryAdapter(db_path=db_path)
        
        # Initialize Registry and Tools
        register = Register(loop=loop, memory_adapter=memory_adapter)
        registry = register.registry
        app.state.register = register
        app.state.registry = registry
        
        # Initialize Tools Helpers and Bind Listeners
        tools_helpers = ToolsHelpers(presenters)
        tools_instances = register.tools_instances

        add_event_listeners = AddEventListeners(tools_instances=tools_instances ,tools_helpers=tools_helpers)
        add_event_listeners.to_timer()
        add_event_listeners.to_stopwatch()
        add_event_listeners.to_pomodoro()


        if args.sim:
            robot_adapter = MujocoRobot(loop=loop)
            sim_thread = threading.Thread(target=robot_adapter.run, daemon=True, name="mujoco_sim_thread")
            sim_thread.start()
            app.state.sim_thread = sim_thread
            robot_adapter.wait_until_ready(timeout=10)
            
            robot_adapter.status_update_event.add_listener(manager.broadcast_status)
            robot_adapter.status_update_event.add_listener(notifier.notify_status)
            presenters.append(robot_adapter.bsh)
            
        else:
            from mind.core.ports.base_robot_controller_port import BaseRobotController
            class DummyRobot(BaseRobotController):
                def __init__(self, bus=None): 
                    self.bus = bus
                    self.show_clock = True
                def status(self): return "shutdown"
                def wake_up(self): pass
                def sleep(self): pass
                def shutdown(self): pass 
                def run(self): pass
                def stop(self): pass

            robot_adapter = DummyRobot()
    
        app.state.agents = {}
        # Initialize Symbolic Handler
        app.state.symbolic_handler = SymbolicHandler(tools_instances, presenters, robot_adapter)
        
        #  aliafshar/gemma3-it-qat-tools:4b
        async with OllamaAdapter(model="smollm2") as llm_adapter:
            main_agent = Hitomi(decision_maker=llm_adapter, robot_controller=robot_adapter, notifier=notifier, loop=loop)
            app.state.agents["main"] = main_agent
            
            if args.wake_up_on_start and robot_adapter:
                app.state.agents["main"].wake_up()

            yield # lifespan's yield 

        # Cleanup
        if robot_adapter:
            app.state.agents["main"].stop()
            if hasattr(app.state, 'sim_thread'):
                app.state.sim_thread.join(timeout=5.0)
        
        manager.broadcast_status("shutdown")

    app = FastAPI(lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/static")), name="static")
    templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/templates"))

    @app.get("/")
    def root(request: Request):
        return templates.TemplateResponse("index.html", {"request": request})

    @app.get("/favicon.ico", include_in_schema=False)
    def favicon():
        return FileResponse(os.path.join(BASE_DIR, "adapters/fastapi_adapters/static/favicon.ico"))

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        manager = app.state.connection_manager
        main_agent = app.state.agents["main"]
        
        await manager.connect(websocket)
        
        try:
            initial_status = RobotStatus.SHUTDOWN.value
            if main_agent.robot_controller:
                initial_status = main_agent.robot_controller.status()
            manager.broadcast_status(initial_status)

            response_handler = main_agent.input_handler()  # Single instance per connection
            current_task = None  # NEW: Track streaming task
            current_session = None  # NEW: Track session
            app.state.chat_names = ["casual chat", "doubt chat"]

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
                    ask_doubt = True if current_session == "doubt" else False

                    # Symbolic Handler Logic
                    if user_text.strip().startswith("/"):
                        sym_hand_output = await app.state.symbolic_handler.process(user_text)
                        # Optionally send a confirmation "end of stream" signal so UI knows cmd is done
                        if sym_hand_output:
                            app.state.chat_names[0] = sym_hand_output 
                        await websocket.send_text(json.dumps({
                            "type": "stream_end",
                            "session": session
                        }))
                        continue
                    
                    # NEW: Cancel previous task if running
                    if current_task and not current_task.done():
                        current_task.cancel()
                        try:
                            await current_task
                        except asyncio.CancelledError:
                            pass
                    
                    context = msg.get("context")

                    # NEW: Create newjson.dump task for streaming
                    async def stream_response():
                        try:
                            if hasattr(response_handler, 'astream'):
                                async for chunk in response_handler.astream(user_text, ask_doubt, app.state.chat_names, context=context):
                                    await websocket.send_text(json.dumps({
                                        "type": "chunk", 
                                        "text": chunk, 
                                        "session": session  # NEW: Include session
                                    }))
                                    await asyncio.sleep(0.01)
                            else:
                                full = response_handler.get_full_response(user_text, ask_doubt, app.state.chat_names, context=context)
                                await websocket.send_text(json.dumps({
                                    "type": "response", 
                                    "text": full,
                                    "session": session
                                }))   
                        except asyncio.CancelledError:
                            # NEW: Graceful cancel
                            pass
                        finally:
                            # NEW: Send end signal
                            await websocket.send_text(json.dumps({
                                "type": "stream_end", 
                                "session": session
                            }))
                    
                    current_task = asyncio.create_task(stream_response())

                elif msg_type == "stop":  # UPDATED: Ensure end signal
                    stop_session = msg.get("session")
                    if stop_session == current_session and current_task and not current_task.done():
                        current_task.cancel()
                        logger.debug("called stop")
                        try:
                            await current_task
                        except asyncio.CancelledError:
                            pass
                        # Force end signal
                        await websocket.send_text(json.dumps({"type": "stream_end", "session": stop_session}))
                        current_task = None
                        current_session = None
                    else:
                        # If no task, still send end to reset frontend
                        await websocket.send_text(json.dumps({"type": "stream_end", "session": stop_session}))

                elif msg_type == "power":
                    action = msg["data"]["action"]
                    new_status = main_agent.handle_power_command(action)  # NEW: Get new status
                    manager.broadcast_status(new_status)  # NEW: Broadcast update
                    
        except WebSocketDisconnect:
            # NEW: Cancel task on disconnect
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