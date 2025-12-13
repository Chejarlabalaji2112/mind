#
import os
import json
import time
import asyncio
import threading
import uvicorn
import argparse
from pydantic import BaseModel
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from dataclasses import dataclass

# Architecture Imports
from mind.core.agent import Agent
from mind.core.status import RobotStatus
from mind.adapters.llm_adapters.llm_without_agnetv1 import OllamaAdapter
from mind.adapters.robot_controller_adapters.mujoco_robot_adapter import MujocoRobot
from mind.utils import BASE_DIR
from mind.utils.logging_handler import setup_logger
from mind.ports.notification_port import NotificationPort
from typing import Optional


logger = setup_logger(__name__)


# --- CONNECTION MANAGER ---
class ConnectionManager:
    # 1. FIX: Accept 'loop' to handle cross-thread scheduling
    def __init__(self, loop: asyncio.AbstractEventLoop):
        self.active_connections: set[WebSocket] = set()
        self._lock = threading.Lock()
        self.loop = loop

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        with self._lock:
            self.active_connections.add(websocket)

    def disconnect(self, websocket: WebSocket):
        with self._lock:
            self.active_connections.discard(websocket)

    def broadcast_status(self, status: str):
        msg = json.dumps({"type": "status", "data": status})
        # 2. FIX: Use run_coroutine_threadsafe to schedule from Sim Thread to Main Loop
        asyncio.run_coroutine_threadsafe(self._send_to_all(msg), self.loop)

    def broadcast_screen(self, active: bool, content: str = ""):
        payload = {
            "type": "screen_update",
            "data": {
                "active" : active,
                "content" : content
            }
        }
        # 2. FIX: Use run_coroutine_threadsafe here as well
        asyncio.run_coroutine_threadsafe(self._send_to_all(json.dumps(payload)), self.loop)

    async def _send_to_all(self, message: str):
        with self._lock:
            current_sockets = list(self.active_connections)
        for ws in current_sockets:
            try:
                await ws.send_text(message)
            except:
                self.disconnect(ws)

class WebSocketNotifier(NotificationPort):
    def __init__(self, connection_manager: ConnectionManager, loop: asyncio.AbstractEventLoop):
        self.manager = connection_manager
        self.loop = loop

    # 3. FIX: Removed _schedule wrapper. 
    # Calling self.manager.broadcast_*() is now fire-and-forget and thread-safe.

    def notify_status(self, status):
        if status == "active": 
            main_text = "I'm awake!" 
            sub_text = "Ready to assist you." 
        elif status == "sleep": 
            main_text = "Going to sleep." 
            sub_text = "See you later!" 
        elif status == "shutdown": 
            main_text = "Shutting down." 
            sub_text = "Goodbye!" 
        html = ( 
            f"<h1 class='hero-title' style='font-size:3.5rem'>{main_text}</h1>" 
            f"<div class='subtitle'>{sub_text or ''}</div>")
        # 5. FIX: Call manager directly (no _schedule wrapping)
        self.manager.broadcast_screen(active=True, content=html)

    def notify_content(self, content: str):
        pass

    def notify_image(self, image_url: str):
        html = f"<img src='{image_url}' class='circle-fit-img' />"
        # 5. FIX: Call manager directly
        self.manager.broadcast_screen(active=True, content=html)

    def notify_clear_display(self):
        # 5. FIX: Call manager directly
        self.manager.broadcast_screen(active=False)


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
        # 1. Initialize Event Bus with the MAIN LOOP
        loop = asyncio.get_running_loop()
        
        # 2. Setup Connection Manager (Pass loop here)
        manager = ConnectionManager(loop=loop)
        app.state.connection_manager = manager
        notifier = WebSocketNotifier(manager, loop=loop)

        # 4. Instantiate Adapters
        llm_adapter = OllamaAdapter()
        robot_adapter = None

        if args.sim:
            # Inject event_bus into Robot
            robot_adapter = MujocoRobot(loop=loop)
            sim_thread = threading.Thread(target=robot_adapter.run, daemon=True, name="mujoco_sim_thread")
            sim_thread.start()
            robot_adapter.wait_until_ready(timeout=10)
            app.state.sim_thread = sim_thread
            
            # These listeners are called from the Sim Thread. 
            # Manager now handles thread-safety internally.
            robot_adapter.status_update_event.add_listener(manager.broadcast_status)
            robot_adapter.status_update_event.add_listener(notifier.notify_status)
            
        else:
            from mind.ports.base_robot_controller_port import BaseRobotController
            class DummyRobot(BaseRobotController):
                def __init__(self, bus=None): self.bus = bus
                def status(self): return "shutdown"
                def wake_up(self): pass
                def sleep(self): pass
                def shutdown(self): pass 
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
                # 6. FIX: Increased timeout to reduce chance of SegFault during cleanup
                app.state.sim_thread.join(timeout=5.0)
        
        manager.broadcast_status("shutdown")

    app = FastAPI(lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/static")), name="static")
    templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/templates"))

    @app.get("/")
    def root(request: Request):
        return templates.TemplateResponse("index.html", {"request": request})

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        manager = app.state.connection_manager
        agent = app.state.agent
        
        await manager.connect(websocket)
        
        try:
            initial_status = RobotStatus.SHUTDOWN.value
            if agent.robot_controller:
                initial_status = agent.robot_controller.status()
            manager.broadcast_status(initial_status)
            while True:
                data = await websocket.receive_text()
                try:
                    msg = json.loads(data)
                except json.JSONDecodeError:
                    continue

                msg_type = msg.get("type")

                if msg_type == "text":
                    user_text = msg["data"]

                    response_handler = agent.handle_input(user_text)
                    if hasattr(response_handler, 'astream'):
                        async for chunk in agent.response_handler.astream(user_text):
                            await websocket.send_text(json.dumps({"type": "chunk", "text": chunk}))
                            await asyncio.sleep(0.01)

                    else:
                        full = response_handler.get_full_response()
                        await websocket.send_text(json.dumps({"type": "response", "text": full}))   

                elif msg_type == "power":
                    action = msg["data"]["action"]
                    # We only call the agent. We DO NOT broadcast here. 
                    # We wait for the Event listener to trigger broadcast.
                    agent.handle_power_command(action)

        except WebSocketDisconnect:
            manager.disconnect(websocket)
        except Exception as e:
            logger.error(f"WS Error: {e}")
            manager.disconnect(websocket)

    return app

# ... run_app and main remain same ...
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