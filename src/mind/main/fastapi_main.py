import os
import json
import time
import asyncio
import threading
import uvicorn
import argparse
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


from mind.utils.resource_monitor import ResourceMonitor
logger = setup_logger(__name__)

# --- CONNECTION MANAGER (As defined in previous fix) ---
class ConnectionManager:
    def __init__(self):
        self.active_connections: set[WebSocket] = set()
        self._lock = threading.Lock()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        with self._lock:
            self.active_connections.add(websocket)

    def disconnect(self, websocket: WebSocket):
        with self._lock:
            self.active_connections.discard(websocket)

    async def broadcast_status(self, status: str):
        msg = json.dumps({"type": "status", "data": status})
        with self._lock:
            current_sockets = list(self.active_connections)
        for ws in current_sockets:
            try:
                await ws.send_text(msg)
            except:
                self.disconnect(ws)

@dataclass
class Args:
    """Arguments for configuring the root entry."""
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
        # monitor = ResourceMonitor(interval=5.0)
        # monitor.start()
        # 1. Instantiate Adapters (Driven)
        # Note: No global variable. We create it here.
        
        llm_adapter = OllamaAdapter()
        robot_adapter =  None

        if args.sim:
            robot_adapter = MujocoRobot()
            # Start Sim Thread
            sim_thread = threading.Thread(target=robot_adapter.run, daemon=True, name="mujoco_sim_thread")
            sim_thread.start()
            robot_adapter.wait_until_ready(timeout=10)
            app.state.sim_thread = sim_thread

        else:
            # Handle the case where there is no robot
            from mind.ports.base_robot_controller_port import BaseRobotController
            class DummyRobot(BaseRobotController):
                def status(self): return "shutdown"
                def wake_up(self): pass
                def sleep(self): pass
                def shutdown(self): pass
                def stop(self): pass

            robot_adapter = DummyRobot()

        # 2. Instantiate Core (Hexagon) and Inject Adapters
        agent = Agent(decision_maker=llm_adapter, robot_controller=robot_adapter)
        
        # 3. Store in State for Dependency Injection in Endpoints
        app.state.agent = agent
        app.state.connection_manager = ConnectionManager()
        
        if args.wake_up_on_start and robot_adapter:
            agent.wake_up()

        yield

        # 4. Cleanup
        # monitor.stop()
        
        if robot_adapter:
            agent.stop() # Agent stops the robot
            if hasattr(app.state, 'sim_thread'):
                logger.debug("need the sim_thread to join")
                app.state.sim_thread.join(timeout=2)
                

        if hasattr(app.state, "connection_manager"):
            # We explicitly send "shutdown" because the server is dying.
            await app.state.connection_manager.broadcast_status("shutdown")



    app = FastAPI(lifespan=lifespan)
    
    # Mounts
    app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/static")), name="static")
    templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/templates"))

    @app.get("/")
    def root(request: Request):
        return templates.TemplateResponse("index.html", {"request": request})

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        manager = app.state.connection_manager
        agent = app.state.agent  # Retrieve the Agent instance
        
        await manager.connect(websocket)
        
        try:
            # Send initial status
            initial_status = RobotStatus.SHUTDOWN.value
            if agent.robot_controller:
                initial_status = agent.robot_controller.status()
            await websocket.send_text(json.dumps({"type": "status", "data": initial_status}))

            while True:
                data = await websocket.receive_text()
                
                try:
                    msg = json.loads(data)
                except json.JSONDecodeError:
                    continue

                msg_type = msg.get("type")

                # --- CASE 1: CHAT ---
                if msg_type == "text":
                    user_text = msg["data"]
                    
                    # VIOLATION FIXED: We call agent.handle_input_stream, NOT llm.stream
                    async for chunk in agent.handle_input_stream(user_text):
                        # Send chunks immediately to frontend
                        response_packet = json.dumps({"type": "chunk", "text": chunk})
                        await websocket.send_text(response_packet)
                        # Minimal throttle to prevent network flooding if local LLM is too fast
                        await asyncio.sleep(0.01)

                # --- CASE 2: POWER ---
                elif msg_type == "power":
                    action = msg["data"]["action"]
                    # VIOLATION FIXED: Logic moved to Agent
                    new_status = agent.handle_power_command(action)
                    await manager.broadcast_status(new_status.value)

        except WebSocketDisconnect:
            manager.disconnect(websocket)
        except Exception as e:
            logger.error(f"WS Error: {e}")
            manager.disconnect(websocket)

    return app

# ... run_app and main remain mostly the same ...

def run_app(args: Args) -> None:
    """Run the FastAPI app with Uvicorn."""
    app = create_app(args)
    try:
        uvicorn.run(app, host=args.fastapi_host, port=args.fastapi_port)
    except Exception as e:
        logger.error(f"error in run_app: {e}")



def main() -> None:
    """Run the FastAPI app with Uvicorn"""
    default_args = Args()

    parser = argparse.ArgumentParser(description="Main entry of the mind.")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--scene", type=str, default=default_args.scene, help="Name of the scene to load (default: empty)")
    parser.add_argument("--wake-up-on-start", action="store_true", help="Wake up the robot on daemon start (default: False).")

    parser.add_argument("--fastapi-host", type=str, default=default_args.fastapi_host)
    parser.add_argument("--fastapi-port", type=int, default=default_args.fastapi_port)
    parser.add_argument("--localhost-only", action="store_true", help="Restrict the server to localhost only (default: False).")
    parsed_args = parser.parse_args()

    run_app(Args(**vars(parsed_args)))

if __name__ == "__main__":
    logger.info("="*50)
    main()