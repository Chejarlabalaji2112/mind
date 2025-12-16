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
from mind.core.agent import Agent
from mind.core.status import RobotStatus

from mind.utils.logging_handler import setup_logger

from mind.adapters.llm_adapters.llm_without_agnetv1 import OllamaAdapter
from mind.adapters.robot_controller_adapters.mujoco_robot_adapter import MujocoRobot
from mind.adapters.fastapi_adapters.helper_adapters import ConnectionManager, FNScreenUpdater, Notifier, output_tuner
from mind.tools.tools_registry.core import Register

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
        llm_adapter = OllamaAdapter()
        robot_adapter = None
        screen_updater = FNScreenUpdater(manager)
        presenters = [screen_updater]
        register = Register(loop=loop)
        registry = register.registry
        app.state.register = register
        app.state.registry = registry


        if args.sim:
            # Inject event_bus into Robot
            robot_adapter = MujocoRobot(loop=loop)
            sim_thread = threading.Thread(target=robot_adapter.run, daemon=True, name="mujoco_sim_thread")
            sim_thread.start()
            app.state.sim_thread = sim_thread
            robot_adapter.wait_until_ready(timeout=10)
            
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
        
        await manager.connect(websocket)
        
        try:
            initial_status = RobotStatus.SHUTDOWN.value
            if agent.robot_controller:
                initial_status = agent.robot_controller.status()
            manager.broadcast_status(initial_status)
            # app.state.screen_updater.show(title="status", content="Activating", bottom='just a sec')
    

            response_handler = agent.input_handler() # A single instance per connection.

            while True:
                data = await websocket.receive_text()
                try:
                    msg = json.loads(data)
                except json.JSONDecodeError:
                    continue

                msg_type = msg.get("type")

                if msg_type == "text":
                    user_text = msg["data"]  
                    
                    if hasattr(response_handler, 'astream'):
                        async for chunk in response_handler.astream(user_text):
                            await websocket.send_text(json.dumps({"type": "chunk", "text": chunk}))
                            await asyncio.sleep(0.01)

                    else:
                        full = response_handler.get_full_response()
                        await websocket.send_text(json.dumps({"type": "response", "text": full}))   

                elif msg_type == "power":
                    action = msg["data"]["action"]
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