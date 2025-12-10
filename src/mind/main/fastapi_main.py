import os
import time
import uvicorn
import argparse
import threading
import json
import asyncio
from enum import Enum
from typing import AsyncGenerator
from mind.utils import BASE_DIR             # constants
from dataclasses import dataclass
from mind.core.agent import Agent
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from contextlib import asynccontextmanager
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from mind.utils.logging_handler import setup_logger
from mind.adapters.llm_adapters.llm_without_agnetv1 import OllamaAdapter
from mind.adapters.robot_controller_adapters.mujoco_robot_adapter import MujocoRobot

logger = setup_logger(__name__)

llm = OllamaAdapter()  # Initialize LLM globally

class RobotStatus(Enum):
    SHUTDOWN = "shutdown"
    SLEEP = "sleep"
    ACTIVE = "active"

active_websockets = set()  # Track connected WS for broadcasts
ws_lock = threading.Lock()  # For thread-safe access

@dataclass
class Args:
    """Arguments for configuring the root entry."""
    sim: bool = False
    scene: str = "empty"
    wake_up_on_start: bool = False
    fastapi_host: str = "0.0.0.0"
    fastapi_port: int = 8000
    localhost_only: bool = False

def create_app(args: Args) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        robot_controller = None
        agent = None
        try:
            if args.sim:
                robot_controller = MujocoRobot()
                app.state.robot_controller = robot_controller
                agent = Agent(llm, robot_controller)
                app.state.agent = agent
                # Thread setup for spawning mujoco in a separate thread
                sim_thread = threading.Thread(target=robot_controller.run, daemon=True, name="Mujocorobot_loop")
                app.state.sim_thread = sim_thread
                logger.info("Created the sim robot thread and not started the thread yet.")
                sim_thread.start()
                logger.info("Started the thread and waiting for viewer...")

                if robot_controller.wait_until_ready(timeout=10):
                    logger.info("Viewer is ready now.")
                else:
                    logger.warning("Timed out waiting for viewer.")

            # Initialize state via agent (guarded)
            if args.wake_up_on_start and agent:
                agent.wake_up()  # Delegate to agent

            yield

        finally:
            if robot_controller:
                logger.info("Initiated stop")
                if agent:
                    agent.stop()                  
                if hasattr(app.state, "sim_thread") and app.state.sim_thread.is_alive():
                    logger.info("Waiting for simulation thread to join...")
                    
                    # Wait indefinitely (or with a long timeout) for the thread to close the window
                    app.state.sim_thread.join()
                    
                    logger.info("Simulation thread joined. Viewer closed.")
                # Broadcast final status
                final_status = robot_controller.get_status() if robot_controller else RobotStatus.SHUTDOWN.value
                broadcast_msg = json.dumps({"type": "status", "data": final_status})
                with ws_lock:
                    for ws in list(active_websockets):
                        try:
                            await ws.send_text(broadcast_msg)
                        except Exception as e:
                            logger.debug(f"Failed to broadcast shutdown: {e}")
                            active_websockets.discard(ws)

    app = FastAPI(lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/static")), name="static")
    templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "adapters/fastapi_adapters/templates"))

    @app.get("/")
    def root(request: Request):
        return templates.TemplateResponse("index.html", {"request": request})
    
    @app.get("/favicon.ico")
    def favicon():
        return FileResponse(os.path.join(BASE_DIR, "adapters/fastapi_adapters/static/favicon.ico"))

    # Single WebSocket (updated power handler to call agent methods)
    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()
        with ws_lock:
            active_websockets.add(websocket)
        try:
            # Get agent/robot from state (fallback if no sim)
            agent = getattr(app.state, 'agent', None)
            robot_controller = getattr(app.state, 'robot_controller', None)
            # Send initial status
            initial_status = (robot_controller.get_status() if robot_controller else RobotStatus.SHUTDOWN.value)
            await websocket.send_text(json.dumps({"type": "status", "data": initial_status}))
            
            while True:
                try:
                    data = await websocket.receive()
                    if 'text' in data and data['text'] is not None:
                        msg = json.loads(data['text'])
                    elif 'bytes' in data and data['bytes'] is not None:  # Binary for audio
                        msg = {"type": "audio", "data": data['bytes']}
                    else:
                        continue  #no msg
                    
                    msg_type = msg.get("type")
                    
                    if msg_type == "text":  # Chat
                        async for chunk in stream_llm_response(msg["data"]):
                            await websocket.send_text(chunk)
                            
                    elif msg_type == "audio":  # Placeholder
                        await websocket.send_text(json.dumps({"type": "audio_response", "data": "simulated_audio_base64"}))
                    
                    elif msg_type == "power":  # Delegate to agent
                        action = msg["data"]["action"]
                        if not agent:
                            await websocket.send_text(json.dumps({"type": "error", "data": "Agent not ready"}))
                            continue
                        
                        current_status = robot_controller.get_status() if robot_controller else RobotStatus.SHUTDOWN.value
                        
                        # Allow Wake Up if Shutdown OR Sleep
                        if action == "on" and current_status in [RobotStatus.SHUTDOWN.value, RobotStatus.SLEEP.value]:
                            agent.wake_up()
                        
                        # Allow Shutdown if Active OR Sleep
                        elif action == "shutdown" and current_status in [RobotStatus.ACTIVE.value, RobotStatus.SLEEP.value]:
                            agent.shutdown()
                            
                        # Allow Sleep only if Active
                        elif action == "sleep" and current_status == RobotStatus.ACTIVE.value:
                            agent.sleep()

                        else:
                            # Log the rejection for debugging
                            logger.warning(f"Invalid power transition: Action {action} not allowed in State {current_status}")
                            await websocket.send_text(json.dumps({"type": "error", "data": "Invalid action for current state"}))
                            continue
                        
                        # Get new status and broadcast
                        new_status = robot_controller.get_status() if robot_controller else current_status
                        broadcast_msg = json.dumps({"type": "status", "data": new_status})
                        with ws_lock:
                            for ws in list(active_websockets):
                                try:
                                    await ws.send_text(broadcast_msg)
                                except:
                                    active_websockets.discard(ws)
                
                except RuntimeError:
                    logger.info("got the websocker forcible disconnect error")
                    break
                except WebSocketDisconnect:
                    logger.info("webSocket disconnected")
                    break
                except Exception as e:
                    if "disconnect" in str(e).lower():
                        logger.info("break inside the websocket loop by converting to str")
                        break
                    logger.error(f"ws error: {e}")
                    break
                            
        except WebSocketDisconnect:
            logger.info("WebSocket disconnected")
        finally:
            with ws_lock:
                active_websockets.discard(websocket)


    # Helper: Stream LLM tokens (fixed annotation)

# In stream_llm_response (full function):
    async def stream_llm_response(user_input: str) -> AsyncGenerator[str, None]:
        # Use adapter's stream if available
        try:
            for chunk in llm.stream(user_input):  # Yields AIMessageChunk or str
                text = chunk.content if hasattr(chunk, 'content') else str(chunk)  # Handle both types
                if text:  # Skip empty
                    yield json.dumps({"type": "chunk", "text": text}) + "\n"
                    await asyncio.sleep(0.05)  # Throttle for smooth UI
        except AttributeError:
            # Fallback to chunked non-streaming
            full_response = llm.handle_input(user_input)
            for chunk in [full_response[i:i+50] for i in range(0, len(full_response), 50)]:
                yield json.dumps({"type": "chunk", "text": chunk}) + "\n"
                await asyncio.sleep(0.1)

    return app
    

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
    main()