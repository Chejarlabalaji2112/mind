import os
import json
import html
import time
import asyncio
import argparse
import threading
import webbrowser
from typing import Optional
from urllib import response
from pydantic import BaseModel
from dataclasses import dataclass
from contextlib import asynccontextmanager

import fastapi
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
from mind.ports.notification_port import NotificationPort
from mind.adapters.llm_adapters.llm_without_agnetv1 import OllamaAdapter
from mind.adapters.robot_controller_adapters.mujoco_robot_adapter import MujocoRobot


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



class ScreenUpdater:
    """Infrastructure-specific utility for screen updates: Generates payloads and broadcasts via WebSocket.
    Encapsulates the full 'show on screen' concern (UI templating + delivery).
    """
    def __init__(self, connection_manager: ConnectionManager):
        self.manager = connection_manager
        self._CIRCLE_TEMPLATE = """
        <div class="circle-content">
            {title_html}
            {content_html}
            {bottom_html}
        </div>
        """

    def _generate_screen_payload(self, title: Optional[str] = None, content: str = "", bottom: Optional[str] = None) -> str:
        """Internal: Generate HTML payload for a 25%/50%/25% circle layout."""
        # Escape to prevent XSS (infrastructure concern)
        escaped_title = html.escape(title) if title else ""
        escaped_content = html.escape(content) if content else ""
        escaped_bottom = html.escape(bottom) if bottom else ""

        title_html = f'<div class="circle-section top"><h4>{escaped_title}</h4></div>' if title else '<div class="circle-section top"></div>'
        content_html = f'<div class="circle-section middle"><h1>{escaped_content}</h1></div>' if content else '<div class="circle-section middle"></div>'
        bottom_html = f'<div class="circle-section bottom"><div class="subtitle">{escaped_bottom}</div></div>' if bottom else '<div class="circle-section bottom"></div>'

        return self._CIRCLE_TEMPLATE.format(
            title_html=title_html,
            content_html=content_html,
            bottom_html=bottom_html
        )

    def _generate_image_payload(self, image_url: str) -> str:
        """Internal: Specialized payload for images (wrap in middle section)."""
        img_html = f'<img src="{image_url}" class="circle-fit-img" alt="Notification Image" />'
        return self._generate_screen_payload(title=None, content=img_html, bottom=None)

    def _generate_content_payload(self, content: str) -> str:
        """Internal: Simple text-only payload (full middle)."""
        return self._generate_screen_payload(title=None, content=content, bottom=None)

    def show(self, title: Optional[str] = None, content: str = "", bottom: Optional[str] = None):
        """Public: Generate payload and broadcast screen update. Use for general screen notifications."""
        payload = self._generate_screen_payload(title=title, content=content, bottom=bottom)
        self.manager.broadcast_screen(active=True, content=payload)

    def show_image(self, image_url: str):
        """Public: Show image using the circle template (wrapped in middle)."""
        payload = self._generate_image_payload(image_url)
        self.manager.broadcast_screen(active=True, content=payload)

    def show_content(self, content: str):
        """Public: Show text content using the circle template."""
        payload = self._generate_content_payload(content)
        self.manager.broadcast_screen(active=True, content=payload)

    def clear(self):
        """Public: Broadcast clear command (no payload needed)."""
        self.manager.broadcast_screen(active=False)


class Notifier(NotificationPort):
    """Adapter implementing NotificationPort: Broadcasts domain events to WebSocket clients.
    Delegates full screen concerns to ScreenUpdater.
    """
    def __init__(self, manager: ConnectionManager):
        self.screen_updater = ScreenUpdater(manager)  # Inject manager for broadcasting

    def notify_status(self, status: str):
        """Broadcast status event: Delegates to ScreenUpdater.show() for payload gen + broadcast."""
        if status == "active": 
            main_text = "ACTIVE" 
            sub_text = "Ready to assist you." 
        elif status == "sleep": 
            main_text = "SLEEPING" 
            sub_text = "See you later!" 
        elif status == "shutdown": 
            main_text = "OFFLINE" 
            sub_text = "Bye" 
        else:
            main_text = status
            sub_text = "just a sec"
        
        # Delegate fully: Generate + broadcast inside ScreenUpdater
        self.screen_updater.show(title="status", content=main_text, bottom=sub_text)

    def notify_content(self, content: str):
        """Broadcast content: Delegates to ScreenUpdater."""
        self.screen_updater.show_content(content)

    def notify_image(self, image_url: str):
        """Broadcast image: Delegates to ScreenUpdater."""
        self.screen_updater.show_image(image_url)

    def notify_clear_display(self):
        """Broadcast clear command: Delegates to ScreenUpdater."""
        self.screen_updater.clear()

    # Future: Passthrough for direct structured calls from domain (if port evolves)
    def notify_screen(self, title: Optional[str] = None, content: str = "", bottom: Optional[str] = None):
        """Flexible screen notification; delegates to ScreenUpdater."""
        self.screen_updater.show(title=title, content=content, bottom=bottom)


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

        llm_adapter = OllamaAdapter()
        robot_adapter = None

        if args.sim:
            # Inject event_bus into Robot
            robot_adapter = MujocoRobot(loop=loop)
            sim_thread = threading.Thread(target=robot_adapter.run, daemon=True, name="mujoco_sim_thread")
            sim_thread.start()
            robot_adapter.wait_until_ready(timeout=10)
            app.state.sim_thread = sim_thread
            
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
        app.state.connection_manager = manager
        app.state.screen_updater = ScreenUpdater(manager)
        
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