import html
import json
import asyncio
from json import tool
import threading
from typing import Optional
from fastapi import WebSocket
from mind.ports.act_port import Presenter
from mind.ports.notification_port import NotificationPort

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


class FNScreenUpdater(Presenter):
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
        self.screen_updater = FNScreenUpdater(manager)  # Inject manager for broadcasting

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


def output_tuner(tool_name: str, message: str, extra: str = "") -> str:
    """Simple output tuner for formatting tool outputs."""
    output = {"top":{tool_name}, 
              "middle":{message},
              "bottom":{extra}}
    return output

