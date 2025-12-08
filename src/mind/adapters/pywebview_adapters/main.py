import webview
import threading
import time
import os
import json
import sys
from mind.utils.logging_handler import setup_logger

from mind.tools import Stopwatch
from mind.tools import Pomodoro
from mind.tools import Timer

logger = setup_logger(__name__)

class AdapterAPI:
    def __init__(self):
        self._window = None
        
        # --- INSTANTIATE TOOLS ---
        # We assume standard threaded loop for now (simpler than asyncio for pywebview basic) #TODO: Revisit about threading vs asyncio
        self.stopwatch = Stopwatch()
        self.pomodoro = Pomodoro()

        # --- BIND EVENTS ---
        # Connect Python Tool Events -> UI Sender Methods
        self.stopwatch.on_tick.add_listener(self._sync_stopwatch_tick)
        self.stopwatch.on_lap.add_listener(self._sync_stopwatch_lap)
        
        self.pomodoro.on_tick.add_listener(self._sync_pomodoro_tick)
        self.pomodoro.on_phase_end.add_listener(self._sync_pomodoro_phase)

    def set_window(self, window):
        self._window = window

    # --- SENDER (Python -> JS) ---
    def send_event(self, event_name, data):
        """Generic method to send JSON events to the generic JS bridge"""
        if self._window:
            # We wrap the JS call safely
            json_data = json.dumps(data)
            js = f'if(window.app && app.bridge) app.bridge.receive("{event_name}", {json_data})'
            self._window.evaluate_js(js)

    # --- EVENT LISTENERS (Tool Specific) ---
    def _sync_stopwatch_tick(self, elapsed_time, elapsed_time_formatted, **kwargs):
        self.send_event("stopwatch_tick", {"formatted_time": elapsed_time_formatted})

    def _sync_stopwatch_lap(self, lap_time_formatted, **kwargs):
        self.send_event("stopwatch_lap", {"formatted_time": lap_time_formatted})

    def _sync_pomodoro_tick(self, remaining_time, phase, remaining_time_formatted, **kwargs):
        self.send_event("pomodoro_tick", {
            "formatted_time": remaining_time_formatted,
            "phase": phase
        })

    def _sync_pomodoro_phase(self, previous_phase, current_cycle, **kwargs):
        # Optional: Send a notification or sound here
        logger.info("Pomodoro phase ended", extra={"previous_phase": previous_phase, "cycle": current_cycle})

    # --- RECEIVER (JS -> Python) ---
    def receive_from_ui(self, data):
        command = data.get('command')
        payload = data.get('payload', {})
        logger.debug("Received command from UI", extra={"command": command, "payload": payload})

        # --- ROUTER ---
        
        # 1. STOPWATCH
        if command == 'stopwatch_action':
            action = payload.get('action')
            if action == 'start': self.stopwatch.start()
            if action == 'pause': self.stopwatch.pause()
            if action == 'reset': self.stopwatch.reset()
            if action == 'lap':   self.stopwatch.lap()

        # 2. POMODORO
        elif command == 'pomodoro_action':
            action = payload.get('action')
            if action == 'start': self.pomodoro.start()
            if action == 'pause': self.pomodoro.pause()
            if action == 'reset': self.pomodoro.reset()

        # 3. CHAT (Existing)
        elif command == 'chat_query':
            user_text = payload.get('prompt')
            t = threading.Thread(target=self._process_chat, args=(user_text,))
            t.start()

        # 4. NAVIGATION LOGGING
        elif command == 'navigate':
            # You could pause background tools here if needed
            pass

    def _process_chat(self, user_text):
        """Mock AI processing"""
        time.sleep(1)
        response = f"I processed: {user_text}"
        
        # Integration Example: Text command controlling tools
        if "start stopwatch" in user_text.lower():
            self.stopwatch.start()
            response = "I have started the stopwatch for you."
            self.send_event('navigate', {'view': 'stopwatch'}) # Auto-navigate

        self.send_event('ai_response', {'text': response})

def start_app():
    api = AdapterAPI()
    
    # Path Setup
    base_dir = os.path.dirname(os.path.abspath(__file__))
    html_path = os.path.join(base_dir, 'assets', 'index.html')

    if not os.path.exists(html_path):
        logger.error("UI HTML not found", extra={"path": html_path})
        return

    window = webview.create_window(
        title='HITOMI Interface',
        url=html_path,
        width=1200,
        height=800,
        resizable=True,
        js_api=api,
        background_color='#000000'
    )
    
    api.set_window(window)
    webview.start(debug=True)

if __name__ == '__main__':
    start_app()