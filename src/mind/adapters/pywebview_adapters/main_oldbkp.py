import webview
import threading
import time
import sys
import os
import json
from mind.utils.logging_handler import setup_logger

logger = setup_logger(__name__)

class AdapterAPI:
    def __init__(self):
        self._window = None

    def set_window(self, window):
        self._window = window

    def _send_to_ui(self, action, data):
        if self._window:
            data_json = json.dumps(data)
            self._window.evaluate_js(f'python_execute("{action}", {data_json})')

    def receive_from_ui(self, data):
        command = data.get('command')
        payload = data.get('payload', {})
        logger.debug("Received command from UI", extra={"command": command, "payload": payload})

        if command == 'chat_query':
            user_text = payload.get('prompt')
            # Start processing in a separate thread so UI doesn't freeze
            t = threading.Thread(target=self._process_chat, args=(user_text,))
            t.start()

    def _process_chat(self, user_text):
        time.sleep(0.5) # Reduced sleep for snappier feel
        
        # Default response text
        response_text = f"I heard you say: '{user_text}'"

        # Logic: Perform actions, but update the response text instead of returning
        if "angry" in user_text.lower():
             self._send_to_ui('navigate', {'view': 'eyes'})
             self._send_to_ui('eyes_mood', {'mood': 'ANGRY'})
             response_text = "I am getting ANGRY now!"

        elif "happy" in user_text.lower():
             self._send_to_ui('navigate', {'view': 'eyes'})
             self._send_to_ui('eyes_mood', {'mood': 'HAPPY'})
             response_text = "I am feeling HAPPY!"
        
        elif "laugh" in user_text.lower():
             self._send_to_ui('navigate', {'view': 'eyes'})
             self._send_to_ui('eyes_anim', {'type': 'laugh'})
             response_text = "Hahaha! That is funny."

        # CRITICAL: This must ALWAYS run to unlock the JS input
        self._send_to_ui('ai_response', {'text': response_text})

def start_app():
    api = AdapterAPI()
    
    base_dir = os.path.dirname(os.path.abspath(__file__))
    html_path = os.path.join(base_dir, 'assets', 'index.html')

    window = webview.create_window(
        title='Hitomi Assistant',
        url=html_path,
        width=1100,
        height=750,
        resizable=True,
        js_api=api,
        background_color='#000000'
    )
    
    api.set_window(window)
    webview.start(debug=True)

if __name__ == '__main__':
    start_app()
