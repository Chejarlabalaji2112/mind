import webview
import threading
import time
import sys
import os
import json

class AdapterAPI:
    def __init__(self):
        self._window = None

    def set_window(self, window):
        self._window = window

    def _send_to_ui(self, action, data):
        """Helper to call the global python_execute function in JS."""
        if self._window:
            data_json = json.dumps(data)
            self._window.evaluate_js(f'python_execute("{action}", {data_json})')

    def receive_from_ui(self, data):
        command = data.get('command')
        payload = data.get('payload', {})
        print(f"Received from UI: {command} with payload: {payload}")

        if command == 'mode_eyes':
            # This logic is now handled in JS router, but we can do extra stuff here
            print("Eyes Mode Activated")
        
        elif command == 'chat_query':
            user_text = payload.get('prompt')
            t = threading.Thread(target=self._process_chat, args=(user_text,))
            t.start()

    def _process_chat(self, user_text):
        time.sleep(1.5) # Fake thinking time
        
        # DEMO: If user says "angry", make eyes angry
        if "angry" in user_text.lower():
             self._send_to_ui('navigate', {'view': 'eyes'})
             self._send_to_ui('eyes_mood', {'mood': 'ANGRY'})
             return

        if "happy" in user_text.lower():
             self._send_to_ui('navigate', {'view': 'eyes'})
             self._send_to_ui('eyes_mood', {'mood': 'HAPPY'})
             return

        response_text = f"I received your message: '{user_text}'. I am running on the Python backend."
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