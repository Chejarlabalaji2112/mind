import webview
import threading
import time
import sys
import os

class AdapterAPI:
    def __init__(self):
        self._window = None

    def set_window(self, window):
        self._window = window

    def receive_from_ui(self, data):
        """
        Callback from JS: window.pywebview.api.receive_from_ui(data)
        """
        command = data.get('command')
        print(f"Received from UI: {command}")

        if command == 'mode_eyes':
            # Example: In the future, this would trigger the immersive overlay
            print("Switching Core Logic to EYES mode...")
            # For now, let's just log it. 
            # In next steps, we will implement the overlay trigger.

def start_app():
    api = AdapterAPI()
    
    # Get absolute path to assets for production stability
    base_dir = os.path.dirname(os.path.abspath(__file__))
    html_path = os.path.join(base_dir, 'assets', 'index.html')

    # Create the window
    window = webview.create_window(
        title='Hitomi Assistant',
        url=html_path,
        width=1000,
        height=700,
        resizable=True,
        js_api=api,
        # transparent=True, # Uncomment if you want rounded corners on Windows 11/Mac
        background_color='#f4f6f8'
    )
    
    api.set_window(window)
    webview.start(gui='qt', debug=True) # debug=True allows Right Click -> Inspect Element

if __name__ == '__main__':
    start_app()
