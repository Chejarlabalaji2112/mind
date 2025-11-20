# main.py

from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.core.window import Window
from kivy.graphics import Color
from roboeyes_widget import RoboEyesWidget, TIRED, ANGRY, HAPPY, N, NE, E, SE, S, SW, W, NW, DEFAULT

class RoboEyesTestApp(App):
    def build(self):
        # Set window size for a good display representation
        Window.size = (400, 600)
        
        main_layout = BoxLayout(orientation='vertical', padding=10, spacing=10)
        
        # 1. RoboEyes Display Area
        # We enforce a 2:1 aspect ratio to mimic a 128x64 screen
        display_container = BoxLayout(size_hint_y=0.4)
        self.eyes = RoboEyesWidget(size_hint=(None, None), size=(380, 190))
        
        # Center the eyes widget in the container
        display_container.bind(size=lambda *args: self.eyes.setter('pos')(self.eyes, (
            display_container.center_x - self.eyes.width / 2,
            display_container.center_y - self.eyes.height / 2
        )))
        display_container.add_widget(self.eyes)
        main_layout.add_widget(display_container)
        
        # 2. Controls Area
        controls = BoxLayout(orientation='vertical', size_hint_y=0.6, spacing=5)

        # --- Basic Actions ---
        controls.add_widget(Label(text="Basic Actions", size_hint_y=None, height=20))
        action_row = BoxLayout(size_hint_y=None, height=40, spacing=5)
        action_row.add_widget(Button(text="Blink", on_press=lambda x: self.eyes.blink()))
        action_row.add_widget(Button(text="Confused", on_press=lambda x: self.eyes.anim_confused()))
        action_row.add_widget(Button(text="Laugh", on_press=lambda x: self.eyes.anim_laugh()))
        controls.add_widget(action_row)

        # --- Mood Controls ---
        controls.add_widget(Label(text="Mood Controls", size_hint_y=None, height=20))
        mood_row = BoxLayout(size_hint_y=None, height=40, spacing=5)
        mood_row.add_widget(Button(text="Default", on_press=lambda x: self.eyes.set_mood(DEFAULT)))
        mood_row.add_widget(Button(text="Tired", on_press=lambda x: self.eyes.set_mood(TIRED)))
        mood_row.add_widget(Button(text="Angry", on_press=lambda x: self.eyes.set_mood(ANGRY)))
        mood_row.add_widget(Button(text="Happy", on_press=lambda x: self.eyes.set_mood(HAPPY)))
        controls.add_widget(mood_row)
        
        # --- Mode Toggles ---
        controls.add_widget(Label(text="Mode Toggles", size_hint_y=None, height=20))
        mode_row = BoxLayout(size_hint_y=None, height=40, spacing=5)
        mode_row.add_widget(Button(text="Auto Blink Toggle", on_press=self.toggle_autoblink))
        mode_row.add_widget(Button(text="Idle Mode Toggle", on_press=self.toggle_idle))
        mode_row.add_widget(Button(text="Sweat Toggle", on_press=self.toggle_sweat))
        mode_row.add_widget(Button(text="Cyclops Toggle", on_press=self.toggle_cyclops))
        controls.add_widget(mode_row)
        
        # --- Position Controls ---
        controls.add_widget(Label(text="Gaze Position", size_hint_y=None, height=20))
        pos_grid = BoxLayout(size_hint_y=None, height=120, orientation='vertical', spacing=2)
        
        pos_grid.add_widget(self._create_pos_row([("NW", NW), ("N", N), ("NE", NE)]))
        pos_grid.add_widget(self._create_pos_row([("W", W), ("CENTER", DEFAULT), ("E", E)]))
        pos_grid.add_widget(self._create_pos_row([("SW", SW), ("S", S), ("SE", SE)]))
        
        controls.add_widget(pos_grid)

        main_layout.add_widget(controls)
        return main_layout
        
    def _create_pos_row(self, positions):
        """Helper to create a row of position buttons."""
        row = BoxLayout(spacing=2)
        for text, pos in positions:
            row.add_widget(Button(text=text, font_size='12sp', on_press=lambda btn, p=pos: self.eyes.set_position(p)))
        return row

    def toggle_autoblink(self, instance):
        self.eyes.autoblinker = not self.eyes.autoblinker
        instance.text = f"Auto Blink {'ON' if self.eyes.autoblinker else 'OFF'}"
        if self.eyes.autoblinker:
            self.eyes.set_autoblinker(True, 2, 3) # Set default interval/variation

    def toggle_idle(self, instance):
        self.eyes.idle = not self.eyes.idle
        instance.text = f"Idle Mode {'ON' if self.eyes.idle else 'OFF'}"
        if self.eyes.idle:
            self.eyes.set_idle_mode(True, 3, 5)

    def toggle_sweat(self, instance):
        self.eyes.sweat = not self.eyes.sweat
        instance.text = f"Sweat {'ON' if self.eyes.sweat else 'OFF'}"
        
    def toggle_cyclops(self, instance):
        self.eyes.set_cyclops(not self.eyes.cyclops)
        instance.text = f"Cyclops {'ON' if self.eyes.cyclops else 'OFF'}"


if __name__ == '__main__':
    RoboEyesTestApp().run()