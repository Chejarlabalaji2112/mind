import threading
import sys
import time
from random import randint, random, choice
from enum import Enum

# Kivy Imports
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.graphics import Color, RoundedRectangle, Triangle, Rectangle
from kivy.properties import ListProperty, BooleanProperty
from kivy.core.window import Window

import cv2
from deepface import DeepFace
from mind.utils.logging_handler import setup_logger

logger = setup_logger(__name__)

# --- 1. ENUMS ---

class Mood(Enum):
    DEFAULT = 0; TIRED = 1; ANGRY = 2; HAPPY = 3

class PosDir(Enum):
    CENTER = 0; N = 1; NE = 2; E = 3; SE = 4; S = 5; SW = 6; W = 7; NW = 8

class RoboEyes(Widget):
    # Colors
    primary_color = ListProperty([0.25, 0.88, 0.82, 1]) # Turquoise
    bg_color = ListProperty([0, 0, 0, 1])         # Dark Gray
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.frame_rate = 60.0
        
        # --- Geometry Settings (C++ Defaults) ---
        self.SCREEN_W = 128
        self.SCREEN_H = 64
        self.def_eye_w = 36
        self.def_eye_h = 36
        self.def_space = 10
        self.radius = 8
        
        # --- State Variables (Target vs Current) ---
        # All logic coordinates are Top-Left Origin (Y=0 is top)
        self.scale_factor = 1.0
        
        # Current Physics State
        self.cur_lx = 0; self.cur_ly = 0
        self.cur_rx = 0; self.cur_ry = 0
        self.cur_lw = self.def_eye_w; self.cur_lh = 1 # Start closed
        self.cur_rw = self.def_eye_w; self.cur_rh = 1
        self.cur_space = self.def_space
        
        # Eyelid States
        
        self.cur_lid_tired = 0
        self.cur_lid_angry = 0
        self.cur_lid_happy = 0 # This is the bottom offset
        
        # Targets
        self.tgt_lx = 0; self.tgt_ly = 0
        self.tgt_lw = self.def_eye_w; self.tgt_rw = self.def_eye_w
        self.tgt_lh = self.def_eye_h; self.tgt_rh = self.def_eye_h
        self.tgt_space = self.def_space
        
        self.tgt_lid_tired = 0
        self.tgt_lid_angry = 0
        self.tgt_lid_happy = 0

        # Flags & Modes
        self.is_cyclops = False
        self.is_curious = False
        self.is_idle = False
        self.eye_l_open = True
        self.eye_r_open = True
        
        # Timers
        self.timer_blink = 0
        self.timer_idle = 0
        
        # --- Macro Animations (Sweat, Laugh, Confused) ---
        self.anim_sweat = False
        self.sweat_drops = [] # List of dicts for drops
        
        self.anim_confused = False
        self.timer_confused = 0
        self.toggle_confused = True
        
        self.anim_laugh = False
        self.timer_laugh = 0
        self.toggle_laugh = True
        
        self.flicker_h = False
        self.flicker_v = False
        self.flicker_amp_h = 0
        self.flicker_amp_v = 0
        
        # Init Sweat Drops
        self._init_sweat()
        
        # Bindings
        self.bind(size=self._calc_scale, pos=self._calc_scale)
        Clock.schedule_interval(self.update, 1.0 / self.frame_rate)
        
        # Initial Position
        self.center_eyes()

    def _init_sweat(self):
        # Initialize 3 drops
        self.sweat_drops = [
            {'x': 0, 'y': 2, 'y_max': 20, 'w': 1, 'h': 2, 'x_init': 0, 'active': False},
            {'x': 0, 'y': 2, 'y_max': 20, 'w': 1, 'h': 2, 'x_init': 0, 'active': False},
            {'x': 0, 'y': 2, 'y_max': 20, 'w': 1, 'h': 2, 'x_init': 0, 'active': False}
        ]

    def _calc_scale(self, *args):
        if self.width == 0 or self.height == 0: return
        # Fit 128x64 into the window while maintaining aspect ratio
        s_w = self.width / self.SCREEN_W
        s_h = self.height / self.SCREEN_H
        self.scale_factor = min(s_w, s_h) * 0.9 # 0.9 padding

    def update(self, dt):
        t = Clock.get_time()
        
        # 1. Auto Blink
        if t > self.timer_blink:
            self.blink()
            self.timer_blink = t + 3.0 + random()*4.0
            
        # 2. Idle Mode
        if self.is_idle and t > self.timer_idle:
            self._random_pos()
            self.timer_idle = t + 1.0 + random()*3.0
            
        # 3. Confused Animation (Horizontal Shake)
        if self.anim_confused:
            if self.toggle_confused:
                self.flicker_h = True; self.flicker_amp_h = 20
                self.timer_confused = t * 1000
                self.toggle_confused = False
            elif (t*1000) >= self.timer_confused + 500: # 500ms duration
                self.flicker_h = False; self.flicker_amp_h = 0
                self.toggle_confused = True
                self.anim_confused = False

        # 4. Laugh Animation (Vertical Shake)
        if self.anim_laugh:
            if self.toggle_laugh:
                self.flicker_v = True; self.flicker_amp_v = 5
                self.timer_laugh = t * 1000
                self.toggle_laugh = False
            elif (t*1000) >= self.timer_laugh + 500:
                self.flicker_v = False; self.flicker_amp_v = 0
                self.toggle_laugh = True
                self.anim_laugh = False

        # --- PHYSICS STEP (Zeno's Paradox Smoothing) ---
        
        # Curious Mode Offset
        h_off_l = 0; h_off_r = 0
        if self.is_curious:
            if self.tgt_lx <= 10: h_off_l = 8
            if self.tgt_lx >= (self.get_constraint_x() - 10): h_off_r = 8

        # Height Physics
        eff_tgt_lh = (self.tgt_lh + h_off_l) if self.eye_l_open else 1
        eff_tgt_rh = (self.tgt_rh + h_off_r) if self.eye_r_open else 1
        
        # Center eyes vertically while closing (C++ logic: eyeLy += (def-curr)/2)
        # In our explicit variable model, we just smooth the dimensions and coords
        self.cur_lh = (self.cur_lh + eff_tgt_lh) / 2
        self.cur_rh = (self.cur_rh + eff_tgt_rh) / 2
        
        # Width & Space Physics
        self.cur_lw = (self.cur_lw + self.tgt_lw) / 2
        
        if self.is_cyclops:
            self.cur_rw = 0; self.cur_space = 0; self.cur_rh = 0
        else:
            self.cur_rw = (self.cur_rw + self.tgt_rw) / 2
            self.cur_space = (self.cur_space + self.tgt_space) / 2

        # Position Physics
        self.cur_lx = (self.cur_lx + self.tgt_lx) / 2
        self.cur_ly = (self.cur_ly + self.tgt_ly) / 2
        
        # Right Eye Position (Dependent on Left)
        tgt_rx = self.tgt_lx + self.cur_lw + self.cur_space
        self.cur_rx = (self.cur_rx + tgt_rx) / 2
        self.cur_ry = self.cur_ly # Same Y
        
        # Eyelid Physics
        self.cur_lid_tired = (self.cur_lid_tired + self.tgt_lid_tired) / 2
        self.cur_lid_angry = (self.cur_lid_angry + self.tgt_lid_angry) / 2
        self.cur_lid_happy = (self.cur_lid_happy + self.tgt_lid_happy) / 2

        # --- RENDER ---
        self._draw()

    def _draw(self):
        self.canvas.clear()
        s = self.scale_factor
        
        # Center the virtual screen in the window
        off_x = (self.width - (self.SCREEN_W * s)) / 2
        off_y = (self.height - (self.SCREEN_H * s)) / 2
        
        # Apply Flicker Offsets (Instant)
        fx = 0; fy = 0
        if self.flicker_h: fx = self.flicker_amp_h if int(Clock.get_time()*20)%2==0 else -self.flicker_amp_h
        if self.flicker_v: fy = self.flicker_amp_v if int(Clock.get_time()*20)%2==0 else -self.flicker_amp_v
        
        # --- Helper: Map Logic (Top-Left) to Screen (Bottom-Left) ---
        def map_rect(lx, ly, w, h):
            # logic_y is distance from TOP. Kivy y is distance from BOTTOM.
            # Screen Y = OffsetY + ((ScreenHeight - LogicY - Height) * Scale)
            scr_x = off_x + ((lx + fx) * s)
            scr_y = off_y + ((self.SCREEN_H - (ly + fy) - h) * s) 
            return (scr_x, scr_y, w*s, h*s)

        def map_tri(x1, y1, x2, y2, x3, y3):
            # Helper to map 3 points
            def m(x, y):
                return off_x + ((x+fx)*s), off_y + ((self.SCREEN_H - (y+fy))*s)
            rx1, ry1 = m(x1, y1)
            rx2, ry2 = m(x2, y2)
            rx3, ry3 = m(x3, y3)
            return [rx1, ry1, rx2, ry2, rx3, ry3]

        with self.canvas:
            # 1. Background
            Color(*self.bg_color)
            Rectangle(pos=(0,0), size=self.size)
            
            # 2. Main Eyes
            Color(*self.primary_color)
            # Left
            lx, ly, lw, lh = map_rect(self.cur_lx, self.cur_ly, self.cur_lw, self.cur_lh)
            RoundedRectangle(pos=(lx, ly), size=(lw, lh), radius=[self.radius * s])
            # Right
            if not self.is_cyclops:
                rx, ry, rw, rh = map_rect(self.cur_rx, self.cur_ry, self.cur_rw, self.cur_rh)
                RoundedRectangle(pos=(rx, ry), size=(rw, rh), radius=[self.radius * s])

            # 3. Eyelids (Masks in BG Color)
            Color(*self.bg_color)
            
            # --- TIRED (Top Triangle) ---
            if self.cur_lid_tired > 0.5:
                h = self.cur_lid_tired
                # Left
                Triangle(points=map_tri(
                    self.cur_lx, self.cur_ly, # Top Left
                    self.cur_lx + self.cur_lw, self.cur_ly, # Top Right
                    self.cur_lx, self.cur_ly + h # Down Left
                ))
                if not self.is_cyclops:
                    Triangle(points=map_tri(
                        self.cur_rx, self.cur_ry, # Top Left
                        self.cur_rx + self.cur_rw, self.cur_ry, # Top Right
                        self.cur_rx + self.cur_rw, self.cur_ry + h # Down Right (C++ style)
                    ))

            # --- ANGRY (Angled Top Triangle) ---
            if self.cur_lid_angry > 0.5:
                h = self.cur_lid_angry
                # Left (Slopes down to center)
                Triangle(points=map_tri(
                    self.cur_lx, self.cur_ly, 
                    self.cur_lx + self.cur_lw, self.cur_ly,
                    self.cur_lx + self.cur_lw, self.cur_ly + h
                ))
                if not self.is_cyclops:
                    Triangle(points=map_tri(
                        self.cur_rx, self.cur_ry,
                        self.cur_rx + self.cur_rw, self.cur_ry,
                        self.cur_rx, self.cur_ry + h
                    ))

            # --- HAPPY (Bottom Cheek Moving Up) ---
            # C++ Logic: fillRoundRect at (y + height) - offset
            if self.cur_lid_happy > 0.5:
                offset = self.cur_lid_happy
                # We draw a BG colored RoundedRect OVER the bottom of the eye
                # Logic Y of cheek top = (EyeTop + EyeHeight) - Offset
                cheek_y = (self.cur_ly + self.cur_lh) - offset
                
                cx, cy, cw, ch = map_rect(self.cur_lx - 1, cheek_y + 1, self.cur_lw + 2, self.def_eye_h)
                RoundedRectangle(pos=(cx, cy), size=(cw, ch), radius=[self.radius * s])
                
                if not self.is_cyclops:
                    cx, cy, cw, ch = map_rect(self.cur_rx - 1, cheek_y + 1, self.cur_rw + 2, self.def_eye_h)
                    RoundedRectangle(pos=(cx, cy), size=(cw, ch), radius=[self.radius * s])

            # 4. Sweat Animation
            if self.anim_sweat:
                Color(*self.primary_color)
                self._draw_sweat_logic(s, off_x, off_y)

    def _draw_sweat_logic(self, s, off_x, off_y):
        # This matches the C++ random update logic
        # Drop 1 (Left)
        self._update_single_drop(0, 0, 30)
        # Drop 2 (Center)
        self._update_single_drop(1, 30, self.SCREEN_W - 60)
        # Drop 3 (Right)
        self._update_single_drop(2, self.SCREEN_W - 30, 30)
        
        for drop in self.sweat_drops:
             # Render
             # Logic Y is increasing (falling down)
             scr_x = off_x + (drop['x'] * s)
             scr_y = off_y + ((self.SCREEN_H - drop['y'] - drop['h']) * s)
             RoundedRectangle(pos=(scr_x, scr_y), size=(drop['w']*s, drop['h']*s), radius=[2*s])

    def _update_single_drop(self, idx, x_min_base, x_range):
        d = self.sweat_drops[idx]
        if d['y'] <= d['y_max']:
            d['y'] += 0.5 # Fall speed
        else:
            # Reset
            d['x_init'] = x_min_base + random() * x_range
            d['y'] = 2
            d['y_max'] = 10 + random() * 10
            d['w'] = 1
            d['h'] = 2
            
        # Grow/Shrink
        if d['y'] <= d['y_max']/2:
            d['w'] += 0.5; d['h'] += 0.5
        else:
            d['w'] -= 0.1; d['h'] -= 0.5
            
        d['x'] = d['x_init'] - (d['w']/2)

    # --- Public API ---
    def set_mood(self, mood):
        self.tgt_lid_tired = (self.cur_lh/2) if mood == Mood.TIRED else 0
        self.tgt_lid_angry = (self.cur_lh/2) if mood == Mood.ANGRY else 0
        self.tgt_lid_happy = (self.cur_lh/2) if mood == Mood.HAPPY else 0

    def set_pos(self, pos_dir):
        con_x = self.get_constraint_x()
        con_y = self.get_constraint_y()
        
        tx = con_x / 2
        ty = con_y / 2
        
        if pos_dir == PosDir.N: ty = 0
        elif pos_dir == PosDir.NE: tx = con_x; ty = 0
        elif pos_dir == PosDir.E: tx = con_x; ty = con_y/2
        elif pos_dir == PosDir.SE: tx = con_x; ty = con_y
        elif pos_dir == PosDir.S: tx = con_x/2; ty = con_y
        elif pos_dir == PosDir.SW: tx = 0; ty = con_y
        elif pos_dir == PosDir.W: tx = 0; ty = con_y/2
        elif pos_dir == PosDir.NW: tx = 0; ty = 0
        
        self.tgt_lx = tx
        self.tgt_ly = ty

    def center_eyes(self):
        self.set_pos(PosDir.CENTER)

    def get_constraint_x(self):
        if self.is_cyclops: return self.SCREEN_W - self.tgt_lw
        return self.SCREEN_W - self.tgt_lw - self.tgt_space - self.tgt_rw

    def get_constraint_y(self):
        return self.SCREEN_H - self.def_eye_h

    def blink(self):
        self.eye_l_open = False; self.eye_r_open = False
        Clock.schedule_once(lambda dt: setattr(self, 'eye_l_open', True), 0.15)
        Clock.schedule_once(lambda dt: setattr(self, 'eye_r_open', True), 0.15)

    def _random_pos(self):
        self.tgt_lx = randint(0, int(self.get_constraint_x()))
        self.tgt_ly = randint(0, int(self.get_constraint_y()))

    def trigger_anim(self, name):
        if name == 'laugh': self.anim_laugh = True; self.toggle_laugh = True
        if name == 'confused': self.anim_confused = True; self.toggle_confused = True


class EmotionEngine(threading.Thread):
    def __init__(self, app_instance, **kwargs):
        super().__init__(**kwargs)
        self.daemon = True
        self.app = app_instance
        self.running = True
        self.cap = None

    def run(self):
        logger.info("Starting emotion engine")
        
        # --- CV INITIALIZATION ---
        if cv2 is None:
            logger.error("Missing DeepFace/OpenCV; running simulation fallback")
            self._run_simulation()
            return

        try:
            self.cap = cv2.VideoCapture(0)  # Open default webcam
            if not self.cap.isOpened():
                raise IOError("Cannot open webcam (index 0). Check device access.")
            logger.info("Webcam opened successfully")
        except Exception as e:
            logger.error("Critical error during webcam init", exc_info=e)
            self._run_simulation()
            return
            
        # --- CV MAIN LOOP ---
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                logger.warning("Failed to read frame from webcam")
                time.sleep(0.1)
                continue

            try:
                # DeepFace analysis is computationally expensive, run it less often (e.g., every 0.5s)
                # The analysis will detect face, align it, and run the model.
                results = DeepFace.analyze(
                    frame, 
                    actions=['emotion'], 
                    enforce_detection=False, # Allows processing if face is slightly out of view
                    silent=True
                )
                
                # Check if any face was detected
                if results and len(results) > 0:
                    # DeepFace returns a list of results (one per face)
                    emotion = results[0]['dominant_emotion']
                    
                    # Pass the detected emotion to the Kivy UI thread
                    Clock.schedule_once(lambda dt, cmd=emotion: self.app.exe_cmd(cmd), 0)
                
            except ValueError:
                # This often happens if no face is detected in the frame
                # If no face is found, default to 'neutral'
                Clock.schedule_once(lambda dt, cmd='neutral': self.app.exe_cmd(cmd), 0)
            except Exception as e:
                # Handle other potential DeepFace errors
                logger.error("DeepFace analysis error", exc_info=e)

            # Control the frame rate of the CV analysis
            time.sleep(0.5) 

        # --- CV CLEANUP ---
        if self.cap and self.cap.isOpened():
            self.cap.release()
        logger.info("Emotion engine stopped")

    def _run_simulation(self):
        """Fallback simulation if CV fails to initialize."""
        self.emotion_map = ['neutral', 'happy', 'angry', 'tired', 'confused']
        while self.running:
            emotion = choice(self.emotion_map)
            Clock.schedule_once(lambda dt, cmd=emotion: self.app.exe_cmd(cmd), 0)
            time.sleep(1 + random() * 1.5)
        
    def stop(self):
        self.running = False

# --- 4. APP & INPUT WRAPPER ---

class RoboEyesApp(App):
    def build(self):
        self.eyes = RoboEyes()
        Window.bind(on_key_down=self._on_key)
        
        # Start console listener (manual commands)
        self.console_thread = threading.Thread(target=self.console_loop, daemon=True)
        self.console_thread.start()
        
        # Start Emotion Detection Engine (automatic commands)
        # self.emotion_engine = EmotionEngine(self)
        # self.emotion_engine.start()
        
        self.print_help()
        return self.eyes

    def on_stop(self):
        logger.info("Stopping threads")
        # self.emotion_engine.stop()

    def print_help(self):
        logger.info("--- ROBO EYES COMMANDS ---")
        logger.info("MOODS:     happy, angry, tired, neutral")
        logger.info("POSITIONS: n, ne, e, se, s, sw, w, nw, center")
        logger.info("ACTIONS:   blink, laugh, confused, sweat (toggle), cyclops, idle")
        logger.info("KEYS:      1-4 (moods), Arrows (Pos), Space (Blink), C (Cyclops)")

    # --- Unified Command Execution (Called by all input methods) ---
    def exe_cmd(self, cmd):
        e = self.eyes
        
        # Map CV Emotion Output and Console Commands to Eye Animations
        if cmd == 'happy': e.set_mood(Mood.HAPPY)
        elif cmd == 'angry': e.set_mood(Mood.ANGRY)
        elif cmd == 'tired' or cmd == 'sad': e.set_mood(Mood.TIRED) # Map sad CV output
        elif cmd == 'neutral': e.set_mood(Mood.DEFAULT)
        elif cmd == 'confused': e.trigger_anim('confused')
        elif cmd == 'laugh': e.trigger_anim('laugh')
        
        # Manual/Toggle/Position Commands
        elif cmd == 'blink': e.blink()
        elif cmd == 'sweat': e.anim_sweat = not e.anim_sweat
        elif cmd == 'cyclops': e.is_cyclops = not e.is_cyclops; e.center_eyes()
        elif cmd == 'idle': e.is_idle = not e.is_idle
        elif cmd == 'curious': e.is_curious = not e.is_curious
        elif cmd == 'n': e.set_pos(PosDir.N)
        # ... (rest of position commands)
        elif cmd == 'center': e.set_pos(PosDir.CENTER)
        
        elif cmd in ['q', 'quit']: self.stop(); sys.exit()
        # else: print(f">> Executing: {cmd}") # Uncomment for debugging CV output

    def console_loop(self):
        while True:
            try:
                cmd = input(">> ").strip().lower()
                Clock.schedule_once(lambda dt, c=cmd: self.exe_cmd(c))
            except EOFError: break

    def _on_key(self, instance, keyboard, keycode, text, modifiers):
        # Safely handle corrupted events
        if not isinstance(keycode, tuple) or len(keycode) < 2: return True
        k = keycode[1]
        
        if k == 'space': self.exe_cmd('blink')
        elif k == '1': self.exe_cmd('neutral')
        elif k == '2': self.exe_cmd('tired')
        elif k == '3': self.exe_cmd('angry')
        elif k == '4': self.exe_cmd('happy')
        elif k == 'c': self.exe_cmd('cyclops')
        elif k == 'i': self.exe_cmd('idle')
        elif k == 'l': self.exe_cmd('laugh')
        elif k == 'up': self.exe_cmd('n')
        elif k == 'down': self.exe_cmd('s')
        elif k == 'left': self.exe_cmd('w')
        elif k == 'right': self.exe_cmd('e')
        return True

if __name__ == '__main__':
    RoboEyesApp().run()