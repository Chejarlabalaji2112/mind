from kivy.app import App
from kivy.clock import Clock
from kivy.graphics import Color, RoundedRectangle, Triangle, Rectangle
from kivy.uix.widget import Widget
from kivy.core.window import Window
import random
import time




# --- mood constants ---
DEFAULT = 0
TIRED = 1
ANGRY = 2
HAPPY = 3

# --- positions ---
N, NE, E, SE, S, SW, W, NW = 1,2,3,4,5,6,7,8


class RoboEyesWidget(Widget):
    """
    Kivy port of the FluxGarage RoboEyes engine.
    Supports dynamic scaling, moods, blinking, flicker, idle mode,
    laughing, confusion, sweat, and cyclops rendering.
    """

    # === Settings ===
    color_main = (0.2, 0.9, 1.0, 1)      # turquoise-blue
    color_bg = (0, 0, 0, 1)              # background

    # --- animation state flags ---
    tired = angry = happy = False
    curious = False
    cyclops = False
    eyeL_open = False
    eyeR_open = False

    hFlicker = False
    vFlicker = False
    autoblinker = False
    idle = False
    confused = False
    laugh = False
    sweat = False

    # === Constructor ===
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # dynamic geometry (ratios of widget size)
        self.base_eye_w = 0.28      # % of width
        self.base_eye_h = 0.45      # % of height
        self.base_spacing = 0.15    # % of width
        self.base_radius = 0.15     # relative corner radius

        # current tweened values
        self.eyeLw = self.eyeRw = 0
        self.eyeLh = self.eyeRh = 1
        self.eyeLw_t = self.eyeRw_t = 0
        self.eyeLh_t = self.eyeRh_t = self.base_eye_h
        self.eyeLx = self.eyeLy = 0
        self.eyeRx = self.eyeRy = 0
        self.eyeLx_t = self.eyeLy_t = 0
        self.eyeRx_t = self.eyeRy_t = 0
        self.space = self.base_spacing

        # eyelid states
        self.tired_val = 0
        self.angry_val = 0
        self.happy_bottom_val = 0

        # timers
        self.next_blink_t = 0
        self.next_idle_t = 0

        Clock.schedule_interval(self.update, 1/60)
        Window.bind(size=self.handle_resize)
        self.handle_resize(Window, Window.size)


    # ============================================================
    # SCREEN RESIZE → RECALCULATE DEFAULT POSITIONS
    # ============================================================
    def handle_resize(self, *args):
        w, h = self.size

        self.eyeLw_t = w * self.base_eye_w
        self.eyeRw_t = w * self.base_eye_w
        self.eyeLw = self.eyeRw = self.eyeLw_t

        self.eyeLh_t = h * self.base_eye_h
        self.eyeRh_t = h * self.base_eye_h
        self.eyeLh = self.eyeRh = 1  # start closed

        self.space = w * self.base_spacing

        self.center_eyes()


    # ============================================================
    # BASIC SETTERS
    # ============================================================
    def center_eyes(self):
        w, h = self.size

        Lx = w / 2 - self.eyeLw - self.space/2
        Rx = w / 2 + self.space/2
        y = h/2 - self.eyeLh/2

        self.eyeLx = self.eyeLx_t = Lx
        self.eyeRx = self.eyeRx_t = Rx
        self.eyeLy = self.eyeLy_t = y
        self.eyeRy = self.eyeRy_t = y


    def setMood(self, mood):
        self.tired = self.angry = self.happy = False
        if mood == TIRED: self.tired = True
        elif mood == ANGRY: self.angry = True
        elif mood == HAPPY: self.happy = True


    def setPosition(self, pos):
        """Position relative to window."""
        w, h = self.size

        pos_map = {
            N:  (0.5, 1.0),
            NE: (1.0, 1.0),
            E:  (1.0, 0.5),
            SE: (1.0, 0.0),
            S:  (0.5, 0.0),
            SW: (0.0, 0.0),
            W:  (0.0, 0.5),
            NW: (0.0, 1.0),
            DEFAULT: (0.5, 0.5)
        }

        px, py = pos_map.get(pos, pos_map[DEFAULT])

        Lx = px * w - self.eyeLw - self.space/2
        Ly = py * h - self.eyeLh/2

        self.eyeLx_t, self.eyeLy_t = Lx, Ly
        self.eyeRx_t = Lx + self.eyeLw + self.space
        self.eyeRy_t = Ly


    # ============================================================
    # BASIC ANIMATIONS
    # ============================================================
    def blink(self):
        self.eyeLh_t = 1
        self.eyeRh_t = 1
        self.eyeL_open = True
        self.eyeR_open = True

    def open(self):
        self.eyeL_open = self.eyeR_open = True

    def close(self):
        self.eyeLh_t = 1
        self.eyeRh_t = 1
        self.eyeL_open = self.eyeR_open = False


    # ============================================================
    # MACRO ANIMATION TRIGGERS
    # ============================================================
    def anim_confused(self):
        self.confused = True
        self.conf_start = time.time()

    def anim_laugh(self):
        self.laugh = True
        self.laugh_start = time.time()


    # ============================================================
    # MAIN UPDATE LOOP
    # ============================================================
    def update(self, dt):
        w, h = self.size

        # Open eye after blink
        if self.eyeL_open and self.eyeLh <= 1.1:
            self.eyeLh_t = h * self.base_eye_h
        if self.eyeR_open and self.eyeRh <= 1.1:
            self.eyeRh_t = h * self.base_eye_h

        # Tweening
        self.eyeLh = (self.eyeLh + self.eyeLh_t) / 2
        self.eyeRh = (self.eyeRh + self.eyeRh_t) / 2
        self.eyeLw = (self.eyeLw + self.eyeLw_t) / 2
        self.eyeRw = (self.eyeRw + self.eyeRw_t) / 2

        self.eyeLx = (self.eyeLx + self.eyeLx_t) / 2
        self.eyeLy = (self.eyeLy + self.eyeLy_t) / 2
        self.eyeRx = self.eyeLx + self.eyeLw + self.space
        self.eyeRy = self.eyeLy

        # Mood eyelids
        self.tired_val = (self.tired_val + (self.eyeLh/2 if self.tired else 0)) / 2
        self.angry_val = (self.angry_val + (self.eyeLh/2 if self.angry else 0)) / 2
        self.happy_bottom_val = (self.happy_bottom_val + (self.eyeLh/2 if self.happy else 0)) / 2

        # Auto blinking
        now = time.time()
        if self.autoblinker and now >= self.next_blink_t:
            self.blink()
            self.next_blink_t = now + random.uniform(2, 6)

        # Idle wandering
        if self.idle and now >= self.next_idle_t:
            nx = random.uniform(0.1, 0.6) * w
            ny = random.uniform(0.2, 0.7) * h
            self.eyeLx_t = nx
            self.eyeLy_t = ny
            self.next_idle_t = now + random.uniform(1.5, 4)

        # Confused shake
        if self.confused:
            if now - self.conf_start < 0.5:
                self.eyeLx += random.choice([-15, 15])
            else:
                self.confused = False

        # Laugh vertical shake
        if self.laugh:
            if now - self.laugh_start < 0.5:
                self.eyeLy += random.choice([-8, 8])
            else:
                self.laugh = False

        self.draw()


    # ============================================================
    # KIVY DRAWING ENGINE
    # ============================================================
    def draw(self):
        self.canvas.clear()
        with self.canvas:

            # ---- EYEBALLS ----
            Color(*self.color_main)

            # Left eye
            RoundedRectangle(
                pos=(self.eyeLx, self.eyeLy),
                size=(self.eyeLw, self.eyeLh),
                radius=[self.eyeLw * self.base_radius]
            )

            # Right eye
            if not self.cyclops:
                RoundedRectangle(
                    pos=(self.eyeRx, self.eyeRy),
                    size=(self.eyeRw, self.eyeRh),
                    radius=[self.eyeRw * self.base_radius]
                )

            # ---- TIRED eyelids ----
            if self.tired_val > 1:
                Color(*self.color_bg)
                # Left
                Triangle(
                    points=[
                        self.eyeLx, self.eyeLy,
                        self.eyeLx + self.eyeLw, self.eyeLy,
                        self.eyeLx, self.eyeLy + self.tired_val
                    ]
                )
                # Right
                if not self.cyclops:
                    Triangle(
                        points=[
                            self.eyeRx, self.eyeRy,
                            self.eyeRx + self.eyeRw, self.eyeRy,
                            self.eyeRx + self.eyeRw, self.eyeRy + self.tired_val
                        ]
                    )

            # ---- ANGRY eyelids ----
            if self.angry_val > 1:
                Color(*self.color_bg)
                # Left
                Triangle(
                    points=[
                        self.eyeLx, self.eyeLy,
                        self.eyeLx + self.eyeLw, self.eyeLy,
                        self.eyeLx + self.eyeLw, self.eyeLy + self.angry_val
                    ]
                )
                # Right
                if not self.cyclops:
                    Triangle(
                        points=[
                            self.eyeRx, self.eyeRy,
                            self.eyeRx + self.eyeRw, self.eyeRy,
                            self.eyeRx, self.eyeRy + self.angry_val
                        ]
                    )

            # ---- HAPPY bottom eyelids ----
            if self.happy_bottom_val > 1:
                Color(*self.color_bg)
                RoundedRectangle(
                    pos=(self.eyeLx, self.eyeLy + self.eyeLh - self.happy_bottom_val),
                    size=(self.eyeLw, self.eyeLh),
                    radius=[self.eyeLw * self.base_radius]
                )
                if not self.cyclops:
                    RoundedRectangle(
                        pos=(self.eyeRx, self.eyeRy + self.eyeRh - self.happy_bottom_val),
                        size=(self.eyeRw, self.eyeRh),
                        radius=[self.eyeRw * self.base_radius]
                    )


class RoboEyesApp(App):
    def build(self):
        return RoboEyesWidget()


if __name__ == "__main__":
    RoboEyesApp().run()
