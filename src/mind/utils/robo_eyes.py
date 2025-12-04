import time
import random
from PIL import Image, ImageDraw

# --- Constants ---
# Moods
DEFAULT = 0
TIRED = 1
ANGRY = 2
HAPPY = 3

# Positions
N = 1  # North
NE = 2  # North East
E = 3  # East
SE = 4  # South East
S = 5  # South
SW = 6  # South West
W = 7  # West
NW = 8  # North West
CENTER = 0




class RoboEyes:
    def __init__(self, screen_updater, width=128, height=64, framerate=30, active=False):
        self.is_active = active
        self.screen_updater = screen_updater
        self.width = width
        self.height = height
        self.framerate = framerate
        self.frame_interval = 1.0 / framerate
        self.last_update = time.time()

        # Reference resolution (from original C++ code) used for scaling logic
        self.REF_W = 128.0
        self.REF_H = 64.0

        # Calculate scaling factors
        self.scale_x = self.width / self.REF_W
        self.scale_y = self.height / self.REF_H

        # Colors (0=Black, 255=White for monochrome, or tuples for RGB)
        self.col_bg = 0
        self.col_main = (255, 255, 0) # screen_updater expects bgr always and converts it into rgb. so here we provide bgr here it will result as rgb.

        # Geometry Defaults (Reference Scale)
        self.default_eye_w = 36
        self.default_eye_h = 36
        self.default_space = 10
        self.default_radius = 8

        # State Variables (Current)
        self.curr = {
            'eyeL_w': self.default_eye_w,
            'eyeL_h': 1,  # Start closed
            'eyeL_x': 0,
            'eyeL_y': 0,
            'eyeL_radius': self.default_radius,
            'eyeR_w': self.default_eye_w,
            'eyeR_h': 1,  # Start closed
            'eyeR_x': 0,
            'eyeR_y': 0,
            'eyeR_radius': self.default_radius,
            'space': self.default_space,
            'lid_tired': 0,
            'lid_angry': 0,
            'lid_happy_offset': 0,
            'eyeL_height_offset': 0,
            'eyeR_height_offset': 0
        }

        # Target Variables (Next)
        self.next = {
            'eyeL_w': self.default_eye_w,
            'eyeL_h': self.default_eye_h,
            'eyeL_x': 0,
            'eyeL_y': 0,
            'eyeL_radius': self.default_radius,
            'eyeR_w': self.default_eye_w,
            'eyeR_h': self.default_eye_h,
            'eyeR_x': 0,
            'eyeR_y': 0,
            'eyeR_radius': self.default_radius,
            'space': self.default_space,
            'lid_tired': 0,
            'lid_angry': 0,
            'lid_happy_offset': 0,
            'eyeL_height_offset': 0,
            'eyeR_height_offset': 0
        }

        # Flags and Modes
        self.mood = DEFAULT
        self.curious = False
        self.cyclops = False
        self.sweat = False
        self.h_flicker = False
        self.v_flicker = False
        self.h_flicker_amp = 2
        self.v_flicker_amp = 10
        self.eyeL_open = False
        self.eyeR_open = False

        # Timers
        self.blink_timer = 0
        self.blink_interval = 2.0
        self.blink_var = 1.0
        self.idle_timer = 0
        self.idle_interval = 2.0
        self.idle_var = 1.0

        # Animation States
        self.anim_confused = False
        self.anim_laugh = False
        self.anim_timer = 0
        self.anim_toggle = True
        self.confused_duration = 0.5
        self.laugh_duration = 0.5

        # Sweat particles state
        self.sweat_border_radius = 3
        self.sweat_drops = [
            {'x': 2, 'y': 2.0, 'max_y': 0, 'w': 1.0, 'h': 2.0, 'active': True, 'x_initial': 2},
            {'x': 0, 'y': 2.0, 'max_y': 0, 'w': 1.0, 'h': 2.0, 'active': True, 'x_initial': 0},
            {'x': 0, 'y': 2.0, 'max_y': 0, 'w': 1.0, 'h': 2.0, 'active': True, 'x_initial': 0}
        ]
        self.reset_sweat_initials()

        # Initial positioning
        self.set_position(CENTER)

    def begin(self):
        """Startup: Clear display and set initial closed eyes."""
        frame = Image.new("RGB", (self.width, self.height), self.col_bg)
        self.screen_updater.show(frame)
        self.curr['eyeL_h'] = 1
        self.curr['eyeR_h'] = 1

    def set_framerate(self, fps):
        """Set frame rate."""
        self.framerate = fps
        self.frame_interval = 1.0 / fps

    def set_display_colors(self, bg, main):
        """Set colors."""
        self.col_bg = bg
        self.col_main = main

    def set_width(self, left, right):
        """Set eye widths."""
        self.default_eye_w = left  # Assuming symmetric, but store separately if needed
        self.next['eyeL_w'] = left
        self.next['eyeR_w'] = right
        self.curr['eyeL_w'] = left
        self.curr['eyeR_w'] = right

    def set_height(self, left, right):
        """Set eye heights."""
        self.default_eye_h = left  # Symmetric
        self.next['eyeL_h'] = left
        self.next['eyeR_h'] = right
        self.curr['eyeL_h'] = left
        self.curr['eyeR_h'] = right

    def set_border_radius(self, left, right):
        """Set border radii."""
        self.next['eyeL_radius'] = left
        self.next['eyeR_radius'] = right
        self.curr['eyeL_radius'] = left
        self.curr['eyeR_radius'] = right
        self.default_radius = left  # Symmetric

    def set_space_between(self, space):
        """Set space between eyes."""
        self.default_space = space
        self.next['space'] = space
        self.curr['space'] = space

    def set_mood(self, mood):
        """Set mood expression."""
        self.mood = mood
        if mood == TIRED:
            self.next['lid_tired'] = self.curr['eyeL_h'] / 2
            self.next['lid_angry'] = 0
            self.next['lid_happy_offset'] = 0
        elif mood == ANGRY:
            self.next['lid_angry'] = self.curr['eyeL_h'] / 2
            self.next['lid_tired'] = 0
            self.next['lid_happy_offset'] = 0
        elif mood == HAPPY:
            self.next['lid_happy_offset'] = self.curr['eyeL_h'] / 2
            self.next['lid_tired'] = 0
            self.next['lid_angry'] = 0
        else:
            self.next['lid_tired'] = 0
            self.next['lid_angry'] = 0
            self.next['lid_happy_offset'] = 0

    def set_position(self, pos):
        """Sets the target X/Y coordinates based on predefined positions."""
        max_x = self.REF_W - self.next['eyeL_w'] - self.next['space'] - self.next['eyeR_w']
        max_y = self.REF_H - self.default_eye_h

        tx, ty = 0, 0

        if pos == N:
            tx, ty = max_x / 2, 0
        elif pos == NE:
            tx, ty = max_x, 0
        elif pos == E:
            tx, ty = max_x, max_y / 2
        elif pos == SE:
            tx, ty = max_x, max_y
        elif pos == S:
            tx, ty = max_x / 2, max_y
        elif pos == SW:
            tx, ty = 0, max_y
        elif pos == W:
            tx, ty = 0, max_y / 2
        elif pos == NW:
            tx, ty = 0, 0
        else:  # CENTER
            tx, ty = max_x / 2, max_y / 2
        self.next['eyeL_x'] = tx
        self.next['eyeL_y'] = ty

    def set_auto_blink(self, active, interval=2, variation=1):
        """Set auto blink."""
        self.auto_blink = active  # Note: was missing in original port
        self.blink_interval = interval
        self.blink_var = variation

    def set_idle_mode(self, active, interval=2, variation=1):
        """Set idle mode."""
        self.idle_mode = active  # Note: was missing in original port
        self.idle_interval = interval
        self.idle_var = variation

    def set_curiosity(self, active):
        """Set curious mode."""
        self.curious = active

    def set_cyclops(self, active):
        """Set cyclops mode."""
        self.cyclops = active

    def set_h_flicker(self, active, amplitude=2):
        """Set horizontal flicker."""
        self.h_flicker = active
        self.h_flicker_amp = amplitude

    def set_v_flicker(self, active, amplitude=10):
        """Set vertical flicker."""
        self.v_flicker = active
        self.v_flicker_amp = amplitude

    def set_sweat(self, active):
        """Set sweat."""
        self.sweat = active

    def close(self, left=True, right=True):
        """Close eye(s)."""
        if left:
            self.next['eyeL_h'] = 1
            self.eyeL_open = False
        if right:
            self.next['eyeR_h'] = 1
            self.eyeR_open = False

    def open(self, left=True, right=True):
        """Flag to open eye(s)."""
        if left:
            self.eyeL_open = True
        if right:
            self.eyeR_open = True

    def blink(self, left=True, right=True):
        """Trigger blink."""
        self.close(left, right)
        self.open(left, right)

    def trigger_confused(self):
        """Trigger confused animation."""
        self.anim_confused = True
        self.anim_timer = time.time()

    def trigger_laugh(self):
        """Trigger laugh animation."""
        self.anim_laugh = True
        self.anim_timer = time.time()

    def reset_sweat_initials(self):
        """Reset sweat drop initial positions (zoned)."""
        self.sweat_drops[0]['x_initial'] = random.randint(0, 30)
        self.sweat_drops[1]['x_initial'] = random.randint(30, int(self.REF_W - 30))
        self.sweat_drops[2]['x_initial'] = random.randint(int(self.REF_W - 30), int(self.REF_W))
        for drop in self.sweat_drops:
            drop['y'] = 2.0
            drop['max_y'] = random.randint(10, 20)
            drop['w'] = 1.0
            drop['h'] = 2.0
            drop['x'] = drop['x_initial'] - (drop['w'] / 2)

    def _tween_value(self, current, target):
        """Tween: (current + target) / 2 for ease-out."""
        return (current + target) / 2.0

    def _get_screen_constraint_x(self):
        """Max X for left eye."""
        return self.REF_W - self.curr['eyeL_w'] - self.curr['space'] - self.curr['eyeR_w']

    def _get_screen_constraint_y(self):
        """Max Y for left eye."""
        return self.REF_H - self.default_eye_h

    def update(self):
        now = time.time()
        if now - self.last_update < self.frame_interval:
            return
        self.last_update = now

        # --- Auto Blink ---
        if hasattr(self, 'auto_blink') and self.auto_blink and now > self.blink_timer:
            self.blink()
            self.blink_timer = now + self.blink_interval + random.uniform(0, self.blink_var)

        # --- Idle Movement ---
        if hasattr(self, 'idle_mode') and self.idle_mode and now > self.idle_timer:
            max_x = self._get_screen_constraint_x()
            max_y = self._get_screen_constraint_y()
            self.next['eyeL_x'] = random.uniform(0, max_x)
            self.next['eyeL_y'] = random.uniform(0, max_y)
            self.idle_timer = now + self.idle_interval + random.uniform(0, self.idle_var)

        # --- Curious Height Offset ---
        self.next['eyeL_height_offset'] = 0
        self.next['eyeR_height_offset'] = 0
        if self.curious:
            if self.next['eyeL_x'] <= 10:
                self.next['eyeL_height_offset'] = 8
            if self.next['eyeR_x'] >= self.REF_W - self.next['eyeR_w'] - 10:
                self.next['eyeR_height_offset'] = 8

        # --- Tween Geometry ---
        self.curr['eyeL_w'] = self._tween_value(self.curr['eyeL_w'], self.next['eyeL_w'])
        self.curr['eyeR_w'] = self._tween_value(self.curr['eyeR_w'], self.next['eyeR_w'])
        self.curr['space'] = self._tween_value(self.curr['space'], self.next['space'])
        self.curr['eyeL_radius'] = self._tween_value(self.curr['eyeL_radius'], self.next['eyeL_radius'])
        self.curr['eyeR_radius'] = self._tween_value(self.curr['eyeR_radius'], self.next['eyeR_radius'])

        # Height with offset
        self.curr['eyeL_h'] = self._tween_value(self.curr['eyeL_h'], self.next['eyeL_h'] + self.next['eyeL_height_offset'])
        self.curr['eyeR_h'] = self._tween_value(self.curr['eyeR_h'], self.next['eyeR_h'] + self.next['eyeR_height_offset'])
        self.curr['eyeL_height_offset'] = self._tween_value(self.curr['eyeL_height_offset'], self.next['eyeL_height_offset'])
        self.curr['eyeR_height_offset'] = self._tween_value(self.curr['eyeR_height_offset'], self.next['eyeR_height_offset'])

        # Positions
        self.curr['eyeL_x'] = self._tween_value(self.curr['eyeL_x'], self.next['eyeL_x'])
        self.curr['eyeL_y'] = self._tween_value(self.curr['eyeL_y'], self.next['eyeL_y'])
        self.curr['eyeR_x'] = self._tween_value(self.curr['eyeR_x'], self.next['eyeL_x'] + self.curr['eyeL_w'] + self.curr['space'])
        self.curr['eyeR_y'] = self.curr['eyeL_y']

        # Y centering on height change and offset
        self.curr['eyeL_y'] += (self.default_eye_h - self.curr['eyeL_h']) / 2
        self.curr['eyeL_y'] -= self.curr['eyeL_height_offset'] / 2
        self.curr['eyeR_y'] += (self.default_eye_h - self.curr['eyeR_h']) / 2
        self.curr['eyeR_y'] -= self.curr['eyeR_height_offset'] / 2

        # Re-open eyes
        if self.eyeL_open and self.curr['eyeL_h'] <= 1 + self.curr['eyeL_height_offset']:
            self.next['eyeL_h'] = self.default_eye_h
        if self.eyeR_open and self.curr['eyeR_h'] <= 1 + self.curr['eyeR_height_offset']:
            self.next['eyeR_h'] = self.default_eye_h

        # Cyclops
        if self.cyclops:
            self.curr['eyeR_w'] = 0
            self.curr['eyeR_h'] = 0
            self.curr['space'] = 0
            self.next['eyeR_w'] = 0
            self.next['eyeR_h'] = 0
            self.next['space'] = 0

        # --- Animations: Confused/Laugh ---
        off_x = 0
        off_y = 0
        if self.anim_confused:
            if now - self.anim_timer > self.confused_duration:
                self.anim_confused = False
                self.h_flicker = False
            else:
                if self.anim_toggle:
                    off_x = self.h_flicker_amp * 2  # Amp for confused
                else:
                    off_x = -self.h_flicker_amp * 2
                self.anim_toggle = not self.anim_toggle
        if self.anim_laugh:
            if now - self.anim_timer > self.laugh_duration:
                self.anim_laugh = False
                self.v_flicker = False
            else:
                if self.anim_toggle:
                    off_y = self.v_flicker_amp / 2  # Amp for laugh
                else:
                    off_y = -self.v_flicker_amp / 2
                self.anim_toggle = not self.anim_toggle

        # General Flicker Offsets
        h_alt = False
        if self.h_flicker:
            if self.anim_toggle:
                off_x += self.h_flicker_amp
            else:
                off_x -= self.h_flicker_amp
            h_alt = not h_alt
        v_alt = False
        if self.v_flicker:
            if self.anim_toggle:
                off_y += self.v_flicker_amp
            else:
                off_y -= self.v_flicker_amp
            v_alt = not v_alt

        self.curr['eyeL_x'] += off_x
        self.curr['eyeR_x'] += off_x
        self.curr['eyeL_y'] += off_y
        self.curr['eyeR_y'] += off_y

        # --- Mood Targets ---
        if self.mood == TIRED:
            self.next['lid_tired'] = self.curr['eyeL_h'] / 2
        elif self.mood == ANGRY:
            self.next['lid_angry'] = self.curr['eyeL_h'] / 2
        elif self.mood == HAPPY:
            self.next['lid_happy_offset'] = self.curr['eyeL_h'] / 2

        # Tween Lids
        self.curr['lid_tired'] = self._tween_value(self.curr['lid_tired'], self.next['lid_tired'])
        self.curr['lid_angry'] = self._tween_value(self.curr['lid_angry'], self.next['lid_angry'])
        self.curr['lid_happy_offset'] = self._tween_value(self.curr['lid_happy_offset'], self.next['lid_happy_offset'])

        # --- DRAWING ---
        image = Image.new("RGB", (self.width, self.height), self.col_bg)
        draw = ImageDraw.Draw(image)

        def s(x, y):
            return (int(x * self.scale_x), int(y * self.scale_y))

        def s_dim(w, h):
            return (int(w * self.scale_x), int(h * self.scale_y))

        # Draw Left Eye
        lx, ly = self.curr['eyeL_x'] + off_x, self.curr['eyeL_y'] + off_y
        lw, lh = self.curr['eyeL_w'], self.curr['eyeL_h']
        lr = int(self.curr['eyeL_radius'] * self.scale_x)
        draw.rounded_rectangle([s(lx, ly), s(lx + lw, ly + lh)], radius=lr, fill=self.col_main)

        # Draw Right Eye
        if not self.cyclops:
            rx, ry = self.curr['eyeR_x'] + off_x, self.curr['eyeR_y'] + off_y
            rw, rh = self.curr['eyeR_w'], self.curr['eyeR_h']
            rr = int(self.curr['eyeR_radius'] * self.scale_x)
            draw.rounded_rectangle([s(rx, ry), s(rx + rw, ry + rh)], radius=rr, fill=self.col_main)
        else:
            rx, ry, rw, rh, rr = 0, 0, 0, 0, 0

        # Eyelids (BG masks)
        # Happy Bottom
        if self.curr['lid_happy_offset'] > 1:
            off = self.curr['lid_happy_offset']
            # Left
            happy_h = self.default_eye_h * self.scale_y  # Full height for mask
            draw.rounded_rectangle(
                [s(lx - 1, (ly + lh - off) + 1), s(lx + lw + 1, ly + happy_h + 1)],
                radius=lr, fill=self.col_bg
            )
            # Right
            if not self.cyclops:
                draw.rounded_rectangle(
                    [s(rx - 1, (ry + rh - off) + 1), s(rx + rw + 1, ry + happy_h + 1)],
                    radius=rr, fill=self.col_bg
                )

        # Tired Top (Triangles: left-leaning for left, right-leaning for right)
        if self.curr['lid_tired'] > 1:
            h_lid = self.curr['lid_tired']
            # Left
            poly_l = [s(lx, ly - 1), s(lx + lw, ly - 1), s(lx, ly + h_lid - 1)]
            draw.polygon(poly_l, fill=self.col_bg)
            # Right
            if not self.cyclops:
                poly_r = [s(rx, ly - 1), s(rx + rw, ly - 1), s(rx + rw, ly + h_lid - 1)]
                draw.polygon(poly_r, fill=self.col_bg)
            else:
                # Cyclops: Split left eye
                half_w = lw / 2
                poly_l1 = [s(lx, ly - 1), s(lx + half_w, ly - 1), s(lx, ly + h_lid - 1)]
                draw.polygon(poly_l1, fill=self.col_bg)
                poly_l2 = [s(lx + half_w, ly - 1), s(lx + lw, ly - 1), s(lx + lw, ly + h_lid - 1)]
                draw.polygon(poly_l2, fill=self.col_bg)

        # Angry Top (Triangles: right-leaning left, left-leaning right)
        if self.curr['lid_angry'] > 1:
            h_lid = self.curr['lid_angry']
            # Left
            poly_l = [s(lx, ly - 1), s(lx + lw, ly - 1), s(lx + lw, ly + h_lid - 1)]
            draw.polygon(poly_l, fill=self.col_bg)
            # Right
            if not self.cyclops:
                poly_r = [s(rx, ly - 1), s(rx + rw, ly - 1), s(rx, ly + h_lid - 1)]
                draw.polygon(poly_r, fill=self.col_bg)
            else:
                # Cyclops: Split
                half_w = lw / 2
                poly_l1 = [s(lx, ly - 1), s(lx + half_w, ly - 1), s(lx + half_w, ly + h_lid - 1)]
                draw.polygon(poly_l1, fill=self.col_bg)
                poly_l2 = [s(lx + half_w, ly - 1), s(lx + lw, ly - 1), s(lx + half_w, ly + h_lid - 1)]
                draw.polygon(poly_l2, fill=self.col_bg)

        # --- Sweat ---
        if self.sweat:
            sr = int(self.sweat_border_radius * self.scale_x)
            for drop in self.sweat_drops:
                if drop['y'] <= drop['max_y']:
                    drop['y'] += 0.5
                    if drop['y'] <= drop['max_y'] / 2:
                        drop['w'] += 0.5
                        drop['h'] += 0.5
                    else:
                        drop['w'] = max(1.0, drop['w'] - 0.1)
                        drop['h'] = max(1.0, drop['h'] - 0.5)
                else:
                    drop['x_initial'] = random.randint(0, int(self.REF_W))  # Zoned in reset, but random here per C++
                    drop['y'] = 2.0
                    drop['max_y'] = random.randint(10, 20)
                    drop['w'] = 1.0
                    drop['h'] = 2.0
                drop['x'] = drop['x_initial'] - (drop['w'] / 2)
                dx, dy = int(drop['x'] * self.scale_x), int(drop['y'] * self.scale_y)
                dw, dh = int(drop['w'] * self.scale_x), int(drop['h'] * self.scale_y)
                draw.rounded_rectangle([ (dx, dy), (dx + dw, dy + dh) ], radius=sr, fill=self.col_main)

        self.screen_updater.show(image)