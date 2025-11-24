import pygame
import random
import sys

# --- Constants ---
# Moods
DEFAULT = 0
TIRED = 1
ANGRY = 2
HAPPY = 3

# Positions
N = 1
NE = 2
E = 3
SE = 4
S = 5
SW = 6
W = 7
NW = 8
CENTER = 0

class RoboEyes:
    def __init__(self):
        # We set arbitrary initial values; they will be overwritten instantly by the main loop
        self.screen_w = 800
        self.screen_h = 600
        
        # Colors
        self.bg_color = (0, 0, 0)
        self.main_color = (0, 255, 255) # Cyan
        
        # State
        self.mood = DEFAULT
        self.tired = False
        self.angry = False
        self.happy = False
        self.cyclops = False
        
        # Blinking state
        self.eyeL_open = True
        self.eyeR_open = True
        
        # Current Geometry (will be calculated on first update)
        self.eye_w = 100
        self.eye_h = 100
        self.eyeL_h_curr = 1
        self.eyeR_h_curr = 1
        self.eyeL_w_curr = 100
        self.eyeR_w_curr = 100
        
        # Position State (Current coordinates)
        self.eyeL_x = 0
        self.eyeL_y = 0
        self.eyeR_x = 0
        self.eyeR_y = 0
        
        # Target State (Where we want to go)
        self.target_x_off = 0 # Offset from center (0 = center)
        self.target_y_off = 0
        self.target_h = 100
        
        # Eyelid Animation vars
        self.eyelids_tired_h = 0
        self.eyelids_angry_h = 0
        self.eyelids_happy_offset = 0
        
        # Timers
        self.last_blink_time = pygame.time.get_ticks()
        self.blink_interval = 3000
        self.auto_blink = True

    def update_layout(self, width, height, force_center=False):
        """
        Recalculate sizes based on window size.
        IMPORTANT: We use a 'virtual' resolution of 2x for super-sampling.
        So all calculations here are multiplied by scale_factor inside the draw loop,
        but here we keep logic in 'screen pixels'.
        """
        self.screen_w = width
        self.screen_h = height
        
        # Dynamic Sizing based on the SMALLER dimension (keeps aspect ratio sane)
        base_unit = min(width, height)
        
        self.eye_w_default = int(base_unit * 0.35)
        self.eye_h_default = int(base_unit * 0.35)
        self.space_between = int(base_unit * 0.05)
        self.border_radius = int(base_unit * 0.05)
        
        # Calculate the absolute center point of the screen
        self.center_x = width // 2
        self.center_y = height // 2
        
        # Calculate offsets for Left and Right eyes relative to center
        # Left eye is shifted left by half its width + half the space
        self.eyeL_center_offset_x = -(self.eye_w_default + self.space_between) / 2
        # Right eye is shifted right
        self.eyeR_center_offset_x = (self.eye_w_default + self.space_between) / 2
        
        self.target_h = self.eye_h_default

        # If we are initializing or resizing, snap eyes to new positions immediately
        if force_center:
            self.eyeL_x = self.center_x + self.eyeL_center_offset_x
            self.eyeL_y = self.center_y - (self.eye_h_default / 2)
            self.target_x_off = 0
            self.target_y_off = 0

    def get_movement_limits(self):
        # How far can we move from center?
        # Approx 1/4th of the screen in any direction
        limit_x = (self.screen_w // 2) - (self.eye_w_default // 2)
        limit_y = (self.screen_h // 2) - (self.eye_h_default // 2)
        return limit_x, limit_y

    def set_position(self, pos):
        lim_x, lim_y = self.get_movement_limits()
        
        # Set Target Offsets relative to center
        if pos == CENTER: self.target_x_off, self.target_y_off = 0, 0
        elif pos == N:    self.target_x_off, self.target_y_off = 0, -lim_y
        elif pos == S:    self.target_x_off, self.target_y_off = 0, lim_y
        elif pos == E:    self.target_x_off, self.target_y_off = lim_x, 0
        elif pos == W:    self.target_x_off, self.target_y_off = -lim_x, 0
        # Diagonals
        elif pos == NE:   self.target_x_off, self.target_y_off = lim_x, -lim_y
        elif pos == NW:   self.target_x_off, self.target_y_off = -lim_x, -lim_y
        elif pos == SE:   self.target_x_off, self.target_y_off = lim_x, lim_y
        elif pos == SW:   self.target_x_off, self.target_y_off = -lim_x, lim_y

    def set_mood(self, mood):
        self.tired = (mood == TIRED)
        self.angry = (mood == ANGRY)
        self.happy = (mood == HAPPY)

    def blink(self):
        self.eyeL_h_curr = 1
        self.eyeR_h_curr = 1
        self.eyeL_open = False
        self.eyeR_open = False

    def update(self):
        now = pygame.time.get_ticks()
        
        # Auto Blink
        if self.auto_blink and now - self.last_blink_time > self.blink_interval:
            self.blink()
            self.last_blink_time = now + random.randint(0, 2000)

        # --- PHYSICS (TWEENING) ---
        ease = 0.2

        # 1. Height Animation (Blinking)
        self.eyeL_h_curr += (self.target_h - self.eyeL_h_curr) * ease
        self.eyeR_h_curr += (self.target_h - self.eyeR_h_curr) * ease
        
        # Snap to target if close
        if abs(self.target_h - self.eyeL_h_curr) < 1: self.eyeL_h_curr = self.target_h
        if abs(self.target_h - self.eyeR_h_curr) < 1: self.eyeR_h_curr = self.target_h

        # Re-open logic
        if not self.eyeL_open and self.eyeL_h_curr <= 5: self.eyeL_open = True
        if not self.eyeR_open and self.eyeR_h_curr <= 5: self.eyeR_open = True

        # 2. Position Animation
        # Calculate desired absolute coordinates
        target_L_x = self.center_x + self.eyeL_center_offset_x + self.target_x_off
        target_L_y = self.center_y - (self.eye_h_default // 2) + self.target_y_off
        
        self.eyeL_x += (target_L_x - self.eyeL_x) * ease
        self.eyeL_y += (target_L_y - self.eyeL_y) * ease
        
        # Right eye is relative to Left to keep spacing consistent
        self.eyeR_x = self.eyeL_x + self.eye_w_default + self.space_between
        self.eyeR_y = self.eyeL_y

        # 3. Eyelid Animation
        t_tired = self.eyeL_h_curr / 2 if self.tired else 0
        t_angry = self.eyeL_h_curr / 2 if self.angry else 0
        t_happy = self.eyeL_h_curr / 2 if self.happy else 0
        
        self.eyelids_tired_h += (t_tired - self.eyelids_tired_h) * 0.1
        self.eyelids_angry_h += (t_angry - self.eyelids_angry_h) * 0.1
        self.eyelids_happy_offset += (t_happy - self.eyelids_happy_offset) * 0.1

    def draw(self, display_surface):
        """
        Super-Sampling Draw Method:
        1. Create a temporary surface 2x larger than screen.
        2. Draw everything multiplied by 2.
        3. Scale down nicely to display_surface.
        """
        SCALE = 2
        
        # Create high-res canvas
        canvas_w = self.screen_w * SCALE
        canvas_h = self.screen_h * SCALE
        canvas = pygame.Surface((canvas_w, canvas_h))
        canvas.fill(self.bg_color)
        
        # Helper to scale coordinates
        def s(val): return int(val * SCALE)
        
        # --- DRAW EYES (High Res) ---
        
        # Calculate scaled rects
        # We center vertically based on the current height vs max height
        h_offset_L = (self.eye_h_default - self.eyeL_h_curr) / 2
        h_offset_R = (self.eye_h_default - self.eyeR_h_curr) / 2
        
        rect_L = pygame.Rect(s(self.eyeL_x), s(self.eyeL_y + h_offset_L), s(self.eye_w_default), s(self.eyeL_h_curr))
        
        # Draw Left Eye
        pygame.draw.rect(canvas, self.main_color, rect_L, border_radius=s(self.border_radius))
        
        rect_R = None
        if not self.cyclops:
            rect_R = pygame.Rect(s(self.eyeR_x), s(self.eyeR_y + h_offset_R), s(self.eye_w_default), s(self.eyeR_h_curr))
            pygame.draw.rect(canvas, self.main_color, rect_R, border_radius=s(self.border_radius))

        # --- EYELIDS (Masking) ---
        
        # TIRED
        if self.eyelids_tired_h > 1:
            h = s(self.eyelids_tired_h)
            # Left
            pts = [(rect_L.left, rect_L.top), (rect_L.right, rect_L.top), (rect_L.left, rect_L.top + h)]
            pygame.draw.polygon(canvas, self.bg_color, pts)
            # Right
            if rect_R:
                pts = [(rect_R.left, rect_R.top), (rect_R.right, rect_R.top), (rect_R.right, rect_R.top + h)]
                pygame.draw.polygon(canvas, self.bg_color, pts)

        # ANGRY
        if self.eyelids_angry_h > 1:
            h = s(self.eyelids_angry_h)
            # Left
            pts = [(rect_L.left, rect_L.top), (rect_L.right, rect_L.top), (rect_L.right, rect_L.top + h)]
            pygame.draw.polygon(canvas, self.bg_color, pts)
            # Right
            if rect_R:
                pts = [(rect_R.left, rect_R.top), (rect_R.right, rect_R.top), (rect_R.left, rect_R.top + h)]
                pygame.draw.polygon(canvas, self.bg_color, pts)

        # HAPPY
        if self.eyelids_happy_offset > 1:
            h = s(self.eyelids_happy_offset)
            # Left mask (draw rectangle from bottom up)
            mask_L = pygame.Rect(rect_L.left - 5, rect_L.bottom - h, rect_L.width + 10, h + 20)
            pygame.draw.rect(canvas, self.bg_color, mask_L, border_radius=s(self.border_radius))
            # Right mask
            if rect_R:
                mask_R = pygame.Rect(rect_R.left - 5, rect_R.bottom - h, rect_R.width + 10, h + 20)
                pygame.draw.rect(canvas, self.bg_color, mask_R, border_radius=s(self.border_radius))

        # --- FINAL RENDER ---
        # Smoothscale down to actual screen size (Antialiasing effect)
        pygame.transform.smoothscale(canvas, (self.screen_w, self.screen_h), display_surface)

# --- MAIN LOOP ---
def main():
    pygame.init()
    
    # Default size
    WIDTH, HEIGHT = 800, 600
    
    # Set flags for a resizable window
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
    pygame.display.set_caption("RoboEyes V2.0 - Smooth & Responsive")
    
    clock = pygame.time.Clock()
    eyes = RoboEyes()
    
    # Initial layout calculation
    eyes.update_layout(WIDTH, HEIGHT, force_center=True)
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # HANDLE RESIZE
            elif event.type == pygame.VIDEORESIZE:
                WIDTH, HEIGHT = event.w, event.h
                # Update eyes and FORCE CENTER to fix positioning bug
                eyes.update_layout(WIDTH, HEIGHT, force_center=True)
            
            # CONTROLS
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1: eyes.set_mood(DEFAULT)
                if event.key == pygame.K_2: eyes.set_mood(TIRED)
                if event.key == pygame.K_3: eyes.set_mood(ANGRY)
                if event.key == pygame.K_4: eyes.set_mood(HAPPY)
                if event.key == pygame.K_SPACE: eyes.blink()
                
                if event.key == pygame.K_UP: eyes.set_position(N)
                if event.key == pygame.K_DOWN: eyes.set_position(S)
                if event.key == pygame.K_LEFT: eyes.set_position(W)
                if event.key == pygame.K_RIGHT: eyes.set_position(E)
                if event.key == pygame.K_c: eyes.set_position(CENTER)

        # Update Logic
        eyes.update()
        
        # Draw (Passing screen surface)
        eyes.draw(screen)
        
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()