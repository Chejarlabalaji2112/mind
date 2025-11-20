# roboeyes_widget.py

from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, BooleanProperty, ListProperty
from kivy.graphics import Color, RoundedRectangle, Triangle
from kivy.clock import Clock
from kivy.utils import get_color_from_hex
import random
import time

# --- CONSTANTS ---
BGCOLOR_HEX = '#000000' # Black
MAINCOLOR_HEX = '#40E0D0' # Turquoise blue

# For mood type switch
DEFAULT, TIRED, ANGRY, HAPPY = 0, 1, 2, 3

# For predefined positions
N, NE, E, SE, S, SW, W, NW = 1, 2, 3, 4, 5, 6, 7, 8
# Note: Kivy widgets use the bottom-left corner (0, 0) as their origin.

class RoboEyesWidget(Widget):
    """
    Kivy port of the FluxGarage RoboEyes library.
    Handles all eye animation and drawing logic within its canvas.
    """

    # --- Kivy Properties for drawing and animation ---
    # These properties trigger redraws automatically when changed if bound (though we use a manual Clock update)
    eyeLwidthCurrent = NumericProperty(1)
    eyeLheightCurrent = NumericProperty(1)
    eyeLx = NumericProperty(0)
    eyeLy = NumericProperty(0)
    eyeLborderRadiusCurrent = NumericProperty(0)

    eyeRwidthCurrent = NumericProperty(1)
    eyeRheightCurrent = NumericProperty(1)
    eyeRx = NumericProperty(0)
    eyeRy = NumericProperty(0)
    eyeRborderRadiusCurrent = NumericProperty(0)

    spaceBetweenCurrent = NumericProperty(10)

    tired = BooleanProperty(False)
    angry = BooleanProperty(False)
    happy = BooleanProperty(False)
    cyclops = BooleanProperty(False)
    sweat = BooleanProperty(False)
    curious = BooleanProperty(False)
    
    # Eyelid and flicker state (used in draw_eyes calculation)
    eyelidsTiredHeight = NumericProperty(0)
    eyelidsAngryHeight = NumericProperty(0)
    eyelidsHappyBottomOffset = NumericProperty(0)
    
    hFlickerAlternate = BooleanProperty(False)
    vFlickerAlternate = BooleanProperty(False)
    
    # --- Internal/Hidden Variables (Default C++ values) ---
    eyeLwidthDefault = 36
    eyeLheightDefault = 36
    eyeRwidthDefault = 36
    eyeRheightDefault = 36
    eyeLborderRadiusDefault = 8
    eyeRborderRadiusDefault = 8
    spaceBetweenDefault = 10
    
    eyeLwidthNext = 36
    eyeLheightNext = 36
    eyeRwidthNext = 36
    eyeRheightNext = 36
    eyeLborderRadiusNext = 8
    eyeRborderRadiusNext = 8
    spaceBetweenNext = 10
    
    eyeL_open = False
    eyeR_open = False
    
    eyeLxNext = 0
    eyeLyNext = 0
    
    # Macro Animations
    autoblinker = False
    blinkInterval = 1
    blinkIntervalVariation = 4
    blinktimer = 0
    
    idle = False
    idleInterval = 1
    idleIntervalVariation = 3
    idleAnimationTimer = 0
    
    confused = False
    confusedAnimationDuration = 500
    confusedAnimationTimer = 0
    confusedToggle = True
    
    laugh = False
    laughAnimationDuration = 500
    laughAnimationTimer = 0
    laughToggle = True

    hFlicker = False
    hFlickerAmplitude = 2
    vFlicker = False
    vFlickerAmplitude = 10
    
    # Sweat properties (initialised in __init__)
    sweat1YPos = 2.0
    sweat1YPosMax = 10
    sweat1XPosInitial = 0
    sweat2YPos = 2.0
    sweat2YPosMax = 10
    sweat2XPosInitial = 0
    sweat3YPos = 2.0
    sweat3YPosMax = 10
    sweat3XPosInitial = 0
    sweatBorderradius = 3
    
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # Calculate initial coordinates (center of the widget)
        self.bind(size=self._recalculate_defaults)
        
        # Initialize properties to closed eyes (as per C++ begin())
        self.eyeLwidthCurrent = self.eyeLwidthDefault
        self.eyeLheightCurrent = 1
        self.eyeRwidthCurrent = self.eyeRwidthDefault
        self.eyeRheightCurrent = 1
        
        # Initialise sweat drop positions (random initialisation is done later)
        self._reset_sweat_drops()
        
        # Start the update loop
        self.set_framerate(50) # Default 50 fps
        
    def _get_millis(self):
        """Returns current time in milliseconds, equivalent to Arduino millis()."""
        return int(time.time() * 1000)
    
    def _recalculate_defaults(self, *args):
        """Called when widget size changes to recalculate default positions."""
        
        # Calculate central eye position (center-aligned for both eyes)
        total_eye_area_width = self.eyeLwidthDefault + self.spaceBetweenDefault + self.eyeRwidthDefault
        
        # X position of the left eye when centered
        self.eyeLxDefault = (self.width - total_eye_area_width) / 2
        
        # Y position of the eye when centered (Kivy y is inverted relative to C++ y: y=0 is bottom)
        # C++ eyeLyDefault: ((screenHeight - eyeLheightDefault) / 2) -> Center-Y on screen
        self.eyeLyDefault = (self.height - self.eyeLheightDefault) / 2
        
        # Set current/next to defaults
        self.eyeLx = self.eyeLxNext = self.eyeLxDefault
        self.eyeLy = self.eyeLyNext = self.eyeLyDefault
        
        # Right eye position depends on left eye + width + space
        self.eyeRxDefault = self.eyeLxDefault + self.eyeLwidthDefault + self.spaceBetweenDefault
        self.eyeRyDefault = self.eyeLyDefault
        self.eyeRx = self.eyeRxNext = self.eyeRxDefault
        self.eyeRy = self.eyeRyNext = self.eyeRyDefault
        
        
    # --- GETTERS METHODS ---
    def get_screen_constraint_x(self):
        """Returns the max x position for left eye (right edge constraint)"""
        # (Widget width) - (Total width of both eyes and space)
        return self.width - self.eyeLwidthCurrent - self.spaceBetweenCurrent - self.eyeRwidthCurrent

    def get_screen_constraint_y(self):
        """Returns the max y position for left eye (top edge constraint)"""
        # C++ uses default height for constraint calculation: screenHeight - eyeLheightDefault
        return self.height - self.eyeLheightDefault
        
        
    # --- SETTERS METHODS ---
    def set_framerate(self, fps):
        """Calculates frame interval and starts the Kivy Clock update."""
        if hasattr(self, '_frame_event'):
            self._frame_event.cancel()
            
        frame_interval_sec = 1.0 / fps
        self._frame_event = Clock.schedule_interval(self.update, frame_interval_sec)

    def set_mood(self, mood):
        self.tired = (mood == TIRED)
        self.angry = (mood == ANGRY)
        self.happy = (mood == HAPPY)
        
    def set_position(self, position):
        # Kivy Y is bottom-up, C++ Y is top-down. The constraint calculation handles this.
        constraint_x = self.get_screen_constraint_x()
        constraint_y = self.get_screen_constraint_y()
        
        # Calculate new X and Y positions (eyeLxNext, eyeLyNext)
        if position == N:       (self.eyeLxNext, self.eyeLyNext) = (constraint_x/2, constraint_y)
        elif position == NE:    (self.eyeLxNext, self.eyeLyNext) = (constraint_x, constraint_y)
        elif position == E:     (self.eyeLxNext, self.eyeLyNext) = (constraint_x, constraint_y/2)
        elif position == SE:    (self.eyeLxNext, self.eyeLyNext) = (constraint_x, 0) # Kivy (0, 0) is bottom-left
        elif position == S:     (self.eyeLxNext, self.eyeLyNext) = (constraint_x/2, 0)
        elif position == SW:    (self.eyeLxNext, self.eyeLyNext) = (0, 0)
        elif position == W:     (self.eyeLxNext, self.eyeLyNext) = (0, constraint_y/2)
        elif position == NW:    (self.eyeLxNext, self.eyeLyNext) = (0, constraint_y)
        else: # DEFAULT (Middle center)
            (self.eyeLxNext, self.eyeLyNext) = (constraint_x/2, constraint_y/2)

    def set_autoblinker(self, active, interval=1, variation=4):
        self.autoblinker = active
        self.blinkInterval = interval
        self.blinkIntervalVariation = variation

    def set_idle_mode(self, active, interval=1, variation=3):
        self.idle = active
        self.idleInterval = interval
        self.idleIntervalVariation = variation

    def set_hflicker(self, flicker_bit, amplitude=2):
        self.hFlicker = flicker_bit
        self.hFlickerAmplitude = amplitude

    def set_vflicker(self, flicker_bit, amplitude=10):
        self.vFlicker = flicker_bit
        self.vFlickerAmplitude = amplitude
        
    def set_sweat(self, sweat_bit):
        self.sweat = sweat_bit
        if not sweat_bit:
            self._reset_sweat_drops()
        
    def set_cyclops(self, cyclops_bit):
        self.cyclops = cyclops_bit
        
    # --- BASIC ANIMATION METHODS ---
    def close(self):
        self.eyeLheightNext = 1
        self.eyeRheightNext = 1
        self.eyeL_open = False
        self.eyeR_open = False

    def open(self):
        self.eyeL_open = True
        self.eyeR_open = True

    def blink(self):
        self.close()
        self.open()
        
    # --- MACRO ANIMATION METHODS ---
    def anim_confused(self):
        self.confused = True
        
    def anim_laugh(self):
        self.laugh = True

    # --- MAIN UPDATE LOOP ---
    def update(self, dt):
        """Called by Kivy Clock, equivalent to C++ update() combined with drawEyes() setup."""
        self._calculate_tweens()
        self._apply_macro_animations()
        self.canvas.ask_update() # Ensure Kivy redraws the canvas in the next frame

    def _calculate_tweens(self):
        """Performs all the size, position, and transition calculations."""
        
        # --- PRE-CALCULATIONS - EYE SIZES AND VALUES FOR ANIMATION TWEENINGS ---
        
        # Vertical size offset for curious gaze
        eyeLheightOffset = 0
        eyeRheightOffset = 0
        if self.curious:
            # Note: Kivy Y constraint (0 is bottom, constraint_y is top)
            constraint_x = self.get_screen_constraint_x()
            if self.eyeLxNext <= 10: eyeLheightOffset = 8
            elif self.eyeLxNext >= (constraint_x - 10) and self.cyclops: eyeLheightOffset = 8
            
            # This logic is complex for the right eye in Kivy's coordinate system, keeping basic translation
            if self.eyeRxNext >= self.width - self.eyeRwidthCurrent - 10: eyeRheightOffset = 8
            
        # Left eye height tweening and centering (Kivy Y must be updated carefully)
        self.eyeLheightCurrent = (self.eyeLheightCurrent + self.eyeLheightNext + eyeLheightOffset) / 2
        # C++: eyeLy+= ((eyeLheightDefault-eyeLheightCurrent)/2)
        # We need to adjust eyeLy based on the difference from default height to keep it centered vertically
        self.eyeLy += (self.eyeLheightDefault - self.eyeLheightCurrent) / 2
        self.eyeLy -= eyeLheightOffset / 2
        
        # Right eye height tweening and centering
        self.eyeRheightCurrent = (self.eyeRheightCurrent + self.eyeRheightNext + eyeRheightOffset) / 2
        self.eyeRy += (self.eyeRheightDefault - self.eyeRheightCurrent) / 2
        self.eyeRy -= eyeRheightOffset / 2
        
        # Open eyes again after closing them
        if self.eyeL_open and self.eyeLheightCurrent <= 1 + eyeLheightOffset: self.eyeLheightNext = self.eyeLheightDefault
        if self.eyeR_open and self.eyeRheightCurrent <= 1 + eyeRheightOffset: self.eyeRheightNext = self.eyeRheightDefault
        
        # Other tweens
        self.eyeLwidthCurrent = (self.eyeLwidthCurrent + self.eyeLwidthNext) / 2
        self.eyeRwidthCurrent = (self.eyeRwidthCurrent + self.eyeRwidthNext) / 2
        self.spaceBetweenCurrent = (self.spaceBetweenCurrent + self.spaceBetweenNext) / 2
        
        # Left eye coordinates tween
        self.eyeLx = (self.eyeLx + self.eyeLxNext) / 2
        self.eyeLy = (self.eyeLy + self.eyeLyNext) / 2
        
        # Right eye coordinates tween
        self.eyeRxNext = self.eyeLxNext + self.eyeLwidthCurrent + self.spaceBetweenCurrent
        self.eyeRyNext = self.eyeLyNext
        self.eyeRx = (self.eyeRx + self.eyeRxNext) / 2
        self.eyeRy = (self.eyeRy + self.eyeRyNext) / 2
        
        # Border radius tweens
        self.eyeLborderRadiusCurrent = (self.eyeLborderRadiusCurrent + self.eyeLborderRadiusNext) / 2
        self.eyeRborderRadiusCurrent = (self.eyeRborderRadiusCurrent + self.eyeRborderRadiusNext) / 2
        
        # Prepare mood type transitions
        eyeLheight_current_safe = max(1, self.eyeLheightCurrent)
        if self.tired:
            eyelidsTiredHeightNext = eyeLheight_current_safe / 2
            eyelidsAngryHeightNext = 0
        else: eyelidsTiredHeightNext = 0
            
        if self.angry:
            eyelidsAngryHeightNext = eyeLheight_current_safe / 2
            eyelidsTiredHeightNext = 0
        else: eyelidsAngryHeightNext = 0
            
        if self.happy:
            eyelidsHappyBottomOffsetNext = eyeLheight_current_safe / 2
        else: eyelidsHappyBottomOffsetNext = 0
        
        self.eyelidsTiredHeight = (self.eyelidsTiredHeight + eyelidsTiredHeightNext) / 2
        self.eyelidsAngryHeight = (self.eyelidsAngryHeight + eyelidsAngryHeightNext) / 2
        self.eyelidsHappyBottomOffset = (self.eyelidsHappyBottomOffset + eyelidsHappyBottomOffsetNext) / 2


    def _apply_macro_animations(self):
        """Applies all temporal and flicking animations."""
        current_millis = self._get_millis()
        
        # Autoblinker
        if self.autoblinker and current_millis >= self.blinktimer:
            self.blink()
            interval_ms = (self.blinkInterval * 1000) + (random.randrange(self.blinkIntervalVariation + 1) * 1000)
            self.blinktimer = current_millis + interval_ms
            
        # Laughing
        if self.laugh:
            if self.laughToggle:
                self.set_vflicker(True, 5)
                self.laughAnimationTimer = current_millis
                self.laughToggle = False
            elif current_millis >= self.laughAnimationTimer + self.laughAnimationDuration:
                self.set_vflicker(False, 0)
                self.laughToggle = True
                self.laugh = False

        # Confused
        if self.confused:
            if self.confusedToggle:
                self.set_hflicker(True, 20)
                self.confusedAnimationTimer = current_millis
                self.confusedToggle = False
            elif current_millis >= self.confusedAnimationTimer + self.confusedAnimationDuration:
                self.set_hflicker(False, 0)
                self.confusedToggle = True
                self.confused = False
                
        # Idle
        if self.idle and current_millis >= self.idleAnimationTimer:
            constraint_x = self.get_screen_constraint_x()
            constraint_y = self.get_screen_constraint_y()
            
            self.eyeLxNext = random.randrange(int(constraint_x) + 1)
            self.eyeLyNext = random.randrange(int(constraint_y) + 1)
            
            interval_ms = (self.idleInterval * 1000) + (random.randrange(self.idleIntervalVariation + 1) * 1000)
            self.idleAnimationTimer = current_millis + interval_ms

        # Horizontal Flickering
        if self.hFlicker:
            if self.hFlickerAlternate:
                self.eyeLx += self.hFlickerAmplitude
                self.eyeRx += self.hFlickerAmplitude
            else:
                self.eyeLx -= self.hFlickerAmplitude
                self.eyeRx -= self.hFlickerAmplitude
            self.hFlickerAlternate = not self.hFlickerAlternate

        # Vertical Flickering
        if self.vFlicker:
            if self.vFlickerAlternate:
                self.eyeLy += self.vFlickerAmplitude
                self.eyeRy += self.vFlickerAmplitude
            else:
                self.eyeLy -= self.vFlickerAmplitude
                self.eyeRy -= self.vFlickerAmplitude
            self.vFlickerAlternate = not self.vFlickerAlternate

    def _reset_sweat_drops(self):
        """Resets sweat drop positions (equivalent to C++ else block on max Y)."""
        self.sweat1XPosInitial = random.randrange(30)
        self.sweat1YPos = 2.0
        self.sweat1YPosMax = random.randrange(10, 20)
        
        self.sweat2XPosInitial = random.randrange(int(self.width) - 60) + 30
        self.sweat2YPos = 2.0
        self.sweat2YPosMax = random.randrange(10, 20)
        
        self.sweat3XPosInitial = (int(self.width) - 30) + random.randrange(30)
        self.sweat3YPos = 2.0
        self.sweat3YPosMax = random.randrange(10, 20)
        
    def _update_sweat_drops(self, dt):
        """Calculates position for drawing sweat drops."""
        # Note: Kivy does not handle floating point positions well, so we use integers for drawing, but floats for movement
        
        def move_drop(x_initial, y_pos, y_max):
            if y_pos <= y_max: y_pos += 0.5
            else:
                # Reset for next drop
                x_initial = random.randrange(int(self.width))
                y_pos = 2.0
                y_max = random.randrange(10, 20)
            
            # Simple scaling logic based on drop position
            width, height = 1.0, 2.0
            if y_pos <= y_max / 2:
                width += 0.5
                height += 0.5
            else:
                width -= 0.1
                height -= 0.5
                
            x_pos = x_initial - (width / 2) # Keep centered
            return x_pos, y_pos, y_max, width, height
        
        # Move and update properties
        x1, self.sweat1YPos, self.sweat1YPosMax, w1, h1 = move_drop(self.sweat1XPosInitial, self.sweat1YPos, self.sweat1YPosMax)
        x2, self.sweat2YPos, self.sweat2YPosMax, w2, h2 = move_drop(self.sweat2XPosInitial, self.sweat2YPos, self.sweat2YPosMax)
        x3, self.sweat3YPos, self.sweat3YPosMax, w3, h3 = move_drop(self.sweat3XPosInitial, self.sweat3YPos, self.sweat3YPosMax)
        
        # Store for use in on_draw
        self._sweat_drops_data = [
            (x1, self.sweat1YPos, w1, h1),
            (x2, self.sweat2YPos, w2, h2),
            (x3, self.sweat3YPos, w3, h3),
        ]
        
    # --- Kivy Drawing Handler ---
    def on_canvas(self, *args):
        """Main drawing method, equivalent to the C++ 'ACTUAL DRAWINGS' section."""
        
        # Cyclops mode, set second eye's size and space between to 0 (overriding current values for drawing)
        eyeR_width = 0 if self.cyclops else self.eyeRwidthCurrent
        eyeR_height = 0 if self.cyclops else self.eyeRheightCurrent
        
        self.canvas.clear()
        
        with self.canvas:
            
            # --- Draw basic eye rectangles (MAINCOLOR) ---
            Color(*get_color_from_hex(MAINCOLOR_HEX)) 
            
            # Kivy Y must be calculated as (widget_height - C++_Y - element_height)
            eyeL_y_kivy = self.height - self.eyeLy - self.eyeLheightCurrent
            eyeR_y_kivy = self.height - self.eyeRy - eyeR_height

            # Left Eye
            RoundedRectangle(
                pos=(self.eyeLx, eyeL_y_kivy),
                size=(self.eyeLwidthCurrent, self.eyeLheightCurrent),
                radius=[self.eyeLborderRadiusCurrent]
            )

            # Right Eye (if not cyclops)
            if not self.cyclops:
                RoundedRectangle(
                    pos=(self.eyeRx, eyeR_y_kivy),
                    size=(eyeR_width, eyeR_height),
                    radius=[self.eyeRborderRadiusCurrent]
                )

            # --- Draw eyelids/overlays (BGCOLOR) ---
            Color(*get_color_from_hex(BGCOLOR_HEX))
            
            # Kivy Y inversion: Y_top_c++ = Y_bottom_kivy + Height
            
            # 1. Draw tired top eyelids (Obscure top corners)
            if self.tired:
                y_top = self.height - self.eyeLy + 1 # Top edge of eye
                
                # Left Eye (Triangle to obscure top-left corner)
                Triangle(
                    points=[
                        self.eyeLx, y_top, # Top-left (y-inverted)
                        self.eyeLx + self.eyeLwidthCurrent, y_top, # Top-right (y-inverted)
                        self.eyeLx, self.height - (self.eyeLy + self.eyelidsTiredHeight) + 1 # Bottom-left corner of triangle
                    ]
                )
                if not self.cyclops:
                    # Right Eye (Triangle to obscure top-right corner)
                    Triangle(
                        points=[
                            self.eyeRx, y_top,
                            self.eyeRx + eyeR_width, y_top,
                            self.eyeRx + eyeR_width, self.height - (self.eyeRy + self.eyelidsTiredHeight) + 1
                        ]
                    )

            # 2. Draw angry top eyelids (Obscure top corners symmetrically)
            if self.angry:
                y_top = self.height - self.eyeLy + 1 # Top edge of eye
                y_bottom_lid = self.height - (self.eyeLy + self.eyelidsAngryHeight) + 1 # Bottom edge of lid
                
                # Left Eye (Obscure top-right corner)
                Triangle(points=[self.eyeLx, y_top, self.eyeLx + self.eyeLwidthCurrent, y_top, self.eyeLx + self.eyeLwidthCurrent, y_bottom_lid])
                
                if not self.cyclops:
                    # Right Eye (Obscure top-left corner)
                    Triangle(points=[self.eyeRx, y_top, self.eyeRx + eyeR_width, y_top, self.eyeRx, y_bottom_lid])

            # 3. Draw happy bottom eyelids (Obscure bottom half)
            if self.happy:
                # Kivy Y: The point where the cutout *starts* (y_bottom_kivy - offset)
                y_cutout = eyeL_y_kivy - self.eyelidsHappyBottomOffset + 1 
                
                # Left Eye (Draw BGCOLOR rect to cover the bottom part)
                RoundedRectangle(
                    pos=(self.eyeLx - 1, y_cutout),
                    size=(self.eyeLwidthCurrent + 2, self.eyeLheightDefault), # Use default height for full cutout size
                    radius=[self.eyeLborderRadiusCurrent]
                )
                
                if not self.cyclops:
                    y_cutout_r = eyeR_y_kivy - self.eyelidsHappyBottomOffset + 1
                    # Right Eye
                    RoundedRectangle(
                        pos=(self.eyeRx - 1, y_cutout_r),
                        size=(eyeR_width + 2, self.eyeRheightDefault),
                        radius=[self.eyeRborderRadiusCurrent]
                    )
            
            # 4. Add sweat drops
            if self.sweat:
                Color(*get_color_from_hex(MAINCOLOR_HEX)) # Reset to MAINCOLOR
                self._update_sweat_drops(0) # Calculate positions just before drawing
                for x, y, w, h in self._sweat_drops_data:
                    # Kivy Y: y_drop_kivy = widget_height - y_drop_c++ - h_drop
                    RoundedRectangle(
                        pos=(x, self.height - y - h),
                        size=(w, h),
                        radius=[self.sweatBorderradius]
                    )