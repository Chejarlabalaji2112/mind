import cv2
import mujoco
import numpy as np
import threading
from mind.adapters.display_adapters import DisplayObj

#TODO: implement clear screen.

class ScreenUpdater(DisplayObj):
    def __init__(self, model, texture_name):
        self.model = model
        self.texture_name = texture_name
        
        # ----- Locate Texture -----
        self.tex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, texture_name)
        if self.tex_id == -1:
            raise ValueError(f"Texture '{texture_name}' not found.")
            
        self.tex_adr = model.tex_adr[self.tex_id]
        self.width = model.tex_width[self.tex_id]
        self.height = model.tex_height[self.tex_id]
        self.nchannel = model.tex_nchannel[self.tex_id]
        self.size = self.width * self.height * self.nchannel
        
        # Pending frame for async write
        self._pending_frame = None
        self._lock = threading.Lock()
        
        # Preallocate reusable canvas for text drawing
        self._text_canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Font settings
        self.font = cv2.FONT_HERSHEY_DUPLEX

        # ----- Precompute anchors for ultra-fast rendering -----
        self.anchor_top_left     = (10, 40)
        self.anchor_top_center   = (self.width // 2, 40)
        self.anchor_top_right    = (self.width - 10, 40)
        self.anchor_center       = (self.width // 2, self.height // 2)
        self.anchor_bottom_left  = (10, self.height - 20)
        self.anchor_bottom_right = (self.width - 10, self.height - 20)


    # =========================================================
    #  Basic Image Update
    # =========================================================

    def show(self, frame_bgr):
        if not isinstance(frame_bgr, np.ndarray):
            frame_bgr = np.array(frame_bgr)

        # Grayscale support
        if len(frame_bgr.shape) == 2:
            frame_bgr = cv2.cvtColor(frame_bgr, cv2.COLOR_GRAY2BGR)

        # Resize if mismatch
        if frame_bgr.shape[0] != self.height or frame_bgr.shape[1] != self.width:
            frame_bgr = cv2.resize(frame_bgr, (self.width, self.height),
                                   interpolation=cv2.INTER_LINEAR)
            
        # Convert to RGB(A) flatten for Mujoco
        if self.nchannel == 3:
            flat_data = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).flatten()
        else:
            flat_data = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2BGRA).flatten()

        with self._lock:
            self._pending_frame = flat_data


    # =========================================================
    #  Simple Centered Text
    # =========================================================

    def show_text(self, text, bg_color=(0,0,0), text_color=(255,255,255), scale=1.0):
        self._text_canvas[:] = bg_color
        
        font = self.font
        thickness = 2
        
        tw, th = cv2.getTextSize(text, font, scale, thickness)[0]
        x = (self.width - tw) // 2
        y = (self.height + th) // 2
        
        cv2.putText(self._text_canvas, text, (x, y),
                    font, scale, text_color, thickness, cv2.LINE_AA)
        
        self.show(self._text_canvas)


    # =========================================================
    #  Efficient UI Layout Templating (Timer/Stopwatch/Pomodoro)
    # =========================================================

    def _put_text_centered(self, canvas, text, anchor, scale, color, thickness):
        """Fast centered text for top-center and middle text."""
        if not text:
            return
        tw, th = cv2.getTextSize(text, self.font, scale, thickness)[0]
        x = anchor[0] - tw // 2
        y = anchor[1] + th // 2
        cv2.putText(canvas, text, (x, y),
                    self.font, scale, color, thickness, cv2.LINE_AA)


    def show_layout(self, data, bg_color=(0,0,0), styles=None):
        """
        data = {
            "top_left": str,
            "top_center": str,
            "center": str,         # main counter
            "bottom_left": str,
            "bottom_right": str
        }
        """
        # Reuse canvas
        self._text_canvas[:] = bg_color

        if styles is None:
            styles = {}

        # Default per-region styles (min overhead)
        default_styles = {
            "top_left":     (0.7, (255,255,255), 1),
            "top_center":   (0.7, (255,255,255), 1),
            "top_right" :   (0.7, (255,255,255), 1),
            "center":       (2.0, (255,255,255), 2),
            "bottom_left":  (0.6, (200,200,200), 1),
            "bottom_right": (0.6, (200,200,200), 1),
        }

        # Merge defaults with overrides
        merged = {k: styles.get(k, default_styles[k]) for k in default_styles}

        # ----- TOP-CENTER -----
        self._put_text_centered(
            self._text_canvas,
            data.get("top_center",""),
            self.anchor_top_center,
            *merged["top_center"]
        )

        # ----- TOP-LEFT (left aligned) -----
        tl = data.get("top_left","")
        if tl:
            scl, col, th = merged["top_left"]
            cv2.putText(self._text_canvas, tl,
                        self.anchor_top_left,
                        self.font, scl, col, th, cv2.LINE_AA)
            
        tr = data.get("top_right", "")
        if tr:
            scl, col, th = merged["top_right"]
            tw, ths = cv2.getTextSize(tr, self.font, scl, th)[0]
            x = self.anchor_top_right[0] - tw
            y = self.anchor_top_right[1]
            cv2.putText(self._text_canvas, tr, (x, y),
                        self.font, scl, col, th, cv2.LINE_AA)

        # ----- CENTER (center aligned) -----
        self._put_text_centered(
            self._text_canvas,
            data.get("center",""),
            self.anchor_center,
            *merged["center"]
        )

        # ----- BOTTOM-LEFT -----
        bl = data.get("bottom_left","")
        if bl:
            scl, col, th = merged["bottom_left"]
            cv2.putText(self._text_canvas, bl,
                        self.anchor_bottom_left,
                        self.font, scl, col, th, cv2.LINE_AA)

        # ----- BOTTOM-RIGHT (right aligned) -----
        br = data.get("bottom_right","")
        if br:
            scl, col, th = merged["bottom_right"]
            tw, ths = cv2.getTextSize(br, self.font, scl, th)[0]
            x = self.anchor_bottom_right[0] - tw
            y = self.anchor_bottom_right[1]
            cv2.putText(self._text_canvas, br,
                        (x, y),
                        self.font, scl, col, th, cv2.LINE_AA)

        # Push frame to pipeline
        self.show(self._text_canvas)


    # =========================================================
    #  MuJoCo GPU Upload
    # =========================================================

    def update(self, viewer=None):
        with self._lock:
            if self._pending_frame is None:
                return
            data_to_write = self._pending_frame
            self._pending_frame = None

        if hasattr(self.model, 'tex_data'):
            self.model.tex_data[self.tex_adr : self.tex_adr + self.size] = data_to_write
        elif hasattr(self.model, 'tex_rgb'):
            self.model.tex_rgb[self.tex_adr : self.tex_adr + self.size] = \
                data_to_write.astype(np.float32) / 255.0

        if viewer:
            viewer.update_texture(self.tex_id)
