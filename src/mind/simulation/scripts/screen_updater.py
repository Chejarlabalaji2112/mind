
import cv2
import mujoco
import numpy as np
import time
import threading
from adapters.display_adapters import DisplayObj

class ScreenUpdater(DisplayObj):
    """
    A passive display class. It holds a buffer of the 'latest' image/text 
    and writes it to Mujoco memory only when update() is called in the render loop.
    """
    def __init__(self, model, texture_name):
        self.model = model
        self.texture_name = texture_name
        
        # Locate Texture
        self.tex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, texture_name)
        if self.tex_id == -1:
            raise ValueError(f"Texture '{texture_name}' not found.")
            
        self.tex_adr = model.tex_adr[self.tex_id]
        self.width = model.tex_width[self.tex_id]
        self.height = model.tex_height[self.tex_id]
        self.nchannel = model.tex_nchannel[self.tex_id] # Usually 3 (RGB)
        self.size = self.width * self.height * self.nchannel
        
        # Buffer to hold the frame waiting to be uploaded
        self._pending_frame = None
        self._lock = threading.Lock() # Thread safety for async updates from AV thread

    def show(self, frame_bgr):
        """
        Receive a generic BGR frame (from OpenCV or PyAV).
        This does NOT write to Mujoco yet (fast operation).
        """
        if not isinstance(frame_bgr, np.ndarray):
            frame_bgr = np.array(frame_bgr)

        if len(frame_bgr.shape) == 2:
            frame_bgr = cv2.cvtColor(frame_bgr, cv2.COLOR_GRAY2BGR)

        # Resize to match texture dimensions
        if frame_bgr.shape[0] != self.height or frame_bgr.shape[1] != self.width:
            frame_bgr = cv2.resize(frame_bgr, (self.width, self.height))
            
        # Convert to Mujoco format (RGB or RGBA flattened)
        if self.nchannel == 3:
            flat_data = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).flatten()
        else:
            # Handle RGBA texture
            flat_data = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2BGRA).flatten()

        with self._lock:
            self._pending_frame = flat_data

    def show_text(self, text, bg_color=(0, 0, 0), text_color=(255, 255, 255), scale=1.0):
        """
        Generates an image from text and puts it into the buffer.
        """
        # Create canvas
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        canvas[:] = bg_color
        
        # Calculate centering
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, scale, 2)[0]
        text_x = (self.width - text_size[0]) // 2
        text_y = (self.height + text_size[1]) // 2
        
        cv2.putText(canvas, text, (text_x, text_y), font, scale, text_color, 2)
        self.show(canvas)

    def update(self, viewer=None):
        """
        Call this in the main Mujoco loop.
        Checks if a new frame is available and writes it to GPU memory.
        """
        with self._lock:
            if self._pending_frame is None:
                return
            data_to_write = self._pending_frame
            self._pending_frame = None # Consume the frame

        # Write to model memory
        if hasattr(self.model, 'tex_data'):
            self.model.tex_data[self.tex_adr : self.tex_adr + self.size] = data_to_write
        elif hasattr(self.model, 'tex_rgb'):
             # Legacy/Float support
             self.model.tex_rgb[self.tex_adr : self.tex_adr + self.size] = data_to_write.astype(np.float32) / 255.0

        # Signal viewer to refresh this texture
        if viewer:
            try:
                viewer.update_texture(self.tex_id)
            except:
                pass