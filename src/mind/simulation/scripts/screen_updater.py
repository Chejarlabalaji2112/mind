
import cv2
import mujoco
import numpy as np
import time
import threading

# class ScreenUpdater:
#     def __init__(self, model, texture_name, update_fps=30):
#         self.model = model
#         self.tex_name = texture_name
#         self.update_fps = min(update_fps, 60)  # Cap to prevent overload
#         self.last_update_time = 0.0
#         self.updated = False  # For static images: update once
        
#         # --- 1. Locate Texture Memory ---
#         self.tex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, texture_name)
#         if self.tex_id == -1:
#             raise ValueError(f"Texture '{texture_name}' not found in XML.")
            
#         self.tex_adr = model.tex_adr[self.tex_id]
#         self.width = model.tex_width[self.tex_id]
#         self.height = model.tex_height[self.tex_id]
#         self.nchannel = model.tex_nchannel[self.tex_id]
#         self.size = self.width * self.height * self.nchannel
        
#         # Internal state
#         self.cap = None
#         self.frame_bgr = None
#         self.is_video = False
#         self.frame_count = 0  # For FPS testing

#     def load(self, file_path, resize_factor=1.0):
#         """Intelligently determines if file is Image or Video. Optional resize_factor for scalability."""
#         if self.cap: 
#             self.cap.release()
#             self.cap = None

#         # --- STEP 1: Try as Image FIRST ---
#         img = cv2.imread(file_path)
        
#         if img is not None:
#             # Resize if factor <1 for low-res optimization
#             if resize_factor != 1.0:
#                 h, w = int(img.shape[0] * resize_factor), int(img.shape[1] * resize_factor)
#                 img = cv2.resize(img, (w, h))
#             self.is_video = False
#             self.frame_bgr = self._resize(img)
#             print(f"Loaded STATIC IMAGE: {file_path} (resized to {self.frame_bgr.shape[:2] if resize_factor != 1.0 else 'original'})")
#             return

#         # --- STEP 2: If imread failed, try as Video ---
#         self.cap = cv2.VideoCapture(file_path)
#         if self.cap.isOpened():
#             ret, _ = self.cap.read()
#             if ret:
#                 self.is_video = True
#                 fps = self.cap.get(cv2.CAP_PROP_FPS)
#                 self.update_fps = min(fps or 30.0, 60.0)
#                 self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset
#                 # Optional: Set resolution for decode efficiency (not all codecs support)
#                 self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.width * resize_factor))
#                 self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.height * resize_factor))
#                 print(f"Loaded VIDEO STREAM: {file_path} ({self.update_fps:.1f} FPS)")
#                 return
        
#         raise ValueError(f"File cannot be read as Image or Video: {file_path}")

#     def update(self, viewer, data, profile=False):
#         """
#         Lightweight update. Throttled by real time for accurate FPS matching.
#         Optional profile: Logs timing/FPS for testing.
#         """
#         current_time = time.time()
#         if current_time - self.last_update_time < (1.0 / self.update_fps):
#             return

#         start_time = time.perf_counter() if profile else None

#         # Early exit for static after first upload
#         if not self.is_video and self.updated:
#             return

#         # 2. GET DATA
#         frame_data = None
        
#         if self.is_video:
#             # Read next frame
#             ret, frame = self.cap.read()
#             if not ret:
#                 # Video ended: Loop it, read frame 0 immediately
#                 self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
#                 ret, frame = self.cap.read()
#                 if not ret:
#                     print("Warning: Failed to loop video")
#                     return
#             frame_data = self._convert_to_mujoco(frame)
#             self.frame_count += 1
#         else:
#             frame_data = self._convert_to_mujoco(self.frame_bgr, resize_needed=False)

#         # 3. UPLOAD (Expensive part, now throttled)
#         if frame_data is not None:
#             self._write_memory(frame_data)
#             try:
#                 viewer.update_texture(self.tex_id)
#             except Exception as e:
#                 print(f"Texture update failed: {e}")
#                 return  # Skip timestamp on error
            
#             self.last_update_time = current_time
#             self.updated = True  # Mark for static
            
#             if profile:
#                 update_time = time.perf_counter() - start_time
#                 measured_fps = self.frame_count / (current_time - (self.last_update_time - (1.0 / self.update_fps) * self.frame_count)) if self.frame_count > 1 else 0
#                 print(f"Update #{self.frame_count}: {update_time*1000:.1f}ms | Measured FPS: {measured_fps:.1f} (target: {self.update_fps})")

#     def _resize(self, frame):
#         """Resize only to texture dims."""
#         if frame.shape[0] != self.height or frame.shape[1] != self.width:
#             return cv2.resize(frame, (self.width, self.height))
#         return frame

#     def _convert_to_mujoco(self, frame, resize_needed=True):
#         """Convert BGR to RGB/RGBA flat array."""
#         if resize_needed:
#             frame = self._resize(frame)
#         if self.nchannel == 3:
#             return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).flatten()
#         else:  # Assume 4 (RGBA); add alpha=255
#             bgr = frame[:, :, :3] if len(frame.shape) == 3 and frame.shape[2] == 4 else frame
#             bgra = cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA)
#             return bgra.flatten()

#     def _write_memory(self, flat_frame):
#         """Direct memory write with checks."""
#         if len(flat_frame) != self.size:
#             raise ValueError(f"Frame size {len(flat_frame)} != expected {self.size}")
#         if hasattr(self.model, 'tex_data'):
#             self.model.tex_data[self.tex_adr : self.tex_adr + self.size] = flat_frame
#         else:
#             # Legacy tex_rgb (may be float 0-1)
#             if self.model.tex_rgb.dtype == np.float32:
#                 self.model.tex_rgb[self.tex_adr : self.tex_adr + self.size] = flat_frame.astype(np.float32) / 255.0
#             else:
#                 self.model.tex_rgb[self.tex_adr : self.tex_adr + self.size] = flat_frame


class ScreenUpdater:
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

    def put_frame(self, frame_bgr):
        """
        Receive a generic BGR frame (from OpenCV or PyAV).
        This does NOT write to Mujoco yet (fast operation).
        """
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

    def put_text(self, text, bg_color=(0, 0, 0), text_color=(255, 255, 255), scale=1.0):
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
        self.put_frame(canvas)

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