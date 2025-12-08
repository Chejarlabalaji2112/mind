import mujoco
import mujoco.viewer
import numpy as np
import time
import cv2
from PIL import Image  # For robust PNG loading (handles alpha, transparency
# Load the model from XML string
model = mujoco.MjModel.from_xml_path("/home/badri/mine/hitomi/mind/simulation/description/scene.xml")
data = mujoco.MjData(model)


# Find texture
texid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, b'display_top')
if texid < 0:
    raise RuntimeError("Texture 'display_tex' not found!")

width  = model.tex_width[texid]
height = model.tex_height[texid]

# Compute channels & address
if texid < model.ntex - 1:
    tex_size = model.tex_adr[texid + 1] - model.tex_adr[texid]
else:
    tex_size = model.ntexdata - model.tex_adr[texid]
channels = tex_size // (width * height)
print(f"Texture: {width}×{height}, {channels} channels")

# ==================== VIDEO READER ====================
VIDEO_PATH = "/home/badri/mine/hitomi/mind/simulation/videos/shin.mp4"          # ← CHANGE THIS
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    raise RuntimeError(f"Cannot open video: {VIDEO_PATH}")

# Optional: match video FPS for smooth playback
fps = cap.get(cv2.CAP_PROP_FPS) or 30
frame_delay = 1.0 / fps

# ==================== TEXTURE UPDATE FUNCTION ====================
def set_frame_texture(frame_bgr):
    """Convert OpenCV BGR frame → correct RGB/RGBA and write to MuJoCo texture"""
    # Resize if needed (highly recommended to avoid slowdowns)
    frame = cv2.resize(frame_bgr, (width, height), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)          # OpenCV uses BGR

    if channels == 3:
        data = rgb
    elif channels == 4:
        # Add full opacity alpha channel
        alpha = np.full((height, width, 1), 255, dtype=np.uint8)
        data = np.dstack((rgb, alpha))
    else:
        raise RuntimeError("Unsupported texture channels")

    offset = model.tex_adr[texid]
    model.tex_data[offset:offset + tex_size] = data.ravel()

# ==================== MAIN LOOP WITH VIEWER ====================
sim_timestep = model.opt.timestep
steps_per_video_frame = int(frame_delay / sim_timestep)

print(f"Syncing: Running {steps_per_video_frame} physics steps per video frame.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    next_frame_time = start_time

    while viewer.is_running():
        # 1. READ VIDEO
        ret, frame = cap.read()
        if not ret:
            # Handle Loop
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        # 2. UPDATE TEXTURE
        set_frame_texture(frame)
        viewer.update_texture(texid)

        # 3. PHYSICS SUB-STEPPING (The Fix)
        # We catch up the physics to match the time passed in the video
        for _ in range(steps_per_video_frame):
            mujoco.mj_step(model, data)

        # 4. RENDER
        viewer.sync()

        # 5. WALL CLOCK SYNC (Keep video from playing too fast)
        next_frame_time += frame_delay
        sleep_time = next_frame_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

cap.release()
print("Done")


import cv2
import mujoco
import numpy as np

class UniversalScreen:
    def __init__(self, model, texture_name):
        self.model = model
        self.tex_name = texture_name
        self.is_video = False
        self.cap = None
        self.image_frame = None
        self.fps = 30.0  # Default fallback
        
        # --- Locate Texture in Memory ---
        self.tex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, texture_name)
        if self.tex_id == -1:
            raise ValueError(f"Texture '{texture_name}' not found defined in XML.")
            
        self.tex_adr = model.tex_adr[self.tex_id]
        self.width = model.tex_width[self.tex_id]
        self.height = model.tex_height[self.tex_id]
        self.size = self.width * self.height * 3

    def load(self, file_path):
        """Loads a new image or video file."""
        # Release previous video if exists
        if self.cap:
            self.cap.release()
            
        # Try opening as video first
        self.cap = cv2.VideoCapture(file_path)
        
        # Check if it's actually a video with frames
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.is_video = True
                self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0) # Reset to start
                print(f"Loaded VIDEO: {file_path} ({self.fps} FPS)")
                return
        
        # If not a video (or read failed), treat as Image
        self.is_video = False
        self.image_frame = cv2.imread(file_path)
        if self.image_frame is None:
            raise ValueError(f"Could not load file as image or video: {file_path}")
        print(f"Loaded IMAGE: {file_path}")
        
        # Pre-process image immediately
        self.image_frame = self._process_frame(self.image_frame)

    def update(self, viewer, data):
        """
        Call this every simulation step. 
        data: needed for video synchronization (time)
        """
        frame_to_upload = None

        if self.is_video:
            # --- Video Sync Logic ---
            # Calculate which frame should be visible at this exact simulation time
            current_sim_time = data.time
            target_frame_idx = int(current_sim_time * self.fps)
            
            # Get current position of video reader
            current_video_idx = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
            
            # Only read a new frame if sim time has caught up to the next video frame
            if target_frame_idx > current_video_idx:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame_idx)
                ret, frame = self.cap.read()
                if ret:
                    frame_to_upload = self._process_frame(frame)
                else:
                    # Optional: Loop video
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        else:
            # Static Image: Just return the pre-loaded frame
            # (We only need to upload it once, but repeating is safe)
            frame_to_upload = self.image_frame

        # --- Upload to GPU ---
        if frame_to_upload is not None:
            self._write_to_memory(frame_to_upload)
            viewer.update_texture(self.tex_id)

    def _process_frame(self, frame):
        """Resizes and converts BGR (OpenCV) to RGB (MuJoCo)"""
        frame = cv2.resize(frame, (self.width, self.height))
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).flatten()

    def _write_to_memory(self, frame_flat):
        """Writes raw bytes to MuJoCo model memory"""
        if hasattr(self.model, 'tex_data'):
            self.model.tex_data[self.tex_adr : self.tex_adr + self.size] = frame_flat
        else:
            self.model.tex_rgb[self.tex_adr : self.tex_adr + self.size] = frame_flat