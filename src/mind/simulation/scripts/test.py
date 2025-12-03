import time
import threading
import queue
import cv2
import numpy as np
import mujoco
import av  # PyAV
import sounddevice as sd
import mujoco.viewer

# ==============================================================================
# MODULE 1: MUJOCO TEXTURE DISPLAY (The Sink)
# ==============================================================================
class MujocoScreen:
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

# ==============================================================================
# MODULE 2: AUDIO MANAGER (Sound I/O via sounddevice)
# ==============================================================================
class AudioManager:
    """
    Handles Audio Input (Mic) and Output (Speakers) using sounddevice.
    Works natively with NumPy arrays.
    """
    def __init__(self, rate=44100, channels=2, dtype='int16'):
        self.rate = rate
        self.channels = channels
        self.dtype = dtype
        
        # Streams
        self.output_stream = None
        self.input_stream = None

    # --- OUTPUT (Playback) Methods ---
    def start_output_stream(self, rate=None, channels=None):
        """Initialize the speaker output stream."""
        # Update config if provided (e.g., from video file metadata)
        if rate: self.rate = rate
        if channels: self.channels = channels

        if self.output_stream is None:
            self.output_stream = sd.OutputStream(
                samplerate=self.rate,
                channels=self.channels,
                dtype=self.dtype
            )
            self.output_stream.start()

    def write_chunk(self, data_array):
        """
        Write numpy array to speakers.
        data_array shape should be (frames, channels).
        Blocks if buffer is full (syncing audio automatically).
        """
        if self.output_stream:
            try:
                self.output_stream.write(data_array)
            except Exception as e:
                print(f"Audio Write Error: {e}")

    # --- INPUT (Microphone) Methods ---
    def start_input_stream(self, device_index=None):
        """Initialize the microphone input stream."""
        if self.input_stream is None:
            self.input_stream = sd.InputStream(
                samplerate=self.rate,
                channels=self.channels,
                dtype=self.dtype,
                device=device_index
            )
            self.input_stream.start()

    def read_chunk(self, num_frames=1024):
        """
        Read 'num_frames' from microphone.
        Returns: numpy array of shape (num_frames, channels)
        """
        if self.input_stream:
            try:
                data, overflow = self.input_stream.read(num_frames)
                if overflow:
                    print("Audio Input Overflow")
                return data
            except Exception as e:
                print(f"Audio Read Error: {e}")
                return None
        return None

    def close(self):
        """Cleanup streams."""
        if self.output_stream:
            self.output_stream.stop()
            self.output_stream.close()
            self.output_stream = None
            
        if self.input_stream:
            self.input_stream.stop()
            self.input_stream.close()
            self.input_stream = None

# ==============================================================================
# MODULE 3: CAMERA SOURCE
# ==============================================================================
class CameraSource:
    """
    Standalone camera handler.
    """
    def __init__(self, device_index=0):
        self.cap = cv2.VideoCapture(device_index)
        
    def get_frame(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                return frame
        return None
        
    def release(self):
        self.cap.release()

# ==============================================================================
# MODULE 4: AV ORCHESTRATOR (The Logic Backend)
# ==============================================================================
class AVOrchestrator:
    """
    Reads a video file using PyAV.
    - Decodes Audio -> Sends to AudioManager (NumPy)
    - Decodes Video -> Sends to MujocoScreen
    - Handles Synchronization (Video follows Audio clock)
    """
    def __init__(self, screen_ref: MujocoScreen, audio_ref: AudioManager):
        self.screen = screen_ref
        self.audio = audio_ref
        self.running = False
        self.thread = None

    def play_file(self, file_path):
        """Starts the playback thread."""
        self.stop() # Stop any existing playback
        self.running = True
        self.thread = threading.Thread(target=self._decoding_loop, args=(file_path,))
        self.thread.start()

    def play_camera(self, device_index=0):
        """Starts camera passthrough thread."""
        self.stop()
        self.running = True
        self.thread = threading.Thread(target=self._camera_loop, args=(device_index,))
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            self.thread = None

    def _camera_loop(self, device_index):
        cam = CameraSource(device_index)
        fps_target = 30
        delay = 1.0 / fps_target
        
        while self.running:
            start = time.time()
            frame = cam.get_frame()
            if frame is not None:
                self.screen.put_frame(frame)
            
            # Simple throttle
            elapsed = time.time() - start
            if elapsed < delay:
                time.sleep(delay - elapsed)
        
        cam.release()

    def _decoding_loop(self, file_path):
        """
        Complex logic to sync Audio and Video using PyAV.
        Audio is played immediately (blocking write), Video is skipped/slept to match.
        """
        try:
            container = av.open(file_path)
        except Exception as e:
            print(f"Error opening file: {e}")
            self.running = False
            return
        
        # Setup streams
        video_stream = next((s for s in container.streams if s.type == 'video'), None)
        audio_stream = next((s for s in container.streams if s.type == 'audio'), None)
        
        if not video_stream:
            print("No video stream found.")
            return

        # Configure Audio Output if available
        if audio_stream:
            # sounddevice needs (frames, channels)
            # We configure the audio manager to match the file
            self.audio.start_output_stream(rate=audio_stream.rate, channels=audio_stream.channels)

        start_time = time.time()
        
        # Demuxer Loop
        try:
            for packet in container.demux(video_stream, audio_stream):
                if not self.running:
                    break
                    
                for frame in packet.decode():
                    # --- AUDIO HANDLING ---
                    if isinstance(frame, av.AudioFrame):
                        # Convert to numpy array
                        # format='s16' ensures int16 output which matches our default AudioManager dtype
                        # PyAV usually gives planar data (channels, frames), sounddevice wants (frames, channels)
                        # So we Transpose (.T)
                        data_array = frame.to_ndarray(format='s16').T
                        self.audio.write_chunk(data_array)
                        
                    # --- VIDEO HANDLING ---
                    elif isinstance(frame, av.VideoFrame):
                        # 1. Get current clock (relative to start)
                        current_wall_time = time.time() - start_time
                        pts = frame.time
                        
                        if pts > current_wall_time:
                            # Video is ahead of real time, sleep
                            time.sleep(pts - current_wall_time)
                            
                            # Convert AV frame to numpy BGR
                            img = frame.to_ndarray(format='bgr24')
                            self.screen.put_frame(img)
                            
                        else:
                            # Video is behind real time (lagging)
                            # Drop frame (don't send to screen) to catch up
                            if (current_wall_time - pts) < 0.05:
                                # If lag is small, display it anyway
                                img = frame.to_ndarray(format='bgr24')
                                self.screen.put_frame(img)
        except Exception as e:
            print(f"Playback error: {e}")
        finally:                  
            container.close()
            print("Playback finished.")

# ==============================================================================
# EXAMPLE USAGE
# ==============================================================================
if __name__ == "__main__":
    # 1. Setup Mujoco (Standard Boilerplate)
    # ---------------------------------------------------------
    xml = """
    <mujoco>
        <asset>
            <texture name="screen_tex" type="2d" builtin="checker" width="300" height="300" rgb1="0 0 0" rgb2="0 0 0"/>
            <material name="screen_mat" texture="screen_tex" texuniform="true"/>
        </asset>
        <worldbody>
            <body name="monitor" pos="0 0 1">
                <geom type="box" size="0.2 0.01 0.15" material="screen_mat" />
            </body>
            <light pos="0 0 3" dir="0 0 -1" />
        </worldbody>
    </mujoco>
    """
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)

    # 2. Initialize Modules
    # ---------------------------------------------------------
    # The Screen (Sink)
    screen = MujocoScreen(m, "screen_tex")
    
    # The Audio (Sink/Source) - Now using sounddevice
    audio = AudioManager(rate=44100, channels=2)
    
    # The Orchestrator (Backend)
    player = AVOrchestrator(screen, audio)

    # 3. Main Viewer Loop
    # ---------------------------------------------------------
    print("Starting... Press 'v' for Video, 'c' for Camera, 't' for Text")
    
    with mujoco.viewer.launch_passive(m, d) as viewer:
        start_t = time.time()
        
        screen.put_text("System Ready (SoundDevice)")

        while viewer.is_running():
            step_start = time.time()
            
            # Physics step
            mujoco.mj_step(m, d)
            
            # --- CRITICAL: UPDATE SCREEN ---
            screen.update(viewer)

            viewer.sync()

            # Example: Triggering actions based on time
            elapsed = time.time() - start_t
            if 2.0 < elapsed < 2.1:
               screen.put_text("Loading Video...")
            elif 4.0 < elapsed < 4.1:
                player.play_file("/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/my_video.mp4") 

            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    # Cleanup
    player.stop()
    audio.close()