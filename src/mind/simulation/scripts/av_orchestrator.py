# av_orchestrator.py (Fixed: Removed unreliable frame.stream check for audio)
import threading 
import time
import av  # PyAV
import numpy as np  # For dtype handling
from .screen_updater import ScreenUpdater as MujocoScreen
from adapters.audio_adapters import AudioManager
from adapters.camera_adapers import CameraSource

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

    def play_file(self, file_path, audio_track_index=0):
        """Starts the playback thread. Select audio track by index (0=first)."""
        if self.running:
            return 
        self.stop() # Stop any existing playback
        self.running = True
        self.thread = threading.Thread(target=self._decoding_loop, args=(file_path, audio_track_index))
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

    def _decoding_loop(self, file_path, audio_track_index=0):
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
        audio_streams = [s for s in container.streams if s.type == 'audio']
        
        if not video_stream:
            print("No video stream found.")
            return

        # Handle multiple audio tracks
        audio_stream = None
        if audio_streams:
            if audio_track_index >= len(audio_streams):
                print(f"Warning: Audio track index {audio_track_index} out of range (0-{len(audio_streams)-1}). Using track 0.")
                audio_track_index = 0
            audio_stream = audio_streams[audio_track_index]
            
            # Print available tracks for user awareness
            print("Available audio tracks:")
            for i, stream in enumerate(audio_streams):
                codec_name = stream.codec_context.codec.name if stream.codec_context else "Unknown"
                rate = stream.rate or "Unknown"
                channels = stream.channels or "Unknown"
                print(f"  Track {i}: {codec_name} ({channels}ch, {rate}Hz)")
            print(f"Selected: Track {audio_track_index}")

        # Configure Audio Output if available
        if audio_stream:
            self.audio.start_output_stream(rate=audio_stream.rate, channels=audio_stream.channels)

        start_time = time.time()
        audio_start_time = None  # Track when first audio chunk plays for better sync
        
        # Demuxer Loop (demux video + selected audio only)
        streams_to_demux = [video_stream]
        if audio_stream:
            streams_to_demux.append(audio_stream)
        
        try:
            for packet in container.demux(*streams_to_demux):
                if not self.running:
                    break
                    
                for frame in packet.decode():
                    # --- AUDIO HANDLING ---
                    if isinstance(frame, av.AudioFrame):  # FIX: Removed 'frame.stream == audio_stream' (redundant with demux selection)
                        # No 'format' arg! Get native NumPy array (planar: (channels, samples))
                        data_array = frame.to_ndarray()
                        
                        # Ensure int16 for AudioManager (scale if float -1..1)
                        if data_array.dtype.kind == 'f':  # float32/etc.
                            data_array = np.clip(data_array * 32767, -32768, 32767).astype(np.int16)
                        else:
                            data_array = data_array.astype(np.int16)
                        
                        # Transpose: PyAV planar (channels, samples) → sounddevice interleaved (samples, channels)
                        data_array = data_array.T
                        
                        # Ensure C-contiguous for sounddevice.write()
                        data_array = np.ascontiguousarray(data_array)
                        
                        if audio_start_time is None:
                            audio_start_time = time.time()  # Set on first audio
                        
                        self.audio.write_chunk(data_array)
                        
                    # --- VIDEO HANDLING ---
                    elif isinstance(frame, av.VideoFrame):
                        # 1. Get current clock (relative to start)
                        current_wall_time = time.time() - start_time
                        # Tie to audio time if available for better sync
                        current_audio_time = (audio_start_time - start_time) + (time.time() - audio_start_time) if audio_start_time else current_wall_time
                        pts = frame.time  # PTS in seconds
                        
                        if pts > current_audio_time:
                            # Video ahead: sleep to sync
                            time.sleep(pts - current_audio_time)
                            
                            # Convert to BGR NumPy (OpenCV-friendly)
                            img = frame.to_ndarray(format='bgr24')
                            self.screen.put_frame(img)
                            
                        else:
                            # Video lagging: drop to catch up, unless tiny lag
                            lag = current_audio_time - pts
                            if lag < 0.05:  # 50ms tolerance
                                img = frame.to_ndarray(format='bgr24')
                                self.screen.put_frame(img)
                                # Optional: Print lag for debugging
                                # print(f"Minor lag: {lag*1000:.1f}ms")
        except Exception as e:
            print(f"Playback error: {e}")
        finally:                  
            container.close()
            if audio_stream:
                self.audio.close()  # Ensure stream cleanup
            print("Playback finished.")