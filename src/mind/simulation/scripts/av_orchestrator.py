# av_orchestrator.py (Updated for modular display fallback + audio-only handling)
import threading 
import time
import av  # PyAV
import numpy as np  # For dtype handling
import cv2  # For fallback
from mind.simulation.scripts.screen_updater import ScreenUpdater  # Optional MuJoCo import
from mind.simulation.scripts.cv_display import CvDisplay  # Fallback import
from mind.adapters.audio_adapters.sd_adapter import AudioManager
from mind.adapters.camera_handler import CameraSource
from mind.adapters.display_adapters import DisplayObj

class AVOrchestrator:
    """
    Modular AV player: Handles audio/video files (any combo).
    - Audio: Always plays via AudioManager.
    - Video: Uses provided display (MuJoCo texture or OpenCV window).
    - Sync: Video follows audio clock.
    """
    def __init__(self, display_obj: DisplayObj = None, audio_ref: AudioManager = None):
        self.audio = audio_ref or AudioManager(rate=44100, channels=2)
        self.display = display_obj  # Generic: Must have show() and update()
        self.no_video_playing = True
        
        # Auto-setup display if none provided
        if display_obj is None:
            try:
                # Try MuJoCo context (import optional)
                import mujoco
                model = mujoco.MjModel.from_xml_path("default.xml")  # Placeholder; user provides in combined.py
                self.display = ScreenUpdater(model, "display_top")
                print("Using MuJoCo texture display.")
            except (ImportError, FileNotFoundError, ValueError):
                self.display = CvDisplay("AV Playback")
                print("Using OpenCV fallback window.")
        
        self.running = False
        self.thread = None

    def play_file(self, file_path, audio_track_index=0):
        """Play any file: audio-only, video-only, or A/V. Modular entry point."""
        self.stop()
        self.running = True
        self.thread = threading.Thread(target=self._decoding_loop, args=(file_path, audio_track_index))
        self.thread.start()
        self.no_video_playing = False

    def play_camera(self, device_index=0):
        """Camera passthrough (video-only)."""
        self.stop()
        self.running = True
        self.thread = threading.Thread(target=self._camera_loop, args=(device_index,))
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
            self.thread = None
        if hasattr(self.display, 'close'):
            self.display.close()
        self.audio.close()

    def _camera_loop(self, device_index):
        cam = CameraSource(device_index)
        fps_target = 30
        delay = 1.0 / fps_target
        
        while self.running:
            start = time.time()
            frame = cam.get_frame()
            if frame is not None:
                self.display.show(frame)
            elapsed = time.time() - start
            if elapsed < delay:
                time.sleep(delay - elapsed)
            if hasattr(self.display, 'update'):  # Call update for display
                self.display.update()
        
        cam.release()

    def _decoding_loop(self, file_path, audio_track_index=0):
        try:
            container = av.open(file_path)
        except Exception as e:
            print(f"Error opening file: {e}")
            self.running = False
            return
        
        # Setup streams
        video_stream = next((s for s in container.streams if s.type == 'video'), None)
        audio_streams = [s for s in container.streams if s.type == 'audio']
        
        has_video = video_stream is not None
        has_audio = bool(audio_streams)
        
        if not has_video and not has_audio:
            print("No audio or video streams found.")
            return

        # Handle audio tracks (if any)
        audio_stream = None
        if has_audio:
            if audio_track_index >= len(audio_streams):
                print(f"Warning: Audio track {audio_track_index} out of range. Using 0.")
                audio_track_index = 0
            audio_stream = audio_streams[audio_track_index]
            print("Available audio tracks:")
            for i, stream in enumerate(audio_streams):
                codec_name = stream.codec_context.codec.name if stream.codec_context else "Unknown"
                rate = stream.rate or "Unknown"
                channels = stream.channels or "Unknown"
                print(f"  Track {i}: {codec_name} ({channels}ch, {rate}Hz)")
            print(f"Selected: Track {audio_track_index}")
            self.audio.start_output_stream(rate=audio_stream.rate, channels=audio_stream.channels)

        # Setup display loop if video (in thread or main; here we assume main calls display.update())
        if has_video:
            print("Video playback enabled.")
        else:
            self.no_video_playing = True
            print("Audio-only playback (no display updates).")

        start_time = time.time()
        audio_start_time = None
        
        # Demux streams
        streams_to_demux = [s for s in [video_stream, audio_stream] if s]
        
        try:
            for packet in container.demux(*streams_to_demux):
                if not self.running:
                    break
                    
                for frame in packet.decode():
                    # --- AUDIO HANDLING (if present) ---
                    if isinstance(frame, av.AudioFrame):
                        data_array = frame.to_ndarray()
                        if data_array.dtype.kind == 'f':
                            data_array = np.clip(data_array * 32767, -32768, 32767).astype(np.int16)
                        else:
                            data_array = data_array.astype(np.int16)
                        data_array = data_array.T
                        data_array = np.ascontiguousarray(data_array)
                        
                        if audio_start_time is None:
                            audio_start_time = time.time()
                        
                        self.audio.write_chunk(data_array)
                        
                    # --- VIDEO HANDLING (if present) ---
                    elif isinstance(frame, av.VideoFrame) and has_video:
                        current_wall_time = time.time() - start_time
                        current_audio_time = (audio_start_time - start_time) + (time.time() - audio_start_time) if audio_start_time else current_wall_time
                        pts = frame.time
                        
                        if pts > current_audio_time:
                            time.sleep(pts - current_audio_time)
                            img = frame.to_ndarray(format='bgr24')
                            self.display.show(img)
                        else:
                            lag = current_audio_time - pts
                            if lag < 0.05:
                                img = frame.to_ndarray(format='bgr24')
                                self.display.show(img)
        except Exception as e:
            print(f"Playback error: {e}")
        finally:
            container.close()
            self.no_video_playing = True
            if has_audio:
                self.audio.close()
            print("Playback finished.")