import mujoco
import mujoco.viewer
import time
import numpy as np
import os
import threading
import queue
from enum import Enum, auto

# Adapters and Utils
from mind.simulation.scripts.screen_updater import ScreenUpdater
from mind.simulation.scripts.av_orchestrator import AVOrchestrator
from mind.adapters.audio_adapters.sd_adapter import AudioManager
from mind.adapters.camera_handler import CameraSource
from mind.simulation.scripts.motion_controller import MotionController
from mind.utils.robo_eyes import RoboEyes, HAPPY

#constants
from mind.utils import SIMULATION_DIR


# Paths (Keep these configurable or constants)
XML_PATH = f"{SIMULATION_DIR}/description/scene.xml"
CLOSE_AUDIO_PATH = f"{SIMULATION_DIR}/media/audio/shutdown.mp3"

class RobotCommand(Enum):
    WAKE_UP = auto()
    SHUT_DOWN = auto()
    PLAY_VIDEO = auto()
    SET_EYES = auto()
    UPDATE_SCREEN = auto()
    TIMER = auto()


class MujocoBackend:
    def __init__(self, scene_path=XML_PATH):
        # 1. MuJoCo Model Setup
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        
        # Keyframe IDs
        self.home_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        self.open_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "open")

        # Initial Pose
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.home_id)
        mujoco.mj_forward(self.model, self.data)

        # 2. Components
        self.screen_top = ScreenUpdater(self.model, "display_top")
        self.screen_bottom = ScreenUpdater(self.model, "display_bottom")
        self.audio = AudioManager(rate=44100, channels=2)
        self.player = AVOrchestrator(self.screen_top, self.audio)
        self.eyes = RoboEyes(self.screen_top, width=512, height=512)
        self.motion = MotionController(self.model)

        # 3. Control Flags
        self._command_queue = queue.Queue()
        self._stop_event = threading.Event()
        self._is_ready_event = threading.Event() # Changed to an Event for thread safety

        self.model.opt.timestep = 0.002

        self.show_clock = True




    # -----------------------------------------------------
    # LIFECYCLE API
    # -----------------------------------------------------
    def run(self):
        """
        Blocking method that runs the MuJoCo viewer and physics loop.
        Optimized to decouple Physics (500Hz) from Rendering (30Hz).
        """
        print("[Backend] Launching Viewer...")
        
        with mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=True
        ) as viewer:
            
            # Initial Camera Setup
            with viewer.lock():
                viewer.cam.lookat[:] = np.array([0.0, 0.05, 0.1])
                viewer.cam.azimuth = 0.0
                viewer.cam.elevation = -30.0
                viewer.cam.distance = 0.3
            
            self._is_ready_event.set()
            
            # Timers
            last_clock_update = 0
            last_render_time = 0
            render_fps = 30  # Limit rendering to 30 FPS to save resources
            render_interval = 1.0 / render_fps

            loop_times = []

            # --- THE MAIN LOOP ---
            while viewer.is_running() and not self._stop_event.is_set():
                step_start = time.perf_counter()
                
                # ------------------------------------
                # 1. Process Commands (Fast)
                # ------------------------------------
                try:
                    while not self._command_queue.empty():
                        cmd_type, payload = self._command_queue.get_nowait()
                        self._handle_command(cmd_type, payload, viewer)
                except queue.Empty:
                    pass

                # ------------------------------------
                # 2. Physics Step (Target: 500Hz)
                # ------------------------------------
                self.motion.update_motion(self.data)
                mujoco.mj_step(self.model, self.data)
                
                # Update Eyes Logic (Calculation only, not rendering)
                if self.eyes.is_active:
                    self.eyes.update()

                # ------------------------------------
                # 3. Rendering (Throttled to 30-60Hz)
                # ------------------------------------
                now = time.time()
                
                # Only sync viewer if enough time has passed
                if now - last_render_time >= render_interval:
                    last_render_time = now
                    
                    # Clock Update
                    if self.show_clock:
                        if now - last_clock_update >= 1.0:
                            last_clock_update = now
                            self.screen_bottom.show_text(time.strftime("%H:%M:%S"))

                    # Screen Texture Updates (The heavy part)
                    self.screen_top.update(viewer)
                    self.screen_bottom.update(viewer)
                    
                    # GPU Sync
                    viewer.sync()

                # ------------------------------------
                # 4. Frequency Control (Crucial!)
                # ------------------------------------
                # We intentionally sleep to keep physics real-time (500Hz)
                # otherwise it might run too fast (e.g. 2000Hz)
                step_end = time.perf_counter()
                process_time = step_end - step_start
                
                # Sleep the remainder of the 2ms timestep
                time_to_sleep = self.model.opt.timestep - process_time
                if time_to_sleep > 0:
                    time.sleep(time_to_sleep)
                    

                # ------------------------------------
                # 5. FPS Calculation
                # # ------------------------------------
                loop_end = time.perf_counter()
                dt = loop_end - step_start
                loop_times.append(dt)
                if len(loop_times) > 100:
                    loop_times.pop(0)

                avg_dt = sum(loop_times) / len(loop_times)
                loop_hz = 1.0 / avg_dt
                print(f"\rLoop frequency: {loop_hz:.2f} Hz", end="")

            self._cleanup()

    def stop(self):
        """Signals the loop to exit."""
        self._stop_event.set()

    def wait_until_ready(self, timeout=None):
        """Blocks caller until the viewer is actually running."""
        return self._is_ready_event.wait(timeout)

    # -----------------------------------------------------
    # COMMAND API (Thread-Safe)
    # -----------------------------------------------------
    def wake_up(self, duration=4.0):
        self._command_queue.put((RobotCommand.WAKE_UP, {"duration": duration}))

    def shut_down_animation(self, duration=5.0):
        self._command_queue.put((RobotCommand.SHUT_DOWN, {"duration": duration}))

    def play_video(self, video_path):
        self._command_queue.put((RobotCommand.PLAY_VIDEO, {"path": video_path}))

    def set_eye_mode(self, active=True):
        self._command_queue.put((RobotCommand.SET_EYES, {"active": active}))
        
    def show_text_bottom(self, text):
        self._command_queue.put((RobotCommand.UPDATE_SCREEN, {"text": text}))

    # -----------------------------------------------------
    # INTERNAL HELPERS
    # -----------------------------------------------------
    def _handle_command(self, cmd_type, payload, viewer):
            """Internal command router"""
            if cmd_type == RobotCommand.WAKE_UP:
                qpos_open = self.model.key_qpos[self.open_id]
                ctrl_open = self.model.key_ctrl[self.open_id]
                self.motion.do_open(self.data, qpos_open, ctrl_open, payload['duration'])
                
            elif cmd_type == RobotCommand.SHUT_DOWN:
                print("[Backend] Playing Shutdown Sequence...")
                self.player.stop()
                self.eyes.is_active = False
                self.screen_top.show_text("Goodbye...")
                if os.path.exists(CLOSE_AUDIO_PATH):
                    self.player.play_file(CLOSE_AUDIO_PATH)
                
                self.motion.do_close(
                    self.data, 
                    self.model.key_qpos[self.home_id], 
                    self.model.key_ctrl[self.home_id], 
                    duration=payload['duration']
                )

            elif cmd_type == RobotCommand.PLAY_VIDEO:
                self.player.play_file(payload['path'])

            elif cmd_type == RobotCommand.SET_EYES:
                active = payload['active']
                if active:
                    if not self.eyes.is_active:
                        self.eyes.is_active = True
                        self.eyes.begin()
                        self.eyes.set_auto_blink(True)
                        self.eyes.set_idle_mode(True)
                else:
                    self.eyes.is_active = False

            elif cmd_type == RobotCommand.UPDATE_SCREEN:
                self.screen_bottom.show_text(payload['text'])

    def _cleanup(self):
        print("[Backend] Cleaning up resources...")
        self.player.stop()
        self.audio.close()
        self._is_ready_event.clear()