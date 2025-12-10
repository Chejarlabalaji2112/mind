import os
import time
import queue
import mujoco
import threading
import numpy as np
import mujoco.viewer
from enum import Enum, auto
from mind.utils import SIMULATION_DIR
from mind.utils.robo_eyes import RoboEyes, HAPPY
from mind.utils.logging_handler import setup_logger
from mind.adapters.camera_handler import CameraSource
from mind.adapters.audio_adapters.sd_adapter import AudioManager
from mind.simulation.scripts.screen_updater import ScreenUpdater        # Adapters and Utils
from mind.simulation.scripts.av_orchestrator import AVOrchestrator
from mind.ports.base_robot_controller_port import BaseRobotController
from mind.simulation.scripts.motion_controller import MotionController


# Paths (Keep these configurable or constants)
XML_PATH = f"{SIMULATION_DIR}/description/scene.xml"
CLOSE_AUDIO_PATH = f"{SIMULATION_DIR}/media/audio/shutdown.mp3"
SLEEP_AUDIO_PATH = f"{SIMULATION_DIR}/media/audio/sleep.mp3"  # Add if available

logger = setup_logger(__name__)

class RobotStatus(Enum):  # Mirror backend enum
    SHUTDOWN = "shutdown"
    SLEEP = "sleep"
    ACTIVE = "active"

class RobotCommand(Enum):
    WAKE_UP = auto()
    SHUT_DOWN = auto()
    SLEEP = auto()  # New
    PLAY_VIDEO = auto()
    SET_EYES = auto()
    UPDATE_SCREEN = auto()
    TIMER = auto()


class MujocoRobot(BaseRobotController):
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
        self.motion = MotionController(self.model, robot_controller=self)

        # 3. Control Flags
        self._command_queue = queue.Queue()
        self._stop_event = threading.Event()
        self._is_ready_event = threading.Event()

        # New: Status Tracking (thread-safe)
        self._status_lock = threading.Lock()
        self._current_status = RobotStatus.SHUTDOWN  # Start shutdown

        self.model.opt.timestep = 0.002
        self.show_clock = True

        self._is_opened = True

    # -----------------------------------------------------
    # LIFECYCLE API
    # -----------------------------------------------------
    def run(self):
        """
        Blocking method that runs the MuJoCo viewer and physics loop.
        Optimized to decouple Physics (500Hz) from Rendering (30Hz).
        """
        logger.info("Launching viewer")
        
        with mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
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
                # 5. FPS Calculation (commented)
                # ------------------------------------

            self._cleanup()


    def stop(self):
            """Signals the loop to exit and ensures resources are freed."""
            logger.info("Stop signal received. Closing viewer...")
            
            # 1. Force the loop in run() to break
            self._stop_event.set()
            
            # 2. Ensure the status is updated so the frontend knows we are down
            self.set_status(RobotStatus.SHUTDOWN)



    def wait_until_ready(self, timeout=None):
        """Blocks caller until the viewer is actually running."""
        return self._is_ready_event.wait()

    # -----------------------------------------------------
    # COMMAND API (Thread-Safe)
    # -----------------------------------------------------
    def wake_up(self, duration=4.0):
        self.set_status(RobotStatus.ACTIVE)
        self._command_queue.put((RobotCommand.WAKE_UP, {"duration": duration}))

    def sleep(self, duration=3.0):  # New impl
        self.set_status(RobotStatus.SLEEP)
        self._command_queue.put((RobotCommand.SLEEP, {"duration": duration}))

    def status(self):
        with self._status_lock:
            return self._current_status.value  # Return string for backend compat

    def shutdown(self, duration=5.0):
        self.set_status(RobotStatus.SHUTDOWN)
        self._command_queue.put((RobotCommand.SHUT_DOWN, {"duration": duration}))

    def play_video(self, video_path):
        self._command_queue.put((RobotCommand.PLAY_VIDEO, {"path": video_path}))

    def set_eye_mode(self, active=True):
        self._command_queue.put((RobotCommand.SET_EYES, {"active": active}))
        
    def show_text_bottom(self, text):
        self._command_queue.put((RobotCommand.UPDATE_SCREEN, {"text": text}))

    # New: Get current status (for polling if needed)
    def get_status(self):
        return self.status()

    # Internal: Set status (thread-safe)
    def set_status(self, status: RobotStatus):
        with self._status_lock:
            self._current_status = status

    # -----------------------------------------------------
    # INTERNAL HELPERS
    # -----------------------------------------------------
    def _handle_command(self, cmd_type, payload, viewer):
        """Internal command router"""
        if cmd_type == RobotCommand.WAKE_UP: 
            qpos_open = self.model.key_qpos[self.open_id]
            ctrl_open = self.model.key_ctrl[self.open_id]
            self.motion.do_open(self.data, qpos_open, ctrl_open, payload['duration'])
            # Timer for completion (optional: reset if interrupted)
            
        elif cmd_type == RobotCommand.SLEEP:  # New
            logger.info("Entering sleep mode")
            self.eyes.is_active = False  # Dim eyes
            self.screen_top.show_text("Zzz...")
            if os.path.exists(SLEEP_AUDIO_PATH):
                self.player.play_file(SLEEP_AUDIO_PATH)
            # Gentle close pose
            self.motion.do_close(
                self.data, 
                self.model.key_qpos[self.home_id], 
                self.model.key_ctrl[self.home_id], 
                duration=payload['duration']
            )

        elif cmd_type == RobotCommand.SHUT_DOWN:
            logger.info("Starting shutdown sequence")
            self.player.stop()
            self.eyes.is_active = False
            self.screen_top.show_text("Goodbye...")
            if (self._is_opened):    
                if os.path.exists(CLOSE_AUDIO_PATH):
                    self.player.play_file(CLOSE_AUDIO_PATH)
                
                self.motion.do_close(
                    self.data, 
                    self.model.key_qpos[self.home_id], 
                    self.model.key_ctrl[self.home_id], 
                    duration=payload['duration']
                )
            # After duration, signal stop if needed (but stop() handles it)

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
        logger.info("Cleaning up resources")
        self.player.stop()
        self.audio.close()
        self.set_status(RobotStatus.SHUTDOWN) #I think tehre are so many set_status of shutdown..
        self._is_ready_event.clear()