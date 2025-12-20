import os
import time
import queue
import asyncio
import mujoco
import threading
import numpy as np
import mujoco.viewer
from enum import Enum, auto
from mind.utils import Event
from mind.utils import SIMULATION_DIR
from mind.utils.robo_eyes import RoboEyes, DEFAULT, TIRED, ANGRY ,HAPPY, N, S, E, W, NW, SW, SE, NE
from mind.utils.logging_handler import setup_logger
from mind.adapters.vision_adapters.camera_handler import CameraSource
from mind.adapters.audio_adapters.sd_adapter import AudioManager
from mind.simulation.scripts.screen_updater import ScreenUpdater
from mind.simulation.scripts.av_orchestrator import AVOrchestrator
from mind.core.ports.base_robot_controller_port import BaseRobotController
from mind.simulation.scripts.motion_controller import MotionController

XML_PATH         = f"{SIMULATION_DIR}/description/scene.xml"
CLOSE_AUDIO_PATH = f"{SIMULATION_DIR}/media/audio/shutdown.mp3"
SLEEP_AUDIO_PATH = f"{SIMULATION_DIR}/media/audio/sleep.mp3"
BOOT_VIDEO_PATH  = f"{SIMULATION_DIR}/media/videos/pupil_boot.mp4"

logger = setup_logger(__name__)

class RobotStatus(Enum):
    SHUTDOWN = "shutdown"
    SLEEP = "sleep"
    ACTIVE = "active"

class RobotCommand(Enum):
    WAKE_UP = auto()
    SHUT_DOWN = auto()
    SLEEP = auto()
    PLAY_VIDEO = auto()
    SET_EYES = auto()
    UPDATE_SCREEN = auto()
    MOTION_COMPLETE = auto()
    BOOT_COMPLETE = auto()
    SET_EYES_MODE = auto()
    SET_EYES_POS = auto()

class MujocoRobot(BaseRobotController):
    # 1. Update Init to accept event_bus
    def __init__(self, scene_path=XML_PATH, loop=None):
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        
        self.home_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        self.open_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "open")

        mujoco.mj_resetDataKeyframe(self.model, self.data, self.home_id)
        mujoco.mj_forward(self.model, self.data)

        self.screen_top = ScreenUpdater(self.model, "display_top")
        self.screen_bottom = ScreenUpdater(self.model, "display_bottom")
        self.audio = AudioManager(rate=44100, channels=2)
        self.player = AVOrchestrator(self.screen_top, self.audio)
        self.eyes = RoboEyes(self.screen_top, width=512, height=512)
        self.motion = MotionController(self.model, robot_controller=self)

        self._command_queue = queue.Queue()
        self._stop_event = threading.Event()
        self._is_ready_event = threading.Event()

        self._status_lock = threading.Lock()
        self._current_status = RobotStatus.SHUTDOWN 
        

        self.model.opt.timestep = 0.002
        self.show_clock = True
        self._boot_played = False
        self._is_opened = False

        self.status_update_event = Event(loop=loop)

        self._requested_state = None

    # ... [run, stop, wait_until_ready methods remain unchanged] ...

    def run(self):
        # (Copy the run method logic from your original file here exactly as is)
        # For brevity in this response, I am not repeating the full run loop code 
        # as it doesn't change, but ensure you keep it.
        logger.info("Launching viewer")
        with mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False) as viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = np.array([0.0, 0.05, 0.1])
                viewer.cam.azimuth = 0.0
                viewer.cam.elevation = -30.0
                viewer.cam.distance = 0.3
            self._is_ready_event.set()
            last_clock_update = 0
            last_render_time = 0
            render_fps = 30
            render_interval = 1.0 / render_fps

            while viewer.is_running() and not self._stop_event.is_set():
                step_start = time.perf_counter()
                try:
                    while not self._command_queue.empty():
                        cmd_type, payload = self._command_queue.get_nowait()
                        self._handle_command(cmd_type, payload, viewer)
                except queue.Empty:
                    pass

                self.motion.update_motion(self.data)
                mujoco.mj_step(self.model, self.data)
                
                if self.eyes.is_active:
                    self.eyes.update()

                now = time.time()
                if now - last_render_time >= render_interval:
                    last_render_time = now
                    if self.show_clock:
                        if now - last_clock_update >= 1.0:
                            last_clock_update = now
                            self.screen_bottom.show_text(time.strftime("%H:%M:%S"))
                    self.screen_top.update(viewer)
                    self.screen_bottom.update(viewer)
                    viewer.sync()

                step_end = time.perf_counter()
                process_time = step_end - step_start
                time_to_sleep = self.model.opt.timestep - process_time
                if time_to_sleep > 0:
                    time.sleep(time_to_sleep)
            self._cleanup()
    
    def stop(self):
        logger.info("Stop signal received. Closing viewer...")
        self._stop_event.set()
        # Note: We rely on _cleanup to set final shutdown status

    def wait_until_ready(self, timeout=None):
        return self._is_ready_event.wait(timeout=timeout)

    # -----------------------------------------------------
    # COMMAND API
    # Removed explicit set_status calls here!
    # -----------------------------------------------------

    def wake_up(self, duration=4.0):
        self._command_queue.put((RobotCommand.WAKE_UP, {"duration": duration}))
        self.screen_top.clear_display()
        logger.info(f"Wake-up queued")
    
    def sleep(self, duration=3.0):
        self._requested_state = RobotStatus.SLEEP
        self._command_queue.put((RobotCommand.SLEEP, {"duration": duration}))

    def shutdown(self, duration=5.0):
        self._requested_state = RobotStatus.SHUTDOWN
        self._boot_played = False
        self._command_queue.put((RobotCommand.SHUT_DOWN, {"duration": duration}))

    # ... [play_video, set_eye_mode, show_text_bottom, motion_complete remain same] ...
    def play_video(self, video_path):
        self._command_queue.put((RobotCommand.PLAY_VIDEO, {"path": video_path}))

    def set_eye_mode(self, active=True):
        self._command_queue.put((RobotCommand.SET_EYES, {"active": active}))
        
    def show_text_bottom(self, text):
        self._command_queue.put((RobotCommand.UPDATE_SCREEN, {"text": text}))

    def motion_complete(self, motion_type ):
        if motion_type == 'open':
            self._is_opened = True
            self.set_status(RobotStatus.ACTIVE)
        elif motion_type == 'close':
            self._is_opened = False
            if self._requested_state == RobotStatus.SLEEP :
                self.set_status(RobotStatus.SLEEP)
            elif  self._requested_state == RobotStatus.SHUTDOWN:
                self.set_status(RobotStatus.SHUTDOWN)
                
        self._command_queue.put((RobotCommand.MOTION_COMPLETE, None))

    def status(self):
        with self._status_lock:
            return self._current_status.value
        
    def set_eyes_mode(self, mode):
        self._command_queue.put((RobotCommand.SET_EYES_MODE, {"mode": mode}))

    def set_eyes_position(self, position):
        self._command_queue.put((RobotCommand.SET_EYES_POS, {"position": position}))

    # -----------------------------------------------------
    # INTERNAL HELPERS (Where status changes actually happen)
    # -----------------------------------------------------
    
    def set_status(self, status: RobotStatus):
        with self._status_lock:
            old_status = self._current_status
            self._current_status = status
            
            # Emit event if status changed
            if old_status != status and self.status_update_event:
                logger.info(f"EMITTING STATUS: {status.value}")
                self.status_update_event.emit(status.value)

    def _handle_command(self, cmd_type, payload, viewer):
        if cmd_type == RobotCommand.WAKE_UP: 
            duration = payload.get('duration', 4.0)
            self.screen_top.clear_display() 
            qpos_open = self.model.key_qpos[self.open_id]
            ctrl_open = self.model.key_ctrl[self.open_id]
            self.motion.do_open(self.data, qpos_open, ctrl_open, duration)
            
        elif cmd_type == RobotCommand.SLEEP:
            logger.info("Entering sleep mode")
            self.eyes.is_active = False
            self.player.stop()
            self.screen_top.show_text("Zzz...")          
            if os.path.exists(SLEEP_AUDIO_PATH):
                self.player.play_file(SLEEP_AUDIO_PATH)
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
            else:
                self.set_status(RobotStatus.SHUTDOWN)


        elif cmd_type == RobotCommand.PLAY_VIDEO:
            if self._current_status != RobotStatus.ACTIVE: return
            path = payload['path']
            is_boot = payload.get('is_boot', False)
            def on_complete(): self._command_queue.put((RobotCommand.BOOT_COMPLETE, {}))
            self.player.play_file(path, on_complete=on_complete if is_boot else None)

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

        elif cmd_type == RobotCommand.SET_EYES_MODE:
            if self.eyes.is_active:
                mode = payload['mode']
                if mode == "HAPPY":
                    _mode = HAPPY
                elif mode == "ANGRY":
                    _mode = ANGRY
                elif mode == "TIRED":
                    _mode = TIRED
                else:
                    _mode = DEFAULT
                self.eyes.set_mode(mode)

        elif cmd_type == RobotCommand.SET_EYES_POS:
            if self.eyes.is_active:
                position = payload['position']
                pos_map = {
                    "N": N,
                    "S": S,
                    "E": E,
                    "W": W,
                    "NE": NE,
                    "NW": NW,
                    "SE": SE,
                    "SW": SW
                }
                self.eyes.set_position(pos_map[position])

        elif cmd_type == RobotCommand.UPDATE_SCREEN:
            self.screen_bottom.show_text(payload['text'])

        elif cmd_type == RobotCommand.MOTION_COMPLETE:
            if self._current_status != RobotStatus.ACTIVE: return
            boot_path = BOOT_VIDEO_PATH
            if (self._current_status == RobotStatus.ACTIVE) and not self._boot_played and os.path.exists(boot_path):
                self._command_queue.put((RobotCommand.PLAY_VIDEO, {"path": boot_path, "is_boot": True}))
                self._boot_played = True
            else:
                self._command_queue.put((RobotCommand.SET_EYES, {"active": True}))

        elif cmd_type == RobotCommand.BOOT_COMPLETE:
            if self._current_status != RobotStatus.ACTIVE: return
            self._command_queue.put((RobotCommand.SET_EYES, {"active": True}))

    def _cleanup(self):
        logger.info("Cleaning up resources")
        self.player.stop()
        self.audio.close()
        self.set_status(RobotStatus.SHUTDOWN)
        self._is_ready_event.clear()