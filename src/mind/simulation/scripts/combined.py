import mujoco
import mujoco.viewer
import time
import numpy as np
import os
import atexit
from mind.utils.logging_handler import setup_logger

# Adapters and Utils
from mind.simulation.scripts.screen_updater import ScreenUpdater
from mind.simulation.scripts.av_orchestrator import AVOrchestrator
from mind.adapters.audio_adapters.sd_adapter import AudioManager
from adapters.vision_adapters.camera_handler import CameraSource
from mind.simulation.scripts.motion_controller import MotionController
from mind.utils.robo_eyes import RoboEyes, HAPPY

# Paths
XML_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/description/scene.xml"
BOOT_VIDEO_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/Light.mp4"
CLOSE_AUDIO_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/media/audio/shutdown.mp3"

logger = setup_logger(__name__)


class CombinedSimulation:
    def __init__(self, scene_path=XML_PATH):
        # -------------------------------
        # 1. MuJoCo Model Setup
        # -------------------------------
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)

        # Keyframes
        self.home_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        self.open_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "open")

        self.wake_duration = 4.0
        self.close_duration = 5.0

        # Start at home pose
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.home_id)
        mujoco.mj_forward(self.model, self.data)

        atexit.register(self.cleanup)
        # -------------------------------
        # 2. Components
        # -------------------------------
        self.screen = ScreenUpdater(self.model, "display_top")
        self.screen2 = ScreenUpdater(self.model, "display_bottom")
        self.audio = AudioManager(rate=44100, channels=2)
        self.player = AVOrchestrator(self.screen, self.audio)
        self.eyes = RoboEyes(self.screen, width=512, height=512)

        # NEW: Motion Controller
        self.motion = MotionController(self.model)

        # -------------------------------
        # 3. State
        # -------------------------------
        self.state = "WAKING"
        self.boot_video_triggered = False

        self.last_clock_update = 0

        self.is_happy = False

        self.model.opt.timestep = 0.002
    # -----------------------------------------------------
    # clock update
    # -----------------------------------------------------
    def update_clock(self):
        now = time.time()
        if now - self.last_clock_update >= 1.0:   # update once per second
            self.last_clock_update = now
            self.screen2.show_layout({
                    "top_left"    : "topleft",
                    "top_center"  : time.strftime("%H:%M:%S"),
                    "top_right"   : "top_right",
                    "center"      : "center",
                    "bottom_left" : "bottom_left",
                    "bottom_right": "bottom right"

            })

    # -----------------------------------------------------
    # update all
    # -----------------------------------------------------

    def update_all(self, viewer):
        self.update_clock()
        self.motion.update_motion(self.data)
        mujoco.mj_step(self.model, self.data)
        self.screen.update(viewer)
        self.screen2.update(viewer)
        viewer.sync()

    # -----------------------------------------------------
    # Shutdown (uses non-blocking close motion)
    # -----------------------------------------------------
    def shutdown_sequence(self, viewer):
        logger.info("Initiating shutdown sequence")

        # Stop all media
        self.player.stop()
        self.eyes.is_active = False

        self.screen.show_text("shutting down…")
        self.screen.update(viewer)
        time.sleep(0.7)

        # Play audio
        if os.path.exists(CLOSE_AUDIO_PATH):
            self.player.play_file(CLOSE_AUDIO_PATH)

        # Trigger CLOSE motion (model keyframe home)
        self.motion.do_close(
            self.data,
            self.model.key_qpos[self.home_id],
            self.model.key_ctrl[self.home_id],
            duration=self.close_duration
        )

        # Allow the non-blocking motion to run until done
        start_t = time.time()
        while viewer.is_running() and self.motion.current_motion is not None:
            loop_start = time.perf_counter()
            self.update_all(viewer)
            dt = time.perf_counter() - loop_start
            time.sleep(max(0, self.model.opt.timestep - dt))

        logger.info("Shutdown complete")

    # -----------------------------------------------------
    # MAIN LOOP
    # -----------------------------------------------------

    def run(self):
        logger.info("Launching viewer")

        with mujoco.viewer.launch_passive(
            self.model, self.data,
            show_left_ui=False,
            show_right_ui=False
        ) as viewer:

            # Camera setup
            with viewer.lock():
                viewer.cam.lookat[:] = np.array([0.0, 0.05, 0.1])
                viewer.cam.azimuth = 0.0
                viewer.cam.elevation = -30.0
                viewer.cam.distance = 0.3

            start_time = time.time()

            # Pre-calc wake targets
            qpos_open = self.model.key_qpos[self.open_id]
            ctrl_open = self.model.key_ctrl[self.open_id]

            # Trigger OPEN motion immediately
            self.motion.do_open(
                self.data,
                qpos_open,
                ctrl_open,
                self.wake_duration
            )

            loop_times = []

            try:
                while viewer.is_running():
                    start_t = time.time()
                    loop_start = time.perf_counter()  # high-precision timing

                    # ---------------------------
                    # STATE MACHINE
                    # ---------------------------
                    elapsed = time.time() - start_time

                    if self.state == "WAKING":
                        if self.motion.current_motion is None:
                            logger.info("Wake complete -> booting")
                            self.state = "BOOTING"

                    elif self.state == "BOOTING":
                        if not self.boot_video_triggered:
                            self.player.play_file(BOOT_VIDEO_PATH)
                            self.boot_video_triggered = True
                        elif self.player.no_video_playing:
                            logger.info("Boot finished -> active")
                            self.state = "ACTIVE"

                    elif self.state == "ACTIVE":
                        if not self.eyes.is_active:
                            self.eyes.is_active = True
                            time.sleep(0.1)
                            self.eyes.begin()
                            self.eyes.set_auto_blink(True)
                            self.eyes.set_idle_mode(True)
                        self.eyes.update()

                    # Update everything
                    self.update_all(viewer)
                    took = time.time() - start_t
                    time.sleep(max(0, self.model.opt.timestep - took))

                    # ---------------------------
                    # Loop timing & Hz calculation
                    # ---------------------------
                    loop_end = time.perf_counter()
                    dt = loop_end - loop_start
                    loop_times.append(dt)
                    if len(loop_times) > 100:  # keep last 100 steps
                        loop_times.pop(0)

                    avg_dt = sum(loop_times) / len(loop_times)
                    loop_hz = 1.0 / avg_dt
                    if len(loop_times) % 50 == 0:
                        logger.debug("Loop frequency: %.2f Hz", loop_hz)

            except KeyboardInterrupt:
                self.shutdown_sequence(viewer)

            finally:
                self.cleanup()

    # -----------------------------------------------------
    def cleanup(self):
        logger.info("Cleaning up resources")
        self.player.stop()
        self.audio.close()

    

# -----------------------------------------------------
# ENTRY
# -----------------------------------------------------

if __name__ == "__main__":
    sim = CombinedSimulation()
    sim.run()
