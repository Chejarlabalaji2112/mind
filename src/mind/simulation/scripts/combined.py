import mujoco
import mujoco.viewer
import time
import numpy as np
import os

# Adapters and Utils
from .screen_updater import ScreenUpdater
from .av_orchestrator import AVOrchestrator
from adapters.audio_adapters import AudioManager
from adapters.camera_adapers import CameraSource
from .motion_controller import MotionController
from utils.robo_eyes import RoboEyes

# Paths
XML_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/description/scene.xml"
BOOT_VIDEO_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/Light.mp4"
CLOSE_AUDIO_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/media/audio/shutdown.mp3" # Ensure this path is correct

class CombinedSimulation:
    def __init__(self, scene_path=XML_PATH):
        # 1. Initialize MuJoCo
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)

        # 2. Keyframe Setup
        self.home_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        self.open_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "open")
        self.wake_duration = 4.0 
        self.close_duration = 5.0

        # Start robot in "home" position
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.home_id)
        mujoco.mj_forward(self.model, self.data)

        # 3. Component Initialization
        self.screen = ScreenUpdater(self.model, "display_top")
        self.audio = AudioManager(rate=44100, channels=2)
        self.player = AVOrchestrator(self.screen, self.audio)
        self.eyes = RoboEyes(self.screen, width=512, height=512)

        # 4. State Flags
        self.state = "WAKING"
        self.boot_video_triggered = False

    def interpolate_pose(self, elapsed, start_qpos, start_ctrl, target_qpos, target_ctrl, duration):
        """Generic interpolation between two states."""
        alpha = min(elapsed / duration, 1.0)

        self.data.qpos[:] = (
            (1 - alpha) * start_qpos +
            alpha * target_qpos
        )

        self.data.ctrl[:] = (
            (1 - alpha) * start_ctrl +
            alpha * target_ctrl
        )

    def shutdown_sequence(self, viewer):
        """Handles the graceful shutdown: Stop video, play sound, move to home."""
        print("\n--- Initiating Shutdown Sequence ---")

        # 1. Stop current media/eyes
        self.player.stop()
        self.screen.show_text("shutting down...")
        self.eyes.is_active = False
        self.screen.update(viewer)
        time.sleep(1.0)  # Brief pause to ensure message is seen
        
        # 2. Play Close Audio
        if os.path.exists(CLOSE_AUDIO_PATH):
            self.player.play_file(CLOSE_AUDIO_PATH)
        else:
            print(f"Warning: Close audio not found at {CLOSE_AUDIO_PATH}")

        # 3. Capture CURRENT state (start point) and HOME state (target point)
        start_qpos = self.data.qpos.copy()
        start_ctrl = self.data.ctrl.copy()
        
        target_qpos = self.model.key_qpos[self.home_id]
        target_ctrl = self.model.key_ctrl[self.home_id]

        # 4. Closing Animation Loop
        start_close_time = time.time()
        
        print("Returning to Home Position...")
        while time.time() - start_close_time < self.close_duration:
            elapsed = time.time() - start_close_time
            
            # Interpolate backwards (Current -> Home)
            self.interpolate_pose(elapsed, start_qpos, start_ctrl, target_qpos, target_ctrl, self.close_duration)
            
            # Step Physics
            mujoco.mj_step(self.model, self.data)
            
            # Update screen (keeps it black/static during close)
            self.screen.update(viewer)
            
            # Sync Viewer
            viewer.sync()

        print("Shutdown Sequence Complete.")

    def run(self):
        print("Launching Viewer...")
        
        with mujoco.viewer.launch_passive(self.model, self.data,
                show_left_ui=False, show_right_ui=False) as viewer:
            
            # -- Camera Setup --
            with viewer.lock():
                viewer.cam.lookat[:] = np.array([0.0, 0.05, 0.1])
                viewer.cam.azimuth = 0.0
                viewer.cam.elevation = -30.0
                viewer.cam.distance = 0.3

            start_time = time.time()

            try:
                # Pre-calculate Waking targets
                wake_start_qpos = self.model.key_qpos[self.home_id]
                wake_start_ctrl = self.model.key_ctrl[self.home_id]
                wake_target_qpos = self.model.key_qpos[self.open_id]
                wake_target_ctrl = self.model.key_ctrl[self.open_id]

                while viewer.is_running():
                    elapsed = time.time() - start_time
                    
                    # -------------------------------------------------
                    # STATE 1: WAKING
                    # -------------------------------------------------
                    if self.state == "WAKING":
                        self.interpolate_pose(elapsed, wake_start_qpos, wake_start_ctrl, 
                                              wake_target_qpos, wake_target_ctrl, self.wake_duration)

                        if elapsed >= self.wake_duration:
                            self.state = "BOOTING"
                            print("Waking complete. Starting Boot Sequence.")

                    # -------------------------------------------------
                    # STATE 2: BOOTING
                    # -------------------------------------------------
                    elif self.state == "BOOTING":
                        if not self.boot_video_triggered:
                            self.player.play_file(BOOT_VIDEO_PATH)
                            self.boot_video_triggered = True
                        
                        if self.boot_video_triggered and self.player.no_video_playing:
                             self.state = "ACTIVE"
                             print("Boot complete. Activating Eyes.")

                    # -------------------------------------------------
                    # STATE 3: ACTIVE
                    # -------------------------------------------------
                    elif self.state == "ACTIVE":
                        if not self.eyes.is_active:
                            time.sleep(0.1)
                            self.eyes.is_active = True
                            self.eyes.begin()
                            self.eyes.set_auto_blink(True)
                            self.eyes.set_idle_mode(True)
                        
                        self.eyes.update()

                    # -------------------------------------------------
                    # STEP & RENDER
                    # -------------------------------------------------
                    mujoco.mj_step(self.model, self.data)
                    self.screen.update(viewer)
                    viewer.sync()

            except KeyboardInterrupt:
                # Intercept the Ctrl+C here
                self.shutdown_sequence(viewer)
            
            finally:
                self.cleanup()

    def cleanup(self):
        print("Cleaning up resources...")
        self.player.stop()
        self.audio.close()

# -----------------------------------------------------
# EXECUTION
# -----------------------------------------------------
if __name__ == "__main__":
    sim = CombinedSimulation()
    sim.run()