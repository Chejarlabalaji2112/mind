# import mujoco
# import mujoco.viewer
# from .screen_updater import ScreenUpdater
# from .av_orchestrator import AVOrchestrator
# from adapters.audio_adapters import AudioManager
# from adapters.camera_adapers import CameraSource
# from .motion_controller import MotionController
# import time
# import numpy as np
# from utils.robo_eyes import RoboEyes

# class CombinedSimulation:
#  = "/home/badri/mine/hitomi/mind/src/mind/simulation/description/scene.xml"


# model = mujoco.MjModel.from_xml_path(XML_PATH)
# data  = mujoco.MjData(model)

# # Load "home" keyframe BEFORE viewer starts
# home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
# open_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "open")


# mujoco.mj_resetDataKeyframe(model, data, home_id)

# def open_robot(elapsed, data, model):

#         # Linear interpolation factor
#         alpha = elapsed / 2.0

#         # Interpolate qpos
#         data.qpos[:] = (
#             (1 - alpha) * model.key_qpos[home_id] +
#             alpha * model.key_qpos[open_id]
#         )

#         # Interpolate controls
#         data.ctrl[:] = (
#             (1 - alpha) * model.key_ctrl[home_id] +
#             alpha * model.key_ctrl[open_id]
#         )
    


# # Screen + audio
# screen = ScreenUpdater(model, "display_top")
# audio = AudioManager(rate=44100, channels=2)
# player = AVOrchestrator(screen, audio)
# eyes=RoboEyes(screen, width=512, height=512)



# # -----------------------------------------------------
# # VIEWER
# # -----------------------------------------------------
# with mujoco.viewer.launch_passive(model, data,
#         show_left_ui=False, show_right_ui=False) as viewer:

#     start_t = time.time()
#     boot_started = False
#     boot_completed = False
#     opened = False



#     # Camera view
#     with viewer.lock():
#         viewer.cam.lookat[:] = np.array([0.0, 0.05, 0.1])
#         viewer.cam.azimuth = 0.0
#         viewer.cam.elevation = -20.0
#         viewer.cam.distance = 0.3

#     # MAIN LOOP
#     while viewer.is_running():

#         elapsed = time.time() - start_t

#         # ----------------------------------------
#         # 1) Move from HOME → OPEN during first 2 sec
#         # ----------------------------------------
#         if elapsed < 2.0:
#             open_robot(elapsed, data, model)

#         # ----------------------------------------
#         # 2) When opening animation finishes → play boot.mp4
#         # ----------------------------------------
#         elif not boot_started:
#             opened = True
#             boot_started = True
#             player.play_file("/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/boot.mp4")

#         if boot_started and not boot_completed:
#             if player.no_video_playing:
#                 boot_completed = True
#                 print("Boot sequence completed.")

#         if boot_completed and not eyes.is_active:
#             print("Activating RoboEyes")
#             eyes.is_active = True
#             eyes.begin()
#             eyes.set_auto_blink(True)
#             eyes.open()


#         # Step simulation
#         mujoco.mj_step(model, data)

#         # Update display
#         if eyes.is_active and player.no_video_playing:
#             eyes.update()
#         screen.update(viewer)

#         # Sync viewer
#         viewer.sync()


# player.stop()
# audio.close()

import mujoco
import mujoco.viewer
import time
import numpy as np

# Adapters and Utils (Preserving your imports)
from .screen_updater import ScreenUpdater
from .av_orchestrator import AVOrchestrator
from adapters.audio_adapters import AudioManager
from adapters.camera_adapers import CameraSource
from .motion_controller import MotionController
from utils.robo_eyes import RoboEyes

XML_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/description/scene.xml"
BOOT_VIDEO_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/boot.mp4"

class CombinedSimulation:
    def __init__(self, scene_path=XML_PATH):
        # 1. Initialize MuJoCo
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)

        # 2. Keyframe Setup
        self.home_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        self.open_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "open")
        self.wake_duration = 2.0  # Seconds to transition from home to open

        # Start robot in "home" position
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.home_id)
        mujoco.mj_forward(self.model, self.data) # Propagate kinematics

        # 3. Component Initialization
        self.screen = ScreenUpdater(self.model, "display_top")
        self.audio = AudioManager(rate=44100, channels=2)
        self.player = AVOrchestrator(self.screen, self.audio)
        self.eyes = RoboEyes(self.screen, width=512, height=512)

        # 4. State Flags
        self.state = "WAKING"  # States: WAKING -> BOOTING -> ACTIVE
        self.boot_video_triggered = False

    def interpolate_pose(self, elapsed):
        """Linearly interpolates between HOME and OPEN keyframes."""
        # Clamp alpha between 0 and 1
        alpha = min(elapsed / self.wake_duration, 1.0)

        # Interpolate qpos (positions)
        self.data.qpos[:] = (
            (1 - alpha) * self.model.key_qpos[self.home_id] +
            alpha * self.model.key_qpos[self.open_id]
        )

        # Interpolate ctrl (actuators)
        self.data.ctrl[:] = (
            (1 - alpha) * self.model.key_ctrl[self.home_id] +
            alpha * self.model.key_ctrl[self.open_id]
        )

    def run(self):
        print("Launching Viewer...")
        
        with mujoco.viewer.launch_passive(self.model, self.data,
                show_left_ui=False, show_right_ui=False) as viewer:
            
            # -- Camera Setup --
            with viewer.lock():
                viewer.cam.lookat[:] = np.array([0.0, 0.05, 0.1])
                viewer.cam.azimuth = 0.0
                viewer.cam.elevation = -20.0
                viewer.cam.distance = 0.3

            start_time = time.time()

            try:
                while viewer.is_running():
                    elapsed = time.time() - start_time
                    
                    # -------------------------------------------------
                    # STATE 1: WAKING (0s to 2s)
                    # -------------------------------------------------
                    if self.state == "WAKING":
                        # Perform motion interpolation
                        self.interpolate_pose(elapsed)

                        # Check if waking is done
                        if elapsed >= self.wake_duration:
                            self.state = "BOOTING"
                            print("Waking complete. Starting Boot Sequence.")

                    # -------------------------------------------------
                    # STATE 2: BOOTING (Video Playback)
                    # -------------------------------------------------
                    elif self.state == "BOOTING":
                        # Trigger video once
                        if not self.boot_video_triggered:
                            self.player.play_file(BOOT_VIDEO_PATH)
                            self.boot_video_triggered = True
                        
                        # Check if video finished
                        # We check > 0.1s to ensure the player had time to actually start
                        if self.boot_video_triggered and self.player.no_video_playing:
                            # Wait a tiny bit to ensure no race condition on start
                            
                            if self.player.no_video_playing:
                                self.state = "ACTIVE"
                                print("Boot complete. Activating Eyes.")

                    # -------------------------------------------------
                    # STATE 3: ACTIVE (RoboEyes)
                    # -------------------------------------------------
                    elif self.state == "ACTIVE":
                        # Initialize eyes once
                        if not self.eyes.is_active:
                            time.sleep(0.1)  # Small delay to ensure stability
                            self.eyes.is_active = True
                            self.eyes.begin()
                            self.eyes.set_auto_blink(True)
                            self.eyes.open()
                        
                        # Update eye logic
                        self.eyes.update()

                    # -------------------------------------------------
                    # SIMULATION & RENDER STEPS
                    # -------------------------------------------------
                    
                    # Step Physics
                    mujoco.mj_step(self.model, self.data)

                    # Update Screen Texture (Video or Eyes)
                    self.screen.update(viewer)

                    # Sync Viewer
                    viewer.sync()

            except KeyboardInterrupt:
                print("Simulation stopped by user.")
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