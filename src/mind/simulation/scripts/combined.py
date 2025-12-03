import mujoco
import mujoco.viewer
from .screen_updater import ScreenUpdater
from .av_orchestrator import AVOrchestrator
from adapters.audio_adapters import AudioManager
from adapters.camera_adapers import CameraSource
from .motion_controller import MotionController
import time
import numpy as np


XML_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/description/scene.xml"


model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

# Load "home" keyframe BEFORE viewer starts
home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
open_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "open")


mujoco.mj_resetDataKeyframe(model, data, home_id)


# Screen + audio
screen = ScreenUpdater(model, "display_top")
audio = AudioManager(rate=44100, channels=2)
player = AVOrchestrator(screen, audio)


# -----------------------------------------------------
# VIEWER
# -----------------------------------------------------
with mujoco.viewer.launch_passive(model, data,
        show_left_ui=False, show_right_ui=False) as viewer:

    start_t = time.time()
    boot_started = False
    opened = False


    # Camera view
    with viewer.lock():
        viewer.cam.lookat[:] = np.array([0.0, 0.05, 0.1])
        viewer.cam.azimuth = 0.0
        viewer.cam.elevation = -20.0
        viewer.cam.distance = 0.3

    # MAIN LOOP
    while viewer.is_running():

        elapsed = time.time() - start_t

        # ----------------------------------------
        # 1) Move from HOME → OPEN during first 2 sec
        # ----------------------------------------
        if elapsed < 2.0:
            # Linear interpolation factor
            alpha = elapsed / 2.0

            # Interpolate qpos
            data.qpos[:] = (
                (1 - alpha) * model.key_qpos[home_id] +
                alpha * model.key_qpos[open_id]
            )

            # Interpolate controls
            data.ctrl[:] = (
                (1 - alpha) * model.key_ctrl[home_id] +
                alpha * model.key_ctrl[open_id]
            )

        # ----------------------------------------
        # 2) When opening animation finishes → play boot.mp4
        # ----------------------------------------
        elif not boot_started:
            opened = True
            boot_started = True
            player.play_file("/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/boot.mp4")

        # ----------------------------------------
        # 3) Other timed screen events
        # ----------------------------------------
        if 6.0 < elapsed < 6.1:
            screen.put_text("Loading Video........")

        if 7.0 < elapsed < 7.001:
            player.play_file(
                "/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/eyes.mp4",
                audio_track_index=2
            )

        # Step simulation
        mujoco.mj_step(model, data)

        # Update display
        screen.update(viewer)

        # Sync viewer
        viewer.sync()


player.stop()
audio.close()
