import mujoco
import mujoco.viewer
from .screen_updater import ScreenUpdater
from .av_orchestrator import AVOrchestrator
from adapters.audio_adapters import AudioManager
from adapters.camera_adapers import CameraSource
from .motion_controller import MotionController
import time
import numpy as np

VIDEO_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/shin.mp4"
XML_PATH   = "/home/badri/mine/hitomi/mind/src/mind/simulation/description/scene.xml"
ALPHIE_IMG = "/home/badri/mine/hitomi/mind/src/mind/simulation/media/images/alphie.png"
EYES_IMG   = "/home/badri/mine/hitomi/mind/src/mind/simulation/media/images/eyes.png"




# ============================================================
# MAIN SIMULATION SETUP
# ============================================================
model = mujoco.MjModel.from_xml_path(XML_PATH)
data  = mujoco.MjData(model)

# Load screen
screen = ScreenUpdater(model, "display_top")

audio = AudioManager(rate=44100, channels=2)

player = AVOrchestrator(screen, audio)

# Create motion controller
# motion = MotionController(model)


# ============================================================
# MAIN VIEWER LOOP
# ============================================================
with mujoco.viewer.launch_passive(model, data,
        show_left_ui=False, show_right_ui=False) as viewer:
    start_t = time.time()
        
    screen.put_text("System Ready (SoundDevice)")
    player.play_file("/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/boot.mp4")
    # Camera setup
    with viewer.lock():
        viewer.cam.lookat[:] = np.array([0.0, 0.05, 0.1])
        viewer.cam.azimuth = 0.0
        viewer.cam.elevation = -15.0
        viewer.cam.distance = 0.5

    # Simulation loop
    while viewer.is_running():

        # STEP SIM
        mujoco.mj_step(model, data)

        # Apply YES/NO animation if active
        # motion.update_motion(data)

        # Update the display screen (image/video)
        screen.update(viewer)

        # Sync viewer
        viewer.sync()
        elapsed = time.time() - start_t
        if 2.0 < elapsed < 2.1:
            screen.put_text("Loading Video...")
        elif 4.0 < elapsed < 4.1:
            player.play_file(VIDEO_PATH, audio_track_index=2)


# Cleanup
player.stop()
audio.close()
