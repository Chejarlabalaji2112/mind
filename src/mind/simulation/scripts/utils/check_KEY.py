import mujoco
import mujoco.viewer
import numpy as np

XML_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/description/scene.xml"
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)
home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
mujoco.mj_resetDataKeyframe(model, data, home_id)
mujoco.mj_forward(model, data)

with mujoco.viewer.launch_passive(model, data,
                show_left_ui=False, show_right_ui=False) as viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = np.array([0.0, 0.05, 0.1])
                viewer.cam.azimuth = 0.0
                viewer.cam.elevation = -30.0
                viewer.cam.distance = 0.3

            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()