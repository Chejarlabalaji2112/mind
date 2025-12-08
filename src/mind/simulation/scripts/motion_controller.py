from mind.ports.act_port import Manipulator
import mujoco
import numpy as np


class MotionController(Manipulator):
    """
    A unified non-blocking motion controller using generator-based gestures.
    Supports:
        - YES gesture
        - NO gesture
        - SAY gesture
        - OPEN (full body keyframe interpolation)
        - CLOSE (full body keyframe interpolation)
    """

    def __init__(self, model):
        self.model = model

        # Individual actuators (head, antennae, etc.)
        self.act_y = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_top_y")
        self.act_z = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_top_z")
        self.act_antl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_ant_l")
        self.act_antr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_ant_r")

        # Current running generator
        self.current_motion = None

        # Smooth actuator factor
        self.k = 0.01

    # -----------------------------------------------------
    # Public (abstract requirement)
    # -----------------------------------------------------
    def do(self):
        pass

    # -----------------------------------------------------
    # <===  HIGH-LEVEL GESTURES  ===>
    # -----------------------------------------------------
    def do_yes(self):
        self.current_motion = self._prepare(self._gen_yes())
        next(self.current_motion)

    def do_no(self):
        self.current_motion = self._prepare(self._gen_no())
        next(self.current_motion)

    def do_say(self):
        self.current_motion = self._prepare(self._gen_say())
        next(self.current_motion)

    # -----------------------------------------------------
    # <===  FULL BODY OPEN / CLOSE  ===>
    # -----------------------------------------------------
    def do_open(self, data, target_qpos, target_ctrl, duration=2.5):
        """Home → Open (non-blocking full-body keyframe interpolation)."""
        self.current_motion = self._interp_keyframe(target_qpos, target_ctrl, duration)
        next(self.current_motion)

    def do_close(self, data, target_qpos, target_ctrl, duration=2.0):
        """Open → Home (non-blocking full-body keyframe interpolation)."""
        print(f"closing with {duration}")
        self.current_motion = self._interp_keyframe(target_qpos, target_ctrl, duration)
        next(self.current_motion)

    # -----------------------------------------------------
    # Called every frame from main simulation loop
    # -----------------------------------------------------
    def update_motion(self, data):
        if not self.current_motion:
            return
        try:
            self.current_motion.send(data)
        except StopIteration:
            self.current_motion = None

    # -----------------------------------------------------
    # Smooth actuator interpolation
    # -----------------------------------------------------
    def smooth(self, data, act_id, target):
        curr = data.ctrl[act_id]
        data.ctrl[act_id] += (target - curr) * self.k
        return abs(data.ctrl[act_id] - target) < 0.02

    # -----------------------------------------------------
    # <=== PREPARE: return head to center before gestures
    # -----------------------------------------------------
    def _prepare(self, gesture_gen):
        """Center head (servo_top_z → 0) before any gesture."""
        data = yield

        # Center first
        while not self.smooth(data, self.act_z, 0.0):
            data = yield

        # Now run the actual gesture
        yield from gesture_gen

    # -----------------------------------------------------
    # YES (nod)
    # -----------------------------------------------------
    def _gen_yes(self):
        data = yield

        # Down
        while not self.smooth(data, self.act_y, 1.0):
            data = yield

        # Up
        while not self.smooth(data, self.act_y, 0.0):
            data = yield

    # -----------------------------------------------------
    # NO (shake)
    # -----------------------------------------------------
    def _gen_no(self):
        data = yield

        # Left
        while not self.smooth(data, self.act_z, -0.5):
            data = yield

        # Right
        while not self.smooth(data, self.act_z, 0.5):
            data = yield

        # Center
        while not self.smooth(data, self.act_z, 0.0):
            data = yield

    # -----------------------------------------------------
    # SAY gesture (slight head movement)
    # -----------------------------------------------------
    def _gen_say(self):
        data = yield

        # Slight forward nod
        while not self.smooth(data, self.act_y, 0.4):
            data = yield

        while not self.smooth(data, self.act_y, 0.0):
            data = yield

    # -----------------------------------------------------
    # <=== FULL BODY GENERATOR INTERPOLATION ===>
    # -----------------------------------------------------
    def _interp_keyframe(self, target_qpos, target_ctrl, duration):
        """
        Non-blocking frame-by-frame interpolation for full robot motion.
        This smoothly moves qpos + ctrl toward the target keyframe.
        """
        data = yield

        model = self.model

        start_qpos = data.qpos.copy()
        start_ctrl = data.ctrl.copy()
        start_time = data.time

        while True:
            t = (data.time - start_time) / duration
            if t >= 1.0:
                data.qpos[:] = target_qpos
                data.ctrl[:] = target_ctrl
                break

            alpha = t

            # Interpolate qpos and ctrl arrays
            data.qpos[:] = (1 - alpha) * start_qpos + alpha * target_qpos
            data.ctrl[:] = (1 - alpha) * start_ctrl + alpha * target_ctrl

            data = yield
