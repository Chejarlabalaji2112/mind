from ports.act_port import Manipulator
import mujoco


class MotionController(Manipulator):
    def __init__(self, model):
        self.model = model
        
        self.act_y = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_top_y")
        self.act_z = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_top_z")
        self.act_antl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_ant_l")
        self.act_antr = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_ant_r")

        self.current_motion = None
        self.k = 0.01   # smoothness factor

    # -----------------------------------------------------
    # Public gesture triggers
    # -----------------------------------------------------
    def do(self):
        """just defined to satisfy abstract method"""
        pass 

    def do_yes(self):
        self.current_motion = self._prepare(self._gen_yes())
        next(self.current_motion)

    def do_no(self):
        self.current_motion = self._prepare(self._gen_no())
        next(self.current_motion)

    def do_say(self):
        """Example SAY gesture (top opens slightly while centered)."""
        self.current_motion = self._prepare(self._gen_say())
        next(self.current_motion)

    # -----------------------------------------------------
    # Main updater (called once per frame)
    # -----------------------------------------------------
    def update_motion(self, data):
        if not self.current_motion:
            return
        try:
            self.current_motion.send(data)
        except StopIteration:
            self.current_motion = None

    # -----------------------------------------------------
    # Smooth actuator ctrl → target
    # -----------------------------------------------------
    def smooth(self, data, act_id, target):
        curr = data.ctrl[act_id]
        data.ctrl[act_id] += (target - curr) * self.k
        return abs(data.ctrl[act_id] - target) < 0.02

    # -----------------------------------------------------
    # PREPARATION GENERATOR
    # Forces servo_top_z back to 0 before ANY gesture
    # -----------------------------------------------------
    def _prepare(self, gesture_gen):
        """Ensure neutral center before gesture."""
        data = yield
        
        # Move servo_top_z → 0 first
        while not self.smooth(data, self.act_z, 0.0):
            data = yield

        # Now run the actual gesture generator
        yield from gesture_gen

    # -----------------------------------------------------
    # YES gesture: DOWN (1) → UP (0)
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
    # NO gesture: left → right → center
    # -----------------------------------------------------
    def _gen_no(self):
        data = yield

        while not self.smooth(data, self.act_z, -0.5):
            data = yield
        
        while not self.smooth(data, self.act_z, 0.5):
            data = yield

        while not self.smooth(data, self.act_z, 0.0):
            data = yield

    # -----------------------------------------------------
    # SAY gesture: small flap (example)
    # -----------------------------------------------------
    def _gen_say(self):
        data = yield

        # Slight opening
        while not self.smooth(data, self.act_y, 0.4):
            data = yield
        
        # Return open
        while not self.smooth(data, self.act_y, 0.0):
            data = yield
