import mujoco
import mujoco.viewer
import time
import numpy as np

# Configuration
SCENE_XML_PATH = "scene.xml"
SIMULATION_DURATION = 30.0  # seconds

def get_actuator_ids(model):
    """Retrieves the IDs of the actuators for easy access."""
    ids = {}
    for name in ["servo_top_y", "servo_top_z", "servo_ant_r", "servo_ant_l", "servo_top_com"]:
        try:
            ids[name] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        except Exception:
            print(f"Warning: Actuator {name} not found in model.")
    return ids

def main():
    print(f"Loading model from {SCENE_XML_PATH}...")
    try:
        model = mujoco.MjModel.from_xml_path(SCENE_XML_PATH)
        data = mujoco.MjData(model)
    except ValueError as e:
        print(f"Error loading model: {e}")
        print("Ensure 'meshes/' and 'images/' folders exist and contain your assets.")
        return

    actuator_ids = get_actuator_ids(model)
    actuator_names = list(actuator_ids.keys())
    actuator_indices = list(actuator_ids.values())
    
    # Get Keyframe IDs
    try:
        key_home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        # Try to find 'open', fallback to 'alert' if 'open' doesn't exist, or error if neither
        key_open_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "open")
    except Exception:
        print("Error: Could not find required keyframes ('home' and 'open'/'alert'). Check XML.")
        return

    # Extract control values for keyframes
    # model.key_ctrl is (nkey, nu)
    ctrl_home = model.key_ctrl[key_home_id]
    ctrl_open = model.key_ctrl[key_open_id]

    # Launch the passive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        # Reset to home keyframe initially
        mujoco.mj_resetDataKeyframe(model, data, key_home_id)
        mujoco.mj_forward(model, data)

        print("Simulation started.")
        print("Phase 1: Holding Home")
        
        while viewer.is_running() and time.time() - start_time < SIMULATION_DURATION:
            step_start = time.time()
            current_time = time.time() - start_time

            # --- CONTROL LOGIC ---
            
            # Phase 1: Hold Home (0 - 2 seconds)
            if current_time < 2.0:
                  pass
            # Phase 2: Slowly transition to Open (2 - 5 seconds)
            elif current_time < 5.0:
                # Calculate interpolation factor alpha (0 to 1)
                duration = 3.0
                alpha = (current_time - 2.0) / duration
                # Linear interpolation: (1 - a)*Home + a*Open
                data.ctrl[:] = (1 - alpha) * ctrl_home + alpha * ctrl_open
            
            # Phase 3: Simulate each joint individually (5+ seconds)
            else:
                # Base position is the "Open" pose
                data.ctrl[:] = ctrl_open.copy()
                
                # Cycle through joints: change joint every 3 seconds
                phase_time = current_time - 5.0
                cycle_duration = 3.0
                
                active_joint_idx = int(phase_time / cycle_duration) % len(actuator_indices)
                active_actuator_id = actuator_indices[active_joint_idx]
                
                # Apply a sine wave to the active joint
                # Amplitude 1.0, Frequency 2 rad/s
                oscillation = 1.0 * np.sin(phase_time * 2)
                data.ctrl[active_actuator_id] += oscillation

            # Step the physics
            mujoco.mj_step(model, data)

            # Sync viewer
            viewer.sync()

            # Time keeping
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
