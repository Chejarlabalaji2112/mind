import mujoco
import mujoco.viewer
import numpy as np
import time

# The full XML from your model (paste it here as a string)
xml_string = """
<mujoco>
    <compiler angle="degree" coordinate="local"/>
    <option timestep="0.001" iterations="100" tolerance="1e-10"/>
   
    <!-- ADDED: Keyframe definition for the "Home" position -->
    <!-- qpos values correspond to the order of joints defined below -->
    <!-- Order: j_top_y, j_top_z, j_top_cam, j_top_ant_r, j_top_ant_l -->
    <keyframe>
        <key name="home" qpos="1.85 0 0 0 0" ctrl="2.1 0 0 0 0"/>
        <key name="open" qpos="0 0 0 0 0" ctrl="0 0 0 0 0"/>
    </keyframe>
    <asset>
        <mesh name="bottom_mesh" file="meshes/bottom.stl"/>
        <mesh name="top_mesh" file="meshes/top.stl"/>
        <mesh name="ant_r_mesh" file="meshes/antenna_r.stl"/>
        <mesh name="ant_l_mesh" file="meshes/antenna_l.stl"/>
        <mesh name="camera_mesh" file="meshes/camera.stl"/>
        <texture name="display_tex" type="2d" builtin="checker" width="512" height="512" rgb1="0.1 0.1 0.1" rgb2="0.9 0.9 0.9"/>
        <material name="display_mat" texture="display_tex" specular="0.5" shininess="0.5"/>
    </asset>
    <worldbody>
        <body name="bottom" pos="0 0 0.024">
            <geom type="mesh" mesh="bottom_mesh" rgba="0.7 0.7 0.7 1" density="300"/>
            <body name="top" pos="-0.008 0 0.01">
                <joint name="j_top_y" type="hinge" axis="0 1 0" range="0 106" damping="4" stiffness="10" armature="0.000001" pos="-0.027 -0.01 0.03"/>
                <joint name="j_top_z" type="hinge" axis="0 0 1" range="-10 10" damping="20" stiffness="10" armature="0.01" pos="-0.027 -0.049 -0.01"/>
               
                <geom type="mesh" mesh="top_mesh" rgba="0.7 0.7 0.7 1" density="300"/>
                <geom name="screen_geom" type="box" size="0.031 0.0278 0.0001" material="display_mat" pos="0.1 -0.049 0.073" euler="0 74 90" mass="0"/>
               
                <body name="camera" pos="0.002 0 0.015">
                    <camera name="camera_top" pos="-0.025 0 0.1" euler="0 150 90"/>
                    <joint name="j_top_cam" type="hinge" pos="-0.015 0 0.097" range="0 90" damping="10" stiffness="50" armature="0.01" axis="0 1 0"/>
                    <geom type="mesh" mesh="camera_mesh" rgba="0 0 0 1"/>
                </body>
            </body>
               
            <body name="antenna_right" pos="0 0 0.002">
                <joint name="j_top_ant_r" type="hinge" axis="0 1 0" range="0 180" damping="0.1" stiffness="50" springref="10" pos="-0.025 -0.08 -0.01"/>
                <geom type="mesh" mesh="ant_r_mesh" rgba="0 0 0 1"/>
            </body>
            <body name="antenna_left" pos="0 0 0.002">
                <joint name="j_top_ant_l" type="hinge" axis="0 1 0" range="0 180" damping="0.1" stiffness="50" springref="10" pos="-0.027 0.08 -0.01"/>
                <geom type="mesh" mesh="ant_l_mesh" rgba="0 0 0 1"/>
            </body>
        </body>
    </worldbody>
    <actuator>
        <position name="servo_top_y" joint="j_top_y" kp="100" ctrlrange="0 2.1"/>
        <position name="servo_top_z" joint="j_top_z" kp="90" ctrlrange="-0.5 0.5"/>
        <position name="servo_ant_r" joint="j_top_ant_r" ctrlrange = "-10 15" kp="5"/>
        <position name="servo_ant_l" joint="j_top_ant_l" ctrlrange = "-10 15" kp="5"/>
        <position name="servo_top_com" joint="j_top_cam" ctrlrange="0 3.5" kp="50"/>
    </actuator>
</mujoco>
"""

# Load the model from XML string
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)

# Find the texture ID
texname = b'display_tex'
texid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, texname)
if texid < 0:
    raise ValueError(f"Texture '{texname.decode()}' not found in model.")

# Get texture properties
width = model.tex_width[texid]
height = model.tex_height[texid]

# Compute channels dynamically from allocated data size
if texid < model.ntex - 1:
    tex_size = model.tex_adr[texid + 1] - model.tex_adr[texid]
else:
    tex_size = model.ntexdata - model.tex_adr[texid]
channels = tex_size // (width * height)
print(f"Texture '{texname.decode()}': {width}x{height}, {channels} channels (RGB/RGBA)")

def set_texture_color(model, texid, color, alpha=1.0):
    """
    Set the entire texture to a solid color.
    
    Args:
        model: Loaded MjModel.
        texid: Texture ID.
        color: List or array [R, G, B] in [0,1] range (floats) or [0,255] (ints).
        alpha: Alpha value [0,1] if 4 channels (ignored for 3 channels).
    """
    width = model.tex_width[texid]
    height = model.tex_height[texid]
    
    # Recompute channels
    if texid < model.ntex - 1:
        tex_size = model.tex_adr[texid + 1] - model.tex_adr[texid]
    else:
        tex_size = model.ntexdata - model.tex_adr[texid]
    channels = tex_size // (width * height)
    
    if channels == 4:
        pixel = np.array([*color, alpha], dtype=np.float32)
    elif channels == 3:
        pixel = np.array(color, dtype=np.float32)
    else:
        raise ValueError(f"Unsupported channels: {channels}")
    
    if pixel.max() <= 1.0:
        pixel = (pixel * 255).astype(np.uint8)
    else:
        pixel = pixel.astype(np.uint8)
    
    # Tile the pixel across the texture
    num_pixels = width * height
    flat_data = np.tile(pixel, num_pixels)
    
    # Update starting from tex_adr[texid]
    offset = model.tex_adr[texid]
    model.tex_data[offset:offset + tex_size] = flat_data
    print(f"Set texture {texid} to solid color {color} (alpha={alpha if channels==4 else 'N/A'})")

# Color cycling example: Red -> Green -> Blue -> repeat
colors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
color_idx = 0
update_every_n_steps = 100  # Update color every 100 simulation steps

# Launch passive viewer for control over the loop
with mujoco.viewer.launch_passive(model, data) as viewer:
    step_count = 0
    running = True
    while running:
        for _ in range(10):  # Batch steps for smoother sim (optional)
            mujoco.mj_step(model, data)
            step_count += 1
        
        # Update color periodically
        if step_count % update_every_n_steps == 0:
            set_texture_color(model, texid, colors[color_idx])
            color_idx = (color_idx + 1) % len(colors)
            viewer.update_texture(texid)  # Ensure GPU update
        
        viewer.sync()  # Render the updated scene
        
        # Check for viewer exit
        running = viewer.is_running
        time.sleep(1)  # Small delay to prevent CPU hog

print("Viewer closed.")
