import mujoco
import mujoco.viewer
import numpy as np
import time
from PIL import Image  # For robust PNG loading (handles alpha, transparency); install with pip if needed
from mind.utils.logging_handler import setup_logger

logger = setup_logger(__name__)

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
logger.info(
    "Texture details",
    extra={"name": texname.decode(), "width": width, "height": height, "channels": channels},
)

def update_texture_from_image(model, texid, png_path):
    """
    Update the texture buffer from a PNG file, handling channel conversion automatically.
    
    Args:
        model: Loaded MjModel.
        texid: Texture ID.
        png_path: Path to new PNG (dimensions must match; channels auto-converted to RGB/RGBA).
    
    Raises:
        ValueError: If dimensions mismatch.
    """
    # Load PNG with PIL (handles RGBA, transparency; returns uint8 [0,255])
    img = Image.open(png_path).convert('RGBA')  # Always load as RGBA for flexibility
    img_array = np.array(img)
    
    h, w, c = img_array.shape
    if w != width or h != height:
        raise ValueError(f"PNG dimensions ({w}x{h}) must match texture ({width}x{height})")
    
    # Compute texture channels
    tex_width = model.tex_width[texid]
    tex_height = model.tex_height[texid]
    if texid < model.ntex - 1:
        tex_size = model.tex_adr[texid + 1] - model.tex_adr[texid]
    else:
        tex_size = model.ntexdata - model.tex_adr[texid]
    tex_channels = tex_size // (tex_width * tex_height)
    
    # Convert to target channels (uint8, no normalization needed)
    if tex_channels == 3:  # RGB texture: Drop alpha or set to opaque
        if c == 4:
            # Convert RGBA to RGB (premultiply alpha or just drop; here, simple drop)
            rgb_array = img_array[:, :, :3]
        else:
            rgb_array = img_array
        data = rgb_array.ravel()
        logger.debug("Converted PNG to RGB for texture", extra={"channels": c, "texture_id": texid})
    elif tex_channels == 4:  # RGBA texture: Add alpha=255 if missing
        if c == 3:
            rgba_array = np.dstack([img_array, np.full((h, w, 1), 255, dtype=np.uint8)])
        else:
            rgba_array = img_array
        data = rgba_array.ravel()
        logger.debug("Converted PNG to RGBA for texture", extra={"channels": c, "texture_id": texid})
    else:
        raise ValueError(f"Unsupported texture channels: {tex_channels}")
    
    # Update the buffer
    offset = model.tex_adr[texid]
    model.tex_data[offset:offset + tex_size] = data
    logger.info("Updated texture from image", extra={"texture_id": texid, "path": png_path})

# Example video playback: List your PNG paths here (must be 512x512)
image_paths = ['images/image1.png', 'images/image2.png', 'images/image3.png']  # Replace with your sequence
frame_idx = 0
update_every_n_steps = 50  # Update image every 50 simulation steps (adjust for FPS)

# Launch passive viewer
# ... everything else stays exactly the same ...

with mujoco.viewer.launch_passive(model, data) as viewer:
    step_count = 0
    frame_idx = 0

    while viewer.is_running():
        mujoco.mj_step(model, data)
        step_count += 1

        # Change image every N steps
        if step_count % 30 == 0 and image_paths:
            update_texture_from_image(model, texid, image_paths[frame_idx])
            
            # THIS LINE IS THE KEY — without it nothing appears!
            viewer.update_texture(texid)
            
            frame_idx = (frame_idx + 1) % len(image_paths)

        viewer.sync()
        time.sleep(0.001)

logger.info("Viewer closed")
