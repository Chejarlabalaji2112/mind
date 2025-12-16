XML_PATH         = f"/home/badri/hitomi/mind/src/mind/simulation//description/scene.xml"

import mujoco
import numpy as np
import imageio
import os

# Load model and data
xml_path = 'robot.xml'  # Replace with your model path
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

# Create offscreen renderer (adjust height/width as needed)
renderer = mujoco.Renderer(model, height=480, width=640)

# Simulation parameters
duration = 5.0  # seconds
fps = 60
total_frames = int(duration * fps)
timestep = model.opt.timestep

# Storage for frames
frames = []

print("Running simulation and recording video...")
for i in range(total_frames):
    # Run physics step (add your control logic here if needed)
    mujoco.mj_step(model, data)
    
    # Render RGB frame
    rgb = renderer.render()
    
    # Convert to uint8 and append (imageio expects HWC uint8)
    frame = (255 * np.clip(rgb, 0, 1)).astype(np.uint8)
    frames.append(frame)
    
    if i % 100 == 0:
        print(f"Frame {i}/{total_frames}")

# Save video
output_path = 'simulation.mp4'
imageio.mimsave(output_path, frames, fps=fps)
print(f"Video saved: {os.path.abspath(output_path)}")

