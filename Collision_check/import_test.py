import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import json
import sys
import matplotlib.pyplot as plt

# --- MODIFIABLE WORKSPACE CONFIGURATION ---
WORKSPACE_POSITION = [0.39, 1.08, 0.0]
WORKSPACE_ORIENTATION = [0, 0, -2*1.57] # Euler angles
WORKSPACE_SCALE = 0.001              # mm to m conversion

SAFETY_THRESHOLD = 0.05 # 2cm buffer zone

# Initializations
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.81)

# Load assets and trajectory
pybullet_path = pybullet_data.getDataPath()
current_dir = os.path.dirname(os.path.abspath(__file__))
# Adjusted mesh_dir to step out of current folder to find meshes as discussed
mesh_dir = os.path.normpath(os.path.join(current_dir, "..", "meshes"))

robot_path = os.path.normpath(os.path.join(mesh_dir, "CS612.urdf"))
workspace_path = os.path.normpath(os.path.join(mesh_dir, "Downsized_WORKSPACE_2.stl"))
trajectory_file = os.path.join(current_dir, "q.json")

# Import data from q.json
with open(trajectory_file, "r") as f:
    trajectory_data = json.load(f)
    steps = trajectory_data["steps"]
    dt = trajectory_data["metadata"]["dt"]

p.loadURDF(os.path.join(pybullet_path, "plane.urdf"), [0, 0, 0])

# Load Robot
sys.stderr = open(os.devnull, 'w')
targid = p.loadURDF(robot_path, [0, 0, 0.57 ], useFixedBase=True)
sys.stderr = sys.__stderr__

# Load Workspace (Replacing Table and Pole)
workspace_id = -1
if os.path.exists(workspace_path):
    v_id = p.createVisualShape(p.GEOM_MESH, fileName=workspace_path, meshScale=[WORKSPACE_SCALE]*3)
    c_id = p.createCollisionShape(p.GEOM_MESH, fileName=workspace_path, meshScale=[WORKSPACE_SCALE]*3, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    workspace_id = p.createMultiBody(
        baseMass=0, 
        baseCollisionShapeIndex=c_id, 
        baseVisualShapeIndex=v_id, 
        basePosition=WORKSPACE_POSITION, 
        baseOrientation=p.getQuaternionFromEuler(WORKSPACE_ORIENTATION)
    )
else:
    print(f"Error: Workspace STL not found at {workspace_path}")

# Data storage
time_axis = []
collision_signal = []
any_collision_occurred = False 

# --- PHASE 1: INSTANT BACKGROUND SAFETY CHECK ---
print(f"\n>>> PHASE 1: Running Safety Check (Safety threshold: {SAFETY_THRESHOLD}m)...")
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0) 

for step_data in steps:
    q_values = step_data["q"]
    for i in range(min(p.getNumJoints(targid), len(q_values))):
        p.resetJointState(targid, i, q_values[i])
    
    p.performCollisionDetection()
    
    # Check contact with the workspace
    close_workspace = p.getClosestPoints(bodyA=targid, bodyB=workspace_id, distance=SAFETY_THRESHOLD)
    
    # Check self contact
    close_self = p.getContactPoints(bodyA=targid, bodyB=targid)

    is_too_close = False
    
    # Analyze workspace collision
    for point in close_workspace:
        if point[8] <= SAFETY_THRESHOLD:
            is_too_close = True
            break
            
    if len(close_self) > 0:
        is_too_close = True
    
    if is_too_close:
        any_collision_occurred = True
    
    time_axis.append(step_data["time"])
    collision_signal.append(1 if is_too_close else 0)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

if any_collision_occurred:
    print(f">>> RESULT: Collision detected! (any_collision_occurred = {any_collision_occurred})")
else:
    print(f">>> RESULT: Path is safe. (any_collision_occurred = {any_collision_occurred})")


# --- PHASE 2: VISUALIZATION ---
print("\n>>> PHASE 2: Visualizing movement...")
p.resetDebugVisualizerCamera(2, 135, -50, WORKSPACE_POSITION)

for i in range(p.getNumJoints(targid)):
    p.setJointMotorControl2(targid, i, p.VELOCITY_CONTROL, force=0)

for idx, step_data in enumerate(steps):
    q_values = step_data["q"]
    
    for i in range(min(p.getNumJoints(targid), len(q_values))):
        p.resetJointState(targid, i, q_values[i])
    
    # Change color based on pre-calculated collision signal
    if collision_signal[idx] == 1:
        for link_index in range(-1, p.getNumJoints(targid)):
            p.changeVisualShape(targid, link_index, rgbaColor=[1, 0, 0, 1])
    else:
        for link_index in range(-1, p.getNumJoints(targid)):
            p.changeVisualShape(targid, link_index, rgbaColor=[1, 1, 1, 1])
    
    p.stepSimulation()
    time.sleep(dt)

p.disconnect()

# --- PLOTTING ---
print("\n>>> Generating Plot...")
plt.figure(figsize=(10, 4))
plt.step(time_axis, collision_signal, where='post', color='red', linewidth=2)
plt.fill_between(time_axis, collision_signal, step="post", alpha=0.2, color='red')
plt.title(f"Collision Signal (Boolean Result: {any_collision_occurred})")
plt.xlabel("Time (s)")
plt.ylabel("Collision (1=Yes, 0=No)")
plt.ylim(-0.2, 1.2)
plt.yticks([0, 1])
plt.grid(True)
plt.show()