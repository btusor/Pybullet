import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import json
import sys
import matplotlib.pyplot as plt

# Model configuration
SAFETY_THRESHOLD = 0.02 # 5cm buffer zone

TABLE_POSITION = [-1.25, -0.9, 0.0]
TABLE_ORIENTATION = [1.57, 0, 0]
TABLE_SCALE = 0.001 

POLE_POSITION = [-0.8, -0.4, 0.0]
POLE_ORIENTATION = [1.57, 0, 0]
POLE_SCALE = 0.001 

# Initializations
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.81)

# Load assets and trajectory
pybullet_path = pybullet_data.getDataPath()
current_dir = os.path.dirname(os.path.abspath(__file__))
mesh_dir = os.path.join(current_dir, "meshes")
robot_path = os.path.normpath(os.path.join(mesh_dir, "CS612.urdf"))
trajectory_file = os.path.join(current_dir, "q.json")

# Import data from q.json
with open(trajectory_file, "r") as f:
    trajectory_data = json.load(f)
    steps = trajectory_data["steps"]
    dt = trajectory_data["metadata"]["dt"]

p.loadURDF(os.path.join(pybullet_path, "plane.urdf"), [0, 0, 0])

# Load Robot
sys.stderr = open(os.devnull, 'w')
targid = p.loadURDF(robot_path, [0, 0, 0.5], useFixedBase=True)
sys.stderr = sys.__stderr__

# Load Table
table_path = os.path.join(mesh_dir, "table.stl")
table_id = -1
if os.path.exists(table_path):
    v_id = p.createVisualShape(p.GEOM_MESH, fileName=table_path, meshScale=[TABLE_SCALE]*3)
    c_id = p.createCollisionShape(p.GEOM_MESH, fileName=table_path, meshScale=[TABLE_SCALE]*3, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    table_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=c_id, baseVisualShapeIndex=v_id, basePosition=TABLE_POSITION, baseOrientation=p.getQuaternionFromEuler(TABLE_ORIENTATION))

# Load Pole
pole_path = os.path.join(mesh_dir, "Camera_pole.stl")
pole_id = -1
if os.path.exists(pole_path):
    v_id = p.createVisualShape(p.GEOM_MESH, fileName=pole_path, meshScale=[POLE_SCALE]*3)
    c_id = p.createCollisionShape(p.GEOM_MESH, fileName=pole_path, meshScale=[POLE_SCALE]*3, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    pole_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=c_id, baseVisualShapeIndex=v_id, basePosition=POLE_POSITION, baseOrientation=p.getQuaternionFromEuler(POLE_ORIENTATION))

# Data storage
time_axis = []
collision_signal = []
any_collision_occurred = False 

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
    
    # Check contact with the table
    close_table = p.getClosestPoints(bodyA=targid, bodyB=table_id, distance=SAFETY_THRESHOLD)

    # Check contact with the pole
    close_pole = p.getClosestPoints(bodyA=targid, bodyB=pole_id, distance=SAFETY_THRESHOLD)
    
    # Check self contact
    close_self = p.getContactPoints(bodyA=targid, bodyB=targid)

    # COLLISION LOGIC: 
    # If getClosestPoints returns a list, it means something is within the 'distance' radius.
    # We check if the actual distance (index 8) is less than our threshold.
    is_too_close = False
    
    for point in close_table + close_pole:
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
p.resetDebugVisualizerCamera(6, 135, -40, TABLE_POSITION)

# Disable motors for ghost movement
for i in range(p.getNumJoints(targid)):
    p.setJointMotorControl2(targid, i, p.VELOCITY_CONTROL, force=0)

# Use enumerate so we have 'idx' to look up the collision result
for idx, step_data in enumerate(steps):
    q_values = step_data["q"]
    
    # 1. Move the robot
    for i in range(min(p.getNumJoints(targid), len(q_values))):
        p.resetJointState(targid, i, q_values[i])
    
    # 2. CHANGE COLOR BASED ON PHASE 1 RESULTS
    # We check the signal we recorded in Phase 1 for this specific step
    if collision_signal[idx] == 1:
        # Set robot to bright red if colliding
        for link_index in range(-1, p.getNumJoints(targid)): # -1 handles the base_link too
            p.changeVisualShape(targid, link_index, rgbaColor=[1, 0, 0, 1])
    else:
        # Reset to original color (White/Grey)
        for link_index in range(-1, p.getNumJoints(targid)):
            p.changeVisualShape(targid, link_index, rgbaColor=[1, 1, 1, 1])
    
    # 3. Refresh screen and wait
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

