import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import json  

# Initialize PyBullet
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) 
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
p.resetSimulation()
p.setGravity(0, 0, -9.81)

# 1. Path Setup
pybullet_path = pybullet_data.getDataPath()
current_dir = os.path.dirname(os.path.abspath(__file__))

# Define the meshes directory path
mesh_dir = os.path.join(current_dir, "meshes")

# Define the robot path (pointing inside the meshes folder)
robot_path = os.path.normpath(os.path.join(mesh_dir, "CS612.urdf"))

# --- TRAJECTORY LOADING ---
trajectory_file = os.path.join(current_dir, "q.json")

if not os.path.exists(trajectory_file):
    print(f"!!! ERROR: Could not find trajectory file at {trajectory_file}")
    p.disconnect()
    exit()

with open(trajectory_file, "r") as f:
    trajectory_data = json.load(f)
    steps = trajectory_data["steps"]
    dt = trajectory_data["metadata"]["dt"]

print(f"Loaded trajectory with {len(steps)} steps.")

if not os.path.exists(robot_path):
    print(f"!!! ERROR: Could not find URDF at {robot_path}")
    print(f"Folder contents: {os.listdir(mesh_dir) if os.path.exists(mesh_dir) else 'Folder not found'}")
    p.disconnect()
    exit()



# Load Ground Plane
plane_path = os.path.join(pybullet_path, "plane.urdf")
p.loadURDF(plane_path, [0, 0, 0], [0, 0, 0, 1])

# Load Robot (CS612)
print(f"Loading Robot from: {robot_path}")
targid = p.loadURDF(robot_path, [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)

num_joints = p.getNumJoints(targid)
print(f"Success! CS612 loaded with {num_joints} joints.")

# Load Table (STL)
scale = 0.001 
table_path = os.path.join(mesh_dir, "table.stl")

if os.path.exists(table_path):
    v_id = p.createVisualShape(p.GEOM_MESH, fileName=table_path, meshScale=[scale, scale, scale])
    c_id = p.createCollisionShape(p.GEOM_MESH, fileName=table_path, meshScale=[scale, scale, scale])
    p.createMultiBody(0, c_id, v_id, [-1, -1, 0], p.getQuaternionFromEuler([1.57, 0, 0]))
    print("Table loaded.")

# --- SIMULATION ---
p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.5])

print("Simulation running... You can now move the camera.")

try:
    for step_data in steps:
        # This keeps the GUI responsive to mouse clicks/drags
        p.getMouseEvents() 
        
        q_values = step_data["q"]
        
        for i in range(min(num_joints, len(q_values))):
            p.setJointMotorControl2(
                bodyIndex=targid,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=q_values[i]
            )
        
        p.stepSimulation()
        time.sleep(dt) 
        
except p.error:
    print("Simulation connection closed.")