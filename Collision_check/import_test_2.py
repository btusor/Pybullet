import pybullet as p
import pybullet_data
import time
import os
import json
import sys

# --- CONFIGURATION ---
WORKSPACE_POSITION = [0.5, 1, 0.0]
WORKSPACE_ORIENTATION = [0, 0, -2*1.57] 
WORKSPACE_SCALE = 0.001 

# Initializations
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.81)

# Paths
pybullet_path = pybullet_data.getDataPath()
current_dir = os.path.dirname(os.path.abspath(__file__))
mesh_dir = os.path.normpath(os.path.join(current_dir, "..", "meshes"))

robot_path = os.path.normpath(os.path.join(mesh_dir, "CS612.urdf"))
workspace_path = os.path.normpath(os.path.join(mesh_dir, "Downsized_WORKSPACE.stl"))
trajectory_file = os.path.join(current_dir, "q.json")

# Load trajectory data
with open(trajectory_file, "r") as f:
    trajectory_data = json.load(f)
    steps = trajectory_data["steps"]
    dt = trajectory_data["metadata"]["dt"]

# 1. Load Ground
p.loadURDF(os.path.join(pybullet_path, "plane.urdf"), [0, 0, 0])

# 2. Load Robot
sys.stderr = open(os.devnull, 'w')
targid = p.loadURDF(robot_path, [0, 0, 0.5], useFixedBase=True)
sys.stderr = sys.__stderr__

# 3. Load Workspace
if os.path.exists(workspace_path):
    v_id = p.createVisualShape(p.GEOM_MESH, fileName=workspace_path, meshScale=[WORKSPACE_SCALE]*3)
    c_id = p.createCollisionShape(p.GEOM_MESH, fileName=workspace_path, meshScale=[WORKSPACE_SCALE]*3)
    p.createMultiBody(
        baseMass=0, 
        baseCollisionShapeIndex=c_id, 
        baseVisualShapeIndex=v_id, 
        basePosition=WORKSPACE_POSITION, 
        baseOrientation=p.getQuaternionFromEuler(WORKSPACE_ORIENTATION)
    )
else:
    print(f"Error: STL not found at {workspace_path}")

# 4. Play Trajectory
print("Starting playback...")
p.resetDebugVisualizerCamera(3, 135, -40, WORKSPACE_POSITION)

for step_data in steps:
    q_values = step_data["q"]
    
    # Update joint positions
    for i in range(min(p.getNumJoints(targid), len(q_values))):
        p.resetJointState(targid, i, q_values[i])
    
    p.stepSimulation()
    time.sleep(dt)

print("Playback finished.")
p.disconnect()