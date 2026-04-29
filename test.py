import pybullet as p
import pybullet_data
import time
import os

## Setup
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

## Paths
current_dir = os.path.dirname(os.path.abspath(__file__))
mesh_dir = os.path.join(current_dir, "meshes")
robot_path = os.path.normpath(os.path.join(mesh_dir, "CS612.urdf"))

## Load Assets
p.loadURDF("plane.urdf")

# cameraDistance: Lower numbers = closer zoom
# cameraYaw/Pitch: Angles to look from
# cameraTargetPosition: [x, y, z] coordinates the camera is looking at
p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 1.2])

if not os.path.exists(robot_path):
    print(f"FAILED TO FIND URDF AT: {robot_path}")
else:
    # Load the robot. basePosition moves it up slightly so it doesn't clip the floor
    robo = p.loadURDF(robot_path, basePosition=[0, 0, 0.05], useFixedBase=True)
    
    print(f"Robot successfully loaded.")
    print(f"Total joints/links found: {p.getNumJoints(robo)}")
    
    # Optional: Print the names of all links to verify link7_tcp is there
    for i in range(p.getNumJoints(robo)):
        info = p.getJointInfo(robo, i)
        print(f"Index: {i}, Name: {info[12].decode('utf-8')}, Type: {info[2]}")

    print("\n--- STATIC VIEW ACTIVE ---")
    print("Close the window or press Ctrl+C to exit.")

    # Keep the simulation running so the window stays open
    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1/240.0)
    except KeyboardInterrupt:
        pass

p.disconnect()