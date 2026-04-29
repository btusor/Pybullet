import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import matplotlib.pyplot as plt

# Connect to the PyBullet GUI
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.81)

# --- PATH CONFIGURATION ---
pybullet_path = pybullet_data.getDataPath()
current_dir = os.path.dirname(os.path.abspath(__file__))

# Define the path to the meshes folder where the URDF and STLs live
mesh_dir = os.path.join(current_dir, "meshes")
robot_path = os.path.normpath(os.path.join(mesh_dir, "CS612.urdf"))

# --- LOAD ASSETS ---

# Load floor
plane_path = os.path.join(pybullet_path, "plane.urdf")
p.loadURDF(plane_path, [0, 0, 0], [0, 0, 0, 1])

# Load robot (Pointing to the new location in 'meshes')
if os.path.exists(robot_path):
    print(f"Loading CS612 from: {robot_path}")
    targid = p.loadURDF(robot_path, [0, 0, 0], [0, 0, 0, 1], useFixedBase=True)
else:
    print(f"!!! ERROR: URDF not found at {robot_path}")
    p.disconnect()
    exit()

num_joints = p.getNumJoints(targid)
p.resetDebugVisualizerCamera(3, 45, -30, [0, 0, 0.5])

# --- DATA STORAGE ---
tcp_x, tcp_y, tcp_z = [], [], []
tcp_vx, tcp_vy, tcp_vz = [], [], [] 
joint_positions = [[] for _ in range(num_joints)] 

# Simulation settings
duration = 2    # seconds
dt = 0.01       # time step
steps = int(duration / dt)

print(f"Starting simulation for {duration} seconds...")

# --- SIMULATION LOOP ---
for step in range(steps):
    if num_joints > 0:
        for i in range(num_joints):
            # Sine wave movement for testing
            target = 0.5 * np.sin(step * 0.02 + i)
            p.setJointMotorControl2(targid, i, p.POSITION_CONTROL, targetPosition=target)
            
            # Record Joint Angles
            js = p.getJointState(targid, i)
            joint_positions[i].append(js[0])
    
    # Get TCP State (Using the last link index: num_joints - 1)
    state = p.getLinkState(targid, num_joints - 1, computeLinkVelocity=1)
    pos = state[0]
    vel = state[6] # Linear velocity in world space
    
    tcp_x.append(pos[0])
    tcp_y.append(pos[1])
    tcp_z.append(pos[2])
    
    tcp_vx.append(vel[0])
    tcp_vy.append(vel[1])
    tcp_vz.append(vel[2])

    p.stepSimulation()
    time.sleep(dt)

print("Simulation finished. Generating plots...")
time_axis = np.linspace(0, duration, len(tcp_x))

# --- CALCULATION OF MAGNITUDES ---
# Velocity Magnitude (Speed)
tcp_speed = np.sqrt(np.array(tcp_vx)**2 + np.array(tcp_vy)**2 + np.array(tcp_vz)**2)

# Acceleration (Derivative of velocity: a = dv/dt)
tcp_ax = np.diff(tcp_vx) / dt
tcp_ay = np.diff(tcp_vy) / dt
tcp_az = np.diff(tcp_vz) / dt
accel_mag = np.sqrt(tcp_ax**2 + tcp_ay**2 + tcp_az**2)

# --- VISUALIZATION ---

# 1. 3D Trajectory Plot
fig_3d = plt.figure(figsize=(10, 8))
ax_3d = fig_3d.add_subplot(111, projection='3d')

ax_3d.plot(tcp_x, tcp_y, tcp_z, label='TCP Path', color='blue', linewidth=2, alpha=0.7)
ax_3d.scatter(tcp_x[0], tcp_y[0], tcp_z[0], color='green', s=100, label='START', edgecolors='black')
ax_3d.scatter(tcp_x[-1], tcp_y[-1], tcp_z[-1], color='red', s=100, label='END', edgecolors='black')

ax_3d.set_title('CS612 Robot TCP 3D Trajectory')
ax_3d.set_xlabel('X [m]')
ax_3d.set_ylabel('Y [m]')
ax_3d.set_zlabel('Z [m]')
ax_3d.legend()

# 2. Kinematics Dashboard
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# Subplot 1: Joint Angles
for i in range(num_joints):
    axs[0].plot(time_axis, joint_positions[i], label=f'Joint {i}')
axs[0].set_title('Joint Angles vs Time')
axs[0].set_ylabel('Angle [rad]')
axs[0].legend(loc='upper right', fontsize='small', ncol=2)
axs[0].grid(True)

# Subplot 2: TCP Linear Velocity
axs[1].plot(time_axis, tcp_vx, label='Vx', alpha=0.4)
axs[1].plot(time_axis, tcp_vy, label='Vy', alpha=0.4)
axs[1].plot(time_axis, tcp_vz, label='Vz', alpha=0.4)
axs[1].plot(time_axis, tcp_speed, label='Speed (Mag)', color='black', linewidth=2)
axs[1].set_title('TCP Linear Velocity')
axs[1].set_ylabel('Velocity [m/s]')
axs[1].legend(loc='upper right')
axs[1].grid(True)

# Subplot 3: TCP Linear Acceleration
axs[2].plot(time_axis[:-1], tcp_ax, label='Ax', alpha=0.4)
axs[2].plot(time_axis[:-1], tcp_ay, label='Ay', alpha=0.4)
axs[2].plot(time_axis[:-1], tcp_az, label='Az', alpha=0.4)
axs[2].plot(time_axis[:-1], accel_mag, label='Accel (Mag)', color='red', linewidth=2)
axs[2].set_title('TCP Linear Acceleration')
axs[2].set_ylabel('Acceleration [m/s²]')
axs[2].set_xlabel('Time [s]')
axs[2].legend(loc='upper right')
axs[2].grid(True)

plt.tight_layout()
plt.show()