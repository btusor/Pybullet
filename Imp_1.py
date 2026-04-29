import pybullet as p
import pybullet_data
import time
import numpy as np

# --- 1. Simulation Setup ---
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
# Using the one URDF we know you have
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

# --- 2. 6-DOF Configuration ---
# We only use the first 6 joints for our Impedance Control
active_joints = [0, 1, 2, 3, 4, 5] 
num_active = len(active_joints)
# We will use joint 6 as the End-Effector, but it will be "locked"
ee_index = 6 

# Disable all velocity motors
for i in range(p.getNumJoints(robot_id)):
    p.setJointMotorControl2(robot_id, i, p.VELOCITY_CONTROL, force=0)

# FORCE THE 7TH JOINT TO BE RIGID (This makes it a 6-DOF arm)
p.setJointMotorControl2(robot_id, 6, p.POSITION_CONTROL, targetPosition=0, force=500)

# --- 3. Control Parameters ---
center_pos = np.array([0.5, 0.0, 0.5])
amplitude = 0.2
frequency = 0.2

Kp = 500.0
Kd = 40.0

# Visual Marker
visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 0.8])
target_marker = p.createMultiBody(0, -1, visual_id, center_pos)

print("Running authentic 6-DOF math on a locked 7-DOF chain...")

# --- 4. Main Control Loop ---
start_time = time.time()

try:
    while True:
        t = time.time() - start_time

        # A. Trajectory
        target_pos = center_pos.copy()
        target_pos[1] = center_pos[1] + amplitude * np.sin(2 * np.pi * frequency * t)
        target_vel = np.array([0, amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t), 0])
        p.resetBasePositionAndOrientation(target_marker, target_pos, [0,0,0,1])

        # B. Get State
        ee_state = p.getLinkState(robot_id, ee_index, computeLinkVelocity=1)
        curr_pos = np.array(ee_state[0])
        curr_vel = np.array(ee_state[6])

        # We MUST get all 7 joint positions to satisfy PyBullet's internal physics
        all_joint_states = p.getJointStates(robot_id, [0,1,2,3,4,5,6])
        q_all = [s[0] for s in all_joint_states]
        v_zero = [0.0] * 7 # Zero velocity for gravity comp

        # C. Physics & Jacobian (The 6-DOF Math)
        # 1. Gravity compensation for all joints
        gravity_torques = p.calculateInverseDynamics(robot_id, q_all, v_zero, v_zero)

        # 2. Get Jacobian (3x7)
        jac_lin, _ = p.calculateJacobian(robot_id, ee_index, [0,0,0], q_all, v_zero, v_zero)
        
        # 3. CONVERT TO 6-DOF: Strip the 7th column
        # J is now 3x6. This is the heart of the 6-DOF control.
        J = np.array(jac_lin)[:, :num_active]

        # D. Impedance Law
        error = target_pos - curr_pos
        error_vel = target_vel - curr_vel
        force = Kp * error + Kd * error_vel 

        # E. Map Force to Torque (J.T is 6x3)
        # result is 6 torques for 6 joints
        impedance_torques = np.dot(J.T, force)
        
        # F. Apply Torques ONLY to the first 6 joints
        for i in range(num_active):
            p.setJointMotorControl2(
                bodyUniqueId=robot_id, 
                jointIndex=active_joints[i], 
                controlMode=p.TORQUE_CONTROL, 
                force=impedance_torques[i] + gravity_torques[i]
            )

        p.stepSimulation()
        time.sleep(1./240.)

except Exception as e:
    print(f"Controller error: {e}")
finally:
    p.disconnect()