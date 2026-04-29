import pybullet as p
import pybullet_data
import time
import numpy as np

# Simulation Setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

dt = 1./1000.
p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=10)

robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
p.loadURDF("plane.urdf")

active_joints = [0, 1, 2, 3, 4, 5] 
ee_index = 6 

for i in range(p.getNumJoints(robot_id)):
    p.setJointMotorControl2(robot_id, i, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robot_id, 6, p.POSITION_CONTROL, targetPosition=0, force=500)

# Controller Parameters
Kt = 1500.0 * np.eye(3) 
Ko = 500.0 * np.eye(3) 
B_joint = 15.0          

# FIXED TARGET POINT
# Change these coordinates to move the robot to a different spot
target_p = np.array([0.5, 0.2, 0.4]) 
target_v_lin = np.array([0.0, 0.0, 0.0]) # Target is stationary
Rd = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]) # Pointing down

# Visual Marker for the target point
visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 0.8])
target_marker = p.createMultiBody(0, -1, visual_id, target_p)

print(f"Holding target point at: {target_p}")

try:
    while True:
        # A. Get Current State
        ee_state = p.getLinkState(robot_id, ee_index, computeLinkVelocity=1)
        curr_p, curr_q = np.array(ee_state[0]), ee_state[1]
        curr_v_lin = np.array(ee_state[6])
        curr_v_ang = np.array(ee_state[7])
        
        Rm = np.array(p.getMatrixFromQuaternion(curr_q)).reshape(3, 3)
        all_states = p.getJointStates(robot_id, range(7))
        q_all = [s[0] for s in all_states]
        dq_active = np.array([all_states[i][1] for i in active_joints])

        # B. Geometric Spring Math (Proportional)
        f_spring = Kt @ (target_p - curr_p)
        
        # Orientation Error via Matrix Log (Skew-symmetric part)
        Re = Rm @ Rd.T
        m_spring_vec = -0.5 * np.array([
            Re[2, 1] - Re[1, 2],
            Re[0, 2] - Re[2, 0],
            Re[1, 0] - Re[0, 1]
        ])
        m_spring = Ko @ m_spring_vec

        # C. Damping Terms
        Kv_task = 50.0 
        f_damping = Kv_task * (target_v_lin - curr_v_lin)
        m_damping = 5.0 * (np.zeros(3) - curr_v_ang)

        # D. Total Wrench & Jacobian Mapping
        Wrench = np.concatenate([f_spring + f_damping, m_spring + m_damping])
        
        jac_lin, jac_ang = p.calculateJacobian(robot_id, ee_index, [0,0,0], q_all, [0.]*7, [0.]*7)
        J = np.vstack([jac_lin, jac_ang])[:, :6]
        
        # E. Control Law
        gravity = np.array(p.calculateInverseDynamics(robot_id, q_all, [0.]*7, [0.]*7))
        tau_task = J.T @ Wrench
        tau_joint_damping = -B_joint * dq_active

        # F. Apply Torques
        for i in range(6):
            idx = active_joints[i]
            total_torque = np.clip(tau_task[i] + tau_joint_damping[i] + gravity[idx], -200, 200)
            p.setJointMotorControl2(robot_id, idx, p.TORQUE_CONTROL, force=total_torque)

        p.stepSimulation()
        time.sleep(dt)

except Exception as e:
    print(f"Error: {e}")
finally:
    p.disconnect()