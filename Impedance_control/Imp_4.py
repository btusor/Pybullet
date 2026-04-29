import pybullet as p
import pybullet_data
import time
import numpy as np
import os

# --- 1. Simulation Setup ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

dt = 1./1000.
p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=10)
p.loadURDF("plane.urdf")

script_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(script_dir, "meshes", "CS612.urdf")

# Load Robot - forcing fixed base
robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)

# --- 2. THE ANTI-SPAGHETTI MEASURES ---
# 1. Nail the base down
p.createConstraint(robot_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

num_joints = p.getNumJoints(robot_id)
active_joints = [i for i in range(num_joints) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]
num_dof = len(active_joints)
ee_index = num_joints - 1 

for i in range(num_joints):
    # 2. Inject significant mass and damping to stop the 'drifting'
    p.changeDynamics(robot_id, i, 
                     mass=5.0,            # Heavy links are harder to whip around
                     linearDamping=1.0, 
                     angularDamping=1.0,
                     jointDamping=5.0)    # Internal joint friction

for i in active_joints:
    p.setJointMotorControl2(robot_id, i, p.VELOCITY_CONTROL, force=0)

# --- 3. TARGET & STIFF GAINS ---
target_p = np.array([0.4, 0.0, 0.5]) 
Rd = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]) 

# If it's spaghetti, we need much higher Damping (B) relative to Stiffness (K)
Kt = 3000.0 * np.eye(3) 
Ko = 400.0 * np.eye(3)  
B_task = 150.0  # High task-space damping to stop the drift
B_joint = 60.0  # High joint-space damping to stop the collapse

visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 0.8])
p.createMultiBody(0, -1, visual_id, target_p)

print("Running Anti-Spaghetti Control...")

try:
    while True:
        # A. Feedback
        ee_state = p.getLinkState(robot_id, ee_index, computeLinkVelocity=1)
        curr_p, curr_q = np.array(ee_state[0]), ee_state[1]
        curr_v_lin, curr_v_ang = np.array(ee_state[6]), np.array(ee_state[7])
        
        Rm = np.array(p.getMatrixFromQuaternion(curr_q)).reshape(3, 3)
        joint_states = p.getJointStates(robot_id, active_joints)
        q_active = [s[0] for s in joint_states]
        dq_active = np.array([s[1] for s in joint_states])

        # B. Task-Space Forces
        pos_err = target_p - curr_p
        f_spring = Kt @ pos_err
        f_damping = B_task * (np.zeros(3) - curr_v_lin)
        
        # Orientation
        Re = Rm @ Rd.T
        m_err = -0.5 * np.array([Re[2,1]-Re[1,2], Re[0,2]-Re[2,0], Re[1,0]-Re[0,1]])
        m_spring = Ko @ m_err
        m_damping = 10.0 * (np.zeros(3) - curr_v_ang)
        
        Wrench = np.concatenate([f_spring + f_damping, m_spring + m_damping])
        
        # C. Mapping to Joints
        jac_lin, jac_ang = p.calculateJacobian(robot_id, ee_index, [0, 0, 0], q_active, [0.]*num_dof, [0.]*num_dof)
        J = np.vstack([jac_lin, jac_ang])

        # D. Torque Calculation
        tau_task = J.T @ Wrench
        tau_damping = -B_joint * dq_active
        
        # Manual Gravity Compensation (Since InverseDynamics crashes)
        # We assume 5kg per link, so we push up on the main joints
        grav_comp = np.zeros(num_dof)
        if num_dof >= 3:
            grav_comp[1] = 120.0 # Shoulder lift
            grav_comp[2] = 60.0  # Elbow lift

        # E. Final Output
        for i in range(num_dof):
            torque = np.clip(tau_task[i] + tau_damping[i] + grav_comp[i], -300, 300)
            p.setJointMotorControl2(robot_id, active_joints[i], p.TORQUE_CONTROL, force=torque)

        p.stepSimulation()
        time.sleep(dt)

except Exception as e:
    print(f"Error: {e}")
finally:
    p.disconnect()