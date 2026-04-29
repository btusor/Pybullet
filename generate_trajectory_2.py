import numpy as np
import json
import os

def generate_l_shape_rotation():
    # Configuration
    duration = 10.0  # seconds
    dt = 0.01        # 100Hz
    num_joints = 6   
    t = np.arange(0, duration, dt)
    
    trajectory_data = {
        "metadata": {
            "duration": duration,
            "dt": dt,
            "num_joints": num_joints
        },
        "steps": []
    }

    print(f"Generating L-shape Z-rotation trajectory...")

    for timestamp in t:
        # Joint 0: Rotates around Z (the 'spinning' of the L)
        # 1.0 rad amplitude at 0.2Hz frequency
        j0_base_rotation = 2 * np.sin(2.5 * np.pi * 0.2 * timestamp)
        
        # Joint 1: Shoulder - tilt it to 45 degrees (approx 0.78 rad)
        j1_shoulder = 0.8
        
        # Joint 2: Elbow - bend it 90 degrees (approx 1.57 rad) to create the 'L' tip
        j2_elbow = 1.57
        
        # Joints 3, 4, 5: Keep wrist straight
        j3_wrist = 0.0
        j4_wrist = 0.0
        j5_wrist = 0.0

        q = [j0_base_rotation, j1_shoulder, j2_elbow, j3_wrist, j4_wrist, j5_wrist]
        
        step_entry = {
            "time": round(timestamp, 3),
            "q": q
        }
        trajectory_data["steps"].append(step_entry)

    # Save to JSON
    file_path = os.path.join(os.path.dirname(__file__), "q.json")
    with open(file_path, "w") as f:
        json.dump(trajectory_data, f, indent=4)
    
    print(f"L-shape trajectory saved to: {file_path}")

if __name__ == "__main__":
    generate_l_shape_rotation()