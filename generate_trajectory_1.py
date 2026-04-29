import numpy as np
import json
import os

def generate_sinusoidal_trajectory():
    # Configuration
    duration = 10.0  # seconds
    dt = 0.01        # 100Hz
    num_joints = 6   # Standard for CS612
    t = np.arange(0, duration, dt)
    
    trajectory_data = {
        "metadata": {
            "duration": duration,
            "dt": dt,
            "num_joints": num_joints
        },
        "steps": []
    }

    print(f"Generating {duration}s trajectory...")

    for i, timestamp in enumerate(t):
        # Create a unique sine wave for each joint
        # Format: q = amplitude * sin(frequency * t + phase)
        q = [0.5 * np.sin(2 * np.pi * 0.2 * timestamp + (j * 0.5)) for j in range(num_joints)]
        
        step_entry = {
            "time": round(timestamp, 3),
            "q": q
        }
        trajectory_data["steps"].append(step_entry)

    # Save to JSON
    file_path = os.path.join(os.path.dirname(__file__), "q.json")
    with open(file_path, "w") as f:
        json.dump(trajectory_data, f, indent=4)
    
    print(f"Trajectory saved successfully to: {file_path}")

if __name__ == "__main__":
    generate_sinusoidal_trajectory()