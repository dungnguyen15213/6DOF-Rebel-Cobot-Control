# Exported Trajectory Files - Format Reference

This document explains the format of each exported trajectory file.

## Overview

When you export a simulated trajectory, you get different formats depending on your needs:

```
Simulation → Export Options → Output Files
                           ├→ trajectory.csv (Cartesian coordinates)
                           ├→ trajectory_joint_angles.csv (Joint angles + timing)
                           ├→ trajectory_commands.csv (CRI protocol commands)
                           ├→ trajectory.json (JSON format)
                           └→ execute_trajectory.py (Executable script)
```

---

## 1. CARTESIAN COORDINATES CSV
**File**: `trajectory.csv`  
**Purpose**: 3D visualization and analysis

### Format
```csv
Step,J1 X,J1 Y,J1 Z,J2 X,J2 Y,J2 Z,J3 X,J3 Y,J3 Z,J4 X,J4 Y,J4 Z,J5 X,J5 Y,J5 Z,J6 X,J6 Y,J6 Z,Tip X,Tip Y,Tip Z
0,0.000000,0.000000,150.000000,0.000000,0.000000,150.000000,...,200.000000,-0.000001,400.000002
1,0.000000,0.000000,150.000000,...
```

### Columns Explanation
- **Step**: Waypoint index (0, 1, 2, ...)
- **J1-J6 X/Y/Z**: Cartesian coordinates of each joint (in mm)
  - J1: Base joint
  - J2-J6: Arm joints
- **Tip X/Y/Z**: End effector position (in mm)

### Use Cases
- Visualize the 3D path in CAD software
- Analyze joint movements
- Documentation and presentations
- Import into simulation tools

### Example Analysis
```python
import pandas as pd
df = pd.read_csv('trajectory.csv')
print(f"Total waypoints: {len(df)}")
print(f"End effector path:")
print(df[['Step', 'Tip X', 'Tip Y', 'Tip Z']].head(10))
```

---

## 2. JOINT ANGLES CSV
**File**: `trajectory_joint_angles.csv`  
**Purpose**: Direct joint angle inspection and robot command generation

### Format
```csv
Step,Time (s),J1 (°),J2 (°),J3 (°),J4 (°),J5 (°),J6 (°),Velocity (%)
0,0.0000,0.0000,-10.0000,135.0000,0.0000,20.0000,0.0000,50.0
1,0.0050,0.0123,-9.8765,135.2345,0.0456,20.1234,0.0789,50.0
2,0.0100,0.0492,-9.5057,135.9246,0.1823,20.4939,0.3153,50.0
...
```

### Columns Explanation
- **Step**: Waypoint sequence number
- **Time (s)**: Timestamp from start of trajectory (0.0 to duration)
- **J1-J6 (°)**: Joint angle in degrees
  - J1: Base rotation (-180 to +180°)
  - J2: Shoulder (-120 to +120°)
  - J3: Elbow (-120 to +120°)
  - J4: Wrist 1 (-180 to +180°)
  - J5: Wrist 2 (-120 to +120°)
  - J6: Wrist 3 (-180 to +180°)
- **Velocity (%)**: Robot speed percentage (1.0-100.0)

### Use Cases
- Verify joint angles before execution
- Manually control robot via terminal
- Import into other robot control systems
- Analyze motion profiles
- Check for hardware limits violations

### Example Usage
```python
import pandas as pd
import numpy as np

df = pd.read_csv('trajectory_joint_angles.csv')

# Check angle ranges
for i in range(1, 7):
    col = f'J{i} (°)'
    print(f"{col}: {df[col].min():.2f}° to {df[col].max():.2f}°")

# Calculate joint velocities
velocities = np.diff(df[['J1 (°)', 'J2 (°)', 'J3 (°)', 'J4 (°)', 'J5 (°)', 'J6 (°)']].values, axis=0)
print(f"Max velocity between frames: {np.max(np.abs(velocities)):.4f}°/frame")
```

---

## 3. ROBOT COMMANDS CSV
**File**: `trajectory_commands.csv`  
**Purpose**: Direct robot control via CRI protocol

### Format
```csv
Step,Time (s),CRI Command
0,0.0000,"CMD Move Joint 0.0 -10.0 135.0 0.0 20.0 0.0 0.0 0.0 0.0 50.0"
1,0.0050,"CMD Move Joint 0.0123 -9.8765 135.2345 0.0456 20.1234 0.0789 0.0 0.0 0.0 50.0"
2,0.0100,"CMD Move Joint 0.0492 -9.5057 135.9246 0.1823 20.4939 0.3153 0.0 0.0 0.0 50.0"
...
```

### Command Format
```
CMD Move Joint A1 A2 A3 A4 A5 A6 E1 E2 E3 velocity
```

| Parameter | Range | Description |
|-----------|-------|-------------|
| A1-A6 | degrees | Joint angles |
| E1-E3 | degrees | End-effector/extension axes (usually 0) |
| velocity | 1.0-100.0 | Movement speed as percentage |

### Use Cases
- Send commands directly to robot via TCP
- Batch process multiple trajectories
- Custom robot control applications
- Protocol verification

### Example: Send Commands to Robot
```python
import pandas as pd
import socket

df = pd.read_csv('trajectory_commands.csv')

# Connect to robot (example)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.0.1', 3920))  # Robot IP and CRI port

# Send each command
for idx, row in df.iterrows():
    cmd = row['CRI Command']
    print(f"Sending: {cmd}")
    sock.send(cmd.encode() + b'\r\n')
    # Wait for robot response, etc.

sock.close()
```

---

## 4. JSON FORMAT
**File**: `trajectory.json`  
**Purpose**: Programmatic access, flexibility, API integration

### Format
```json
{
  "metadata": {
    "algorithm": "Cubic Polynomial",
    "total_duration": 3.0,
    "total_frames": 180,
    "update_rate_hz": 60.0
  },
  "trajectory": [
    {
      "step": 0,
      "time_seconds": 0.0,
      "joint_angles": {
        "A1": 0.0,
        "A2": -10.0,
        "A3": 135.0,
        "A4": 0.0,
        "A5": 20.0,
        "A6": 0.0
      },
      "end_effector_axes": {
        "E1": 0.0,
        "E2": 0.0,
        "E3": 0.0
      },
      "velocity_percent": 50.0
    },
    ...
  ]
}
```

### Use Cases
- REST API integration
- Web application import
- Python/JS application parsing
- Data exchange with other systems
- Database storage

### Example: Load and Process
```python
import json

with open('trajectory.json') as f:
    data = json.load(f)

print(f"Algorithm: {data['metadata']['algorithm']}")
print(f"Duration: {data['metadata']['total_duration']}s")
print(f"Waypoints: {len(data['trajectory'])}")

for waypoint in data['trajectory'][:5]:
    angles = waypoint['joint_angles']
    print(f"Step {waypoint['step']}: J1={angles['A1']:.2f}°, Time={waypoint['time_seconds']:.4f}s")
```

---

## 5. PYTHON EXECUTION SCRIPT
**File**: `execute_trajectory.py`  
**Purpose**: Standalone robot execution

### Features
- Complete executable script
- Robot connection handling
- Error management
- Progress reporting
- Configurable velocity
- Command-line IP override

### Usage
```bash
# Use configured IP from GUI
python execute_trajectory.py

# Override with different IP
python execute_trajectory.py 192.168.0.100
```

### What It Does
1. Connects to robot at specified IP
2. Enables motors
3. Executes trajectory waypoint by waypoint
4. Handles errors gracefully
5. Reports progress
6. Cleans up on completion

### File Structure
```python
#!/usr/bin/env python3
"""Auto-generated robot trajectory execution script."""

from cri_lib.cri_controller import CRIController

def execute_trajectory(robot_ip="192.168.0.1", ...):
    # Connection and execution logic
    ...

if __name__ == "__main__":
    execute_trajectory()
```

### Customization
You can edit the generated script to:
- Add pre/post-motion routines
- Implement conditional logic
- Add sensor feedback handling
- Create sequences of trajectories
- Add safety checks

Example modification:
```python
# Before executing, move to safe position
robot.move_joints(...home_position..., velocity=30.0, wait_move_finished=True)

# Then execute trajectory
execute_trajectory(robot_ip="192.168.0.1")

# After completion, return home
robot.move_joints(...home_position..., velocity=30.0, wait_move_finished=True)
```

---

## File Comparison

| Aspect | Cartesian CSV | Joint Angles CSV | CRI Commands CSV | JSON | Python Script |
|--------|---|---|---|---|---|
| **Human Readable** | Excellent | Good | Good | Good | Excellent |
| **Robot Ready** | No | No | Yes | No | Yes |
| **Editable** | Hard | Easy | Medium | Easy | Very Easy |
| **Import Other Tools** | Yes | Yes | No | Yes | No |
| **Executable** | No | No | No | No | Yes |
| **File Size** | Large | Medium | Medium | Medium | Large |
| **Processing** | Python/Excel | Python/Excel | Command Loop | Python/JS | Python Direct |

---

## Tips

1. **Always Inspect Before Executing**
   - Open joint angles CSV
   - Verify angle ranges
   - Check time values

2. **Test with Reduced Velocity**
   - Generate script at 30% velocity
   - Test before increasing to 100%
   - Monitor robot during execution

3. **Keep Exports Organized**
   ```
   trajectories/
   ├── pick_place_50percent/
   │   ├── trajectory.csv
   │   ├── trajectory_joint_angles.csv
   │   └── execute_trajectory.py
   └── assembly_30percent/
       └── ...
   ```

4. **Document Your Trajectories**
   ```
   # Add to script comments
   # Motion: Pick object from position A, place at position B
   # Velocity: 50%
   # Duration: 3 seconds
   # Safety: Clear work space before execution
   ```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| CSV won't open in Excel | Use UTF-8 encoding, try opening as text first |
| JSON parse error | Validate with `json.tool` or online validator |
| Script won't connect | Check robot IP, network, firewall |
| Angles out of range | Check kinematics or original trajectory generation |
| Wrong coordinate system | Verify mm vs cm, degree vs radian |

---

**For complete usage guide, see TRAJECTORY_EXPORT_GUIDE.md**
