# Trajectory Export & Robot Control Guide

## Overview

This guide explains how to export simulation trajectories to formats suitable for controlling the physical Igus ReBeL robot.

## Workflow

### 1. **Simulate a Trajectory**
   - Set **Start Position** (X, Y, Z coordinates in mm)
   - Set **Target Position** (X, Y, Z coordinates in mm)
   - Select a **Motion Algorithm**:
     - Cubic Polynomial (smooth, default)
     - Linear LERP (simple linear interpolation)
     - Quintic Polynomial (smoothest, with zero acceleration at endpoints)
   - Set **Duration** (how long the motion should take in seconds)
   - Click **"🧠 Calculate Path"** button

### 2. **Verify the Simulation**
   - The 3D viewer shows the robot's motion
   - Use the **Playback Controls** to preview:
     - ▶ Play: Animate the trajectory
     - ⏸ Pause: Pause playback
     - ⏹ Reset: Clear the simulation
   - The slider allows you to scrub through individual waypoints

### 3. **Set Robot Execution Parameters**
   - Adjust **Robot Velocity (%)** slider (1-100%)
   - This velocity is used for all export formats

### 4. **Export for Robot Control**

You now have multiple export options:

#### **Option A: Export Pathway CSV (Cartesian)**
- **Button**: 📊 Export Pathway CSV (Cartesian)
- **Purpose**: Visualize the 3D path as joint and end-effector Cartesian coordinates
- **Format**: CSV with columns for each joint position (X, Y, Z) and tool tip position
- **Use Case**: Documentation, visualization in other tools, analysis

#### **Option B: Export Robot Joint Angles**
- **Button**: 🤖 Export Robot Joint Angles
- **Purpose**: Export trajectory as joint angles with timing information
- **Format**: CSV with columns:
  - Step (waypoint index)
  - Time (s) (timestamp from 0 to trajectory duration)
  - J1-J6 (joint angles in degrees)
  - Velocity (%) (robot execution speed)
- **Use Case**: Import into other robotic systems, manual inspection of angles

#### **Option C: Export Robot Commands (CRI)**
- **Button**: ⚙️ Export Robot Commands (CRI)
- **Purpose**: Export as CRI protocol commands ready to send to the robot
- **Format**: CSV with columns:
  - Step (waypoint index)
  - Time (s) (timestamp)
  - CRI Command (complete "CMD Move Joint" command)
- **Example Command**:
  ```
  CMD Move Joint 0.0 -10.0 135.0 0.0 20.0 0.0 0.0 0.0 0.0 50.0
  ```
- **Use Case**: Direct robot control, integration with custom control scripts

#### **Option D: Generate Execution Script**
- **Button**: 🐍 Generate Execution Script
- **Purpose**: Auto-generate a complete Python script that executes the trajectory on the robot
- **Output**: A standalone Python script that:
  - Connects to the robot controller at the specified IP address
  - Enables the robot
  - Executes all waypoints sequentially
  - Handles errors gracefully
  - Provides progress feedback
- **Use Case**: Easy deployment, automated execution, integration into larger systems

## Execution Methods

### **Method 1: Manual Command Input**
Using the CRI Commands export:
1. Export to CSV (Option C)
2. Manually send each command to the robot via the CRI protocol
3. Can be done through any TCP client or custom application

### **Method 2: Python Script**
Using the Generated Execution Script (Option D):

```bash
# Basic execution with configured IP
python execute_trajectory.py

# Override robot IP
python execute_trajectory.py 192.168.0.100
```

### **Method 3: Programmatic Integration**
Using the TrajectoryExecutor class in your own code:

```python
from core.trajectory_executor import TrajectoryExecutor

executor = TrajectoryExecutor()
executor.load_from_simulation(
    joint_angles_list=simulated_path,
    duration=3.0,
    algorithm="Cubic Polynomial"
)

# Export in any format
executor.export_to_csv_joint_angles("my_trajectory.csv", velocity_percent=50.0)
executor.export_to_csv_robot_commands("my_commands.csv", velocity_percent=50.0)
executor.generate_execution_script("run_trajectory.py", robot_ip="192.168.0.1")
```

## Export File Format Details

### Joint Angles CSV
```
Step,Time (s),J1 (°),J2 (°),J3 (°),J4 (°),J5 (°),J6 (°),Velocity (%)
0,0.0000,0.0000,-10.0000,135.0000,0.0000,20.0000,0.0000,50.0
1,0.0050,0.0123,-9.8765,135.2345,0.0456,20.1234,0.0789,50.0
...
```

### Robot Commands CSV
```
Step,Time (s),CRI Command
0,0.0000,"CMD Move Joint 0.0 -10.0 135.0 0.0 20.0 0.0 0.0 0.0 0.0 50.0"
1,0.0050,"CMD Move Joint 0.0123 -9.8765 135.2345 0.0456 20.1234 0.0789 0.0 0.0 0.0 50.0"
...
```

### Cartesian Pathway CSV
```
Step,J1 X,J1 Y,J1 Z,J2 X,...,J6 Z,Tip X,Tip Y,Tip Z
0,0.000000,0.000000,150.000000,...,200.000000,-0.000001,400.000002
...
```

## Tips & Best Practices

1. **Trajectory Duration**
   - Longer durations = slower, more controlled movements
   - Shorter durations = faster movements (check hardware limits)
   - Start with 2-5 seconds for safe testing

2. **Robot Velocity**
   - 30-50% for general operation
   - 10-30% for precise, careful movements
   - 70-100% for fast automated operations
   - Always test with lower speeds first

3. **Algorithm Selection**
   - **Cubic**: Good balance of speed and smoothness (recommended)
   - **Linear**: Simplest, but less smooth motion
   - **Quintic**: Smoothest with minimal acceleration jerks (best for sensitive operations)

4. **Testing Safety**
   - Always test trajectories with reduced velocity first
   - Monitor the robot during first execution
   - Have emergency stop ready
   - Check for collisions in the 3D viewer before execution

5. **Batch Operations**
   - Export multiple trajectories for complex tasks
   - Use the script generation for sequential execution
   - Can modify scripts for conditional logic

## Troubleshooting

### Export Fails with "No Path"
- Run the simulation first (click "Calculate Path")
- Ensure start and target points are reachable

### Robot Commands Won't Execute
- Verify robot IP address in connection settings
- Check robot is powered on and connected
- Ensure velocity is between 1.0-100.0

### Script Can't Connect to Robot
- Check IP address and port (default 3920)
- Verify network connectivity
- Ensure robot controller (iRC) is running

### Trajectory Moves Too Fast/Slow
- Adjust velocity percentage
- Regenerate exports with new velocity
- Modify script and re-run

## File Organization

After exporting, you'll have:
```
project_root/
├── trajectory.csv                    # Original Cartesian path
├── trajectory_joint_angles.csv       # Robot joint angles format
├── trajectory_commands.csv           # CRI commands
├── execute_trajectory.py             # Executable Python script
└── (other project files)
```

## Next Steps

1. **Test with Simulation**: Preview in the 3D viewer
2. **Export Carefully**: Start with joint angles or commands for inspection
3. **Dry Run**: Execute script in offline mode if available
4. **Execute Safely**: Start with reduced velocity on real robot
5. **Monitor**: Watch first execution closely
6. **Optimize**: Adjust duration/velocity/algorithm as needed

## Additional Resources

- **CRI Protocol Documentation**: See `cri_lib/cri_controller.py`
- **Kinematics**: See `core/kinematics.py` for forward/inverse kinematics
- **Robot Manager**: See `hardware/robot_manager.py` for low-level control

---

**Version**: 1.0  
**Last Updated**: 2026-06-23
