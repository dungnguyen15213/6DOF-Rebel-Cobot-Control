# Quick Start: Export Simulation to Robot Control

## In 5 Steps

### 1. **Simulate a Motion**
```
- Set Start Position: [200, 0, 400] mm
- Set Target Position: [200, 200, 200] mm
- Duration: 3.0 seconds
- Click "🧠 Calculate Path"
```

### 2. **Preview the Motion**
```
- Watch the 3D visualization
- Use ▶ Play / ⏸ Pause / ⏹ Reset to preview
- Adjust slider to scrub through waypoints
```

### 3. **Set Robot Speed**
```
- Adjust "Robot Velocity (%)" spinner (1-100)
- Start with 30-50% for testing
- Recommended: 50% for balanced speed/safety
```

### 4. **Choose Export Format**
```
Option A: 📊 Export Pathway CSV (Cartesian)
→ For visualization and analysis

Option B: 🤖 Export Robot Joint Angles  
→ For inspection and other systems

Option C: ⚙️ Export Robot Commands (CRI)
→ For direct robot control

Option D: 🐍 Generate Execution Script
→ For automated robot execution
```

### 5. **Execute**
```
Method 1 - Automatic Script:
  python execute_trajectory.py

Method 2 - Manual Commands:
  Use the CRI command CSV with your control system

Method 3 - Programmatic:
  from core.trajectory_executor import TrajectoryExecutor
  executor.load_from_simulation(path, duration)
  executor.export_to_csv_joint_angles("file.csv")
```

## What Gets Exported?

| Format | File Name | Contains | Use For |
|--------|-----------|----------|---------|
| **Cartesian** | `trajectory.csv` | Joint XYZ positions | Visualization |
| **Joint Angles** | `trajectory_joint_angles.csv` | J1-J6 degrees + timing | Inspection |
| **CRI Commands** | `trajectory_commands.csv` | CMD Move Joint commands | Robot control |
| **Script** | `execute_trajectory.py` | Full Python script | Execution |

## Real Robot Execution

### Safe Testing Workflow:
1. **Export** → Generate Execution Script
2. **Verify** → Check script content and command format
3. **Test with Low Speed** → Run at 20% velocity first
4. **Monitor** → Watch for collisions or unexpected behavior  
5. **Increase Speed** → Gradually increase velocity up to desired level

### Script Execution:
```bash
# Use configured IP from the GUI
python execute_trajectory.py

# Override IP if needed
python execute_trajectory.py 192.168.0.50
```

## Example: Create and Export Your First Trajectory

```python
# Use the examples script
python example_trajectory_export.py
```

This runs 4 examples showing:
- Exporting in all formats
- Robot execution (if connected)
- Trajectory analysis
- Algorithm comparison

## Troubleshooting Quick Fixes

| Problem | Solution |
|---------|----------|
| "No Path" error | Click "Calculate Path" first |
| Export button disabled | Simulate a trajectory first |
| Script can't connect | Check robot IP and network |
| Robot won't move | Check velocity (1-100), power on robot |
| Trajectory too slow | Reduce duration or increase velocity |
| Export location not found | Use absolute paths or check working directory |

## Key Shortcuts

| Action | Shortcut |
|--------|----------|
| Simulate | Click "🧠 Calculate Path" |
| Export | Click any export button 📊/🤖/⚙️/🐍 |
| Play/Pause | Press ▶/⏸ buttons |
| Set Speed | Adjust "Robot Velocity (%)" |

## File Locations

After export, files are in current working directory (usually project root):
```
trajectory.csv                 ← Cartesian coordinates
trajectory_joint_angles.csv   ← Robot joint angles
trajectory_commands.csv       ← CRI commands
execute_trajectory.py         ← Execution script
```

## Important Notes

⚠️ **Always test with reduced velocity first!**
- Start at 20-30% velocity
- Monitor the robot during execution
- Have emergency stop ready
- Check for collision paths in 3D viewer

🎯 **Velocity Guidelines**
- 10-30%: Careful, precise movements
- 30-50%: Normal, safe operation (recommended)
- 70-100%: Fast, automated operation (advanced users only)

📋 **Recommended Workflow**
1. Simulate motion with long duration (3-5s)
2. Export as joint angles for inspection
3. Export script with 30% velocity
4. Test on real robot with monitoring
5. Increase velocity only after successful test

---

**See TRAJECTORY_EXPORT_GUIDE.md for complete documentation**
