#!/usr/bin/env python3
"""
Example script demonstrating how to use the TrajectoryExecutor class
for exporting and executing robot trajectories.

This example shows:
1. Loading a simulated trajectory into the executor
2. Exporting in multiple formats
3. Direct robot execution
4. Trajectory manipulation and analysis
"""

import sys
import time
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

from core.trajectory_executor import TrajectoryExecutor
from core.kinematics import ReBelKinematics
from core.trajectory import TrajectoryPlanner
from hardware.robot_manager import RebelManager


def example_1_export_simulated_trajectory():
    """Example 1: Export a simulated trajectory in multiple formats."""
    print("\n" + "="*60)
    print("EXAMPLE 1: Export Simulated Trajectory")
    print("="*60)
    
    # Create components
    kinematics = ReBelKinematics()
    planner = TrajectoryPlanner()
    executor = TrajectoryExecutor()
    
    # Define start and target positions
    start_xyz = [200.0, 0.0, 400.0]      # mm
    target_xyz = [200.0, 200.0, 200.0]   # mm
    duration = 3.0                        # seconds
    
    # Calculate joint angles using inverse kinematics
    print(f"\nStart position: {start_xyz}")
    print(f"Target position: {target_xyz}")
    
    start_angles = kinematics.inverse_kinematics(start_xyz)
    target_angles = kinematics.inverse_kinematics(target_xyz, initial_guess_angles=start_angles)
    
    if start_angles is None or target_angles is None:
        print("❌ Error: Could not reach target position with kinematics!")
        return
    
    print(f"Start angles: {[f'{a:.2f}°' for a in start_angles]}")
    print(f"Target angles: {[f'{a:.2f}°' for a in target_angles]}")
    
    # Generate trajectory using cubic polynomial
    trajectory_path = planner.generate_cubic_path(start_angles, target_angles, duration)
    
    # Load into executor
    executor.load_from_simulation(trajectory_path, duration, algorithm="Cubic Polynomial")
    
    print(f"\n✓ Generated trajectory with {len(trajectory_path)} waypoints")
    print(f"  Duration: {duration} seconds")
    print(f"  Update rate: {executor.trajectory_metadata['update_rate_hz']:.1f} Hz")
    
    # Export in all formats
    output_dir = Path("exported_trajectories")
    output_dir.mkdir(exist_ok=True)
    
    # 1. Joint angles CSV
    joint_angles_file = output_dir / "example_joint_angles.csv"
    executor.export_to_csv_joint_angles(str(joint_angles_file), velocity_percent=50.0)
    print(f"\n✓ Exported joint angles to: {joint_angles_file}")
    
    # 2. Robot commands CSV
    commands_file = output_dir / "example_commands.csv"
    executor.export_to_csv_robot_commands(str(commands_file), velocity_percent=50.0)
    print(f"✓ Exported robot commands to: {commands_file}")
    
    # 3. JSON format
    json_file = output_dir / "example_trajectory.json"
    executor.export_to_json(str(json_file), velocity_percent=50.0)
    print(f"✓ Exported to JSON format: {json_file}")
    
    # 4. Execution script
    script_file = output_dir / "example_execute.py"
    executor.generate_execution_script(
        str(script_file),
        robot_ip="192.168.0.1",
        robot_port=3920,
        velocity_percent=50.0
    )
    print(f"✓ Generated execution script: {script_file}")
    
    # Print summary
    summary = executor.generate_summary_report()
    print(summary)


def example_2_execute_trajectory_on_robot():
    """Example 2: Execute a trajectory directly on a real robot (if connected)."""
    print("\n" + "="*60)
    print("EXAMPLE 2: Execute Trajectory on Robot")
    print("="*60)
    
    # Create robot manager
    robot_manager = RebelManager()
    
    # Try to connect
    robot_ip = "192.168.0.1"
    print(f"\nAttempting to connect to robot at {robot_ip}...")
    
    if not robot_manager.connect(robot_ip):
        print("❌ Could not connect to robot. Check IP address and network connectivity.")
        print("Skipping this example. The generated scripts can be used for actual execution.")
        return
    
    print("✓ Connected to robot")
    
    try:
        # Prepare trajectory (same as Example 1)
        kinematics = ReBelKinematics()
        planner = TrajectoryPlanner()
        
        start_angles = kinematics.inverse_kinematics([200.0, 0.0, 400.0])
        target_angles = kinematics.inverse_kinematics([200.0, 200.0, 200.0], 
                                                      initial_guess_angles=start_angles)
        
        trajectory_path = planner.generate_cubic_path(start_angles, target_angles, 3.0)
        
        # Enable robot
        print("\nEnabling robot...")
        robot_manager.enable_motors()
        time.sleep(0.5)
        
        # Move home first
        print("Moving to home position...")
        home_position = [0.0, -10.0, 135.0, 0.0, 20.0, 0.0]
        robot_manager.go_home(home_position, 30.0)
        
        # Execute trajectory
        print("Executing trajectory...")
        for idx, angles in enumerate(trajectory_path):
            if idx % 10 == 0:  # Print progress every 10 waypoints
                print(f"  Waypoint {idx}/{len(trajectory_path)}")
            
            success = robot_manager.robot.move_joints(
                A1=angles[0], A2=angles[1], A3=angles[2],
                A4=angles[3], A5=angles[4], A6=angles[5],
                E1=0.0, E2=0.0, E3=0.0,
                velocity=50.0,
                wait_move_finished=False
            )
            
            if not success:
                print(f"❌ Failed at waypoint {idx}")
                break
        
        print("\n✓ Trajectory execution completed")
        
    except Exception as e:
        print(f"❌ Error during execution: {e}")
    finally:
        # Cleanup
        try:
            robot_manager.robot.disable()
            print("Robot disabled")
        except:
            pass
        robot_manager.disconnect()
        print("Disconnected from robot")


def example_3_analyze_trajectory():
    """Example 3: Analyze a trajectory without executing it."""
    print("\n" + "="*60)
    print("EXAMPLE 3: Analyze Trajectory")
    print("="*60)
    
    kinematics = ReBelKinematics()
    planner = TrajectoryPlanner()
    executor = TrajectoryExecutor()
    
    # Create trajectory
    start_angles = kinematics.inverse_kinematics([200.0, 0.0, 400.0])
    target_angles = kinematics.inverse_kinematics([200.0, 200.0, 200.0], 
                                                  initial_guess_angles=start_angles)
    trajectory_path = planner.generate_cubic_path(start_angles, target_angles, 3.0)
    
    executor.load_from_simulation(trajectory_path, 3.0, algorithm="Cubic Polynomial")
    
    # Analyze
    print(f"\nTrajectory Analysis:")
    print(f"  Total waypoints: {len(trajectory_path)}")
    print(f"  Duration: 3.0 seconds")
    print(f"  Update rate: {executor.trajectory_metadata['update_rate_hz']:.1f} Hz")
    
    # Analyze joint angle ranges
    import numpy as np
    angles_array = np.array([angles for angles, _ in executor.trajectory_data])
    
    print(f"\nJoint Angle Ranges:")
    for j in range(6):
        min_angle = np.min(angles_array[:, j])
        max_angle = np.max(angles_array[:, j])
        avg_angle = np.mean(angles_array[:, j])
        range_angle = max_angle - min_angle
        print(f"  J{j+1}: Min={min_angle:7.2f}°, Max={max_angle:7.2f}°, Avg={avg_angle:7.2f}°, Range={range_angle:7.2f}°")
    
    # Calculate maximum velocity between waypoints
    print(f"\nMaximum Joint Velocities (degrees per update frame):")
    max_velocities = [0.0] * 6
    for i in range(1, len(angles_array)):
        velocity = np.abs(angles_array[i] - angles_array[i-1])
        max_velocities = np.maximum(max_velocities, velocity)
    
    for j in range(6):
        frame_rate = executor.trajectory_metadata['update_rate_hz']
        velocity_per_second = max_velocities[j] * frame_rate
        print(f"  J{j+1}: {max_velocities[j]:.4f}°/frame ({velocity_per_second:.2f}°/sec)")


def example_4_compare_algorithms():
    """Example 4: Compare different trajectory generation algorithms."""
    print("\n" + "="*60)
    print("EXAMPLE 4: Compare Trajectory Algorithms")
    print("="*60)
    
    kinematics = ReBelKinematics()
    planner = TrajectoryPlanner()
    
    # Generate start and target
    start_angles = kinematics.inverse_kinematics([200.0, 0.0, 400.0])
    target_angles = kinematics.inverse_kinematics([200.0, 200.0, 200.0], 
                                                  initial_guess_angles=start_angles)
    
    algorithms = [
        ("Cubic", planner.generate_cubic_path),
        ("Linear", planner.generate_linear_path),
        ("Quintic", planner.generate_quintic_path),
    ]
    
    import numpy as np
    
    print("\nComparing algorithms for same motion (3 seconds):\n")
    
    for name, algo_func in algorithms:
        trajectory = algo_func(start_angles, target_angles, 3.0)
        angles_array = np.array(trajectory)
        
        # Calculate velocity profile
        velocities = np.abs(np.diff(angles_array, axis=0))
        max_velocity = np.max(velocities)
        avg_velocity = np.mean(velocities)
        
        print(f"{name:10s}: {len(trajectory):3d} waypoints | "
              f"Max vel: {max_velocity:6.3f}°/frame | "
              f"Avg vel: {avg_velocity:6.3f}°/frame")
    
    print("\nNotes:")
    print("  - Cubic: Good balance of smoothness and speed")
    print("  - Linear: Simple but can have jerky acceleration")
    print("  - Quintic: Smoothest, zero acceleration at endpoints")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("TRAJECTORY EXECUTOR EXAMPLES")
    print("="*60)
    
    # Run examples
    example_1_export_simulated_trajectory()
    example_2_execute_trajectory_on_robot()
    example_3_analyze_trajectory()
    example_4_compare_algorithms()
    
    print("\n" + "="*60)
    print("All examples completed!")
    print("="*60)
    print("\nFor more information, see TRAJECTORY_EXPORT_GUIDE.md")
