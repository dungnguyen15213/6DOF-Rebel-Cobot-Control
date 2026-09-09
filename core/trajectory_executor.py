"""
Trajectory Execution Module
Handles exporting simulated trajectories to formats suitable for robot control
and executing them on the physical robot.
"""

import csv
import json
from typing import List, Tuple, Dict, Optional
from pathlib import Path
import numpy as np


class TrajectoryExecutor:
    """Manages trajectory export and execution on the robot."""
    
    def __init__(self):
        self.trajectory_data = []  # List of (angles, timestamp)
        self.trajectory_metadata = {}
    
    def load_from_simulation(self, joint_angles_list: List[List[float]], 
                            duration: float, algorithm: str = "Cubic"):
        """
        Load trajectory from simulation results.
        
        Parameters:
        -----------
        joint_angles_list : List[List[float]]
            List of joint angle configurations, each as [J1, J2, J3, J4, J5, J6] in degrees
        duration : float
            Total duration of the trajectory in seconds
        algorithm : str
            Name of the trajectory generation algorithm used
        """
        self.trajectory_data = []
        num_frames = len(joint_angles_list)
        
        if num_frames == 0:
            raise ValueError("Empty trajectory data")
        
        # Calculate time for each frame
        for i, angles in enumerate(joint_angles_list):
            timestamp = (i / max(num_frames - 1, 1)) * duration if num_frames > 1 else 0
            self.trajectory_data.append((angles, timestamp))
        
        self.trajectory_metadata = {
            "algorithm": algorithm,
            "total_duration": duration,
            "total_frames": num_frames,
            "update_rate_hz": num_frames / duration if duration > 0 else 30
        }
    
    def export_to_csv_joint_angles(self, filepath: str, velocity_percent: float = 50.0):
        """
        Export trajectory as joint angles with timing information.
        Format suitable for direct robot control.
        
        Parameters:
        -----------
        filepath : str
            Output CSV file path
        velocity_percent : float
            Default velocity percentage for robot execution (1.0-100.0)
        """
        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Header
            header = ["Step", "Time (s)", "J1 (°)", "J2 (°)", "J3 (°)", "J4 (°)", "J5 (°)", "J6 (°)", "Velocity (%)"]
            writer.writerow(header)
            
            # Data rows
            for idx, (angles, timestamp) in enumerate(self.trajectory_data):
                row = [idx, f"{timestamp:.4f}"] + [f"{angle:.4f}" for angle in angles] + [velocity_percent]
                writer.writerow(row)
    
    def export_to_csv_robot_commands(self, filepath: str, velocity_percent: float = 50.0):
        """
        Export trajectory as CRI robot commands ready for execution.
        Each row is a complete 'CMD Move Joint' command.
        
        Parameters:
        -----------
        filepath : str
            Output CSV file path
        velocity_percent : float
            Default velocity percentage for robot execution (1.0-100.0)
        """
        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Header
            header = ["Step", "Time (s)", "CRI Command"]
            writer.writerow(header)
            
            # Data rows
            for idx, (angles, timestamp) in enumerate(self.trajectory_data):
                # Format: CMD Move Joint A1 A2 A3 A4 A5 A6 E1 E2 E3 velocity
                cmd = f"CMD Move Joint {angles[0]:.4f} {angles[1]:.4f} {angles[2]:.4f} " \
                      f"{angles[3]:.4f} {angles[4]:.4f} {angles[5]:.4f} 0.0 0.0 0.0 {velocity_percent:.1f}"
                row = [idx, f"{timestamp:.4f}", cmd]
                writer.writerow(row)
    
    def export_to_json(self, filepath: str, velocity_percent: float = 50.0):
        """
        Export trajectory as JSON format for maximum flexibility.
        
        Parameters:
        -----------
        filepath : str
            Output JSON file path
        velocity_percent : float
            Default velocity percentage for robot execution (1.0-100.0)
        """
        trajectory_list = []
        for idx, (angles, timestamp) in enumerate(self.trajectory_data):
            trajectory_list.append({
                "step": idx,
                "time_seconds": round(timestamp, 4),
                "joint_angles": {
                    "A1": round(angles[0], 4),
                    "A2": round(angles[1], 4),
                    "A3": round(angles[2], 4),
                    "A4": round(angles[3], 4),
                    "A5": round(angles[4], 4),
                    "A6": round(angles[5], 4),
                },
                "end_effector_axes": {
                    "E1": 0.0,
                    "E2": 0.0,
                    "E3": 0.0
                },
                "velocity_percent": velocity_percent
            })
        
        export_data = {
            "metadata": self.trajectory_metadata,
            "trajectory": trajectory_list
        }
        
        with open(filepath, 'w') as jsonfile:
            json.dump(export_data, jsonfile, indent=2)
    
    def generate_execution_script(self, filepath: str, robot_ip: str, 
                                 robot_port: int = 3920, velocity_percent: float = 50.0):
        """
        Generate a Python script that can execute the trajectory on the robot.
        
        Parameters:
        -----------
        filepath : str
            Output Python script file path
        robot_ip : str
            IP address of the robot controller
        robot_port : int
            Port of the robot controller (default 3920)
        velocity_percent : float
            Default velocity percentage for robot execution (1.0-100.0)
        """
        script_content = f'''#!/usr/bin/env python3
"""
Auto-generated robot trajectory execution script.
Executes the simulated trajectory on the physical Igus ReBeL robot.

Generated trajectory metadata:
- Algorithm: {self.trajectory_metadata.get('algorithm', 'Unknown')}
- Total duration: {self.trajectory_metadata.get('total_duration', 0):.2f} seconds
- Total frames: {self.trajectory_metadata.get('total_frames', 0)}
- Update rate: {self.trajectory_metadata.get('update_rate_hz', 30):.1f} Hz
"""

import sys
import time
from cri_lib.cri_controller import CRIController

def execute_trajectory(robot_ip="{robot_ip}", robot_port={robot_port}, velocity={velocity_percent}):
    """Execute the trajectory on the robot."""
    
    # Initialize robot controller
    robot = CRIController()
    
    print(f"Connecting to robot at {{robot_ip}}:{{robot_port}}...")
    if not robot.connect(robot_ip, robot_port):
        print("Failed to connect to robot!")
        return False
    
    print("Robot connected successfully.")
    
    # Enable the robot
    try:
        robot.enable()
        print("Robot enabled.")
        time.sleep(0.5)
    except Exception as e:
        print(f"Failed to enable robot: {{e}}")
        robot.close()
        return False
    
    # Trajectory waypoints
    trajectory = [
'''
        
        # Add trajectory waypoints
        for idx, (angles, timestamp) in enumerate(self.trajectory_data):
            script_content += f'        ({{"time": {timestamp:.4f}, "angles": [{angles[0]:.4f}, {angles[1]:.4f}, {angles[2]:.4f}, {angles[3]:.4f}, {angles[4]:.4f}, {angles[5]:.4f}]}}),\n'
        
        script_content += f'''    ]
    
    print(f"Executing trajectory with {{len(trajectory)}} waypoints...")
    start_time = time.time()
    
    try:
        for idx, waypoint in enumerate(trajectory):
            angles = waypoint["angles"]
            timestamp = waypoint["time"]
            
            # Execute movement to target angles
            success = robot.move_joints(
                A1=angles[0], A2=angles[1], A3=angles[2],
                A4=angles[3], A5=angles[4], A6=angles[5],
                E1=0.0, E2=0.0, E3=0.0,
                velocity=velocity,
                wait_move_finished=False
            )
            
            if not success:
                print(f"Failed to move to waypoint {{idx}}")
                break
            
            if idx % 10 == 0 or idx == len(trajectory) - 1:
                print(f"Progress: {{idx + 1}}/{{len(trajectory)}} waypoints")
        
        print("Trajectory execution completed!")
        elapsed = time.time() - start_time
        print(f"Actual execution time: {{elapsed:.2f}} seconds")
        
    except KeyboardInterrupt:
        print("\\nTrajectory interrupted by user!")
    except Exception as e:
        print(f"Error during trajectory execution: {{e}}")
    finally:
        # Disable robot
        try:
            robot.disable()
            print("Robot disabled.")
        except:
            pass
        robot.close()
    
    return True

if __name__ == "__main__":
    # Optional: Override robot IP from command line argument
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "{robot_ip}"
    execute_trajectory(robot_ip=robot_ip)
'''
        
        with open(filepath, 'w') as scriptfile:
            scriptfile.write(script_content)
    
    def generate_summary_report(self) -> str:
        """Generate a human-readable summary of the trajectory."""
        if not self.trajectory_data:
            return "No trajectory data loaded."
        
        summary = f"""
=== TRAJECTORY SUMMARY ===
Algorithm: {self.trajectory_metadata.get('algorithm', 'Unknown')}
Total Duration: {self.trajectory_metadata.get('total_duration', 0):.2f} seconds
Total Waypoints: {self.trajectory_metadata.get('total_frames', 0)}
Update Rate: {self.trajectory_metadata.get('update_rate_hz', 30):.1f} Hz

Joint Angle Ranges:
"""
        
        # Calculate ranges for each joint
        angles_array = np.array([angles for angles, _ in self.trajectory_data])
        for j in range(6):
            min_angle = np.min(angles_array[:, j])
            max_angle = np.max(angles_array[:, j])
            range_angle = max_angle - min_angle
            summary += f"  J{j+1}: [{min_angle:.2f}°, {max_angle:.2f}°] (range: {range_angle:.2f}°)\n"
        
        return summary
