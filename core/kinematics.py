import numpy as np
from scipy.optimize import minimize

class ReBelKinematics:
    def __init__(self):
        # ---------------------------------------------------------
        # PHYSICAL LINK LENGTHS (Igus ReBeL 6-DOF-02)
        # Converted to MILLIMETERS (mm)
        # ---------------------------------------------------------
        self.d1 = 150.0  # Height from table to the center of Shoulder
        self.a2 = 239.0  # Upper Arm length
        self.a3 = 239.0  # Forearm length
        self.d4 = 65.0   # Wrist 1 housing
        self.d5 = 65.0   # Wrist 2 housing
        self.d6 = 50.0   # Flange / Tool Tip offset
        self.last_ik_error = ""

    # --- KINEMATIC TRANSFORMATION MATRICES ---
    def R_y(self, theta):
        return np.array([
            [np.cos(theta),  0, np.sin(theta), 0],
            [0,              1, 0,             0],
            [-np.sin(theta), 0, np.cos(theta), 0],
            [0,              0, 0,             1]
        ])

    def R_z(self, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta),  np.cos(theta), 0, 0],
            [0,              0,             1, 0],
            [0,              0,             0, 1]
        ])

    def T_z(self, d):
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, d],
            [0, 0, 0, 1]
        ])

    # --- FORWARD KINEMATICS ---
    def get_stick_points(self, joint_angles_rad):
        """Calculates the 3D coordinates of every joint to draw the robot in the simulator."""
        q1, q2, q3, q4, q5, q6 = joint_angles_rad
        
        T0 = np.eye(4) 
        T1 = T0 @ self.T_z(self.d1) @ self.R_z(q1) 
        T2 = T1 @ self.R_y(q2) 
        T3 = T2 @ self.T_z(self.a2) @ self.R_y(q3)
        T4 = T3 @ self.T_z(self.a3) @ self.R_z(q4)
        T5 = T4 @ self.T_z(self.d4) @ self.R_y(q5)
        T6 = T5 @ self.T_z(self.d5) @ self.R_z(q6)
        T_tip = T6 @ self.T_z(self.d6)

        points = [
            T0[:3, 3], T1[:3, 3], T3[:3, 3], 
            T4[:3, 3], T5[:3, 3], T6[:3, 3], T_tip[:3, 3]
        ]
        return np.array(points)

    def get_joint_positions(self, joint_angles_rad):
        """Return the XYZ positions of joints 1..6 (T1..T6) as an array shape (6,3)."""
        q1, q2, q3, q4, q5, q6 = joint_angles_rad

        T0 = np.eye(4)
        T1 = T0 @ self.T_z(self.d1) @ self.R_z(q1)
        T2 = T1 @ self.R_y(q2)
        T3 = T2 @ self.T_z(self.a2) @ self.R_y(q3)
        T4 = T3 @ self.T_z(self.a3) @ self.R_z(q4)
        T5 = T4 @ self.T_z(self.d4) @ self.R_y(q5)
        T6 = T5 @ self.T_z(self.d5) @ self.R_z(q6)

        joints = [T1[:3, 3], T2[:3, 3], T3[:3, 3], T4[:3, 3], T5[:3, 3], T6[:3, 3]]
        return np.array(joints)

    def get_tip_position(self, joint_angles_rad):
        """Returns ONLY the (X, Y, Z) of the very end of the robot."""
        points = self.get_stick_points(joint_angles_rad)
        return points[-1]

    def get_tip_transform(self, joint_angles_rad):
        """Return the homogeneous transform of the tool tip."""
        q1, q2, q3, q4, q5, q6 = joint_angles_rad
        T1 = self.T_z(self.d1) @ self.R_z(q1)
        T2 = T1 @ self.R_y(q2)
        T3 = T2 @ self.T_z(self.a2) @ self.R_y(q3)
        T4 = T3 @ self.T_z(self.a3) @ self.R_z(q4)
        T5 = T4 @ self.T_z(self.d4) @ self.R_y(q5)
        T6 = T5 @ self.T_z(self.d5) @ self.R_z(q6)
        return T6 @ self.T_z(self.d6)

    # --- INVERSE KINEMATICS (IK) ---
    def inverse_kinematics(self, target_xyz, initial_guess_angles=None,
                           locked_joints=None, target_rotation=None):
        """
        Calculates the 6 joint angles required to reach a specific (X, Y, Z) target in mm.
        Uses a multi-seed approach to prevent getting trapped in local minimums.
        Returns a list of angles in degrees on success, or None if physically unreachable.
        """
        target_xyz = np.array(target_xyz, dtype=float)
        locked_joints = locked_joints or {}
        target_rotation = None if target_rotation is None else np.asarray(target_rotation, dtype=float)

        if any(index < 0 or index >= 6 for index in locked_joints):
            self.last_ik_error = "Invalid locked joint index; expected values from 0 to 5."
            return None

        if target_rotation is not None and target_rotation.shape != (3, 3):
            self.last_ik_error = "Invalid target orientation; expected a 3x3 rotation matrix."
            return None

        # The objective function to minimize (Euclidean Distance to target)
        def objective_function(angles):
            current_tip = self.get_tip_position(angles)
            position_error = np.linalg.norm(current_tip - target_xyz)
            if target_rotation is None:
                return position_error

            current_rotation = self.get_tip_transform(angles)[:3, :3]
            rotation_delta = target_rotation.T @ current_rotation
            rotation_cos = np.clip((np.trace(rotation_delta) - 1.0) / 2.0, -1.0, 1.0)
            orientation_error_rad = np.arccos(rotation_cos)
            return position_error + 100.0 * orientation_error_rad

        # REVISED BOUNDS: 
        # J2 (Shoulder) and J5 (Wrist) loosened to ~120 degrees (+/- 2.1 radians)
        # This stops the math solver from artificially failing when folding inwards.
        bounds = [
            (-np.pi, np.pi),       # J1: Base spin
            (-2.1, 2.1),           # J2: Shoulder pitch
            (-np.pi, np.pi),       # J3: Elbow pitch
            (-np.pi, np.pi),       # J4: Forearm twist
            (-2.1, 2.1),           # J5: Wrist pitch
            (-np.pi, np.pi)        # J6: Flange twist
        ]
        for joint_index, angle_degrees in locked_joints.items():
            angle_radians = np.radians(float(angle_degrees))
            bounds[joint_index] = (angle_radians, angle_radians)

        seeds = []
        
        # Seed 1: The current robot posture (Best for smooth, consecutive movements)
        if initial_guess_angles is not None:
            seeds.append(np.radians(initial_guess_angles))

        # Dynamically calculate where the base (Joint 1) should point to face the target X,Y
        # This helps the solver immensely by giving it the correct horizontal direction instantly.
        j1_target = np.arctan2(target_xyz[1], target_xyz[0])

        # Seed 2: Standard Forward Reach
        seeds.append(np.array([j1_target, 0.0, np.radians(90.0), 0.0, np.radians(45.0), 0.0]))
        
        # Seed 3: Folded Downwards (Perfect for reaching Z: -23.0 or picking from the table)
        seeds.append(np.array([j1_target, np.radians(45.0), np.radians(-90.0), 0.0, np.radians(-45.0), 0.0]))
        
        # Seed 4: Reaching Backwards (Over the shoulder)
        seeds.append(np.array([j1_target + np.pi, np.radians(-45.0), np.radians(90.0), 0.0, np.radians(45.0), 0.0]))
        
        # Seed 5: Straight Up in the air
        seeds.append(np.array([j1_target, 0.0, 0.0, 0.0, 0.0, 0.0]))

        best_result = None
        best_distance = float('inf')

        # Run the solver across our different posture seeds
        for seed in seeds:
            for joint_index, angle_degrees in locked_joints.items():
                seed[joint_index] = np.radians(float(angle_degrees))
            result = minimize(objective_function, seed, method='L-BFGS-B', bounds=bounds)
            
            # Record the best result found so far
            if result.fun < best_distance:
                best_distance = result.fun
                best_result = result
                
            # Early Exit: If we found a mathematical solution within 1.0 mm, stop searching!
            if best_distance < 1.0:
                break

        # Safety Check: Did the absolute best attempt actually reach the target?
        best_angles = best_result.x if best_result is not None else None
        position_error = (
            np.linalg.norm(self.get_tip_position(best_angles) - target_xyz)
            if best_angles is not None else float("inf")
        )
        orientation_error = 0.0
        if best_angles is not None and target_rotation is not None:
            current_rotation = self.get_tip_transform(best_angles)[:3, :3]
            rotation_delta = target_rotation.T @ current_rotation
            orientation_cos = np.clip((np.trace(rotation_delta) - 1.0) / 2.0, -1.0, 1.0)
            orientation_error = float(np.degrees(np.arccos(orientation_cos)))

        if position_error > 1.0 or orientation_error > 1.0:
            constraints = []
            if locked_joints:
                constraints.append("locked joints")
            if target_rotation is not None:
                constraints.append("fixed tool orientation")
            constraint_text = f" with {', '.join(constraints)}" if constraints else ""
            self.last_ik_error = (
                f"No IK solution{constraint_text}: closest position error was "
                f"{position_error:.2f} mm"
            )
            if target_rotation is not None:
                self.last_ik_error += f" and orientation error was {orientation_error:.2f}°."
            else:
                self.last_ik_error += "."
            print(f"[IK Warning] {self.last_ik_error}")
            return None

        # Convert back to degrees for the GUI and Hardware to understand
        self.last_ik_error = ""
        return np.degrees(best_result.x).tolist()