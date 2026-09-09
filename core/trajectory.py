import numpy as np

class TrajectoryPlanner:
    def __init__(self, update_rate_hz=30):
        """
        update_rate_hz: How many times per second the system loop runs.
        30Hz matches the 33ms timer in your PyQt app.
        """
        self.dt = 1.0 / update_rate_hz

    def generate_cubic_path(self, start_angles, target_angles, duration):
        """
        Generates a smooth point-to-point trajectory using a cubic polynomial.
        Formula: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        Ensures zero velocity at the start and end
        """
        start = np.array(start_angles)
        target = np.array(target_angles)
        
        # Number of frames/steps needed
        steps = int(duration / self.dt)
        if steps < 1:
            return [target.tolist()]

        path = []
        for i in range(steps + 1):
            t = i * self.dt
            
            # Cubic easing coefficients
            # a0 = start
            # a1 = 0 (start velocity)
            # a2 = 3 * (target - start) / duration^2
            # a3 = -2 * (target - start) / duration^3
            
            a2 = 3.0 * (target - start) / (duration ** 2)
            a3 = -2.0 * (target - start) / (duration ** 3)
            
            # Calculate position at time t
            q_t = start + a2 * (t ** 2) + a3 * (t ** 3)
            path.append(q_t.tolist())
            
        return path

    def generate_linear_path(self, start_angles, target_angles, duration):
        """
        Generates a simple linear interpolation (LERP) between start and target joint angles.
        """
        start = np.array(start_angles)
        target = np.array(target_angles)

        steps = int(duration / self.dt)
        if steps < 1:
            return [target.tolist()]

        path = []
        for i in range(steps + 1):
            t = i * self.dt
            alpha = min(max(t / duration, 0.0), 1.0)
            q_t = (1.0 - alpha) * start + alpha * target
            path.append(q_t.tolist())

        return path

    def generate_quintic_path(self, start_angles, target_angles, duration):
        """
        Generates a smooth point-to-point trajectory using a quintic polynomial.
        Ensures zero velocity and zero acceleration at start and end.
        q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        """
        start = np.array(start_angles)
        target = np.array(target_angles)

        steps = int(duration / self.dt)
        if steps < 1:
            return [target.tolist()]

        T = duration
        delta = target - start

        a0 = start
        a1 = np.zeros_like(start)
        a2 = np.zeros_like(start)
        a3 = 10.0 * delta / (T ** 3)
        a4 = -15.0 * delta / (T ** 4)
        a5 = 6.0 * delta / (T ** 5)

        path = []
        for i in range(steps + 1):
            t = i * self.dt
            q_t = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3) + a4 * (t ** 4) + a5 * (t ** 5)
            path.append(q_t.tolist())

        return path

    def generate_cartesian_line(self, start_xyz, target_xyz, duration, solve_waypoint):
        """Generate joint waypoints whose tool tip follows a straight XYZ line.

        ``solve_waypoint`` receives ``(xyz, previous_angles)`` and must return
        the joint solution for that point, or ``None`` when the point cannot
        be reached under the requested constraints.
        """
        start = np.asarray(start_xyz, dtype=float)
        target = np.asarray(target_xyz, dtype=float)
        steps = max(1, int(round(duration / self.dt)))
        path = []
        previous_angles = None

        for step in range(steps + 1):
            alpha = step / steps
            point = (1.0 - alpha) * start + alpha * target
            angles = solve_waypoint(point.tolist(), previous_angles)
            if angles is None:
                raise ValueError(f"Cartesian waypoint {step + 1}/{steps + 1} is unreachable")
            path.append(angles)
            previous_angles = angles

        return path

    def validate_trajectory(self, path):
        """
        Checks the generated path to ensure no joint exceeds maximum physical velocities.
        Returns: (is_safe: bool, error_message: str)
        """
        if not path or len(path) < 2:
            return True, ""

        # Set a safe maximum joint speed limit (e.g., 180 degrees per second)
        MAX_SPEED_DPS = 180.0

        for i in range(1, len(path)):
            prev_angles = path[i-1]
            curr_angles = path[i]

            for joint_idx in range(6):
                # Calculate how far the joint moved in this single frame
                delta_angle = abs(curr_angles[joint_idx] - prev_angles[joint_idx])
                
                # Calculate the speed (Degrees per Second)
                speed = delta_angle / self.dt

                if speed > MAX_SPEED_DPS:
                    error_msg = (f"Joint {joint_idx + 1} is moving too fast!\n"
                                 f"Required Speed: {speed:.1f}°/sec\n"
                                 f"Hardware Limit: {MAX_SPEED_DPS}°/sec")
                    return False, error_msg

        return True, "Trajectory is safe."