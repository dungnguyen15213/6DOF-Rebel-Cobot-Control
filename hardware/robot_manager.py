import math
import time
from core.kinematics import JOINT_LIMITS_DEG
from core.path_planner import PathPlanner
from cri_lib.cri_controller import CRIController
from cri_lib.cri_errors import CRICommandTimeOutError


class RebelManager:
    """Handles all communication with the physical Igus ReBeL robot via CRI."""

    DEFAULT_SUPPLY_VOLTAGE_MV = 24000
    WAYPOINT_TOLERANCE_DEG = 1.0
    MIN_WAYPOINT_STEP_DEG = 0.01
    FEEDBACK_MAX_AGE_SEC = 1.0
    WAYPOINT_REACH_TIMEOUT_SEC = 20.0
    WAYPOINT_POLL_SEC = 0.01

    def __init__(self):
        self.robot = CRIController()
        self.simplify_tolerance_deg = 0.0
        self.last_connection_error: str | None = None
        self.current_jog_speeds = {
            "A1": 0.0, "A2": 0.0, "A3": 0.0, "A4": 0.0, "A5": 0.0, "A6": 0.0,
            "E1": 0.0, "E2": 0.0, "E3": 0.0
        }

    def connect(self, ip_address):
        """Creates a fresh controller and connects to the iRC TCP socket."""
        self.last_connection_error = None
        self.robot = CRIController()
        if not self.robot.connect(ip_address, 3920):
            self.last_connection_error = (
                f"CRI TCP connection failed for {ip_address}:3920. "
                "Check the robot IP, network cable, and iRC CRI server."
            )
            return False

        if not self.robot.set_active_control(True):
            self.last_connection_error = (
                "TCP connected, but the robot rejected active control. "
                "Close other CRI clients and verify the robot is ready for external control."
            )
            try:
                self.robot.close()
            except Exception:
                pass
            return False

        return True

    def disconnect(self):
        if self.robot.connected:
            try:
                self.robot.register_status_callback(None)
                self.robot.set_active_control(False)
                self.robot.disable()
            except Exception:
                print("[Warning] Robot ignored polite disconnect. Forcing close...")
            finally:
                self.robot.close()

    def get_joint_angles_list(self):
        """Returns joint angles as a standard Python list for the 3D Viewer."""
        if not self.robot.connected:
            return [0.0] * 6
        j = self.robot.robot_state.joints_current
        return [j.A1, j.A2, j.A3, j.A4, j.A5, j.A6]

    def get_joint_currents_list(self):
        """Returns joint current draw as a standard Python list."""
        with self.robot.robot_state_lock:
            state = self.robot.robot_state
            if "CURRENTJOINTS" not in state.status_fields_seen:
                return [float("nan")] * 6
            currents = list(state.current_joints[:6])
        currents.extend([float("nan")] * (6 - len(currents)))
        return [float(current) for current in currents]

    def get_total_current(self):
        """Returns the sum of the six joint currents in milliamps.

        The robot's CURRENTALL field can stay at zero while joints draw current,
        so it is only used when CURRENTJOINTS is not available.
        """
        with self.robot.robot_state_lock:
            state = self.robot.robot_state
            fields_seen = set(state.status_fields_seen)
            total = float(state.current_total)
            joint_currents = list(state.current_joints[:6])
        if "CURRENTJOINTS" in fields_seen and joint_currents:
            return float(sum(joint_currents))
        if "CURRENTALL" in fields_seen:
            return total
        return float("nan")

    def has_supply_voltage(self):
        return "SUPPLY" in self.robot.robot_state.status_fields_seen

    def get_supply_voltage_mV(self):
        """Returns the fixed nominal supply voltage in millivolts."""
        return float(self.DEFAULT_SUPPLY_VOLTAGE_MV)

    def get_supply_voltage_V(self):
        return self.get_supply_voltage_mV() / 1000.0

    def get_supply_power_W(self):
        """Calculates supply power from fixed 24 V and CURRENTALL in milliamps.

        Formula: P = V * I = 24 V * (CURRENTALL mA / 1000)
        """
        voltage_V = self.get_supply_voltage_V()
        current_mA = self.get_total_current()
        return voltage_V * (current_mA / 1000.0)

    def has_total_current(self):
        return "CURRENTALL" in self.robot.robot_state.status_fields_seen

    def has_joint_currents(self):
        return "CURRENTJOINTS" in self.robot.robot_state.status_fields_seen

    def get_supply_voltage(self):
        """Returns the current supply voltage of the robot in millivolts."""
        return float(self.robot.robot_state.supply_voltage)

    def get_hardware_error(self):
        return self.robot.robot_state.combined_axes_error

    def reset_errors(self):
        self.robot.reset()

    def enable_motors(self):
        self.robot.reset()
        time.sleep(0.1)
        self.robot.enable()

    def reference_all(self):
        self.robot.reference_all_joints()

    def go_home(self, joints, speed):
        """Moves to the custom home position defined in the config file."""
        if len(joints) == 6:
            # move_joints expects: A1, A2, A3, A4, A5, A6, E1, E2, E3, Velocity
            self.robot.move_joints(joints[0], joints[1], joints[2], 
                                   joints[3], joints[4], joints[5], 
                                   0.0, 0.0, 0.0, speed)

    def start_jog(self, axis_key, speed):
        self.current_jog_speeds[axis_key] = speed
        self.robot.start_jog()
        self.robot.set_jog_values(**self.current_jog_speeds)

    def stop_jog(self):
        for key in self.current_jog_speeds:
            self.current_jog_speeds[key] = 0.0
        self.robot.set_jog_values(**self.current_jog_speeds)
        self.robot.stop_jog()
        
    def maintain_jog(self):
        if self.robot.connected and any(v != 0.0 for v in self.current_jog_speeds.values()):
            self.robot.set_jog_values(**self.current_jog_speeds)

    def _joints_near(self, target, tolerance_deg):
        """True/False when fresh joint feedback is within tolerance of target; None if feedback is
        unavailable, stale or non-finite (callers must treat None as NOT reached)."""
        try:
            robot = self.robot
            received_at = getattr(robot, "last_status_received_at", None)
            if received_at is not None and time.monotonic() - received_at > self.FEEDBACK_MAX_AGE_SEC:
                return None
            with robot.robot_state_lock:
                joints = robot.robot_state.joints_current
                measured = [float(v) for v in (joints.A1, joints.A2, joints.A3, joints.A4, joints.A5, joints.A6)]
            if not all(math.isfinite(v) for v in measured):
                return None
            return max(abs(m - float(t)) for m, t in zip(measured, target[:6])) <= tolerance_deg
        except (TypeError, ValueError, AttributeError):
            return None

    def _wait_until_near(self, target, stop_event, tolerance_deg=None):
        """Block until the robot is within tolerance of a waypoint; cancellation is checked first."""
        tolerance_deg = self.WAYPOINT_TOLERANCE_DEG if tolerance_deg is None else tolerance_deg
        deadline = time.monotonic() + self.WAYPOINT_REACH_TIMEOUT_SEC
        while True:
            if stop_event is not None and stop_event.is_set():
                return "Execution stopped by user."
            if not self.robot.connected:
                return "Robot disconnected during execution!"
            near = self._joints_near(target, tolerance_deg)
            if near:
                return None
            if time.monotonic() > deadline:
                if near is None:
                    return "Joint feedback unavailable or stale; waypoint not confirmed."
                return "Robot did not reach the waypoint in time."
            time.sleep(self.WAYPOINT_POLL_SEC)

    def validate_waypoints(self, path_data):
        """Return (cleaned_path, error). Checks shape, finiteness and joint limits; keeps order."""
        if not path_data:
            return None, "Empty path"
        cleaned = []
        for idx, point in enumerate(path_data):
            try:
                values = [float(v) for v in point]
            except (TypeError, ValueError):
                return None, f"Waypoint {idx} is not numeric."
            if len(values) < 6 or not all(math.isfinite(v) for v in values[:6]):
                return None, f"Waypoint {idx} needs 6 finite joint angles."
            for j, (low, high) in enumerate(JOINT_LIMITS_DEG):
                if not low <= values[j] <= high:
                    return None, f"Waypoint {idx}: J{j + 1}={values[j]:.2f} deg is outside {low:.0f}..{high:.0f}."
            cleaned.append(values[:6])
        cleaned = PathPlanner.clean_path(cleaned, self.MIN_WAYPOINT_STEP_DEG, self.simplify_tolerance_deg)
        return cleaned, None

    def execute_path(self, path_data, velocity, acceleration=None, smoothing=None,
                     stop_event=None, replace_loaded_program=False):
        """Run the whole path as ONE controller-managed CRI joint program (velocity only by default).

        All waypoints are buffered via ``PROG ... JOINT`` lines, ``CMD StartProgram`` is sent once
        after every line is acknowledged, and success means ``EXECEND ... PLAN``. No ``Move Joint``
        is ever sent here and there is no fallback: unsupported firmware is reported as an error.
        ``acceleration``/``smoothing`` are optional and omitted from the PROG lines when None.
        """
        if not self.robot.connected:
            return False, "Robot not connected"
        cleaned, error = self.validate_waypoints(path_data)
        if error:
            return False, error
        outcome = self.robot.run_joint_program(
            cleaned, velocity, acceleration, smoothing,
            replace_loaded_program=replace_loaded_program, stop_event=stop_event,
        )
        if outcome.success:
            return True, None
        return False, outcome.message or f"Program ended: {outcome.reason}"

    def controller_has_loaded_program(self):
        return self.robot.has_foreign_program_loaded()

    def stop_execution(self):
        """Stop an active CRI program (not just a Python flag)."""
        try:
            if not self.robot.connected:
                return False
            stopped = self.robot.stop_move()
            return bool(self.robot.request_program_stop() or stopped)
        except Exception:
            return False

    def execute_path_point_to_point(self, path_data, velocity, stop_event=None):
        """Run the CSV waypoints in order using CRI ``Move Joint``, one point after another.

        Each move is sent with ``wait_move_finished=True``: the CRI library waits for the
        controller's EXECEND before the next waypoint is sent, so every point is reached and
        no move is replaced by the following one. Motion between points is stopped/started
        by the controller (not blended).
        """
        cleaned, error = self.validate_waypoints(path_data)
        if error:
            return False, error
        if not self.robot.connected:
            return False, "Robot not connected"

        for idx, angles in enumerate(cleaned):
            if stop_event is not None and stop_event.is_set():
                return False, "Execution stopped by user."
            if not self.robot.connected:
                return False, "Robot disconnected during execution!"
            try:
                success = self.robot.move_joints(
                    A1=angles[0], A2=angles[1], A3=angles[2],
                    A4=angles[3], A5=angles[4], A6=angles[5],
                    E1=0.0, E2=0.0, E3=0.0,
                    velocity=velocity,
                    wait_move_finished=True,
                    move_finished_timeout=60.0,
                )
            except CRICommandTimeOutError:
                return False, f"Timed out on waypoint {idx + 1}/{len(cleaned)}."
            except Exception as e:
                return False, f"Error on waypoint {idx + 1}/{len(cleaned)}: {e}"
            if not success:
                if stop_event is not None and stop_event.is_set():
                    return False, "Execution stopped by user."
                detail = getattr(self.robot, "last_move_error", "")
                return False, f"Waypoint {idx + 1}/{len(cleaned)} failed." + (f" {detail}" if isinstance(detail, str) and detail else "")
        return True, None
