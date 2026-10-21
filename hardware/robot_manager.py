import time
from cri_lib.cri_controller import CRIController
from cri_lib.cri_errors import CRICommandTimeOutError


class RebelManager:
    """Handles all communication with the physical Igus ReBeL robot via CRI."""

    DEFAULT_SUPPLY_VOLTAGE_MV = 24000
    WAYPOINT_TOLERANCE_DEG = 1.0
    # The next waypoint is sent once the robot is this close to the current one, so the
    # controller never decelerates to a stop between points.
    WAYPOINT_BLEND_DEG = 2.0
    PATH_ACCELERATION_PERCENT = 100.0
    WAYPOINT_REACH_TIMEOUT_SEC = 20.0
    WAYPOINT_POLL_SEC = 0.01

    def __init__(self):
        self.robot = CRIController()
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
        """True when measured joints are within tolerance of target; None if feedback is unavailable."""
        try:
            measured = [float(value) for value in self.get_joint_angles_list()]
            return max(abs(m - float(t)) for m, t in zip(measured, target[:6])) <= tolerance_deg
        except (TypeError, ValueError):
            return None

    def _wait_until_near(self, target, stop_event, tolerance_deg=None):
        """Block until the robot is within tolerance of a waypoint, so the next move does not replace it early."""
        tolerance_deg = self.WAYPOINT_BLEND_DEG if tolerance_deg is None else tolerance_deg
        deadline = time.monotonic() + self.WAYPOINT_REACH_TIMEOUT_SEC
        while True:
            near = self._joints_near(target, tolerance_deg)
            if near is None or near:
                return None
            if stop_event is not None and stop_event.is_set():
                return "Execution stopped by user."
            if not self.robot.connected:
                return "Robot disconnected during execution!"
            if time.monotonic() > deadline:
                return "Robot did not reach the waypoint in time."
            time.sleep(self.WAYPOINT_POLL_SEC)

    def execute_path(self, path_data, velocity, stop_event=None):
        """Send ordered targets one at a time, advancing as the robot reaches each waypoint.

        A new CRI ``Move Joint`` replaces the move in progress, so streaming all points at a fixed
        rate makes the robot skip straight to the last one. Each waypoint is therefore sent only
        once the previous one has been reached (within ``WAYPOINT_TOLERANCE_DEG``); CRI keeps
        profiling each move, and the final ``EXECEND`` confirms physical completion.
        """
        if not path_data:
            return False, "Empty path"
        if not self.robot.connected:
            return False, "Robot not connected"

        error_message = None
        final_waypoint_index = len(path_data) - 1
        accel = self.PATH_ACCELERATION_PERCENT
        for idx, angles in enumerate(path_data):
            if stop_event is not None and stop_event.is_set():
                error_message = "Execution stopped by user."
                break
            if not self.robot.connected:
                error_message = "Robot disconnected during execution!"
                break

            try:
                move_args = dict(
                    A1=angles[0], A2=angles[1], A3=angles[2],
                    A4=angles[3], A5=angles[4], A6=angles[5],
                    E1=0.0, E2=0.0, E3=0.0,
                    velocity=velocity,
                    wait_move_finished=idx == final_waypoint_index,
                    move_finished_timeout=60.0,
                )
                success = self.robot.move_joints(acceleration=accel, **move_args)
                if not success and idx == 0 and accel is not None:
                    # Older controller firmware rejects the acceleration argument.
                    accel = None
                    success = self.robot.move_joints(**move_args)
            except CRICommandTimeOutError:
                error_message = f"Timed out sending waypoint {idx}."
                break
            except Exception as e:
                error_message = f"Error sending waypoint {idx}: {e}"
                break
            if not success:
                detail = getattr(self.robot, "last_move_error", "")
                error_message = f"Failed to execute waypoint {idx}." + (f" {detail}" if isinstance(detail, str) and detail else "")
                break
            if idx != final_waypoint_index:
                error_message = self._wait_until_near(angles, stop_event)
                if error_message:
                    error_message = f"Waypoint {idx}: {error_message}"
                    break

        return error_message is None, error_message
