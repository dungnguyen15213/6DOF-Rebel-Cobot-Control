import time
from cri_lib.cri_controller import CRIController
from cri_lib.cri_errors import CRICommandTimeOutError


class RebelManager:
    """Handles all communication with the physical Igus ReBeL robot via CRI."""

    DEFAULT_SUPPLY_VOLTAGE_MV = 24000
    PATH_COMMAND_INTERVAL_SEC = 1.0 / 30.0

    def __init__(self):
        self.robot = CRIController()
        self.current_jog_speeds = {
            "A1": 0.0, "A2": 0.0, "A3": 0.0, "A4": 0.0, "A5": 0.0, "A6": 0.0,
            "E1": 0.0, "E2": 0.0, "E3": 0.0
        }

    def connect(self, ip_address):
        """Creates a fresh controller and connects to the iRC TCP socket."""
        self.robot = CRIController()
        if not self.robot.connect(ip_address, 3920):
            return False

        if not self.robot.set_active_control(True):
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
        currents = self.robot.robot_state.current_joints
        return [float(currents[i]) for i in range(6)]

    def get_total_current(self):
        """Returns the total current draw reported by the robot STATUS stream in milliamps.

        If CURRENTALL is not provided by the robot, fall back to summing individual
        CURRENTJOINTS values.
        """
        total = float(self.robot.robot_state.current_total)
        if total == 0.0:
            joint_currents = self.robot.robot_state.current_joints
            if joint_currents:
                return float(sum(joint_currents))
        return total

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

    def execute_path(self, path_data, velocity, stop_event=None):
        """Dispatch ordered targets at 30 Hz and wait for final CRI completion.

        This matches the established execution behavior: intermediate point
        targets are paced over CRI rather than sent as a TCP burst, while CRI
        remains responsible for all motion profiling. The final ``EXECEND``
        confirms physical completion before the UI reports success.
        """
        if not path_data:
            return False, "Empty path"
        if not self.robot.connected:
            return False, "Robot not connected"

        error_message = None
        start_time = time.monotonic()
        final_waypoint_index = len(path_data) - 1
        for idx, angles in enumerate(path_data):
            if stop_event is not None and stop_event.is_set():
                error_message = "Execution stopped by user."
                break
            if not self.robot.connected:
                error_message = "Robot disconnected during execution!"
                break

            dispatch_time = start_time + idx * self.PATH_COMMAND_INTERVAL_SEC
            remaining = dispatch_time - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)

            try:
                success = self.robot.move_joints(
                    A1=angles[0], A2=angles[1], A3=angles[2],
                    A4=angles[3], A5=angles[4], A6=angles[5],
                    E1=0.0, E2=0.0, E3=0.0,
                    velocity=velocity,
                    wait_move_finished=idx == final_waypoint_index,
                    move_finished_timeout=60.0,
                )
            except CRICommandTimeOutError:
                error_message = f"Timed out sending waypoint {idx}."
                break
            except Exception as e:
                error_message = f"Error sending waypoint {idx}: {e}"
                break
            if not success:
                error_message = f"Failed to execute waypoint {idx}."
                break

        return error_message is None, error_message
