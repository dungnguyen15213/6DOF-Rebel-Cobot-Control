import socket
import threading
import time

class CRIClient:
    def __init__(self, ip='192.168.3.11', port=3920):
        self.ip = ip
        self.port = port
        self.socket = None
        self.is_connected = False
        self.real_joint_angles = [0.0] * 6

    def connect(self):
        """Attempts to connect to the physical ReBeL cobot."""
        if self.is_connected:
            return True
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2.0)
            self.socket.connect((self.ip, self.port))
            self.is_connected = True
            print("Successfully connected to physical hardware.")
            return True
        except Exception as e:
            print(f"Hardware Connection Failed: {e}")
            return False

    def disconnect(self):
        if self.socket and self.is_connected:
            self.socket.close()
            self.is_connected = False
            print("Disconnected from hardware.")

    def send_joint_target(self, angles):
        """Sends a movement command to the real robot using the CRI protocol."""
        if not self.is_connected:
            return
        
        # Format the 6 joint angles into the CRI string format
        # Example: CRISTART 1234 MSG MoveJoints 10.0 20.0 0 0 0 0 CRIEND
        angle_str = " ".join([f"{a:.2f}" for a in angles])
        msg = f"CRISTART 1111 MSG MoveJoints {angle_str} CRIEND\n"
        
        try:
            self.socket.sendall(msg.encode('utf-8'))
        except socket.error as e:
            print(f"Failed to send command: {e}")
            self.disconnect()