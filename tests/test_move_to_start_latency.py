import os
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication

from gui.main_window import MainWindow


class MoveToStartLatencyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_move_to_start_does_not_create_execution_latency_request(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            previous_directory = os.getcwd()
            os.chdir(temp_dir)
            try:
                with patch("gui.main_window.CONFIG_FILE", os.path.join(temp_dir, "rebel_config.json")):
                    window = MainWindow()
                    window.manager.robot = Mock(connected=True)
                    window.path_executor.path_data = [[0.0] * 6]
                    window.move_to_start()

                    self.assertIsNone(window._pending_motion_request)
                    self.assertIsNone(window.execution_latency.snapshot)
                    window.close()
            finally:
                os.chdir(previous_directory)


if __name__ == "__main__":
    unittest.main()
