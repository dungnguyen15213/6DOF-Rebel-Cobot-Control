import sys
from pathlib import Path


def _ensure_local_venv_on_path() -> None:
    """Add the workspace virtual environment's site-packages to sys.path."""
    project_root = Path(__file__).resolve().parent
    venv_root = project_root / "igus_env"

    candidates = [
        venv_root / "Lib" / "site-packages",
        venv_root / "Lib",
        venv_root / "Scripts",
    ]

    for candidate in candidates:
        if candidate.exists() and str(candidate) not in sys.path:
            sys.path.insert(0, str(candidate))


_ensure_local_venv_on_path()

from PyQt6.QtWidgets import QApplication
from gui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()