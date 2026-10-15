"""Trajectory Planning Studio window.

Authoring, preview and validation only. This module never imports the robot
layer; it can hand a validated CSV to the existing Pathways workflow through the
``loadRequested`` signal, after which execution stays under the main window's
operator-controlled Run button.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Callable

from PyQt6.QtCore import QAbstractTableModel, QModelIndex, Qt, pyqtSignal
from PyQt6.QtGui import QAction, QColor, QGuiApplication, QKeySequence, QUndoCommand, QUndoStack
from PyQt6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDockWidget, QDoubleSpinBox,
    QFileDialog, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QInputDialog, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QSlider, QSpinBox, QSplitter,
    QTableView, QTableWidget, QTableWidgetItem, QTabWidget, QToolBar, QTreeWidget, QTreeWidgetItem,
    QVBoxLayout, QWidget,
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

from trajectory_studio import generators as gen
from trajectory_studio.experiment_planner import Campaign, coverage
from trajectory_studio.library import Library
from trajectory_studio.models import CATEGORIES, JOINT_NAMES, Trajectory, Waypoint
from trajectory_studio.serialization import (
    CsvImportError, export_csv, import_csv, load_recipe, save_recipe, sha256_file,
)
from trajectory_studio.validators import ERROR, validate
from trajectory_studio.visualization import draw_coverage, draw_positions

LIGHT_STYLE = """
QMainWindow, QDockWidget, QDialog { background: #f4f6f8; }
QWidget { font-family: 'Segoe UI', 'Helvetica Neue', sans-serif; font-size: 10pt; color: #1b2733; }
QGroupBox { border: 1px solid #c7d0d9; border-radius: 6px; margin-top: 10px; padding: 8px; background: #ffffff; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; font-weight: 600; }
QPushButton { background: #2f6fed; color: white; border: none; border-radius: 4px; padding: 5px 12px; }
QPushButton:hover { background: #1f5bd1; } QPushButton:disabled { background: #aab4c0; }
QPushButton#secondary { background: #e3e8ee; color: #1b2733; }
QTableView { gridline-color: #d9e0e7; alternate-background-color: #f0f4f8; selection-background-color: #cfe0ff; selection-color: #0b1b33; }
QHeaderView::section { background: #e8edf3; padding: 4px; border: none; border-right: 1px solid #d0d8e0; font-weight: 600; }
QPlainTextEdit, QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox, QTreeWidget { background: white; border: 1px solid #c7d0d9; border-radius: 3px; padding: 2px; }
QToolBar { spacing: 6px; background: #e8edf3; border: none; padding: 3px; }
"""
DARK_STYLE = LIGHT_STYLE.replace("#f4f6f8", "#1e252d").replace("#ffffff", "#2a323b").replace("#1b2733", "#e7edf3") \
    .replace("#f0f4f8", "#252d36").replace("#e8edf3", "#323c47").replace("#e3e8ee", "#3b4652") \
    .replace("#c7d0d9", "#46525f").replace("white;", "#2a323b;").replace("#cfe0ff", "#37507a").replace("#0b1b33", "#ffffff")

COL_INDEX, COL_PHASE, COL_J0, COL_VEL, COL_NOTE = 0, 1, 2, 8, 9
HEADERS = ["#", "Phase", *[f"{n} (deg)" for n in JOINT_NAMES], "Velocity (%)", "Note"]


def _state(traj: Trajectory):
    return (traj.snapshot(), traj.velocity_percent, list(traj.locked_joints))


class SnapshotCommand(QUndoCommand):
    """Undo step that swaps the editable state; the first redo() is skipped because the edit is already applied."""

    def __init__(self, model: "WaypointTableModel", text: str, before, after):
        super().__init__(text)
        self._model, self._before, self._after, self._first = model, before, after, True

    def redo(self):
        if self._first:
            self._first = False
            return
        self._model.restore(self._after)

    def undo(self):
        self._model.restore(self._before)


class WaypointTableModel(QAbstractTableModel):
    changed = pyqtSignal()

    def __init__(self, traj: Trajectory | None = None, parent=None):
        super().__init__(parent)
        self.traj = traj or Trajectory()
        self.undo_stack = QUndoStack(self)

    # --- Qt model API ---
    def rowCount(self, parent=QModelIndex()):
        return 0 if parent.isValid() else len(self.traj.waypoints)

    def columnCount(self, parent=QModelIndex()):
        return 0 if parent.isValid() else len(HEADERS)

    def headerData(self, section, orientation, role=Qt.ItemDataRole.DisplayRole):
        if role == Qt.ItemDataRole.DisplayRole and orientation == Qt.Orientation.Horizontal:
            return HEADERS[section]
        return None

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):
        if not index.isValid():
            return None
        w, c = self.traj.waypoints[index.row()], index.column()
        if role in (Qt.ItemDataRole.DisplayRole, Qt.ItemDataRole.EditRole):
            if c == COL_INDEX:
                return index.row()
            if c == COL_PHASE:
                return w.phase
            if COL_J0 <= c < COL_J0 + 6:
                return f"{w.joints[c - COL_J0]:.4f}" if role == Qt.ItemDataRole.DisplayRole else w.joints[c - COL_J0]
            if c == COL_VEL:
                return "" if self.traj.velocity_percent is None else f"{self.traj.velocity_percent:g}"
            return w.note
        if role == Qt.ItemDataRole.TextAlignmentRole and COL_J0 <= c < COL_J0 + 6:
            return int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        if role == Qt.ItemDataRole.ForegroundRole and COL_J0 <= c < COL_J0 + 6:
            if self.traj.locked_joints[c - COL_J0]:
                return QColor("#7a8794")
        if role == Qt.ItemDataRole.BackgroundRole and c == COL_PHASE and w.phase == "PREPARATION":
            return QColor("#ffe9b8")
        return None

    def flags(self, index):
        base = Qt.ItemFlag.ItemIsSelectable | Qt.ItemFlag.ItemIsEnabled
        c = index.column()
        if COL_J0 <= c < COL_J0 + 6 and not self.traj.locked_joints[c - COL_J0] or c == COL_NOTE:
            base |= Qt.ItemFlag.ItemIsEditable
        return base

    def setData(self, index, value, role=Qt.ItemDataRole.EditRole):
        if role != Qt.ItemDataRole.EditRole or not index.isValid():
            return False
        c = index.column()
        if COL_J0 <= c < COL_J0 + 6:
            try:
                number = float(str(value).strip())
            except ValueError:
                self.error = "Joint angle must be a number."
                return False
            if not math.isfinite(number):
                self.error = "Joint angle must be finite."
                return False
            if self.traj.locked_joints[c - COL_J0]:
                self.error = f"{JOINT_NAMES[c - COL_J0]} is locked."
                return False
            self.edit(f"Edit {JOINT_NAMES[c - COL_J0]} @ row {index.row()}",
                      lambda t: t.set_joint(index.row(), c - COL_J0, number))
            return True
        if c == COL_NOTE:
            self.edit(f"Edit note @ row {index.row()}", lambda t: setattr(t.waypoints[index.row()], "note", str(value)))
            return True
        return False

    # --- undoable edits ---
    def edit(self, text: str, fn: Callable[[Trajectory], None]) -> None:
        before = _state(self.traj)
        fn(self.traj)
        after = _state(self.traj)
        self.beginResetModel()
        self.endResetModel()
        self.undo_stack.push(SnapshotCommand(self, text, before, after))
        self.changed.emit()

    def restore(self, state) -> None:
        waypoints, velocity, locked = state
        self.beginResetModel()
        self.traj.waypoints = [w.copy() for w in waypoints]
        self.traj.velocity_percent = velocity
        self.traj.locked_joints = list(locked)
        self.endResetModel()
        self.changed.emit()

    def set_trajectory(self, traj: Trajectory) -> None:
        self.beginResetModel()
        self.traj = traj
        self.endResetModel()
        self.undo_stack.clear()
        self.changed.emit()

    # --- row operations (undoable) ---
    def insert_after(self, row: int, waypoint: Waypoint | None = None) -> None:
        if waypoint is None:
            base = self.traj.waypoints[row] if 0 <= row < len(self.traj.waypoints) else None
            waypoint = Waypoint("TASK", list(base.joints) if base else [0.0] * 6)
        self.edit("Insert waypoint", lambda t: t.insert_waypoint(row + 1, waypoint))

    def delete_rows(self, rows: list[int]) -> None:
        if rows:
            self.edit("Delete waypoints", lambda t: t.delete_waypoints(rows))

    def duplicate_rows(self, rows: list[int]) -> None:
        if rows:
            self.edit("Duplicate waypoints", lambda t: t.duplicate_waypoints(rows))

    def move_row(self, row: int, delta: int) -> int:
        target = row + delta
        if not (0 <= row < len(self.traj.waypoints)) or not (0 <= target < len(self.traj.waypoints)):
            return row
        self.edit("Reorder waypoint", lambda t: t.move_waypoint(row, target))
        return target

    def set_velocity(self, value: float) -> None:
        if value != self.traj.velocity_percent:
            self.edit("Set velocity", lambda t: setattr(t, "velocity_percent", float(value)))

    def set_locked(self, joint: int, locked: bool) -> None:
        if self.traj.locked_joints[joint] != locked:
            self.edit(f"{'Lock' if locked else 'Unlock'} {JOINT_NAMES[joint]}",
                      lambda t: t.locked_joints.__setitem__(joint, locked))

    def set_joint_on_rows(self, joint: int, value: float, rows: list[int]) -> None:
        if self.traj.locked_joints[joint]:
            raise ValueError(f"{JOINT_NAMES[joint]} is locked")
        self.edit(f"Set {JOINT_NAMES[joint]} on {len(rows)} row(s)",
                  lambda t: [t.set_joint(r, joint, value) for r in rows])

    def rows_to_text(self, rows: list[int]) -> str:
        return "\n".join("\t".join(f"{v:.4f}" for v in self.traj.waypoints[r].joints) for r in sorted(rows))

    def paste_text(self, after_row: int, text: str) -> int:
        parsed = []
        for line_no, line in enumerate(text.strip().splitlines(), start=1):
            parts = [p for p in line.replace(",", "\t").split("\t") if p.strip()]
            try:
                values = [float(p) for p in parts]
            except ValueError:
                raise ValueError(f"line {line_no}: all values must be numbers") from None
            if len(values) != 6 or not all(math.isfinite(v) for v in values):
                raise ValueError(f"line {line_no}: expected six finite joint values, found {len(values)}")
            parsed.append(Waypoint("TASK", values))
        if not parsed:
            raise ValueError("clipboard is empty")

        def insert(t: Trajectory):
            for k, w in enumerate(parsed):
                t.waypoints.insert(after_row + 1 + k, w)
            t.normalize_phases()
        self.edit("Paste waypoints", insert)
        return len(parsed)


def _optional_float(text: str) -> float | None:
    text = text.strip()
    return float(text) if text else None


class TemplateDialog(QDialog):
    """Collects operator-supplied parameters. Nothing is pre-filled with angles or speeds."""

    TEMPLATES = ["Single-Joint Sweep", "Bidirectional Sweep", "Coordinated Multi-Joint",
                 "Stationary Hold (no motion)", "TEST_B_J4_BIDIRECTIONAL", "TEST_B_J6_BIDIRECTIONAL"]

    def __init__(self, pose_provider: Callable[[], list[float] | None] | None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Generate trajectory from template")
        self.pose_provider = pose_provider
        self.result_trajectory: Trajectory | None = None
        layout = QVBoxLayout(self)
        note = QLabel("Every angle and speed is entered by the operator. Leave required fields empty to create a "
                      "non-executable template. Generated points are linear joint-space interpolation (authoring), "
                      "not a validated motion profile.")
        note.setWordWrap(True)
        layout.addWidget(note)
        form = QFormLayout()
        self.template = QComboBox()
        self.template.addItems(self.TEMPLATES)
        self.template.currentIndexChanged.connect(self._template_changed)
        self.name = QLineEdit()
        self.joint = QComboBox()
        self.joint.addItems(JOINT_NAMES)
        self.start = [QLineEdit() for _ in range(6)]
        start_row = QHBoxLayout()
        for i, edit in enumerate(self.start):
            edit.setPlaceholderText(JOINT_NAMES[i])
            edit.setMaximumWidth(70)
            start_row.addWidget(edit)
        self.btn_from_robot = QPushButton("Copy current robot pose")
        self.btn_from_robot.setObjectName("secondary")
        self.btn_from_robot.setToolTip("Deliberate copy of the live joint readout into the start pose (no motion).")
        self.btn_from_robot.setEnabled(pose_provider is not None)
        self.btn_from_robot.clicked.connect(self._copy_robot_pose)
        start_row.addWidget(self.btn_from_robot)
        self.positive, self.negative = QLineEdit(), QLineEdit()
        self.positive.setPlaceholderText("absolute angle [deg] or empty")
        self.negative.setPlaceholderText("absolute angle [deg] or empty")
        self.coord_pos, self.coord_neg = QLineEdit(), QLineEdit()
        self.coord_pos.setPlaceholderText("e.g. 2:55, 3:95  (joint:absolute deg)")
        self.coord_neg.setPlaceholderText("optional, same format")
        self.speed = QLineEdit()
        self.speed.setPlaceholderText("controller-command %, required")
        self.segments = QSpinBox()
        self.segments.setRange(1, 200)
        self.segments.setValue(4)
        self.cycles = QSpinBox()
        self.cycles.setRange(1, 50)
        form.addRow("Template", self.template)
        form.addRow("Recipe name", self.name)
        form.addRow("Moving joint", self.joint)
        form.addRow("Start pose (hold pose for fixed joints)", start_row)
        form.addRow("Positive target (single-joint)", self.positive)
        form.addRow("Negative target (single-joint)", self.negative)
        form.addRow("Coordinated + targets", self.coord_pos)
        form.addRow("Coordinated - targets", self.coord_neg)
        form.addRow("Velocity (%)", self.speed)
        form.addRow("Waypoints per leg", self.segments)
        form.addRow("Repeat cycles", self.cycles)
        layout.addLayout(form)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self._template_changed()

    def _template_changed(self):
        text = self.template.currentText()
        if text.startswith("TEST_B_J"):
            self.joint.setCurrentIndex(int(text[8]) - 1)
            self.name.setText(text)
        elif not self.name.text() or self.name.text().startswith("TEST_B_"):
            self.name.setText("")
        self.joint.setEnabled(not text.startswith("TEST_B_J") and "Coordinated" not in text and "Stationary" not in text)

    def _copy_robot_pose(self):
        pose = self.pose_provider() if self.pose_provider else None
        if not pose or len(pose) != 6:
            QMessageBox.information(self, "Robot pose", "No live robot pose available (robot not connected).")
            return
        for edit, value in zip(self.start, pose):
            edit.setText(f"{value:.4f}")

    @staticmethod
    def _parse_targets(text: str) -> dict[int, float]:
        out: dict[int, float] = {}
        for part in text.replace(";", ",").split(","):
            if part.strip():
                j, v = part.split(":")
                joint = int(j.strip().upper().lstrip("J"))
                if not 1 <= joint <= 6:
                    raise ValueError("joint must be 1..6")
                out[joint - 1] = float(v)
        return out

    def _accept(self):
        try:
            self.result_trajectory = self._build()
        except gen.IncompleteTemplate as exc:
            QMessageBox.warning(self, "Missing operator values", str(exc))
            return
        except (ValueError, IndexError) as exc:
            QMessageBox.warning(self, "Invalid input", str(exc))
            return
        self.accept()

    def _build(self) -> Trajectory:
        kind = self.template.currentText()
        name = self.name.text().strip() or kind.split(" (")[0].replace(" ", "_").upper()
        joint = self.joint.currentIndex()
        start_values = [_optional_float(e.text()) for e in self.start]
        start = start_values if all(v is not None for v in start_values) else None
        speed = _optional_float(self.speed.text())
        segments, cycles = self.segments.value(), self.cycles.value()
        pos, neg = _optional_float(self.positive.text()), _optional_float(self.negative.text())
        if kind.startswith("Stationary"):
            return gen.stationary_hold(name, start)
        if kind.startswith("TEST_B_J"):
            template = gen.test_b_template(int(kind[8]))
            template.name = name
            if start is None or speed is None or (pos is None and neg is None):
                return template  # stays non-executable until the operator completes it
            return gen.complete_template(template, start_pose=start, positive_deg=pos, negative_deg=neg,
                                         segments_per_leg=segments, cycles=cycles, velocity=speed)
        if speed is None:
            raise gen.IncompleteTemplate("velocity (%) is required")
        if kind.startswith("Single"):
            if pos is None and neg is None:
                raise gen.IncompleteTemplate("a target angle is required")
            target = pos if pos is not None else neg
            return gen.single_joint_sweep(name, joint, start, target, segments, speed)
        if kind.startswith("Bidirectional"):
            return gen.bidirectional_sweep(name, joint, start, pos, neg, segments, speed, cycles)
        return gen.coordinated_motion(name, start, self._parse_targets(self.coord_pos.text()),
                                      self._parse_targets(self.coord_neg.text()) or None, segments, speed, cycles)


class TrajectoryStudioWindow(QMainWindow):
    """Left: library. Center: waypoint table + preview. Right: six-joint editor. Bottom: validation."""

    loadRequested = pyqtSignal(str)

    def __init__(self, parent=None, library_root: str | Path = "trajectory_library",
                 pathways_dir: str | Path = "Pathways",
                 robot_pose_provider: Callable[[], list[float] | None] | None = None,
                 software_revision: str = ""):
        super().__init__(parent)
        self.setWindowTitle("Trajectory Planning Studio  -  authoring & preview only (no robot motion)")
        self.resize(1500, 900)
        self.library = Library(library_root)
        self.pathways_dir = Path(pathways_dir)
        self.pose_provider = robot_pose_provider
        self.software_revision = software_revision
        self.campaign: Campaign | None = None
        self.model = WaypointTableModel(Trajectory(name="untitled"), self)
        self.model.changed.connect(self._refresh)
        self.setStyleSheet(LIGHT_STYLE)
        self._dark = False
        self._build_toolbar()
        self._build_center()
        self._build_library_dock()
        self._build_joint_dock()
        self._build_validation_dock()
        self.statusBar().showMessage("Preview and validation never move the robot. Execute from the Pathways tab only.")
        self._refresh()

    # ------------------------------------------------------------------ construction
    def _action(self, text, slot, shortcut=None, tip=""):
        act = QAction(text, self)
        act.triggered.connect(slot)
        if shortcut:
            act.setShortcut(shortcut)
        act.setToolTip(tip or text)
        return act

    def _build_toolbar(self):
        bar = QToolBar("Main")
        bar.setMovable(False)
        self.addToolBar(bar)
        self.act_undo = self.model.undo_stack.createUndoAction(self, "Undo")
        self.act_redo = self.model.undo_stack.createRedoAction(self, "Redo")
        self.act_undo.setShortcut(QKeySequence.StandardKey.Undo)
        self.act_redo.setShortcut(QKeySequence.StandardKey.Redo)
        for act in (
            self._action("Import CSV", self.import_csv_dialog, tip="Load an execution-format trajectory CSV"),
            self._action("Open recipe", self.open_recipe_dialog),
            self._action("Save revision", self.save_revision, QKeySequence.StandardKey.Save,
                         "Save the recipe as a new version (older versions are kept)"),
            None,
            self.act_undo, self.act_redo,
            self._action("Revert", self.revert_changes, tip="Discard unsaved edits"),
            None,
            self._action("Generate…", self.open_template_dialog, tip="Create a trajectory from a template"),
            self._action("Validate", self.show_validation, tip="Static validation, no movement"),
            self._action("Export CSV…", self.export_csv_dialog, tip="Write the execution-compatible CSV"),
            self._action("Send to Pathways", self.send_to_pathways,
                         tip="Export + load into the Pathways tab. Does not move the robot; use Run there."),
            None,
            self._action("Light/Dark", self.toggle_theme),
        ):
            if act is None:
                bar.addSeparator()
            else:
                bar.addAction(act)

    def _build_center(self):
        tabs = QTabWidget()
        editor = QSplitter(Qt.Orientation.Vertical)
        self.table = QTableView()
        self.table.setModel(self.model)
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.selectionModel().selectionChanged.connect(self._selection_changed)
        table_box = QWidget()
        tl = QVBoxLayout(table_box)
        tl.setContentsMargins(0, 0, 0, 0)
        row_tools = QHBoxLayout()
        self.row_buttons = {}
        for label, slot, tip in (
            ("Add", self._add_row, "Append a copy of the last waypoint"),
            ("Insert after", self._insert_row, "Insert a copy after the selected row"),
            ("Duplicate", self._duplicate_rows, "Duplicate selected rows"),
            ("Delete", self._delete_rows, "Delete selected rows"),
            ("Move up", lambda: self._move_row(-1), "Reorder"),
            ("Move down", lambda: self._move_row(1), "Reorder"),
            ("Copy", self._copy_rows, "Copy selected joint values"),
            ("Paste", self._paste_rows, "Paste six-value rows after the selection"),
        ):
            btn = QPushButton(label)
            btn.setObjectName("secondary")
            btn.setToolTip(tip)
            btn.clicked.connect(slot)
            row_tools.addWidget(btn)
        row_tools.addStretch(1)
        row_tools.addWidget(QLabel("Velocity (%, controller command):"))
        self.spin_velocity = QDoubleSpinBox()
        self.spin_velocity.setRange(0.0, 100.0)
        self.spin_velocity.setSpecialValueText("not set")
        self.spin_velocity.setDecimals(1)
        self.spin_velocity.setToolTip("One speed for the whole file; the executor has no per-waypoint speed.")
        self.spin_velocity.editingFinished.connect(self._velocity_edited)
        row_tools.addWidget(self.spin_velocity)
        tl.addLayout(row_tools)
        tl.addWidget(self.table)
        editor.addWidget(table_box)
        plot_box = QWidget()
        pl = QVBoxLayout(plot_box)
        pl.setContentsMargins(0, 0, 0, 0)
        self.figure = Figure(figsize=(6, 3), tight_layout=False)
        self.canvas = FigureCanvasQTAgg(self.figure)
        pl.addWidget(NavigationToolbar2QT(self.canvas, self))
        pl.addWidget(self.canvas)
        editor.addWidget(plot_box)
        editor.setSizes([360, 420])
        tabs.addTab(editor, "Trajectory editor")
        tabs.addTab(self._build_campaign_tab(), "Experiment planner")
        self.setCentralWidget(tabs)

    def _build_campaign_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        top = QHBoxLayout()
        self.campaign_experiment = QLineEdit("ID_NORMAL_001")
        self.campaign_repeats = QSpinBox()
        self.campaign_repeats.setRange(1, 10)
        self.campaign_repeats.setValue(2)
        btn_new = QPushButton("Create 15-recipe starting plan")
        btn_new.clicked.connect(self.create_campaign)
        btn_save = QPushButton("Save campaign…")
        btn_save.setObjectName("secondary")
        btn_save.clicked.connect(self.save_campaign)
        for w in (QLabel("Experiment ID"), self.campaign_experiment, QLabel("Planned repeats"),
                  self.campaign_repeats, btn_new, btn_save):
            top.addWidget(w)
        top.addStretch(1)
        layout.addLayout(top)
        info = QLabel("Research-design templates only: no joint targets or speeds are pre-filled. Open a recipe "
                      "from the table, complete it in the editor, and approve it before use.")
        info.setWordWrap(True)
        layout.addWidget(info)
        split = QSplitter(Qt.Orientation.Vertical)
        self.campaign_table = QTableWidget(0, 7)
        self.campaign_table.setHorizontalHeaderLabels(
            ["ID", "Recipe", "Joints", "Speed label", "Runs", "Run IDs", "Status"])
        self.campaign_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.campaign_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.campaign_table.cellDoubleClicked.connect(self._open_campaign_recipe)
        split.addWidget(self.campaign_table)
        cov = QWidget()
        cl = QHBoxLayout(cov)
        self.cov_figure = Figure(figsize=(5, 3))
        self.cov_canvas = FigureCanvasQTAgg(self.cov_figure)
        cl.addWidget(self.cov_canvas, 3)
        self.cov_text = QPlainTextEdit()
        self.cov_text.setReadOnly(True)
        cl.addWidget(self.cov_text, 2)
        split.addWidget(cov)
        layout.addWidget(split)
        return page

    def _build_library_dock(self):
        dock = QDockWidget("Trajectory library", self)
        box = QWidget()
        v = QVBoxLayout(box)
        self.lib_search = QLineEdit()
        self.lib_search.setPlaceholderText("Search name / description / type…")
        self.lib_search.textChanged.connect(self.refresh_library)
        v.addWidget(self.lib_search)
        self.lib_tree = QTreeWidget()
        self.lib_tree.setHeaderLabels(["Name", "Ver", "Type", "Status"])
        self.lib_tree.itemDoubleClicked.connect(lambda *_: self.open_selected_library_entry())
        v.addWidget(self.lib_tree)
        grid = QGridLayout()
        for i, (label, slot) in enumerate((
            ("Open", self.open_selected_library_entry), ("Save to library", self.save_to_library),
            ("Duplicate", self.duplicate_library_entry), ("Rename", self.rename_library_entry),
            ("Archive", self.archive_library_entry), ("Import CSV → library", self.import_csv_to_library),
        )):
            btn = QPushButton(label)
            btn.setObjectName("secondary")
            btn.clicked.connect(slot)
            grid.addWidget(btn, i // 2, i % 2)
        v.addLayout(grid)
        dock.setWidget(box)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, dock)
        self.refresh_library()

    def _build_joint_dock(self):
        dock = QDockWidget("Six-joint editor", self)
        box = QWidget()
        v = QVBoxLayout(box)
        grid = QGridLayout()
        for c, h in enumerate(["Show", "Joint", "Angle [deg]", "", "Robot now", "", "Lock"]):
            lbl = QLabel(f"<b>{h}</b>")
            grid.addWidget(lbl, 0, c)
        self.j_visible, self.j_spin, self.j_slider, self.j_live, self.j_lock, self.j_badge = [], [], [], [], [], []
        for j in range(6):
            r = j + 1
            vis = QCheckBox()
            vis.setChecked(True)
            vis.toggled.connect(self._draw_preview)
            badge = QLabel(JOINT_NAMES[j])
            spin = QDoubleSpinBox()
            spin.setRange(-3600.0, 3600.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.5)
            spin.setToolTip("Numeric entry range only; it is not a joint limit.")
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setEnabled(False)
            slider.setToolTip("Enabled only when an operator-declared limit exists for this joint.")
            slider.valueChanged.connect(lambda val, s=spin: s.setValue(val / 100.0))
            live = QLabel("--")
            live.setMinimumWidth(70)
            copy_btn = QPushButton("Copy → editor")
            copy_btn.setObjectName("secondary")
            copy_btn.setToolTip("Deliberately copy this joint's live angle into the editor (no motion).")
            copy_btn.clicked.connect(lambda _=False, jj=j: self._copy_live_joint(jj))
            lock = QCheckBox("Lock")
            lock.toggled.connect(lambda checked, jj=j: self.model.set_locked(jj, checked))
            set_btn = QPushButton("Set on rows")
            set_btn.setObjectName("secondary")
            set_btn.setToolTip("Write this angle into this joint's column for the selected rows.")
            set_btn.clicked.connect(lambda _=False, jj=j: self._set_joint_on_selection(jj))
            for c, w in enumerate((vis, badge, spin, slider, live, copy_btn, lock)):
                grid.addWidget(w, r, c)
            grid.addWidget(set_btn, r, 7)
            self.j_visible.append(vis)
            self.j_spin.append(spin)
            self.j_slider.append(slider)
            self.j_live.append(live)
            self.j_lock.append(lock)
            self.j_badge.append(badge)
        v.addLayout(grid)
        load_btn = QPushButton("Load selected row into editor")
        load_btn.setObjectName("secondary")
        load_btn.clicked.connect(self._load_selected_row_into_editor)
        v.addWidget(load_btn)
        limits = QGroupBox("Operator-declared joint limits (optional)")
        lg = QGridLayout(limits)
        self.limit_min, self.limit_max = [], []
        for j in range(6):
            lo, hi = QLineEdit(), QLineEdit()
            lo.setPlaceholderText("min")
            hi.setPlaceholderText("max")
            lo.editingFinished.connect(self._limits_edited)
            hi.editingFinished.connect(self._limits_edited)
            lg.addWidget(QLabel(JOINT_NAMES[j]), j, 0)
            lg.addWidget(lo, j, 1)
            lg.addWidget(hi, j, 2)
            self.limit_min.append(lo)
            self.limit_max.append(hi)
        self.limit_source = QLineEdit()
        self.limit_source.setPlaceholderText("source of limits (e.g. datasheet, approved by …)")
        self.limit_source.editingFinished.connect(self._limits_edited)
        lg.addWidget(self.limit_source, 6, 0, 1, 3)
        warn = QLabel("The repository holds no authoritative limits. Declared limits are checked but never "
                      "presented as verified.")
        warn.setWordWrap(True)
        lg.addWidget(warn, 7, 0, 1, 3)
        v.addWidget(limits)
        v.addStretch(1)
        dock.setWidget(box)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)
        self.live_timer = None
        if self.pose_provider is not None:
            from PyQt6.QtCore import QTimer
            self.live_timer = QTimer(self)
            self.live_timer.timeout.connect(self._update_live_pose)
            self.live_timer.start(500)

    def _build_validation_dock(self):
        dock = QDockWidget("Validation  (static checks, no movement)", self)
        self.validation_text = QPlainTextEdit()
        self.validation_text.setReadOnly(True)
        self.validation_text.setFont(self.font())
        dock.setWidget(self.validation_text)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)

    # ------------------------------------------------------------------ refresh
    def _refresh(self):
        traj = self.model.traj
        self.spin_velocity.blockSignals(True)
        self.spin_velocity.setValue(traj.velocity_percent or 0.0)
        self.spin_velocity.blockSignals(False)
        moving = set(traj.moving_joints())
        for j in range(6):
            self.j_lock[j].blockSignals(True)
            self.j_lock[j].setChecked(traj.locked_joints[j])
            self.j_lock[j].blockSignals(False)
            self.j_badge[j].setText(f"{JOINT_NAMES[j]}  {'● moves' if j in moving else '○ fixed'}")
            self.j_badge[j].setStyleSheet("font-weight:600;color:#1f9d55;" if j in moving else "color:#7a8794;")
        self._sync_limits_to_widgets()
        self._draw_preview()
        self.show_validation()
        title = f"Trajectory Planning Studio  -  {traj.name} v{traj.version}"
        if not self.model.undo_stack.isClean():
            title += "  *unsaved*"
        self.setWindowTitle(title)

    def _selected_rows(self) -> list[int]:
        return sorted({i.row() for i in self.table.selectionModel().selectedRows()})

    def _selection_changed(self, *_):
        self._draw_preview()

    def _draw_preview(self, *_):
        draw_positions(self.figure, self.model.traj, [c.isChecked() for c in self.j_visible], self._selected_rows())
        self.canvas.draw_idle()

    def show_validation(self):
        report = validate(self.model.traj)
        self.last_report = report
        self.validation_text.setPlainText(report.to_text())
        return report

    def _sync_limits_to_widgets(self):
        traj = self.model.traj
        for j in range(6):
            lim = traj.joint_limits_deg[j]
            for edit, value in ((self.limit_min[j], lim[0] if lim else None), (self.limit_max[j], lim[1] if lim else None)):
                edit.blockSignals(True)
                edit.setText("" if value is None else f"{value:g}")
                edit.blockSignals(False)
            slider = self.j_slider[j]
            slider.setEnabled(bool(lim))
            if lim:
                slider.blockSignals(True)
                slider.setRange(int(lim[0] * 100), int(lim[1] * 100))
                slider.blockSignals(False)
        self.limit_source.blockSignals(True)
        self.limit_source.setText(traj.limits_source)
        self.limit_source.blockSignals(False)

    # ------------------------------------------------------------------ edit slots
    def _error(self, title: str, message: str):
        QMessageBox.warning(self, title, message)

    def _add_row(self):
        self.model.insert_after(len(self.model.traj.waypoints) - 1)

    def _insert_row(self):
        rows = self._selected_rows()
        self.model.insert_after(rows[-1] if rows else len(self.model.traj.waypoints) - 1)

    def _duplicate_rows(self):
        self.model.duplicate_rows(self._selected_rows())

    def _delete_rows(self):
        self.model.delete_rows(self._selected_rows())

    def _move_row(self, delta: int):
        rows = self._selected_rows()
        if len(rows) == 1:
            new = self.model.move_row(rows[0], delta)
            self.table.selectRow(new)

    def _copy_rows(self):
        rows = self._selected_rows()
        if rows:
            QGuiApplication.clipboard().setText(self.model.rows_to_text(rows))

    def _paste_rows(self):
        rows = self._selected_rows()
        try:
            self.model.paste_text(rows[-1] if rows else len(self.model.traj.waypoints) - 1,
                                  QGuiApplication.clipboard().text())
        except ValueError as exc:
            self._error("Paste failed", str(exc))

    def _velocity_edited(self):
        value = self.spin_velocity.value()
        if value > 0:
            self.model.set_velocity(value)

    def _set_joint_on_selection(self, joint: int):
        rows = self._selected_rows()
        if not rows:
            self._error("No rows selected", "Select one or more waypoint rows first.")
            return
        try:
            self.model.set_joint_on_rows(joint, self.j_spin[joint].value(), rows)
        except ValueError as exc:
            self._error("Cannot edit", str(exc))

    def _load_selected_row_into_editor(self):
        rows = self._selected_rows()
        if rows:
            for j in range(6):
                self.j_spin[j].setValue(self.model.traj.waypoints[rows[0]].joints[j])

    def _limits_edited(self):
        traj = self.model.traj
        try:
            limits = []
            for j in range(6):
                lo, hi = self.limit_min[j].text().strip(), self.limit_max[j].text().strip()
                if not lo and not hi:
                    limits.append(None)
                    continue
                lo_v, hi_v = float(lo), float(hi)
                if not (math.isfinite(lo_v) and math.isfinite(hi_v)) or lo_v >= hi_v:
                    raise ValueError(f"{JOINT_NAMES[j]}: need finite min < max")
                limits.append([lo_v, hi_v])
        except ValueError as exc:
            self._error("Invalid limit", str(exc) if "need" in str(exc) else "Limits must be numbers (both min and max).")
            self._sync_limits_to_widgets()
            return
        traj.joint_limits_deg = limits
        traj.limits_source = self.limit_source.text().strip()
        self._sync_limits_to_widgets()
        self.show_validation()

    def _update_live_pose(self):
        pose = self.pose_provider() if self.pose_provider else None
        for j in range(6):
            self.j_live[j].setText("--" if not pose or len(pose) != 6 else f"{pose[j]:.2f}°")

    def _copy_live_joint(self, joint: int):
        pose = self.pose_provider() if self.pose_provider else None
        if not pose or len(pose) != 6:
            self._error("Robot pose", "No live robot pose available (robot not connected).")
            return
        self.j_spin[joint].setValue(float(pose[joint]))

    # ------------------------------------------------------------------ files
    def _confirm_discard(self) -> bool:
        if self.model.undo_stack.isClean():
            return True
        return QMessageBox.question(self, "Unsaved changes", "Discard unsaved changes?") == QMessageBox.StandardButton.Yes

    def load_trajectory(self, traj: Trajectory, clean: bool = True):
        self.model.set_trajectory(traj)
        if clean:
            self.model.undo_stack.setClean()
        self._refresh()

    def import_csv_dialog(self, *_):
        path, _ = QFileDialog.getOpenFileName(self, "Import trajectory CSV", str(self.pathways_dir), "CSV (*.csv)")
        if path:
            self.import_csv_path(path)

    def import_csv_path(self, path: str | Path) -> bool:
        if not self._confirm_discard():
            return False
        try:
            traj, warnings = import_csv(path)
        except CsvImportError as exc:
            self._error("Import failed", str(exc))
            return False
        self.load_trajectory(traj)
        if warnings:
            QMessageBox.information(self, "Import notes", "\n".join(warnings))
        return True

    def open_recipe_dialog(self, *_):
        path, _ = QFileDialog.getOpenFileName(self, "Open recipe", str(self.library.root), "Recipe (*.recipe.json)")
        if path and self._confirm_discard():
            try:
                self.load_trajectory(load_recipe(path))
            except (ValueError, KeyError, json.JSONDecodeError) as exc:
                self._error("Open failed", str(exc))

    def save_revision(self, *_):
        try:
            path = self.library.save(self.model.traj, self.software_revision)
        except ValueError as exc:
            self._error("Save failed", str(exc))
            return None
        self.model.undo_stack.setClean()
        self.refresh_library()
        self._refresh()
        self.statusBar().showMessage(f"Saved {path}")
        return path

    def export_csv_dialog(self, *_):
        report = self.show_validation()
        if not report.exportable:
            self._error("Export blocked", report.to_text())
            return None
        path, _ = QFileDialog.getSaveFileName(self, "Export execution CSV",
                                              str(self.pathways_dir / f"{self.model.traj.name}.csv"), "CSV (*.csv)")
        if not path:
            return None
        return self.export_to(path)

    def export_to(self, path: str | Path):
        """Validate then write the execution-format CSV. Returns None when blocked."""
        report = self.show_validation()
        if not report.exportable:
            return None
        out = export_csv(self.model.traj, path)
        self.model.traj.source_csv_sha256 = sha256_file(out)
        self.statusBar().showMessage(f"Exported {out} (format valid; physical safety NOT VERIFIED)")
        return out

    def send_to_pathways(self, *_):
        report = self.show_validation()
        if not report.exportable:
            self._error("Blocked", report.to_text())
            return
        target = self.pathways_dir / f"{self.model.traj.name}.csv"
        if target.exists() and QMessageBox.question(
                self, "Overwrite?", f"{target.name} already exists in Pathways. Overwrite it?"
        ) != QMessageBox.StandardButton.Yes:
            return
        out = self.export_to(target)
        if out is not None:
            self.loadRequested.emit(str(out))
            QMessageBox.information(
                self, "Loaded into Pathways",
                f"{out.name} was exported and loaded. The robot has NOT moved.\n"
                "To execute, use the Pathways tab Run button (connection check, identification metadata and "
                "operator confirmation still apply).")

    def revert_changes(self, *_):
        self.model.undo_stack.setIndex(0)

    def toggle_theme(self, *_):
        self._dark = not self._dark
        self.setStyleSheet(DARK_STYLE if self._dark else LIGHT_STYLE)

    def open_template_dialog(self, *_):
        dialog = TemplateDialog(self.pose_provider, self)
        if dialog.exec() == QDialog.DialogCode.Accepted and dialog.result_trajectory is not None:
            if self._confirm_discard():
                self.load_trajectory(dialog.result_trajectory, clean=False)
                self.model.undo_stack.resetClean()

    # ------------------------------------------------------------------ library
    def refresh_library(self, *_):
        self.lib_tree.clear()
        entries = self.library.entries(self.lib_search.text())
        groups: dict[str, QTreeWidgetItem] = {}
        for cat in CATEGORIES:
            item = QTreeWidgetItem([cat])
            self.lib_tree.addTopLevelItem(item)
            groups[cat] = item
        for e in entries:
            child = QTreeWidgetItem([e.name, f"v{e.version}", e.kind, e.validation_status])
            child.setToolTip(0, e.description)
            child.setData(0, Qt.ItemDataRole.UserRole, e)
            groups.get(e.category, groups[CATEGORIES[-1]]).addChild(child)
        self.lib_tree.expandAll()

    def _selected_entry(self):
        item = self.lib_tree.currentItem()
        return item.data(0, Qt.ItemDataRole.UserRole) if item else None

    def open_selected_library_entry(self):
        entry = self._selected_entry()
        if entry and self._confirm_discard():
            self.load_trajectory(self.library.load(entry))

    def save_to_library(self):
        self.save_revision()

    def duplicate_library_entry(self):
        entry = self._selected_entry()
        if entry:
            name, ok = QInputDialog.getText(self, "Duplicate", "New recipe name:", text=f"{entry.name}_copy")
            if ok and name.strip():
                self.library.duplicate(entry, name)
                self.refresh_library()

    def rename_library_entry(self):
        entry = self._selected_entry()
        if entry:
            name, ok = QInputDialog.getText(self, "Rename", "New recipe name:", text=entry.name)
            if ok and name.strip():
                self.library.rename(entry, name)
                self.refresh_library()

    def archive_library_entry(self):
        entry = self._selected_entry()
        if entry and QMessageBox.question(self, "Archive", f"Move {entry.name} (all versions) to _archive?") \
                == QMessageBox.StandardButton.Yes:
            self.library.archive(entry)
            self.refresh_library()

    def import_csv_to_library(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import CSV", str(self.pathways_dir), "CSV (*.csv)")
        if path and self.import_csv_path(path):
            self.model.traj.category = "General Robot Pathways"
            self.save_revision()

    # ------------------------------------------------------------------ campaign
    def create_campaign(self):
        self.campaign = Campaign.default_plan(self.campaign_experiment.text().strip() or "ID_NORMAL_001",
                                              self.campaign_repeats.value())
        self._refresh_campaign()

    def _refresh_campaign(self):
        if self.campaign is None:
            return
        self.campaign_table.setRowCount(len(self.campaign.recipes))
        for r, rec in enumerate(self.campaign.recipes):
            exp = rec.experiment
            status = "needs operator values" if rec.generation.get("requires_operator_completion") else \
                ("approved" if rec.approval.get("approved") else "unapproved")
            planned = exp.get("planned_moving_joints", [])
            values = [exp.get("trajectory_id"), rec.name, ",".join(f"J{j}" for j in planned),
                      exp.get("speed_label") or "operator", exp.get("planned_runs"),
                      ", ".join(exp.get("run_ids", [])), status]
            for c, val in enumerate(values):
                self.campaign_table.setItem(r, c, QTableWidgetItem(str(val)))
        data = coverage(self.campaign)
        draw_coverage(self.cov_figure, data)
        self.cov_canvas.draw_idle()
        lines = [data["basis"], data["caveat"], ""]
        for row in data["joints"]:
            lines.append(f"{row['joint']}: runs={row['planned_independent_runs']}, speeds={row['speed_settings_percent']}, "
                         f"combos={row['joint_combinations']}")
            lines += [f"   missing: {m}" for m in row["missing"]]
        self.cov_text.setPlainText("\n".join(lines))

    def _open_campaign_recipe(self, row, _col):
        if self.campaign and self._confirm_discard():
            self.load_trajectory(self.campaign.recipes[row].copy())

    def save_campaign(self):
        if self.campaign is None:
            self._error("No campaign", "Create the starting plan first.")
            return
        directory = QFileDialog.getExistingDirectory(self, "Save campaign to folder", str(self.library.root))
        if directory:
            path = self.campaign.save(directory)
            self.statusBar().showMessage(f"Saved {path}")
