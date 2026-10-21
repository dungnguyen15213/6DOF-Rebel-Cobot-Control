"""Left-hand panel of the Trajectory Studio: build an ordered list of motion steps."""

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QAbstractItemView, QAbstractSpinBox, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QHeaderView,
    QLabel, QMessageBox, QPushButton, QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from core.sequence_planner import MOTION_LABELS, MOTION_LOCKED, SequenceStep

DEFAULT_START = (200.0, 0.0, 300.0)
DEFAULT_END = (200.0, 150.0, 300.0)
COORD_LIMIT = 800.0


def _xyz_text(point) -> str:
    return f"({point[0]:.0f}, {point[1]:.0f}, {point[2]:.0f})"


class SequenceBuilderWidget(QWidget):
    generate_requested = pyqtSignal()
    export_requested = pyqtSignal()
    points_changed = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._steps: list[SequenceStep] = []
        self._status: list = []
        # Optional callable(step) -> None when the step can move, else an error text.
        self.validator = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["#", "Motion", "Start", "End", "Pts", "Lock", "OK"])
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(24)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        for column in (0, 2, 3, 4, 5, 6):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.ResizeToContents)
        self.table.itemSelectionChanged.connect(self._load_selected_step)
        layout.addWidget(self.table, 1)

        layout.addWidget(self._build_editor())

        list_buttons = QHBoxLayout()
        list_buttons.setSpacing(4)
        for text, handler, tip in (
            ("▲", lambda: self._move_step(-1), "Move the selected step up"),
            ("▼", lambda: self._move_step(1), "Move the selected step down"),
            ("Remove", self._remove_step, "Remove the selected step"),
            ("Clear", self._clear_steps, "Remove all steps"),
        ):
            button = QPushButton(text)
            button.setToolTip(tip)
            button.clicked.connect(handler)
            list_buttons.addWidget(button)
        layout.addLayout(list_buttons)

        self.btn_generate = QPushButton("Generate Sequence")
        self.btn_generate.setStyleSheet(
            "background-color: #27ae60; color: white; padding: 6px; font-weight: bold;"
        )
        self.btn_generate.clicked.connect(self.generate_requested.emit)
        layout.addWidget(self.btn_generate)

        self.btn_export = QPushButton("Export Pathway CSV")
        self.btn_export.setEnabled(False)
        self.btn_export.clicked.connect(self.export_requested.emit)
        layout.addWidget(self.btn_export)

        self.lbl_summary = QLabel("Add steps, then press Generate Sequence.")
        self.lbl_summary.setWordWrap(True)
        self.lbl_summary.setStyleSheet("color: #aaa; font-size: 11px;")
        layout.addWidget(self.lbl_summary)

        for button in self.findChildren(QPushButton):
            button.setMinimumHeight(32)
        for box in self.findChildren((QDoubleSpinBox, QSpinBox, QComboBox)):
            box.setMinimumHeight(30)
        self.table.setMinimumHeight(140)

        self._update_lock_enabled()
        for box in (*self.start_inputs, *self.end_inputs):
            box.valueChanged.connect(lambda _=0: self.points_changed.emit())

    def _build_editor(self) -> QWidget:
        editor = QWidget()
        grid = QGridLayout(editor)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setVerticalSpacing(8)
        grid.setHorizontalSpacing(6)

        self.combo_motion = QComboBox()
        for key, label in MOTION_LABELS.items():
            self.combo_motion.addItem(label, key)
        self.combo_motion.currentIndexChanged.connect(self._update_lock_enabled)
        grid.addWidget(QLabel("Motion:"), 0, 0)
        grid.addWidget(self.combo_motion, 0, 1, 1, 6)

        self.start_inputs = self._coord_row(grid, 1, "Start (mm):", DEFAULT_START)
        self.end_inputs = self._coord_row(grid, 2, "End (mm):", DEFAULT_END)

        self.spin_waypoints = QSpinBox()
        self.spin_waypoints.setRange(2, 500)
        self.spin_waypoints.setValue(30)
        grid.addWidget(QLabel("Waypoints:"), 3, 0)
        grid.addLayout(self._with_step_buttons(self.spin_waypoints), 3, 1, 1, 3)

        self.lock_checks = []
        lock_row = QHBoxLayout()
        lock_row.setSpacing(2)
        for index in range(6):
            check = QCheckBox(f"J{index + 1}")
            self.lock_checks.append(check)
            lock_row.addWidget(check)
        grid.addWidget(QLabel("Lock:"), 4, 0)
        grid.addLayout(lock_row, 4, 1, 1, 6)

        buttons = QHBoxLayout()
        self.btn_add = QPushButton("Add Step")
        self.btn_add.clicked.connect(self._add_step)
        self.btn_update = QPushButton("Update Selected")
        self.btn_update.clicked.connect(self._update_step)
        buttons.addWidget(self.btn_add)
        buttons.addWidget(self.btn_update)
        grid.addLayout(buttons, 5, 0, 1, 7)
        return editor

    @staticmethod
    def _coord_row(grid, row, title, defaults):
        grid.addWidget(QLabel(title), row, 0)
        boxes = []
        for column, (axis, value) in enumerate(zip("XYZ", defaults)):
            box = QDoubleSpinBox()
            box.setRange(-COORD_LIMIT, COORD_LIMIT)
            box.setSingleStep(10.0)
            box.setDecimals(1)
            box.setPrefix(f"{axis} ")
            box.setValue(value)
            grid.addLayout(SequenceBuilderWidget._with_step_buttons(box), row, 1 + column * 2, 1, 2)
            boxes.append(box)
        return boxes

    @staticmethod
    def _with_step_buttons(box):
        """Wrap a spin box with separate -/+ buttons (the built-in arrows are too small to click)."""
        box.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        row = QHBoxLayout()
        row.setSpacing(2)
        row.addWidget(box, 1)
        for text, direction in (("−", -1), ("+", 1)):
            button = QPushButton(text)
            button.setFixedWidth(28)
            button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            button.clicked.connect(lambda _=False, d=direction: box.stepBy(d))
            row.addWidget(button)
        return row

    # ---- Public API ----

    def steps(self) -> list[SequenceStep]:
        return list(self._steps)

    def endpoints(self):
        """Live Start/End from the editor form, which the viewer's strong markers follow."""
        return (tuple(b.value() for b in self.start_inputs),
                tuple(b.value() for b in self.end_inputs))

    def chain(self) -> list:
        """Ordered sequence points: P1 = start of step 1, then the end of every step."""
        if not self._steps:
            return []
        return [self._steps[0].start] + [step.end for step in self._steps]

    def set_summary(self, text: str) -> None:
        self.lbl_summary.setText(text)

    def set_step_status(self, results) -> None:
        """Show the per-step validation result (None = OK, text = reason) in the OK column."""
        self._status = list(results)
        self._refresh_table(self.table.currentRow())

    # ---- Editing ----

    def _read_form(self) -> SequenceStep:
        motion = self.combo_motion.currentData()
        locked = tuple(i for i, check in enumerate(self.lock_checks) if check.isChecked())
        return SequenceStep(
            motion=motion,
            start=tuple(box.value() for box in self.start_inputs),
            end=tuple(box.value() for box in self.end_inputs),
            waypoints=self.spin_waypoints.value(),
            locked=locked if motion == MOTION_LOCKED else (),
        )

    def _write_form(self, step: SequenceStep) -> None:
        self.combo_motion.setCurrentIndex(self.combo_motion.findData(step.motion))
        for box, value in zip(self.start_inputs, step.start):
            box.setValue(value)
        for box, value in zip(self.end_inputs, step.end):
            box.setValue(value)
        self.spin_waypoints.setValue(step.waypoints)
        for index, check in enumerate(self.lock_checks):
            check.setChecked(index in step.locked)

    def _update_lock_enabled(self) -> None:
        enabled = self.combo_motion.currentData() == MOTION_LOCKED
        for check in self.lock_checks:
            check.setEnabled(enabled)

    def _check_step(self, step: SequenceStep) -> bool:
        if step.motion == MOTION_LOCKED and not step.locked:
            QMessageBox.warning(self, "Locked Joints", "Select at least one joint to lock.")
            return False
        if self.validator is not None:
            problem = self.validator(step)
            if problem:
                QMessageBox.warning(
                    self, "Step Cannot Move",
                    f"This step is not possible and was not saved.\n\n{problem}",
                )
                return False
        return True

    def _add_step(self) -> None:
        step = self._read_form()
        if not self._check_step(step):
            return
        self._steps.append(step)
        self._status = []
        self._refresh_table(len(self._steps) - 1)
        # The next step normally continues from where this one ends.
        for box, value in zip(self.start_inputs, step.end):
            box.setValue(value)

    def _update_step(self) -> None:
        row = self.table.currentRow()
        if row < 0:
            QMessageBox.information(self, "No Step Selected", "Select a step in the list first.")
            return
        step = self._read_form()
        if self._check_step(step):
            self._steps[row] = step
            self._status = []
            self._refresh_table(row)

    def _remove_step(self) -> None:
        row = self.table.currentRow()
        if row >= 0:
            del self._steps[row]
            self._status = []
            self._refresh_table(min(row, len(self._steps) - 1))

    def _clear_steps(self) -> None:
        self._steps.clear()
        self._status = []
        self._refresh_table(-1)

    def _move_step(self, delta: int) -> None:
        row = self.table.currentRow()
        other = row + delta
        if row < 0 or not 0 <= other < len(self._steps):
            return
        self._steps[row], self._steps[other] = self._steps[other], self._steps[row]
        self._status = []
        self._refresh_table(other)

    def _load_selected_step(self) -> None:
        row = self.table.currentRow()
        if 0 <= row < len(self._steps):
            self._write_form(self._steps[row])

    def _refresh_table(self, select_row: int) -> None:
        self.table.blockSignals(True)
        self.table.setRowCount(len(self._steps))
        for row, step in enumerate(self._steps):
            lock_text = ",".join(f"J{i + 1}" for i in step.locked) if step.locked else "-"
            cells = (str(row + 1), MOTION_LABELS[step.motion], _xyz_text(step.start),
                     _xyz_text(step.end), str(step.waypoints), lock_text)
            for column, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if column != 1:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setItem(row, column, item)
            status_item = QTableWidgetItem("")
            status_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            if row < len(self._status):
                problem = self._status[row]
                status_item.setText("✔" if problem is None else "✖")
                status_item.setForeground(QColor("#2ecc71" if problem is None else "#e74c3c"))
                status_item.setToolTip("Step can move" if problem is None else problem)
            self.table.setItem(row, 6, status_item)
        if 0 <= select_row < len(self._steps):
            self.table.selectRow(select_row)
        self.table.blockSignals(False)
        self.points_changed.emit()
