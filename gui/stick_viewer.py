import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QCheckBox
from PyQt6.QtCore import Qt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import proj3d

class StickViewer(QWidget):
    def __init__(self):
        super().__init__()
        
        # --- Full Viewport Layout Configuration ---
        # Strip all margins to allow the canvas to fill the entire tab perfectly
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # --- Fullscreen Matplotlib Canvas ---
        self.fig = Figure(facecolor='#ffffff') 
        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas)
        
        # Force the 3D plot to stretch to the absolute edges of the figure (removes blank space)
        self.fig.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
        
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.latest_points = None
        self.snap_text = self.ax.text2D(0.02, 0.88, '', transform=self.ax.transAxes,
                                       fontsize=10, color='black', va='top',
                                       bbox=dict(facecolor='white', alpha=0.9, edgecolor='none'))
        self.snap_text.set_visible(False)
        self.snap_marker, = self.ax.plot([], [], [], 'o', markersize=10, color='#f39c12', alpha=0.9)
        self.snap_marker.set_visible(False)
        self.ax.set_proj_type('ortho') # Orthographic CAD projection
        
        # Set default perspective
        self.ax.view_init(elev=30, azim=-45)
        
        # --- FLOATING CAD View Control Toolbar ---
        # By setting 'self.canvas' as the parent, this widget floats ON TOP of the 3D view
        self.toolbar_widget = QWidget(self.canvas)
        self.toolbar_widget.setStyleSheet("""
            QWidget {
                background-color: rgba(245, 245, 245, 200); /* Semi-transparent background */
                border: 1px solid #cccccc;
                border-radius: 6px;
            }
        """)
        
        toolbar_layout = QVBoxLayout(self.toolbar_widget)
        toolbar_layout.setContentsMargins(6, 6, 6, 6)
        toolbar_layout.setSpacing(6)
        
        self.view_angles = {
            "Isometric": (30, -45),
            "Front": (0, -90),
            "Top": (90, -90),
            "Bottom": (-90, -90),
            "Right": (0, 0)
        }
        
        view_row = QHBoxLayout()
        for view_name in self.view_angles.keys():
            btn = QPushButton(view_name)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #ffffff;
                    border: 1px solid #b3b3b3;
                    border-radius: 4px;
                    padding: 4px 10px;
                    font-weight: bold;
                    color: #333;
                    font-size: 11px;
                }
                QPushButton:hover {
                    background-color: #e6e6e6;
                    border-color: #888;
                }
                QPushButton:pressed {
                    background-color: #cccccc;
                }
            """)
            btn.clicked.connect(lambda checked, name=view_name: self.change_view_angle(name))
            view_row.addWidget(btn)
        toolbar_layout.addLayout(view_row)

        self.show_robot = True
        self.show_points = True
        self.show_trail = True
        self.show_grid = True

        toggle_row = QHBoxLayout()
        self.chk_robot = QCheckBox("Robot")
        self.chk_robot.setChecked(True)
        self.chk_robot.toggled.connect(self.toggle_robot)
        toggle_row.addWidget(self.chk_robot)

        self.chk_points = QCheckBox("Points")
        self.chk_points.setChecked(True)
        self.chk_points.toggled.connect(self.toggle_points)
        toggle_row.addWidget(self.chk_points)

        self.chk_trail = QCheckBox("Trail")
        self.chk_trail.setChecked(True)
        self.chk_trail.toggled.connect(self.toggle_trail)
        toggle_row.addWidget(self.chk_trail)

        self.chk_grid = QCheckBox("Grid")
        self.chk_grid.setChecked(True)
        self.chk_grid.toggled.connect(self.toggle_grid)
        toggle_row.addWidget(self.chk_grid)

        toolbar_layout.addLayout(toggle_row)

        # Move the floating toolbar to the top-left corner (X=15, Y=15)
        self.toolbar_widget.move(15, 15)
        
        # --- Robot Structure Line ---
        self.line, = self.ax.plot([], [], [], 'o-', lw=5, color='#2c3e50', markerfacecolor='#e74c3c', markersize=8)
        
        # --- Tool tip trail ---
        self.tip_trail = []
        self.tip_trail_line, = self.ax.plot([], [], [], '-', lw=2, color='#2980b9', alpha=0.8, label='Tool Tip Trail')
        
        # --- Target / Start Point Trackers & Projections ---
        self.start_marker, = self.ax.plot([], [], [], 'go', markersize=8, label='Start Point')
        self.start_proj, = self.ax.plot([], [], [], 'g--', alpha=0.6, lw=1.5)
        
        self.target_marker, = self.ax.plot([], [], [], 'ro', markersize=8, label='Target Point')
        self.target_proj, = self.ax.plot([], [], [], 'r--', alpha=0.6, lw=1.5)
        
        self.ax.legend(loc='upper right') # Moved legend to right to avoid overlapping with toolbar
        
        self.joint_labels = [self.ax.text(0, 0, 0, f"J{i}", color='blue', fontsize=9) for i in range(1, 7)]

        self.start_xyz = None
        self.target_xyz = None

        # --- Grid Bounds & Environment Configurations ---
        self.ax.set_xlim3d([-800, 800])
        self.ax.set_ylim3d([-800, 800])
        self.ax.set_zlim3d([0.0, 900])
        self.ax.set_xlabel('X (mm)', fontweight='bold')
        self.ax.set_ylabel('Y (mm)', fontweight='bold')
        self.ax.set_zlabel('Z (mm)', fontweight='bold')
        
        # Styling panes for standard clear CAD interface look
        self.ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        self.ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
        self.ax.zaxis.set_pane_color((0.96, 0.96, 0.96, 1.0)) 
        self.ax.grid(True, linestyle=':', color='#b0b0b0')

    def resizeEvent(self, event):
        """Ensures the floating widget stays correctly sized when the window is resized."""
        super().resizeEvent(event)
        self.toolbar_widget.adjustSize()

    def change_view_angle(self, view_name):
        """Re-orients the 3D camera instantly to predefined plane angles."""
        if view_name in self.view_angles:
            elev, azim = self.view_angles[view_name]
            self.ax.view_init(elev=elev, azim=azim)
            self.canvas.draw_idle()

    def apply_visibility_states(self):
        self.line.set_visible(self.show_robot)
        for label in self.joint_labels:
            label.set_visible(self.show_robot)

        for item in [self.start_marker, self.start_proj,
                     self.target_marker, self.target_proj]:
            item.set_visible(self.show_points)

        self.tip_trail_line.set_visible(self.show_trail)

        # Only toggle grid visibility here; keep axis numbers and labels visible.
        self.ax.grid(self.show_grid)

    def toggle_robot(self, checked):
        self.show_robot = checked
        self.apply_visibility_states()
        self.canvas.draw_idle()

    def toggle_points(self, checked):
        self.show_points = checked
        self.apply_visibility_states()
        self.canvas.draw_idle()

    def toggle_trail(self, checked):
        self.show_trail = checked
        self.apply_visibility_states()
        self.canvas.draw_idle()

    def set_trail_visibility(self, visible: bool):
        """External API to toggle the tip trail visibility."""
        self.show_trail = bool(visible)
        self.apply_visibility_states()
        self.canvas.draw_idle()

    def toggle_grid(self, checked):
        self.show_grid = checked
        self.apply_visibility_states()
        self.canvas.draw_idle()

    def clear_tip_trail(self):
        self.tip_trail = []
        self.tip_trail_line.set_data([], [])
        self.tip_trail_line.set_3d_properties([])
        self.canvas.draw_idle()

    def set_visual_points(self, start_xyz, target_xyz):
        """Registers external frame inputs coordinates."""
        self.start_xyz = start_xyz
        self.target_xyz = target_xyz

    def update_view(self, points, tip_trail=None):
        """Redraws entire kinematics frame tree and tracking lines dynamically."""
        x_data = points[:, 0]
        y_data = points[:, 1]
        z_data = points[:, 2]
        self.latest_points = points
        
        # 1. Update physical body line paths
        self.line.set_data(x_data, y_data)
        self.line.set_3d_properties(z_data)
        
        # 2. Update start reference tracking metrics
        if self.start_xyz is not None:
            sx, sy, sz = self.start_xyz
            self.start_marker.set_data([sx], [sy])
            self.start_marker.set_3d_properties([sz])
            
            self.start_proj.set_data([sx, sx], [sy, sy])
            self.start_proj.set_3d_properties([0, sz])

        # 3. Update target reference tracking metrics
        if self.target_xyz is not None:
            tx, ty, tz = self.target_xyz
            self.target_marker.set_data([tx], [ty])
            self.target_marker.set_3d_properties([tz])
            
            self.target_proj.set_data([tx, tx], [ty, ty])
            self.target_proj.set_3d_properties([0, tz])
        
        # 4. Refresh spatial node texts tags
        for i in range(6):
            self.joint_labels[i].set_position((x_data[i+1] + 20, y_data[i+1] + 20))
            self.joint_labels[i].set_3d_properties(z_data[i+1] + 20, 'z')

        # 5. Update the tool tip trail.
        if tip_trail is None:
            self.tip_trail.append(points[-1])
            trail_array = np.array(self.tip_trail)
        else:
            self.tip_trail = [np.array(p) for p in tip_trail]
            trail_array = np.array(self.tip_trail)

        if trail_array.size == 0:
            self.tip_trail_line.set_data([], [])
            self.tip_trail_line.set_3d_properties([])
        else:
            self.tip_trail_line.set_data(trail_array[:, 0], trail_array[:, 1])
            self.tip_trail_line.set_3d_properties(trail_array[:, 2])

        self.canvas.draw_idle()

    def on_mouse_move(self, event):
        if event.inaxes is not self.ax or self.latest_points is None:
            self.snap_marker.set_visible(False)
            self.snap_text.set_visible(False)
            self.canvas.draw_idle()
            return

        candidates = []
        if self.start_xyz is not None:
            candidates.append(('Start', np.array(self.start_xyz)))
        if self.target_xyz is not None:
            candidates.append(('Target', np.array(self.target_xyz)))

        for idx, point in enumerate(self.latest_points[1:7], start=1):
            candidates.append((f'Joint {idx}', point))

        if not candidates:
            self.snap_marker.set_visible(False)
            self.snap_text.set_visible(False)
            self.canvas.draw_idle()
            return

        points3d = np.array([p for _, p in candidates])
        x2, y2, _ = proj3d.proj_transform(points3d[:, 0], points3d[:, 1], points3d[:, 2], self.ax.get_proj())
        screen_coords = self.ax.transData.transform(np.vstack([x2, y2]).T)
        pointer = np.array([event.x, event.y])
        distances = np.linalg.norm(screen_coords - pointer, axis=1)
        best_index = int(np.argmin(distances))

        if distances[best_index] <= 12.0:
            name, coord = candidates[best_index]
            self.snap_marker.set_data([coord[0]], [coord[1]])
            self.snap_marker.set_3d_properties([coord[2]])
            self.snap_marker.set_visible(True)
            self.snap_text.set_text(f"{name}: {coord[0]:.0f}, {coord[1]:.0f}, {coord[2]:.0f}")
            self.snap_text.set_visible(True)
        else:
            self.snap_marker.set_visible(False)
            self.snap_text.set_visible(False)

        self.canvas.draw_idle()