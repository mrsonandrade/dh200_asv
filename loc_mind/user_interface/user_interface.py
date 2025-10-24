import ctypes
import sys, os
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QDockWidget, QSizePolicy, QComboBox, QComboBox
from PyQt5.QtWidgets import QSlider, QLabel
from PyQt5.QtWidgets import QTextEdit
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QTextCursor
from PyQt5.QtCore import QObject
import matplotlib.pyplot as plt
import json
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import contextily as ctx
import pyproj
import time

from modules.XBee import XBee

import warnings
warnings.filterwarnings("ignore")

class EmittingStream(QObject):
    textWritten = pyqtSignal(str)

    def write(self, text):
        if text:
            self.textWritten.emit(str(text))

    def flush(self):
        pass

class MapCanvas(FigureCanvas):
    def __init__(self, parent=None):
        fig, ax = plt.subplots(figsize=(15, 15))
        super().__init__(fig)
        self.ax = ax
        self.parent_window = parent  # store reference to MainWindow
        self.vehicles = {}
        self.waypoints = {}

        self.center_lat = -22.8624
        self.center_lon = -43.2205
        self.zoom_level = 17
        self.map_style = "street"

        self._lat_factor_lock = None   # lock latitude factor on first draw
        self._tiles_across = 8        # how many tiles wide the view should cover
        self._half_w = None           # half-width in Web Mercator meters (updated each draw)
        self._half_h = None           # half-height in Web Mercator meters

        self.plot_map()

    def pan(self, dx_tiles=0, dy_tiles=0):
        """Pan the map by dx_tiles horizontally and dy_tiles vertically."""
        proj_wgs84 = pyproj.CRS("EPSG:4326")
        proj_web_mercator = pyproj.CRS("EPSG:3857")
        transformer_to_merc = pyproj.Transformer.from_crs(proj_wgs84, proj_web_mercator, always_xy=True)
        transformer_to_wgs = pyproj.Transformer.from_crs(proj_web_mercator, proj_wgs84, always_xy=True)

        # Convert center to meters
        center_x, center_y = transformer_to_merc.transform(self.center_lon, self.center_lat)

        # Pan distance in meters for given zoom
        tile_size_m = 40075016.68557849 / (2 ** self.zoom_level)
        tiles_across = 4
        view_width_m = tile_size_m * tiles_across
        view_height_m = view_width_m * (self.ax.get_window_extent().height / self.ax.get_window_extent().width)

        # Apply panning
        center_x += dx_tiles * view_width_m / tiles_across
        center_y += dy_tiles * view_height_m / tiles_across

        # Clip to Web Mercator bounds
        merc_max = 20037508.342789244
        center_x = max(-merc_max, min(merc_max, center_x))
        center_y = max(-merc_max, min(merc_max, center_y))

        # Back to lon/lat
        self.center_lon, self.center_lat = transformer_to_wgs.transform(center_x, center_y)

        self.plot_map()

    def pan_left(self):
        self.pan(dx_tiles=-0.5)

    def pan_right(self):
        self.pan(dx_tiles=0.5)

    def pan_up(self):
        self.pan(dy_tiles=0.5)

    def pan_down(self):
        self.pan(dy_tiles=-0.5)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.plot_map()

    def set_center_and_zoom(self, new_center_lon, new_center_lat, new_zoom=None):
        """Recenter and optionally change zoom level, keeping fixed width framing."""
        self.center_lon = new_center_lon
        self.center_lat = new_center_lat
        if new_zoom is not None:
            self.zoom_level = new_zoom
        self.plot_map()

    def connect_toolbar_events(self, toolbar):
        self.toolbar = toolbar
        self.toolbar.pan = self._toolbar_pan
        self.toolbar.zoom = self._toolbar_zoom

    def _toolbar_pan(self, *args, **kwargs):
        pass

    def _toolbar_zoom(self, *args, **kwargs):
        if kwargs.get("direction") == "in":
            self.zoom_in()
        else:
            self.zoom_out()

    def plot_map(self):
        self.ax.cla()
        self.ax.set_axis_off()

        proj_wgs84 = pyproj.CRS("EPSG:4326")
        proj_web_mercator = pyproj.CRS("EPSG:3857")
        transformer = pyproj.Transformer.from_crs(proj_wgs84, proj_web_mercator, always_xy=True)

        # Project center
        center_x, center_y = transformer.transform(self.center_lon, self.center_lat)

        # Aspect ratio
        bbox = self.ax.get_window_extent().transformed(self.figure.dpi_scale_trans.inverted())
        aspect_ratio = bbox.height / bbox.width

        # Fixed width for zoom
        tile_size_m = 40075016.68557849 / (2 ** self.zoom_level)
        tiles_across = 4
        half_w = (tile_size_m * tiles_across) / 2
        half_h = half_w * aspect_ratio

        # Clip to Web Mercator limits
        merc_max = 20037508.342789244
        x_min = max(-merc_max, center_x - half_w)
        x_max = min(merc_max, center_x + half_w)
        y_min = max(-merc_max, center_y - half_h)
        y_max = min(merc_max, center_y + half_h)

        try:
            img, extent = ctx.bounds2img(
                x_min, y_min, x_max, y_max,
                zoom=self.zoom_level,
                source=(
                    ctx.providers.OpenStreetMap.Mapnik
                    if self.map_style == "street"
                    else ctx.providers.Esri.WorldImagery
                )
            )
            self.ax.imshow(img, extent=extent, origin="upper")
        except Exception as e:
            print(f"Basemap fetch error: {e}")
            self.ax.set_facecolor("lightgray")

        # Vehicles
        for vehicle_i, data in self.vehicles.items():
            vx, vy = transformer.transform(data["lng"], data["lat"]) # going from deg to meters
            self.ax.plot(vx, vy, marker=(2, 0, data["heading"]), markersize=20, c='red')
            self.ax.plot(vx, vy, marker=(3, 0, data["heading"]), markersize=10, c='lime')

        try:
            wpts = np.asarray(self.waypoints[self.parent_window.selected_vehicle])#/100000.0
            for ai in range(1, len(wpts[:,0])):
                self.ax.annotate('', xy=(wpts[ai, 0], wpts[ai, 1]), xytext=(wpts[ai-1, 0], wpts[ai-1, 1]),
                                 arrowprops=dict(arrowstyle="->", color="blue", lw=2.0))
            self.ax.plot(wpts[0,0], wpts[0,1], marker='o', markersize=8, c='blue')
            self.ax.plot(wpts[-1, 0], wpts[-1, 1], marker='o', markersize=8, c='green')
        except:
            pass

        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_position([0, 0, 1, 1])
        self.figure.subplots_adjust(0, 0, 1, 1)
        self.ax.margins(0)
        self.ax.set_anchor('C')
        self.ax.set_aspect("auto")

        self.draw()
        self.ax.figure.canvas.flush_events()

    def update_vehicle_position(self, vehicle_id, lat, lng, heading):
        if vehicle_id not in self.vehicles:
            self.vehicles[vehicle_id] = {}
        self.vehicles[vehicle_id]['lat'] = lat
        self.vehicles[vehicle_id]['lng'] = lng
        self.vehicles[vehicle_id]['heading'] = heading
        self.plot_map()

    def toggle_map_style(self):
        self.map_style = "satellite" if self.map_style == "street" else "street"
        self.plot_map()

    def zoom_in(self):
        self.zoom_level = min(self.zoom_level + 1, 20)
        self.plot_map()

    def zoom_out(self):
        self.zoom_level = max(self.zoom_level - 1, 1)
        self.plot_map()


    def send_command_to_vehicle(self, command):
        print(f"Sending command to vehicle: {command}")

class MessageReceiverThread(QThread):
    vehicle_message_signal = pyqtSignal(str, float, float, float)

    def __init__(self):
        super().__init__()

        self.communication = XBee()

        self.serial_connected = False

        if self.communication.ser.isOpen():
            self.serial_connected = True

    def y_to_x_angle(self, angle_y: float) -> float:
        """
        Convert an angle measured from Y-axis (IF +90-angle_y) (CW) to
        an angle measured from X-axis (CCW).

        Parameters:
            angle_y (float): input angle in degrees, 0° at +Y

        Returns:
            float: converted angle in degrees, 0° at +X
        """
        angle_x = (-angle_y) % 360
        return angle_x

    def run(self):
        while True:
            try:
                data = self.communication.data.copy()
                if len(data) > 0: print(data)
                for key_id in data.keys():
                    vehicle_id = key_id
                    for key_info_type in data[key_id].keys():
                        if key_info_type == "pose":
                            heading = self.y_to_x_angle(data[key_id][key_info_type][2])
                        if key_info_type == "lat":
                            lat = data[key_id][key_info_type]
                        if key_info_type == "lng":
                            lng = data[key_id][key_info_type]

                self.vehicle_message_signal.emit(vehicle_id, lat, lng, heading)
            except:
                pass

            time.sleep(1)


class WorkerThread(QThread):
    update_signal = pyqtSignal(str, float, float, float)

    def __init__(self):
        super().__init__()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(
            "SPIRE - System for Planning, Integration, and REmote-control - Laboratory of Waves and Current (LOC/COPPE/UFRJ)")
        self.setGeometry(100, 100, 1800, 900)

        self.remote_control_active = False  # State for remote control
        self.selected_vehicle = "ALL"  # Default selected vehicle
        self.start_waypoints = 0

        self.waypoints_active = False

        self.setStyleSheet("""
            QMainWindow {
                background-color: #121212;
                color: white;
            }
            QPushButton {
                background-color: #333333;
                color: white;
                border-radius: 5px;
                border: 1px solid #444444;
                padding: 10px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #444444;
            }
            QPushButton:pressed {
                background-color: #555555;
            }
            QDockWidget {
                background-color: #1e1e1e;
            }
            QToolBar {
                background-color: #121212;
                border: none;
            }
            QStatusBar {
                background-color: #121212;
                color: white;
            }
            QToolButton {
                font-size: 1px;
                border: none;
                padding: 0px;
            }
        """)

        self.map_canvas = MapCanvas(self)

        self.waypoints = self.map_canvas.waypoints

        self.map_canvas.mpl_connect("button_press_event", self.on_press)
        self.map_canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout()
        layout.addWidget(self.map_canvas)

        # Controls below the map
        controls_layout = QHBoxLayout()

        zoom_in_btn = QPushButton("+ Zoom")
        zoom_in_btn.clicked.connect(lambda: self.map_canvas.zoom_in())
        controls_layout.addWidget(zoom_in_btn)

        zoom_out_btn = QPushButton("- Zoom")
        zoom_out_btn.clicked.connect(lambda: self.map_canvas.zoom_out())
        controls_layout.addWidget(zoom_out_btn)

        btn_up = QPushButton("↑ Up")
        btn_up.clicked.connect(self.map_canvas.pan_up)
        controls_layout.addWidget(btn_up)

        btn_down = QPushButton("↓ Down")
        btn_down.clicked.connect(self.map_canvas.pan_down)
        controls_layout.addWidget(btn_down)

        btn_left = QPushButton("← Left")
        btn_left.clicked.connect(self.map_canvas.pan_left)
        controls_layout.addWidget(btn_left)

        btn_right = QPushButton("→ Right")
        btn_right.clicked.connect(self.map_canvas.pan_right)
        controls_layout.addWidget(btn_right)

        layout.addLayout(controls_layout)

        self.toolbar = NavigationToolbar(self.map_canvas, self)
        self.map_canvas.connect_toolbar_events(self.toolbar)

        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        self.create_side_panel()

        # Initialize and start the message receiver thread
        self.message_receiver_thread = MessageReceiverThread()
        self.message_receiver_thread.vehicle_message_signal.connect(self.update_vehicle_list)
        self.message_receiver_thread.vehicle_message_signal.connect(self.map_canvas.update_vehicle_position)
        self.message_receiver_thread.start()

        # --- Create console panel to replicate the shell---
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setStyleSheet("background-color: black; color: white; font-family: Consolas; font-size: 12px;")

        dock_console = QDockWidget("Console Output", self)
        dock_console.setWidget(self.console)
        dock_console.setMinimumHeight(150)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock_console)

        # Redirect stdout/stderr
        sys.stdout = EmittingStream()
        sys.stderr = EmittingStream()

        # Force queued connection so slot runs in GUI thread
        sys.stdout.textWritten.connect(self.append_console_text, Qt.QueuedConnection)
        sys.stderr.textWritten.connect(self.append_console_text, Qt.QueuedConnection)

    def append_console_text(self, text):
        """Safely append text to console"""
        self.console.moveCursor(QTextCursor.End)
        self.console.insertPlainText(text)
        self.console.moveCursor(QTextCursor.End)

    def closeEvent(self, event):
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        super().closeEvent(event)

    def toggle_map_style(self):
        self.map_canvas.toggle_map_style()
        if self.map_canvas.map_style == "satellite":
            self.map_toggle_btn.setText("Switch to Street View")
        else:
            self.map_toggle_btn.setText("Switch to Satellite View")

    def on_press(self, event):

        if self.waypoint_btn.isChecked() and self.waypoints_active == False:
            self.waypoints[self.selected_vehicle] = []
            self.waypoints_active = True

        if self.waypoint_btn.isChecked() and self.waypoints_active:
            self.waypoints[self.selected_vehicle].append([round(event.xdata, 6),
                                                          round(event.ydata, 6)])
            print(self.waypoints)

    def on_slider_value_changed(self, value):
        self.value_label.setText(f"Motor Power: {value}%")
        try:
            self.message_receiver_thread.communication.send_message(
                "robot_id,%s,command,motors_power,%d" % (self.selected_vehicle, value))
            print("robot_id,%s,command,motors_power,%d" % (self.selected_vehicle, value))
        except:
            pass

    def create_side_panel(self):
        side_panel = QWidget()
        side_layout = QVBoxLayout()
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(0)

        # Dropdown menu for vehicle selection
        self.vehicle_dropdown = QComboBox()
        self.vehicle_dropdown.addItem("ALL")  # Default option
        self.vehicle_dropdown.currentTextChanged.connect(self.select_vehicle)
        side_layout.addWidget(self.vehicle_dropdown)

        self.waypoint_btn = QPushButton("Pin waypoints")
        self.waypoint_btn.setCheckable(True)
        self.waypoint_btn.clicked.connect(self.toggle_waypoint)
        side_layout.addWidget(self.waypoint_btn)

        self.manual_btn = QPushButton("Activate Manual Control")
        self.manual_btn.setCheckable(True)
        self.manual_btn.clicked.connect(self.toggle_manual_control)
        side_layout.addWidget(self.manual_btn)

        mission_btn = QPushButton("Send waypoints")
        mission_btn.clicked.connect(self.send_command_waypoints)
        mission_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        side_layout.addWidget(mission_btn, alignment=Qt.AlignBottom)

        self.mission_btn = QPushButton("Start Mission")
        self.mission_btn.clicked.connect(self.send_command_start_mission)
        self.mission_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        side_layout.addWidget(self.mission_btn, alignment=Qt.AlignBottom)

        # Map style toggle
        self.map_toggle_btn = QPushButton("Switch to Satellite View")
        self.map_toggle_btn.clicked.connect(self.toggle_map_style)
        side_layout.addWidget(self.map_toggle_btn)

        # MOTOR
        side_layout.addSpacing(20)  # Add 20 pixels of space
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(0)  # Initial value
        self.value_label = QLabel(f"Motor Power: {self.slider.value()}%")
        self.value_label.setStyleSheet("color: white;")
        self.value_label.setAlignment(Qt.AlignCenter)
        # Connect slider valueChanged signal to update_label slot
        self.slider.valueChanged.connect(self.on_slider_value_changed)
        side_layout.addWidget(self.slider)
        side_layout.addWidget(self.value_label)

        side_layout.addStretch(1)
        side_layout.setContentsMargins(10, 0, 0, 0)  # 10 pixels margin from the left
        side_panel.setLayout(side_layout)

        dock = QDockWidget("Menu", self)
        dock.setWidget(side_panel)
        dock.setMinimumWidth(100)
        dock.setFeatures(QDockWidget.DockWidgetMovable)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock)

        dock.setFloating(False)

    def toggle_manual_control(self):
        self.manual_control_active = self.manual_btn.isChecked()
        if self.manual_control_active:
            self.manual_btn.setStyleSheet("background-color : green")
            self.manual_btn.setText("Manual Control Active")
            print("Manual control activated.")
        else:
            self.manual_btn.setStyleSheet("background-color : #333333")
            self.manual_btn.setText("Activate Manual Control")
            print("Manual control deactivated.")

    def toggle_waypoint(self):
        if self.waypoint_btn.isChecked():
            self.waypoint_btn.setStyleSheet("background-color : green")
            self.waypoint_btn.setText("Save waypoints")

        else:
            self.waypoint_btn.setStyleSheet("background-color : #333333")
            self.waypoint_btn.setText("Pin waypoints")
            self.waypoints_active = False

    def send_command_waypoints(self):
        wpts = np.asarray(self.waypoints[self.selected_vehicle])
        # EPSG:3857 coordinates (from meters to degrees)
        R = 6378137.0
        wpts[:,0] = np.degrees(wpts[:,0] / R)
        wpts[:,1] = np.degrees(np.arctan(np.sinh(wpts[:,1] / R)))
        wpts = wpts*1000000
        wpts = wpts.astype(int)
        wpts = wpts.tolist()
        command = json.dumps(wpts)
        if self.selected_vehicle == "ALL":
            for vehicle_id in self.map_canvas.vehicles.keys():
                self.message_receiver_thread.communication.send_message("robot_id,%s,command,wpts,%s" % (vehicle_id,command))
                print("robot_id,%s: Sending waypoints" % (vehicle_id))
        else:
            self.message_receiver_thread.communication.send_message("robot_id,%s,command,wpts,%s" % (self.selected_vehicle,command))
            print("robot_id,%s: Sending waypoints" % (self.selected_vehicle))

    def send_command_start_mission(self):

        if self.start_waypoints==0:
            self.start_waypoints=1
            self.mission_btn.setStyleSheet("background-color : red")
            self.mission_btn.setText("Stop mission")
        elif self.start_waypoints==1:
            self.start_waypoints=0
            self.mission_btn.setStyleSheet("background-color : #333333")
            self.mission_btn.setText("Start mission")

        if self.selected_vehicle == "ALL":
            for vehicle_id in self.map_canvas.vehicles.keys():
                self.message_receiver_thread.communication.send_message("robot_id,%s,command,swpts,%d" % (vehicle_id, self.start_waypoints))
                if self.start_waypoints:
                    print("robot_id,%s: Starting waypoints mission" % (vehicle_id))
                else:
                    print("robot_id,%s: Stopping waypoints mission" % (vehicle_id))
        else:
            self.message_receiver_thread.communication.send_message("robot_id,%s,command,swpts,%d" % (self.selected_vehicle, self.start_waypoints))
            if self.start_waypoints:
                print("robot_id,%s: Starting waypoints mission" % (self.selected_vehicle))
            else:
                print("robot_id,%s: Stopping waypoints mission" % (self.selected_vehicle))

    def send_remote_command(self, command):
        if self.selected_vehicle == "ALL":
            for vehicle_id in self.map_canvas.vehicles.keys():
                self.communication.send_message(f"{vehicle_id}:{command}")
        else:
            self.communication.send_message(f"{self.selected_vehicle}:{command}")

    def select_vehicle(self, vehicle_id):
        self.selected_vehicle = vehicle_id
        print(f"Selected vehicle: {vehicle_id}")

    def update_vehicle_list(self, vehicle_id, lat, lng, heading):
        if vehicle_id not in [self.vehicle_dropdown.itemText(i) for i in range(self.vehicle_dropdown.count())]:
            self.vehicle_dropdown.addItem(vehicle_id)

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() == Qt.Key_X:
            if self.manual_control_active and self.selected_vehicle != "ALL":
                print("%s\tStop" % (self.selected_vehicle))
                self.message_receiver_thread.communication.send_message("robot_id,%s,rc,0" % (self.selected_vehicle))
        if event.key() == Qt.Key_W:
            if self.manual_control_active and self.selected_vehicle != "ALL":
                print("%s\tForward" % (self.selected_vehicle))
                self.message_receiver_thread.communication.send_message("robot_id,%s,rc,1" % (self.selected_vehicle))
        elif event.key() == Qt.Key_S:
            if self.manual_control_active and self.selected_vehicle != "ALL":
                print("%s\tBackward" % (self.selected_vehicle))
                self.message_receiver_thread.communication.send_message("robot_id,%s,rc,2" % (self.selected_vehicle))
        elif event.key() == Qt.Key_A:
            if self.manual_control_active and self.selected_vehicle != "ALL":
                print("%s\tLeft" % (self.selected_vehicle))
                self.message_receiver_thread.communication.send_message("robot_id,%s,rc,3" % (self.selected_vehicle))
        elif event.key() == Qt.Key_D:
            if self.manual_control_active and self.selected_vehicle != "ALL":
                print("%s\tRight" % (self.selected_vehicle))
                self.message_receiver_thread.communication.send_message("robot_id,%s,rc,4" % (self.selected_vehicle))
        else:
            if self.manual_control_active and self.selected_vehicle != "ALL":
                super().keyPressEvent(event)

    def mission_planning(self):
        print("Mission Planning clicked - functionality to be implemented!")

    def settings(self):
        print("Settings clicked - functionality to be implemented!")

    def about(self):
        print("About clicked - functionality to be implemented!")

def run_as_admin():
    if ctypes.windll.shell32.IsUserAnAdmin():
        return True
    else:
        ctypes.windll.shell32.ShellExecuteW(None, "runas", sys.executable, " ".join(sys.argv), None, 1)
        return False

if __name__ == "__main__":
    if run_as_admin():
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        sys.exit(app.exec())
