# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import json
import os
import threading
from datetime import datetime

import cv2
import numpy as np

from gcode_program import GCodeError
from gcode_runner import ExecutionState, GCodeRunner, RunResult
from gui_components.image_viewer_widget import ImageViewerWidget
from gui_components.realtime_controller_widget import RealtimeControllerWidget
from hardware.camera_basler import BaslerCamera
from hardware.camera_opencv import OpenCVCamera
from hardware.device_discovery import list_camera_devices, list_serial_devices
from hardware.focus_axis import FocusAxis, MoveOutcome, MoveResult
from hardware.open_micro_stage_api import OpenMicroStageInterface, SerialInterface, format_positive
from image_processing.image_point_tracker import ImagePointTracker
from optical_alignment import OpticalAlignment
from version import __version__
from PySide6.QtCore import Qt, QSettings, QThread, QTimer, Signal
from PySide6.QtGui import QCloseEvent
from PySide6.QtUiTools import loadUiType
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
)

_ui_path = os.path.join(os.path.dirname(__file__), "mainwindow.ui")
Ui_DeviceControlMainWindow, _ = loadUiType(_ui_path)


class CameraStreamWorker(QThread):
    frame_ready = Signal(object)
    stream_error = Signal(str)

    def __init__(self, camera, parent=None):
        super().__init__(parent)
        self.camera = camera
        self.running = True

    def stop(self):
        self.running = False
        try:
            self.camera.stop_grabbing()
        except Exception:
            pass
        self.wait(1500)

    def _handle_frame(self, frame):
        if not self.running:
            return False

        self.frame_ready.emit(frame)
        return True

    def run(self):
        try:
            self.camera.grab_loop(callback=self._handle_frame, timeout_ms=500)
        except Exception as exc:
            if self.running:
                self.stream_error.emit(str(exc))


class CalibrationPlotDialog(QDialog):

    def __init__(self, joint_data, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Axis Calibration")
        self.resize(900, 640)

        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
        from matplotlib.backends.backend_qtagg import NavigationToolbar2QT
        from matplotlib.figure import Figure

        canvas = FigureCanvasQTAgg(Figure(figsize=(10, 7)))
        ax = canvas.figure.subplots(1, 1)

        for joint_index, data in joint_data:
            ax.plot(data[0], data[2], label=f"Actuator {joint_index}")

        ax.set_xlabel("Motor Angle [rad]")
        ax.set_ylabel("Encoder Counts Raw")
        ax.set_title("Encoder Count Plot")
        ax.legend()
        ax.grid(True)
        canvas.figure.tight_layout()

        saved_label = QLabel("Calibration saved to device")
        saved_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addWidget(NavigationToolbar2QT(canvas, self))
        layout.addWidget(canvas)
        layout.addWidget(saved_label)


class DeviceControlMainWindow(QMainWindow, Ui_DeviceControlMainWindow):
    focus_move_finished = Signal(object)

    def __init__(self, oms: OpenMicroStageInterface, camera=None, focus_axis=None):
        super().__init__()

        self.oms = oms
        self.focus_axis = focus_axis if focus_axis is not None else FocusAxis()
        self.focus_thread = None
        self.focus_state = "idle" if self.focus_axis.available else "unavailable"
        self.focus_error_message = "" if self.focus_axis.available else str(self.focus_axis.unavailable_reason)
        self.camera = camera
        self.camera_stream_worker = None
        self.gcode_runner = None
        self.gcode_execution_state = ExecutionState.IDLE
        self.gcode_error_message = ""
        self.gcode_last_result = None

        self.settings = QSettings()
        self.pixel_per_mm = 2000.0/2.2
        self.connected_stage_label = None
        self.connected_camera_label = None
        self.serial_devices = []
        self.camera_devices = []
        self.stage_dependent_widgets = []
        self.camera_dependent_widgets = []

        self.last_frame = None
        self.draw_buffer = None
        self.current_pos = [0, 0, 0]
        self.step_sizes = [1.0, 0.1, 0.01, 0.001, 0.0001]
        self.feedrates = [50.0, 5.0, 5.0, 1.0, 0.1]
        self.step_size_idx = 1
        self.waypoints = []
        self.waypoint_idx = 1000000

        self.image_point_tracker = ImagePointTracker()

        self.init_ui()
        self.refresh_serial_devices()
        self.refresh_camera_devices()
        self.show_placeholder_frame()
        self.update_connection_state()

        if self.camera is not None and self.camera.is_connected():
            self.connected_camera_label = "Configured Camera"
            self.load_camera_settings(self.connected_camera_label)
            self.apply_camera_settings()
            self.start_camera_stream()

        if self.oms.is_connected():
            self.connected_stage_label = "Configured Device"
            self.on_stage_connected()

    def init_ui(self):
        self.setupUi(self)
        self.setWindowTitle(f"{self.windowTitle()}  v{__version__}")

        self.video_viewer.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.serial_refresh_button.clicked.connect(self.refresh_serial_devices)
        self.serial_connect_button.clicked.connect(self.toggle_stage_connection)
        self.camera_refresh_button.clicked.connect(self.refresh_camera_devices)
        self.camera_connect_button.clicked.connect(self.toggle_camera_connection)

        for name, axis, direction in [
            ("btn_y_minus", 1, -1), ("btn_z_plus", 2, +1),
            ("btn_x_minus", 0, -1), ("btn_x_plus", 0, +1),
            ("btn_y_plus", 1, +1), ("btn_z_minus", 2, -1),
        ]:
            btn = getattr(self, name)
            btn.clicked.connect(lambda checked=False, a=axis, d=direction: self.move_axis(a, d))

        self.accel_spinbox.editingFinished.connect(self.apply_acceleration_setting)

        self.tool_controls = [
            (0, self.tool1_spinbox, self.btn_tool1_on),
            (1, self.tool2_spinbox, self.btn_tool2_on),
        ]
        for tool_idx, spinbox, button in self.tool_controls:
            button.toggled.connect(lambda _checked=False, i=tool_idx: self.apply_tool_setting(i))
            spinbox.valueChanged.connect(lambda _value=0.0, i=tool_idx: self.apply_tool_setting(i))

        self.exposure_spinbox.valueChanged.connect(self.apply_camera_settings)
        self.gain_spinbox.valueChanged.connect(self.apply_camera_settings)
        self.white_balance_spinbox.valueChanged.connect(self.apply_camera_settings)

        self.step_button_group = QButtonGroup()
        self.step_button_group.setExclusive(True)
        for i in range(len(self.step_sizes)):
            btn = getattr(self, f"step_btn_{i}")
            btn.clicked.connect(lambda checked=False, idx=i: self.set_step_size(idx))
            self.step_button_group.addButton(btn, i)

        self.realtime_control_widget.setup(self.video_viewer.viewport(), self.oms)
        self.realtime_control_widget.stop_control_signal.connect(self.on_stop_realtime_control)
        self.realtime_control_widget.start_guard = lambda: not self.focus_controls_locked()

        self.register_stage_widget(self.device_controls_frame)
        self.register_stage_widget(self.AdvancedTab)
        self.register_stage_widget(self.PathTab)
        self.register_camera_widget(self.CameraTab)

        self.btn_3point_alignment.clicked.connect(self.run_3point_alignment)
        self.btn_set_origin.clicked.connect(self.set_origin)
        self.btn_set_tracking.clicked.connect(self.set_tracking_point)
        self.btn_clear.clicked.connect(self.clear_draw_buffer)
        self.btn_load_transform.clicked.connect(self.load_transform)
        self.btn_save_transform.clicked.connect(self.save_transform)
        self.btn_fiber_alignment.clicked.connect(self.run_fiber_alignment)
        self.btn_calibrate_axis.clicked.connect(self.run_axis_calibration)

        self.btn_add_waypoint.clicked.connect(self.add_waypoint)
        self.btn_clear_waypoints.clicked.connect(self.clear_waypoints)
        self.btn_run_path.clicked.connect(self.run_path)
        self.btn_save_path.clicked.connect(self.save_path)
        self.run_gcode_button.clicked.connect(self.run_gcode_from_file)

        self.btn_save_screenshot.clicked.connect(self.save_screenshot)
        self.btn_dark_shot_compensation.clicked.connect(self.capture_dark_image)

        self.btn_home.clicked.connect(self.home)

        self.build_focus_controls()

        self.main_tabs.setCurrentIndex(0)
        self.update_waypoint_info()

    def register_stage_widget(self, widget):
        self.stage_dependent_widgets.append(widget)
        return widget

    def register_camera_widget(self, widget):
        self.camera_dependent_widgets.append(widget)
        return widget

    def show_placeholder_frame(self):
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        self.video_viewer.set_image(placeholder, pixel_per_mm=self.pixel_per_mm)

    def clear_visual_state(self):
        self.last_frame = None
        self.draw_buffer = None
        self.image_point_tracker.reset()

    def refresh_serial_devices(self):
        selected_id = self.serial_combo.currentData().get("id") if self.serial_combo.currentData() else None
        if selected_id is None:
            selected_id = self.settings.value("Connections/last_stage_id")
        self.serial_devices = list_serial_devices()
        self.serial_combo.clear()

        for device in self.serial_devices:
            self.serial_combo.addItem(device["label"], device)

        if selected_id is not None:
            for index, device in enumerate(self.serial_devices):
                if device["id"] == selected_id:
                    self.serial_combo.setCurrentIndex(index)
                    break

        self.update_connection_state()

    def refresh_camera_devices(self):
        selected_id = self.camera_combo.currentData().get("id") if self.camera_combo.currentData() else None
        if selected_id is None:
            selected_id = self.settings.value("Connections/last_camera_id")
        self.camera_devices = list_camera_devices()
        self.camera_combo.clear()

        for device in self.camera_devices:
            self.camera_combo.addItem(device["label"], device)

        if selected_id is not None:
            for index, device in enumerate(self.camera_devices):
                if device["id"] == selected_id:
                    self.camera_combo.setCurrentIndex(index)
                    break

        self.update_connection_state()

    def update_connection_state(self):
        stage_connected = self.oms.is_connected()
        camera_connected = self.camera is not None and self.camera.is_connected()

        for widget in self.stage_dependent_widgets:
            widget.setEnabled(stage_connected)

        for widget in self.camera_dependent_widgets:
            widget.setEnabled(camera_connected)

        self.serial_connect_button.setText("Close" if stage_connected else "Open")
        self.serial_connect_button.setEnabled(stage_connected or self.serial_combo.count() > 0)
        self.camera_connect_button.setText("Close" if camera_connected else "Open")
        self.camera_connect_button.setEnabled(camera_connected or self.camera_combo.count() > 0)

        stage_status = f"[{self.connected_stage_label}]" if stage_connected else "[Disconnected]"
        camera_status = f"[{self.connected_camera_label}]" if camera_connected else "[Disconnected]"
        self.serial_status_label.setText(stage_status)
        self.camera_status_label.setText(camera_status)

        for label, connected in (
            (self.serial_status_label, stage_connected),
            (self.camera_status_label, camera_connected),
        ):
            label.setProperty("connected", connected)
            label.style().unpolish(label)
            label.style().polish(label)

    def toggle_stage_connection(self):
        if self.oms.is_connected():
            self.disconnect_stage()
        else:
            self.connect_selected_stage()

    def toggle_camera_connection(self):
        if self.camera is not None and self.camera.is_connected():
            self.disconnect_camera()
        else:
            self.connect_selected_camera()

    def connect_selected_stage(self):
        if self.oms.is_connected():
            return

        device = self.serial_combo.currentData()
        if device is None:
            QMessageBox.warning(self, "No Serial Device", "No serial device is available.")
            return

        port = device["port"]
        if not self.oms.connect(port):
            QMessageBox.critical(self, "Connection Failed", f"Failed to connect to serial device:\n{port}")
            self.connected_stage_label = None
            self.update_connection_state()
            return

        self.connected_stage_label = device["label"]
        self.settings.setValue("Connections/last_stage_id", device["id"])
        self.on_stage_connected()

    def on_stage_connected(self):
        position = self.oms.read_current_position(True)
        if position[0] is not None:
            self.current_pos = list(position)

        self.apply_acceleration_setting()
        for tool_idx, _, _ in self.tool_controls:
            self.apply_tool_setting(tool_idx)
        self.update_connection_state()

    def disconnect_stage(self):
        if self.realtime_control_widget.is_running():
            self.realtime_control_widget.stop_control()

        self.stop_gcode_runner(wait_timeout=15.0)
        self.oms.disconnect()
        self.connected_stage_label = None
        self.current_pos = [0, 0, 0]
        self.update_connection_state()

    def create_camera_from_config(self, config):
        if config["kind"] == "opencv":
            return OpenCVCamera(
                camera_index=config["index"],
                backend=config.get("backend", cv2.CAP_ANY),
                resolution=(1920, 1080)
            )

        if config["kind"] == "basler":
            return BaslerCamera(device_serial=config["id"])

        raise ValueError(f"Unsupported camera kind: {config['kind']}")

    def connect_selected_camera(self):
        if self.camera is not None and self.camera.is_connected():
            return

        config = self.camera_combo.currentData()
        if config is None:
            QMessageBox.warning(self, "No Camera", "No camera is available.")
            return

        try:
            camera = self.create_camera_from_config(config)
        except Exception as exc:
            QMessageBox.critical(self, "Camera Error", str(exc))
            return

        if camera is None or not camera.is_connected():
            if camera is not None:
                camera.close()
            QMessageBox.critical(self, "Camera Error", f"Failed to connect to camera:\n{config['label']}")
            return

        self.disconnect_camera(show_placeholder=False)
        self.camera = camera
        self.connected_camera_label = config["label"]
        self.settings.setValue("Connections/last_camera_id", config["id"])
        self.clear_visual_state()
        self.load_camera_settings(self.connected_camera_label)
        self.apply_camera_settings()
        self.start_camera_stream()
        self.update_connection_state()

    def camera_settings_group(self, camera_name):
        return f"CameraSettings/{camera_name}"

    def load_camera_settings(self, camera_name):
        if not camera_name:
            return

        group = self.camera_settings_group(camera_name)
        for spinbox in (self.exposure_spinbox, self.gain_spinbox, self.white_balance_spinbox):
            spinbox.blockSignals(True)

        try:
            self.exposure_spinbox.setValue(
                self.settings.value(f"{group}/exposure", self.exposure_spinbox.value(), type=float))
            self.gain_spinbox.setValue(
                self.settings.value(f"{group}/gain", self.gain_spinbox.value(), type=float))
            self.white_balance_spinbox.setValue(
                self.settings.value(f"{group}/white_balance", self.white_balance_spinbox.value(), type=float))
        finally:
            for spinbox in (self.exposure_spinbox, self.gain_spinbox, self.white_balance_spinbox):
                spinbox.blockSignals(False)

    def save_camera_settings(self, camera_name):
        if not camera_name:
            return

        group = self.camera_settings_group(camera_name)
        self.settings.setValue(f"{group}/exposure", self.exposure_spinbox.value())
        self.settings.setValue(f"{group}/gain", self.gain_spinbox.value())
        self.settings.setValue(f"{group}/white_balance", self.white_balance_spinbox.value())

    def apply_camera_settings(self):
        if self.camera is None or not self.camera.is_connected():
            return

        try:
            self.camera.set_exposure_time(self.exposure_spinbox.value())
        except Exception:
            pass

        try:
            self.camera.set_gain(self.gain_spinbox.value())
        except Exception:
            pass

        try:
            self.camera.set_white_balance(self.white_balance_spinbox.value())
        except Exception:
            pass

        self.save_camera_settings(self.connected_camera_label)

    def start_camera_stream(self):
        if self.camera is None or not self.camera.is_connected():
            return

        self.stop_camera_stream()
        self.camera_stream_worker = CameraStreamWorker(self.camera, self)
        self.camera_stream_worker.frame_ready.connect(self.on_frame_received)
        self.camera_stream_worker.stream_error.connect(self.on_camera_stream_error)
        self.camera_stream_worker.start()

    def stop_camera_stream(self):
        if self.camera_stream_worker is not None:
            self.camera_stream_worker.stop()
            self.camera_stream_worker = None

    def disconnect_camera(self, show_placeholder=True):
        self.stop_camera_stream()

        if self.camera is not None:
            self.camera.close()
            self.camera = None

        self.connected_camera_label = None
        self.clear_visual_state()
        if show_placeholder:
            self.show_placeholder_frame()
        self.update_connection_state()

    def on_camera_stream_error(self, message):
        self.disconnect_camera()
        QMessageBox.critical(self, "Camera Stream Error", message)

    def on_frame_received(self, frame):
        if frame is None:
            return

        if len(frame.shape) == 2 or (len(frame.shape) == 3 and frame.shape[2] == 1):
            vis_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        else:
            vis_img = frame.copy()

        self.update_controller(frame, vis_img, pixel_per_mm=self.pixel_per_mm)

    def stop_gcode_runner(self, wait_timeout=None):
        runner = self.gcode_runner
        if runner is not None:
            runner.stop()
            if wait_timeout is not None:
                runner.wait(wait_timeout)
        self.set_gcode_button_checked(False)

    def set_gcode_button_checked(self, checked):
        self.run_gcode_button.blockSignals(True)
        self.run_gcode_button.setChecked(checked)
        self.run_gcode_button.blockSignals(False)

    def apply_acceleration_setting(self):
        if self.oms.is_connected():
            self.oms.set_max_acceleration(self.accel_spinbox.value(), 5000)

    def apply_tool_setting(self, tool_idx):
        if not self.oms.is_connected():
            return

        _, spinbox, button = self.tool_controls[tool_idx]
        value = spinbox.value() if button.isChecked() else 0.0
        self.oms.set_tool_output(tool_idx, value, immediate=True)

    def require_stage_connection(self):
        if self.oms.is_connected():
            return True

        QMessageBox.warning(self, "Manipulator Disconnected", "Connect a serial device before using this control.")
        return False

    def require_camera_connection(self):
        if self.camera is not None and self.camera.is_connected():
            return True

        QMessageBox.warning(self, "Camera Disconnected", "Connect a camera before using this control.")
        return False

    def run_fiber_alignment(self):
        if not self.require_stage_connection() or not self.require_camera_connection():
            return

        aligner = OpticalAlignment(self.oms, self.camera)
        pos, _ = aligner.optimize()
        self.camera.start_grabbing(False)
        self.current_pos = pos

    def run_axis_calibration(self, save_result=True):
        if not self.require_stage_connection():
            return

        num_joints = 3
        joint_data = []

        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            for joint_index in range(num_joints):
                res, data = self.oms.calibrate_joint(joint_index, save_result=save_result)
                if data and len(data) >= 3 and len(data[0]) > 0:
                    joint_data.append((joint_index, data))
        except Exception as exc:
            QApplication.restoreOverrideCursor()
            QMessageBox.critical(self, "Calibration Failed", f"An error occurred during calibration:\n{exc}")
            return
        finally:
            QApplication.restoreOverrideCursor()

        if not joint_data:
            QMessageBox.warning(self, "Calibration Failed", "No calibration data was returned by the controller.")
            return

        dialog = CalibrationPlotDialog(joint_data, self)
        dialog.show()

    def on_stop_realtime_control(self):
        if self.oms.is_connected():
            position = self.oms.read_current_position(True)
            if position[0] is not None:
                self.current_pos[:] = position

    def home(self):
        if not self.require_stage_connection():
            return

        self.oms.home()
        position = self.oms.read_current_position(True)
        if position[0] is not None:
            self.current_pos = list(position)

    def move_axis(self, axis, direction):
        if not self.require_stage_connection():
            return

        if self.gcode_runner is not None and self.gcode_runner.running:
            self.statusBar().showMessage("Manual XYZ jog ignored while a G-code program is running", 5000)
            return

        flipped = (1, 1, 1)
        d = self.step_sizes[self.step_size_idx]
        target = list(self.current_pos)
        target[axis] += direction * d * flipped[axis]
        target[axis] = max(min(target[axis], 10), -10)
        status = self.oms.move_to(*target, self.feedrates[self.step_size_idx])
        if status is SerialInterface.ReplyStatus.OK:
            self.current_pos = target
        else:
            self.statusBar().showMessage(f"XYZ move rejected by controller: {status.name}", 5000)

    # -------------------focus axis-----------------
    def build_focus_controls(self):
        frame = QFrame()
        frame.setObjectName("focus_controls_frame")
        frame.setFrameShape(self.device_controls_frame.frameShape())
        frame.setFrameShadow(self.device_controls_frame.frameShadow())
        layout = QVBoxLayout(frame)

        self.focus_mode_label = QLabel(f"Focus Axis (A)  [{self.focus_axis.mode_description()}]")
        self.focus_mode_label.setWordWrap(True)
        if not self.focus_axis.available:
            self.focus_mode_label.setStyleSheet("color: rgb(255, 120, 120); font-weight: bold;")
        elif self.focus_axis.simulated:
            self.focus_mode_label.setStyleSheet("color: rgb(255, 200, 80); font-weight: bold;")
        layout.addWidget(self.focus_mode_label)

        jog_row = QHBoxLayout()
        self.btn_a_minus = QPushButton("A-")
        self.btn_a_plus = QPushButton("A+")
        self.focus_pos_label = QLabel()
        self.focus_pos_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        jog_row.addWidget(self.btn_a_minus)
        jog_row.addWidget(self.focus_pos_label, 1)
        jog_row.addWidget(self.btn_a_plus)
        layout.addLayout(jog_row)

        button_row = QHBoxLayout()
        self.btn_focus_enable = QPushButton("Enable")
        self.btn_focus_enable.setCheckable(True)
        self.btn_focus_zero = QPushButton("Zero A")
        self.btn_focus_stop = QPushButton("Stop A")
        button_row.addWidget(self.btn_focus_enable)
        button_row.addWidget(self.btn_focus_zero)
        button_row.addWidget(self.btn_focus_stop)
        layout.addLayout(button_row)

        self.focus_status_label = QLabel()
        self.focus_status_label.setWordWrap(True)
        layout.addWidget(self.focus_status_label)

        self.btn_a_minus.clicked.connect(lambda checked=False: self.move_focus(-1))
        self.btn_a_plus.clicked.connect(lambda checked=False: self.move_focus(+1))
        self.btn_focus_enable.toggled.connect(self.set_focus_enabled)
        self.btn_focus_zero.clicked.connect(self.zero_focus)
        self.btn_focus_stop.clicked.connect(self.stop_focus)
        self.focus_move_finished.connect(self.on_focus_move_finished)

        self.toolbar_layout.insertWidget(1, frame)

        self.focus_timer = QTimer(self)
        self.focus_timer.timeout.connect(self.update_focus_label)
        self.focus_timer.start(100)
        self.update_focus_label()
        self.set_focus_state(self.focus_state, self.focus_error_message)

        self.gcode_status_label = QLabel("G-code: idle")
        self.gcode_status_label.setWordWrap(True)
        progress_layout = self.gcode_progress_bar.parentWidget().layout()
        index = progress_layout.indexOf(self.gcode_progress_bar)
        if hasattr(progress_layout, "insertWidget") and index >= 0:
            progress_layout.insertWidget(index + 1, self.gcode_status_label)
        else:
            progress_layout.addWidget(self.gcode_status_label)

    def focus_controls_locked(self):
        return self.gcode_runner is not None and self.gcode_runner.running

    def move_focus(self, direction):
        d = self.step_sizes[self.step_size_idx]
        feed = self.feedrates[self.step_size_idx]
        return self.start_focus_move(direction * d, feed)

    def start_focus_move(self, delta, feed):
        if not self.focus_axis.available:
            self.set_focus_state("rejected", f"focus axis unavailable: {self.focus_axis.unavailable_reason}")
            return False
        if self.focus_controls_locked():
            self.set_focus_state("rejected", "manual focus moves are blocked while a G-code program is running")
            return False
        if self.focus_thread is not None and self.focus_thread.is_alive():
            self.set_focus_state("rejected", "focus axis is already moving")
            return False

        start = self.focus_axis.get_position()

        def worker():
            try:
                outcome = self.focus_axis.move_by(delta, feed)
            except Exception as exc:
                outcome = MoveOutcome(MoveResult.FAILED, start + delta, self.focus_axis.get_position(),
                                      f"unexpected error: {exc}")
            self.focus_move_finished.emit(outcome)

        self.set_focus_state("moving", "")
        self.focus_thread = threading.Thread(target=worker, daemon=True)
        self.focus_thread.start()
        return True

    def on_focus_move_finished(self, outcome):
        self.focus_thread = None
        state = outcome.result.value
        message = outcome.message
        if outcome.result is MoveResult.COMPLETED:
            message = message or f"reached commanded A = {outcome.final_position * 1000.0:.1f} µm"
        self.set_focus_state(state, message)

    def set_focus_state(self, state, message):
        self.focus_state = state
        self.focus_error_message = message if state in ("rejected", "failed", "unavailable") else ""
        text = state.upper() + (f": {message}" if message else "")
        self.focus_status_label.setText(text)
        colors = {"failed": "rgb(255, 120, 120)", "rejected": "rgb(255, 170, 90)",
                  "unavailable": "rgb(255, 120, 120)", "cancelled": "rgb(255, 200, 80)"}
        self.focus_status_label.setStyleSheet(f"color: {colors[state]};" if state in colors else "")
        self.update_focus_controls()

    def stop_focus(self):
        self.focus_axis.stop()

    def set_focus_enabled(self, on):
        if not self.focus_axis.set_enabled(on):
            self.set_focus_state("unavailable", str(self.focus_axis.unavailable_reason))
        self.update_focus_label()

    def zero_focus(self):
        if not self.focus_axis.set_zero():
            self.set_focus_state("rejected", "wait for the focus axis to stop before zeroing it")
        self.update_focus_label()

    def update_focus_controls(self):
        available = self.focus_axis.available
        moving = self.focus_state == "moving" or self.focus_axis.is_moving()
        locked = self.focus_controls_locked()
        for button in (self.btn_a_minus, self.btn_a_plus, self.btn_focus_zero):
            button.setEnabled(available and not moving and not locked)
        self.btn_focus_enable.setEnabled(available and not locked)
        self.btn_focus_stop.setEnabled(available)

    def update_focus_label(self):
        pos_um = round(self.focus_axis.get_position() * 1000.0, 1) + 0.0
        state = "  (moving)" if self.focus_axis.is_moving() else ""
        self.focus_pos_label.setText(f"A: {pos_um:.1f} µm commanded{state}")
        self.btn_focus_enable.blockSignals(True)
        self.btn_focus_enable.setChecked(self.focus_axis.enabled)
        self.btn_focus_enable.blockSignals(False)
        self.update_focus_controls()

    def add_waypoint(self):
        if not self.require_stage_connection():
            return

        if self.realtime_control_widget.is_running():
            self.current_pos[:] = self.realtime_control_widget.get_current_pose()

        self.waypoints.append([self.current_pos.copy(), self.feedrates[self.step_size_idx]])
        self.update_waypoint_info()

    def run_path(self):
        if not self.require_stage_connection():
            return

        self.waypoint_idx = 0

    def save_path(self):
        if len(self.waypoints) <= 0:
            QMessageBox.critical(self, "Save Error", "Waypoint list is empty")
            return

        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Path to G-code File",
            "",
            "G-code Files (*.g *.gcode);;Text Files (*.txt);;All Files (*.*)",
        )

        if not path:
            return

        try:
            self.write_waypoint_gcode(path)
        except Exception as exc:
            QMessageBox.critical(self, "Save Error", f"Failed to save G-code file:\n{exc}")

    def waypoint_gcode_text(self):
        lines = ["; Generated by Open Micro Manipulator Controller",
                 "SCALE=1",
                 "G21",
                 "G90",
                 ""]
        for (x, y, z), feedrate in self.waypoints:
            feed_text = format_positive(feedrate * 60.0, "Waypoint feed", '.9g')
            lines.append(f"G0 X{x:.10f} Y{y:.10f} Z{z:.10f} F{feed_text}")
            lines.append("G4 S0.100000")
        lines.append("")
        lines.append("M2")
        return "\n".join(lines) + "\n"

    def write_waypoint_gcode(self, path):
        text = self.waypoint_gcode_text()
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(text)

    def set_origin(self):
        if not self.require_stage_connection():
            return

        position = self.oms.read_current_position(True)
        if position[0] is None:
            QMessageBox.critical(self, "Read Error", "Failed to read current stage position.")
            return

        self.current_pos[:] = position
        transform = self.oms.get_workspace_transform()
        transform[0, 3] += self.current_pos[0]
        transform[1, 3] += self.current_pos[1]
        transform[2, 3] += self.current_pos[2]
        self.oms.set_workspace_transform(transform)
        self.current_pos = [0, 0, 0]
        self.oms.move_to(0, 0, 0, self.feedrates[self.step_size_idx])

    def clear_waypoints(self):
        self.waypoints.clear()
        self.update_waypoint_info()

    def update_waypoint_info(self):
        self.waypoint_info_label.setText(f"Path Control  [ {len(self.waypoints)} Waypoints ]")

    def set_step_size(self, index):
        self.step_size_idx = index

    def update_controller(self, frame, vis_image, pixel_per_mm):
        if self.oms.is_connected() and self.waypoint_idx <= len(self.waypoints) and len(self.waypoints) > 0:
            self.current_pos[:], feedrate = self.waypoints[self.waypoint_idx % len(self.waypoints)]
            self.oms.move_to(*self.current_pos, feedrate)
            self.oms.dwell(0.1, False)
            self.waypoint_idx += 1
        else:
            self.waypoint_idx = 1000000

        prev_pos = self.image_point_tracker.prev_pos
        current_pos = self.image_point_tracker.update(frame)

        if current_pos is not None:
            px, py = current_pos
            cv2.circle(vis_image, (px, py), 4, (255, 0, 0), thickness=-1)

            if self.draw_buffer is None:
                self.draw_buffer = np.zeros_like(vis_image)
            elif prev_pos is not None:
                cv2.line(self.draw_buffer, prev_pos, current_pos, color=(255, 255, 255), thickness=1)

            if self.draw_buffer is not None:
                mask = self.draw_buffer[:, :, 0] != 0
                vis_image[mask, :] = np.array((0, 255, 100), dtype=np.uint8)

        self.video_viewer.set_image(vis_image, pixel_per_mm=pixel_per_mm)
        self.last_frame = frame
        return vis_image

    def set_tracking_point(self):
        if not self.require_camera_connection():
            return

        if self.last_frame is None:
            return

        self.image_point_tracker.set_track_point(
            self.last_frame,
            self.last_frame.shape[1] // 2,
            self.last_frame.shape[0] // 2,
        )

    def clear_draw_buffer(self):
        if self.draw_buffer is not None:
            self.draw_buffer = None
            self.image_point_tracker.reset()

    def run_gcode_from_file(self, checked):
        if not checked:
            if self.gcode_runner is not None:
                self.gcode_runner.stop()
                self.set_gcode_status("stopping: waiting for runner cleanup")
            return

        if not self.require_stage_connection():
            self.set_gcode_button_checked(False)
            return

        path, _ = QFileDialog.getOpenFileName(
            self,
            "Open G-code File",
            "",
            "G-code Files (*.g *.gcode);;Text Files (*.txt);;All Files (*.*)",
        )

        if not path:
            self.set_gcode_button_checked(False)
            return

        try:
            with open(path, "r", encoding="utf-8") as handle:
                gcode = handle.read()
        except Exception as exc:
            QMessageBox.critical(self, "Error", f"Failed to open file:\n{exc}")
            self.set_gcode_button_checked(False)
            return

        error = self.start_gcode(gcode)
        if error:
            QMessageBox.critical(self, "G-code Rejected", error)

    def start_gcode(self, gcode):
        if self.gcode_runner is not None and self.gcode_runner.running:
            return "A G-code program is already running"
        if self.focus_thread is not None and self.focus_thread.is_alive():
            self.set_gcode_button_checked(False)
            return "Wait for the manual focus move to finish before starting a program"
        if self.realtime_control_widget.is_running():
            self.set_gcode_button_checked(False)
            return "Stop realtime mouse control before starting a program"
        try:
            runner = GCodeRunner(gcode, self.oms, max_feedrate=0.5, focus_axis=self.focus_axis)
        except GCodeError as exc:
            self.gcode_execution_state = ExecutionState.FAILED
            self.gcode_error_message = str(exc)
            self.set_gcode_status(f"rejected before start: {exc}")
            self.set_gcode_button_checked(False)
            return f"The program was not started because it contains unsupported or malformed G-code:\n{exc}"

        self.gcode_runner = runner
        runner.progress_updated.connect(self.on_gcode_progress)
        runner.finished.connect(self.on_gcode_finished)
        try:
            runner.run(loop_playback=False, tool_power=self.tool1_spinbox.value())
        except Exception as exc:
            self.gcode_runner = None
            self.gcode_execution_state = ExecutionState.FAILED
            self.gcode_error_message = str(exc)
            self.set_gcode_status(f"failed to start: {exc}")
            self.set_gcode_button_checked(False)
            return str(exc)

        self.gcode_execution_state = ExecutionState.RUNNING
        self.gcode_error_message = ""
        self.gcode_progress_bar.setValue(0)
        self.set_gcode_button_checked(True)
        self.set_gcode_status("running")
        self.update_focus_controls()
        return None

    def on_gcode_progress(self, fraction):
        self.gcode_progress_bar.setValue(int(fraction * 100))

    def on_gcode_finished(self, result: RunResult):
        if self.sender() is not None and self.sender() is not self.gcode_runner:
            return
        self.gcode_runner = None
        self.gcode_last_result = result
        self.gcode_execution_state = result.state
        self.gcode_error_message = result.message if result.state is ExecutionState.FAILED else ""
        details = [f"{result.state.value}: {result.message}"]
        details += [f"warning: {w}" for w in result.warnings]
        details += [f"cleanup: {c}" for c in result.cleanup]
        self.set_gcode_status("\n".join(details))
        self.set_gcode_button_checked(False)
        self.update_focus_controls()

    def set_gcode_status(self, text):
        self.gcode_status_label.setText(f"G-code: {text}")
        self.statusBar().showMessage(f"G-code: {text.splitlines()[0]}", 10000)

    def run_3point_alignment(self):
        if not self.require_stage_connection():
            return

        if len(self.waypoints) != 3:
            QMessageBox.critical(self, "Error", "Exactly 3 Waypoints need to be recorded for this function")
            return

        old_workspace_transform = self.oms.get_workspace_transform()

        p0 = np.array(self.waypoints[0][0])
        p1 = np.array(self.waypoints[1][0])
        p2 = np.array(self.waypoints[2][0])

        v1 = p1 - p0
        v2 = p2 - p0
        z_axis = np.cross(v1, v2)
        if z_axis[2] < 0.0:
            z_axis = -z_axis
        z_axis /= np.linalg.norm(z_axis)

        global_x = np.array([1.0, 0.0, 0.0])
        x_proj = global_x - np.dot(global_x, z_axis) * z_axis
        x_axis = x_proj / np.linalg.norm(x_proj)

        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        transform = np.eye(4)
        transform[:3, 0] = x_axis
        transform[:3, 1] = y_axis
        transform[:3, 2] = z_axis
        transform[:3, 3] = p0

        self.oms.set_workspace_transform(transform @ old_workspace_transform)
        QMessageBox.information(self, "Alignment Complete", "3-point alignment complete.")

        self.oms.move_to(0, 0, 0, self.feedrates[self.step_size_idx])
        self.current_pos = [0, 0, 0]

    def load_transform(self):
        if not self.require_stage_connection():
            return

        try:
            with open("transform.json", "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except Exception as exc:
            QMessageBox.critical(self, "Load Error", f"Failed to load transform:\n{exc}")
            return

        transform = np.array(data)
        self.oms.set_workspace_transform(transform)
        self.oms.move_to(0, 0, 0, self.feedrates[self.step_size_idx])
        self.current_pos = [0, 0, 0]

    def save_transform(self, pressed=True, ask=True):
        if ask:
            confirmed = QMessageBox.question(
                None,
                "Save Transform",
                "Are you sure?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            ) == QMessageBox.StandardButton.Yes
            if not confirmed:
                return

        try:
            transform = self.oms.get_workspace_transform()
            with open("transform.json", "w", encoding="utf-8") as handle:
                json.dump(transform.tolist(), handle)
        except Exception as exc:
            QMessageBox.critical(self, "Save Error", f"Failed to save transform:\n{exc}")

    def closeEvent(self, event: QCloseEvent):
        if self.realtime_control_widget.is_running():
            self.realtime_control_widget.stop_control()

        self.stop_gcode_runner(wait_timeout=15.0)
        self.focus_axis.stop()
        if self.focus_thread is not None:
            self.focus_thread.join(timeout=2.0)
        self.focus_timer.stop()
        self.focus_axis.close()
        self.disconnect_camera()
        self.disconnect_stage()
        super().closeEvent(event)

    def capture_dark_image(self):
        if not self.require_camera_connection():
            return
        self.camera.capture_dark_image()

    def save_screenshot(self):
        if self.last_frame is None:
            return

        os.makedirs("./screenshots", exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"screenshot_{timestamp}.png"
        path = os.path.join("./screenshots", filename)
        img = cv2.cvtColor(self.last_frame, cv2.COLOR_RGB2BGR)
        cv2.imwrite(path, img)

    def run_spacekey_command(self):
        self.realtime_control_widget.stop_control()
        pos = self.oms.read_current_position(True)

        mode = 1
        if mode == -1:
            self.oms.set_tool_output(0, self.tool1_spinbox.value(), False)
            self.oms.dwell(0.1, False)
            self.oms.set_tool_output(0, 0.00, False)
            self.oms.dwell(0.1, False)
            return
        if mode == 1:
            num_cycles = 1
            on_step = 0.05
            off_step = 0.00
            accel_step = 0.001
            decel_step = 0.001
            feed = 0.05

            y = pos[1] + accel_step
            self.oms.move_to(pos[0], y, pos[2], feed)
            for _ in range(num_cycles):
                self.oms.set_tool_output(0, self.tool1_spinbox.value(), False)
                y += on_step
                self.oms.move_to(pos[0], y, pos[2], feed)
                self.oms.set_tool_output(0, 0.00, False)
                y += off_step
                self.oms.move_to(pos[0], y, pos[2], feed)
            y += decel_step
            self.oms.move_to(pos[0], y, pos[2], feed)
            self.oms.move_to(pos[0], pos[1], pos[2]-0.001, 10)
            self.oms.dwell(0.1, True)
        if mode == 2:
            side = 0.05
            feed = 0.1

            self.oms.set_tool_output(0, self.tool1_spinbox.value(), False)
            self.oms.move_to(pos[0]-side, pos[1], pos[2], feed)
            self.oms.move_to(pos[0]-side, pos[1]+side, pos[2], feed)
            self.oms.move_to(pos[0], pos[1]+side, pos[2], feed)
            self.oms.move_to(pos[0], pos[1], pos[2], feed)
            self.oms.move_to(pos[0], pos[1], pos[2]-0.003, feed)
            self.oms.set_tool_output(0, 0.0, False)
            self.oms.dwell(0.1, True)
        if mode == 3:
            num_lines = 10
            spacing = 0.01
            length = 0.3
            feed = 0.1
            up = pos[2] - 0.01

            self.oms.set_tool_output(0, self.tool1_spinbox.value(), False)
            for i in range(num_lines):
                x = pos[0] + i * spacing
                if i > 0:
                    self.oms.move_to(x, pos[1], up, feed*10)
                self.oms.move_to(x, pos[1], pos[2], feed*10)
                self.oms.set_tool_output(0, self.tool1_spinbox.value(), False)
                self.oms.move_to(x, pos[1] + length, pos[2], feed)
                self.oms.set_tool_output(0, 0.0, False)

            self.oms.move_to(pos[0], pos[1], pos[2], feed)
            self.oms.move_to(pos[0], pos[1], pos[2]+0.003, feed)
            self.oms.set_tool_output(0, 0.0, False)
            self.oms.dwell(0.1, True)
        if mode == 4:
            on_time_s = self.tool1_spinbox.value()
            self.oms.set_tool_output(0, 1.0, False)
            self.oms.dwell(on_time_s, True)
            self.oms.set_tool_output(0, 0.0, False)
            self.oms.dwell(0.5, True)

        pass

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_A:
            self.move_axis(0, -1)
        elif event.key() == Qt.Key.Key_D:
            self.move_axis(0, +1)
        elif event.key() == Qt.Key.Key_W:
            self.move_axis(1, -1)
        elif event.key() == Qt.Key.Key_S:
            self.move_axis(1, +1)
        elif event.key() == Qt.Key.Key_R:
            self.move_axis(2, +1)
        elif event.key() == Qt.Key.Key_F:
            self.move_axis(2, -1)
        elif event.key() == Qt.Key.Key_E:
            self.move_focus(+1)
        elif event.key() == Qt.Key.Key_Q:
            self.move_focus(-1)
        elif event.key() == Qt.Key.Key_K:
            self.add_waypoint()
        elif event.key() == Qt.Key.Key_P:
            self.save_screenshot()
        elif event.key() == Qt.Key.Key_F11:
            self.toolbar_widget.setVisible(not self.toolbar_widget.isVisible())
        elif event.key() == Qt.Key.Key_Space:
            self.run_spacekey_command()
        else:
            super().keyPressEvent(event)
