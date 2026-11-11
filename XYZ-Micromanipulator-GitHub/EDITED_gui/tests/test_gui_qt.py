import pytest
from PySide6.QtCore import QSettings, Qt
from PySide6.QtWidgets import QMessageBox

import mainwindow as mw
from gcode_program import GCodeProgram
from gcode_runner import ExecutionState
from gui_components import realtime_controller_widget as rtw
from hardware.focus_axis import FocusAxis, FocusAxisConfig, SimulatedBackend


class ExplodingBackend(SimulatedBackend):
    def write(self, pin, level):
        if pin == FocusAxisConfig().step_pin:
            raise OSError("driver fault")
        super().write(pin, level)


def slow_focus(feed=0.01, backend=None):
    cfg = FocusAxisConfig(steps_per_mm=1000.0, max_feed=feed)
    if backend is not None:
        return FocusAxis(config=cfg, backend=backend)
    return FocusAxis(config=cfg, mode="simulate")


@pytest.fixture
def make_window(qtbot, monkeypatch, tmp_path, oms):
    QSettings.setDefaultFormat(QSettings.Format.IniFormat)
    QSettings.setPath(QSettings.Format.IniFormat, QSettings.Scope.UserScope, str(tmp_path))
    monkeypatch.setattr(mw, "list_serial_devices", lambda: [])
    monkeypatch.setattr(mw, "list_camera_devices", lambda: [])
    monkeypatch.setattr(rtw.RealtimeControllerWidget, "constrain_cursor", lambda self: None)
    monkeypatch.setattr(QMessageBox, "critical", lambda *a, **k: QMessageBox.StandardButton.Ok)
    monkeypatch.setattr(QMessageBox, "warning", lambda *a, **k: QMessageBox.StandardButton.Ok)
    windows = []

    def factory(focus_axis=None):
        window = mw.DeviceControlMainWindow(oms, focus_axis=focus_axis or slow_focus(feed=50.0))
        qtbot.addWidget(window)
        windows.append(window)
        return window

    yield factory
    for window in windows:
        if window.isVisible() or window.gcode_runner is not None:
            window.close()


# -------------------realtime-----------------
def test_realtime_off_stops_worker_and_submissions(qtbot, make_window, controller):
    window = make_window()
    widget = window.realtime_control_widget
    widget.mouse_control_button.setChecked(True)
    assert widget.is_running()
    worker = widget.update_thread
    qtbot.waitUntil(lambda: any(c.startswith("G24") for c in controller.commands), timeout=3000)

    widget.mouse_control_button.setChecked(False)
    assert not widget.is_running()
    assert worker.isFinished()
    count = sum(c.startswith("G24") for c in controller.commands)
    qtbot.wait(150)
    assert sum(c.startswith("G24") for c in controller.commands) == count


def test_realtime_blocked_while_program_runs(qtbot, make_window, controller):
    window = make_window(slow_focus(feed=0.01))
    assert window.start_gcode("SCALE=1\nG1 A1 F0.6\n") is None
    qtbot.waitUntil(window.focus_axis.is_moving, timeout=3000)
    window.realtime_control_widget.mouse_control_button.setChecked(True)
    assert not window.realtime_control_widget.is_running()
    window.stop_gcode_runner()
    qtbot.waitUntil(lambda: window.gcode_execution_state is ExecutionState.CANCELLED, timeout=5000)


# -------------------focus manual moves-----------------
def test_stop_button_cancels_manual_focus_move(qtbot, make_window):
    window = make_window(slow_focus(feed=0.01))
    window.set_step_size(0)
    assert window.move_focus(+1)
    qtbot.waitUntil(window.focus_axis.is_moving, timeout=2000)
    assert not window.btn_a_plus.isEnabled()
    qtbot.mouseClick(window.btn_focus_stop, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: window.focus_state == "cancelled", timeout=2000)
    assert "CANCELLED" in window.focus_status_label.text()
    assert window.btn_a_plus.isEnabled()
    assert 0 < window.focus_axis.get_position() < 1.0


def test_focus_worker_failure_visible_in_gui(qtbot, make_window):
    window = make_window(slow_focus(feed=50.0, backend=ExplodingBackend()))
    assert window.move_focus(+1)
    qtbot.waitUntil(lambda: window.focus_state == "failed", timeout=2000)
    assert "driver fault" in window.focus_error_message
    assert "FAILED" in window.focus_status_label.text()
    assert not window.focus_axis.is_moving()


def test_out_of_range_focus_jog_rejected_in_gui(qtbot, make_window):
    window = make_window(slow_focus(feed=50.0))
    window.set_step_size(0)
    window.focus_axis.steps = int(9.5 * 1000)
    window.move_focus(+1)
    qtbot.waitUntil(lambda: window.focus_state == "rejected", timeout=2000)
    assert "outside" in window.focus_error_message
    assert window.focus_axis.get_position() == pytest.approx(9.5)


def test_unavailable_focus_axis_disables_controls(qtbot, make_window):
    axis = FocusAxis(mode="gpio")
    if axis.available:
        pytest.skip("GPIO present")
    window = make_window(axis)
    assert window.focus_state == "unavailable"
    assert not window.btn_a_plus.isEnabled()
    assert "UNAVAILABLE" in window.focus_mode_label.text()
    assert window.move_focus(+1) is False


# -------------------program execution-----------------
def test_cancelled_focus_move_prevents_later_xyz(qtbot, make_window, controller):
    window = make_window(slow_focus(feed=0.01))
    assert window.start_gcode("SCALE=1\nG1 X0.1 F60\nG1 A1 F0.6\nG1 X0.9 F60\n") is None
    assert window.run_gcode_button.isChecked()
    qtbot.waitUntil(window.focus_axis.is_moving, timeout=3000)
    assert not window.btn_a_plus.isEnabled()
    assert window.move_focus(+1) is False
    window.run_gcode_button.click()
    qtbot.waitUntil(lambda: window.gcode_execution_state is ExecutionState.CANCELLED, timeout=5000)
    assert not any("X0.900000" in c for c in controller.commands)
    assert not window.run_gcode_button.isChecked()
    assert window.btn_a_plus.isEnabled()
    assert "cancelled" in window.gcode_status_label.text()


def test_program_failure_reaches_gui(qtbot, make_window, controller):
    controller.reject_prefixes = {"G0 X0.200000": "target pose unreachable"}
    window = make_window()
    assert window.start_gcode("SCALE=1\nG1 X0.1 F60\nG1 X0.2\n") is None
    qtbot.waitUntil(lambda: window.gcode_execution_state is ExecutionState.FAILED, timeout=5000)
    assert "line 3" in window.gcode_error_message
    assert "failed" in window.gcode_status_label.text()
    assert not window.run_gcode_button.isChecked()
    assert window.gcode_runner is None


def test_invalid_program_rejected_before_start(qtbot, make_window, controller):
    window = make_window()
    before = list(controller.commands)
    error = window.start_gcode("G21\nG2 X1 Y1 I1 J0\n")
    assert error and "arc" in error
    assert window.gcode_execution_state is ExecutionState.FAILED
    assert controller.commands == before
    assert not window.run_gcode_button.isChecked()


def test_completed_program_updates_controls(qtbot, make_window, controller):
    window = make_window()
    assert window.start_gcode("SCALE=1\nG1 X0.1 F60\nG1 A0.01 F600\nM2\n") is None
    qtbot.waitUntil(lambda: window.gcode_execution_state is ExecutionState.COMPLETED, timeout=5000)
    assert window.gcode_progress_bar.value() == 100
    assert window.gcode_last_result.tool_off_confirmed
    assert window.focus_axis.get_position() == pytest.approx(0.01)
    assert window.btn_a_plus.isEnabled()


def test_close_with_active_workers(qtbot, make_window, controller):
    window = make_window(slow_focus(feed=0.01))
    window.show()
    assert window.start_gcode("SCALE=1\nG1 A1 F0.6\n") is None
    qtbot.waitUntil(window.focus_axis.is_moving, timeout=3000)
    runner = window.gcode_runner
    axis = window.focus_axis
    window.close()
    assert runner.worker_thread is not None and not runner.worker_thread.is_alive()
    assert runner.result.state is ExecutionState.CANCELLED
    assert not axis.available
    assert not window.oms.is_connected()


# -------------------waypoints-----------------
def test_waypoint_save_reopen_preserves_coordinates(qtbot, make_window, controller, tmp_path):
    window = make_window()
    points = [([0.0123, -0.5, 1.25], 0.5), ([-2.000001, 3.3, 0.0], 0.05)]
    window.waypoints = [[list(p), f] for p, f in points]
    path = tmp_path / "path.gcode"
    window.write_waypoint_gcode(str(path))

    program = GCodeProgram(path.read_text(encoding="utf-8"))
    resolved = [r for _, r in program.iter_resolved_moves(initial_scale=0.001)]
    for (xyz, feed), r in zip(points, resolved):
        assert [r["X"], r["Y"], r["Z"]] == pytest.approx(xyz, abs=1e-9)
        assert r["F"] / 60.0 == pytest.approx(feed)

    assert window.start_gcode(path.read_text(encoding="utf-8")) is None
    qtbot.waitUntil(lambda: window.gcode_execution_state is ExecutionState.COMPLETED, timeout=5000)
    sent = [c for c in controller.commands if c.startswith("G0")]
    assert sent[0].startswith("G0 X0.012300 Y-0.500000 Z1.250000")
    assert sent[1].startswith("G0 X-2.000001 Y3.300000 Z0.000000")


def test_legacy_waypoint_file_without_scale_is_micrometres():
    legacy = "G90 ; absolute positioning\nG0 X1.0000000000 Y0 Z0 F30.000\n"
    resolved = [r for _, r in GCodeProgram(legacy).iter_resolved_moves(initial_scale=0.001)]
    assert resolved[0]["X"] == pytest.approx(0.001)
