import threading
import time

import pytest

from fake_controller import wait_for
from gcode_program import GCodeError
from gcode_runner import ExecutionState, GCodeRunner
from hardware.focus_axis import FocusAxis, FocusAxisConfig, SimulatedBackend


class RecordingBackend(SimulatedBackend):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.violations = []
        self.steps = 0

    def write(self, pin, level):
        if pin == FocusAxisConfig().step_pin and level == 1:
            self.steps += 1
            c = self.controller
            with c.lock:
                if c.queue or c.active is not None or any(c.physical_tools):
                    self.violations.append((list(c.physical_tools), len(c.queue), c.active))
        super().write(pin, level)


def run_to_end(runner, timeout=10.0):
    runner.run(tool_power=0.5)
    assert runner.wait(timeout)
    return runner.result


def moves(controller):
    return [c for c in controller.commands if c.startswith("G0")]


# -------------------rejection and status-----------------
def test_rejected_move_fails_and_stops(controller, oms):
    controller.reject_prefixes = {"G0 X0.200000": "target pose unreachable"}
    runner = GCodeRunner("SCALE=1\nG1 X0.1 F60\nG1 X0.2\nG1 X0.3\n", oms, max_feedrate=0.5)
    result = run_to_end(runner)
    assert result.state is ExecutionState.FAILED
    assert result.line_no == 3
    assert "ERROR" in result.message
    assert not any("X0.300000" in m for m in moves(controller))


def test_invalid_program_rejected_before_any_motion(controller, oms):
    before = list(controller.commands)
    with pytest.raises(GCodeError):
        GCodeRunner("SCALE=1\nG1 X0.1 F60\nG2 X1 Y1 I1 J0\n", oms, max_feedrate=0.5)
    assert controller.commands == before


def test_completed_program_scales_and_reports(controller, oms):
    runner = GCodeRunner("SCALE=1\nG21\nG90\nG1 X0.1 Y0.2 Z0.3 F60\nG4 P100\nM2\n", oms, max_feedrate=0.5)
    result = run_to_end(runner)
    assert result.state is ExecutionState.COMPLETED
    assert moves(controller)[0] == "G0 X0.100000 Y0.200000 Z0.300000 F0.5"
    assert "G4 S0.100000" in controller.commands
    assert result.tool_off_confirmed


def test_default_scale_is_micrometres(controller, oms):
    run_to_end(GCodeRunner("G1 X100 F60000\n", oms, max_feedrate=0.5))
    assert moves(controller)[0].startswith("G0 X0.100000")


def test_feed_clamped_to_max(controller, oms):
    run_to_end(GCodeRunner("SCALE=1\nG1 X0.1 F600\n", oms, max_feedrate=0.5))
    assert moves(controller)[0].endswith("F0.5")


def test_missing_feed_fails(controller, oms):
    result = run_to_end(GCodeRunner("SCALE=1\nG1 X0.1\n", oms, max_feedrate=0.5))
    assert result.state is ExecutionState.FAILED
    assert "feed" in result.message


def test_m5_turns_off_selected_tool(controller, oms):
    run_to_end(GCodeRunner("SCALE=1\nM3 T1 S0.4\nG1 X0.1 F60\nM5 T1\nG1 X0.2\n", oms, max_feedrate=0.5))
    assert "M3 T1 S0.4000" in controller.commands
    assert "M3 T1 S0.0000" in controller.commands


def test_comment_axis_not_commanded(controller, oms):
    run_to_end(GCodeRunner("SCALE=1\nG1 X0.1 F30 ; Y99\n", oms, max_feedrate=0.5))
    assert moves(controller)[0] == "G0 X0.100000 Y0.000000 Z0.000000 F0.5"


# -------------------focus sequencing-----------------
def make_focus(controller, feed=50.0):
    backend = RecordingBackend(controller)
    axis = FocusAxis(config=FocusAxisConfig(steps_per_mm=1000.0, max_feed=feed), backend=backend)
    return axis, backend


def test_focus_move_waits_for_idle_stage_with_tool_off(controller, oms):
    axis, backend = make_focus(controller)
    program = ("SCALE=1\nM3 S0.5\nG1 X0.1 F60\nG1 X0.2\nG1 X0.3\n"
               "G1 A0.01 F600\nG1 X0.4\nM3 S0\nM2\n")
    result = run_to_end(GCodeRunner(program, oms, max_feedrate=50.0, focus_axis=axis))
    assert result.state is ExecutionState.COMPLETED
    assert backend.steps == 10
    assert backend.violations == []
    idx_focus = controller.commands.index("M53")
    last_move_before = max(i for i, c in enumerate(controller.commands[:idx_focus]) if c.startswith("G0"))
    assert last_move_before < idx_focus
    restore = [i for i, c in enumerate(controller.commands) if c == "M3 T0 S0.5000"]
    assert len(restore) == 2
    assert axis.get_position() == pytest.approx(0.01)


def test_focus_move_without_axis_fails(controller, oms):
    result = run_to_end(GCodeRunner("SCALE=1\nG1 A0.01 F60\n", oms, max_feedrate=0.5))
    assert result.state is ExecutionState.FAILED
    assert "no focus axis" in result.message


def test_focus_limit_rejection_fails_program(controller, oms):
    axis, _ = make_focus(controller)
    program = "SCALE=1\nG1 A20 F60\nG1 X0.5 F60\n"
    result = run_to_end(GCodeRunner(program, oms, max_feedrate=0.5, focus_axis=axis))
    assert result.state is ExecutionState.FAILED
    assert "outside" in result.message
    assert moves(controller) == []


def test_cancel_during_focus_move_prevents_later_xyz(controller, oms):
    axis, _ = make_focus(controller, feed=0.01)
    program = "SCALE=1\nG1 X0.1 F60\nG1 A1 F0.6\nG1 X0.9 F60\n"
    runner = GCodeRunner(program, oms, max_feedrate=0.5, focus_axis=axis)
    runner.run()
    assert wait_for(axis.is_moving, 5)
    runner.stop()
    assert runner.wait(5)
    assert runner.result.state is ExecutionState.CANCELLED
    assert not any("X0.900000" in m for m in moves(controller))
    assert 0 < axis.get_position() < 1


def test_cleanup_attempts_every_action_and_reports(controller, oms):
    controller.reject_prefixes = {"M3 T0 S0.0000": "simulated failure", "G0 X0.200000": "unreachable"}
    runner = GCodeRunner("SCALE=1\nG1 X0.1 F60\nG1 X0.2\n", oms, max_feedrate=0.5)
    result = run_to_end(runner)
    assert result.state is ExecutionState.FAILED
    assert "M3 T1 S0.0000" in controller.commands
    assert any("tool-off NOT confirmed" in line for line in result.cleanup)
    assert not result.tool_off_confirmed


def test_cleanup_after_disconnect_does_not_claim_off(controller, oms):
    controller.segments_per_poll = 0
    runner = GCodeRunner("SCALE=1\nG1 X0.1 F60\n", oms, max_feedrate=0.5, cleanup_timeout=0.2)
    runner.run()
    assert wait_for(lambda: len(moves(controller)) == 1)
    oms.disconnect()
    runner.stop()
    assert runner.wait(5)
    assert not runner.result.tool_off_confirmed
    assert any("unavailable" in c or "NOT confirmed" in c for c in runner.result.cleanup)
    assert not any("confirmed off" in c for c in runner.result.cleanup)


def test_end_to_end_stl_to_controller_with_focus_layers(controller, oms):
    import os
    import sys
    slicer_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "NEW_slicer"))
    sys.path.insert(0, slicer_dir)
    import pbm_slicer as ps
    from slicer_shapes import ring_triangles

    settings = ps.Settings(layer_height=0.01, hatch_spacing=0.01, layer_axis="A", decimals=6,
                           write_speed=600.0, travel_speed=600.0, max_speed=600.0)
    lines, stats = ps.generate_gcode(ring_triangles(), settings)
    axis, backend = make_focus(controller, feed=50.0)
    runner = GCodeRunner("\n".join(lines), oms, max_feedrate=50.0, focus_axis=axis)
    result = run_to_end(runner, timeout=60)
    assert result.state is ExecutionState.COMPLETED, result.message
    assert backend.violations == []
    assert len(moves(controller)) == stats.write_moves + stats.travel_moves
    assert axis.get_position() == pytest.approx(0.025)
    assert controller.physical_tools == [0.0, 0.0]
    assert result.tool_off_confirmed


def test_diagnose_gcode_uses_parsed_program(controller, oms):
    runner = GCodeRunner("SCALE=1\nG1 X0.001 F60\nG1 X1 F6000\n", oms, max_feedrate=0.5)
    issues = runner.diagnose_gcode(min_length=0.01, min_feedrate=50)
    assert [i["line"] for i in issues] == [2]
