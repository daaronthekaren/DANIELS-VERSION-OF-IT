# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import math
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from PySide6.QtCore import QObject, Signal

from gcode_program import GCodeProgram, UnitResolver
from hardware.focus_axis import MoveResult
from hardware.open_micro_stage_api import (
    MotionCommandError,
    OpenMicroStageInterface,
    OperationCancelled,
    SerialInterface,
    StageIdleStatus,
)


# -------------------execution state-----------------
class ExecutionState(Enum):
    IDLE = 'idle'
    RUNNING = 'running'
    COMPLETED = 'completed'
    CANCELLED = 'cancelled'
    FAILED = 'failed'


@dataclass
class RunResult:
    state: ExecutionState
    message: str
    line_no: Optional[int] = None
    warnings: list = field(default_factory=list)
    cleanup: list = field(default_factory=list)
    tool_off_confirmed: bool = False


OK = SerialInterface.ReplyStatus.OK


# -------------------runner-----------------
class GCodeRunner(QObject):
    progress_updated = Signal(float)
    state_changed = Signal(object)
    finished = Signal(object)

    def __init__(self, gcode, oms: OpenMicroStageInterface, max_feedrate, scale=0.001, focus_axis=None,
                 command_timeout=30.0, idle_timeout=60.0, cleanup_timeout=10.0):
        super().__init__()
        self.program = gcode if isinstance(gcode, GCodeProgram) else GCodeProgram(gcode)
        self.lines = self.program.lines
        self.oms = oms
        self.focus_axis = focus_axis
        self.initial_scale = scale
        self.max_feedrate = max_feedrate
        self.command_timeout = command_timeout
        self.idle_timeout = idle_timeout
        self.cleanup_timeout = cleanup_timeout

        self.current_line_idx = 0
        self.state = ExecutionState.IDLE
        self.result = None
        self.worker_thread = None
        self.on_finished = None
        self.on_iteration_finished = None
        self.tool_power = 1.0
        self.tool_nr = 0
        self._cancel = threading.Event()
        self._reset_machine_state()

    def _reset_machine_state(self):
        self.position = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'A': None}
        self.feed = None
        self.units = UnitResolver(self.initial_scale)
        self.extrude_relative = False
        self.last_e = 0.0
        self.tool_values = [0.0, 0.0]
        self.tool_on = False
        self.warnings = []

    @property
    def running(self):
        return self.state is ExecutionState.RUNNING

    def is_cancel_requested(self):
        return self._cancel.is_set()

    # -------------------control-----------------
    def run(self, on_finished=None, on_iteration_finished=None, loop_playback=False, tool_power=0.0):
        if self.state is ExecutionState.RUNNING:
            raise RuntimeError("GCodeRunner is already running")
        if not self.oms.is_connected():
            raise MotionCommandError("Stage is not connected")
        if not 0.0 <= tool_power <= 1.0:
            raise ValueError("tool_power must be within [0, 1]")
        self.on_finished = on_finished
        self.on_iteration_finished = on_iteration_finished
        self.tool_power = tool_power
        self._cancel.clear()
        self._set_state(ExecutionState.RUNNING)
        self.worker_thread = threading.Thread(target=self._worker, args=(loop_playback,), daemon=True)
        self.worker_thread.start()

    def stop(self):
        self._cancel.set()
        if self.focus_axis is not None:
            self.focus_axis.stop()

    def wait(self, timeout=None):
        if self.worker_thread is not None:
            self.worker_thread.join(timeout)
            return not self.worker_thread.is_alive()
        return True

    def _set_state(self, state):
        self.state = state
        self.state_changed.emit(state)

    # -------------------worker-----------------
    def _worker(self, loop_playback):
        result = None
        current_line = None
        try:
            while True:
                self._reset_machine_state()
                for idx, line in enumerate(self.lines):
                    if self._cancel.is_set():
                        raise OperationCancelled("Stopped by user")
                    current_line = line
                    self.current_line_idx = idx
                    ended = self._execute(line)
                    self.progress_updated.emit((idx + 1) / max(len(self.lines), 1))
                    if ended:
                        break
                self._wait_for_motion_end()
                if self.on_iteration_finished:
                    self.on_iteration_finished()
                if not loop_playback or self._cancel.is_set():
                    break
            result = RunResult(ExecutionState.COMPLETED,
                               "Program finished; controller reported all queued motion complete")
        except OperationCancelled as exc:
            result = RunResult(ExecutionState.CANCELLED, str(exc) or "Cancelled",
                               current_line.line_no if current_line else None)
        except Exception as exc:
            line_no = current_line.line_no if current_line else None
            prefix = f"line {line_no}: " if line_no else ""
            result = RunResult(ExecutionState.FAILED, f"{prefix}{exc}", line_no)
        finally:
            if result is None:
                result = RunResult(ExecutionState.FAILED, "Runner stopped unexpectedly")
            result.warnings = list(self.warnings)
            self._cleanup(result)
            self.result = result
            self._set_state(result.state)
            self.finished.emit(result)
            if self.on_finished:
                self.on_finished()

    def _wait_for_motion_end(self):
        status = self.oms.wait_for_stop(cancel_event=self._cancel)
        if status is not OK:
            raise MotionCommandError(f"Could not confirm end of motion: {status.name}")

    def _cleanup(self, result):
        report = []
        if self.focus_axis is not None and self.focus_axis.is_moving():
            self.focus_axis.stop()
            report.append("focus axis stop requested")

        if not self.oms.is_connected():
            report.append("serial connection unavailable: tool output state could not be commanded or confirmed")
            result.cleanup = report
            return

        failures = 0
        for tool_idx in range(len(self.tool_values)):
            try:
                status = self.oms.set_tool_output(tool_idx, 0.0, immediate=False)
                report.append(f"tool {tool_idx} off command: {status.name}")
                failures += status is not OK
            except Exception as exc:
                failures += 1
                report.append(f"tool {tool_idx} off command failed: {exc}")
        try:
            status = self.oms.dwell(0.001, blocking=True, timeout=self.cleanup_timeout)
            report.append(f"dwell carrying tool-off state: {status.name}")
            failures += status is not OK
        except Exception as exc:
            failures += 1
            report.append(f"dwell carrying tool-off state failed: {exc}")

        if failures:
            report.append("tool-off NOT confirmed")
            result.cleanup = report
            return

        try:
            status = self.oms.wait_for_stop(timeout=self.cleanup_timeout)
        except Exception as exc:
            status = None
            report.append(f"idle query failed: {exc}")
        if status is OK:
            result.tool_off_confirmed = self.oms.firmware_fork_revision >= 1
            if result.tool_off_confirmed:
                report.append("controller idle after tool-off segment: tool outputs confirmed off")
            else:
                report.append("controller reported idle after tool-off segment; firmware does not "
                              "check the active segment, so tool-off is not confirmed")
        elif status is not None:
            report.append(f"controller did not report idle within {self.cleanup_timeout} s "
                          f"({status.name}); queued motion may still be running and tool-off "
                          "applies only after it")
        result.cleanup = report

    # -------------------line execution-----------------
    def _execute(self, line):
        kind = line.kind
        if kind in ('scale', 'modal'):
            self.units.apply(line)
            return False
        if kind == 'end':
            return True
        if kind == 'extrude_mode':
            self.extrude_relative = line.params['relative']
            return False
        if kind == 'set_e':
            self.last_e = line.params['E']
            return False
        if kind == 'tool':
            self._set_tool(line.params['tool'], line.params['value'])
            return False
        if kind == 'dwell':
            self._check(self.oms.dwell(line.params['seconds'], blocking=True, timeout=self.command_timeout),
                        "dwell")
            return False
        if kind == 'move':
            self._move(line)
            return False
        raise MotionCommandError(f"internal: unhandled line kind {kind}")

    def _check(self, status, what):
        if status is not OK:
            name = status.name if isinstance(status, Enum) else repr(status)
            raise MotionCommandError(f"{what} rejected by controller: {name}")

    def _set_tool(self, tool_idx, value):
        self._check(self.oms.set_tool_output(tool_idx, value, immediate=False), f"tool {tool_idx} output")
        self.tool_values[tool_idx] = value

    def _feed_mm_s(self, line):
        if 'F' in line.params:
            self.feed = line.params['F']
        if self.feed is None:
            raise MotionCommandError("no feed rate (F) set before first move")
        feed = self.feed / 60.0 * self.units.scale['F']
        if not math.isfinite(feed) or feed <= 0:
            raise MotionCommandError(f"feed resolves to {feed!r} mm/s")
        if feed > self.max_feedrate:
            feed = self.max_feedrate
        return feed

    def _move(self, line):
        params = line.params
        has_xyz = any(k in params for k in 'XYZ')
        feed = 0.0
        if not has_xyz and 'A' not in params and 'E' not in params:
            if 'F' in params:
                self.feed = params['F']
            return
        if has_xyz or 'A' in params:
            feed = self._feed_mm_s(line)
        elif 'F' in params:
            self.feed = params['F']

        if 'A' in params:
            self._focus_move(params['A'] * self.units.scale['A'], feed)

        if 'E' in params:
            if self.extrude_relative:
                delta_e = params['E']
            else:
                delta_e = params['E'] - self.last_e
                self.last_e = params['E']
            turn_on = delta_e > 0
            if turn_on != self.tool_on:
                self._set_tool(self.tool_nr, self.tool_power if turn_on else 0.0)
                self.tool_on = turn_on

        if not has_xyz:
            return

        target = {k: params.get(k, self.position[k]) for k in 'XYZ'}
        status = self.oms.move_to(target['X'] * self.units.scale['X'],
                                  target['Y'] * self.units.scale['Y'],
                                  target['Z'] * self.units.scale['Z'],
                                  feed, blocking=True, timeout=self.command_timeout)
        self._check(status, "move")
        self.position.update(target)

    def _focus_move(self, target_mm, feed):
        if self.focus_axis is None:
            raise MotionCommandError("program contains an A (focus) move but no focus axis is configured")
        if self._cancel.is_set():
            raise OperationCancelled("Stopped by user")

        idle_status = self.oms.ensure_stage_idle_and_tool_off(
            tool_indices=range(len(self.tool_values)), timeout=self.idle_timeout, cancel_event=self._cancel)
        if idle_status is StageIdleStatus.UNCONFIRMED_FIRMWARE:
            message = ("focus move started after controller reported idle, but this firmware does not "
                       "check the active segment (tool-off sequencing relies on the trailing dwell)")
            if message not in self.warnings:
                self.warnings.append(message)
        if self._cancel.is_set():
            raise OperationCancelled("Stopped by user")

        outcome = self.focus_axis.move_to(target_mm, feed, cancel_event=self._cancel)
        if outcome.result is MoveResult.CANCELLED:
            raise OperationCancelled(f"Focus move cancelled at A={outcome.final_position:.6f} mm")
        if outcome.result is not MoveResult.COMPLETED:
            raise MotionCommandError(f"focus move {outcome.result.value}: {outcome.message}")
        self.position['A'] = target_mm

        for tool_idx, value in enumerate(self.tool_values):
            if value > 0.0:
                self._check(self.oms.set_tool_output(tool_idx, value, immediate=False),
                            f"restoring tool {tool_idx} output")

    # -------------------diagnostics-----------------
    def diagnose_gcode(self, min_length=0.1, min_feedrate=50):
        issues = []
        prev = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
        for line, resolved in self.program.iter_resolved_moves(self.initial_scale):
            if not any(k in line.params for k in 'XYZ'):
                continue
            length = math.sqrt(sum((resolved[k] - prev[k]) ** 2 for k in 'XYZ'))
            feedrate = resolved['F'] / 60 if resolved['F'] is not None else 0.0
            issue = {}
            if length < min_length:
                issue['length'] = length
            if min_feedrate is not None and feedrate < min_feedrate:
                issue['feedrate'] = feedrate
            if issue:
                issues.append({'line': line.line_no, 'text': line.text.strip(), **issue})
            prev = {k: resolved[k] for k in 'XYZ'}
        return issues
