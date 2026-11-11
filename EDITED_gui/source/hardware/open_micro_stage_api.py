# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import math
import os
import threading
import time
import re
from enum import Enum

import serial
import numpy as np
from colorama import Fore, Style, init
from serial.tools import list_ports


# -------------------errors-----------------
class MotionCommandError(RuntimeError):
    pass


class CommandTimeout(MotionCommandError):
    pass


class OperationCancelled(Exception):
    pass


class StageIdleStatus(Enum):
    CONFIRMED = 'confirmed'
    UNCONFIRMED_FIRMWARE = 'unconfirmed_firmware'


ACTIVE_SEGMENT_FIX_REVISION = 1
MIN_FEED_MM_S = 1e-6
MAX_FEED_MM_S = 1000.0
NUM_TOOLS = 2


def format_positive(value, name, fmt='.7g', minimum=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive, got {value!r}")
    if value < minimum:
        raise ValueError(f"{name} {value!r} is below the supported minimum {minimum!r}")
    text = format(value, fmt)
    if not math.isfinite(float(text)) or float(text) <= 0:
        raise ValueError(f"{name} {value!r} is below the supported serialization range")
    return text


def format_finite(value, name, fmt='.6f'):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite, got {value!r}")
    return format(value, fmt)


# --- SerialInterface --------------------------------------------------------------------------------------------------

class SerialInterface:

    class ReplyStatus(Enum):
        OK = 'ok'
        ERROR = 'error'
        TIMEOUT = 'timeout'
        BUSY = 'busy'

    class LogLevel(Enum):
        DEBUG = 'debug'
        INFO = 'info'
        WARNING = 'warning'
        ERROR = 'error'

    log_level_prefix_map = {
        "D)": LogLevel.DEBUG,
        "I)": LogLevel.INFO,
        "W)": LogLevel.WARNING,
        "E)": LogLevel.ERROR,
    }

    def __init__(self, port: str, baud_rate: int = 115200,
                 command_msg_callback=None,
                 log_msg_callback=None,
                 unsolicited_msg_callback=None,
                 reconnect_timeout: int = 5,
                 serial_factory=None):
        self.port = port
        self.baud_rate = baud_rate
        self.reconnect_timeout = reconnect_timeout
        self.serial = None
        self._serial_factory = serial_factory if serial_factory is not None else serial.Serial

        self.command_msg_callback = command_msg_callback
        self.log_message_callback = log_msg_callback
        self.unsolicited_msg_callback = unsolicited_msg_callback

        self._transaction_lock = threading.Lock()
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._waiting_for_response = False
        self._response_string = ""
        self._response_status = None
        self._response_error_msg = None
        self._stale_replies = 0
        self._stop_event = threading.Event()
        self._reader_thread = None

        if self.connect(self.reconnect_timeout):
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()
            while not self._reader_thread.is_alive():
                time.sleep(0.001)

    def connect(self, timeout):
        if self._stop_event.is_set():
            return False

        deadline = time.time() + timeout
        print(Fore.MAGENTA, end='')
        print(f"[SerialInterface] Connecting to port '{self.port}'...", end='')
        while time.time() < deadline:
            try:
                self.serial = self._serial_factory(self.port, self.baud_rate, timeout=2)
                with self._condition:
                    self._stale_replies = 0
                print(f" [OK]")
                print(Style.RESET_ALL, end='')
                return True
            except (serial.SerialException, OSError) as e:
                if getattr(e, 'errno', None) == 13:
                    import sys
                    print(f"\n[SerialInterface] Permission denied on '{self.port}'.")
                    if sys.platform.startswith('linux'):
                        username = os.getenv('USER') or os.getenv('LOGNAME') or 'your_user'
                        print(f"[SerialInterface] Add your user to the dialout group and re-login: sudo usermod -aG dialout {username}")
                    print(Style.RESET_ALL, end='')
                    self.serial = None
                    return False
                print('.', end='')
                time.sleep(0.2)

        print(f" [FAILED] Timeout after {timeout} seconds.")
        print(f"[SerialInterface] Connection is permanently closed")
        print(Style.RESET_ALL, end='')
        self.serial = None
        return False

    def _reader_loop(self):
        buffer = ""
        while not self._stop_event.is_set():
            try:
                waiting = self.serial.in_waiting if self.serial is not None else 0
                if waiting:
                    data = self.serial.read(waiting).decode('ascii', errors='ignore')
                    for char in data:
                        if char in ['\n', '\r']:
                            if len(buffer) > 0:
                                self._handle_line(buffer)
                                buffer = ""
                        else:
                            buffer += char
                else:
                    time.sleep(0.001)
            except (serial.SerialException, OSError) as e:
                if self._stop_event.is_set():
                    break

                print(Fore.MAGENTA+f"[SerialInterface] Lost connection: {e}"+Style.RESET_ALL)
                try:
                    if self.serial is not None and self.serial.is_open:
                        self.serial.close()
                except Exception:
                    pass

                self.serial = None
                buffer = ""
                self.connect(self.reconnect_timeout)

    @staticmethod
    def _terminal_status(line: str):
        line_lower = line.lower()
        if line_lower.startswith("ok"):
            return SerialInterface.ReplyStatus.OK
        if line_lower.startswith("busy"):
            return SerialInterface.ReplyStatus.BUSY
        if line_lower.startswith("error"):
            return SerialInterface.ReplyStatus.ERROR
        return None

    def _handle_line(self, line: str):
        log_level, log_msg = self._check_log_msg(line)
        if log_level is not None:
            if self.log_message_callback:
                self.log_message_callback(log_level, log_msg)
            return

        unsolicited = False
        with self._condition:
            status = self._terminal_status(line)
            if self._stale_replies > 0:
                if status is not None:
                    self._stale_replies -= 1
            elif self._waiting_for_response:
                if status is not None:
                    self._response_status = status
                    if status == SerialInterface.ReplyStatus.ERROR:
                        parts = line.split(":", 1)
                        self._response_error_msg = parts[1].strip() if len(parts) > 1 else ""
                    self._condition.notify_all()
                else:
                    self._response_string += line + '\n'
            else:
                unsolicited = True

        if unsolicited and self.unsolicited_msg_callback:
            self.unsolicited_msg_callback(line)

    def _check_log_msg(self, msg: str):
        if len(msg) < 2:
            return None, ''
        return self.log_level_prefix_map.get(msg[:2]), msg[2:]

    def pending_stale_replies(self):
        with self._condition:
            return self._stale_replies

    def send_command(self, cmd: str, timeout: float = 2.0) -> tuple[ReplyStatus, str]:
        with self._transaction_lock:
            with self._condition:
                if not self.serial or not self.serial.is_open:
                    return SerialInterface.ReplyStatus.ERROR, 'Serial not open'

                self._waiting_for_response = True
                self._response_string = ""
                self._response_error_msg = ""
                self._response_status = None

                cmd = (cmd.strip() + "\n")
                if self.command_msg_callback:
                    self.command_msg_callback(cmd, None, '')

                try:
                    self.serial.write(cmd.encode('ascii'))
                    self.serial.flush()
                except (serial.SerialException, OSError) as exc:
                    self._waiting_for_response = False
                    return SerialInterface.ReplyStatus.ERROR, f'Serial write failed: {exc}'

                completed = self._condition.wait_for(
                    lambda: self._response_status is not None or self._stop_event.is_set(), timeout=timeout)
                self._waiting_for_response = False
                if self._response_status is None and self._stop_event.is_set():
                    return SerialInterface.ReplyStatus.ERROR, 'Connection closed while waiting for reply'
                if not completed:
                    self._stale_replies += 1
                    print(Fore.MAGENTA + f"[SerialInterface] Command timeout, device didn't reply in time: {cmd.strip()}" + Style.RESET_ALL)
                    return SerialInterface.ReplyStatus.TIMEOUT, self._response_string

                status = self._response_status
                assert status is not None
                response = self._response_string
                error_msg = self._response_error_msg

            if self.command_msg_callback:
                self.command_msg_callback(response, status, error_msg)
            return status, response

    def close(self):
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.serial = None

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)

# --- OpenMicroStageInterface ------------------------------------------------------------------------------------------

class OpenMicroStageInterface:
    LOG_COLORS = {
        SerialInterface.LogLevel.DEBUG: Fore.WHITE+Style.DIM,
        SerialInterface.LogLevel.INFO: Style.RESET_ALL,
        SerialInterface.LogLevel.WARNING: Fore.YELLOW,
        SerialInterface.LogLevel.ERROR: Fore.RED,
    }

    def __init__(self, show_communication=True, show_log_messages=True):
        self.serial = None
        self.workspace_transform = np.eye(4)
        self.workspace_transform_inv = np.linalg.inv(self.workspace_transform)
        self.show_communication = show_communication
        self.show_log_messages = show_log_messages
        self.disable_message_callbacks = False
        self.firmware_version = (0, 0, 0)
        self.firmware_fork_revision = 0

    @staticmethod
    def enumerate_devices():
        devices = []

        for port in sorted(list_ports.comports(), key=lambda item: item.device):
            description = (port.description or "").strip()
            device_name = os.path.basename(port.device)
            label = device_name
            if description and description.lower() != "n/a" and description != port.device:
                label = f"{device_name} - {description}"

            devices.append({
                "id": port.device,
                "label": label,
                "port": port.device,
            })

        return devices

    def connect(self, port: str, baud_rate: int = 921600, serial_factory=None):
        def version_to_str(v):
            return f"v{v[0]}.{v[1]}.{v[2]}"

        if self.serial is not None:
            self.disconnect()

        serial_interface = SerialInterface(port, baud_rate,
                                           log_msg_callback=self.log_msg_callback,
                                           command_msg_callback=self.command_msg_callback,
                                           unsolicited_msg_callback=self.unsolicited_msg_callback,
                                           serial_factory=serial_factory)
        if serial_interface.serial is None:
            serial_interface.close()
            return False

        self.serial = serial_interface

        self.disable_message_callbacks = True
        fw_version = self.read_firmware_version()
        min_fw_version = (1, 0, 1)
        print(Fore.MAGENTA + f"Firmware version: {version_to_str(fw_version)}" + Style.RESET_ALL)
        if fw_version < min_fw_version:
            print(Fore.MAGENTA + f"Firmware version {version_to_str(fw_version)} incompatible. "
                                 f"At least {version_to_str(min_fw_version)} required" + Style.RESET_ALL)
            self.serial.close()
            self.serial = None
            print('')
            self.disable_message_callbacks = False
            return False
        print('')
        self.disable_message_callbacks = False
        return True

    def disconnect(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def is_connected(self):
        serial_interface = self.serial
        return serial_interface is not None and not serial_interface._stop_event.is_set()

    def log_msg_callback(self, log_level, msg):
        if not self.show_log_messages or self.disable_message_callbacks:
            return

        color = OpenMicroStageInterface.LOG_COLORS.get(log_level, Fore.WHITE)
        if log_level not in [SerialInterface.LogLevel.INFO, SerialInterface.LogLevel.DEBUG]:
            print(f"{color}[{log_level.name}] {msg}{Style.RESET_ALL}")
        else:
            print(f"{color}{msg}{Style.RESET_ALL}")

    def command_msg_callback(self, msg, reply_status: SerialInterface.ReplyStatus, error_msg: str):
        if not self.show_communication or self.disable_message_callbacks:
            return

        if reply_status is not None:
            if msg:
                msg = '\n'.join('> ' + line for line in msg.splitlines())
                print(f"{msg.rstrip()}")
            if error_msg:
                print(f"{Style.BRIGHT}{str(reply_status.name)}:{Style.RESET_ALL} {error_msg}\n")
            else:
                print(f"{Style.BRIGHT}{str(reply_status.name)} {Style.RESET_ALL}\n")
        else:
            print(f"{Fore.GREEN+Style.BRIGHT}{msg.rstrip()}{Style.RESET_ALL}")

    def unsolicited_msg_callback(self, msg):
        print(Fore.CYAN+msg+Style.RESET_ALL)

    def set_workspace_transform(self, transform):
        self.workspace_transform = transform
        self.workspace_transform_inv = np.linalg.inv(self.workspace_transform)

    def get_workspace_transform(self):
        return self.workspace_transform

    def read_firmware_version(self):
        ok, response = self.serial.send_command("M58")
        self.firmware_version = (0, 0, 0)
        self.firmware_fork_revision = 0
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return self.firmware_version

        match = re.match(r'\s*v(\d+)\.(\d+)\.(\d+)(?:-sd\.(\d+))?', response)
        if match is None:
            return self.firmware_version

        major, minor, patch = map(int, match.groups()[:3])
        self.firmware_version = (major, minor, patch)
        self.firmware_fork_revision = int(match.group(4)) if match.group(4) else 0
        return self.firmware_version

    def home(self, axis_list=None):
        cmd = 'G28'
        axis_chars = ['A', 'B', 'C', 'D', 'E', 'F']
        if axis_list is None:
            axis_list = [i for i in range(len(axis_chars))]

        for axis_idx in axis_list:
            if not 0 <= axis_idx < len(axis_chars):
                raise ValueError('Axis index out of range')
            cmd += ' '+axis_chars[axis_idx]

        res, msg = self.serial.send_command(cmd + "\n", 10)
        return res

    def calibrate_joint(self, joint_index: int, save_result: bool):
        cmd = f"M56 J{joint_index} P"
        if save_result: cmd += ' S'
        res, msg = self.serial.send_command(cmd, 30)

        calibration_data = self._parse_table_data(msg, 3)
        return res, calibration_data

    def move_to(self, x, y, z, f, move_immediately=False, blocking=True, timeout=1.0):
        x_s = format_finite(x, 'X')
        y_s = format_finite(y, 'Y')
        z_s = format_finite(z, 'Z')
        if float(f) > MAX_FEED_MM_S:
            raise ValueError(f"Feed {f!r} mm/s exceeds the supported maximum {MAX_FEED_MM_S} mm/s")
        f_s = format_positive(f, 'Feed', minimum=MIN_FEED_MM_S)

        transformed = self.workspace_transform @ np.array([float(x_s), float(y_s), float(z_s), 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G0 X{format_finite(x_t, 'X')} Y{format_finite(y_t, 'Y')} Z{format_finite(z_t, 'Z')} F{f_s}"
        if move_immediately:
            cmd += " I"

        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def dwell(self, time_s, blocking, timeout=1.0):
        time_text = format_finite(time_s, 'Dwell time')
        if float(time_text) < 0:
            raise ValueError(f"Dwell time must not be negative, got {time_s!r}")
        cmd = f"G4 S{time_text}"
        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def set_max_acceleration(self, linear_accel, angular_accel):
        linear_accel = max(linear_accel, 0.01)
        angular_accel = max(angular_accel, 0.01)
        cmd = f"M204 L{linear_accel:.6f} A{angular_accel:.6f}\n"
        res, msg = self.serial.send_command(cmd)
        return res

    def wait_for_stop(self, polling_interval_ms=50, disable_callbacks=True, timeout=None, cancel_event=None):
        disable_message_callbacks_prev = self.disable_message_callbacks
        if disable_callbacks:
            self.disable_message_callbacks = True
        deadline = None if timeout is None else time.monotonic() + timeout
        try:
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    raise OperationCancelled("Wait for stage stop was cancelled")
                res, msg = self.serial.send_command("M53\n")
                if res != SerialInterface.ReplyStatus.OK:
                    return res
                if msg.strip() == "1":
                    return SerialInterface.ReplyStatus.OK
                if deadline is not None and time.monotonic() >= deadline:
                    return SerialInterface.ReplyStatus.TIMEOUT
                if cancel_event is not None:
                    if cancel_event.wait(polling_interval_ms * 0.001):
                        raise OperationCancelled("Wait for stage stop was cancelled")
                else:
                    time.sleep(polling_interval_ms * 0.001)
        finally:
            self.disable_message_callbacks = disable_message_callbacks_prev

    def ensure_stage_idle_and_tool_off(self, tool_indices=(0, 1), timeout=30.0, cancel_event=None):
        for tool_idx in tool_indices:
            status = self.set_tool_output(tool_idx, 0.0, immediate=False)
            if status is not SerialInterface.ReplyStatus.OK:
                raise MotionCommandError(f"Tool {tool_idx} off command rejected: {status.name}")

        status = self.dwell(0.001, blocking=True, timeout=max(1.0, timeout))
        if status is not SerialInterface.ReplyStatus.OK:
            raise MotionCommandError(f"Dwell carrying tool-off state rejected: {status.name}")

        status = self.wait_for_stop(timeout=timeout, cancel_event=cancel_event)
        if status is SerialInterface.ReplyStatus.TIMEOUT:
            raise CommandTimeout(f"Stage did not report idle within {timeout} s")
        if status is not SerialInterface.ReplyStatus.OK:
            raise MotionCommandError(f"Idle query failed: {status.name}")

        if self.firmware_fork_revision >= ACTIVE_SEGMENT_FIX_REVISION:
            return StageIdleStatus.CONFIRMED
        return StageIdleStatus.UNCONFIRMED_FIRMWARE

    def read_current_position(self, apply_inv_workspace_transform):
        ok, response = self.serial.send_command("M50")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return None, None, None

        match = re.search(
            r"X([-+]?\d*\.?\d+)\s*Y([-+]?\d*\.?\d+)\s*Z([-+]?\d*\.?\d+)",
            response
        )
        if not match:
            raise ValueError(f"Invalid format: {response}")

        x, y, z = match.groups()
        if apply_inv_workspace_transform:
            transformed = self.workspace_transform_inv @ np.array([float(x), float(y), float(z), 1.0])
            x, y, z = transformed[:3] / transformed[3]

        return float(x), float(y), float(z)

    def read_encoder_angles(self):
        ok, response = self.serial.send_command("M51")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return []
        return []

    def read_device_state_info(self):
        res, msg = self.serial.send_command("M57")
        return res, msg

    def set_servo_parameter(self, pos_kp=150, pos_ki=50000, vel_kp=0.2, vel_ki=100, vel_filter_tc=0.0025):
        cmd = f"M55 A{pos_kp:.6f} B{pos_ki:.6f} C{vel_kp:.6f} D{vel_ki:.6f} F{vel_filter_tc:.6f}"
        res, msg = self.serial.send_command(cmd)
        return res

    def enable_motors(self, enable: bool):
        cmd = f"M17" if enable else "M18"
        res, msg = self.serial.send_command(cmd, timeout=5)
        return res

    def set_pose(self, x: float, y: float, z: float):
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G24 X{format_finite(x_t, 'X')} Y{format_finite(y_t, 'Y')} Z{format_finite(z_t, 'Z')}"
        res, msg = self.serial.send_command(cmd)
        return res

    def set_tool_output(self, tool_idx: int, output_value: float, immediate: bool = True):
        if int(tool_idx) != tool_idx or not 0 <= int(tool_idx) < NUM_TOOLS:
            raise ValueError(f"Tool index {tool_idx!r} is outside [0, {NUM_TOOLS - 1}]")
        value_text = format_finite(output_value, 'Tool output', '.4f')
        if not 0.0 <= float(value_text) <= 1.0:
            raise ValueError(f"Tool output {output_value!r} is outside [0, 1]")
        cmd = f"M3 T{int(tool_idx)} S{value_text}"
        res, msg = self.serial.send_command(cmd)

        if immediate and res is SerialInterface.ReplyStatus.OK:
            dwell_res, _ = self.serial.send_command("G4 S0.001")
            if dwell_res is not SerialInterface.ReplyStatus.OK:
                return dwell_res

        return res

    def send_command(self, cmd: str, timeout_s: float=5):
        res, msg = self.serial.send_command(cmd, timeout_s)
        return res, msg

    @staticmethod
    def _parse_table_data(data_string, cols):
        data = [[] for _ in range(cols)]

        for line in data_string.strip().splitlines():
            parts = line.strip().split(',')
            if len(parts) != cols:
                continue
            numbers = map(float, parts)
            for i, n in enumerate(numbers):
                data[i].append(n)

        return data
