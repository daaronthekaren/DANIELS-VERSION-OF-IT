import math
import os
import threading
import time
from dataclasses import dataclass
from enum import Enum


# -------------------settings-----------------
@dataclass(frozen=True)
class FocusAxisConfig:
    step_pin: int = 17
    dir_pin: int = 27
    enable_pin: int = 22
    enable_active_low: bool = True
    invert_dir: bool = False
    steps_per_mm: float = 6400.0
    max_feed: float = 0.5
    min_pos: float = -10.0
    max_pos: float = 10.0
    pulse_us: float = 5.0
    gpio_chips: tuple = (0, 4)


MODE_ENV = "OMM_FOCUS_MODE"


# -------------------results-----------------
class MoveResult(Enum):
    COMPLETED = 'completed'
    CANCELLED = 'cancelled'
    REJECTED = 'rejected'
    FAILED = 'failed'


@dataclass(frozen=True)
class MoveOutcome:
    result: MoveResult
    requested_target: float
    final_position: float
    message: str = ""
    feed_used: float = 0.0


# -------------------backends-----------------
class SimulatedBackend:
    simulated = True
    description = "SIMULATION (no hardware output)"

    def __init__(self):
        self.writes = []

    def claim_outputs(self, config, enable_level):
        self.writes.append((config.enable_pin, enable_level))

    def write(self, pin, level):
        self.writes.append((pin, level))

    def close(self):
        pass


class LgpioBackend:
    simulated = False

    def __init__(self, config):
        try:
            import lgpio
        except ImportError as exc:
            raise RuntimeError(f"lgpio is not installed ({exc})")
        self._lgpio = lgpio
        self.handle = None
        errors = []
        for chip in config.gpio_chips:
            try:
                handle = lgpio.gpiochip_open(chip)
            except Exception as exc:
                errors.append(f"gpiochip{chip}: {exc}")
                continue
            self.handle = handle
            self.chip = chip
            break
        if self.handle is None:
            raise RuntimeError("no usable GPIO chip (" + "; ".join(errors) + ")")
        self.description = f"GPIO (gpiochip{self.chip})"

    def claim_outputs(self, config, enable_level):
        try:
            self._lgpio.gpio_claim_output(self.handle, config.step_pin, 0)
            self._lgpio.gpio_claim_output(self.handle, config.dir_pin, 0)
            self._lgpio.gpio_claim_output(self.handle, config.enable_pin, enable_level)
        except Exception:
            self.close()
            raise

    def write(self, pin, level):
        self._lgpio.gpio_write(self.handle, pin, level)

    def close(self):
        if self.handle is not None:
            self._lgpio.gpiochip_close(self.handle)
            self.handle = None


# -------------------focus axis-----------------
class FocusAxis:
    def __init__(self, config=None, mode=None, backend=None):
        self.config = config or FocusAxisConfig()
        self.mode = (mode or os.environ.get(MODE_ENV, "gpio")).lower()
        self.steps = 0
        self.enabled = False
        self.backend = None
        self.unavailable_reason = None
        self._moving = threading.Event()
        self._stop = threading.Event()
        self._move_lock = threading.Lock()
        self._state_lock = threading.Lock()

        try:
            if backend is not None:
                candidate = backend
            elif self.mode == "simulate":
                candidate = SimulatedBackend()
            elif self.mode == "gpio":
                candidate = LgpioBackend(self.config)
            else:
                raise RuntimeError(f"unknown focus axis mode {self.mode!r} (use 'gpio' or 'simulate')")
            candidate.claim_outputs(self.config, self._enable_level(False))
            self.backend = candidate
        except Exception as exc:
            self.backend = None
            self.unavailable_reason = str(exc)

    @property
    def available(self):
        return self.backend is not None

    @property
    def simulated(self):
        return self.backend is not None and self.backend.simulated

    def mode_description(self):
        if self.backend is None:
            return f"UNAVAILABLE: {self.unavailable_reason}"
        return self.backend.description

    def _enable_level(self, on):
        if self.config.enable_active_low:
            return 0 if on else 1
        return 1 if on else 0

    def set_enabled(self, on):
        if self.backend is None:
            self.enabled = False
            return False
        if not on and self.is_moving():
            self.stop()
        self.backend.write(self.config.enable_pin, self._enable_level(on))
        self.enabled = bool(on)
        return True

    def is_moving(self):
        return self._moving.is_set()

    def get_position(self):
        with self._state_lock:
            return self.steps / self.config.steps_per_mm

    def set_zero(self):
        if not self._move_lock.acquire(blocking=False):
            return False
        try:
            with self._state_lock:
                self.steps = 0
            return True
        finally:
            self._move_lock.release()

    def move_by(self, delta, feed, cancel_event=None):
        try:
            delta = float(delta)
        except (TypeError, ValueError):
            delta = float('nan')
        return self.move_to(self.get_position() + delta, feed, cancel_event)

    def _outcome(self, result, target, message="", feed=0.0):
        return MoveOutcome(result, target, self.get_position(), message, feed)

    def move_to(self, target, feed, cancel_event=None):
        cfg = self.config
        if self.backend is None:
            return self._outcome(MoveResult.REJECTED, target, f"focus axis unavailable: {self.unavailable_reason}")
        try:
            target = float(target)
            feed = float(feed)
        except (TypeError, ValueError):
            return self._outcome(MoveResult.REJECTED, target, "target and feed must be numbers")
        if not math.isfinite(target):
            return self._outcome(MoveResult.REJECTED, target, f"focus target {target!r} is not finite")
        if not cfg.min_pos <= target <= cfg.max_pos:
            return self._outcome(MoveResult.REJECTED, target,
                                 f"focus target {target} mm is outside [{cfg.min_pos}, {cfg.max_pos}] mm")
        if not math.isfinite(feed) or feed <= 0:
            return self._outcome(MoveResult.REJECTED, target, f"focus feed {feed!r} must be finite and positive")
        if not self._move_lock.acquire(blocking=False):
            return self._outcome(MoveResult.REJECTED, target, "focus axis is already moving")

        message = ""
        if feed > cfg.max_feed:
            message = f"feed limited from {feed} to {cfg.max_feed} mm/s"
            feed = cfg.max_feed
        try:
            self._stop.clear()
            if cancel_event is not None and cancel_event.is_set():
                return self._outcome(MoveResult.CANCELLED, target, "cancelled before start", feed)
            steps = int(round(target * cfg.steps_per_mm)) - self.steps
            if steps == 0:
                return self._outcome(MoveResult.COMPLETED, target, message, feed)
            self._moving.set()
            if not self.enabled:
                self.set_enabled(True)
            return self._run(steps, target, feed, cancel_event, message)
        except Exception as exc:
            return self._outcome(MoveResult.FAILED, target, f"hardware error: {exc}", feed)
        finally:
            self._moving.clear()
            self._move_lock.release()

    def _cancelled(self, cancel_event):
        return self._stop.is_set() or (cancel_event is not None and cancel_event.is_set())

    def _wait_until(self, deadline, cancel_event):
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return False
            if self._cancelled(cancel_event):
                return True
            if remaining > 0.002:
                self._stop.wait(min(remaining - 0.001, 0.005))

    def _run(self, steps, target, feed, cancel_event, message):
        cfg = self.config
        backend = self.backend
        if backend is None:
            return self._outcome(MoveResult.FAILED, target, "focus axis backend closed", feed)
        direction = 1 if steps > 0 else -1
        level = 1 if direction > 0 else 0
        if cfg.invert_dir:
            level = 1 - level
        backend.write(cfg.dir_pin, level)
        period = 1.0 / (feed * cfg.steps_per_mm)
        next_t = time.perf_counter()
        for _ in range(abs(steps)):
            if self._cancelled(cancel_event):
                return self._outcome(MoveResult.CANCELLED, target, "cancelled", feed)
            backend.write(cfg.step_pin, 1)
            if not backend.simulated:
                t_end = time.perf_counter() + cfg.pulse_us * 1e-6
                while time.perf_counter() < t_end:
                    pass
            backend.write(cfg.step_pin, 0)
            with self._state_lock:
                self.steps += direction
            next_t += period
            if self._wait_until(next_t, cancel_event):
                return self._outcome(MoveResult.CANCELLED, target, "cancelled", feed)
        return self._outcome(MoveResult.COMPLETED, target, message, feed)

    def stop(self):
        self._stop.set()

    def wait_idle(self, timeout=None):
        if not self._move_lock.acquire(timeout=-1 if timeout is None else timeout):
            return False
        self._move_lock.release()
        return True

    def close(self):
        self.stop()
        self.wait_idle(timeout=2.0)
        if self.backend is not None:
            try:
                self.set_enabled(False)
            finally:
                self.backend.close()
                self.backend = None
                self.unavailable_reason = "closed"
