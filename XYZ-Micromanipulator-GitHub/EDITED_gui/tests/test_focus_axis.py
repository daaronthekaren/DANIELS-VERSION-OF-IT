import threading
import time

import pytest

from hardware.focus_axis import FocusAxis, FocusAxisConfig, MoveResult, SimulatedBackend


class FailingBackend(SimulatedBackend):
    def __init__(self, fail_after=3):
        super().__init__()
        self.fail_after = fail_after
        self.step_writes = 0

    def write(self, pin, level):
        if pin == FocusAxisConfig().step_pin and level == 1:
            self.step_writes += 1
            if self.step_writes > self.fail_after:
                raise OSError("GPIO write failed")
        super().write(pin, level)


def make_axis(**kwargs):
    return FocusAxis(config=FocusAxisConfig(**kwargs), mode="simulate")


# -------------------mode-----------------
def test_simulation_is_explicit():
    axis = make_axis()
    assert axis.available and axis.simulated
    assert "SIMULATION" in axis.mode_description()


def test_gpio_failure_does_not_fall_back_to_simulation():
    axis = FocusAxis(mode="gpio")
    if axis.available:
        pytest.skip("real GPIO available on this machine")
    assert not axis.simulated
    assert "UNAVAILABLE" in axis.mode_description()
    outcome = axis.move_to(0.001, 0.1)
    assert outcome.result is MoveResult.REJECTED
    assert axis.get_position() == 0.0


def test_unknown_mode_is_unavailable():
    axis = FocusAxis(mode="banana")
    assert not axis.available


# -------------------limits and validation-----------------
@pytest.mark.parametrize("target", [10.0001, -10.5, float("nan"), float("inf")])
def test_out_of_range_target_rejected(target):
    axis = make_axis()
    outcome = axis.move_to(target, 0.1)
    assert outcome.result is MoveResult.REJECTED
    assert axis.get_position() == 0.0
    assert axis.backend.writes == [(22, 1)]


@pytest.mark.parametrize("feed", [0.0, -1.0, float("nan")])
def test_invalid_feed_rejected(feed):
    outcome = make_axis().move_to(0.001, feed)
    assert outcome.result is MoveResult.REJECTED


def test_completed_move_reports_commanded_position():
    axis = make_axis(steps_per_mm=1000.0, max_feed=50.0)
    outcome = axis.move_to(0.01, 50.0)
    assert outcome.result is MoveResult.COMPLETED
    assert outcome.final_position == pytest.approx(0.01)
    assert axis.get_position() == pytest.approx(0.01)


def test_feed_above_limit_is_reported():
    axis = make_axis(steps_per_mm=1000.0, max_feed=5.0)
    outcome = axis.move_to(0.002, 50.0)
    assert outcome.result is MoveResult.COMPLETED
    assert outcome.feed_used == 5.0
    assert "limited" in outcome.message


# -------------------cancellation and errors-----------------
def test_stop_interrupts_long_move_promptly():
    axis = make_axis(steps_per_mm=1000.0, max_feed=0.01)
    result = {}
    worker = threading.Thread(target=lambda: result.setdefault("o", axis.move_to(1.0, 0.01)))
    worker.start()
    time.sleep(0.2)
    started = time.monotonic()
    axis.stop()
    worker.join(2.0)
    assert not worker.is_alive()
    assert time.monotonic() - started < 0.5
    assert result["o"].result is MoveResult.CANCELLED
    assert 0 < axis.get_position() < 1.0


def test_external_cancel_event():
    axis = make_axis(steps_per_mm=1000.0, max_feed=0.01)
    cancel = threading.Event()
    threading.Timer(0.1, cancel.set).start()
    outcome = axis.move_to(1.0, 0.01, cancel_event=cancel)
    assert outcome.result is MoveResult.CANCELLED


def test_cancelled_before_start():
    cancel = threading.Event()
    cancel.set()
    axis = make_axis()
    assert axis.move_to(0.001, 0.1, cancel_event=cancel).result is MoveResult.CANCELLED
    assert axis.get_position() == 0.0


def test_hardware_error_reported_as_failed():
    axis = FocusAxis(config=FocusAxisConfig(steps_per_mm=1000.0, max_feed=50.0),
                     backend=FailingBackend(fail_after=3))
    outcome = axis.move_to(0.01, 50.0)
    assert outcome.result is MoveResult.FAILED
    assert "GPIO write failed" in outcome.message
    assert axis.get_position() == pytest.approx(0.003)
    assert not axis.is_moving()


def test_concurrent_move_rejected_and_zero_blocked():
    axis = make_axis(steps_per_mm=1000.0, max_feed=0.01)
    worker = threading.Thread(target=axis.move_to, args=(1.0, 0.01))
    worker.start()
    time.sleep(0.1)
    assert axis.move_to(-1.0, 0.01).result is MoveResult.REJECTED
    assert axis.set_zero() is False
    axis.stop()
    worker.join(2.0)
    assert axis.set_zero() is True
    assert axis.get_position() == 0.0


def test_close_stops_and_disables():
    axis = make_axis(steps_per_mm=1000.0, max_feed=0.01)
    worker = threading.Thread(target=axis.move_to, args=(1.0, 0.01))
    worker.start()
    time.sleep(0.05)
    backend = axis.backend
    axis.close()
    worker.join(2.0)
    assert not worker.is_alive()
    assert backend.writes[-1] == (22, 1)
    assert not axis.available
