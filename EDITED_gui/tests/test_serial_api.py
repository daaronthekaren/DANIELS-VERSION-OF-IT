import threading

import pytest

from fake_controller import FakeController, FakeSerial, connect_fake
from hardware.open_micro_stage_api import (
    CommandTimeout,
    MotionCommandError,
    OpenMicroStageInterface,
    OperationCancelled,
    SerialInterface,
    StageIdleStatus,
    format_positive,
)

OK = SerialInterface.ReplyStatus.OK


# -------------------reply status-----------------
def test_error_reply_is_truthy_enum_but_not_ok(controller, oms):
    controller.homed = False
    status = oms.move_to(0.1, 0, 0, 1.0)
    assert status is SerialInterface.ReplyStatus.ERROR
    assert bool(status)
    assert status is not OK


def test_firmware_version_and_fork_revision(controller, oms):
    assert oms.firmware_version == (1, 0, 5)
    assert oms.firmware_fork_revision == 1
    stock = OpenMicroStageInterface(show_communication=False, show_log_messages=False)
    connect_fake(stock, FakeController(fork_firmware=False))
    assert stock.firmware_fork_revision == 0
    stock.disconnect()


# -------------------feed serialization-----------------
@pytest.mark.parametrize("feed", [0.0, -1.0, float("nan"), float("inf"), 1e-9, 5e-7])
def test_invalid_or_tiny_feed_rejected_before_sending(controller, oms, feed):
    before = len(controller.commands)
    with pytest.raises(ValueError):
        oms.move_to(0, 0, 0, feed)
    assert len(controller.commands) == before


def test_small_valid_feed_serializes_positive(controller, oms):
    assert oms.move_to(0, 0, 0, 2e-6) is OK
    sent = controller.commands[-1]
    feed_text = sent.split("F")[1]
    assert float(feed_text) > 0
    assert format_positive(2e-6, "f") == "2e-06"


def test_feed_upper_bound(oms):
    with pytest.raises(ValueError):
        oms.move_to(0, 0, 0, 1e6)


def test_nonfinite_coordinates_rejected(controller, oms):
    with pytest.raises(ValueError):
        oms.move_to(float("nan"), 0, 0, 1.0)


def test_tool_output_validated(controller, oms):
    with pytest.raises(ValueError):
        oms.set_tool_output(0, 50, immediate=False)
    with pytest.raises(ValueError):
        oms.set_tool_output(2, 0.5, immediate=False)
    assert oms.set_tool_output(1, 0.5, immediate=False) is OK
    assert controller.commands[-1] == "M3 T1 S0.5000"


# -------------------transactions-----------------
def test_concurrent_callers_get_their_own_replies(controller, oms):
    controller.reply_delays = {"ECHO a": 0.05, "ECHO c": 0.02}
    results = {}

    def call(tag):
        status, text = oms.serial.send_command(f"ECHO {tag}", timeout=2)
        results[tag] = (status, text.strip())

    threads = [threading.Thread(target=call, args=(t,)) for t in "abcdefgh"]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    for tag, (status, text) in results.items():
        assert status is OK
        assert text == f"data:{tag}"


def test_late_reply_is_not_used_for_next_command(controller, oms):
    controller.reply_delays = {"ECHO slow": 0.3}
    status, text = oms.serial.send_command("ECHO slow", timeout=0.05)
    assert status is SerialInterface.ReplyStatus.TIMEOUT
    assert oms.serial.pending_stale_replies() == 1
    status, text = oms.serial.send_command("ECHO next", timeout=2)
    assert status is OK
    assert text.strip() == "data:next"
    assert oms.serial.pending_stale_replies() == 0


def test_closed_serial_reports_error(oms):
    oms.serial.serial.close()
    status, text = oms.serial.send_command("M58")
    assert status is SerialInterface.ReplyStatus.ERROR


# -------------------idle and tool-off-----------------
def test_tool_off_modal_command_alone_does_not_change_output(controller, oms):
    assert oms.set_tool_output(0, 0.5, immediate=False) is OK
    assert oms.move_to(0.1, 0, 0, 1.0) is OK
    assert oms.wait_for_stop(polling_interval_ms=1) is OK
    assert controller.physical_tools[0] == 0.5
    assert oms.set_tool_output(0, 0.0, immediate=False) is OK
    assert oms.wait_for_stop(polling_interval_ms=1) is OK
    assert controller.physical_tools[0] == 0.5


def test_ensure_stage_idle_and_tool_off_applies_off_state(controller, oms):
    oms.set_tool_output(0, 0.5, immediate=False)
    oms.set_tool_output(1, 0.3, immediate=False)
    for x in (0.1, 0.2, 0.3):
        oms.move_to(x, 0, 0, 1.0)
    status = oms.ensure_stage_idle_and_tool_off(timeout=5)
    assert status is StageIdleStatus.CONFIRMED
    assert controller.physical_tools == [0.0, 0.0]
    assert controller.queue == [] and controller.active is None
    assert controller.position == (0.3, 0.0, 0.0)


def test_stock_firmware_tool_off_reported_unconfirmed():
    controller = FakeController(fork_firmware=False)
    stage = OpenMicroStageInterface(show_communication=False, show_log_messages=False)
    connect_fake(stage, controller)
    stage.set_tool_output(0, 0.5, immediate=False)
    stage.move_to(0.1, 0, 0, 1.0)
    assert stage.ensure_stage_idle_and_tool_off(timeout=5) is StageIdleStatus.UNCONFIRMED_FIRMWARE
    assert controller.physical_tools[0] == 0.0
    stage.disconnect()


def test_tool_off_rejection_raises(controller, oms):
    controller.homed = False
    with pytest.raises(MotionCommandError, match="Dwell"):
        oms.ensure_stage_idle_and_tool_off(timeout=1)


def test_idle_wait_timeout_raises(controller, oms):
    controller.segments_per_poll = 0
    oms.move_to(0.1, 0, 0, 1.0)
    with pytest.raises(CommandTimeout):
        oms.ensure_stage_idle_and_tool_off(timeout=0.2)


def test_idle_wait_cancellation(controller, oms):
    controller.segments_per_poll = 0
    oms.move_to(0.1, 0, 0, 1.0)
    cancel = threading.Event()
    threading.Timer(0.1, cancel.set).start()
    with pytest.raises(OperationCancelled):
        oms.ensure_stage_idle_and_tool_off(timeout=5, cancel_event=cancel)
