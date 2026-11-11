import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "source"))
sys.path.insert(0, HERE)

import pytest

from fake_controller import FakeController, connect_fake
from hardware.open_micro_stage_api import OpenMicroStageInterface


@pytest.fixture
def controller():
    return FakeController()


@pytest.fixture
def oms(controller):
    stage = OpenMicroStageInterface(show_communication=False, show_log_messages=False)
    connect_fake(stage, controller)
    yield stage
    stage.disconnect()
