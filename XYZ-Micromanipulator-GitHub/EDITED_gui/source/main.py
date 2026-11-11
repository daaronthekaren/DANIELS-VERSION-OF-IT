# --------------------------------------------------------------------------------------
# Project: OpenMicroManipulator
# License: MIT (see LICENSE file for full description)
#          All text in here must be included in any redistribution.
# Author:  M. S. (diffraction limited)
# --------------------------------------------------------------------------------------

import argparse
import os
import sys

os.environ['QT_SCALE_FACTOR'] = '1'
os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
os.environ['GDK_SCALE'] = '1'
os.environ['GDK_DPI_SCALE'] = '1'

from PySide6.QtWidgets import QApplication

from hardware.focus_axis import FocusAxis
from hardware.open_micro_stage_api import OpenMicroStageInterface
from mainwindow import DeviceControlMainWindow


def main():
    parser = argparse.ArgumentParser(description="Open Micro-Manipulator GUI")
    parser.add_argument("--simulate-focus", action="store_true",
                        help="run the focus (A) axis in explicit simulation mode instead of GPIO")
    args, qt_args = parser.parse_known_args()

    oms = OpenMicroStageInterface(show_communication=False, show_log_messages=True)
    app = QApplication([sys.argv[0]] + qt_args)
    app.setOrganizationName("OpenMicroManipulator")
    app.setApplicationName("OpenMicroManipulatorGUI")
    focus_axis = FocusAxis(mode="simulate" if args.simulate_focus else None)
    gui = DeviceControlMainWindow(oms, focus_axis=focus_axis)
    gui.show()
    app.exec()

if __name__ == "__main__":
    main()
