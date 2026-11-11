# Python API

This folder contains a lightweight Python interface for the Open Micro-Manipulator serial protocol, plus two small example scripts:

- `usage_example.py`: homes the device, performs a simple move, and prints device state information.
- `calibration_plotter.py`: runs joint calibration for the first three actuators and plots the returned data.

## Requirements

Install the Python dependencies with:

```bash
python -m venv env
source env/bin/activate
pip install -r requirements.txt
```

The API itself is implemented in `open_micro_stage_api.py`.

## Serial Port Selection

Both scripts support:

- `--list-ports`: list detected serial devices and exit
- `--port <PORT>`: explicitly select a serial port

If `--port` is not provided, the scripts try to choose a port automatically:

1. If exactly one detected device contains `Pico` in its name, that port is used.
2. Otherwise, if there is exactly one detected serial device, that port is used.
3. Otherwise, the script lists the available ports and exits.

## Running The Example Script

From this folder:

```bash
python usage_example.py --list-ports
python usage_example.py --port /dev/ttyACM0
```

On Windows, a typical command looks like:

```bash
python usage_example.py --port COM3
```

## Running The Calibration Plotter

From this folder:

```bash
python calibration_plotter.py --list-ports
python calibration_plotter.py --port /dev/ttyACM0
```

The calibration script opens a matplotlib window with the measured calibration curves.

## Running From The Repository Root

If you prefer to run the scripts from the repository root, use:

```bash
python software/PythonAPI/usage_example.py --port /dev/ttyACM0
python software/PythonAPI/calibration_plotter.py --port /dev/ttyACM0
```
