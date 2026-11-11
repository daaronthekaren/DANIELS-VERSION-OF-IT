import argparse

import matplotlib.pyplot as plt

from open_micro_stage_api import OpenMicroStageInterface

plt.rcParams['figure.dpi'] = 200


def plot_calibration_data(ax_encoder_counts, ax_field_angel, label, data):
    # Plot on the provided Axes object
    if ax_encoder_counts is not None:
        ax_encoder_counts.plot(data[0], data[2], label=label)
        ax_encoder_counts.set_xlabel('Motor Angle [rad]')
        ax_encoder_counts.set_ylabel('Encoder Counts Raw')
        ax_encoder_counts.set_title('Encoder Count Plot')
        ax_encoder_counts.legend()
        ax_encoder_counts.grid(True)

    # Plot on the provided Axes object
    if ax_field_angel is not None:
        ax_field_angel.plot(data[0], data[1], label=label)
        ax_field_angel.set_xlabel('Motor Angle [rad]')
        ax_field_angel.set_ylabel('Motor Field Angle [rad]')
        ax_field_angel.set_title('Field Angle Plot')
        ax_field_angel.legend()
        ax_field_angel.grid(True)


def list_available_ports():
    devices = OpenMicroStageInterface.enumerate_devices()
    if not devices:
        print('No serial devices detected.')
        return devices

    print('Available serial devices:')
    for device in devices:
        print(f"  {device['label']}")

    return devices


def resolve_port(port):
    if port:
        return port

    devices = OpenMicroStageInterface.enumerate_devices()
    pico_devices = [device for device in devices if 'pico' in device['label'].lower()]

    if len(pico_devices) == 1:
        print(f"Using detected Pico serial device: {pico_devices[0]['label']}")
        return pico_devices[0]['port']

    if len(pico_devices) > 1:
        print('Multiple Pico serial devices detected. Pass --port to choose one:')
        for device in pico_devices:
            print(f"  {device['label']}")
        raise SystemExit(1)

    if len(devices) == 1:
        print(f"Using detected serial device: {devices[0]['label']}")
        return devices[0]['port']

    if devices:
        print('Multiple serial devices detected. Pass --port to choose one:')
        for device in devices:
            print(f"  {device['label']}")
    else:
        print('No serial devices detected.')

    raise SystemExit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run joint calibration and plot the measured data.',
    )
    parser.add_argument(
        '--port',
        help='Serial port to use (for example /dev/ttyACM0 or COM3).',
    )
    parser.add_argument(
        '--list-ports',
        action='store_true',
        help='List detected serial devices and exit.',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.list_ports:
        list_available_ports()
        return

    port = resolve_port(args.port)
    oms = OpenMicroStageInterface(show_communication=True, show_log_messages=True)
    if not oms.connect(port):
        raise SystemExit(f'Could not connect to {port}.')

    try:
        # Create subplots
        fig, ax = plt.subplots(1, 1, figsize=(10, 7), sharex='all')

        for i in range(3):
            res, data = oms.calibrate_joint(i, save_result=False)
            plot_calibration_data(ax, None, f'Actuator {i}', data)

        # Adjust layout and show
        plt.tight_layout()
        plt.show()
    finally:
        oms.disconnect()


if __name__ == '__main__':
    main()
