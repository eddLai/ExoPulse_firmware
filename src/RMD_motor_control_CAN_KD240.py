#!/usr/bin/env python3
import can
import struct
import argparse
import time

def send_can_frame(bus, can_id, data):
    """ Send a CAN frame over the bus. """
    # Create CAN message with ID and data (8 bytes max)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    try:
        # Send the message on the bus
        bus.send(msg)
        print(f"Sent: CAN ID=0x{can_id:X}, Data={data}")
    except can.CanError:
        print(f"Error: Failed to send CAN message with ID=0x{can_id:X}")

def open_bus(interface):
    """ Open the CAN bus with the provided interface name. """
    return can.interface.Bus(channel=interface, bustype="socketcan")

def create_data_array(data_list):
    """ Convert a list of hex values to a byte array. """
    return [int(x, 16) if isinstance(x, str) else x for x in data_list]

def send_raw_frame(bus, interface, can_id, data_list):
    """ Send a raw CAN frame. """
    data = create_data_array(data_list)
    send_can_frame(bus, can_id, data)

def create_rmd_frame(bus, command, motor_id, params):
    """ Create a frame for RMD motor commands. """
    # Motor ID base is 0x140 + motor_id
    tx_id = 0x140 + motor_id
    data = bytearray(8)

    if command == 'status1':
        # 0x9A: Request status 1 (temperature, voltage, error state)
        data[0] = 0x9A
        send_can_frame(bus, tx_id, data)

    elif command == 'speed':
        # 0xA2: Speed control (dps to int32)
        data[0] = 0xA2
        speed = int(params['dps'] / 0.01)  # dps * 100
        data[4:8] = struct.pack("<I", speed)  # Little-endian int32
        send_can_frame(bus, tx_id, data)

    elif command == 'position':
        # 0xA4: Absolute position control
        data[0] = 0xA4
        max_speed = params['maxdps'] & 0xFFFF  # Ensure 16-bit max speed
        data[2:4] = struct.pack("<H", max_speed)
        angle = int(params['deg'] * 100)  # degrees to int
        data[4:8] = struct.pack("<I", angle)
        send_can_frame(bus, tx_id, data)

    else:
        print("Unknown command")

def main():
    # Argument parser for command-line inputs
    ap = argparse.ArgumentParser(description="Send CAN frames to KD240.")
    subparsers = ap.add_subparsers(dest='mode', required=True)

    # Raw CAN frame sender
    raw_parser = subparsers.add_parser('raw', help='Send raw CAN frame')
    raw_parser.add_argument('--iface', default='can0', help='CAN interface (default: can0)')
    raw_parser.add_argument('--id', type=lambda x: int(x, 0), required=True, help='CAN ID (e.g., 0x123)')
    raw_parser.add_argument('--data', nargs=8, required=True, help='Data bytes to send (e.g., 11 22 33 44 55 66 77 88)')
    raw_parser.set_defaults(func=send_raw_frame)

    # RMD motor commands
    rmd_parser = subparsers.add_parser('rmd', help='Send RMD motor commands')
    rmd_parser.add_argument('--iface', default='can0', help='CAN interface (default: can0)')
    rmd_parser.add_argument('--mid', type=int, required=True, help='Motor ID (1..32)')
    rmd_parser.add_argument('--command', choices=['status1', 'speed', 'position'], required=True, help='RMD command')
    rmd_parser.add_argument('--dps', type=float, help='Speed in dps for speed command')
    rmd_parser.add_argument('--deg', type=float, help='Angle in degrees for position command')
    rmd_parser.add_argument('--maxdps', type=int, default=500, help='Max speed in dps for position command')
    rmd_parser.set_defaults(func=create_rmd_frame)

    args = ap.parse_args()

    # Open the bus
    bus = open_bus(args.iface)

    if args.mode == 'raw':
        send_raw_frame(bus, args.iface, args.id, args.data)
    elif args.mode == 'rmd':
        params = {}
        if args.command == 'speed':
            params['dps'] = args.dps
        elif args.command == 'position':
            params['deg'] = args.deg
            params['maxdps'] = args.maxdps
        create_rmd_frame(bus, args.command, args.mid, params)

if __name__ == '__main__':
    main()
