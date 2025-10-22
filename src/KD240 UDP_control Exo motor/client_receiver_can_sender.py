#!/usr/bin/env python3
import can
import struct
import argparse
import time
import socket
import json

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

    elif command == 'torque':
        # 0xA1: Torque control (iqControl in 0.01 A/LSB)
        data[0] = 0xA1
        iq_ampere = params['iq']  # torque in amperes
        iq_cnt = int(iq_ampere * 100.0)  # Convert to 0.01 A/LSB
        data[4:6] = struct.pack("<h", iq_cnt)  # Little-endian int16
        send_can_frame(bus, tx_id, data)

    else:
        print("Unknown command")

def receive_udp_data(ip, port, bus, interface):
    """ Listen for UDP data, parse it, and send CAN frames. """
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((ip, port))
    print(f"Listening for UDP packets on {ip}:{port}...")

    while True:
        try:
            # Receive packet
            data, addr = udp_socket.recvfrom(1024)  # buffer size
            packet = json.loads(data.decode())

            # Print received packet for debugging
            print(f"Received packet from {addr}: {packet}")

            # Parse received JSON
            if "right_hip" in packet and "left_hip" in packet:
                # Translate angles to torque commands (if using angles)
                right_hip = packet["right_hip"]
                left_hip = packet["left_hip"]
                right_torque = right_hip * 0.02  # simple scaling factor
                left_torque = left_hip * 0.02   # simple scaling factor

                # Create RMD command and send via CAN
                create_rmd_frame(bus, "torque", 1, {"iq": right_torque})  # Right motor torque
                create_rmd_frame(bus, "torque", 2, {"iq": left_torque})  # Left motor torque

            elif "command" in packet:
                # Handle RMD motor command
                motor_id = packet.get("motor_id", 1)  # default to motor 1 if not provided
                command = packet["command"]
                params = packet.get("params", {})

                create_rmd_frame(bus, command, motor_id, params)

        except Exception as e:
            print(f"Error receiving or processing packet: {e}")
            continue

def main():
    # Argument parser for command-line inputs
    ap = argparse.ArgumentParser(description="Receive UDP data and send CAN frames to KD240.")
    ap.add_argument('--iface', default='can0', help='CAN interface (default: can0)')
    ap.add_argument('--ip', default='0.0.0.0', help='IP address to listen for UDP packets (default: 192.168.0.150)')
    ap.add_argument('--port', type=int, default=4210, help='UDP port (default: 4210)')
    args = ap.parse_args()

    # Open the CAN bus
    bus = open_bus(args.iface)

    # Start receiving UDP data and send corresponding CAN frames
    receive_udp_data(args.ip, args.port, bus, args.iface)

if __name__ == '__main__':
    main()