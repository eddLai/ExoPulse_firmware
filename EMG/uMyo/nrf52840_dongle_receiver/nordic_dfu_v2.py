#!/usr/bin/env python3
"""
Nordic Serial DFU for nRF52840 USB Dongle (Open Bootloader)
Uses protobuf format for init packet (without signing for open bootloader)
"""
import serial
import struct
import time
import sys
import os
import binascii
import hashlib

# SLIP encoding
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

# DFU opcodes
NRF_DFU_OP_PROTOCOL_VERSION = 0x00
NRF_DFU_OP_OBJECT_CREATE = 0x01
NRF_DFU_OP_RECEIPT_NOTIF_SET = 0x02
NRF_DFU_OP_CRC_GET = 0x03
NRF_DFU_OP_OBJECT_EXECUTE = 0x04
NRF_DFU_OP_OBJECT_SELECT = 0x06
NRF_DFU_OP_MTU_GET = 0x07
NRF_DFU_OP_OBJECT_WRITE = 0x08
NRF_DFU_OP_PING = 0x09
NRF_DFU_OP_RESPONSE = 0x60

# Object types
NRF_DFU_OBJ_TYPE_COMMAND = 0x01
NRF_DFU_OBJ_TYPE_DATA = 0x02

class SlipSerial:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()

    def encode_slip(self, data):
        result = bytearray([SLIP_END])
        for b in data:
            if b == SLIP_END:
                result.extend([SLIP_ESC, SLIP_ESC_END])
            elif b == SLIP_ESC:
                result.extend([SLIP_ESC, SLIP_ESC_ESC])
            else:
                result.append(b)
        result.append(SLIP_END)
        return bytes(result)

    def decode_slip(self, data):
        result = bytearray()
        escape = False
        for b in data:
            if escape:
                if b == SLIP_ESC_END:
                    result.append(SLIP_END)
                elif b == SLIP_ESC_ESC:
                    result.append(SLIP_ESC)
                else:
                    result.append(b)
                escape = False
            elif b == SLIP_ESC:
                escape = True
            elif b != SLIP_END:
                result.append(b)
        return bytes(result)

    def send(self, data):
        encoded = self.encode_slip(data)
        self.ser.write(encoded)
        self.ser.flush()

    def recv(self, timeout=5):
        start = time.time()
        buffer = bytearray()
        in_packet = False

        while time.time() - start < timeout:
            if self.ser.in_waiting:
                b = self.ser.read(1)
                if not b:
                    continue
                b = b[0]

                if b == SLIP_END:
                    if in_packet and len(buffer) > 0:
                        return self.decode_slip(bytes(buffer))
                    in_packet = True
                    buffer = bytearray()
                elif in_packet:
                    buffer.append(b)
            else:
                time.sleep(0.001)

        return None


class NordicDFU:
    def __init__(self, port):
        self.slip = SlipSerial(port)
        self.mtu = 64
        self.prn = 0

    def close(self):
        self.slip.close()

    def send_request(self, opcode, data=b''):
        request = bytes([opcode]) + data
        self.slip.send(request)

        response = self.slip.recv(timeout=10)
        if response is None:
            raise Exception(f"Timeout waiting for response to 0x{opcode:02X}")

        if len(response) < 3:
            raise Exception(f"Response too short: {response.hex()}")

        if response[0] != NRF_DFU_OP_RESPONSE:
            raise Exception(f"Invalid response opcode: 0x{response[0]:02X}")

        if response[1] != opcode:
            raise Exception(f"Response mismatch: expected 0x{opcode:02X}, got 0x{response[1]:02X}")

        result = response[2]
        if result != 0x01:  # SUCCESS
            raise Exception(f"DFU error: 0x{result:02X}")

        return response[3:]

    def ping(self, id=1):
        request = bytes([NRF_DFU_OP_PING, id])
        self.slip.send(request)
        response = self.slip.recv(timeout=2)
        if response and len(response) >= 3:
            return response[2] == id
        return False

    def get_protocol_version(self):
        response = self.send_request(NRF_DFU_OP_PROTOCOL_VERSION)
        if len(response) >= 1:
            return response[0]
        return 0

    def set_prn(self, prn=0):
        self.prn = prn
        self.send_request(NRF_DFU_OP_RECEIPT_NOTIF_SET, struct.pack('<H', prn))

    def get_mtu(self):
        response = self.send_request(NRF_DFU_OP_MTU_GET, struct.pack('<H', 256))
        if len(response) >= 2:
            self.mtu = struct.unpack('<H', response[:2])[0]
        return self.mtu

    def select_object(self, obj_type):
        response = self.send_request(NRF_DFU_OP_OBJECT_SELECT, bytes([obj_type]))
        if len(response) >= 12:
            max_size = struct.unpack('<I', response[0:4])[0]
            offset = struct.unpack('<I', response[4:8])[0]
            crc32 = struct.unpack('<I', response[8:12])[0]
            return max_size, offset, crc32
        return 0, 0, 0

    def create_object(self, obj_type, size):
        data = bytes([obj_type]) + struct.pack('<I', size)
        self.send_request(NRF_DFU_OP_OBJECT_CREATE, data)

    def write_data(self, data):
        request = bytes([NRF_DFU_OP_OBJECT_WRITE]) + data
        self.slip.send(request)

    def get_crc(self):
        response = self.send_request(NRF_DFU_OP_CRC_GET)
        if len(response) >= 8:
            offset = struct.unpack('<I', response[0:4])[0]
            crc32 = struct.unpack('<I', response[4:8])[0]
            return offset, crc32
        return 0, 0

    def execute_object(self):
        self.send_request(NRF_DFU_OP_OBJECT_EXECUTE)

    def stream_data(self, data):
        chunk_size = min(self.mtu - 1, 244)
        idx = 0
        while idx < len(data):
            chunk = data[idx:idx + chunk_size]
            self.write_data(chunk)
            idx += len(chunk)
            time.sleep(0.003)

    def send_init_packet(self, init_data):
        print("  Sending init packet...")

        max_size, offset, crc = self.select_object(NRF_DFU_OBJ_TYPE_COMMAND)
        print(f"    Command object: max_size={max_size}, offset={offset}")

        self.create_object(NRF_DFU_OBJ_TYPE_COMMAND, len(init_data))
        self.stream_data(init_data)

        recv_offset, recv_crc = self.get_crc()
        expected_crc = binascii.crc32(init_data) & 0xFFFFFFFF
        print(f"    Written {recv_offset} bytes, CRC: 0x{recv_crc:08X}")

        if recv_crc != expected_crc:
            raise Exception("CRC mismatch!")

        self.execute_object()
        print("    Init packet executed")
        time.sleep(0.3)

    def send_firmware(self, fw_data):
        print(f"  Sending firmware ({len(fw_data)} bytes)...")

        max_size, offset, crc = self.select_object(NRF_DFU_OBJ_TYPE_DATA)
        print(f"    Data object: max_size={max_size}")

        if max_size == 0:
            max_size = 4096

        total_sent = 0
        while total_sent < len(fw_data):
            chunk = fw_data[total_sent:total_sent + max_size]

            self.create_object(NRF_DFU_OBJ_TYPE_DATA, len(chunk))
            self.stream_data(chunk)

            recv_offset, recv_crc = self.get_crc()
            expected_crc = binascii.crc32(chunk) & 0xFFFFFFFF

            total_sent += len(chunk)
            progress = (total_sent * 100) // len(fw_data)
            print(f"    Progress: {progress}%")

            if recv_crc != expected_crc:
                raise Exception(f"CRC mismatch!")

            self.execute_object()
            time.sleep(0.1)

        print("    Firmware uploaded!")


def hex_to_bin(hex_file):
    """Convert Intel HEX to binary"""
    data = {}
    min_addr = 0xFFFFFFFF
    max_addr = 0
    extended_addr = 0

    with open(hex_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line.startswith(':'):
                continue

            byte_count = int(line[1:3], 16)
            address = int(line[3:7], 16)
            record_type = int(line[7:9], 16)

            if record_type == 0:
                full_addr = extended_addr + address
                for i in range(byte_count):
                    byte_val = int(line[9 + i*2:11 + i*2], 16)
                    data[full_addr + i] = byte_val
                    min_addr = min(min_addr, full_addr + i)
                    max_addr = max(max_addr, full_addr + i)
            elif record_type == 2:
                extended_addr = int(line[9:13], 16) << 4
            elif record_type == 4:
                extended_addr = int(line[9:13], 16) << 16
            elif record_type == 1:
                break

    if min_addr > max_addr:
        return None, 0

    result = bytearray(max_addr - min_addr + 1)
    for i in range(len(result)):
        result[i] = data.get(min_addr + i, 0xFF)

    return bytes(result), min_addr


def encode_varint(value):
    """Encode integer as protobuf varint"""
    result = bytearray()
    while value > 127:
        result.append((value & 0x7F) | 0x80)
        value >>= 7
    result.append(value)
    return bytes(result)


def encode_field(field_num, wire_type, value):
    """Encode a protobuf field"""
    tag = (field_num << 3) | wire_type
    return encode_varint(tag) + value


def create_init_packet_protobuf(fw_size, fw_hash, hw_version=52, fw_version=1):
    """
    Create init packet in protobuf format for nRF DFU

    Message structure (dfu-cc.proto):
    message Packet {
        optional Command command = 1;
        optional SignedCommand signed_command = 2;
    }

    message Command {
        optional OpCode op_code = 1;
        optional InitCommand init = 2;
    }

    message InitCommand {
        optional uint32 fw_version = 1;
        optional uint32 hw_version = 2;
        repeated uint32 sd_req = 3;
        optional FwType type = 4;
        optional uint32 sd_size = 5;
        optional uint32 bl_size = 6;
        optional uint32 app_size = 7;
        optional Hash hash = 8;
        optional bool is_debug = 9;
    }

    message Hash {
        required HashType hash_type = 1;
        required bytes hash = 2;
    }
    """

    # Build InitCommand
    init_cmd = bytearray()

    # Field 1: fw_version (varint)
    init_cmd.extend(encode_field(1, 0, encode_varint(fw_version)))

    # Field 2: hw_version (varint)
    init_cmd.extend(encode_field(2, 0, encode_varint(hw_version)))

    # Field 3: sd_req (repeated varint) - no softdevice required
    # For open bootloader with no SD, use 0x00 or skip
    init_cmd.extend(encode_field(3, 0, encode_varint(0x00)))

    # Field 4: type (varint) - APPLICATION = 4
    init_cmd.extend(encode_field(4, 0, encode_varint(4)))

    # Field 7: app_size (varint)
    init_cmd.extend(encode_field(7, 0, encode_varint(fw_size)))

    # Field 8: hash (embedded message)
    # HashType: SHA256 = 3
    hash_msg = bytearray()
    hash_msg.extend(encode_field(1, 0, encode_varint(3)))  # hash_type = SHA256
    hash_msg.extend(encode_field(2, 2, bytes([len(fw_hash)]) + fw_hash))  # hash bytes

    init_cmd.extend(encode_field(8, 2, bytes([len(hash_msg)]) + hash_msg))

    # Build Command
    command = bytearray()
    # Field 1: op_code (varint) - INIT = 1
    command.extend(encode_field(1, 0, encode_varint(1)))
    # Field 2: init (embedded message)
    command.extend(encode_field(2, 2, bytes([len(init_cmd)]) + init_cmd))

    # Build Packet
    packet = bytearray()
    # Field 1: command (embedded message)
    packet.extend(encode_field(1, 2, bytes([len(command)]) + command))

    return bytes(packet)


def main():
    port = "/dev/ttyACM0"
    hex_file = "build/led_blink_test.hex"

    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3:
        hex_file = sys.argv[2]

    if not os.path.exists(hex_file):
        print(f"Error: {hex_file} not found")
        sys.exit(1)

    print(f"=== Nordic DFU Uploader v2 ===")
    print(f"Port: {port}")
    print(f"Firmware: {hex_file}")

    # Convert hex to binary
    print("\nPreparing firmware...")
    fw_data, start_addr = hex_to_bin(hex_file)
    if fw_data is None:
        print("Failed to convert HEX file")
        sys.exit(1)

    print(f"  Size: {len(fw_data)} bytes, start: 0x{start_addr:08X}")

    # Calculate SHA256 hash
    fw_hash = hashlib.sha256(fw_data).digest()
    print(f"  SHA256: {fw_hash.hex()[:32]}...")

    # Create init packet
    init_packet = create_init_packet_protobuf(len(fw_data), fw_hash)
    print(f"  Init packet: {len(init_packet)} bytes")

    # Connect
    print("\nConnecting to bootloader...")
    dfu = NordicDFU(port)

    try:
        if dfu.ping():
            print("  Ping OK")
        else:
            print("  Warning: No ping response")

        dfu.set_prn(0)
        print("  PRN set")

        mtu = dfu.get_mtu()
        print(f"  MTU: {mtu}")

        print("\nUploading...")
        dfu.send_init_packet(init_packet)
        dfu.send_firmware(fw_data)

        print("\n=== DFU Complete! ===")
        print("The LED should start blinking now!")

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        dfu.close()


if __name__ == "__main__":
    main()
