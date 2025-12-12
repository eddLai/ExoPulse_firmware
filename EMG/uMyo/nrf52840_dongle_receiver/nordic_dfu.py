#!/usr/bin/env python3
"""
Nordic Serial DFU for nRF52840 USB Dongle (Open Bootloader)
Implements the full Nordic DFU protocol over serial SLIP
"""
import serial
import struct
import time
import sys
import os
import binascii
import json
import zipfile
from io import BytesIO

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
NRF_DFU_OP_HARDWARE_VERSION = 0x0A
NRF_DFU_OP_FIRMWARE_VERSION = 0x0B
NRF_DFU_OP_ABORT = 0x0C
NRF_DFU_OP_RESPONSE = 0x60

# Object types
NRF_DFU_OBJ_TYPE_COMMAND = 0x01
NRF_DFU_OBJ_TYPE_DATA = 0x02

# Results
NRF_DFU_RES_CODE_SUCCESS = 0x01
NRF_DFU_RES_CODE_INVALID = 0x02
NRF_DFU_RES_CODE_NOT_SUPPORTED = 0x03
NRF_DFU_RES_CODE_INVALID_PARAMETER = 0x04
NRF_DFU_RES_CODE_INSUFFICIENT_RESOURCES = 0x05
NRF_DFU_RES_CODE_INVALID_OBJECT = 0x06
NRF_DFU_RES_CODE_UNSUPPORTED_TYPE = 0x07
NRF_DFU_RES_CODE_OPERATION_NOT_PERMITTED = 0x08
NRF_DFU_RES_CODE_OPERATION_FAILED = 0x0A
NRF_DFU_RES_CODE_EXT_ERROR = 0x0B

class SlipSerial:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        self.ser.close()

    def encode_slip(self, data):
        """Encode data with SLIP framing"""
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
        """Decode SLIP framed data"""
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
        """Send SLIP encoded data"""
        encoded = self.encode_slip(data)
        self.ser.write(encoded)
        self.ser.flush()

    def recv(self, timeout=5):
        """Receive SLIP decoded data"""
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
        """Send request and wait for response"""
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
        if result != NRF_DFU_RES_CODE_SUCCESS:
            raise Exception(f"DFU error: 0x{result:02X}")

        return response[3:]

    def ping(self, id=1):
        """Ping bootloader"""
        request = bytes([NRF_DFU_OP_PING, id])
        self.slip.send(request)
        response = self.slip.recv(timeout=2)
        if response and len(response) >= 3:
            return response[2] == id
        return False

    def get_protocol_version(self):
        """Get protocol version"""
        response = self.send_request(NRF_DFU_OP_PROTOCOL_VERSION)
        if len(response) >= 1:
            return response[0]
        return 0

    def set_prn(self, prn=0):
        """Set Packet Receipt Notification value"""
        self.prn = prn
        self.send_request(NRF_DFU_OP_RECEIPT_NOTIF_SET, struct.pack('<H', prn))

    def get_mtu(self):
        """Get Maximum Transfer Unit"""
        response = self.send_request(NRF_DFU_OP_MTU_GET, struct.pack('<H', 256))
        if len(response) >= 2:
            self.mtu = struct.unpack('<H', response[:2])[0]
        return self.mtu

    def select_object(self, obj_type):
        """Select object type and get info"""
        response = self.send_request(NRF_DFU_OP_OBJECT_SELECT, bytes([obj_type]))
        if len(response) >= 12:
            max_size = struct.unpack('<I', response[0:4])[0]
            offset = struct.unpack('<I', response[4:8])[0]
            crc32 = struct.unpack('<I', response[8:12])[0]
            return max_size, offset, crc32
        return 0, 0, 0

    def create_object(self, obj_type, size):
        """Create object"""
        data = bytes([obj_type]) + struct.pack('<I', size)
        self.send_request(NRF_DFU_OP_OBJECT_CREATE, data)

    def write_data(self, data):
        """Write data (no response expected)"""
        request = bytes([NRF_DFU_OP_OBJECT_WRITE]) + data
        self.slip.send(request)

    def get_crc(self):
        """Get CRC of current object"""
        response = self.send_request(NRF_DFU_OP_CRC_GET)
        if len(response) >= 8:
            offset = struct.unpack('<I', response[0:4])[0]
            crc32 = struct.unpack('<I', response[4:8])[0]
            return offset, crc32
        return 0, 0

    def execute_object(self):
        """Execute current object"""
        self.send_request(NRF_DFU_OP_OBJECT_EXECUTE)

    def stream_data(self, data, crc, offset):
        """Stream data to device with write operations"""
        chunk_size = self.mtu - 1  # -1 for opcode
        if chunk_size > 244:  # Limit chunk size
            chunk_size = 244

        idx = 0
        while idx < len(data):
            chunk = data[idx:idx + chunk_size]
            self.write_data(chunk)

            idx += len(chunk)
            offset += len(chunk)
            crc = binascii.crc32(chunk, crc) & 0xFFFFFFFF

            # Small delay for processing
            time.sleep(0.003)

        return crc, offset

    def send_init_packet(self, init_data):
        """Send init packet (command object)"""
        print("  Sending init packet...")

        max_size, offset, crc = self.select_object(NRF_DFU_OBJ_TYPE_COMMAND)
        print(f"    Command object: max_size={max_size}, offset={offset}, crc=0x{crc:08X}")

        # Create command object
        self.create_object(NRF_DFU_OBJ_TYPE_COMMAND, len(init_data))

        # Stream init packet data
        crc, offset = self.stream_data(init_data, 0xFFFFFFFF, 0)

        # Verify CRC
        recv_offset, recv_crc = self.get_crc()
        expected_crc = binascii.crc32(init_data) & 0xFFFFFFFF
        print(f"    Written {recv_offset} bytes, CRC: 0x{recv_crc:08X} (expected: 0x{expected_crc:08X})")

        if recv_crc != expected_crc:
            raise Exception("CRC mismatch for init packet!")

        # Execute
        self.execute_object()
        print("    Init packet executed")
        time.sleep(0.5)

    def send_firmware(self, fw_data):
        """Send firmware (data object)"""
        print(f"  Sending firmware ({len(fw_data)} bytes)...")

        max_size, offset, crc = self.select_object(NRF_DFU_OBJ_TYPE_DATA)
        print(f"    Data object: max_size={max_size}, offset={offset}, crc=0x{crc:08X}")

        if max_size == 0:
            max_size = 4096  # Default

        # Send in chunks based on max object size
        total_sent = 0
        while total_sent < len(fw_data):
            chunk = fw_data[total_sent:total_sent + max_size]

            # Create data object for this chunk
            self.create_object(NRF_DFU_OBJ_TYPE_DATA, len(chunk))

            # Stream the chunk
            chunk_crc, _ = self.stream_data(chunk, 0xFFFFFFFF, 0)

            # Verify CRC
            recv_offset, recv_crc = self.get_crc()
            expected_crc = binascii.crc32(chunk) & 0xFFFFFFFF

            # Progress
            total_sent += len(chunk)
            progress = (total_sent * 100) // len(fw_data)
            print(f"    Progress: {progress}% ({total_sent}/{len(fw_data)} bytes)")

            if recv_crc != expected_crc:
                raise Exception(f"CRC mismatch at offset {total_sent}!")

            # Execute this chunk
            self.execute_object()
            time.sleep(0.1)

        print("    Firmware uploaded successfully!")


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

            if record_type == 0:  # Data
                full_addr = extended_addr + address
                for i in range(byte_count):
                    byte_val = int(line[9 + i*2:11 + i*2], 16)
                    data[full_addr + i] = byte_val
                    min_addr = min(min_addr, full_addr + i)
                    max_addr = max(max_addr, full_addr + i)
            elif record_type == 2:  # Extended segment address
                extended_addr = int(line[9:13], 16) << 4
            elif record_type == 4:  # Extended linear address
                extended_addr = int(line[9:13], 16) << 16
            elif record_type == 1:  # EOF
                break

    if min_addr > max_addr:
        return None, 0

    # Create binary
    result = bytearray(max_addr - min_addr + 1)
    for i in range(len(result)):
        result[i] = data.get(min_addr + i, 0xFF)

    return bytes(result), min_addr


def create_init_packet_pb(fw_size, fw_crc, fw_type=4, hw_version=52, fw_version=1, sd_req=[]):
    """
    Create init packet in protobuf format for Nordic Secure DFU
    This is a simplified version for Open Bootloader
    """
    # For Open Bootloader, we need a simple init packet format
    # The init packet contains firmware info

    # Simplified init packet format (not full protobuf):
    # Field 1 (fw_type): varint = 4 (APPLICATION)
    # Field 2 (app): contains firmware info

    # Using raw binary init packet format
    init_packet = bytearray()

    # Magic word for init packet
    # Format: [size_le16][data...]
    # Data contains firmware version, hardware version, SD requirements, etc.

    # Simple init command format (legacy)
    # This format works with Open Bootloader
    init_cmd = bytearray()

    # Device type (0xFFFF = any)
    init_cmd.extend(struct.pack('<H', 0xFFFF))
    # Device revision (0xFFFF = any)
    init_cmd.extend(struct.pack('<H', 0xFFFF))
    # Application version
    init_cmd.extend(struct.pack('<I', fw_version))
    # Softdevice array length
    init_cmd.extend(struct.pack('<H', len(sd_req)))
    # Softdevice requirements
    for sd in sd_req:
        init_cmd.extend(struct.pack('<H', sd))
    # CRC16 is not needed for Open Bootloader

    return bytes(init_cmd)


def create_dfu_package(hex_file, hw_version=52, fw_version=1):
    """Create DFU package with init packet and firmware"""
    # Convert hex to binary
    fw_data, start_addr = hex_to_bin(hex_file)
    if fw_data is None:
        raise Exception("Failed to convert HEX file")

    print(f"  Firmware size: {len(fw_data)} bytes, start: 0x{start_addr:08X}")

    # Calculate CRC32
    fw_crc = binascii.crc32(fw_data) & 0xFFFFFFFF
    print(f"  Firmware CRC32: 0x{fw_crc:08X}")

    # Create init packet
    init_packet = create_init_packet_pb(len(fw_data), fw_crc, hw_version=hw_version, fw_version=fw_version)

    return init_packet, fw_data


def main():
    port = "/dev/ttyACM0"
    hex_file = "build/nrf52840_dongle_receiver.hex"

    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3:
        hex_file = sys.argv[2]

    if not os.path.exists(hex_file):
        print(f"Error: {hex_file} not found")
        sys.exit(1)

    print(f"=== Nordic DFU Uploader ===")
    print(f"Port: {port}")
    print(f"Firmware: {hex_file}")

    # Create DFU package
    print("\nPreparing firmware...")
    init_packet, fw_data = create_dfu_package(hex_file)

    # Connect to bootloader
    print("\nConnecting to bootloader...")
    dfu = NordicDFU(port)

    try:
        # Ping
        if dfu.ping():
            print("  Bootloader responded to ping")
        else:
            print("  Warning: No ping response, continuing anyway...")

        # Get protocol version
        try:
            version = dfu.get_protocol_version()
            print(f"  Protocol version: {version}")
        except:
            print("  Could not get protocol version")

        # Setup
        dfu.set_prn(0)
        print(f"  PRN set to 0")

        mtu = dfu.get_mtu()
        print(f"  MTU: {mtu}")

        # Send init packet
        print("\nSending init packet...")
        dfu.send_init_packet(init_packet)

        # Send firmware
        print("\nSending firmware...")
        dfu.send_firmware(fw_data)

        print("\n=== DFU Complete! ===")
        print("The device should now boot with the new firmware.")
        print("The LED should start blinking if the firmware is working.")

    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        dfu.close()


if __name__ == "__main__":
    main()
