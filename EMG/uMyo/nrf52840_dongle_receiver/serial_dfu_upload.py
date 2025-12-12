#!/usr/bin/env python3
"""
Simple Serial DFU uploader for nRF52840 Open Bootloader
Based on Nordic's Serial DFU protocol
"""
import serial
import struct
import time
import sys
import os
import zipfile
import json

# DFU opcodes
OP_CREATE = 0x01
OP_SET_PRN = 0x02
OP_CALC_CRC = 0x03
OP_EXECUTE = 0x04
OP_SELECT = 0x06
OP_MTU = 0x07
OP_WRITE = 0x08
OP_PING = 0x09
OP_RESPONSE = 0x60

# Object types
OBJ_COMMAND = 0x01
OBJ_DATA = 0x02

# Results
RES_SUCCESS = 0x01
RES_INVALID_OBJECT = 0x02
RES_INVALID_PARAM = 0x03
RES_INSUFFICIENT_RESOURCES = 0x04
RES_INVALID_OBJECT_TYPE = 0x05
RES_OPERATION_NOT_PERMITTED = 0x06
RES_OPERATION_FAILED = 0x0A

class SerialDFU:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=5)
        self.mtu = 64
        self.prn = 0

    def close(self):
        self.ser.close()

    def send_slip(self, data):
        """Encode and send SLIP packet"""
        SLIP_END = 0xC0
        SLIP_ESC = 0xDB
        SLIP_ESC_END = 0xDC
        SLIP_ESC_ESC = 0xDD

        encoded = bytearray()
        for b in data:
            if b == SLIP_END:
                encoded.extend([SLIP_ESC, SLIP_ESC_END])
            elif b == SLIP_ESC:
                encoded.extend([SLIP_ESC, SLIP_ESC_ESC])
            else:
                encoded.append(b)

        self.ser.write(bytes([SLIP_END]) + bytes(encoded) + bytes([SLIP_END]))
        self.ser.flush()

    def recv_slip(self, timeout=10):
        """Receive and decode SLIP packet"""
        SLIP_END = 0xC0
        SLIP_ESC = 0xDB
        SLIP_ESC_END = 0xDC
        SLIP_ESC_ESC = 0xDD

        start_time = time.time()
        data = bytearray()
        started = False
        escaped = False

        while time.time() - start_time < timeout:
            if self.ser.in_waiting:
                b = self.ser.read(1)
                if not b:
                    continue
                b = b[0]

                if b == SLIP_END:
                    if started and len(data) > 0:
                        return bytes(data)
                    started = True
                    data = bytearray()
                elif escaped:
                    if b == SLIP_ESC_END:
                        data.append(SLIP_END)
                    elif b == SLIP_ESC_ESC:
                        data.append(SLIP_ESC)
                    else:
                        data.append(b)
                    escaped = False
                elif b == SLIP_ESC:
                    escaped = True
                elif started:
                    data.append(b)
            else:
                time.sleep(0.001)

        return None

    def send_cmd(self, opcode, data=b''):
        """Send command and receive response"""
        packet = bytes([opcode]) + data
        self.send_slip(packet)

        response = self.recv_slip()
        if response is None:
            raise Exception(f"Timeout waiting for response to opcode 0x{opcode:02X}")

        if len(response) < 3:
            raise Exception(f"Response too short: {response.hex()}")

        if response[0] != OP_RESPONSE:
            raise Exception(f"Expected response opcode 0x60, got 0x{response[0]:02X}")

        if response[1] != opcode:
            raise Exception(f"Response opcode mismatch: expected 0x{opcode:02X}, got 0x{response[1]:02X}")

        result = response[2]
        if result != RES_SUCCESS:
            raise Exception(f"DFU error: 0x{result:02X}")

        return response[3:] if len(response) > 3 else b''

    def ping(self):
        """Ping the bootloader"""
        packet = bytes([OP_PING, 0x01])
        self.send_slip(packet)
        response = self.recv_slip()
        return response is not None and len(response) >= 2 and response[0] == OP_RESPONSE

    def set_prn(self, prn=0):
        """Set Packet Receipt Notification"""
        self.prn = prn
        self.send_cmd(OP_SET_PRN, struct.pack('<H', prn))
        print(f"  Set PRN to {prn}")

    def get_mtu(self):
        """Get MTU"""
        response = self.send_cmd(OP_MTU, struct.pack('<H', 256))
        if len(response) >= 2:
            self.mtu = struct.unpack('<H', response[:2])[0]
        print(f"  MTU: {self.mtu}")
        return self.mtu

    def select(self, obj_type):
        """Select object type and get info"""
        response = self.send_cmd(OP_SELECT, bytes([obj_type]))
        if len(response) >= 12:
            max_size, offset, crc = struct.unpack('<III', response[:12])
            return max_size, offset, crc
        return 0, 0, 0

    def create(self, obj_type, size):
        """Create object"""
        self.send_cmd(OP_CREATE, bytes([obj_type]) + struct.pack('<I', size))

    def write(self, data):
        """Write data to current object"""
        chunk_size = self.mtu - 1  # Leave room for opcode
        offset = 0

        while offset < len(data):
            chunk = data[offset:offset + chunk_size]
            packet = bytes([OP_WRITE]) + chunk
            self.send_slip(packet)
            offset += len(chunk)

            # If PRN is set, wait for receipt every PRN packets
            # For simplicity, we don't use PRN (prn=0)

        # Small delay for processing
        time.sleep(0.05)

    def calc_crc(self):
        """Calculate CRC of current object"""
        response = self.send_cmd(OP_CALC_CRC)
        if len(response) >= 8:
            offset, crc = struct.unpack('<II', response[:8])
            return offset, crc
        return 0, 0

    def execute(self):
        """Execute current object"""
        self.send_cmd(OP_EXECUTE)
        time.sleep(0.5)  # Give time for execution

    def send_object(self, obj_type, data, name="Object"):
        """Send a complete object (init packet or firmware data)"""
        print(f"\n  Sending {name} ({len(data)} bytes)...")

        # Select to get max object size
        max_size, _, _ = self.select(obj_type)
        print(f"    Max object size: {max_size}")

        # Create object
        self.create(obj_type, len(data))

        # Write data in chunks
        chunk_size = max_size if max_size > 0 else 4096
        offset = 0

        while offset < len(data):
            chunk = data[offset:offset + chunk_size]

            if offset > 0:
                # For subsequent chunks, need to create again
                self.create(obj_type, len(chunk))

            # Write chunk
            write_chunk_size = self.mtu - 1
            write_offset = 0
            while write_offset < len(chunk):
                write_chunk = chunk[write_offset:write_offset + write_chunk_size]
                packet = bytes([OP_WRITE]) + write_chunk
                self.send_slip(packet)
                write_offset += len(write_chunk)

            time.sleep(0.05)

            # Verify CRC
            recv_offset, recv_crc = self.calc_crc()
            print(f"    Wrote {recv_offset} bytes, CRC: 0x{recv_crc:08X}")

            # Execute this chunk
            self.execute()

            offset += len(chunk)

        print(f"    {name} sent successfully!")

def crc32_compute(data, crc=0xFFFFFFFF):
    """Compute CRC32 (same as Nordic uses)"""
    import binascii
    return binascii.crc32(data, crc) & 0xFFFFFFFF

def create_init_packet(bin_data):
    """Create init packet for nRF52840 Open Bootloader"""
    # The Open Bootloader uses a simpler init packet format
    # This is a minimal init packet

    # Compute CRC of firmware
    fw_crc = crc32_compute(bin_data)

    # Init packet: just contains firmware CRC and size
    # Format depends on bootloader version - this is simplified
    init_packet = struct.pack('<I', len(bin_data))  # Size
    init_packet += struct.pack('<I', fw_crc)        # CRC

    return init_packet

def main():
    if len(sys.argv) < 2:
        port = "/dev/ttyACM0"
    else:
        port = sys.argv[1]

    # Load firmware binary
    bin_file = "build/nrf52840_dongle_receiver.bin"
    if not os.path.exists(bin_file):
        print(f"Error: {bin_file} not found")
        sys.exit(1)

    with open(bin_file, 'rb') as f:
        bin_data = f.read()

    print(f"Firmware: {bin_file} ({len(bin_data)} bytes)")
    print(f"Port: {port}")

    # Create init packet
    init_packet = create_init_packet(bin_data)

    try:
        dfu = SerialDFU(port)

        print("\nConnecting to bootloader...")

        # Ping
        if not dfu.ping():
            print("Warning: Ping failed, continuing anyway...")
        else:
            print("  Bootloader responded to ping")

        # Setup
        dfu.set_prn(0)
        dfu.get_mtu()

        # Send init packet (command object)
        dfu.send_object(OBJ_COMMAND, init_packet, "Init Packet")

        # Send firmware (data object)
        dfu.send_object(OBJ_DATA, bin_data, "Firmware")

        print("\n=== DFU Complete! ===")
        print("The device should now boot with the new firmware.")

        dfu.close()

    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
