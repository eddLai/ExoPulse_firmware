#!/usr/bin/env python3
"""
Create a simple DFU package for nRF52840 Open Bootloader
"""
import zipfile
import json
import struct
import os

def hex_to_bin(hex_file, bin_file):
    """Convert Intel HEX to binary"""
    data = {}
    min_addr = 0xFFFFFFFF
    max_addr = 0
    
    with open(hex_file, 'r') as f:
        extended_addr = 0
        for line in f:
            line = line.strip()
            if not line.startswith(':'):
                continue
            
            byte_count = int(line[1:3], 16)
            address = int(line[3:7], 16)
            record_type = int(line[7:9], 16)
            
            if record_type == 0:  # Data record
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
        return None
    
    # Create binary, filling gaps with 0xFF
    bin_data = bytearray(max_addr - min_addr + 1)
    for i in range(len(bin_data)):
        bin_data[i] = data.get(min_addr + i, 0xFF)
    
    with open(bin_file, 'wb') as f:
        f.write(bin_data)
    
    return min_addr

def create_init_packet(app_size, hw_version=52, fw_version=1, sd_req=0):
    """Create init packet for DFU"""
    # Simple init packet format for Open Bootloader
    # This is a minimal init packet without signature
    
    # Init packet structure (simplified):
    # - Device type (uint16): 0xFFFF = any
    # - Device revision (uint16): 0xFFFF = any  
    # - Application version (uint32)
    # - SoftDevice count (uint16)
    # - SoftDevice array (uint16 each)
    # - CRC16 of image
    
    init_data = bytearray()
    init_data.extend(struct.pack('<H', 0xFFFF))  # Device type
    init_data.extend(struct.pack('<H', 0xFFFF))  # Device revision
    init_data.extend(struct.pack('<I', fw_version))  # App version
    init_data.extend(struct.pack('<H', 1))  # SoftDevice count
    init_data.extend(struct.pack('<H', sd_req))  # Required SD (0 = none)
    # CRC would go here but Open Bootloader might not require it
    
    return init_data

def create_manifest(app_bin_name, app_dat_name):
    """Create manifest.json for DFU package"""
    manifest = {
        "manifest": {
            "application": {
                "bin_file": app_bin_name,
                "dat_file": app_dat_name
            }
        }
    }
    return json.dumps(manifest, indent=2)

def main():
    hex_file = "build/nrf52840_dongle_receiver.hex"
    zip_file = "build/nrf52840_dongle_receiver.zip"
    
    # Convert hex to bin
    bin_file = "build/nrf52840_dongle_receiver.bin"
    start_addr = hex_to_bin(hex_file, bin_file)
    print(f"Converted HEX to BIN, start address: 0x{start_addr:08X}")
    
    # Get binary size
    bin_size = os.path.getsize(bin_file)
    print(f"Binary size: {bin_size} bytes")
    
    # Create init packet
    init_data = create_init_packet(bin_size)
    init_file = "build/nrf52840_dongle_receiver.dat"
    with open(init_file, 'wb') as f:
        f.write(init_data)
    
    # Create manifest
    manifest = create_manifest("nrf52840_dongle_receiver.bin", "nrf52840_dongle_receiver.dat")
    
    # Create ZIP package
    with zipfile.ZipFile(zip_file, 'w', zipfile.ZIP_DEFLATED) as zf:
        zf.write(bin_file, "nrf52840_dongle_receiver.bin")
        zf.write(init_file, "nrf52840_dongle_receiver.dat")
        zf.writestr("manifest.json", manifest)
    
    print(f"Created DFU package: {zip_file}")
    print(f"Upload with: nrfutil dfu serial -pkg {zip_file} -p /dev/ttyACM0")

if __name__ == "__main__":
    main()
