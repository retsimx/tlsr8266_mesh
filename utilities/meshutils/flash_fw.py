#!/usr/bin/env python3
"""
Firmware Update Utility for TLSR8266 Mesh Light System

This script implements the firmware update protocol for TLSR8266 mesh devices.
It handles device authentication, firmware version checking, and secure OTA updates
over BLE.

The update process consists of the following phases:
1. Authentication and Session Key Establishment
2. Firmware Version Verification
3. Chunked Firmware Transfer
4. Update Verification

Usage:
  python flash_fw.py --fw_file <file> --mac <addr> --mesh_name <name> --mesh_password <pwd> [--force]

Examples:
  # Update firmware on device with MAC 11:22:33:44:55:66
  python flash_fw.py --fw_file firmware.bin --mac 11:22:33:44:55:66 --mesh_name myhome --mesh_password securepass

  # Force update even if firmware version is older
  python flash_fw.py --fw_file firmware.bin --mac 11:22:33:44:55:66 --mesh_name myhome --mesh_password securepass --force
"""

import asyncio
import struct
import sys
import argparse
from asyncio import sleep

from bleak import BleakClient

from mesh_common import (
    # Constants
    mesh_name, mesh_pass, shared_key, VERSION_MASK, VERSION_16MHZ,
    pair_characteristic_uuid, notify_characteristic_uuid,
    command_characteristic_uuid, ota_characteristic_uuid,
    firmware_revision_uuid, PAIR_OP_VERIFY_CREDENTIALS,
    # Functions
    encode_mesh_credentials, encrypt_data, reverse_section,
    process_session_key, PacketParser, crc16
)

# Constants for firmware update
PACKET_SIZE = 16          # Size of each firmware packet
PACKET_HEADER_SIZE = 4    # Size of packet header (2 bytes index + 2 bytes CRC)

class BLEPacketParser(PacketParser):
    """
    BLE-specific implementation of PacketParser for OTA updates over BLE.
    
    This class generates packets with appropriate headers and CRC for the BLE OTA protocol.
    """
    def __init__(self, data):
        """
        Initialize the parser with firmware data.
        
        Args:
            data (bytes): The firmware binary data
        """
        super().__init__(data, PACKET_SIZE)

    def get_packet(self, index):
        """
        Generate a firmware packet for the given index.
        
        This method:
        1. Extracts the appropriate chunk of firmware data
        2. Adds packet header (index)
        3. Fills remaining space with 0xFF
        4. Calculates and adds CRC
        
        Args:
            index (int): The packet index to generate
            
        Returns:
            bytearray: Complete firmware packet
        """
        length = len(self.data)
        
        # Calculate packet size (16 bytes, or less for final packet)
        if length > self.packet_size:
            if index + 1 == self.total:
                packet_size = length - index * self.packet_size
            else:
                packet_size = self.packet_size
        else:
            packet_size = length

        # Add 4 bytes for header and CRC
        packet_size += PACKET_HEADER_SIZE

        # Create packet with 0xFF fill
        packet = [0xFF] * 20
        
        # Copy firmware data chunk
        packet[2:2+packet_size-PACKET_HEADER_SIZE] = self.data[index*self.packet_size:index*self.packet_size+packet_size-PACKET_HEADER_SIZE]

        # Add packet index and CRC
        self.fill_index(packet, index)
        calc_crc = crc16(packet)
        self.fill_crc(packet, calc_crc)

        return bytearray(packet)

    def fill_crc(self, packet, crc):
        """
        Add CRC to packet trailer.
        
        Args:
            packet (list): Packet to modify
            crc (int): CRC value
        """
        offset = len(packet) - 2
        packet[offset] = crc & 0xFF
        packet[offset+1] = (crc >> 8) & 0xFF


async def main():
    """
    Main function to update device firmware.
    
    This function:
    1. Parses command line arguments
    2. Authenticates with the device
    3. Verifies firmware compatibility
    4. Transfers firmware in chunks
    5. Verifies successful update
    """
    # Parse command line arguments
    args_parser = argparse.ArgumentParser(description='TLSR8266 Mesh Device Firmware Update Tool')
    args_parser.add_argument("--fw_file", help="Firmware file to flash", required=True, type=str)
    args_parser.add_argument("--mac", help="MAC address of device to flash", required=True, type=str)
    args_parser.add_argument('--mesh_name', help="The name of the mesh. eg `mymesh`. (Must be 16 bytes or fewer)", required=True, type=str)
    args_parser.add_argument('--mesh_password', help="The password of the mesh. eg `mymeshpwd` (Must be 16 bytes or fewer)", required=True, type=str)
    args_parser.add_argument("--force", help="Force update even if firmware version is older", required=False, default=False, action=argparse.BooleanOptionalAction)

    args = args_parser.parse_args()

    print(f"Trying to flash {args.fw_file} to device {args.mac}")

    try:
        print("Trying to connect to device...")
        async with BleakClient(args.mac) as client:
            # Read and parse firmware file
            with open(args.fw_file, 'rb') as f:
                parser = BLEPacketParser(f.read())

            # ----- PHASE 1: VERSION CHECK -----
            print("Reading firmware version from device...")
            dev_ver = await client.read_gatt_char(firmware_revision_uuid)

            # Extract version info from device and firmware
            dev_ver = struct.unpack("I", dev_ver[:4])[0]
            fw_ver = struct.unpack("I", parser.data[2:6])[0]

            # Check crystal frequency compatibility
            dev_16mhz = (dev_ver >> 31) == 1
            dev_ver = dev_ver & VERSION_MASK

            fw_16mhz = (fw_ver >> 31) == 1
            fw_ver = fw_ver & VERSION_MASK

            print(f"Device version: {dev_ver}, new version: {fw_ver}")

            # Validate crystal frequency compatibility
            if dev_16mhz != fw_16mhz:
                print(f"Attempt to flash incompatible crystal version of firmware. Bailing out. Device is 16MHz? "
                      f"{dev_16mhz}, FW is 16MHz? {fw_16mhz}", file=sys.stderr)
                if not args.force:
                    print("Use --force to override this safety check.")
                    return 1
                else:
                    print("Forcing flash despite crystal mismatch")

            # Check version numbers unless forced
            if not args.force:
                if dev_ver >= fw_ver:
                    print("Device firmware is newer or the same as the provided firmware file. Nothing to do.")
                    print("Use --force to override this safety check.")
                    return 2
            else:
                print("Forcing flash as requested")

            # ----- PHASE 2: AUTHENTICATION -----
            print("Authenticating with device...")

            # Set mesh credentials from arguments
            global mesh_name, mesh_pass
            mesh_name = args.mesh_name
            mesh_pass = args.mesh_password

            # Generate and verify mesh credentials
            plaintext_mesh_credentials = encode_mesh_credentials()

            # Prepare authentication request
            enc_key = bytearray(shared_key)[:] + b'\0' * 8  # Pad shared key to 16 bytes
            encrypted = encrypt_data(enc_key, plaintext_mesh_credentials)

            # Build authentication command with PAIR_OP_VERIFY_CREDENTIALS opcode
            command_data = bytearray(b'\0' * 17)
            command_data[0] = PAIR_OP_VERIFY_CREDENTIALS  # Verify credentials opcode
            command_data[1:1 + len(shared_key)] = shared_key[:]  # Client random challenge
            command_data[9:9 + 8] = encrypted[8:]  # Proof of credentials

            # Adjust byte order for device compatibility
            reverse_section(command_data, 9, 16)

            # Send authentication request and process server response
            await client.write_gatt_char(pair_characteristic_uuid, command_data)
            response = await client.read_gatt_char(pair_characteristic_uuid)
            
            # Extract session key from response
            session_key = process_session_key(response, plaintext_mesh_credentials)

            print("Authentication successful, session established")

            # ----- PHASE 3: FIRMWARE TRANSFER -----
            print("Starting firmware transfer...")

            # Send first packet
            await client.write_gatt_char(ota_characteristic_uuid, parser.get_next_packet())

            # Track and display progress
            completion = int(parser.index / parser.total * 100)
            while parser.has_next_packet():
                new_completion = int(parser.index / parser.total * 100)
                if new_completion != completion:
                    completion = new_completion
                    print(f"Progress: {completion}%")
                    # Get acknowledgment for current packet before sending next
                    await client.read_gatt_char(ota_characteristic_uuid)

                await client.write_gatt_char(ota_characteristic_uuid, parser.get_next_packet())

            # ----- PHASE 4: UPDATE VERIFICATION -----
            print("Transfer complete, verifying update...")
            # Send final packet and verify
            await client.write_gatt_char(ota_characteristic_uuid, parser.get_next_packet())
            await client.read_gatt_char(ota_characteristic_uuid)
            
            print(f"Firmware update completed successfully for device {args.mac}.")
            return 0
            
    except Exception as e:
        # Handle common BLE connection errors gracefully
        print(f"Error during firmware update: {e}", file=sys.stderr)
        return 3

# Run the main function with proper error handling
if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
