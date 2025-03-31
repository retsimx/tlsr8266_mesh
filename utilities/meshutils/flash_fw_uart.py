#!/usr/bin/env python3
"""
UART-based Firmware Update Utility for TLSR8266 Mesh Light System

This script implements a firmware update protocol for TLSR8266 mesh devices over UART.
It handles firmware version checking and OTA updates through a serial connection.

The update process consists of the following phases:
1. UART Communication Initialization
2. Firmware Version Verification
3. Crystal Frequency Compatibility Check
4. Chunked Firmware Transfer

Usage:
  python flash_fw_uart.py --fw_file <file> --mesh_address <addr> [--force]

Examples:
  # Update firmware on device with mesh address 5
  python flash_fw_uart.py --fw_file firmware.bin --mesh_address 5

  # Force update even if firmware version is older
  python flash_fw_uart.py --fw_file firmware.bin --mesh_address 5 --force
"""

import argparse
import datetime
import struct
import sys
import threading
from time import sleep

import serial

from mesh_common import PacketParser, crc16, VERSION_MASK, VERSION_16MHZ

# UART Protocol Constants
# Packet structure:
# [0] = counter byte
# [1] = command type (or 0xFF for ACK)
# [2] = command code (for normal packets) or unused (for ACK)
# [3-42] = data payload (40 bytes)
# [42-43] = CRC16 of bytes 0-42

# Command codes
ENABLE_UART = 0x01    # Enable UART reporting mode
LIGHT_CTRL = 0x02     # Light control command
LIGHT_STATUS = 0x03   # Light status response
MESH_MESSAGE = 0x04   # Mesh network message
PANIC_MESSAGE = 0x05  # Panic/error message
PRINT_MESSAGE = 0x06  # Debug print message
ACK = 0xFF            # Acknowledgment code

# OTA update command codes
OTA_START = 0x24      # Start OTA update process
OTA_DATA = 0x26       # OTA data packet
OTA_END = 0x28        # End OTA update process

# OTA response codes
OTA_START_RESP = 0x25 # Response to OTA start
OTA_DATA_RESP = 0x27  # Response to OTA data packet

# Timing and packet constants
ACK_TIMEOUT = 0.2     # Timeout for ACK reception in seconds
PACKET_SIZE = 8       # Size of each firmware data packet
PACKET_LENGTH = 44    # Total UART packet length
CRC_OFFSET = 42       # Offset of CRC in packet
UART_BAUDRATE = 115200 # Serial connection baudrate
PAYLOAD_OFFSET = 3    # Offset of payload in packets
MAX_PAYLOAD_SIZE = 15  # Maximum size of payload data in LIGHT_CTRL packets
DEVICE_VER_OFFSET = 13 # Offset of device version in response packet

# Global variables
counter = 0
connection = None
events = {}
last_packet = None
last_packet_wait = threading.Event()


class UARTPacketParser(PacketParser):
    """
    UART-specific implementation of PacketParser for OTA updates over UART.
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
        
        Args:
            index (int): The packet index to generate
            
        Returns:
            bytes: Complete firmware packet
        """
        length = len(self.data)
        size = self.packet_size

        # Calculate packet size (standard or smaller for last packet)
        if length > size:
            if index + 1 == self.total:
                packet_size = length - index * size
            else:
                packet_size = size
        else:
            packet_size = length

        # Include index in packet size
        packet_size += 2

        # Create packet with 0xFF fill
        packet = [0xFF] * 10
        
        # Copy firmware data chunk
        packet[2:2+packet_size-2] = self.data[index*size:index*size+packet_size-2]

        # Add packet index
        self.fill_index(packet, index)

        return bytearray(packet)


def reset_connection():
    """
    Reset and initialize the UART connection to the device.
    """
    global connection
    connection = serial.Serial('/dev/ttyUSB0', UART_BAUDRATE, timeout=1)


def send_cmd(cmd, data):
    """
    Send a command packet to the device and wait for acknowledgment.
    
    Args:
        cmd (int): Command code to send
        data (list): Data payload (max 15 bytes)
        
    Raises:
        AssertionError: If data exceeds maximum payload size
    """
    assert len(data) <= MAX_PAYLOAD_SIZE, f"Payload too large: {len(data)} > {MAX_PAYLOAD_SIZE}"
    
    global counter
    while True:
        packet = [0] * PACKET_LENGTH

        # Populate packet header
        packet[0] = counter & 0xFF
        packet[2] = cmd

        # Copy payload data
        packet[PAYLOAD_OFFSET:PAYLOAD_OFFSET+len(data)] = data[:]
        
        # Clear unused bytes in standard location
        packet[18] = 0
        packet[19] = 0

        # Calculate and add CRC
        crc = crc16(packet, PACKET_LENGTH - 2)
        packet[CRC_OFFSET] = crc & 0xFF
        packet[CRC_OFFSET+1] = (crc >> 8) & 0xFF

        # Send packet
        connection.write(packet)
        connection.flush()

        # Set up event to wait for ACK
        idx = counter & 0xFF
        counter += 1
        events[idx] = threading.Event()
        
        if events[idx].wait(ACK_TIMEOUT):
            del events[idx]
            break


def get_last_packet():
    """
    Get the last received packet from the device.
    
    Returns:
        bytes: Last received packet or None if no packet
    """
    return last_packet


def clear_last_packet():
    """
    Clear the last packet buffer.
    """
    global last_packet
    last_packet = None


def recv_thread():
    """
    Receiver thread function that continuously processes incoming packets.
    
    This function:
    1. Reads data from UART
    2. Validates packet CRC
    3. Processes ACKs and responses
    4. Handles debug and panic messages
    """
    read_data = bytearray()
    while True:
        # Read enough data for a complete packet
        while len(read_data) < PACKET_LENGTH:
            read_data.extend(connection.read())

        packet = read_data[:PACKET_LENGTH]
        read_data = read_data[PACKET_LENGTH:]

        # Handle debug print messages
        if packet[2] == PRINT_MESSAGE:
            msg = packet[PAYLOAD_OFFSET:]
            print(msg[:msg.index(0) if 0 in msg else len(msg)].decode('ascii', errors='ignore'), end="")

        # Handle panic messages
        if packet[2] == PANIC_MESSAGE:
            packet = packet[PAYLOAD_OFFSET:]
            print(packet[:packet.index(0) if 0 in packet else len(packet)].decode('ascii', errors='ignore'), end="")
            continue

        # Validate packet CRC
        crc = crc16(packet, PACKET_LENGTH - 2)
        if crc & 0xFF != packet[CRC_OFFSET] or (crc >> 8) & 0xFF != packet[CRC_OFFSET+1]:
            # CRC error, reset connection
            reset_connection()
            continue

        # Handle ACK packets
        if packet[1] == ACK:
            if packet[0] in events:
                events[packet[0]].set()
                continue
            else:
                continue

        # Send ACK for received packet
        ack_pkt = [0] * PACKET_LENGTH
        ack_pkt[0] = packet[0]
        ack_pkt[1] = ACK

        crc = crc16(ack_pkt, PACKET_LENGTH - 2)
        ack_pkt[CRC_OFFSET] = crc & 0xFF
        ack_pkt[CRC_OFFSET+1] = (crc >> 8) & 0xFF

        connection.write(ack_pkt)
        connection.flush()

        # Store the packet for processing
        global last_packet
        last_packet = packet
        last_packet_wait.set()


def get_response(message, local_id, response_id, resp_val=None, max_retry=None):
    """
    Send a command and wait for a specific response.
    
    Args:
        message (list): Message payload
        local_id (int): Target device mesh ID
        response_id (int): Expected response ID
        resp_val (bytes, optional): Expected response value
        max_retry (int, optional): Maximum retry attempts
        
    Returns:
        bytes: Response packet or None if no response after retries
    """
    # Prepare and send command
    data = [local_id & 0xFF, (local_id >> 8) & 0xFF] + message
    send_cmd(LIGHT_CTRL, data)

    # Wait for matching response
    while not get_last_packet() or get_last_packet()[6] != local_id or get_last_packet()[10] & 0x3F != response_id or \
        (resp_val is not None and get_last_packet()[11:11+len(resp_val)] != resp_val):

        last_packet_wait.clear()
        if not last_packet_wait.wait(0.5):
            # Retry if needed
            if max_retry is not None:
                if max_retry == 0:
                    return None

                max_retry -= 1

            send_cmd(LIGHT_CTRL, data)

    # Return and clear the response
    result = get_last_packet()
    clear_last_packet()
    return result


def flash_node(local_id, fw_file, force):
    """
    Flash firmware to a specific mesh node.
    
    This function:
    1. Checks current firmware version
    2. Validates crystal frequency compatibility
    3. Transfers firmware in chunks
    4. Finalizes the update
    
    Args:
        local_id (int): Target device mesh ID
        fw_file (str): Path to firmware file
        force (bool): Force update even if version is older
    """
    print(f"Writing FW to node {local_id}")

    # Load firmware file
    with open(fw_file, 'rb') as f:
        parser = UARTPacketParser(f.read())

    # Send OTA start command and get current firmware version
    dev_ver = get_response([OTA_START, 0], local_id, OTA_START_RESP, max_retry=10)

    # Check for device response
    if dev_ver is None:
        print("Device didn't respond.")
        return

    # Extract version information
    dev_ver = struct.unpack("I", dev_ver[DEVICE_VER_OFFSET:DEVICE_VER_OFFSET + 4])[0]
    fw_ver = struct.unpack("I", parser.data[2:6])[0]

    # Extract crystal frequency flags and version numbers
    dev_16mhz = (dev_ver >> 31) == 1
    dev_ver = dev_ver & VERSION_MASK

    fw_16mhz = (fw_ver >> 31) == 1
    fw_ver = fw_ver & VERSION_MASK

    print(f"Device version: {dev_ver}, new version: {fw_ver}, 16MHz device: {dev_16mhz}")

    # Check crystal frequency compatibility
    if dev_16mhz != fw_16mhz:
        print(f"Attempt to flash incompatible crystal version of firmware. Bailing out. Device is 16MHz? "
              f"{dev_16mhz}, FW is 16MHz? {fw_16mhz}", file=sys.stderr)
        return

    # Check firmware version unless forced
    if dev_ver >= fw_ver:
        print("Device firmware is newer or the same as the provided firmware file. Nothing to do.")
        if not force:
            print("Bailing out.")
            return
        else:
            print("Forcing update as requested.")

    # Begin firmware transfer
    now = datetime.datetime.now()
    completion = int(parser.index / parser.total * 100.)
    
    while parser.has_next_packet():
        new_completion = int(parser.index / parser.total * 100.)
        if new_completion != completion:
            completion = new_completion
            print(f"{completion}% {datetime.datetime.now() - now}")

        # Send firmware packet and wait for acknowledgment
        packet = list(parser.get_next_packet())
        response = get_response([OTA_DATA, 0, 0] + packet, local_id, OTA_DATA_RESP, bytearray(packet[0:2]), max_retry=100)
        
        # Check if device is still responsive
        if response is None:
            print("Device has disappeared")
            break

        # Verify packet index acknowledgment
        assert bytearray(packet[0:2]) == bytearray(response[11:13])
        
        # Throttle transmission rate slightly
        sleep(0.05)

    # Send final packet and complete OTA update
    packet = list(parser.get_next_packet())
    data = [local_id & 0xFF, (local_id >> 8) & 0xFF, OTA_END, 0, 0] + packet

    # Send finalization command multiple times to ensure receipt
    for _ in range(10):
        send_cmd(LIGHT_CTRL, data)

    print("Firmware written")


def main():
    """
    Main function to handle command line arguments and initiate firmware update.
    """
    args_parser = argparse.ArgumentParser(description='TLSR8266 Mesh Device Firmware Update Tool (UART)')
    args_parser.add_argument("--fw_file", help="Firmware file to flash", required=True, type=str)
    args_parser.add_argument("--mesh_address", help="Mesh address of the node to update (number between 1 and 63)", 
                            required=True, type=int)
    args_parser.add_argument("--force", help="Force update even if firmware version is older", 
                            required=False, default=False, action=argparse.BooleanOptionalAction)
    args = args_parser.parse_args()

    # Initialize UART connection
    reset_connection()

    # Start receiver thread
    threading.Thread(target=recv_thread, daemon=True).start()

    # Enable UART reporting mode
    send_cmd(ENABLE_UART, [])
    print("UART initialised")

    # Flash firmware to target node
    flash_node(args.mesh_address, args.fw_file, args.force)


if __name__ == "__main__":
    main()