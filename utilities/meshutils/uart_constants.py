#!/usr/bin/env python3
"""
Shared UART protocol constants for TLSR8266 Mesh Light System.

Used by flash_fw_uart.py and uart_debug_monitor.py.
"""

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
PACKET_SIZE = 8       # Size of each firmware data packet in LIGHT_CTRL
PACKET_LENGTH = 44    # Total UART packet length
CRC_OFFSET = 42       # Offset of CRC in packet
UART_BAUDRATE = 115200 # Serial connection baudrate
PAYLOAD_OFFSET = 3    # Offset of payload in packets
MAX_PAYLOAD_SIZE = 15  # Maximum size of payload data in LIGHT_CTRL packets
DEVICE_VER_OFFSET = 13 # Offset of device version in response packet
