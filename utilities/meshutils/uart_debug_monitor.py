#!/usr/bin/env python3
"""
UART Debug Monitor for TLSR8266 Mesh Light System

This script connects to a TLSR8266 mesh device via UART and continuously monitors
and displays debugging messages, panic messages, and other communications from the device.

The script handles the UART protocol and automatically decodes various message types:
- Debug print messages (PRINT_MESSAGE)
- Panic/error messages (PANIC_MESSAGE) 
- Light status messages (LIGHT_STATUS)
- Mesh network messages (MESH_MESSAGE)
- General packet information

Usage:
  python uart_debug_monitor.py [--port /dev/ttyUSB0] [--baudrate 115200] [--verbose]

Examples:
  # Monitor with default settings
  python uart_debug_monitor.py

  # Monitor on specific port
  python uart_debug_monitor.py --port /dev/ttyUSB1

  # Monitor with verbose packet information
  python uart_debug_monitor.py --verbose

  # Monitor with different baudrate
  python uart_debug_monitor.py --baudrate 9600
"""

import argparse
import datetime
import signal
import struct
import sys
import threading
from time import sleep

import serial

from mesh_common import crc16

# UART Protocol Constants (from flash_fw_uart.py)
# Packet structure:
# [0] = counter byte
# [1] = command type (or 0xFF for ACK)
# [2] = command code (for normal packets) or unused (for ACK)
# [3-42] = data payload (40 bytes)
# [42-43] = CRC16 of bytes 0-42

from uart_constants import *

# Global variables
connection = None
running = True
verbose = False
message_count = 0
print_message_buffer = ""  # Buffer for assembling chunked print messages


def signal_handler(signum, frame):
    """
    Handle Ctrl+C gracefully by setting running flag to False.
    """
    global running, print_message_buffer
    print("\n[Monitor] Received interrupt signal, shutting down...")
    
    # Print any remaining buffered content
    if print_message_buffer.strip():
        timestamp = format_timestamp()
        print(f"[{timestamp}] DEBUG: {print_message_buffer.strip()}")
        
    running = False


def init_connection(port, baudrate):
    """
    Initialize and return the UART connection to the device.
    
    Args:
        port (str): Serial port device path
        baudrate (int): Serial connection baudrate
        
    Returns:
        serial.Serial: Initialized serial connection
        
    Raises:
        Exception: If connection cannot be established
    """
    try:
        conn = serial.Serial(port, baudrate, timeout=1)
        print(f"[Monitor] Connected to {port} at {baudrate} baud")
        return conn
    except Exception as e:
        print(f"[Monitor] Failed to connect to {port}: {e}")
        raise


def send_enable_uart():
    """
    Send the ENABLE_UART command to start receiving debug messages.
    """
    global connection
    
    if not connection or not connection.is_open:
        return False
    
    try:
        # Create ENABLE_UART packet
        packet = [0] * PACKET_LENGTH
        packet[0] = 0  # counter
        packet[1] = 0  # command type (not ACK)
        packet[2] = ENABLE_UART  # command code
        
        # Calculate and add CRC
        crc = crc16(packet, PACKET_LENGTH - 2)
        packet[CRC_OFFSET] = crc & 0xFF
        packet[CRC_OFFSET + 1] = (crc >> 8) & 0xFF
        
        # Send packet
        connection.write(packet)
        connection.flush()
        return True
    except Exception as e:
        if verbose:
            print(f"[Monitor] Failed to send ENABLE_UART: {e}")
        return False


def enable_uart_thread():
    """
    Thread function that periodically sends ENABLE_UART commands.
    
    This ensures the device stays in UART monitoring mode even after reboots
    or other state changes that might disable UART output.
    """
    global running
    
    first_send = True
    
    while running:
        if send_enable_uart():
            if first_send or verbose:
                print("[Monitor] Sent ENABLE_UART command")
                first_send = False
        
        # Wait 1 second before sending next command
        for _ in range(5):  # Sleep in 0.1s increments to be responsive to shutdown
            if not running:
                break
            sleep(0.1)


def format_timestamp():
    """
    Return a formatted timestamp string for message prefixes.
    
    Returns:
        str: Formatted timestamp
    """
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def decode_light_status(payload):
    """
    Decode a LIGHT_STATUS message payload.
    
    Args:
        payload (bytes): Message payload
        
    Returns:
        str: Human-readable status information
    """
    try:
        if len(payload) >= 8:
            mesh_id = struct.unpack("<H", payload[0:2])[0]
            brightness = payload[2] if len(payload) > 2 else 0
            color_temp = payload[3] if len(payload) > 3 else 0
            status_flags = payload[4] if len(payload) > 4 else 0
            
            return f"Mesh ID: {mesh_id}, Brightness: {brightness}, Color Temp: {color_temp}, Flags: 0x{status_flags:02X}"
        else:
            return f"Raw data: {payload.hex()}"
    except Exception as e:
        return f"Decode error: {e}, Raw data: {payload.hex()}"


def decode_mesh_message(payload):
    """
    Decode a MESH_MESSAGE payload.
    
    The payload contains an AppCmdValue structure:
    [0-2]   sno (3 bytes)
    [3-4]   src (2 bytes, little-endian)  
    [5-6]   dst (2 bytes, little-endian)
    [7]     op (1 byte)
    [8-9]   vendor_id (2 bytes, little-endian)
    [10-19] par (10 bytes)
    
    Args:
        payload (bytes): Message payload containing AppCmdValue
        
    Returns:
        str: Human-readable mesh message information
    """
    try:
        if len(payload) >= 10:  # Need at least sno + src + dst + op + vendor_id
            # Extract addresses (little-endian u16) - display as hex
            src_addr = struct.unpack("<H", payload[3:5])[0]
            dst_addr = struct.unpack("<H", payload[5:7])[0]
            
            # Extract operation code
            op = payload[7]
            
            # Extract vendor ID (little-endian u16)
            vendor_id = struct.unpack("<H", payload[8:10])[0]
            
            # Extract parameter data (up to 10 bytes)
            params = payload[10:20] if len(payload) >= 20 else payload[10:]
            params_hex = params.hex() if params else "(no params)"
            
            return f"Src: 0x{src_addr:04X}, Dst: 0x{dst_addr:04X}, Op: 0x{op:02X}, Vendor: 0x{vendor_id:04X}, Params: {params_hex}"
        else:
            return f"Raw data: {payload.hex()}"
    except Exception as e:
        return f"Decode error: {e}, Raw data: {payload.hex()}"


def send_ack(counter_byte):
    """
    Send an ACK packet back to the device.
    
    Args:
        counter_byte (int): Counter byte from the received packet to acknowledge
    """
    global connection
    
    # Create ACK packet
    ack_packet = [0] * PACKET_LENGTH
    ack_packet[0] = counter_byte  # Echo back the counter
    ack_packet[1] = ACK           # ACK command type
    # ack_packet[2] remains 0 (unused for ACK)
    
    # Calculate and add CRC
    crc = crc16(ack_packet, PACKET_LENGTH - 2)
    ack_packet[CRC_OFFSET] = crc & 0xFF
    ack_packet[CRC_OFFSET + 1] = (crc >> 8) & 0xFF
    
    # Send ACK
    connection.write(ack_packet)
    connection.flush()


def handle_print_message_chunk(fragment, timestamp):
    """
    Handle a chunk of a print message, buffering and assembling complete messages.
    
    Args:
        fragment (str): Text fragment from this packet
        timestamp (str): Formatted timestamp for this message
    """
    global print_message_buffer
    
    # Debug: show what we received in this chunk
    if verbose:
        print(f"[{timestamp}] CHUNK: '{fragment}' (len={len(fragment)}, bytes={[ord(c) for c in fragment]})")
        print(f"[{timestamp}] BUFFER BEFORE: '{print_message_buffer}' (len={len(print_message_buffer)})")
    
    # Add this fragment to the buffer
    print_message_buffer += fragment
    
    # Check if we have complete lines (ending with newline)
    while '\n' in print_message_buffer:
        line, print_message_buffer = print_message_buffer.split('\n', 1)
        line = line.rstrip('\r')  # Remove any carriage return
        if line:  # Only print non-empty lines
            print(f"[{timestamp}] DEBUG: {line}")
    
    # Alternative strategy: Look for message boundaries by pattern matching
    # Common patterns that indicate a new message is starting
    message_start_patterns = ["BLE:", "MESH:", "OTA:", "ERROR:", "WARN:", "INFO:", "DEBUG:", "PANIC:"]
    
    # Check if current buffer contains what looks like multiple messages concatenated
    for pattern in message_start_patterns:
        if print_message_buffer.count(pattern) > 1:
            # Find the last occurrence of the pattern - this is likely the start of a new message
            last_pattern_pos = print_message_buffer.rfind(pattern)
            if last_pattern_pos > 0:
                # Split at this position
                complete_message = print_message_buffer[:last_pattern_pos].rstrip()
                if complete_message:
                    print(f"[{timestamp}] DEBUG: {complete_message}")
                print_message_buffer = print_message_buffer[last_pattern_pos:]
                break
    
    # Handle very long lines without newlines (prevent unlimited buffer growth)
    if len(print_message_buffer) > 200:  # Increased limit
        # Print what we have and reset buffer
        if print_message_buffer.strip():
            print(f"[{timestamp}] DEBUG: {print_message_buffer.strip()}")
        print_message_buffer = ""


def process_packet(packet):
    """
    Process and display information from a received packet.
    
    Args:
        packet (bytes): Complete UART packet
    """
    global message_count, verbose
    
    counter = packet[0]
    cmd_type = packet[1]
    cmd_code = packet[2]
    payload = packet[PAYLOAD_OFFSET:CRC_OFFSET]
    
    # Remove null bytes from payload for text messages
    payload_clean = payload[:payload.index(0) if 0 in payload else len(payload)]
    
    message_count += 1
    timestamp = format_timestamp()
    
    if cmd_code == PRINT_MESSAGE:
        # Debug print message - handle chunked messages
        try:
            debug_fragment = payload_clean.decode('ascii', errors='replace')
            handle_print_message_chunk(debug_fragment, timestamp)
        except Exception as e:
            print(f"[{timestamp}] DEBUG (decode error): {payload_clean.hex()}")
            
    elif cmd_code == PANIC_MESSAGE:
        # Panic/error message
        try:
            panic_msg = payload_clean.decode('ascii', errors='replace').rstrip()
            if panic_msg:  # Only print non-empty messages
                print(f"[{timestamp}] PANIC: {panic_msg}")
        except Exception as e:
            print(f"[{timestamp}] PANIC (decode error): {payload_clean.hex()}")
            
    elif cmd_code == LIGHT_STATUS:
        # Light status message
        status_info = decode_light_status(payload)
        print(f"[{timestamp}] LIGHT_STATUS: {status_info}")
        
    elif cmd_code == MESH_MESSAGE:
        # Mesh network message
        mesh_info = decode_mesh_message(payload)
        print(f"[{timestamp}] MESH_MSG: {mesh_info}")
        
    elif cmd_type == ACK:
        # ACK packet - already handled in recv_thread, this shouldn't be reached
        if verbose:
            print(f"[{timestamp}] ACK: Counter {counter}")
            
    elif cmd_code in [OTA_START, OTA_DATA, OTA_END, OTA_START_RESP, OTA_DATA_RESP]:
        # OTA-related messages
        ota_commands = {
            OTA_START: "OTA_START",
            OTA_DATA: "OTA_DATA", 
            OTA_END: "OTA_END",
            OTA_START_RESP: "OTA_START_RESP",
            OTA_DATA_RESP: "OTA_DATA_RESP"
        }
        print(f"[{timestamp}] OTA: {ota_commands[cmd_code]} (Counter: {counter})")
        
    else:
        # Unknown or other message types
        if verbose or (cmd_code != 0 and cmd_type != 0):  # Skip empty packets unless verbose
            print(f"[{timestamp}] UNKNOWN: Type=0x{cmd_type:02X}, Code=0x{cmd_code:02X}, "
                  f"Counter={counter}, Payload={payload_clean.hex()}")


def recv_thread():
    """
    Receiver thread function that continuously processes incoming packets.
    
    This function:
    1. Reads data from UART
    2. Validates packet CRC
    3. Processes and displays messages
    4. Handles connection errors gracefully
    """
    global connection, running, print_message_buffer
    
    read_data = bytearray()
    crc_errors = 0
    last_message_time = 0
    
    print(f"[Monitor] Starting message monitoring... (Press Ctrl+C to stop)")
    
    while running:
        try:
            # Read enough data for a complete packet
            while len(read_data) < PACKET_LENGTH and running:
                new_data = connection.read(PACKET_LENGTH - len(read_data))
                if new_data:
                    read_data.extend(new_data)
                else:
                    # Timeout occurred, continue reading
                    continue
            
            if not running:
                break
                
            packet = read_data[:PACKET_LENGTH]
            read_data = read_data[PACKET_LENGTH:]
            
            # Validate packet CRC
            calculated_crc = crc16(packet, PACKET_LENGTH - 2)
            received_crc = packet[CRC_OFFSET] | (packet[CRC_OFFSET + 1] << 8)
            
            if calculated_crc != received_crc:
                crc_errors += 1
                if verbose:
                    print(f"[{format_timestamp()}] CRC ERROR: Expected 0x{calculated_crc:04X}, "
                          f"got 0x{received_crc:04X} (Error #{crc_errors})")
                continue
            
            # Handle ACK packets (don't send ACK for ACK)
            if packet[1] == ACK:
                if verbose:
                    print(f"[{format_timestamp()}] ACK: Counter {packet[0]}")
                continue
            
            # Send ACK for non-ACK packets
            send_ack(packet[0])
            
            # Process valid packet
            process_packet(packet)
            last_message_time = datetime.datetime.now().timestamp()
            
        except serial.SerialException as e:
            print(f"[Monitor] Serial connection error: {e}")
            running = False
            break
        except Exception as e:
            print(f"[Monitor] Unexpected error: {e}")
            if verbose:
                import traceback
                traceback.print_exc()
        
        # Check for stale buffered content (flush after 2 seconds of no new data)
        current_time = datetime.datetime.now().timestamp()
        if (print_message_buffer.strip() and 
            last_message_time > 0 and 
            current_time - last_message_time > 2.0):
            timestamp = format_timestamp()
            print(f"[{timestamp}] DEBUG: {print_message_buffer.strip()}")
            print_message_buffer = ""


def main():
    """
    Main function to handle command line arguments and start monitoring.
    """
    global connection, verbose, running
    
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description='TLSR8266 Mesh Device UART Debug Monitor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                              # Monitor with default settings
  %(prog)s --port /dev/ttyUSB1         # Monitor on specific port  
  %(prog)s --verbose                   # Show verbose packet information
  %(prog)s --baudrate 9600             # Use different baudrate
        """
    )
    
    parser.add_argument("--port", 
                       help="Serial port device (default: /dev/ttyUSB0)", 
                       default="/dev/ttyUSB0", 
                       type=str)
    parser.add_argument("--baudrate", 
                       help="Serial connection baudrate (default: 115200)", 
                       default=UART_BAUDRATE, 
                       type=int)
    parser.add_argument("--verbose", 
                       help="Show verbose packet information including ACKs and unknown packets", 
                       action="store_true")
    
    args = parser.parse_args()
    verbose = args.verbose
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Initialize UART connection
        connection = init_connection(args.port, args.baudrate)
        
        # Start receiver thread
        recv_thread_handle = threading.Thread(target=recv_thread, daemon=True)
        recv_thread_handle.start()
        
        # Start UART enable thread (sends enable command every second)
        enable_thread_handle = threading.Thread(target=enable_uart_thread, daemon=True)
        enable_thread_handle.start()
        
        print("[Monitor] Started periodic ENABLE_UART sender (every 1 second)")
        
        # Brief pause to let connection settle, then send initial command
        sleep(0.1)
        send_enable_uart()
        
        # Keep main thread alive until interrupted
        while running:
            sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n[Monitor] Keyboard interrupt received")
    except Exception as e:
        print(f"[Monitor] Error: {e}")
        return 1
    finally:
        # Clean up
        running = False
        if connection and connection.is_open:
            connection.close()
            print("[Monitor] Serial connection closed")
        
        print(f"[Monitor] Total messages received: {message_count}")
        print("[Monitor] Debug monitor stopped")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
