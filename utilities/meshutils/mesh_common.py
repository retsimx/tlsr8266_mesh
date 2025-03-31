#!/usr/bin/env python3
"""
Common utilities for TLSR8266 Mesh Light System

This module contains shared functionality used by both the mesh device provisioning
and firmware update tools. It implements the core cryptographic operations and 
command processing required for secure communication with TLSR8266 devices.
"""

import random
import math
from Cryptodome.Cipher import AES
from Cryptodome.Random import get_random_bytes

# Default values for unprovisioned devices
mesh_name = "out_of_mesh"   # Name used to find unprovisioned devices
mesh_pass = "123"           # Default password for unprovisioned devices
shared_key = get_random_bytes(8)  # Random challenge for authentication (pair_randm)

# GATT characteristic UUIDs
pair_characteristic_uuid = "00010203-0405-0607-0809-0a0b0c0d1914"  # Pairing operations
notify_characteristic_uuid = '00010203-0405-0607-0809-0a0b0c0d1911' # Notifications 
command_characteristic_uuid = '00010203-0405-0607-0809-0a0b0c0d1912' # Command execution
ota_characteristic_uuid = '00010203-0405-0607-0809-0a0b0c0d1913'    # Firmware update
firmware_revision_uuid = '00002a26-0000-1000-8000-00805f9b34fb'     # Device firmware version

# Pairing operation opcodes
PAIR_OP_EXCHANGE_RANDOM = 0x01     # Exchange random challenge
PAIR_OP_SET_MESH_NAME = 0x04       # Set mesh network name
PAIR_OP_SET_MESH_PASSWORD = 0x05   # Set mesh network password
PAIR_OP_SET_MESH_LTK = 0x06        # Set long-term key
PAIR_OP_GET_MESH_LTK = 0x08        # Request long-term key
PAIR_OP_VERIFY_CREDENTIALS = 0x0C  # Verify credentials (12)

# Common constants for firmware updates
VERSION_MASK = 0x7FFFFFFF  # Mask to extract version number
VERSION_16MHZ = 0x80000000 # Flag indicating 16MHz variant

# Other constants
KEY_SIZE = 16                      # Size of cryptographic keys (16 bytes)
RANDOM_CHALLENGE_SIZE = 8          # Size of random challenge values (8 bytes)
MESH_FLAG = 0x01                   # Flag indicating this is a mesh LTK

def crc16(packet, length=None):
    """
    Calculate CRC16 for packet verification.
    
    Uses the standard CRC-16-MODBUS polynomial (0xA001).
    
    Args:
        packet (list): Packet data including header but excluding CRC
        length (int, optional): Length of data to process, defaults to len(packet) - 2
        
    Returns:
        int: 16-bit CRC value
    """
    if length is None:
        length = len(packet) - 2

    poly = [0, 0xA001]  # CRC-16-MODBUS polynomial
    crc = 0xFFFF        # Initial value

    # Process each byte
    for j in range(0, length):
        ds = packet[j]
        # Process each bit
        for i in range(8):
            crc = (crc >> 1) ^ poly[(crc ^ ds) & 1] & 0xFFFF
            ds >>= 1

    return crc

class PacketParser:
    """
    Handles firmware binary parsing and packet generation for OTA updates.
    
    This class:
    - Splits firmware into fixed-size packets
    - Adds headers and CRC to each packet
    - Tracks transfer progress
    """
    def __init__(self, data, packet_size):
        """
        Initialize the parser with firmware data.
        
        Args:
            data (bytes): The firmware binary data
            packet_size (int): Size of each firmware packet
        """
        self.progress = 0
        self.index = -1
        self.data = data
        self.packet_size = packet_size

        # Calculate total number of packets needed
        if len(self.data) % self.packet_size == 0:
            self.total = len(self.data) // self.packet_size
        else:
            self.total = math.floor(len(self.data) / self.packet_size) + 1

    def has_next_packet(self):
        """
        Check if there are more packets to send.
        
        Returns:
            bool: True if there are more packets, False otherwise
        """
        return self.total > 0 and (self.index + 1) < self.total

    def get_next_packet_index(self):
        """
        Get the index of the next packet.
        
        Returns:
            int: Index of the next packet
        """
        return self.index + 1

    def get_next_packet(self):
        """
        Get the next packet in the sequence.
        
        Returns:
            bytearray: The next firmware packet
        """
        index = self.get_next_packet_index()
        packet = self.get_packet(index)
        self.index = index
        return packet

    def get_packet(self, index):
        """
        Generate a firmware packet for the given index.
        Must be implemented by derived classes for specific protocols.
        
        Args:
            index (int): The packet index to generate
            
        Returns:
            bytearray: Complete firmware packet
        """
        raise NotImplementedError("Subclasses must implement get_packet()")

    def fill_index(self, packet, index):
        """
        Add packet index to packet header.
        
        Args:
            packet (list): Packet to modify
            index (int): Packet index
        """
        packet[0] = index & 0xFF
        packet[1] = (index >> 8) & 0xFF

def encode_mesh_credentials():
    """
    Generate credential verification material by XORing the mesh name and password.
    
    This is used for authentication during the pairing process. Both the client and
    server must generate the same verification material to establish a trusted connection.
    
    Returns:
        bytes: XORed mesh name and password (16 bytes)
    """
    global mesh_name, mesh_pass
    
    # Truncate mesh name to 16 bytes if needed
    if len(mesh_name) > KEY_SIZE:
        mesh_name = mesh_name[:KEY_SIZE]

    # Convert mesh name to bytes and pad to 16 bytes with null bytes
    mesh_name_bytes = mesh_name.encode('utf-8')
    while len(mesh_name_bytes) < KEY_SIZE:
        mesh_name_bytes += b'\0'

    # Truncate password to 16 bytes if needed
    if len(mesh_pass) > KEY_SIZE:
        mesh_pass = mesh_pass[:KEY_SIZE]

    # Convert password to bytes and pad to 16 bytes with null bytes
    mesh_pass_bytes = mesh_pass.encode('utf-8')
    while len(mesh_pass_bytes) < KEY_SIZE:
        mesh_pass_bytes += b'\0'

    # XOR each byte of mesh name and password
    plaintext = b''
    for i in range(KEY_SIZE):
        plaintext += (mesh_name_bytes[i] ^ mesh_pass_bytes[i]).to_bytes(length=1, byteorder='little')

    return plaintext

def encrypt_data(enc_key, data):
    """
    Encrypt data using AES-ECB with byte order manipulation.
    
    The TLSR8266 firmware expects a specific byte order, so this function:
    1. Reverses the byte order of both the key and data
    2. Performs AES encryption in ECB mode
    3. Returns the resulting ciphertext with original byte order
    
    Args:
        enc_key (bytes/list): Encryption key (16 bytes)
        data (bytes/list): Data to encrypt (16 bytes)
        
    Returns:
        bytearray: Encrypted data (16 bytes)
    """
    # Ensure we're working with bytearrays and reverse byte order for compatibility
    enc_key = bytearray(enc_key)
    enc_key.reverse()

    data = bytearray(data)
    data.reverse()

    # Perform AES encryption in ECB mode
    cipher = AES.new(bytes(enc_key), AES.MODE_ECB)
    result = bytearray(cipher.encrypt(bytes(data)))

    return result

def reverse_section(array, begin, end):
    """
    Reverse a section of bytes within an array.
    
    This is used for various byte manipulations required by the protocol
    to ensure compatibility with the device firmware.
    
    Args:
        array (bytearray): The array to modify in-place
        begin (int): Starting index (inclusive)
        end (int): Ending index (inclusive)
    """
    while begin < end:
        tmp = array[end]
        array[end] = array[begin]
        array[begin] = tmp
        begin += 1
        end -= 1

def process_session_key(data, plaintext_mesh_credentials):
    """
    Process the server's response to verify and generate the session key.
    
    This function:
    1. Extracts the server's response components (session key, random challenge)
    2. Verifies the server's session key against expectations
    3. Generates the final session key for ongoing communication
    
    Args:
        data (bytes): The server's response to the authentication request
        plaintext_mesh_credentials (bytes): The XORed mesh name and password
        
    Returns:
        bytearray: The derived session key for encrypted communication
        
    Raises:
        Exception: If session key verification fails
    """
    # Extract components from server response
    sk = data[1:1 + KEY_SIZE]                  # Server's session key
    rands = data[1:1 + RANDOM_CHALLENGE_SIZE]  # Server's random challenge (pair_rands)

    # Pad server's random challenge to 16 bytes
    key = rands[:]
    while len(key) < KEY_SIZE:
        key += b'\0'

    # Encrypt the mesh credentials with the server's random challenge
    encrypted = encrypt_data(key, plaintext_mesh_credentials)
    
    # Construct verification data by combining server random and part of encrypted credentials
    result = bytearray(b'\0' * KEY_SIZE)
    result[:len(rands)] = rands[:]  # Copy server random to first half
    result[RANDOM_CHALLENGE_SIZE:RANDOM_CHALLENGE_SIZE + RANDOM_CHALLENGE_SIZE] = \
        encrypted[RANDOM_CHALLENGE_SIZE:RANDOM_CHALLENGE_SIZE + RANDOM_CHALLENGE_SIZE]  # Copy second half of encrypted data
    
    # Reverse byte order in second half of result for server compatibility
    reverse_section(result, RANDOM_CHALLENGE_SIZE, KEY_SIZE - 1)

    # Verify the server's session key matches our expectations
    if result != sk:
        raise Exception("session_key invalid - server response doesn't match expected value")

    # Create final session key using shared key and server random
    key = bytearray(shared_key[:])  # Start with client random (pair_randm)
    key[RANDOM_CHALLENGE_SIZE:] = rands[:]  # Add server random (pair_rands)

    # Encrypt with mesh credentials and reverse byte order for final session key
    session_key = bytearray(encrypt_data(plaintext_mesh_credentials, key))
    reverse_section(session_key, 0, len(session_key) - 1)

    return session_key

class UniqueHeaderGenerator:
    """
    Generates unique header IDs for mesh commands.
    
    The header IDs are used to:
    - Uniquely identify commands
    - Prevent replay attacks
    - Generate nonces for encryption
    """
    def __init__(self):
        # Start with a large value to avoid conflicts with existing IDs
        self.counter = 0x7fffffff

    def generate_next(self):
        """
        Generate the next unique header ID.
        
        Uses a counter with overflow protection to ensure uniqueness.
        
        Returns:
            int: A unique 24-bit header ID
        """
        max_num = 0xffffff  # Maximum 24-bit value
        
        # Reset counter if it reaches the maximum
        if self.counter > max_num:
            # Restart with a random value in a specific range
            self.counter = round(random.random() * (max_num - 0x80FFFF)) + 1

        self.counter += 1
        return self.counter

# Global header ID generator
unique_header_id_generator = UniqueHeaderGenerator()

class Command:
    """
    Represents a BLE command to be sent to a device.
    
    Encapsulates the command data and manages the GATT characteristic 
    interaction with the device.
    """
    def __init__(self, data, characteristic_uuid, device):
        """
        Initialize a new command.
        
        Args:
            data (bytearray): The command data to send
            characteristic_uuid (str): The UUID of the target GATT characteristic
            device (BleakClient): The connected BLE device client
        """
        self.data = data
        self.characteristic_uuid = characteristic_uuid
        self.device = device

    async def write(self):
        """
        Write the command data to the device.
        
        Returns:
            None
        """
        await self.device.write_gatt_char(self.characteristic_uuid, self.data)

    async def read(self):
        """
        Read response data from the device.
        
        Returns:
            tuple: A tuple containing the read data
        """
        return await self.device.read_gatt_char(self.characteristic_uuid),

class CommandAction:
    """
    Handles the encryption and transmission of mesh commands.
    
    This class implements the command encryption protocol used for 
    secure communication with TLSR8266 devices after pairing.
    """
    def __init__(self, base):
        """
        Initialize a command action from a base configuration.
        
        Args:
            base (BaseCommandAction): Base configuration for the command
        """
        self.mac_address = base.mac_address
        self.session_key = base.session_key
        self.callback = base.callback
        self.no_response = base.no_response
        self.mesh_address = base.mesh_address
        self.opcode = base.opcode
        self.params = base.params
        self.vendor_id = base.vendor_id
        self.header_id = unique_header_id_generator.generate_next()

    async def encode_and_send(self, con):
        """
        Encode and send a command to the device.
        
        Args:
            con (BleakClient): Connected BLE client
            
        Returns:
            None
        """
        await self.send_command(self.encode_command(), con)

    def encode_command(self):
        """
        Encode a mesh command with proper header and payload.
        
        Formats the command according to the TLSR8266 mesh protocol:
        - 3 bytes: Header ID
        - 2 bytes: Reserved
        - 2 bytes: Target mesh address
        - 1 byte: Command opcode
        - 2 bytes: Vendor ID
        - Remaining bytes: Command parameters
        
        Returns:
            bytearray: Encoded command
        """
        command = bytearray(b'\0'*20)
        offset = 0

        # Write the 24-bit header ID (3 bytes)
        command[offset] = self.header_id & 0xFF               # Low byte
        offset += 1
        command[offset] = (self.header_id >> 8) & 0xFF        # Middle byte
        offset += 1
        command[offset] = (self.header_id >> 16) & 0xFF       # High byte
        offset += 1
        
        # Skip 2 reserved bytes
        offset += 2
        
        # Write the 16-bit mesh address (2 bytes)
        command[offset] = self.mesh_address & 0xFF            # Low byte
        offset += 1
        command[offset] = (self.mesh_address >> 8) & 0xFF     # High byte
        offset += 1
        
        # Write command opcode with high bits set (op | 0xC0)
        command[offset] = (self.opcode & 0xff).to_bytes(length=4, byteorder='little', signed=True)[0] | 0xC0
        offset += 1
        
        # Write the 16-bit vendor ID (2 bytes)
        command[offset] = (self.vendor_id >> 8) & 0xFF        # High byte
        offset += 1
        command[offset] = self.vendor_id & 0xFF               # Low byte
        offset += 1
        
        # Write command parameters if provided
        if self.params:
            command[offset:offset+len(self.params)] = self.params[:]

        return command

    def generate_nonce(self):
        """
        Generate a nonce for command encryption.
        
        The nonce combines:
        - Device MAC address
        - Command header ID
        - Additional fixed values
        
        This helps ensure each command has a unique encryption context.
        
        Returns:
            bytearray: 8-byte nonce for encryption
        """
        # Start with the device MAC address
        mac_bytes = bytearray(self.mac_address)[:]
        ivs = bytearray(b'\0'*8)
        ivs[:len(mac_bytes)] = mac_bytes[:]
        
        # Add fixed value for protocol version
        ivs[4] = 1
        
        # Add header ID components (24-bit value)
        ivs[5] = self.header_id & 0xFF                 # Low byte
        ivs[6] = (self.header_id >> 8) & 0xFF          # Middle byte
        ivs[7] = (self.header_id >> 16) & 0xFF         # High byte

        return ivs

    async def send_command(self, command, device):
        """
        Encrypt and send a command to the device.
        
        Args:
            command (bytearray): The command to encrypt and send
            device (BleakClient): Connected BLE client
            
        Returns:
            None
        """
        # Generate a nonce for encryption
        nonce = self.generate_nonce()
        
        # Encrypt the command data
        data = self.generate_data(self.session_key, nonce, command)

        # Create a Command object and write it to the device
        command = Command(
            data,
            command_characteristic_uuid,
            device
        )

        await command.write()

    def encrypt_data_chunk(self, session_key, r):
        """
        Encrypt a chunk of data with byte order manipulation.
        
        The TLSR8266 protocol requires specific byte ordering for compatibility.
        
        Args:
            session_key (bytearray): Encryption key
            r (bytearray): Data to encrypt
            
        Returns:
            bytearray: Encrypted data with proper byte ordering
        """
        # Encrypt using the standard AES encryption function
        encrypted = encrypt_data(session_key, r)
        
        # Reverse the byte order of the result
        p1 = 0
        p2 = len(encrypted)
        r = bytearray(b'\0' * len(encrypted))
        while True:
            p2 -= 1
            if p2 < 0:
                break

            r[p2] = encrypted[p1]
            p1 += 1

        return r

    def generate_data(self, session_key, nonce, command):
        """
        Generate encrypted command data using AES-CCM style encryption.
        
        This implements a custom encryption protocol compatible with the
        TLSR8266 mesh firmware. The algorithm:
        1. Generates a MAC tag from the nonce and command
        2. Updates the command with the MAC tag
        3. Encrypts the command payload
        
        Args:
            session_key (bytearray): Session key for encryption
            nonce (bytearray): Nonce for this command
            command (bytearray): Command to encrypt
            
        Returns:
            bytearray: Fully encrypted command ready for transmission
        """
        # --- MAC Tag Generation ---
        # Initialize MAC computation block with nonce
        r = bytearray(b'\0'*16)
        r[:8] = nonce[:]  # Copy nonce to first 8 bytes
        r[8] = 15         # Message length (15 bytes)

        # Encrypt the initial block
        r = self.encrypt_data_chunk(session_key, r)

        # XOR command data with encrypted block to generate MAC
        for i in range(15):
            r[i & 15] ^= command[i+5]

            # Re-encrypt after filling block or at end
            if (i & 15) == 15 or i == 14:
                r = self.encrypt_data_chunk(session_key, r)

        # Update command with MAC tag (first 2 bytes)
        for i in range(2):
            command[i+3] = r[i]

        # --- Command Encryption ---
        # Initialize counter mode block with nonce
        r = bytearray(b'\0' * 16)
        r[1:8+1] = nonce[:]  # Copy nonce to bytes 1-8

        # Use counter mode to encrypt command payload
        e = bytearray(b'\0'*16)
        for i in range(15):
            if (i & 15) == 0:
                # Generate new keystream block for each 16-byte chunk
                e = self.encrypt_data_chunk(session_key, r)
                r[0] += 1  # Increment counter

            # XOR command bytes with keystream
            command[i+5] ^= e[i & 15]

        return command

class BaseCommandAction:
    """
    Base configuration for a mesh command.
    
    This class serves as a builder for CommandAction objects, holding the
    parameters needed to create and send a mesh command.
    """
    def __init__(self,
                 mac_address=None,
                 session_key=None,
                 no_response=False,
                 opcode=0,
                 mesh_address=0,
                 params=None,
                 vendor_id=0,
                 callback=None
    ):
        """
        Initialize a new command configuration.
        
        Args:
            mac_address (bytes): MAC address of the device
            session_key (bytearray): Session key for encryption
            no_response (bool): Whether to expect a response
            opcode (int): Command operation code
            mesh_address (int): Target mesh address (0 for direct device)
            params (list): Command parameters
            vendor_id (int): Vendor ID for the device
            callback (function): Callback for command response
        """
        self.mac_address = mac_address
        self.session_key = session_key
        self.no_response = no_response
        self.opcode = opcode
        self.mesh_address = mesh_address
        self.params = params
        self.vendor_id = vendor_id
        self.callback = callback

    def build_command_action(self):
        """
        Build a CommandAction from this configuration.
        
        Returns:
            CommandAction: The command action ready for execution
        """
        return CommandAction(self)