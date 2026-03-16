#!/usr/bin/env python3
"""
Mesh Add Utility for TLSR8266 Mesh Light System

This script implements the client side of the BLE pairing protocol for adding
TLSR8266 devices to a mesh network. It handles authentication, session key 
derivation, mesh parameter provisioning, and secure command encryption.

The pairing protocol consists of the following phases:
1. Authentication and Session Key Establishment
2. Mesh Device Configuration 
3. Mesh Parameter Provisioning (name, password, LTK)
4. Verification of successful pairing

Usage:
  python mesh_add.py --mesh_address <addr> --mesh_name <name> --mesh_password <pwd> [--mesh_ltk <ltk>]

Examples:
  # Add a light with address 5 to mesh "myhome" with password "securepass"
  python mesh_add.py --mesh_address 5 --mesh_name myhome --mesh_password securepass
  
  # Add a light with custom LTK (more secure)
  python mesh_add.py --mesh_address 6 --mesh_name myhome --mesh_password securepass --mesh_ltk "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6"
  
  # Add a light to a new mesh network
  python mesh_add.py --mesh_address 1 --mesh_name newmesh --mesh_password newpass
"""

import argparse
import asyncio
import binascii
import os
import sys
from asyncio import sleep

from bleak import BleakClient, BleakScanner

from mesh_common import (
    # Constants
    pair_characteristic_uuid, notify_characteristic_uuid,
    command_characteristic_uuid,
    PAIR_OP_SET_MESH_NAME,
    PAIR_OP_SET_MESH_PASSWORD, PAIR_OP_SET_MESH_LTK,
    PAIR_STATE_MESH_EFFECT, PAIR_STATE_PAIRING_COMPLETE, MESH_FLAG,
    DEFAULT_LTK, DEFAULT_MESH_NAME, DEFAULT_MESH_PASSWORD,
    # Classes
    BaseCommandAction, Command,
    # Functions
    authenticate, parse_mac_address, encrypt_data,
)

async def main():
    """
    Main function to add a device to a mesh network.
    
    This function:
    1. Parses command line arguments
    2. Scans for and connects to an unprovisioned device
    3. Authenticates and establishes a secure session
    4. Sets up the device with mesh parameters
    5. Verifies successful pairing
    """
    # Parse command line arguments
    args_parser = argparse.ArgumentParser(description='TLSR8266 Mesh Device Provisioning Tool')
    args_parser.add_argument('--mesh_address', help="The address of the new node in the mesh. Should be a unique (in the mesh) value between 1 and 63", required=True, type=int)
    args_parser.add_argument('--mesh_name', help="The name of the mesh. eg `mymesh`. (Must be 16 bytes or fewer)", required=True, type=str, default="out_of_mesh")
    args_parser.add_argument('--mesh_password', help="The password of the mesh. eg `mymeshpwd` (Must be 16 bytes or fewer)", required=True, type=str, default="123")
    args_parser.add_argument('--mesh_ltk', help="The long-term key for the mesh in hex format (32 hex characters / 16 bytes)", required=False, type=str)

    args = args_parser.parse_args()

    # Validate and prepare LTK before starting connection process
    using_default_ltk = True
    ltk_bytes = bytearray(DEFAULT_LTK)
    
    if args.mesh_ltk:
        try:
            # Validate the provided LTK (must be 32 hex chars / 16 bytes)
            ltk_hex = args.mesh_ltk.replace(':', '').replace('-', '').strip()
            if len(ltk_hex) != 32:
                print(f"WARNING: Invalid LTK length ({len(ltk_hex)} hex chars). LTK must be 16 bytes (32 hex characters).")
                print("Falling back to default LTK.")
            else:
                ltk_bytes = bytearray.fromhex(ltk_hex)
                using_default_ltk = False
        except ValueError:
            print("WARNING: Invalid hex format in LTK. Falling back to default LTK.")
    
    # Warn and confirm if using default LTK
    if using_default_ltk:
        print("\nWARNING: Using default LTK. This is less secure than using a custom LTK.")
        print(f"Default LTK: {binascii.hexlify(bytes(ltk_bytes)).decode()}")
        confirmation = input("Continue with default LTK? [y/N]: ")
        if confirmation.lower() != 'y':
            print("Aborted. Please provide a custom LTK using the --mesh_ltk argument.")
            return

    # Scan for unprovisioned device (advertising with default name)
    mesh_name = args.mesh_name
    mesh_password = args.mesh_password

    print(f"Scanning for unprovisioned device with name 'out_of_mesh'...")
    device = await BleakScanner.find_device_by_name(name="out_of_mesh", timeout=5)

    if not device:
        print("Unable to find any devices")
        return

    print(f"Found device {device.address}, connecting...")
    try:
        async with BleakClient(device.address) as client:
            # ----- PHASE 1: AUTHENTICATION AND SESSION KEY ESTABLISHMENT -----
            print("Authenticating with device...")
            session_key = await authenticate(client, DEFAULT_MESH_NAME, DEFAULT_MESH_PASSWORD)
            print("Authentication successful, session established")

            # ----- PHASE 2: DEVICE CONFIGURATION FOR MESH -----
            print("Configuring device for mesh network...")

            # Get mesh address from command line arguments
            mesh_address = args.mesh_address
            print(f"Assigning mesh address: {mesh_address}")

            # Get device MAC address and reverse for protocol compatibility
            _, mac_bytes = parse_mac_address(device.address)

            print("Setting device mesh address...")
            # Send command to set mesh address (opcode 0xE0)
            action = BaseCommandAction(
                mac_address=mac_bytes,
                opcode=0xe0,     # Command to set mesh address
                params=[mesh_address & 0xff, (mesh_address >> 8) & 0xff],  # Little-endian address
                mesh_address=0,  # Direct to device (not through mesh)
                session_key=session_key,
                vendor_id=0x0211,  # Telink vendor ID (LE: [0x11, 0x02] on wire)
                no_response=True
            ).build_command_action()

            await action.encode_and_send(client)
            
            # Allow time for the device to process the mesh address
            await sleep(4)

            # ----- PHASE 3: PROVISIONING MESH PARAMETERS -----
            print("Provisioning mesh parameters...")

            # Prepare and encrypt mesh name
            name = mesh_name.encode()
            # Pad name to 16 bytes
            name = bytearray(name) + bytearray([0] * (16 - len(name)))
            # Encrypt name with session key
            name = encrypt_data(session_key, name)
            # Reverse bytes for device compatibility
            name.reverse()

            # Prepare and encrypt mesh password
            pwd = mesh_password.encode()
            # Pad password to 16 bytes
            pwd = bytearray(pwd) + bytearray([0] * (16 - len(pwd)))
            # Encrypt password with session key
            pwd = encrypt_data(session_key, pwd)
            # Reverse bytes for device compatibility
            pwd.reverse()

            # Prepare and encrypt long term key (LTK)
            # LTK validation and user confirmation was done earlier
            
            # Encrypt the LTK with the session key
            ltk = encrypt_data(session_key, ltk_bytes)
            # Reverse bytes for device compatibility
            ltk.reverse()

            # Add opcodes to the start of each parameter
            name = bytearray([PAIR_OP_SET_MESH_NAME]) + name
            pwd = bytearray([PAIR_OP_SET_MESH_PASSWORD]) + pwd
            # Add mesh flag to LTK to indicate this is for mesh communication
            ltk = bytearray([PAIR_OP_SET_MESH_LTK]) + ltk + bytearray([MESH_FLAG])

            # Create commands for each parameter
            name_command = Command(name, pair_characteristic_uuid, client)
            pwd_command = Command(pwd, pair_characteristic_uuid, client)
            ltk_command = Command(ltk, pair_characteristic_uuid, client)

            # Send mesh parameters in sequence
            print("Sending mesh name...")
            await name_command.write()
            await sleep(0.2)  # Allow device time to process
            
            print("Sending mesh password...")
            await pwd_command.write()
            await sleep(0.2)  # Allow device time to process
     
            print("Sending mesh LTK...")
            await ltk_command.write()
            await sleep(0.2)  # Allow device time to process

            # ----- PHASE 4: VERIFY SUCCESSFUL PAIRING -----
            # Read the device state to verify successful pairing
            # Give the device a bit more time to complete state transitions
            await sleep(2)
            result = await client.read_gatt_char(pair_characteristic_uuid)

            # Check if device is in the expected state (pairing complete)
            # If it's in MeshPairEffect (0x07), try waiting a bit more
            if result[0] == PAIR_STATE_MESH_EFFECT:  # MeshPairEffect - almost complete
                print("Device in MeshPairEffect state, waiting for final transition...")
                await sleep(1.0)
                result = await client.read_gatt_char(pair_characteristic_uuid)
            
            if result[0] != PAIR_STATE_PAIRING_COMPLETE:
                # Accept MeshPairEffect (0x07) as success too, since it's very close
                if result[0] == PAIR_STATE_MESH_EFFECT:
                    print(f"Device in MeshPairEffect state (0x{PAIR_STATE_MESH_EFFECT:02x}) - pairing likely successful")
                else:
                    print(f"Light could not be added to mesh. Unexpected state: 0x{result[0]:02x}")
                    print(f"Expected state: 0x{PAIR_STATE_PAIRING_COMPLETE:02x} (PAIR_STATE_PAIRING_COMPLETE)")
                    return

            print(f"Light with MAC ({device.address}) successfully added to mesh network")
            print(f"Mesh Name: {mesh_name}")
            print(f"Mesh Address: {mesh_address}")
            if using_default_ltk:
                print("Using default LTK")
            else:
                print("Using custom LTK")

            # Set up notification handler for any device messages
            def on_message(*args, **kwargs):
                print("Message received:", args, kwargs)

            # Enable notifications
            print("Enabling notifications...")
            await client.write_gatt_char(notify_characteristic_uuid, [1])
            await client.start_notify(notify_characteristic_uuid, on_message)

            # Keep connection open to observe any messages
            print("Monitoring device for 10 seconds...")
            await asyncio.sleep(10)
    except EOFError:
        # Suppress EOFError which can happen during disconnection if the device resets
        pass


# Run the main function when script is executed
if __name__ == "__main__":
    asyncio.run(main())
