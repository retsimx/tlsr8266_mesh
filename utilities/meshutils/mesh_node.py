#!/usr/bin/env python3
"""
Mesh Node Control Utility for TLSR8266 Mesh Light System
=========================================================

Provides direct, authenticated BLE control of individual TLSR8266 mesh light
nodes without going through the smart-device-daemon or MQTT broker.
Useful for:

  - Commissioning a newly flashed or factory-reset light into an existing mesh
  - Inspecting the live state of a single node (brightness, colour temperature,
    group memberships)
  - Repairing mesh configuration on a node that the daemon cannot reach (wrong
    mesh address, wrong group, corrupted provisioning data, etc.)
  - Testing protocol changes or new firmware builds against real hardware

How it works
------------
Performs the full BLE mesh authentication handshake (exchanging a random
challenge, deriving a 16-byte AES session key from the mesh credentials) and
then encrypts commands using the same AES-CCM-style scheme used by the
firmware.  Notifications from the light are decrypted with the same session
key and printed as human-readable output.

Usage
-----
  python3 mesh_node.py --address <MAC> --mesh-address <N> \\
      --mesh-name <name> --mesh-password <password> <command>

Arguments
---------
  --address           BLE MAC address of the target node (e.g., 12:34:56:78:9A:1D)
  --mesh-address      Mesh node address as a decimal integer (e.g., 29)
                      Not required for --get-ltk.
  --mesh-name         Mesh network name (e.g., mlm)
  --mesh-password     Mesh network password (e.g., your-mesh-password)
  --verbose           Print raw decrypted notification bytes

Commands (mutually exclusive)
------------------------------
  --on                    Turn the light on
  --off                   Turn the light off
  --brightness <0-32767>  Set the cold-white brightness level
  --temperature <0-32767> Set the warm-white (colour temperature) level
  --get-status            Query and print the current brightness values
  --get-groups            Query and print the mesh groups this node belongs to
  --add-group <addr>      Add the node to a mesh group (e.g., 0x8010)
  --del-group <addr>      Remove the node from a mesh group
  --set-mesh-address <N>  Change the node's mesh address (takes effect after
                          reboot; reconnect using the new address)
  --set-mac-address <MAC> Overwrite the node's BLE MAC address stored in flash
                          (device reboots; reconnect on the new MAC)
  --kick-out              Factory-reset the node: clears provisioning data and
                          reboots the device advertising as "out_of_mesh".
                          Re-provision afterwards with mesh_add.py.
  --get-ltk               Retrieve the mesh Long-Term Key (LTK) stored in the
                          device flash.  Does not require --mesh-address.

Examples
--------
  # Query status of node 29
  python3 mesh_node.py --address 12:34:56:78:9A:1D --mesh-address 29 \\
      --mesh-name mlm --mesh-password your-mesh-password --get-status

  # Turn on, then check status
  python3 mesh_node.py --address 12:34:56:78:9A:1D --mesh-address 29 \\
      --mesh-name mlm --mesh-password your-mesh-password --on

  # Add node to the Living Room group
  python3 mesh_node.py --address 12:34:56:78:9A:1D --mesh-address 29 \\
      --mesh-name mlm --mesh-password your-mesh-password --add-group 0x8010

  # Factory-reset a node, then re-provision it
  python3 mesh_node.py --address 12:34:56:78:9A:1D --mesh-address 29 \\
      --mesh-name mlm --mesh-password your-mesh-password --kick-out
  python3 mesh_add.py --mesh_address 29 --mesh_name mlm \\
      --mesh_password your-mesh-password --mesh_ltk <ltk_hex>

  # Read the LTK from a provisioned node
  python3 mesh_node.py --address 12:34:56:78:9A:1D \\
      --mesh-name mlm --mesh-password your-mesh-password --get-ltk
"""

import argparse
import asyncio
import binascii
import sys
import traceback
from asyncio import sleep
from Cryptodome.Random import get_random_bytes
from bleak import BleakClient

from mesh_common import (
    # Constants
    shared_key,
    pair_characteristic_uuid, 
    notify_characteristic_uuid,
    command_characteristic_uuid,
    PAIR_OP_VERIFY_CREDENTIALS,
    PAIR_OP_GET_MESH_LTK,
    # Classes
    BaseCommandAction,
    # Functions
    authenticate,
    decrypt_notification,
    encode_mesh_credentials,
    encrypt_data,
    reverse_section,
    parse_mac_address,
    aes_att_decryption,
)

# Opcodes from light.rs
LGT_CMD_LIGHT_ONOFF = 0x10
LGT_CMD_LIGHT_CONFIG_GRP = 0x17
LGT_CMD_LIGHT_READ_STATUS = 0x1a
LGT_CMD_LIGHT_STATUS = 0x1b
LGT_CMD_LIGHT_GRP_RSP1 = 0x14
LGT_CMD_LIGHT_GRP_REQ = 0x1d
LGT_CMD_CONFIG_DEV_ADDR = 0x20
LGT_CMD_KICK_OUT = 0x23
LGT_CMD_SET_LIGHT = 0x30
LGT_CMD_SET_MAC_ADDR = 0x31

# Parameter constants
LIGHT_ADD_GRP_PARAM = 0x01
LIGHT_DEL_GRP_PARAM = 0x00
MAX_LUM_BRIGHTNESS_VALUE = 0x7fff


async def main():
    parser = argparse.ArgumentParser(description='TLSR8266 Mesh Node Control Tool')
    parser.add_argument('--address', required=True, help="MAC address")
    parser.add_argument('--mesh-address', type=int, default=0, help="Node mesh address (not required for --get-ltk)")
    parser.add_argument('--mesh-name', required=True)
    parser.add_argument('--mesh-password', required=True)
    parser.add_argument('--verbose', action='store_true')
    
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--on', action='store_true', help="Turn the light ON")
    group.add_argument('--off', action='store_true', help="Turn the light OFF")
    group.add_argument('--brightness', type=lambda x: int(x, 0), help="Set brightness (0-32767)")
    group.add_argument('--temperature', type=lambda x: int(x, 0), help="Set temperature (0-32767)")
    group.add_argument('--add-group', type=lambda x: int(x, 0), help="Add device to group (e.g., 0x8010)")
    group.add_argument('--del-group', type=lambda x: int(x, 0), help="Remove device from group (e.g., 0x8010)")
    group.add_argument('--set-mesh-address', type=int, help="Change node mesh address")
    group.add_argument('--set-mac-address', help="Change device MAC address (format: aabbccddeeff)")
    group.add_argument('--kick-out', action='store_true', help="Factory reset the device")
    group.add_argument('--get-groups', action='store_true', help="Query group memberships")
    group.add_argument('--get-status', action='store_true', help="Query current status")
    group.add_argument('--get-ltk', action='store_true', help="Retrieve the mesh Long-Term Key from device flash")

    args = parser.parse_args()
    
    print(f"Connecting to {args.address}...")
    try:
        async with BleakClient(args.address) as client:
            print("Authenticating...")
            session_key = await authenticate(client, args.mesh_name, args.mesh_password)
            print("Authentication successful.")

            mac_bytes_fwd, mac_bytes_rev = parse_mac_address(args.address)

            async def send_mesh_command(opcode, params):
                action = BaseCommandAction(
                    mac_address=mac_bytes_rev,
                    opcode=opcode,
                    params=params,
                    mesh_address=args.mesh_address,
                    session_key=session_key,
                    vendor_id=0x0211,
                    no_response=True
                ).build_command_action()
                await action.encode_and_send(client)

            if args.get_groups or args.get_status:
                def notification_handler(sender, data):
                    decrypted = decrypt_notification(data, session_key, mac_bytes_fwd)
                    if decrypted is not None:
                        op = decrypted[0] & 0x3F
                        params = decrypted[3:]
                        if op == LGT_CMD_LIGHT_GRP_RSP1:
                            groups = [f"0x{0x8000 | b:04x}" for b in params[:10] if b != 0xFF]
                            print(f"  => Groups: {', '.join(groups) if groups else 'None'}")
                        elif op == LGT_CMD_LIGHT_STATUS:
                            cw = params[0] | (params[1] << 8)
                            ww = params[2] | (params[3] << 8)
                            br = params[4] | (params[5] << 8)
                            print(f"  => Status: CW={cw}, WW={ww}, Brightness={br}")
                        else:
                            print(f"  => Decrypted: op=0x{op:02x} params={params.hex()}")
                    else:
                        print("  [Notification] DECRYPTION FAILED")

                await client.start_notify(notify_characteristic_uuid, notification_handler)
                if args.get_groups:
                    await send_mesh_command(LGT_CMD_LIGHT_GRP_REQ, [0x01, 0x01])
                else:
                    await send_mesh_command(LGT_CMD_LIGHT_READ_STATUS, [0x01])
                await sleep(2)
                await client.stop_notify(notify_characteristic_uuid)

            elif args.on or args.off:
                await send_mesh_command(LGT_CMD_LIGHT_ONOFF, [1 if args.on else 0, 0, 0])
                print(f"Turned {'ON' if args.on else 'OFF'}")

            elif args.brightness is not None:
                val = min(max(args.brightness, 0), MAX_LUM_BRIGHTNESS_VALUE)
                # params[0..2]=val, params[8]=0x01
                p = [val & 0xFF, (val >> 8) & 0xFF, 0, 0, 0, 0, 0, 0, 0x01]
                await send_mesh_command(LGT_CMD_SET_LIGHT, p)
                print(f"Set brightness to {val}")

            elif args.temperature is not None:
                val = min(max(args.temperature, 0), MAX_LUM_BRIGHTNESS_VALUE)
                # params[2..4]=val, params[8]=0x02
                p = [0, 0, val & 0xFF, (val >> 8) & 0xFF, 0, 0, 0, 0, 0x02]
                await send_mesh_command(LGT_CMD_SET_LIGHT, p)
                print(f"Set temperature to {val}")

            elif args.add_group is not None:
                await send_mesh_command(LGT_CMD_LIGHT_CONFIG_GRP, [LIGHT_ADD_GRP_PARAM, args.add_group & 0xFF, (args.add_group >> 8) & 0xFF])
                print(f"Added to group {hex(args.add_group)}")

            elif args.del_group is not None:
                await send_mesh_command(LGT_CMD_LIGHT_CONFIG_GRP, [LIGHT_DEL_GRP_PARAM, args.del_group & 0xFF, (args.del_group >> 8) & 0xFF])
                print(f"Removed from group {hex(args.del_group)}")

            elif args.set_mesh_address is not None:
                val = args.set_mesh_address
                await send_mesh_command(LGT_CMD_CONFIG_DEV_ADDR, [val & 0xFF, (val >> 8) & 0xFF])
                print(f"Requested mesh address change to {val}")

            elif args.set_mac_address is not None:
                mac_hex = args.set_mac_address.replace(':', '').replace('-', '')
                # MAC is stored LSB-first in flash (BLE display order is reversed)
                mac_new = list(reversed(binascii.unhexlify(mac_hex)))
                await send_mesh_command(LGT_CMD_SET_MAC_ADDR, mac_new)
                print(f"Requested MAC address change to {args.set_mac_address}")

            elif args.kick_out:
                # reason 0 = OutOfMesh
                await send_mesh_command(LGT_CMD_KICK_OUT, [0])
                print("Requested factory reset (kick out)")

            elif args.get_ltk:
                plaintext_credentials = encode_mesh_credentials(args.mesh_name, args.mesh_password)
                my_randm = get_random_bytes(8)
                tmp_key = bytearray(my_randm) + b'\x00' * 8
                proof_enc = encrypt_data(tmp_key, plaintext_credentials)

                ltk_request = bytearray(17)
                ltk_request[0] = PAIR_OP_GET_MESH_LTK
                ltk_request[1:9] = my_randm
                ltk_request[9:17] = proof_enc[8:]
                reverse_section(ltk_request, 9, 16)

                await client.write_gatt_char(pair_characteristic_uuid, ltk_request)
                response = await client.read_gatt_char(pair_characteristic_uuid)

                if response[0] != 0x09:
                    print(f"Unexpected response state: {hex(response[0])} (expected 0x09 = RequestingLtk)")
                    sys.exit(1)

                encrypted_ltk = bytes(response[1:17])
                pair_work = bytearray(my_randm) + b'\x00' * 8
                pair_sk = bytearray(c ^ w for c, w in zip(plaintext_credentials, pair_work))
                ltk = aes_att_decryption(pair_sk, encrypted_ltk)
                print(f"LTK: {ltk.hex()}")

            await sleep(1)

    except EOFError:
        pass
    except Exception as e:
        print(f"\nError: {e}")
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())
