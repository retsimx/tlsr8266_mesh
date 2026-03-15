# Mesh Node Control Utility

`mesh_node.py` provides direct Bluetooth Low Energy (BLE) control of individual
TLSR8266 mesh light nodes, without going through the smart-device-daemon or
MQTT broker. It is the primary tool for diagnosis, configuration, and
maintenance of provisioned nodes.

## Features

- **Authentication**: Full BLE mesh handshake using existing mesh credentials,
  deriving a per-session AES key.
- **Light Control**: Turn on/off, set brightness (0–32767), set colour temperature (0–32767).
- **Group Management**: Add/remove nodes from mesh groups, query current memberships.
- **Device Configuration**: Change node mesh address, change BLE MAC address.
- **Maintenance**: Factory reset (kick out), retrieve the mesh Long-Term Key (LTK).
- **Status Querying**: Read current CW, WW, and brightness values.
- **Notification Decryption**: Full AES-CCM decryption and MIC verification for
  all slave-to-master responses.

## Prerequisites

- Python 3.8 or higher
- `bleak` and `pycryptodomex` libraries (see `utilities/meshutils/requirements.txt`)

## Usage

```bash
python3 mesh_node.py --address <mac> --mesh-address <node_id> \
    --mesh-name <name> --mesh-password <password> <command>
```

`--mesh-address` is not required for `--get-ltk`.

## Arguments

| Argument | Description |
|---|---|
| `--address` | BLE MAC address of the target node (e.g. `12:34:56:78:9A:1D`) |
| `--mesh-address` | Node address within the mesh (1–63) |
| `--mesh-name` | Mesh network name |
| `--mesh-password` | Mesh network password |
| `--verbose` | Print raw decrypted notification bytes |

## Commands (mutually exclusive)

| Command | Description |
|---|---|
| `--on` / `--off` | Turn the light on or off |
| `--brightness <val>` | Set cold-white brightness (0–32767) |
| `--temperature <val>` | Set warm-white / colour temperature (0–32767) |
| `--get-status` | Query current CW, WW, and brightness values |
| `--get-groups` | Query which mesh groups the node belongs to |
| `--add-group <addr>` | Add the node to a mesh group (e.g. `0x8010`) |
| `--del-group <addr>` | Remove the node from a mesh group |
| `--set-mesh-address <N>` | Change the node's mesh address (device reboots) |
| `--set-mac-address <mac>` | Overwrite the BLE MAC stored in flash (device reboots) |
| `--kick-out` | Factory-reset: clears provisioning data, device reboots as `out_of_mesh` |
| `--get-ltk` | Retrieve the mesh Long-Term Key from device flash |

## Examples

```bash
# Query status
python3 mesh_node.py --address 12:34:56:78:9A:1D --mesh-address 29 \
    --mesh-name mlm --mesh-password your-mesh-password --get-status

# Set brightness to 50%
python3 mesh_node.py --address 12:34:56:78:9A:1D --mesh-address 29 \
    --mesh-name mlm --mesh-password your-mesh-password --brightness 16384

# Add to the Living Room group
python3 mesh_node.py --address 12:34:56:78:9A:1D --mesh-address 29 \
    --mesh-name mlm --mesh-password your-mesh-password --add-group 0x8010

# Factory reset, then re-provision
python3 mesh_node.py --address 12:34:56:78:9A:1D --mesh-address 29 \
    --mesh-name mlm --mesh-password your-mesh-password --kick-out
python3 mesh_add.py --mesh_address 29 --mesh_name mlm \
    --mesh_password your-mesh-password --mesh_ltk <ltk_hex>

# Retrieve the LTK (no mesh-address needed)
python3 mesh_node.py --address 12:34:56:78:9A:1D \
    --mesh-name mlm --mesh-password your-mesh-password --get-ltk
```

## Protocol Details

### Notification Decryption

Slave-to-master notifications are encrypted with AES-CCM. The Initialization
Vector is constructed as:

```
IVS = MAC_ID[0:3] | sno[0:3] | src[0:2]
```

where `MAC_ID` is in firmware memory order (LSB-first, reversed from BLE
display order). A 2-byte MIC is verified before the payload is accepted.

### LTK Retrieval

The `--get-ltk` command uses a second authenticated challenge (`PAIR_OP_GET_MESH_LTK`)
after the normal login handshake. The firmware responds with the LTK encrypted
as `aes_att_encryption(pair_sk, ltk)`, where `pair_sk` is derived from the
mesh name, password, and the fresh random challenge. The script decrypts this
with `aes_att_decryption` to recover the plaintext LTK.
