# TLSR8266 Mesh Utilities

A collection of Python scripts for managing TLSR8266-based mesh lights over BLE.

## Included Tools

### `mesh_node.py`
Directly controls provisioned mesh nodes via BLE. Supports light control,
group management, device configuration, status queries, LTK retrieval, and
factory reset.

**Usage:**
```bash
python3 mesh_node.py --address <mac> --mesh-address <id> \
    --mesh-name <name> --mesh-password <pwd> [command]
```

See [docs/mesh_node.md](../../docs/mesh_node.md) for full documentation.

### `mesh_add.py`
Provisions unprovisioned devices (advertising as `out_of_mesh`) into an
existing mesh network. Handles the full pairing handshake, setting mesh
name/password/LTK, and assigning a mesh address.

**Usage:**
```bash
python3 mesh_add.py --mesh_address <id> --mesh_name <name> \
    --mesh_password <pwd> --mesh_ltk <hex>
```

### `flash_fw.py`
Flashes new firmware to a provisioned device over BLE OTA.

### `flash_fw_uart.py`
Flashes new firmware via UART/SWire for devices that cannot be reached over BLE.

### `uart_debug_monitor.py`
Monitors UART debug output from a connected device.

### `mesh_common.py`
Shared library used by all scripts above. Contains the authentication
handshake, AES helpers, notification decryption, and BLE command encoding.

## Setup

1. Create a virtual environment:
   ```bash
   python3 -m venv venv
   ```
2. Install dependencies:
   ```bash
   ./venv/bin/pip install -r requirements.txt
   ```

## Documentation

- [Mesh Node Control](../../docs/mesh_node.md)
- [Pairing and Security](../../docs/pairing.md)
