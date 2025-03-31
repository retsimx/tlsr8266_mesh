# UART Updater

It is possible to use the UART updater once the E104-BT05-TB has been updated with this firmware. The UART updater can update both the TB and any node in the connected mesh. Note that the UART updater is slower than the BLE updater, but doesn't require you to be in range of the node you're trying to update. It's not uncommon for a firmware update to a node that is several hops away on the mesh to take 30 minutes or more in some cases.

## Firmware Update Process

The UART updater implements a firmware update protocol over a serial connection. This allows you to update devices that are:
- Connected directly to your computer via UART
- Connected indirectly through the mesh network via a UART-connected device

The update process consists of the following phases:
1. UART Communication Initialization
2. Firmware Version Verification
3. Crystal Frequency Compatibility Check
4. Chunked Firmware Transfer

## Prerequisites

Before using the UART updater:

1. Ensure the device is powered on and connected to a USB-to-UART adapter
2. Make sure you have Python 3 and required dependencies installed (run `pip install -r utilities/meshutils/requirements.txt`)
3. Know the mesh address of the target device
4. Have the appropriate firmware binary file ready
5. Verify the target device is accessible through the mesh network if updating a remote node

## Command Line Arguments

The `meshutils/flash_fw_uart.py` script requires the following parameters:

* `--fw_file` (required): The firmware binary file to flash to the target device
* `--mesh_address` (required): The mesh node address to flash the firmware to (This is the address provided to the node when it was paired)
* `--force` (optional): Flag to bypass version checks. This will allow flashing older firmware or firmware with the same version number, but will NOT bypass clock speed compatibility checks

## Critical Warning: Clock Speed Compatibility

> **⚠️ WARNING: Clock Speed Mismatch Will Permanently Brick Your Device ⚠️**
>
> TLSR8266 devices operate at either 12MHz or 16MHz clock speeds. The firmware **MUST** match the device's hardware clock speed.
>
> - **TYBT1 devices typically use a 12MHz crystal**
> - The UART updater includes a safety check that will prevent flashing incompatible firmware
> - This check CANNOT be bypassed even with the `--force` flag
> - Recovery from a clock speed mismatch is generally impossible once bricked

## Usage Examples

```bash
# Update firmware on device with mesh address 5
python flash_fw_uart.py --fw_file firmware.bin --mesh_address 5

# Force update when flashing same or older version
python flash_fw_uart.py --fw_file firmware.bin --mesh_address 5 --force
```

## Update Process Details

When you run the UART update script, it will:

1. Initialize the UART connection (default: /dev/ttyUSB0 at 115200 baud)
2. Enable UART reporting mode on the connected device
3. Send an OTA start command to the target device
4. Verify firmware compatibility:
   - Check current firmware version on the device
   - Compare with the new firmware version
   - Validate crystal frequency compatibility (12MHz vs 16MHz)
5. Transfer firmware in small chunks (8 bytes per packet)
6. Display progress as a percentage along with elapsed time
7. Send OTA completion command when all chunks are transferred
8. Verify successful installation

## Progress Monitoring

During the update process, the script will display:
- Percentage completion
- Elapsed time since the start of the update
- Any errors or issues encountered during transfer

For distant mesh nodes, the update can be very slow due to the multi-hop mesh network communication. Patience is required, particularly for nodes that are several hops away from the UART-connected device.

## Troubleshooting

If the firmware update fails:

- **Connection issues**:
  - Verify the device is connected to the correct serial port (default is /dev/ttyUSB0)
  - Check physical connections and ensure proper power supply
  - Try resetting the UART connection

- **Non-responsive device**:
  - The script will retry communications multiple times before giving up
  - If the device "disappears" during update, check physical connections
  - Power cycle the device and try again

- **Version check failures**:
  - Use `--force` when intentionally downgrading or flashing same version
  - NEVER try to bypass clock speed mismatch warnings

- **Transfer interruptions**:
  - Keep the UART connection stable during the entire update
  - Ensure the device has a stable power supply

## Hardware Setup

The UART updater expects a standard USB-to-UART adapter connected to the E104-BT05-TB module with the following connections:

- UART TX from computer connected to RX on the module
- UART RX from computer connected to TX on the module
- GND connected between the adapter and the module
- VCC should be provided by a stable power source (either through USB or external)

The default serial port is `/dev/ttyUSB0` with a baud rate of 115200. If your setup uses a different port, you'll need to modify the `reset_connection()` function in the script.

