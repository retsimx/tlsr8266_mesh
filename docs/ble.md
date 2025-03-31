# BLE OTA (Over-the-Air Updates)

The TLSR8266 mesh system supports firmware updates over Bluetooth Low Energy (BLE). This feature allows you to update device firmware without requiring physical access to programming pins.

## Firmware Update Compatibility

The included BLE tools provide a `flash_fw.py` script that can be used to flash new firmware to TLSR8266 devices such as the TYBT1. This script is compatible with:
- The OEM firmware that ships with these devices
- Custom firmware built from this codebase

## Critical Warning: Clock Speed Compatibility

> **⚠️ WARNING: Clock Speed Mismatch Will Permanently Brick Your Device ⚠️**
>
> TLSR8266 devices operate at either 12MHz or 16MHz clock speeds. The firmware **MUST** match the device's hardware clock speed.
>
> - **TYBT1 devices typically use a 12MHz crystal**
> - Flashing 16MHz firmware to a 12MHz device (or vice versa) will **permanently brick** the device
> - Recovery from a clock speed mismatch is generally impossible because:
>   - TYBT1 modules are typically soldered to boards without access to the SWS programming pad

When upgrading from the OEM firmware, you'll need to pass the `--force` flag to override version checks, but you should **NEVER** override the clock speed check unless you are 100% certain about the device's crystal frequency.

## Using the Firmware Update Script

The `meshutils/flash_fw.py` script performs secure OTA updates through the following process:
1. Authentication with the device using mesh credentials
2. Verification of firmware compatibility (version and clock speed)
3. Chunked transfer of encrypted firmware data
4. Verification of successful installation

### Prerequisites

Before running the script:

1. Ensure the device is powered on and within BLE range
2. Make sure you have Python 3 and required dependencies installed (run `pip install -r utilities/meshutils/requirements.txt`)
3. Ensure you have Bluetooth permissions on your system
4. Know the current mesh credentials of the device (name and password)

### Command Line Arguments

The script requires the following arguments:

* `--mesh_name` (required): The mesh name that the device is currently paired with. For unpaired devices, this is usually `out_of_mesh`.
* `--mesh_password` (required): The mesh password that the device is currently using. For unpaired devices, this is usually `123`.
* `--fw_file` (required): Path to the firmware binary file (.bin) to flash to the device.
* `--mac` (required): The BLE MAC address of the target device in standard format, e.g., `BC:23:4C:07:CE:CE`.
* `--force` (optional): Flag to bypass version checks. This will allow flashing older firmware or firmware with the same version number, but will NOT bypass clock speed compatibility checks unless explicitly modified.

### Usage Examples

```bash
# Basic firmware update to a paired device
python flash_fw.py --fw_file firmware.bin --mac BC:23:4C:07:CE:CE --mesh_name myhome --mesh_password securepass

# Update an unpaired device (factory default credentials)
python flash_fw.py --fw_file firmware.bin --mac BC:23:4C:07:CE:CE --mesh_name out_of_mesh --mesh_password 123

# Force update when flashing same or older version (e.g., when flashing custom firmware over OEM)
python flash_fw.py --fw_file firmware.bin --mac BC:23:4C:07:CE:CE --mesh_name myhome --mesh_password securepass --force
```

### Update Process

During the update, the script will:
1. Connect to the specified device
2. Authenticate using the provided mesh credentials
3. Check firmware compatibility (version number and clock speed)
4. Transfer firmware data in encrypted chunks
5. Display progress as a percentage
6. Verify successful installation

A successful update will end with a confirmation message. The device will automatically reboot with the new firmware.

### Troubleshooting

If the firmware update fails:

- **Connection issues**:
  - Verify the device is powered on and within range
  - Check that the MAC address is correct
  - Ensure Bluetooth is enabled on your computer

- **Authentication failures**:
  - Confirm you're using the correct mesh name and password
  - For factory reset devices, use the default credentials

- **Version check failures**:
  - Use `--force` when intentionally downgrading
  - NEVER force-override clock speed mismatch warnings

- **Transfer interruptions**:
  - Keep the device within stable BLE range during the entire update
  - Ensure the device has stable power supply

### Recovery Options

If a device becomes unresponsive after a failed update (but not due to clock mismatch):
1. Try resetting the device by power cycling
2. Attempt the firmware update process again
3. For devices with accessible SWIRE (SWS) pads, use direct programming via the Raspberry Pi Pico SWIRE tools included in the `utilities/picow_swire` directory. See [SWIRE Interface](swire.md) for detailed instructions on wiring connections and usage.