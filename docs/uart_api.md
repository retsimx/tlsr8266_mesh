# UART Command Packet Reference

This document describes the UART communication protocol used by the TLSR8266 mesh firmware and the `flash_fw_uart.py` utility.

## Serial Connection

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 |
| Data bits | 8 |
| Stop bits | 1 |
| Parity    | None |
| Default port | `/dev/ttyUSB0` |

## Packet Structure

All UART packets are exactly **44 bytes** long and have the following layout:

| Offset | Length | Field | Description |
|--------|--------|-------|-------------|
| 0 | 1 | Counter | Rolling packet counter (increments with each send) |
| 1 | 1 | Command type | Command category (see [Command Codes](#command-codes)) |
| 2 | 1 | Command code | Specific command within the category (or unused for ACK) |
| 3–41 | 39 | Payload | Command-specific data |
| 42–43 | 2 | CRC16 | CRC-16 checksum of bytes 0–41 (little-endian) |

ACK packets use `0xFF` in the command type field; all other fields are ignored.

## Command Codes

### Host → Device

| Code | Name | Description |
|------|------|-------------|
| `0x01` | `ENABLE_UART` | Enable UART reporting mode on the connected device |
| `0x02` | `LIGHT_CTRL` | Send a mesh command or OTA sub-command to a mesh node |

### Device → Host

| Code | Name | Description |
|------|------|-------------|
| `0x03` | `LIGHT_STATUS` | Light state notification from a device |
| `0x04` | `MESH_MESSAGE` | Forwarded mesh network message |
| `0x05` | `PANIC_MESSAGE` | Firmware panic / error report |
| `0x06` | `PRINT_MESSAGE` | Debug print output from firmware |
| `0xFF` | `ACK` | Acknowledgment of a received host command |

## OTA Sub-Commands

OTA update commands are transported inside `LIGHT_CTRL` (`0x02`) packets. The sub-command code is embedded in the payload along with the target mesh node address.

| Code | Name | Direction | Description |
|------|------|-----------|-------------|
| `0x24` | `OTA_START` | Host → Device | Initiates an OTA update on the target node |
| `0x25` | `OTA_START_RESP` | Device → Host | Response to `OTA_START`; contains current firmware version |
| `0x26` | `OTA_DATA` | Host → Device | Carries an 8-byte firmware data chunk with its packet index |
| `0x27` | `OTA_DATA_RESP` | Device → Host | Acknowledges receipt of a firmware data chunk |
| `0x28` | `OTA_END` | Host → Device | Signals the end of firmware transfer |

## LIGHT_CTRL Payload Layout

When sending a mesh command (including OTA sub-commands), the `LIGHT_CTRL` payload is structured as follows:

| Offset (within payload) | Length | Field | Description |
|-------------------------|--------|-------|-------------|
| 0–1 | 2 | Target address | Mesh node address (little-endian 16-bit) |
| 2 | 1 | Sub-command | OTA command code (e.g. `0x24` for `OTA_START`) |
| 3 | 1 | Sub-command flags | Additional flags (e.g. `0x00` for none) |
| 4+ | up to 15 | Data | Sub-command-specific data |

## OTA Update Flow

The following sequence shows the complete exchange for a firmware update to mesh node `N`:

```
Host                                  Device (node N)
  |                                        |
  |-- LIGHT_CTRL [OTA_START, N] ---------> |
  |<- LIGHT_STATUS [OTA_START_RESP, N] --- |  (contains firmware version)
  |                                        |
  |-- LIGHT_CTRL [OTA_DATA, N, idx=0] ---> |  (8 bytes of firmware)
  |<- LIGHT_STATUS [OTA_DATA_RESP, N] ---- |
  |                                        |
  |  ... (repeat for each 8-byte chunk) ...|
  |                                        |
  |-- LIGHT_CTRL [OTA_END, N] -----------> |
  |                                        |
```

The host retries each packet up to 100 times if no acknowledgment is received within 0.5 seconds.
