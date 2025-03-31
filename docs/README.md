# Documentation

### [Firmware](firmware.md)
Comprehensive guide to building custom firmware for TLSR8266 devices, including the unique process of using Rust with LLVM to generate TC32 compatible bytecode. Covers toolchain setup, build process workflow, and LLVM compilation instructions.

### [SWire Interface](swire.md)
Details about the Single-Wire Interface (SWire) protocol using a Raspberry Pi Pico/W with micropython to program TLSR chips. Includes circuit diagram, pin mappings, timing considerations for different crystal speeds, and usage instructions for flashing firmware.

### [BLE OTA Updater](ble.md)
Complete guide for over-the-air firmware updates via Bluetooth Low Energy. Features critical warnings about clock speed compatibility, detailed command line arguments, secure update process workflow, and troubleshooting tips to prevent device bricking.

### [UART Updater](uart.md)
Instructions for updating firmware using the UART interface, which allows remote updates to mesh nodes that aren't within BLE range. Documents the flash_fw_uart.py script parameters and notes that UART updates may take significantly longer for distant mesh nodes.

### [Mesh Adoption](adoption.md)
Detailed process for adding new devices to the mesh network, including mesh address management, security best practices, and step-by-step instructions for using the mesh_add.py script. Covers mesh creation, custom Long-Term Key (LTK) generation, and troubleshooting.

### [Pairing](pairing.md)
Comprehensive documentation on the BLE pairing and security mechanisms for the TLSR8266 mesh system. Covers the pairing state machine, protocol flow, security features, key management, credential verification methods, and includes practical examples for implementation.

### [UART Command Packet Examples](uart_api.md)
Reference documentation with examples of UART command packets for device control (work in progress).

