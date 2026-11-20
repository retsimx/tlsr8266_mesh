# Documentation

### [Firmware](firmware.md)
Comprehensive guide to building custom firmware for TLSR8266 devices, including the unique process of using Rust with LLVM to generate TC32 compatible bytecode. Covers toolchain setup, build process workflow, and LLVM compilation instructions.

### [SWire Interface](swire.md)
Guide to the Rust SWire probe: `probe-pico` firmware for a Raspberry Pi Pico/Pico W and the `probe-tlsr` host CLI. Covers the circuit diagram and pin mapping, boot-time crystal auto-detection, flashing, RTT logs, crash inspection, and ready-to-run stack-watermark and NVM backup/restore recipes.

### [BLE OTA Updater](ble.md)
Complete guide for over-the-air firmware updates via Bluetooth Low Energy. Covers detailed command line arguments, the secure update process workflow, and troubleshooting tips for failed updates.

### [UART Updater](uart.md)
Instructions for updating firmware using the UART interface, which allows remote updates to mesh nodes that aren't within BLE range. Documents the flash_fw_uart.py script parameters and notes that UART updates may take significantly longer for distant mesh nodes.

### [Mesh Adoption](adoption.md)
Detailed process for adding new devices to the mesh network, including mesh address management, security best practices, and step-by-step instructions for using the mesh_add.py script. Covers mesh creation, custom Long-Term Key (LTK) generation, and troubleshooting.

### [Pairing](pairing.md)
Comprehensive documentation on the BLE pairing and security mechanisms for the TLSR8266 mesh system. Covers the pairing state machine, protocol flow, security features, key management, credential verification methods, and includes practical examples for implementation.

### [UART Command Packet Examples](uart_api.md)
Reference documentation with examples of UART command packets for device control (work in progress).

### [BLE & Mesh Hardware/Software Reference](ble_hardware_reference.md)
Comprehensive technical datasheet covering the complete BLE and proprietary mesh implementation. Includes hardware register reference, RF subsystem configuration, interrupt architecture, BLE link-layer state engine, packet formats, GATT attribute table, pairing protocol, OTA update mechanisms, AES cryptography, flash storage layout, and full documentation of the proprietary flooding mesh protocol: access code derivation, mesh/BLE coexistence state machine, packet routing, node status propagation, over-mesh re-pairing, and timing reference.

### [TLSR8266 TC32 Instruction Set](tc32_instruction_set.md)
Recovered executable reference for the 8266 `.code 16` instruction set,
based on the shipped Telink assembler/disassembler opcode tables and validated
assembler probes. Includes exact 16/32-bit patterns, operand domains,
assembler/disassembler spelling differences, atomic/wait limitations, and
explicitly unsupported inherited ARM/Thumb forms.

