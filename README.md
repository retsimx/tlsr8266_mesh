# TLSR8266 BLE Mesh Firmware in Rust

[![CI](https://github.com/retsimx/tlsr8266_mesh/actions/workflows/ci.yml/badge.svg)](https://github.com/retsimx/tlsr8266_mesh/actions/workflows/ci.yml)

A 100% Rust BLE mesh firmware for TLSR8266-based smart lights (such as the TYBT1), with Python utilities for flashing, OTA updates, and mesh management. The firmware runs without any Telink proprietary library dependency, using a custom LLVM backend to generate TC32-compatible code from Rust source.

For the unmodified mesh SDK, see the project releases or the early commits of this repository.

## Hardware

This project targets the following hardware:

- **TLSR8266-based devices** — e.g. TYBT1 smart light bulbs
- **E104-BT05-TB** — a TLSR8266 development board used for UART-based mesh management ([product page](https://www.cdebyte.com/products/E104-BT05-TB))
- **Raspberry Pi Pico or Pico W** — used as a SWire programmer for initial flashing (physical access required)

## Includes

- TC32 toolchain
- 100% Rust BLE mesh firmware with Makefile (no `libble` or vendor library dependency)
- Raspberry Pi Pico/W SWire interface for direct flash programming
- Original TYBT1 firmware backup
- BLE OTA updater (compatible with OEM firmware and this firmware)
- UART OTA updater (requires this firmware on the E104-BT05-TB)
- Mesh adoption and management utilities

## References

Heavily inspired by [Raphael Baron's reverse-engineering work](https://rbaron.net/blog/2021/07/06/Reverse-engineering-the-M6-smart-fitness-band.html) on the M6 fitness band ([GitHub](https://github.com/rbaron/m6-reveng)), and [trust1995's improved TC32 processor specification](https://github.com/trust1995/Ghidra_TELink_TC32) for Ghidra.

## Documentation

See the full documentation [here](docs/README.md).