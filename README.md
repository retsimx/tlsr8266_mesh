# TLSR 8266 BLE Mesh Firmware in Rust

This project provides a TLSR 8266 BLE mesh firmware written in rust, with accompanying tools for flashing and OTA updates, and mesh management.



For the unmodified mesh SDK, look at the releases or at the early commits of this project.



## References

Heavily inspired by the work from Raphael Baron's project here https://rbaron.net/blog/2021/07/06/Reverse-engineering-the-M6-smart-fitness-band.html (https://github.com/rbaron/m6-reveng)



And the reverse engineering via Ghidra and trust1995's improved TC32 processor spec https://github.com/trust1995/Ghidra_TELink_TC32



## Includes

* TC32 Toolchain
* 100% Rust BLE Mesh project and makefile targeting TLSR8266 (No libble or vendor library dependence)
* Raspberry Pi Pico/W SWire interface for flashing the TLSR8266 (Physical access required)
* Original TYBT1 firmware
* OTA update via Bluetooth (Works with the originally shipped TYBT1 firmware, and with this firmware)
* OTA update via UART (Works only with this firmware - and requires a board such as the https://www.cdebyte.com/products/E104-BT05-TB)
* Mesh adoption utility



## Documentation

See the documentation [here](docs/README.md)