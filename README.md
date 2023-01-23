# TLSR 8266 BLE Mesh Firmware in Rust

This project provides a TLSR 8266 BLE mesh firmware written in rust.



For the unmodified mesh SDK, look at the releases or at the early commits of this project.



## References

Heavily inspired by the work from Raphael Baron's project here https://rbaron.net/blog/2021/07/06/Reverse-engineering-the-M6-smart-fitness-band.html (https://github.com/rbaron/m6-reveng)



## Includes

* telink mesh SDK

* tc32 toolchain

* working mesh project and makefile targeting TLSR8266

* telink mesh android app

* raspberry pi pico w SWire interface

* original TYBT1 firmware

* vendor light firmware rewritten in rust (But still using libble from the SDK)

  

### Versions

I ended up using `rustup default nightly-2022-08-06-i686-unknown-linux-gnu` , as it's the last nightly version before LLVM 15 which changed to opaque pointers, and doesn't work correctly with CBE yet.

To get rust ready for development:-

1. `rustup default nightly-2022-08-06-i686-unknown-linux-gnu`
2. `rustup component add rust-src`



### Building

The process to convert Rust code in to the TC32 bytecode is somewhat roundabout. Since the toolchain provided by Telink only supports C and their changes are not open sourced, we have to use their compiler. 

1. Using a Rust compiler that supports the same word size (8266 is a 32bit MCU - so I used the i686 target), generate LLVM bytecode.
2. Transpile that LLVM bytecode to C using the LLVM-CBE project (My port has some required changes here: https://github.com/retsimx/llvm-cbe)
3. Parse the C code as required for uncompilable code. (Thinks like symbols from the Rust std library, weird static definitions from CBE etc)
4. Compile the CBE code to object files using the TC32 toolchain
5. Link the object files to create the final firmware



The entire build process can be run using `make clean && make` from the repository root, the built firmware will be in the `_build` directory. Some makefile changes may need to be made to correct paths to your local LLVM-CBE.



## Flashing

I wrote some other BLE tools that I use for sending the firmware over OTA, but I've not yet uploaded this code. However included in this project is a Raspberry Pi Pico W micropython project that allows SWire communication with the TSLR chips. The client is 95% rbarons code, and the micropython code is mostly my own. It uses PIO to communicate with the MCU, and WiFi to communicate with the client which can run anywhere in your network.