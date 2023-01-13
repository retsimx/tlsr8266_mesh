#### References
Heavily inspired by the work from Raphael Baron's project here https://rbaron.net/blog/2021/07/06/Reverse-engineering-the-M6-smart-fitness-band.html (https://github.com/rbaron/m6-reveng)




#### Includes
* telink mesh SDK (View first few commits for this unmodified)

* tc32 toolchain

* working mesh project and makefile targeting TLSR8266 (View early commits for this)

* telink mesh android app

* raspberry pi pico w SWire interface

* original TYBT1 firmware

* Rust implementation of the light mesh project.

  

#### Versions
rust 1.64.0 works with llvm-cbe built against LLVM 14

Rust needs:

* `rustup default 1.64.0-i686-unknown-linux-gnu`



#### How this works

In a very roundabout and hacky way, the rust project is compiled to LLVM bytecode using a 32 bit target (in this case i686). The LLVM bytecode is then transpiled to C using LLVM-CBE. The generated C code is then parsed through a python script that fixes some issues with the C code. The parsed C code is then built using the original TC32 GCC toolchain.

https://github.com/retsimx/llvm-cbe - this fork of LLVM-CBE is required as it builds against LLVM 14 and includes some extra intrinsics.

Once Rust is installed, just run `make clean && make`, the `_build/*.bin` file is the firmware for flashing.



#### Extra notes

I added my own ram code OTA startup that doesn't need to live in a separate firmware location as required by the original telink mesh projects. This ram code is run immediately when the main function is started before any other code. If it detects a new OTA firmware, it moves it over the old firmware and reboots. Some OTA checks (eg. LIGHT_MODEL) have been relaxed as well to make the process more resilient.