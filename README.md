# TLSR 8266 BLE Mesh Firmware in Rust

This project provides a TLSR 8266 BLE mesh firmware written in rust.



For the unmodified mesh SDK, look at the releases or at the early commits of this project.



## References

Heavily inspired by the work from Raphael Baron's project here https://rbaron.net/blog/2021/07/06/Reverse-engineering-the-M6-smart-fitness-band.html (https://github.com/rbaron/m6-reveng)



And the reverse engineering via Ghidra and trust1995's improved TC32 processor spec https://github.com/trust1995/Ghidra_TELink_TC32



## Includes

* TC32 Toolchain

* 100% Rust BLE Mesh project and makefile targeting TLSR8266 (No libble or vendor library dependence)

* Telink mesh android app (todo - improve)

* Raspberry Pi Pico/W SWire interface

* Original TYBT1 firmware

  

### Versions

I ended up using `nightly-i686-unknown-linux-gnu` for building and running tests, and `thumbv6m-none-eabi` for targeting the MCU, (currently `1.72.0-nightly (8c74a5d27 2023-06-14)` at time of writing)

Make sure you have `rustup` installed - the correct toolchain will be installed automatically by the `rust/rust-toolchain.toml` file, which will install the correct compiler, targets and components.



### Building

The process to convert Rust code in to the TC32 bytecode is somewhat roundabout. Since the toolchain provided by Telink only supports C and their changes are not open sourced, we have to use their compiler - or at least toolchain. Thankfully LLVM can generate way better code (citation needed) than the provided TC32 GCC. Using LLVM in this manner also means you can theoretically compile any language that has an LLVM frontend, for the TLSR8266.



The process is as follows:-

1. Using a Rust compiler that supports the same word size (8266 is a Thumb16 MCU with 32bit word size - so I used the i686 target), generate LLVM bytecode.
2. Fix the LLVM bytecode if required. TC32 doesn't have some Thumb instructions such as LDREX or STREX, nor any profiling like arm.hint or instructions such as SEV. We parse and change these in the IR as required with replacements, or remove them altogether.
3. Send the LLVM IR through an external LLVM build with relevant TC32 changes and emit Thumb16 arm assembly code. I've forked LLVM with the relevant changes here: https://github.com/retsimx/llvm-project (Instructions below on how I build this)
4. Parse the generated Thumb16 assembly and convert instructions to TC32 equivalents where applicable. It's **almost** 1 to 1 mapping between Thumb16 and TC32 instructions. This step also fixes up unique section names and ram-code for use with --gc-sections.
5. Send the parsed TC32 assembly language directly in to the TC32 `as` and generate the object files (For each rust crate - including this project which is itself a library crate)
6. Link all the generated object files together using the vendor linker script.
7. Flash the binary to the MCU (Or BLE OTA it across)
8. ???
9. Profit

The entire build process can be run using `make clean && make` from the repository root, the built firmware will be in the `_build` directory. Some makefile changes may need to be made to correct paths to your local LLVM build.

### Testing
It's also possible to cross compile the test suite and run it locally on the host to validate the code. This works by more or less remapping the register space of the MCU to a local u8 array which is read/written instead and can be confirmed to be correct by tests. To run the test suite, you'll need an x86 compatible machine, and there is a helper script `rust/run_tests.sh`

Todo: Allow mocking complex processes such as flash operations which require sequences of responding to register changes.



## Flashing

I wrote some other BLE tools that I use for sending the firmware over OTA, but I've not yet uploaded this code. However included in this project is a Raspberry Pi Pico/W micropython project that allows SWire communication with the TSLR chips. The client is 95% rbarons code, and the micropython code is mostly my own. It uses PIO to communicate with the MCU, and the micropython Raw REPL to interface with the pico.



Depending on the crystal speed attached to your TLSR chip, you might need to fiddle with the timings. I've got 12Mhz and 16Mhz devices - and they need different timings. I've provided timings that work for me in the code.



## LLVM

The steps I use to build my LLVM fork are as follows:-

* Check out the repo
  * `git clone https://github.com/retsimx/llvm-project.git`
* Create a build directory
  * `mkdir build; cd build`
* Configure the build (Overly verbose, I know - we're only building the ARM target)
  * `CMAKE_PREFIX_PATH="" DESTDIR="" "cmake" "../llvm-project/llvm" "-G" "Ninja" "-DLLVM_ENABLE_ASSERTIONS=ON" "-DLLVM_ENABLE_PLUGINS=OFF" "-DLLVM_TARGETS_TO_BUILD=ARM" "-DLLVM_INCLUDE_EXAMPLES=OFF" "-DLLVM_INCLUDE_DOCS=OFF" "-DLLVM_INCLUDE_BENCHMARKS=OFF" "-DLLVM_INCLUDE_TESTS=OFF" "-DLLVM_ENABLE_TERMINFO=OFF" "-DLLVM_ENABLE_LIBEDIT=OFF" "-DLLVM_ENABLE_BINDINGS=OFF" "-DLLVM_ENABLE_Z3_SOLVER=OFF" "-DLLVM_PARALLEL_COMPILE_JOBS=12" "-DLLVM_ENABLE_WARNINGS=ON" "-DLLVM_INSTALL_UTILS=ON" "-DLLVM_ENABLE_ZSTD=OFF" "-DLLVM_ENABLE_ZLIB=ON" "-DLLVM_ENABLE_LIBXML2=OFF" "-DCMAKE_INSTALL_MESSAGE=LAZY" "-DCMAKE_C_COMPILER=cc" "-DCMAKE_CXX_COMPILER=c++" "-DCMAKE_ASM_COMPILER=cc" "-DCMAKE_C_FLAGS=-ffunction-sections -fdata-sections -fPIC -m64" "-DCMAKE_CXX_FLAGS=-ffunction-sections -fdata-sections -fPIC -m64" "-DCMAKE_SHARED_LINKER_FLAGS=" "-DCMAKE_MODULE_LINKER_FLAGS=" "-DCMAKE_EXE_LINKER_FLAGS=" "-DCMAKE_ASM_FLAGS= -ffunction-sections -fdata-sections -fPIC -m64" "-DCMAKE_BUILD_TYPE=Release"`
* Build LLVM
  * `ninja`
* The resulting files should be in `./bin`, you need to make sure that the `llc` referenced in `toolchain/rust2c.sh` is correct.