# Firmware Development Guide

## Toolchain Versions

The required Rust toolchain for this project includes:
- `nightly-i686-unknown-linux-gnu` for building and running tests
- `thumbv6m-none-eabi` for targeting the MCU (currently `nightly-2025-03-30`)

> **Note:** Make sure you have `rustup` installed. The correct toolchain will be installed automatically by the `rust/rust-toolchain.toml` file, which will install the correct compiler, targets and components.

## Building the Firmware

The process to convert Rust code into TC32 bytecode is somewhat roundabout. Since the toolchain provided by Telink only supports C and their changes are not open sourced, it's necessary to use their compiler - or at least toolchain. Thankfully LLVM can generate better code (citation needed) than the provided TC32 GCC. Using LLVM in this manner also means it's theoretically possible to compile any language that has an LLVM frontend for the TLSR8266.

### Build Process

The compilation process follows these steps:

1. **Generate LLVM bytecode**: Using a Rust compiler that supports the same word size (8266 is a Thumb16 MCU with 32bit word size - so i686 target is used)

2. **Fix LLVM bytecode**: TC32 doesn't have some Thumb instructions such as LDREX or STREX, nor any profiling like arm.hint or instructions such as SEV. The IR is parsed and changed as required with replacements, or instructions are removed altogether.

3. **Generate Thumb16 assembly**: The LLVM IR is sent through an external LLVM build with relevant TC32 changes to emit Thumb16 ARM assembly code. The forked LLVM with the relevant changes is available at: https://github.com/retsimx/llvm-project (see [LLVM build instructions](#llvm) below)

4. **Convert to TC32 assembly**: The generated Thumb16 assembly is parsed and instructions are converted to TC32 equivalents where applicable. It's **almost** 1:1 mapping between Thumb16 and TC32 instructions. This step also fixes up unique section names and ram-code for use with --gc-sections.

5. **Generate object files**: The parsed TC32 assembly language is sent directly into the TC32 `as` to generate the object files (for each Rust crate, including this project which is itself a library crate)

6. **Link object files**: All the generated object files are linked together using the vendor linker script.

7. **Flash the firmware**: The binary is flashed to the MCU (or BLE/UART OTA it across)

### Building Instructions

The entire build process can be run using:

```bash
make clean && make
```

from the repository root. The built firmware will be in the `_build` directory. Some makefile changes may need to be made to correct paths to the local LLVM build.

## Testing

> **Status:** Work in Progress

It's also possible to cross compile the test suite and run it locally on the host to validate the code. This works by remapping the register space of the MCU to a local u8 array which is read/written instead and can be confirmed to be correct by tests.

To run the test suite, an x86 compatible machine is needed, and there is a helper script:

```bash
./rust/run_tests.sh
```

**Todo:** Allow mocking complex processes such as flash operations which require sequences of responding to register changes.

<a id="llvm"></a>
## LLVM Build Instructions

The steps to build the required LLVM fork are as follows:

1. **Clone the repository**:
   ```bash
   git clone https://github.com/retsimx/llvm-project.git
   ```

2. **Create a build directory**:
   ```bash
   mkdir build
   cd build
   ```

3. **Configure the build** (Only building the ARM target):
   ```bash
   CMAKE_PREFIX_PATH="" DESTDIR="" "cmake" "../llvm-project/llvm" "-G" "Ninja" \
   "-DLLVM_ENABLE_ASSERTIONS=ON" \
   "-DLLVM_ENABLE_PLUGINS=OFF" \
   "-DLLVM_TARGETS_TO_BUILD=ARM" \
   "-DLLVM_INCLUDE_EXAMPLES=OFF" \
   "-DLLVM_INCLUDE_DOCS=OFF" \
   "-DLLVM_INCLUDE_BENCHMARKS=OFF" \
   "-DLLVM_INCLUDE_TESTS=OFF" \
   "-DLLVM_ENABLE_TERMINFO=OFF" \
   "-DLLVM_ENABLE_LIBEDIT=OFF" \
   "-DLLVM_ENABLE_BINDINGS=OFF" \
   "-DLLVM_ENABLE_Z3_SOLVER=OFF" \
   "-DLLVM_PARALLEL_COMPILE_JOBS=12" \
   "-DLLVM_ENABLE_WARNINGS=ON" \
   "-DLLVM_INSTALL_UTILS=ON" \
   "-DLLVM_ENABLE_ZSTD=OFF" \
   "-DLLVM_ENABLE_ZLIB=ON" \
   "-DLLVM_ENABLE_LIBXML2=OFF" \
   "-DCMAKE_INSTALL_MESSAGE=LAZY" \
   "-DCMAKE_C_COMPILER=cc" \
   "-DCMAKE_CXX_COMPILER=c++" \
   "-DCMAKE_ASM_COMPILER=cc" \
   "-DCMAKE_C_FLAGS=-ffunction-sections -fdata-sections -fPIC -m64" \
   "-DCMAKE_CXX_FLAGS=-ffunction-sections -fdata-sections -fPIC -m64" \
   "-DCMAKE_SHARED_LINKER_FLAGS=" \
   "-DCMAKE_MODULE_LINKER_FLAGS=" \
   "-DCMAKE_EXE_LINKER_FLAGS=" \
   "-DCMAKE_ASM_FLAGS= -ffunction-sections -fdata-sections -fPIC -m64" \
   "-DCMAKE_BUILD_TYPE=Release"
   ```

4. **Build LLVM**:
   ```bash
   ninja llc
   ```

5. **Locate the resulting files**: The output should be in `./bin`. Make sure that the `llc` referenced in `toolchain/rust2c.sh` points to the correct location.