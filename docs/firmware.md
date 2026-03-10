# Firmware Development Guide

## Toolchain Versions

The required Rust toolchain for this project includes:
- `nightly-i686-unknown-linux-gnu` for building and running tests
- `thumbv6m-none-eabi` for targeting the MCU (currently `nightly-2025-03-30`)

> **Note:** Make sure you have `rustup` installed. The correct toolchain will be installed automatically by the `rust/rust-toolchain.toml` file, which will install the correct compiler, targets and components.

## Building the Firmware

The process to convert Rust code into TC32 bytecode is somewhat roundabout. Since the toolchain provided by Telink only supports C and their changes are not open sourced, it's necessary to use their compiler - or at least toolchain. Thankfully LLVM can generate more optimised code than the provided TC32 GCC, and using LLVM in this manner also means it's theoretically possible to compile any language that has an LLVM frontend for the TLSR8266.

### Build Process

The compilation process follows these steps:

1. **Generate LLVM IR**: Using a Rust compiler that supports the same word size (8266 is a Thumb16 MCU with 32bit word size - so i686 target is used), the Rust crates are compiled to LLVM IR (`.ll` files).

2. **Generate TC32 assembly**: The LLVM IR is compiled by a patched `llc` from our LLVM fork which emits TC32-compatible assembly directly. The fork handles all instruction renaming, section layout, and TC32-specific fixups natively — no post-processing scripts required. The forked LLVM is available at: https://github.com/retsimx/llvm-project (see [LLVM build instructions](#llvm) below)

3. **Generate object files**: The TC32 assembly is assembled directly by the TC32 `as` into object files (one per Rust crate, plus the C startup file).

4. **Link object files**: All the generated object files are linked together using the vendor linker script with `--gc-sections` to eliminate unused code.

5. **Flash the firmware**: The binary is flashed to the MCU (or BLE/UART OTA it across)

### Building Instructions

The entire build process can be run using:

```bash
make clean && make
```

from the repository root. The built firmware will be in the `_build` directory. The Makefile assumes `llc` is at `../../llvm/build/bin/llc` relative to this repo; override with `LLC=/path/to/llc make` if needed.

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

### Prerequisites

The following tools must be installed before building LLVM:

- `cmake` (3.20 or later recommended)
- `ninja`
- A C/C++ compiler: `clang` and `clang++` (recommended), or `gcc`/`g++`

On Ubuntu/Debian:

```bash
sudo apt-get install -y cmake ninja-build clang lld
```

### Build Steps

1. **Clone the repository** (use the `tc32` branch):
   ```bash
   git clone -b tc32 https://github.com/retsimx/llvm-project.git
   ```

2. **Create a build directory**:
   ```bash
   mkdir build
   cd build
   ```

3. **Configure the build** (ARM target only, building `llc`, `llvm-link`, and `opt`):
   ```bash
   cmake "../llvm-project/llvm" -G Ninja \
   -DLLVM_ENABLE_ASSERTIONS=ON \
   -DLLVM_ENABLE_PLUGINS=OFF \
   -DLLVM_TARGETS_TO_BUILD=ARM \
   -DLLVM_TOOLS_TO_BUILD="llc;llvm-link;opt" \
   -DLLVM_INCLUDE_EXAMPLES=OFF \
   -DLLVM_INCLUDE_DOCS=OFF \
   -DLLVM_INCLUDE_BENCHMARKS=OFF \
   -DLLVM_INCLUDE_TESTS=OFF \
   -DLLVM_ENABLE_TERMINFO=OFF \
   -DLLVM_ENABLE_LIBEDIT=OFF \
   -DLLVM_ENABLE_BINDINGS=OFF \
   -DLLVM_ENABLE_Z3_SOLVER=OFF \
   -DLLVM_ENABLE_WARNINGS=ON \
   -DLLVM_ENABLE_ZSTD=OFF \
   -DLLVM_ENABLE_ZLIB=ON \
   -DLLVM_ENABLE_LIBXML2=OFF \
   -DCMAKE_C_COMPILER=clang \
   -DCMAKE_CXX_COMPILER=clang++ \
   -DCMAKE_BUILD_TYPE=Release
   ```

4. **Build the tools**:
   ```bash
   ninja -j$(nproc) llc llvm-link opt
   ```

5. **Locate the binaries**: The outputs will be at `build/bin/llc`, `build/bin/llvm-link`, and `build/bin/opt`. Update the `LLC`, `LLVM_LINK`, and `OPT` variables in the `Makefile` (or pass them on the command line) if your build directory differs from the default:

   ```bash
   LLC=/path/to/llc LLVM_LINK=/path/to/llvm-link OPT=/path/to/opt make
   ```
