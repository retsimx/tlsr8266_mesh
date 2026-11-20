# Firmware Development Guide

## Toolchain Versions

`rust/rust-toolchain.toml` pins the Rust toolchain to `nightly-2026-11-20` (`minimal` profile) with:
- **Components**: `rust-src` (for `-Z build-std=core`), `rustc`, `rustfmt`, `clippy`, and `llvm-tools`
- **Targets**:
  - `thumbv6m-none-eabi` for compiling the firmware to LLVM IR
  - `i686-unknown-linux-gnu` for running host tests (`./rust/run_tests.sh`) and Clippy

> **Note:** Make sure you have `rustup` installed. Running Cargo or `rustup show` inside `rust/` automatically installs the pinned `nightly-2026-11-20` toolchain, targets, and components from `rust/rust-toolchain.toml`. Additionally, `rustc`'s LLVM major version must not exceed the `retsimx/llvm-project` (`tc32` branch) fork's LLVM major version (`sdk/build_firmware.py` enforces this check).

## Building the Firmware

The process to convert Rust code into TC32 bytecode is somewhat roundabout. Since the toolchain provided by Telink only supports C and their changes are not open sourced, it's necessary to use their binutils (`tc32-elf-{gcc,as,ld,objcopy}`). Using LLVM allows whole-program IR optimization and emits TC32-compatible assembly natively via a patched ARM backend.

### Build Process

`sdk/build_firmware.py` (invoked by `make`) orchestrates the whole-module LLVM IR pipeline:

1. **Generate per-crate LLVM IR**: Cargo builds `rust/Cargo.toml` (`--lib --release -Z build-std=core --target thumbv6m-none-eabi --message-format=json`) with `RUSTFLAGS="--emit=llvm-ir"`, emitting `.ll` IR files for `core`, dependencies, and `tlsr8266_mesh`. `sdk/build_firmware.py` maps Cargo's JSON `compiler-artifact` records to their exact `.ll` modules.

2. **Link and optimize whole-program IR**: `llvm-link` merges all selected `.ll` modules into a single bitcode module (`merged.bc`), and `opt` runs the project optimization pipeline (`OPT_PASSES`, inline threshold `50`) to produce `merged_opt.bc`.

3. **Generate TC32 assembly**: A patched `llc` from our LLVM fork (`-march=thumb -mcpu=arm9 -mattr=+soft-float,-v6`) lowers `merged_opt.bc` into a single TC32-compatible assembly file (`merged.s`). The fork handles instruction renaming, section layout, and TC32-specific fixups natively — no post-processing scripts required. The forked LLVM is available at: https://github.com/retsimx/llvm-project (see [LLVM build instructions](#llvm) below).

4. **Assemble and link**: `tc32-elf-as` assembles `merged.s` into `merged.o`, while `tc32-elf-gcc` separately compiles `sdk/cstartup_8266.S` into `cstartup_8266.o`. `tc32-elf-ld` links `merged.o` and `cstartup_8266.o` using `sdk/boot.link` and `--gc-sections` into `_build/lightblemesh`, and `tc32-elf-objcopy` produces `_build/lightblemesh.bin`. No C runtime archive is linked; Rust's `compiler_builtins` supplies the helpers emitted by the backend.

5. **Flash the firmware**: The binary is flashed to the MCU via SWire (`probe-tlsr`) or BLE/UART OTA.

### Building Instructions

From the repository root, run:

```bash
make
```

The built firmware will be in the `_build` directory (`_build/lightblemesh` and `_build/lightblemesh.bin`; use `make FEATURES=rtt` for RTT logging and `make check` to run `stack-check` and `image-size-check`). `sdk/build_firmware.py` resolves `llc`, `llvm-link`, and `opt` from, in order, the `--llc`/`--llvm-link`/`--opt` flags, the `LLC`/`LLVM_LINK`/`OPT` environment variables, a `.env` file at the repository root, and finally `../llvm/build/bin/<tool>`; `LLC=/path/to/llc make` still works because the Makefile forwards these as flags. Repeated `make` runs are incremental thanks to the per-feature manifest cache, and `make bump-version` is the explicit way to bump `BUILD_VERSION`.

> **Clean builds on toolchain bumps:** Whenever `rust/rust-toolchain.toml` is bumped — especially across Cargo target-layout changes (such as Cargo 1.100+ moving build outputs from `deps/` to `build/<crate>/<hash>/out/`) — run `cd rust && cargo clean` (or `make distclean`) before rebuilding so stale `deps/` artifacts and new `build/` artifacts do not linger together in `rust/target/`. Use `make clean` to wipe `_build` alone, or `make distclean` to wipe `_build` and `rust/target/`.

## Testing

> **Status:** Work in Progress

It's also possible to cross compile the test suite and run it locally on the host to validate the code. This works by remapping the register space of the MCU to a local u8 array which is read/written instead and can be confirmed to be correct by tests.

To run the test suite, an x86 compatible machine is needed, and there is a helper script:

```bash
./rust/run_tests.sh
```

> **Coverage tool version:** `nightly-2026-11-20` (Cargo 1.100+ target layout) requires `cargo-llvm-cov >= 0.8.6`. If an older version is already installed locally, upgrade it with `cargo install cargo-llvm-cov --force`.

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

5. **Locate the binaries**: The outputs will be at `build/bin/llc`, `build/bin/llvm-link`, and `build/bin/opt`. If your build directory differs from the default, pass the paths as command-line flags or environment variables (equivalent to `--llc`/`--llvm-link`/`--opt`), or set them in a `.env` file at the repository root:

   ```bash
   make LLC=/path/to/llc LLVM_LINK=/path/to/llvm-link OPT=/path/to/opt
   ```
