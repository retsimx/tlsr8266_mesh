# Firmware Development Guide

## Toolchain Versions

`rust/rust-toolchain.toml` pins the Rust toolchain to `nightly-2026-11-20` (`minimal` profile) with:
- **Components**: `rust-src` (for `-Z build-std=core`), `rustc`, `rustfmt`, `clippy`, and `llvm-tools`
- **Targets**:
  - `thumbv6m-none-eabi` for compiling the firmware to LLVM IR
  - `i686-unknown-linux-gnu` for running host tests (`./rust/run_tests.sh`) and Clippy

> **Note:** Make sure you have `rustup` installed. Running Cargo or `rustup show` inside `rust/` automatically installs the pinned `nightly-2026-11-20` toolchain, targets, and components from `rust/rust-toolchain.toml`. Additionally, `rustc`'s LLVM major version must not exceed the `retsimx/llvm-project` (`tc32` branch) fork's LLVM major version (`sdk/build_firmware.py` enforces this check).

## Building the Firmware

The firmware builds entirely with LLVM: a forked ARM backend lowers whole-program Rust IR straight to TC32 ELF objects (the core is standard ARMv4T Thumb-1 with a permuted top nibble), `ld.lld` links them, and `llvm-objcopy` produces the flash image. Telink's proprietary `tc32-elf-*` binutils are no longer used or vendored.

### Build Process

`sdk/build_firmware.py` (invoked by `make`) orchestrates the whole-module LLVM IR pipeline:

1. **Generate per-crate LLVM IR**: Cargo builds `rust/Cargo.toml` (`--lib --release -Z build-std=core --target thumbv6m-none-eabi --message-format=json`) with `RUSTFLAGS="--emit=llvm-ir"`, emitting `.ll` IR files for `core`, dependencies, and `tlsr8266_mesh`. `sdk/build_firmware.py` maps Cargo's JSON `compiler-artifact` records to their exact `.ll` modules.

2. **Link and optimize whole-program IR**: `llvm-link` merges all selected `.ll` modules into a single bitcode module (`merged.bc`), and `opt` runs the project optimization pipeline (`OPT_PASSES`, inline threshold `50`) to produce `merged_opt.bc`.

3. **Emit a TC32 object**: A patched `llc` from our LLVM fork (`-march=thumb -mcpu=arm9 -mattr=+soft-float,-v6,+tc32`) lowers `merged_opt.bc` directly to an ELF object (`merged.o`) with `--filetype=obj --function-sections --data-sections`. The `+tc32` subtarget feature applies the top-nibble permutation at instruction-encode time; the assembler text layer is standard ARM Thumb-1 and needs no post-processing. The startup code (`reset` vectors and IRQ entry) is Rust `global_asm!` in `rust/src/startup.rs`, so it is part of the same module; `RUSTFLAGS` also carries `-C target-feature=+tc32` so the per-CGU target attributes that accompany rustc's file-scope asm include the feature (otherwise `llc` would assemble those asm blocks with its own attribute-derived subtarget and skip the permutation). The five TC32-only status/IRQ instructions are written as pre-encoded `.short` values in that asm because rustc's embedded LLVM validates `global_asm!` but does not know them. The forked LLVM is available at: https://github.com/retsimx/llvm-project (see [LLVM build instructions](#llvm) below).

4. **Link and convert**: `ld.lld --tc32 --gc-sections -T ./sdk/boot.link --defsym=BUILD_VERSION=<n>` links `merged.o` into `_build/lightblemesh`, and `llvm-objcopy -O binary` produces `_build/lightblemesh.bin`. The `--tc32` flag teaches the linker to encode `bl` relocations in TC32 form; `--defsym=BUILD_VERSION` injects the single-sourced version word (`rust/src/version.rs`) into the vector table at link time. No C runtime archive is linked; Rust's `compiler_builtins` supplies the helpers emitted by the backend.

5. **Flash the firmware**: The binary is flashed to the MCU via SWire (`probe-tlsr`) or BLE/UART OTA.

### What the LLVM fork provides

The fork is a normal LLVM checkout plus a TC32 backend that is selected with the
`+tc32` subtarget feature (implied by `-mcpu=arm9`):

- **MC layer** — applies the TC32 top-nibble permutation when encoding and its
  inverse when disassembling, including NOP/alignment fill (`ARMMCCodeEmitter`,
  `ARMAsmBackend`, `ARMDisassembler`). `llc -S` therefore prints ordinary ARM
  Thumb-1, and `llvm-objdump -d --mattr=+tc32` decodes any TC32 image.
- **Five TC32-only instructions** as first-class TableGen definitions:
  `mcsr`, `mrcs`, `mssr`, `mrss`, `reti` (plus the `t`-prefixed aliases).
- **`ld.lld --tc32`** — encodes `bl` relocations in TC32 form.
- **Codegen fixes** — fences lower to a zero-instruction compiler barrier,
  acquire/release atomics stay native, `pop {…, pc}` returns, no frame pointer,
  and safe `strex`/`mrs`/`msr` handling in `TC32IRFixupPass`.

The ISA reference now lives with the backend:
`llvm/docs/TC32InstructionSet.md` in
[retsimx/llvm-project](https://github.com/retsimx/llvm-project) (`tc32` branch).

`RUSTFLAGS` carries `-C target-feature=+tc32` as well as `--emit=llvm-ir`. That
is load-bearing: `rustc` attaches `target_features` to every file-scope asm
block, and `llc` assembles those blocks with a subtarget built from the
attributes rather than `-mattr`. Without it, `compiler_builtins` helpers would
be emitted unscrambled.

### Building Instructions

From the repository root, run:

```bash
make
```

The built firmware will be in the `_build` directory (`_build/lightblemesh` and `_build/lightblemesh.bin`; use `make FEATURES=rtt` for RTT logging and `make check` to run `stack-check` and `image-size-check`). `sdk/build_firmware.py` resolves `llc`, `llvm-link`, `opt`, `ld.lld`, and `llvm-objcopy` from, in order, the `--llc`/`--llvm-link`/`--opt`/`--ld`/`--objcopy` flags, the `LLC`/`LLVM_LINK`/`OPT`/`LLD`/`LLVM_OBJCOPY` environment variables, a `.env` file at the repository root, and finally `../llvm/build/bin/<tool>`; `LLC=/path/to/llc make` still works because the Makefile forwards these as flags. Repeated `make` runs are incremental thanks to the per-feature manifest cache, and `make bump-version` is the explicit way to bump `BUILD_VERSION` (it now edits only `rust/src/version.rs`).

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

3. **Configure the build** (ARM target plus `lld`):
   ```bash
   cmake "../llvm-project/llvm" -G Ninja \
   -DLLVM_ENABLE_ASSERTIONS=ON \
   -DLLVM_ENABLE_PLUGINS=OFF \
   -DLLVM_TARGETS_TO_BUILD=ARM \
   -DLLVM_ENABLE_PROJECTS=lld \
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
   ninja -j$(nproc) llc llvm-link opt lld llvm-objcopy llvm-objdump llvm-nm
   ```

   `llvm-mc` and `FileCheck` are only needed to run the fork's two TC32 tests
   (`ninja llvm-mc FileCheck`).

5. **Locate the binaries**: The outputs will be at `build/bin/{llc,llvm-link,opt,ld.lld,llvm-objcopy,llvm-objdump,llvm-nm,llvm-mc}`. If your build directory differs from the default, pass the paths as command-line flags or environment variables (equivalent to `--llc`/`--llvm-link`/`--opt`/`--ld`/`--objcopy`), or set them in a `.env` file at the repository root:

   ```bash
   make LLC=/path/to/llc LLVM_LINK=/path/to/llvm-link OPT=/path/to/opt \
        LD=/path/to/ld.lld OBJCOPY=/path/to/llvm-objcopy
   ```

   Or, equivalently, put them in a repo-root `.env` (gitignored):

   ```bash
   LLC=../llvm/build/bin/llc
   LLVM_LINK=../llvm/build/bin/llvm-link
   OPT=../llvm/build/bin/opt
   LLD=../llvm/build/bin/ld.lld
   LLVM_OBJCOPY=../llvm/build/bin/llvm-objcopy
   LLVM_OBJDUMP=../llvm/build/bin/llvm-objdump
   LLVM_NM=../llvm/build/bin/llvm-nm
   ```

   `make check` uses `llvm-objdump`/`llvm-nm` for the static stack and image-size
   gates. Set `LLVM_OBJDUMP`/`LLVM_NM` if those tools are not on `PATH` (CI does).
   The stack gate **fails closed**: a disassembler that does not understand
   `--mattr=+tc32` would mis-decode every frame, so any such warning aborts the
   check instead of reporting a false margin.

   The driver also **probes `llc`** at startup: an older or stock `llc` accepts
   `-mattr=+tc32` with a warning and would emit an unscrambled, unbootable
   image, so the build stops with exit code 2 instead.

### Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `error: ... does not support the +tc32 subtarget feature` | `--llc`/`LLC` points at a stock or stale LLVM build. Rebuild the fork from the `tc32` branch. |
| `stack gate: ... cannot decode TC32` | `LLVM_OBJDUMP` resolves to a disassembler without `+tc32`; point it at the fork's `llvm-objdump` (or put it on `PATH`). |
| `'+tc32' is not a recognized feature ... (ignoring feature)` from cargo | Expected warning from rustc's own LLVM; the feature is forwarded to the IR and used by `llc`. |
| `undefined symbol: __sync_synchronize` | A stale `llc`/`opt` that still lowers fences to libcalls. Rebuild the fork. |
