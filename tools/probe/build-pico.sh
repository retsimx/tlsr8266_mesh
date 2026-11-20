#!/bin/sh
# Build the RP2040 probe firmware and convert it to a .uf2 you drag onto the
# Pico in BOOTSEL mode. cd to our own directory so cargo resolves the probe
# workspace (and its target/) no matter where the user invokes this from.
set -eu
cd "$(dirname "$0")"

ELF=target/thumbv6m-none-eabi/release/probe-pico
UF2="$ELF.uf2"

# --target-dir pins the artifact to the path we convert below: an exported
# CARGO_TARGET_DIR (or a future .cargo/config.toml target-dir) would otherwise
# send the fresh ELF elsewhere and this script would convert a stale one.
cargo build --release --manifest-path probe-pico/Cargo.toml --target thumbv6m-none-eabi --target-dir target

# picotool is the upstream RP2040 tool; elf2uf2-rs is the lighter fallback.
if command -v picotool >/dev/null 2>&1; then
    picotool uf2 convert -t elf "$ELF" "$UF2"
elif command -v elf2uf2-rs >/dev/null 2>&1; then
    elf2uf2-rs "$ELF" "$UF2"
else
    echo "build-pico.sh: no UF2 converter on PATH (need picotool or elf2uf2-rs)" >&2
    # --no-default-features matters on Linux: the default "deploy" feature pulls
    # libudev-sys, which we do not need because the .uf2 is copied by hand.
    echo "  install: cargo install elf2uf2-rs --locked --no-default-features" >&2
    exit 2
fi

echo "UF2: $PWD/$UF2"
echo "Flash: hold BOOTSEL, plug the Pico in, then copy the .uf2 onto the RPI-RP2 volume."
