#!/bin/bash
#sed -i 's/#\[panic_handler\]/\/\/panic_handler/g' src/sdk/common/compat.rs

# Check if cargo-llvm-cov is installed, install if not
if ! cargo llvm-cov --version &>/dev/null; then
    echo "Installing cargo-llvm-cov..."
    cargo install cargo-llvm-cov
fi

RUST_BACKTRACE=1 cargo llvm-cov --branch --show-missing-lines --target i686-unknown-linux-gnu -- --show-output --test-threads=1 $@
#RUST_BACKTRACE=1 cargo llvm-cov --branch --target i686-unknown-linux-gnu -- --show-output --test-threads=1
