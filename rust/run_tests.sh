#!/bin/bash
set -e

# Check if cargo-llvm-cov is installed, install if not
if ! cargo llvm-cov --version &>/dev/null; then
    echo "Installing cargo-llvm-cov..."
    cargo install cargo-llvm-cov
fi

if [ "${CI:-}" = "true" ]; then
    # CI mode: same as local but without --show-missing-lines
    RUST_BACKTRACE=1 cargo llvm-cov \
        --branch \
        --target i686-unknown-linux-gnu \
        -- --test-threads=1 "$@"
else
    # Local mode: human-readable with missing lines
    RUST_BACKTRACE=1 cargo llvm-cov \
        --branch \
        --show-missing-lines \
        --target i686-unknown-linux-gnu \
        -- --show-output --test-threads=1 "$@"
fi
