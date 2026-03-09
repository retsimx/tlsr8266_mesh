#!/bin/bash
set -e

# Check if cargo-llvm-cov is installed, install if not
if ! cargo llvm-cov --version &>/dev/null; then
    echo "Installing cargo-llvm-cov..."
    cargo install cargo-llvm-cov
fi

LCOV_OUTPUT="${LCOV_OUTPUT:-}"

if [ -n "$LCOV_OUTPUT" ]; then
    # CI mode: emit both human-readable summary and lcov data
    RUST_BACKTRACE=1 cargo llvm-cov \
        --branch \
        --target i686-unknown-linux-gnu \
        --lcov --output-path "$LCOV_OUTPUT" \
        -- --test-threads=1 "$@"
    # Print summary to stdout
    RUST_BACKTRACE=1 cargo llvm-cov report \
        --branch \
        --target i686-unknown-linux-gnu
else
    # Local mode: human-readable with missing lines
    RUST_BACKTRACE=1 cargo llvm-cov \
        --branch \
        --show-missing-lines \
        --target i686-unknown-linux-gnu \
        -- --show-output --test-threads=1 "$@"
fi
