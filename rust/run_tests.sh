#!/bin/bash
#sed -i 's/#\[panic_handler\]/\/\/panic_handler/g' src/sdk/common/compat.rs

cargo test --target i686-unknown-linux-gnu
