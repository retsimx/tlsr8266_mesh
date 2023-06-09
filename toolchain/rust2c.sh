#!/bin/bash

set -e

AS=./toolchain/tc32/bin/tc32-elf-as

[[ -f ".env" ]] && source .env

# if JAVA_HOME not set, then set with default value
if [ "x$LLC" = "x" ]; then
	LLC=../../../llvm/build/bin/llc
fi

cd rust

cargo build --color=always --release -Z build-std=core --all-features

for i in target/thumbv6m-none-eabi/release/deps/*.ll; do
  bname="${i%.*}"

  echo "Doing ${bname}"

  python3 ../toolchain/fix_ir.py $bname.ll

  bash -c "$LLC -march=thumb -mcpu=arm9 -mattr=+soft-float,-v6 $bname.ll.ll2 && python3 ../toolchain/fix_asm.py $bname.ll.ll2.s && ../$AS -o $bname.o $bname.ll.ll2.s.tc32" &
done

wait
