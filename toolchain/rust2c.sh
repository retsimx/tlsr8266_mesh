#!/bin/bash

set -e

AS=./toolchain/tc32/bin/tc32-elf-as
LLC=/home/lewis/Projects/llvm/build/bin/llc

cd rust

cargo build --color=always --release

for i in target/thumbv6m-none-eabi/release/deps/*.ll; do
  bname="${i%.*}"

  echo "Doing ${bname}"

  python ../toolchain/fix_ir.py $bname.ll

  bash -c "$LLC -march=thumb -mcpu=arm9 -mattr=+soft-float,-v6 $bname.ll.ll2 && python ../toolchain/fix_asm.py $bname.ll.ll2.s && ../$AS -o $bname.o $bname.ll.ll2.s.tc32" &
done

wait
