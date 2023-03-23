#!/bin/bash

set -e

CXX=/opt/llvm10/bin/clang
CBE=/home/lewis/Projects/llvm-cbe/build/tools/llvm-cbe/llvm-cbe
CC=./toolchain/tc32/bin/tc32-elf-gcc
LD=./toolchain/tc32/bin/tc32-elf-ld
CP=./toolchain/tc32/bin/tc32-elf-objcopy
AS=./toolchain/tc32/bin/tc32-elf-as
LLC=/home/lewis/Projects/llvm/build/bin/llc

CCFLAGS="-O2 -fshort-wchar -fms-extensions -finline-small-functions -fpack-struct -fshort-enums -Wall -std=gnu99 -ffunction-sections -fdata-sections -w"

cd rust
#find ./ -name "" -exec rm -rf {} \;

cargo build --color=always --release

for i in target/thumbv6m-none-eabi/release/deps/*.ll; do
  bname="${i%.*}"

  echo "Doing ${bname}"

  python ../toolchain/fix_ir.py $bname.ll

  bash -c "$LLC -march=thumb -mcpu=arm9 -mattr=+soft-float,-v6 $bname.ll.ll2 && python ../toolchain/fix_asm.py $bname.ll.ll2.s && ../$AS -o $bname.o $bname.ll.ll2.s.tc32" &
done

wait