#!/bin/bash

set -e

CXX=/opt/llvm10/bin/clang
CBE=/home/lewis/Projects/llvm-cbe/build/tools/llvm-cbe/llvm-cbe
CC=./toolchain/tc32/bin/tc32-elf-gcc
LD=./toolchain/tc32/bin/tc32-elf-ld
CP=./toolchain/tc32/bin/tc32-elf-objcopy

CCFLAGS="-O2 -fshort-wchar -fms-extensions -finline-small-functions -fpack-struct -fshort-enums -Wall -std=gnu99 -ffunction-sections -fdata-sections -w"

cd rust
rm -Rf target/i686-unknown-linux-gnu/release/deps
cargo build --color=always --release

for i in target/i686-unknown-linux-gnu/release/deps/*.ll; do
  bname="${i%.*}"

  $CBE $i

  python ../toolchain/fix_ramcode.py $bname.cbe.c
  ../$CC -c $CCFLAGS -o $bname.o $bname.cbe.c.rc.c
done
