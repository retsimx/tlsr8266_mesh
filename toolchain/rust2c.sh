#!/bin/bash

set -e

CXX=/opt/llvm10/bin/clang
CBE=/home/lewis/Projects/llvm-cbe/build/tools/llvm-cbe/llvm-cbe
CC=./toolchain/tc32/bin/tc32-elf-gcc
LD=./toolchain/tc32/bin/tc32-elf-ld
CP=./toolchain/tc32/bin/tc32-elf-objcopy

CCFLAGS="-O2 -fshort-wchar -fms-extensions -finline-small-functions -fpack-struct -fshort-enums -Wall -std=gnu99 -ffunction-sections -fdata-sections -w"

cd rust
rm -Rf target/i686-unknown-linux-gnu/release/deps/*.c
#find ./ -name "" -exec rm -rf {} \;

cargo build --color=always --release

for i in target/i686-unknown-linux-gnu/release/deps/*.ll; do
  bname="${i%.*}"

  sed -e 's/freeze i128/mul i128 1, /g' -e 's/freeze i64/mul i64 1, /g' $bname.ll > $bname.ll.parsed

  $CBE $bname.ll.parsed

  python ../toolchain/fix_c.py $bname.ll.parsed.cbe.c
  ../$CC -c $CCFLAGS -o $bname.o $bname.ll.parsed.cbe.c.rc.c
done
