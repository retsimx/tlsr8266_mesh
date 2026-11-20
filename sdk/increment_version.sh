#!/bin/sh
# Bumps BUILD_VERSION and regenerates rust/src/version.rs.
#
# Usage: sdk/increment_version.sh   (takes no arguments)
#
# The image header is crystal-free: the firmware detects the external crystal at
# boot, so there is no build-time crystal bit to stamp and bit 31 of the version
# word is always clear. Stray arguments are rejected so a stale caller fails
# loudly instead of silently mis-stamping the header.
set -eu

if [ "$#" -ne 0 ]; then
	echo "usage: $0   (takes no arguments)" >&2
	exit 2
fi

cd sdk
head -n1 version.in > version.in.new
mv version.in.new version.in
oldnum=$(cut -d ',' -f2 version.in)
newnum=$((oldnum + 1))
sed -i "s/.equ BUILD_VERSION,$oldnum\$/.equ BUILD_VERSION,$newnum/g" version.in

echo "pub static BUILD_VERSION: u32 = $newnum;" > ../rust/src/version.rs
