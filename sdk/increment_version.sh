#!/bin/sh
# Bumps BUILD_VERSION and restamps the crystal bit in the image header.
#
# Usage: sdk/increment_version.sh <0|1>
#   0 = 12 MHz build (the `xtal-16mhz` feature is not selected)
#   1 = 16 MHz build (the `xtal-16mhz` feature is selected)
#
# The crystal comes from the caller (the Makefile derives it from FEATURES), not
# from rust/Cargo.toml. The manifest only *declares* the feature; FEATURES
# *selects* it. Reading the declaration let a 12 MHz build stamp itself as
# 16 MHz, and the flash crystal guard then trusted that label.
set -eu

case "${1:-}" in
	0 | 1) xtal=$1 ;;
	*)
		echo "usage: $0 <0|1>   (0 = 12 MHz build, 1 = 16 MHz build)" >&2
		exit 2
		;;
esac

cd sdk
head -n1 version.in > version.in.new
mv version.in.new version.in
oldnum=$(cut -d ',' -f2 version.in)
newnum=$((oldnum + 1))
sed -i "s/.equ BUILD_VERSION,$oldnum\$/.equ BUILD_VERSION,$newnum/g" version.in

echo ".equ XTAL_16MHZ,$xtal" >> version.in

if [ "$xtal" -eq 1 ]; then
	echo "pub static BUILD_VERSION: u32 = (1 << 31) | $newnum;" > ../rust/src/version.rs
else
	echo "pub static BUILD_VERSION: u32 = $newnum;" > ../rust/src/version.rs
fi
