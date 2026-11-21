#!/bin/sh
# Bumps BUILD_VERSION in rust/src/version.rs.
#
# Usage: sdk/increment_version.sh   (takes no arguments)
#
# rust/src/version.rs is the single source of truth for the image header version
# word: rust/src/startup.rs interpolates the constant straight into the vector
# table, so nothing else has to be regenerated.
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

cd "$(dirname "$0")/.."
version_rs=rust/src/version.rs

oldnum=$(sed -n 's/^pub const BUILD_VERSION: u32 = \([0-9][0-9]*\);$/\1/p' "$version_rs")
if [ -z "$oldnum" ]; then
	echo "error: no 'pub const BUILD_VERSION: u32 = <N>;' line in $version_rs" >&2
	exit 1
fi

newnum=$((oldnum + 1))
sed -i "s/^pub const BUILD_VERSION: u32 = $oldnum;\$/pub const BUILD_VERSION: u32 = $newnum;/" "$version_rs"

echo "BUILD_VERSION: $oldnum -> $newnum"
