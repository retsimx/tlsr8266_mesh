//! SWire address mapping with provenance.
//!
//! Only the two physical windows are mapped to wire addresses; a symbol that
//! lives elsewhere (flash, the dead zone) is a mistake, not an MMIO poke.

use probe_proto::FLASH_SIZE;

use crate::CliError;

/// Base of the two SWire-visible physical windows.
///
/// Both windows are mapped by subtracting this base: SRAM
/// `0x0080_0000..=0x0080_0fff` becomes wire `0x0000..=0x0fff`, and the
/// register window `0x0080_8000..=0x0080_bfff` becomes `0x8000..=0xbfff`.
const PHYS_BASE: u64 = 0x0080_0000;

/// Largest address that fits the CPU's 24-bit address space.
const ADDR_MASK: u64 = 0x00ff_ffff;

/// A resolved `read`/`write` address and its provenance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddrSpec {
    /// A `.symtab` symbol; `size == 0` means the size is unknown.
    Symbol { value: u32, size: u32 },
    /// A decimal number typed on the command line.
    Numeric(u64),
    /// A `0x`-prefixed number typed on the command line.
    Wire(u64),
}

/// Maps `spec` to a 16-bit SWire address plus an optional symbol size.
///
/// Only the two physical windows are mapped:
/// `0x0080_0000..=0x0080_0fff -> 0x0000..=0x0fff` and
/// `0x0080_8000..=0x0080_bfff -> 0x8000..=0xbfff`. Already-16-bit wire
/// addresses are accepted for numeric input only; a symbol that lives in
/// flash (or anywhere outside the windows) is a mistake, not an MMIO poke.
pub fn to_swire(spec: &AddrSpec) -> Result<(u16, Option<u32>), CliError> {
    let (addr, size) = match *spec {
        AddrSpec::Symbol { value, size } => (u64::from(value), Some(size)),
        AddrSpec::Numeric(addr) | AddrSpec::Wire(addr) => (addr, None),
    };

    if addr > ADDR_MASK {
        return Err(CliError::InvalidAddress {
            message: format!("0x{addr:x} is wider than the 24-bit address space"),
        });
    }

    if (PHYS_BASE..=PHYS_BASE + 0x0fff).contains(&addr)
        || (PHYS_BASE + 0x8000..=PHYS_BASE + 0xbfff).contains(&addr)
    {
        // `st_size == 0` is unknown size, so it becomes no bound at all.
        return Ok(((addr - PHYS_BASE) as u16, size.filter(|size| *size != 0)));
    }

    if matches!(spec, AddrSpec::Symbol { .. }) {
        let message = if addr < u64::from(FLASH_SIZE) {
            format!("symbol at 0x{addr:x} is in flash, not SWire-accessible")
        } else {
            format!(
                "symbol at 0x{addr:x} is outside the SWire windows (0x800000..=0x800fff, 0x808000..=0x80bfff)"
            )
        };
        return Err(CliError::InvalidAddress { message });
    }

    if (0x0000..=0x0fff).contains(&addr) || (0x8000..=0xbfff).contains(&addr) {
        return Ok((addr as u16, size));
    }

    Err(CliError::InvalidAddress {
        message: format!(
            "0x{addr:x} is not a SWire address (expected 0x0..=0xfff, 0x8000..=0xbfff, 0x800000..=0x800fff or 0x808000..=0x80bfff)"
        ),
    })
}

pub(super) fn invalid_address(spec: &str) -> CliError {
    CliError::InvalidAddress {
        message: format!("`{spec}` is not a symbol in the ELF and not a number"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physical_windows_map_to_wire_addresses() {
        let cases = [
            (0x808000_u64, 0x8000_u16),
            (0x808001, 0x8001),
            (0x800000, 0x0000),
            (0x800fff, 0x0fff),
            (0x80bfff, 0xbfff),
        ];
        for (addr, wire) in cases {
            assert_eq!(
                to_swire(&AddrSpec::Wire(addr)),
                Ok((wire, None)),
                "0x{addr:x}"
            );
        }
    }

    #[test]
    fn already_16_bit_wire_addresses_are_accepted() {
        assert_eq!(to_swire(&AddrSpec::Wire(0x0fff)), Ok((0x0fff, None)));
        assert_eq!(to_swire(&AddrSpec::Numeric(0x8000)), Ok((0x8000, None)));
    }

    #[test]
    fn dead_zones_are_rejected() {
        for addr in [0x1000_u64, 0x7fff, 0xc000, 0x80c000, 0x0100_0000] {
            let err = to_swire(&AddrSpec::Wire(addr)).expect_err("dead zone must fail");
            assert!(
                matches!(err, CliError::InvalidAddress { .. }),
                "0x{addr:x}: {err:?}"
            );
        }
    }

    #[test]
    fn non_swire_symbols_are_rejected() {
        let err = to_swire(&AddrSpec::Symbol {
            value: 0x20,
            size: 0,
        })
        .expect_err("flash symbol must fail");
        assert!(matches!(err, CliError::InvalidAddress { .. }), "{err:?}");
    }

    #[test]
    fn symbol_size_is_reported_but_zero_means_unknown() {
        assert_eq!(
            to_swire(&AddrSpec::Symbol {
                value: 0x808000,
                size: 0x24,
            }),
            Ok((0x8000, Some(0x24)))
        );
        assert_eq!(
            to_swire(&AddrSpec::Symbol {
                value: 0x808010,
                size: 0,
            }),
            Ok((0x8010, None))
        );
    }
}
