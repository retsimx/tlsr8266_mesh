//! ELF helpers: flat flash images, symbol lookup and code-symbol ranges.
//!
//! The firmware ELF links code at `p_paddr` 0 and data at its load address, so
//! the bytes the probe programs are the concatenation of the `PT_LOAD`
//! segments at their **physical** addresses. The portable `ObjectSegment`
//! API reports `p_vaddr` and is deliberately not used here. Address mapping
//! lives in [`addr`].

mod addr;
mod symbols;

pub use addr::{to_swire, AddrSpec};
pub use symbols::{symbolize, SymbolRange};

use object::elf::PT_LOAD;
use object::read::elf::{ElfFile32, ProgramHeader};
use object::{Object, ObjectSymbol};
use probe_proto::FLASH_SIZE;

use crate::CliError;

use self::addr::invalid_address;

/// An ELF file owned by the CLI, parsed on demand.
///
/// Parsing is repeated per query: the parsed views borrow the raw bytes, so
/// storing them next to the bytes would need a self-referential struct.
pub struct ElfImage {
    bytes: Vec<u8>,
}

impl ElfImage {
    /// Parses `bytes` far enough to reject anything that is not a 32-bit
    /// little-endian ELF.
    pub fn parse(bytes: &[u8]) -> Result<Self, CliError> {
        let elf: ElfFile32<'_> = ElfFile32::parse(bytes)
            .map_err(|err| CliError::Elf(format!("cannot parse ELF: {err}")))?;
        if elf.endian() != object::Endianness::Little {
            return Err(CliError::Elf("ELF is not little-endian".to_owned()));
        }
        Ok(Self {
            bytes: bytes.to_vec(),
        })
    }

    /// Returns the packed flash image: the `PT_LOAD` segments copied to their
    /// physical addresses, with gaps zero-filled.
    pub fn flat_image(&self) -> Result<Vec<u8>, CliError> {
        let elf = self.parsed()?;
        let endian = elf.endian();

        // Validate every loadable segment before allocating the output.
        let mut segments = Vec::new();
        for ph in elf.elf_program_headers() {
            if ph.p_type(endian) != PT_LOAD {
                continue;
            }
            let filesz: u64 = ph.p_filesz(endian).into();
            if filesz == 0 {
                continue;
            }
            let paddr: u64 = ph.p_paddr(endian).into();
            let offset: u64 = ph.p_offset(endian).into();
            let seg_end = paddr.checked_add(filesz).ok_or_else(|| {
                CliError::Elf(format!("segment at 0x{paddr:x} has an overflowing length"))
            })?;
            if seg_end > u64::from(FLASH_SIZE) {
                return Err(CliError::Elf(format!(
                    "segment at 0x{paddr:x}..0x{seg_end:x} does not fit in {FLASH_SIZE} bytes of flash"
                )));
            }
            let start = usize::try_from(paddr).map_err(|_| {
                CliError::Elf(format!("address 0x{paddr:x} does not fit in a usize"))
            })?;
            // `data` is the checked accessor: it rejects an offset/length that
            // is outside the file instead of slicing blindly.
            let data = ph.data(endian, self.bytes.as_slice()).map_err(|()| {
                CliError::Elf(format!(
                    "segment at 0x{paddr:x} (offset 0x{offset:x}, {filesz} bytes) extends past end of file"
                ))
            })?;
            segments.push((start, data));
        }

        if segments.is_empty() {
            return Err(CliError::Elf("no PT_LOAD with p_filesz > 0".to_owned()));
        }

        // `p_memsz` is deliberately ignored: objcopy truncates the `.bss`
        // NOBITS tail of the last segment, and honouring `p_memsz` would
        // append several kilobytes of zeros that are not in the real image.
        let end = segments
            .iter()
            .map(|(start, data)| start + data.len())
            .max()
            .unwrap_or(0);
        let mut out = vec![0u8; end];
        // Later segments win on overlap, matching objcopy's segment order.
        for (start, data) in segments {
            out[start..start + data.len()].copy_from_slice(data);
        }
        Ok(out)
    }

    /// Looks `name` up in `.symtab`, returning `(st_value, st_size)`.
    ///
    /// `st_size == 0` means the size is unknown, never that the symbol covers
    /// zero bytes; callers must pick their own default length.
    pub fn lookup_symbol(&self, name: &str) -> Option<(u32, u32)> {
        let elf = self.parsed().ok()?;
        for symbol in elf.symbols() {
            if symbol.is_undefined() {
                continue;
            }
            if symbol.name() != Ok(name) {
                continue;
            }
            let value = u32::try_from(symbol.address()).ok()?;
            let size = u32::try_from(symbol.size()).ok()?;
            return Some((value, size));
        }
        None
    }

    /// Resolves a command-line address, keeping where it came from so that
    /// symbol values are never silently reinterpreted as MMIO addresses.
    pub fn parse_addr(&self, spec: &str) -> Result<AddrSpec, CliError> {
        if let Some(hex) = spec.strip_prefix("0x").or_else(|| spec.strip_prefix("0X")) {
            return u64::from_str_radix(hex, 16)
                .map(AddrSpec::Wire)
                .map_err(|_| invalid_address(spec));
        }
        if let Some((value, size)) = self.lookup_symbol(spec) {
            return Ok(AddrSpec::Symbol { value, size });
        }
        spec.parse::<u64>()
            .map(AddrSpec::Numeric)
            .map_err(|_| invalid_address(spec))
    }

    fn parsed(&self) -> Result<ElfFile32<'_>, CliError> {
        ElfFile32::parse(self.bytes.as_slice())
            .map_err(|err| CliError::Elf(format!("cannot parse ELF: {err}")))
    }
}

#[cfg(test)]
mod tests;

/// Fixture ELF whose first loadable segment starts with `header`.
///
/// Only the crystal-guard tests in `commands::flash` need it: they drive an ELF
/// through the same flattening `flash` uses, so the 12-byte header at `p_paddr`
/// 0 must survive into the flat image.
#[cfg(test)]
pub(crate) use self::tests::header_fixture;
