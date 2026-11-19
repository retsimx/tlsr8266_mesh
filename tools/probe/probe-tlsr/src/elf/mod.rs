//! ELF helpers: flat flash images and symbol lookup.
//!
//! The firmware ELF links code at `p_paddr` 0 and data at its load address, so
//! the bytes the probe programs are the concatenation of the `PT_LOAD`
//! segments at their **physical** addresses. The portable `ObjectSegment`
//! API reports `p_vaddr` and is deliberately not used here. Address mapping
//! lives in [`addr`].

mod addr;

pub use addr::{to_swire, AddrSpec};

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
mod tests {
    use super::*;

    // Fixed layout of the synthetic ELF built by [`fixture`].
    const PH0: usize = 52;
    const PH1: usize = 84;
    const SEG0: usize = 116;
    const SEG1: usize = 132;
    const SYMTAB: usize = 136;
    const STRTAB: usize = 200;
    const SHSTRTAB: usize = 248;
    const SHOFF: usize = 280;
    const FILE_SIZE: usize = 440;

    const SEG0_BYTES: [u8; 16] = [
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
        0x1f,
    ];
    const SEG1_BYTES: [u8; 4] = [0xde, 0xad, 0xbe, 0xef];

    fn put_u16(buf: &mut [u8], at: usize, value: u16) {
        buf[at..at + 2].copy_from_slice(&value.to_le_bytes());
    }

    fn put_u32(buf: &mut [u8], at: usize, value: u32) {
        buf[at..at + 4].copy_from_slice(&value.to_le_bytes());
    }

    /// Fields of a `PT_LOAD` program header written by [`put_program_header`].
    struct SegmentFields {
        offset: u32,
        vaddr: u32,
        paddr: u32,
        filesz: u32,
        memsz: u32,
    }

    fn put_program_header(buf: &mut [u8], at: usize, fields: SegmentFields) {
        put_u32(buf, at, PT_LOAD);
        put_u32(buf, at + 4, fields.offset);
        put_u32(buf, at + 8, fields.vaddr);
        put_u32(buf, at + 12, fields.paddr);
        put_u32(buf, at + 16, fields.filesz);
        put_u32(buf, at + 20, fields.memsz);
        put_u32(buf, at + 24, 0x5); // PF_R | PF_X
        put_u32(buf, at + 28, 4);
    }

    fn put_symbol(buf: &mut [u8], at: usize, name: u32, value: u32, size: u32, shndx: u16) {
        put_u32(buf, at, name);
        put_u32(buf, at + 4, value);
        put_u32(buf, at + 8, size);
        buf[at + 12] = 0x11; // STB_GLOBAL | STT_OBJECT
        buf[at + 13] = 0;
        put_u16(buf, at + 14, shndx);
    }

    /// Fields of a section header written by [`put_section_header`].
    struct SectionFields {
        name: u32,
        sh_type: u32,
        offset: u32,
        size: u32,
        link: u32,
        info: u32,
        entsize: u32,
    }

    fn put_section_header(buf: &mut [u8], at: usize, fields: SectionFields) {
        put_u32(buf, at, fields.name);
        put_u32(buf, at + 4, fields.sh_type);
        put_u32(buf, at + 8, 0);
        put_u32(buf, at + 12, 0);
        put_u32(buf, at + 16, fields.offset);
        put_u32(buf, at + 20, fields.size);
        put_u32(buf, at + 24, fields.link);
        put_u32(buf, at + 28, fields.info);
        put_u32(buf, at + 32, 4);
        put_u32(buf, at + 36, fields.entsize);
    }

    /// Builds a little-endian ELF32 with two loadable segments:
    ///
    /// - `PT_LOAD[0]`: `p_vaddr = p_paddr = 0`, 16 bytes (flash).
    /// - `PT_LOAD[1]`: `p_vaddr = 0x808f00`, `p_paddr = 0x100`, 4 bytes and a
    ///   larger `p_memsz` (the `.bss` tail objcopy would truncate).
    ///
    /// `.symtab` holds `fixture_symbol`, `zero_size_symbol` and `__reset`.
    fn fixture() -> Vec<u8> {
        let mut buf = vec![0u8; FILE_SIZE];

        buf[0..4].copy_from_slice(b"\x7fELF");
        buf[4] = 1; // ELFCLASS32
        buf[5] = 1; // ELFDATA2LSB
        buf[6] = 1; // EV_CURRENT
        put_u16(&mut buf, 16, 2); // ET_EXEC
        put_u16(&mut buf, 18, 0);
        put_u32(&mut buf, 20, 1); // e_version
        put_u32(&mut buf, 24, 0); // e_entry
        put_u32(&mut buf, 28, PH0 as u32); // e_phoff
        put_u32(&mut buf, 32, SHOFF as u32); // e_shoff
        put_u16(&mut buf, 40, 52); // e_ehsize
        put_u16(&mut buf, 42, 32); // e_phentsize
        put_u16(&mut buf, 44, 2); // e_phnum
        put_u16(&mut buf, 46, 40); // e_shentsize
        put_u16(&mut buf, 48, 4); // e_shnum
        put_u16(&mut buf, 50, 3); // e_shstrndx -> .shstrtab

        put_program_header(
            &mut buf,
            PH0,
            SegmentFields {
                offset: SEG0 as u32,
                vaddr: 0,
                paddr: 0,
                filesz: 16,
                memsz: 16,
            },
        );
        put_program_header(
            &mut buf,
            PH1,
            SegmentFields {
                offset: SEG1 as u32,
                vaddr: 0x808f00,
                paddr: 0x100,
                filesz: 4,
                memsz: 0x1000,
            },
        );

        buf[SEG0..SEG0 + 16].copy_from_slice(&SEG0_BYTES);
        buf[SEG1..SEG1 + 4].copy_from_slice(&SEG1_BYTES);

        put_symbol(&mut buf, SYMTAB + 16, 1, 0x808000, 0x24, 1);
        put_symbol(&mut buf, SYMTAB + 32, 16, 0x808010, 0, 1);
        put_symbol(&mut buf, SYMTAB + 48, 33, 0x20, 0, 1);

        let strings = b"\0fixture_symbol\0zero_size_symbol\0__reset\0";
        buf[STRTAB..STRTAB + strings.len()].copy_from_slice(strings);

        let section_names = b"\0.symtab\0.strtab\0.shstrtab\0";
        buf[SHSTRTAB..SHSTRTAB + section_names.len()].copy_from_slice(section_names);

        put_section_header(
            &mut buf,
            SHOFF + 40,
            SectionFields {
                name: 1,    // ".symtab"
                sh_type: 2, // SHT_SYMTAB
                offset: SYMTAB as u32,
                size: 64,
                link: 2, // sh_link -> .strtab
                info: 1,
                entsize: 16,
            },
        );
        put_section_header(
            &mut buf,
            SHOFF + 80,
            SectionFields {
                name: 9,    // ".strtab"
                sh_type: 3, // SHT_STRTAB
                offset: STRTAB as u32,
                size: strings.len() as u32,
                link: 0,
                info: 0,
                entsize: 0,
            },
        );
        put_section_header(
            &mut buf,
            SHOFF + 120,
            SectionFields {
                name: 17,   // ".shstrtab"
                sh_type: 3, // SHT_STRTAB
                offset: SHSTRTAB as u32,
                size: section_names.len() as u32,
                link: 0,
                info: 0,
                entsize: 0,
            },
        );

        buf
    }

    #[test]
    fn flat_image_uses_p_paddr_and_ignores_memsz() {
        let image = ElfImage::parse(&fixture()).expect("fixture parses");
        let flat = image.flat_image().expect("fixture has PT_LOAD segments");

        assert_eq!(flat.len(), 0x104, "image ends at p_paddr + p_filesz");
        assert_eq!(&flat[..16], &SEG0_BYTES);
        assert_eq!(&flat[16..0x100], &[0u8; 0xf0], "gap is zero-filled");
        assert_eq!(&flat[0x100..], &SEG1_BYTES);
    }

    #[test]
    fn symtab_lookup_returns_value_and_size() {
        let image = ElfImage::parse(&fixture()).expect("fixture parses");

        assert_eq!(
            image.lookup_symbol("fixture_symbol"),
            Some((0x808000, 0x24))
        );
        assert_eq!(image.lookup_symbol("zero_size_symbol"), Some((0x808010, 0)));
        assert_eq!(image.lookup_symbol("__reset"), Some((0x20, 0)));
        assert_eq!(image.lookup_symbol("not_a_symbol"), None);
    }

    #[test]
    fn segment_past_end_of_file_is_an_elf_error() {
        let mut bytes = fixture();
        put_u32(&mut bytes, PH1 + 4, 0x10000); // p_offset, far past EOF

        let image = ElfImage::parse(&bytes).expect("program header table parses");
        let err = image.flat_image().expect_err("truncated segment must fail");
        assert!(matches!(err, CliError::Elf(_)), "{err:?}");
    }

    #[test]
    fn image_without_load_segments_is_an_elf_error() {
        let mut bytes = fixture();
        put_u32(&mut bytes, PH0, 0); // PT_NULL
        put_u32(&mut bytes, PH1, 0);

        let image = ElfImage::parse(&bytes).expect("fixture parses");
        let err = image.flat_image().expect_err("no PT_LOAD must fail");
        assert!(matches!(err, CliError::Elf(_)), "{err:?}");
    }

    #[test]
    fn garbage_is_not_an_elf() {
        assert!(matches!(
            ElfImage::parse(b"not an elf"),
            Err(CliError::Elf(_))
        ));
    }

    #[test]
    fn parse_addr_keeps_provenance() {
        let image = ElfImage::parse(&fixture()).expect("fixture parses");

        assert_eq!(
            image.parse_addr("fixture_symbol"),
            Ok(AddrSpec::Symbol {
                value: 0x808000,
                size: 0x24,
            })
        );
        assert_eq!(image.parse_addr("0x808001"), Ok(AddrSpec::Wire(0x808001)));
        assert_eq!(image.parse_addr("0x800000"), Ok(AddrSpec::Wire(0x800000)));
        assert_eq!(image.parse_addr("32768"), Ok(AddrSpec::Numeric(32768)));
        assert!(matches!(
            image.parse_addr("nope"),
            Err(CliError::InvalidAddress { .. })
        ));
        assert!(matches!(
            image.parse_addr("0xzz"),
            Err(CliError::InvalidAddress { .. })
        ));
    }

    #[test]
    fn flat_image_matches_the_objcopy_binary_when_available() {
        let build = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../_build");
        let (Ok(elf), Ok(bin)) = (
            std::fs::read(format!("{build}/lightblemesh")),
            std::fs::read(format!("{build}/lightblemesh.bin")),
        ) else {
            eprintln!("skipping objcopy identity check: no _build/lightblemesh(.bin) here");
            return;
        };

        let image = ElfImage::parse(&elf).expect("build ELF parses");
        assert_eq!(
            image.flat_image().expect("build ELF has PT_LOAD segments"),
            bin,
            "flat image differs from _build/lightblemesh.bin"
        );
    }
}
