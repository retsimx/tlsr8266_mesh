//! Unit tests for the ELF helpers.
//!
//! One synthetic ELF per fixture: `fixture` for the flat image and the symbol
//! lookup, [`header_fixture`](self::header_fixture) for a real firmware header
//! in the flash segment, and [`code_symbols`](self::code_symbols) for the code
//! ranges. All hand-build the bytes the reader actually touches, so no
//! toolchain is needed.

use super::*;

mod code_symbols;

// Fixed layout shared by the synthetic ELFs built by [`fixture`] and
// [`code_symbol_fixture`].
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
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
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

/// Builds a fixture whose first 16-byte loadable payload starts with `header`.
///
/// The `flash` ELF test uses this to prove that the 12-byte firmware header at
/// `p_paddr` 0 survives ELF flattening. `fixture` keeps the original payload.
pub(crate) fn header_fixture(header: [u8; 12]) -> Vec<u8> {
    let mut seg0 = SEG0_BYTES;
    seg0[..header.len()].copy_from_slice(&header);
    fixture_with_seg0(seg0)
}

/// Builds a little-endian ELF32 with two loadable segments:
///
/// - `PT_LOAD[0]`: `p_vaddr = p_paddr = 0`, 16 bytes (flash).
/// - `PT_LOAD[1]`: `p_vaddr = 0x808f00`, `p_paddr = 0x100`, 4 bytes and a
///   larger `p_memsz` (the `.bss` tail objcopy would truncate).
///
/// `.symtab` holds `fixture_symbol`, `zero_size_symbol` and `__reset`.
fn fixture() -> Vec<u8> {
    fixture_with_seg0(SEG0_BYTES)
}

/// Fills the shared fixture layout with `seg0` as the 16-byte flash payload.
fn fixture_with_seg0(seg0: [u8; 16]) -> Vec<u8> {
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

    buf[SEG0..SEG0 + 16].copy_from_slice(&seg0);
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
