//! `code_symbols` and `symbolize` over a synthetic ELF.
//!
//! The fixture places one symbol per documented rule: the three code sections,
//! size-less mapping/linker symbols that must be dropped, two symbols sharing a
//! start, an odd (Thumb) `st_value`, and the non-code sections `code_symbols`
//! must skip.

use super::*;
use crate::elf::{symbolize, ElfImage, SymbolRange};

/// Appends NUL-terminated `names` to a string table, returning their
/// offsets in order. Offset 0 is the mandatory empty string.
fn string_table(names: &[&str]) -> (Vec<u8>, Vec<u32>) {
    let mut table = vec![0u8];
    let mut offsets = Vec::with_capacity(names.len());
    for name in names {
        offsets.push(table.len() as u32);
        table.extend_from_slice(name.as_bytes());
        table.push(0);
    }
    (table, offsets)
}

/// Number of `.symtab` entries in [`code_symbol_fixture`]: the null entry
/// plus 13 symbols.
const SYM_COUNT: usize = 14;
const SYM_SECTION_COUNT: usize = 9;

/// Builds an ELF32 with `.vectors`, `.text`, `.ram_code`, `.bss` and
/// `.data` sections plus symbols that exercise every `code_symbols` rule:
///
/// - `vector_reset` (`.vectors`, 0x00, size 0x10);
/// - `next_after_zero` (`.text`, 0x60, size 0x10);
/// - `long_start` / `short_start` (`.text`, both 0x80, sizes 0x20 / 0x8);
/// - `ram_fn` (`.ram_code`, 0x808100, size 0x40);
/// - `thumb_fn` (`.text`, 0xa1, size 0x10): an odd `st_value` must become 0xa0;
/// - `$t` (`.text`, 0x62, size 0): a mapping symbol inside a real function;
/// - `ZERO` (`.text`, 0x8081c4, size 0) and an unnamed label at 0x8081c0:
///   size-less/unnamed symbols must be dropped;
/// - `data_var` (`.data`) and `bss_var` (`.bss`), which are not code;
/// - a non-UTF-8 name and an undefined symbol, which are skipped.
fn code_symbol_fixture() -> Vec<u8> {
    let (mut strtab, names) = string_table(&[
        "vector_reset",
        "next_after_zero",
        "long_start",
        "short_start",
        "ram_fn",
        "data_var",
        "bss_var",
        "thumb_fn",
        "$t",
        "ZERO",
        "undef_name",
    ]);
    let invalid_name = strtab.len() as u32;
    strtab.extend_from_slice(&[0xff, 0xfe, 0]);

    let (shstrtab, sections) = string_table(&[
        ".symtab",
        ".strtab",
        ".shstrtab",
        ".text",
        ".ram_code",
        ".vectors",
        ".bss",
        ".data",
    ]);

    let symtab_size = SYM_COUNT * 16;
    let strtab_off = SYMTAB + symtab_size;
    let shstrtab_off = strtab_off + strtab.len();
    let shoff = (shstrtab_off + shstrtab.len() + 3) & !3;
    let mut buf = vec![0u8; shoff + SYM_SECTION_COUNT * 40];

    buf[0..4].copy_from_slice(b"\x7fELF");
    buf[4] = 1; // ELFCLASS32
    buf[5] = 1; // ELFDATA2LSB
    buf[6] = 1; // EV_CURRENT
    put_u16(&mut buf, 16, 2); // ET_EXEC
    put_u32(&mut buf, 28, PH0 as u32); // e_phoff
    put_u32(&mut buf, 32, shoff as u32); // e_shoff
    put_u16(&mut buf, 40, 52); // e_ehsize
    put_u16(&mut buf, 42, 32); // e_phentsize
    put_u16(&mut buf, 44, 2); // e_phnum
    put_u16(&mut buf, 46, 40); // e_shentsize
    put_u16(&mut buf, 48, SYM_SECTION_COUNT as u16);
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

    // Section indexes: 4 `.text`, 5 `.ram_code`, 6 `.vectors`,
    // 7 `.bss`, 8 `.data`.
    put_symbol(&mut buf, SYMTAB + 16, names[0], 0x0000, 0x10, 6);
    put_symbol(&mut buf, SYMTAB + 32, names[1], 0x0060, 0x10, 4);
    put_symbol(&mut buf, SYMTAB + 48, names[2], 0x0080, 0x20, 4);
    put_symbol(&mut buf, SYMTAB + 64, names[3], 0x0080, 0x8, 4);
    put_symbol(&mut buf, SYMTAB + 80, names[4], 0x808100, 0x40, 5);
    put_symbol(&mut buf, SYMTAB + 96, names[5], 0x808200, 4, 8);
    put_symbol(&mut buf, SYMTAB + 112, names[6], 0x808400, 4, 7);
    put_symbol(&mut buf, SYMTAB + 128, invalid_name, 0x808180, 4, 4);
    put_symbol(&mut buf, SYMTAB + 144, names[7], 0x00a1, 0x10, 4);
    put_symbol(&mut buf, SYMTAB + 160, names[8], 0x0062, 0, 4);
    put_symbol(&mut buf, SYMTAB + 176, names[9], 0x8081c4, 0, 4);
    put_symbol(&mut buf, SYMTAB + 192, 0, 0x8081c0, 4, 4);
    put_symbol(&mut buf, SYMTAB + 208, names[10], 0x8081f0, 4, 0);

    buf[strtab_off..strtab_off + strtab.len()].copy_from_slice(&strtab);
    buf[shstrtab_off..shstrtab_off + shstrtab.len()].copy_from_slice(&shstrtab);

    put_section_header(
        &mut buf,
        shoff + 40,
        SectionFields {
            name: sections[0], // ".symtab"
            sh_type: 2,        // SHT_SYMTAB
            offset: SYMTAB as u32,
            size: symtab_size as u32,
            link: 2, // sh_link -> .strtab
            info: 1,
            entsize: 16,
        },
    );
    put_section_header(
        &mut buf,
        shoff + 80,
        SectionFields {
            name: sections[1], // ".strtab"
            sh_type: 3,        // SHT_STRTAB
            offset: strtab_off as u32,
            size: strtab.len() as u32,
            link: 0,
            info: 0,
            entsize: 0,
        },
    );
    put_section_header(
        &mut buf,
        shoff + 120,
        SectionFields {
            name: sections[2], // ".shstrtab"
            sh_type: 3,        // SHT_STRTAB
            offset: shstrtab_off as u32,
            size: shstrtab.len() as u32,
            link: 0,
            info: 0,
            entsize: 0,
        },
    );
    put_section_header(
        &mut buf,
        shoff + 160,
        SectionFields {
            name: sections[3], // ".text"
            sh_type: 1,        // SHT_PROGBITS
            offset: SEG0 as u32,
            size: 16,
            link: 0,
            info: 0,
            entsize: 0,
        },
    );
    put_section_header(
        &mut buf,
        shoff + 200,
        SectionFields {
            name: sections[4], // ".ram_code"
            sh_type: 1,        // SHT_PROGBITS
            offset: SEG1 as u32,
            size: 4,
            link: 0,
            info: 0,
            entsize: 0,
        },
    );
    put_section_header(
        &mut buf,
        shoff + 240,
        SectionFields {
            name: sections[5], // ".vectors"
            sh_type: 1,        // SHT_PROGBITS
            offset: SEG0 as u32,
            size: 16,
            link: 0,
            info: 0,
            entsize: 0,
        },
    );
    put_section_header(
        &mut buf,
        shoff + 280,
        SectionFields {
            name: sections[6], // ".bss"
            sh_type: 8,        // SHT_NOBITS
            offset: 0,
            size: 0x40,
            link: 0,
            info: 0,
            entsize: 0,
        },
    );
    put_section_header(
        &mut buf,
        shoff + 320,
        SectionFields {
            name: sections[7], // ".data"
            sh_type: 1,        // SHT_PROGBITS
            offset: SEG1 as u32,
            size: 4,
            link: 0,
            info: 0,
            entsize: 0,
        },
    );

    buf
}

#[test]
fn code_symbols_include_every_code_section_sorted_by_start() {
    let image = ElfImage::parse(&code_symbol_fixture()).expect("fixture parses");
    let ranges = image.code_symbols().expect("symbols parse");

    let names: Vec<&str> = ranges.iter().map(|range| range.name.as_str()).collect();
    assert_eq!(
        names,
        [
            "vector_reset",
            "next_after_zero",
            "long_start",
            "thumb_fn",
            "ram_fn",
        ]
    );
    let starts: Vec<u32> = ranges.iter().map(|range| range.start).collect();
    assert!(
        starts.windows(2).all(|pair| pair[0] < pair[1]),
        "not sorted: {starts:?}"
    );
}

#[test]
fn data_and_bss_symbols_are_not_code() {
    let image = ElfImage::parse(&code_symbol_fixture()).expect("fixture parses");
    let ranges = image.code_symbols().expect("symbols parse");

    for excluded in ["data_var", "bss_var"] {
        assert!(
            !ranges.iter().any(|range| range.name == excluded),
            "{excluded} must be excluded"
        );
    }
}

#[test]
fn zero_size_symbols_are_excluded() {
    let image = ElfImage::parse(&code_symbol_fixture()).expect("fixture parses");
    let ranges = image.code_symbols().expect("symbols parse");

    // `$t` and `ZERO` are size-less mapping/linker symbols: they carry no
    // extent, so keeping them would only shadow real code.
    for excluded in ["$t", "ZERO"] {
        assert!(
            !ranges.iter().any(|range| range.name == excluded),
            "{excluded} must be excluded"
        );
    }
    assert!(
        ranges.iter().all(|range| range.end > range.start),
        "every kept range covers at least one byte: {ranges:?}"
    );
}

#[test]
fn an_unnamed_symbol_does_not_shadow_anything() {
    let image = ElfImage::parse(&code_symbol_fixture()).expect("fixture parses");
    let ranges = image.code_symbols().expect("symbols parse");

    // The fixture has an unnamed 4 B `.text` symbol at 0x8081c0 and `ZERO`
    // right after it; neither may claim an address.
    assert_eq!(symbolize(&ranges, 0x8081c0), None);
    assert_eq!(symbolize(&ranges, 0x8081c4), None);
}

#[test]
fn a_mapping_symbol_does_not_shadow_the_enclosing_function() {
    let image = ElfImage::parse(&code_symbol_fixture()).expect("fixture parses");
    let ranges = image.code_symbols().expect("symbols parse");

    // `$t` sits at 0x62 inside `next_after_zero` (0x60..0x70).
    assert_eq!(
        symbolize(&ranges, 0x62),
        Some("next_after_zero+0x2".to_owned())
    );
}

#[test]
fn an_odd_symbol_value_starts_at_its_even_address() {
    let image = ElfImage::parse(&code_symbol_fixture()).expect("fixture parses");
    let ranges = image.code_symbols().expect("symbols parse");

    // `thumb_fn` has `st_value = 0xa1`; the Thumb bit is not part of the range.
    let thumb = ranges
        .iter()
        .find(|range| range.name == "thumb_fn")
        .expect("thumb_fn is a sized code symbol");
    assert_eq!((thumb.start, thumb.end), (0xa0, 0xb0));
    assert_eq!(symbolize(&ranges, 0xa0), Some("thumb_fn".to_owned()));
    assert_eq!(symbolize(&ranges, 0xa1), Some("thumb_fn+0x1".to_owned()));
}

#[test]
fn symbols_sharing_a_start_collapse_to_the_longest() {
    let image = ElfImage::parse(&code_symbol_fixture()).expect("fixture parses");
    let ranges = image.code_symbols().expect("symbols parse");

    let shared: Vec<&SymbolRange> = ranges.iter().filter(|range| range.start == 0x80).collect();
    assert_eq!(shared.len(), 1, "one range per start");
    assert_eq!(shared[0].name, "long_start");
    assert_eq!(shared[0].end, 0xa0);
}

#[test]
fn symbolize_names_the_start_and_offsets_inside_the_range() {
    let image = ElfImage::parse(&code_symbol_fixture()).expect("fixture parses");
    let ranges = image.code_symbols().expect("symbols parse");

    assert_eq!(symbolize(&ranges, 0x00), Some("vector_reset".to_owned()));
    assert_eq!(symbolize(&ranges, 0x60), Some("next_after_zero".to_owned()));
    assert_eq!(
        symbolize(&ranges, 0x64),
        Some("next_after_zero+0x4".to_owned())
    );
    assert_eq!(symbolize(&ranges, 0x808104), Some("ram_fn+0x4".to_owned()));
}

#[test]
fn symbolize_returns_none_outside_every_range() {
    let image = ElfImage::parse(&code_symbol_fixture()).expect("fixture parses");
    let ranges = image.code_symbols().expect("symbols parse");

    for addr in [0x10, 0x20, 0x160, 0x808140, 0x8081c0, u32::MAX] {
        assert_eq!(symbolize(&ranges, addr), None, "0x{addr:x}");
    }
    assert_eq!(symbolize(&[], 0x20), None, "empty table");
}
