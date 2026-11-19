//! Fixtures for the `inspect` tests: a synthetic ELF and scripted replies.
//!
//! The synthetic ELF is the same hand-built shape `elf/mod.rs` uses: a valid
//! ELF32 header, one `.text` symbol and one `.data` symbol, with only the
//! sections `code_symbols`/`lookup_symbol` actually read.

use std::os::unix::fs::PermissionsExt;
use std::path::{Path, PathBuf};

use probe_proto::{CpuMode, Request, Response};

use super::super::*;
use crate::commands::diag::rtt::{HEARTBEAT_SYMBOL, RTT_SYMBOL};
use crate::commands::test_support::*;
use crate::commands::WATCHDOG_CTRL_ADDR;
use crate::transport::MockTransport;

/// Offsets in the synthetic ELF built by [`elf_with_symbols`].
const SYMTAB: usize = 52;
const SECTION_COUNT: usize = 6;

fn put_u16(buf: &mut [u8], at: usize, value: u16) {
    buf[at..at + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(buf: &mut [u8], at: usize, value: u32) {
    buf[at..at + 4].copy_from_slice(&value.to_le_bytes());
}

/// Appends NUL-terminated `names` to a string table, returning their offsets.
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

fn put_symbol(buf: &mut [u8], at: usize, name: u32, value: u32, size: u32, shndx: u16) {
    put_u32(buf, at, name);
    put_u32(buf, at + 4, value);
    put_u32(buf, at + 8, size);
    buf[at + 12] = 0x11; // STB_GLOBAL | STT_OBJECT
    put_u16(buf, at + 14, shndx);
}

#[allow(clippy::too_many_arguments)]
fn put_section_header(
    buf: &mut [u8],
    at: usize,
    name: u32,
    sh_type: u32,
    offset: u32,
    size: u32,
    link: u32,
    info: u32,
    entsize: u32,
) {
    put_u32(buf, at, name);
    put_u32(buf, at + 4, sh_type);
    put_u32(buf, at + 16, offset);
    put_u32(buf, at + 20, size);
    put_u32(buf, at + 24, link);
    put_u32(buf, at + 28, info);
    put_u32(buf, at + 32, 4);
    put_u32(buf, at + 36, entsize);
}

/// One `.symtab` entry: name, value, size and the section it belongs to
/// (4 `.text`, 5 `.data`).
type Symbol<'a> = (&'a str, u32, u32, u16);

/// Builds a minimal ELF32 with exactly `symbols` in `.symtab`.
///
/// `code_symbols` reads `.text`, `lookup_symbol` reads any section; no program
/// headers are needed for either.
fn elf_with_symbols(symbols: &[Symbol<'_>]) -> Vec<u8> {
    let names: Vec<&str> = symbols.iter().map(|symbol| symbol.0).collect();
    let (strtab, offsets) = string_table(&names);
    let (shstrtab, sections) = string_table(&[".symtab", ".strtab", ".shstrtab", ".text", ".data"]);
    let sym_count = symbols.len() + 1;
    let strtab_off = SYMTAB + sym_count * 16;
    let shstrtab_off = strtab_off + strtab.len();
    let shoff = (shstrtab_off + shstrtab.len() + 3) & !3;
    let mut buf = vec![0u8; shoff + SECTION_COUNT * 40];

    buf[0..4].copy_from_slice(b"\x7fELF");
    buf[4] = 1; // ELFCLASS32
    buf[5] = 1; // ELFDATA2LSB
    buf[6] = 1; // EV_CURRENT
    put_u16(&mut buf, 16, 2); // ET_EXEC
    put_u32(&mut buf, 32, shoff as u32); // e_shoff
    put_u16(&mut buf, 40, 52); // e_ehsize
    put_u16(&mut buf, 46, 40); // e_shentsize
    put_u16(&mut buf, 48, SECTION_COUNT as u16);
    put_u16(&mut buf, 50, 3); // e_shstrndx -> .shstrtab

    // Section indexes: 4 `.text`, 5 `.data`.
    for (index, symbol) in symbols.iter().enumerate() {
        put_symbol(
            &mut buf,
            SYMTAB + (index + 1) * 16,
            offsets[index],
            symbol.1,
            symbol.2,
            symbol.3,
        );
    }

    buf[strtab_off..strtab_off + strtab.len()].copy_from_slice(&strtab);
    buf[shstrtab_off..shstrtab_off + shstrtab.len()].copy_from_slice(&shstrtab);

    let sections_at = |index: usize| shoff + index * 40;
    put_section_header(
        &mut buf,
        sections_at(1),
        sections[0],
        2, // SHT_SYMTAB
        SYMTAB as u32,
        (sym_count * 16) as u32,
        2,
        1,
        16,
    );
    put_section_header(
        &mut buf,
        sections_at(2),
        sections[1],
        3, // SHT_STRTAB
        strtab_off as u32,
        strtab.len() as u32,
        0,
        0,
        0,
    );
    put_section_header(
        &mut buf,
        sections_at(3),
        sections[2],
        3, // SHT_STRTAB
        shstrtab_off as u32,
        shstrtab.len() as u32,
        0,
        0,
        0,
    );
    put_section_header(&mut buf, sections_at(4), sections[3], 1, 0, 0x20, 0, 0, 0);
    put_section_header(&mut buf, sections_at(5), sections[4], 1, 0, 0x10, 0, 0, 0);

    buf
}

/// Builds a minimal ELF32 with one `.text` symbol (`sym` at `0x100`, 32 B) and
/// one `.data` symbol named `irq.0` at `irq.1` with size `irq.2`.
pub(super) fn elf_fixture(irq: (&str, u32, u32)) -> Vec<u8> {
    elf_with_symbols(&[("sym", 0x100, 0x20, 4), (irq.0, irq.1, irq.2, 5)])
}

pub(super) fn image(irq: (&str, u32, u32)) -> ElfImage {
    ElfImage::parse(&elf_fixture(irq)).expect("synthetic ELF parses")
}

/// An ELF whose only `.text` symbol is `(name, value, size)`.
pub(super) fn text_image(symbol: (&str, u32, u32)) -> ElfImage {
    ElfImage::parse(&elf_with_symbols(&[(symbol.0, symbol.1, symbol.2, 4)]))
        .expect("synthetic ELF parses")
}

pub(super) fn no_irq_image() -> ElfImage {
    image(("irq_spare", 0x0080_b000, 0x700))
}

/// Wire address the fixture places `_TLSR_RTT` at.
pub(super) const RTT_WIRE: u16 = 0xb600;

/// Wire address the fixture places `HEARTBEAT` at.
pub(super) const HEARTBEAT_WIRE: u16 = 0xb71c;

/// Builds an ELF carrying only the RTT symbols named in `keep`, all in `.data`
/// inside the SWire SRAM window.
fn rtt_elf(keep: &[&str]) -> Vec<u8> {
    let symbols: Vec<Symbol<'_>> = [
        (RTT_SYMBOL, u32::from(RTT_WIRE), 272),
        (HEARTBEAT_SYMBOL, u32::from(HEARTBEAT_WIRE), 4),
    ]
    .into_iter()
    .filter(|(name, ..)| keep.contains(name))
    // `to_swire` maps the physical SRAM window down to the wire address.
    .map(|(name, wire, size)| (name, 0x0080_0000 + wire, size, 5))
    .collect();
    elf_with_symbols(&symbols)
}

/// An ELF with both symbols `rtt` requires.
pub(super) fn rtt_image() -> ElfImage {
    ElfImage::parse(&rtt_elf(&[RTT_SYMBOL, HEARTBEAT_SYMBOL])).expect("synthetic ELF parses")
}

/// The same image with `omit` left out of `.symtab`.
pub(super) fn rtt_image_without(omit: &str) -> ElfImage {
    let keep: Vec<&str> = [RTT_SYMBOL, HEARTBEAT_SYMBOL]
        .into_iter()
        .filter(|name| *name != omit)
        .collect();
    ElfImage::parse(&rtt_elf(&keep)).expect("synthetic ELF parses")
}

/// `r0..r12` decoded as `0..12`, with `sp`, `lr` and `pc` overridden.
pub(super) fn regs(sp: u32, lr: u32, pc: u32) -> [u32; 16] {
    let mut regs = [0u32; 16];
    for (index, slot) in regs.iter_mut().take(13).enumerate() {
        *slot = index as u32;
    }
    regs[13] = sp;
    regs[14] = lr;
    regs[15] = pc;
    regs
}

pub(super) fn shadow() -> [u32; 14] {
    let mut shadow = [0u32; 14];
    for (index, slot) in shadow.iter_mut().enumerate() {
        *slot = 0x10 + index as u32;
    }
    shadow
}

pub(super) fn regs_data(regs: [u32; 16], psr: u32, shadow: [u32; 14]) -> Response {
    let mut bytes = vec![0u8; usize::from(REGS_LEN)];
    for (index, value) in regs.iter().enumerate() {
        put_u32(&mut bytes, index * 4, *value);
    }
    put_u32(&mut bytes, PSR_OFFSET, psr);
    for (index, value) in shadow.iter().enumerate() {
        put_u32(&mut bytes, SHADOW_OFFSET + index * 4, *value);
    }
    data(bytes)
}

pub(super) fn words_data(words: &[u32]) -> Response {
    let bytes: Vec<u8> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
    data(bytes)
}

/// The four replies every inspect run needs before the register block:
/// `SwsActivate`, `CpuCtrl::Stall`, the watchdog read and its write-back.
pub(super) fn stall_prefix() -> [Response; 4] {
    [Response::Ack, Response::Ack, data([0xb5]), Response::Ack]
}

pub(super) fn assert_stall_requests(mock: &MockTransport) {
    assert_eq!(
        &mock.requests[..4],
        [
            sws_activate_req(),
            Request::CpuCtrl {
                mode: CpuMode::Stall,
            },
            read_mem_req(WATCHDOG_CTRL_ADDR, 1),
            write_mem_req(WATCHDOG_CTRL_ADDR, &[0x35]),
        ]
    );
}

/// The requests one `inspect` capture issues; it reactivates SWire and stalls
/// the already-stopped core itself.
///
/// `rtt` and `trap` both end a fault/hit with this sequence, so it lives here
/// once instead of drifting in two test modules.
pub(super) fn inspect_requests() -> Vec<Request> {
    vec![
        sws_activate_req(),
        Request::CpuCtrl {
            mode: CpuMode::Stall,
        },
        read_mem_req(WATCHDOG_CTRL_ADDR, 1),
        write_mem_req(WATCHDOG_CTRL_ADDR, &[0x35]),
        Request::ReadRegs,
        read_mem_req(0xbfc0, 64),
        Request::ReadAnalog {
            addr: RESET_SENTINEL_ADDR,
        },
        Request::ReadAnalog {
            addr: BREADCRUMB_ADDR,
        },
        read_flash_req(PANIC_SLOT_ADDR, 3),
    ]
}

/// Analog replies and the "no panic record" header.
pub(super) fn tail(sentinel: u8, breadcrumb: u8) -> [Response; 3] {
    [
        Response::Byte(sentinel),
        Response::Byte(breadcrumb),
        data([0x00, 0x00, 0x00]),
    ]
}

pub(super) fn run_inspect(
    responses: impl IntoIterator<Item = Response>,
    elf: Option<&ElfImage>,
) -> Result<InspectReport, CliError> {
    let mut mock = MockTransport::new(responses);
    inspect_report(&mut mock, elf, None)
}

/// A temp directory removed when the test ends (or unwinds).
pub(super) struct TempDir(PathBuf);

impl TempDir {
    pub(super) fn new(name: &str) -> Self {
        let path =
            std::env::temp_dir().join(format!("probe-tlsr-diag-{}-{name}", std::process::id()));
        std::fs::create_dir_all(&path).expect("temp dir creates");
        Self(path)
    }

    pub(super) fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Writes an executable shell script, for the disassembler tests.
pub(super) fn script(dir: &Path, name: &str, body: &str) -> PathBuf {
    let path = dir.join(name);
    std::fs::write(&path, body).expect("script writes");
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755))
        .expect("script is executable");
    path
}
