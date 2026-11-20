//! `inspect`: stall the core and dump its crash state.
//!
//! One SWire session does everything a post-mortem needs: the stall plus
//! watchdog clear, the 128 B register block, the main and IRQ stacks, the
//! retained analog reset sentinel and breadcrumb, and the flash panic record.
//! Addresses are then symbolized against `.symtab` and, when `--elf` and a
//! `tc32-elf-objdump` are both available, a small window around `PC` is
//! disassembled.
//!
//! A failing probe request aborts the command; anything that is merely absent
//! (no `irq_stk` symbol, no panic record, no objdump) degrades to `null`.
//!
//! `inspect` leaves the core stalled, exactly like `halt`: the documented
//! recovery is `probe-tlsr reset --soft`.

use std::ffi::OsStr;
use std::path::{Path, PathBuf};
use std::process::Command;

pub(crate) mod rtt;
pub(crate) mod trap;

use probe_proto::{is_valid_swire_range, Request, Response};
use serde::Serialize;

use super::{
    expect_len, read_analog_value, read_flash, read_mem, stall_core, sws_activate, unexpected,
};
use crate::elf::{symbolize, ElfImage, SymbolRange};
use crate::output::Output;
use crate::transport::ProbeTransport;
use crate::{CliError, Ctx};

/// First byte of the main stack, in the SWire SRAM window.
const MAIN_STACK_FLOOR: u32 = 0x0080_0000 + 0x8000;

/// One past the last main-stack byte; the stack grows down from here.
const MAIN_STACK_TOP: u32 = 0x0080_c000;

/// Most stack bytes a single `ReadMem` may cover.
const STACK_SPAN: u32 = 128;

/// `irq_stk` size assumed when `.symtab` reports `st_size == 0`.
const IRQ_STK_DEFAULT: u32 = 0x700;

/// Required `.symtab` name of the IRQ stack.
const IRQ_STK_SYMBOL: &str = "irq_stk";

/// Exact length of the `ReadRegs` reply.
const REGS_LEN: u16 = 128;

/// Register-block offset of `PSR`.
const PSR_OFFSET: usize = 0x40;

/// Register-block offset of the shadow bank.
const SHADOW_OFFSET: usize = 0x44;

/// Number of shadow words after `PSR`.
const SHADOW_WORDS: usize = 14;

/// Analog register holding the retained reset sentinel.
const RESET_SENTINEL_ADDR: u8 = 0x3b;

/// Analog register holding the last breadcrumb.
const BREADCRUMB_ADDR: u8 = 0x3c;

/// Flash address of the panic record: flag, `u16` length, then the message.
const PANIC_SLOT_ADDR: u32 = 0x7b000;

/// Flag byte that marks a valid panic record.
const PANIC_VALID_FLAG: u8 = 0xde;

/// Largest panic body the record may hold.
const PANIC_MAX: u16 = 256;

/// Name of the TC32 disassembler.
const OBJDUMP_NAME: &str = "tc32-elf-objdump";

/// Bytes disassembled on either side of `PC`.
const DISASM_WINDOW: u32 = 32;

/// Stack bytes below the SWire window base (`0x800000`).
const SRAM_WINDOW_BASE: u32 = 0x0080_0000;

/// Everything `inspect` reports, in the documented JSON shape.
#[derive(Debug, PartialEq, Serialize)]
pub struct InspectReport {
    /// The 16 general-purpose registers plus `PSR`.
    pub regs: Regs,
    /// The shadow bank, in register-block order.
    pub shadow: [u32; SHADOW_WORDS],
    /// The tapped span of the main stack.
    pub stack: StackDump,
    /// The tapped span of `irq_stk`, when the symbol resolves.
    pub irq_stack: Option<StackDump>,
    /// Analog `0x3b` as read.
    pub sentinel: u8,
    /// [`Self::sentinel`] decoded to a printable state.
    pub sentinel_state: &'static str,
    /// Analog `0x3c` as read.
    pub breadcrumb: u8,
    /// The flash panic record, when one is valid.
    pub panic: Option<PanicBody>,
    /// `PC` rendered as `name` or `name+0xNN`.
    pub pc_symbol: Option<String>,
    /// `LR` rendered as `name` or `name+0xNN`.
    pub lr_symbol: Option<String>,
    /// Stack words whose value lands inside a code symbol.
    pub stack_symbols: Vec<StackSymbol>,
    /// `PC`-centred disassembly, when a tool and an ELF path exist.
    pub disassembly: Option<String>,
}

/// The `ReadRegs` block decoded: `r0..r12`, `sp`, `lr`, `pc`, `psr`.
#[derive(Debug, PartialEq, Serialize)]
pub struct Regs {
    pub r0: u32,
    pub r1: u32,
    pub r2: u32,
    pub r3: u32,
    pub r4: u32,
    pub r5: u32,
    pub r6: u32,
    pub r7: u32,
    pub r8: u32,
    pub r9: u32,
    pub r10: u32,
    pub r11: u32,
    pub r12: u32,
    pub sp: u32,
    pub lr: u32,
    pub pc: u32,
    pub psr: u32,
}

/// One tapped stack span: its physical address, byte length and LE words.
#[derive(Debug, PartialEq, Serialize)]
pub struct StackDump {
    pub addr: u32,
    pub len: u32,
    pub words: Vec<u32>,
}

/// The panic message left in flash by the target's panic handler.
#[derive(Debug, PartialEq, Serialize)]
pub struct PanicBody {
    pub len: u32,
    pub message: String,
}

/// A stack word whose value names a code symbol.
#[derive(Debug, PartialEq, Serialize)]
pub struct StackSymbol {
    /// Address of the stack word, not of the code it points at.
    pub addr: u32,
    pub symbol: String,
}

/// `inspect`: build the report and wrap it for rendering.
pub(super) fn inspect(ctx: &mut Ctx) -> Result<Output, CliError> {
    let report = inspect_report(
        ctx.transport.as_mut(),
        ctx.elf.as_ref(),
        ctx.elf_path.as_deref(),
    )?;
    Ok(Output::Inspect {
        report: Box::new(report),
    })
}

/// Runs the whole inspect sequence over `transport`.
///
/// `elf` supplies symbols and code ranges; `elf_path` is the file the
/// disassembler re-reads, which is why it is separate from the parsed image.
pub(crate) fn inspect_report(
    transport: &mut dyn ProbeTransport,
    elf: Option<&ElfImage>,
    elf_path: Option<&Path>,
) -> Result<InspectReport, CliError> {
    sws_activate(transport)?;
    stall_core(transport)?;

    let block = match transport.exchange(&Request::ReadRegs)? {
        Response::Data(data) => data,
        other => return Err(unexpected("Data", &other)),
    };
    expect_len(block.as_slice(), REGS_LEN)?;
    let regs = decode_regs(block.as_slice());
    let shadow = decode_shadow(block.as_slice());

    let start = regs.sp.clamp(MAIN_STACK_FLOOR, MAIN_STACK_TOP);
    let len = (MAIN_STACK_TOP - start).min(STACK_SPAN);
    let stack = stack_dump(transport, start, len)?;

    let irq_stack = read_irq_stack(transport, elf)?;

    let sentinel = read_analog_value(transport, RESET_SENTINEL_ADDR)?;
    let breadcrumb = read_analog_value(transport, BREADCRUMB_ADDR)?;

    let panic = panic_record(transport)?;

    let ranges = match elf {
        Some(image) => image.code_symbols()?,
        None => Vec::new(),
    };
    let mut stack_symbols = symbolize_stack(&ranges, &stack);
    if let Some(irq) = &irq_stack {
        stack_symbols.extend(symbolize_stack(&ranges, irq));
    }

    Ok(InspectReport {
        pc_symbol: symbolize(&ranges, regs.pc),
        lr_symbol: symbolize(&ranges, regs.lr),
        disassembly: disassembly(elf, elf_path, regs.pc),
        regs,
        shadow,
        stack,
        irq_stack,
        sentinel,
        sentinel_state: sentinel_state(sentinel),
        breadcrumb,
        panic,
        stack_symbols,
    })
}

/// Decodes the register block's GPRs and `PSR`; the caller checked 128 B.
fn decode_regs(block: &[u8]) -> Regs {
    Regs {
        r0: word(block, 0x00),
        r1: word(block, 0x04),
        r2: word(block, 0x08),
        r3: word(block, 0x0c),
        r4: word(block, 0x10),
        r5: word(block, 0x14),
        r6: word(block, 0x18),
        r7: word(block, 0x1c),
        r8: word(block, 0x20),
        r9: word(block, 0x24),
        r10: word(block, 0x28),
        r11: word(block, 0x2c),
        r12: word(block, 0x30),
        sp: word(block, 0x34),
        lr: word(block, 0x38),
        pc: word(block, 0x3c),
        psr: word(block, PSR_OFFSET),
    }
}

/// Decodes the 14 shadow words, in register-block order.
fn decode_shadow(block: &[u8]) -> [u32; SHADOW_WORDS] {
    let mut shadow = [0u32; SHADOW_WORDS];
    for (index, slot) in shadow.iter_mut().enumerate() {
        *slot = word(block, SHADOW_OFFSET + index * 4);
    }
    shadow
}

/// Reads one little-endian word at a fixed register-block offset.
fn word(block: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        block[offset],
        block[offset + 1],
        block[offset + 2],
        block[offset + 3],
    ])
}

/// Reads `len` stack bytes at physical `addr`; a zero `len` sends nothing.
fn stack_dump(
    transport: &mut dyn ProbeTransport,
    addr: u32,
    len: u32,
) -> Result<StackDump, CliError> {
    let words = if len == 0 {
        Vec::new()
    } else {
        let wire = stack_wire(addr, len)?;
        let wire_len = u16::try_from(len)
            .map_err(|_| CliError::Device(format!("stack span {len} is too wide")))?;
        decode_words(&read_mem(transport, wire, wire_len)?)
    };
    Ok(StackDump { addr, len, words })
}

/// Maps a physical stack span onto its SWire address, or reports a device
/// fault when it is not entirely inside the mapped SRAM window.
fn stack_wire(addr: u32, len: u32) -> Result<u16, CliError> {
    let wire = addr
        .checked_sub(SRAM_WINDOW_BASE)
        .and_then(|wire| u16::try_from(wire).ok())
        .and_then(|wire| u16::try_from(len).ok().map(|len| (wire, len)))
        .filter(|&(wire, len)| is_valid_swire_range(wire, len))
        .map(|(wire, _)| wire);
    wire.ok_or_else(|| {
        CliError::Device(format!(
            "stack span 0x{addr:08x}+{len} is not inside the SWire SRAM window"
        ))
    })
}

/// Splits bytes into little-endian words; a trailing partial word is dropped.
fn decode_words(bytes: &[u8]) -> Vec<u32> {
    bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|word| u32::from_le_bytes(*word))
        .collect()
}

/// Reads the top [`STACK_SPAN`] bytes of `irq_stk`, or `None` when the symbol
/// is absent or its span cannot be read over SWire.
fn read_irq_stack(
    transport: &mut dyn ProbeTransport,
    elf: Option<&ElfImage>,
) -> Result<Option<StackDump>, CliError> {
    let Some((base, size)) = elf.and_then(|image| image.lookup_symbol(IRQ_STK_SYMBOL)) else {
        return Ok(None);
    };
    let size = if size > 0 { size } else { IRQ_STK_DEFAULT };
    let len = size.min(STACK_SPAN);
    let start = base.saturating_add(size).saturating_sub(len);
    if stack_wire(start, len).is_err() {
        return Ok(None);
    }
    stack_dump(transport, start, len).map(Some)
}

/// Reads the panic record: `0xde`, a `u16` length, then the UTF-8 body.
///
/// An invalid flag or an out-of-range length yields `None` without a second
/// flash request.
fn panic_record(transport: &mut dyn ProbeTransport) -> Result<Option<PanicBody>, CliError> {
    let header = read_flash(transport, PANIC_SLOT_ADDR, 3)?;
    if header[0] != PANIC_VALID_FLAG {
        return Ok(None);
    }
    let len = u16::from_le_bytes([header[1], header[2]]);
    if len == 0 || len > PANIC_MAX {
        return Ok(None);
    }
    let body = read_flash(transport, PANIC_SLOT_ADDR + 3, len)?;
    Ok(Some(PanicBody {
        len: u32::from(len),
        message: String::from_utf8_lossy(&body).into_owned(),
    }))
}

/// Names the retained sentinel value.
fn sentinel_state(sentinel: u8) -> &'static str {
    match sentinel {
        0x00 => "CleanReboot",
        0xA5 => "Running",
        0xE1 => "Panic",
        _ => "Unknown",
    }
}

/// Symbolizes every stack word whose value lands in a code range.
fn symbolize_stack(ranges: &[SymbolRange], stack: &StackDump) -> Vec<StackSymbol> {
    stack
        .words
        .iter()
        .enumerate()
        .filter_map(|(index, &value)| {
            let addr = stack.addr + (index as u32) * 4;
            symbolize(ranges, value).map(|symbol| StackSymbol { addr, symbol })
        })
        .collect()
}

/// Disassembles around `pc` when the ELF path and a tool are both available.
fn disassembly(elf: Option<&ElfImage>, elf_path: Option<&Path>, pc: u32) -> Option<String> {
    elf?;
    let elf_path = elf_path?;
    let root = std::env::current_dir().ok()?;
    let tool = objdump_path(&root, std::env::var_os("PATH").as_deref())?;
    disassemble(&tool, elf_path, pc)
}

/// Finds `tc32-elf-objdump` on `path_env`, else in `root/toolchain/tc32/bin`.
///
/// Returns `None` instead of an error so hosts without the toolchain degrade to
/// `disassembly: null`.
fn objdump_path(root: &Path, path_env: Option<&OsStr>) -> Option<PathBuf> {
    if let Some(path_env) = path_env {
        for dir in std::env::split_paths(path_env) {
            let candidate = dir.join(OBJDUMP_NAME);
            if candidate.is_file() {
                return Some(candidate);
            }
        }
    }
    let candidate = root.join("toolchain/tc32/bin").join(OBJDUMP_NAME);
    candidate.is_file().then_some(candidate)
}

/// Runs `-d` over `pc - 32 .. pc + 32`, returning `None` on any failure.
fn disassemble(tool: &Path, elf_path: &Path, pc: u32) -> Option<String> {
    let start = pc.saturating_sub(DISASM_WINDOW);
    let stop = pc.saturating_add(DISASM_WINDOW);
    let output = Command::new(tool)
        .arg("-d")
        .arg(format!("--start-address=0x{start:x}"))
        .arg(format!("--stop-address=0x{stop:x}"))
        .arg(elf_path)
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let text = String::from_utf8_lossy(&output.stdout)
        .trim_end()
        .to_owned();
    (!text.is_empty()).then_some(text)
}

#[cfg(test)]
mod tests;
