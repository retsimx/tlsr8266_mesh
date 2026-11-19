//! `trap`: arm one hardware breakpoint and wait for it to fire.
//!
//! The probe programs the slot from `SetBreakpoint`; while the core runs, the
//! host polls the slot's control word (`0x0610 + slot * 4`) for bit 31. A hit
//! stops the core terminally, so the wait ends with the full `inspect` capture,
//! an explicit slot clear and a soft reboot — the only way back to a running
//! target. A wait deadline clears the slot *before* it fails, because an
//! orphaned armed slot would re-trap the next boot.
//!
//! The capture is one-shot: the report is written to stdout **before** the
//! clear and the reboot, so a failure in either cannot discard the only copy of
//! the crash dump. Every way out of an armed wait reports whether the slot was
//! really cleared.
//!
//! Breakpoint addresses are code addresses, not SWire-mapped data: resolution
//! reuses the `read`/`write` resolver but deliberately skips
//! [`to_swire`](crate::elf::to_swire), whose physical windows describe MMIO and
//! SRAM, not code.

use std::io::Write;
use std::path::Path;
use std::time::{Duration, Instant};

use probe_proto::{CpuMode, Request};

use super::inspect_report;
use crate::commands::mem::resolve_addr;
use crate::commands::{expect_ack, read_word, sws_activate};
use crate::elf::{AddrSpec, ElfImage};
use crate::output::{print_output, Output};
use crate::transport::ProbeTransport;
use crate::{CliError, Ctx};

/// First breakpoint control register; slot `n` lives at `BASE + n * 4`.
pub(super) const BREAKPOINT_BASE: u16 = 0x0610;

/// Bit 31 of the slot's control word: the core stopped on this breakpoint.
pub(super) const BREAKPOINT_HIT: u32 = 1 << 31;

/// Delay between two polls of the slot; production cadence, like `rtt`.
const TRAP_POLL_MS: u64 = 50;

/// Largest code address: the breakpoint word keeps its address in 24 bits.
const CODE_ADDR_MASK: u32 = 0x00ff_ffff;

/// Everything a hit needs to capture and render its report.
///
/// Grouped into one value so [`wait`] and [`hit`] stay readable, and so the
/// report can be written from the handler before the slot is cleared.
pub(super) struct Capture<'a> {
    /// Where the rendered report goes.
    pub(super) out: &'a mut dyn Write,
    /// Render one JSON object instead of the human block.
    pub(super) json: bool,
    /// Parsed ELF image, for symbolization.
    pub(super) elf: Option<&'a ElfImage>,
    /// ELF path the disassembler re-reads.
    pub(super) elf_path: Option<&'a Path>,
}

/// `trap` at the production poll cadence.
pub(crate) fn run(
    ctx: &mut Ctx,
    out: &mut dyn Write,
    addr_or_symbol: &str,
    slot: u8,
) -> Result<(), CliError> {
    let addr = code_addr(ctx.elf.as_ref(), addr_or_symbol)?;
    let mut capture = Capture {
        out,
        json: ctx.json,
        elf: ctx.elf.as_ref(),
        elf_path: ctx.elf_path.as_deref(),
    };
    wait(
        ctx.transport.as_mut(),
        &mut capture,
        addr,
        slot,
        ctx.timeout_ms,
        Duration::from_millis(TRAP_POLL_MS),
    )
}

/// Resolves `spec` to the raw 24-bit code address a breakpoint needs.
///
/// The shared resolver supplies the provenance (symbol or typed number) and the
/// [`CliError::MissingElf`] failure for a symbol without `--elf`. A symbol's
/// `st_value` carries the Thumb bit, which the hardware comparator does not
/// match, so a symbol-derived address is made even; a numeric address is taken
/// literally. `0` is the wire's "clear this slot" value and anything wider than
/// 24 bits does not fit the breakpoint word, so both are rejected before any
/// request is sent.
pub(super) fn code_addr(elf: Option<&ElfImage>, spec: &str) -> Result<u32, CliError> {
    let addr = match resolve_addr(elf, spec)? {
        AddrSpec::Symbol { value, .. } => u64::from(value & !1),
        AddrSpec::Numeric(addr) | AddrSpec::Wire(addr) => addr,
    };
    if addr == 0 {
        return Err(CliError::InvalidAddress {
            message: format!(
                "`{spec}` resolves to 0, which clears a breakpoint slot; trap needs a non-zero \
                 address inside the 24-bit code space (0x1..=0x00ffffff)"
            ),
        });
    }
    if addr > u64::from(CODE_ADDR_MASK) {
        return Err(CliError::InvalidAddress {
            message: format!(
                "0x{addr:x} is wider than the 24-bit code address space (0x1..=0x00ffffff)"
            ),
        });
    }
    Ok(addr as u32)
}

/// Arms `slot`, then polls it until it fires or the deadline expires.
///
/// `timeout_ms` is the raw `--timeout-ms`: `None` or `0` waits forever. Every
/// way out of the wait clears the slot first, because the armed state is
/// terminal and would re-trap the next boot.
pub(super) fn wait(
    transport: &mut dyn ProbeTransport,
    capture: &mut Capture<'_>,
    addr: u32,
    slot: u8,
    timeout_ms: Option<u64>,
    poll_interval: Duration,
) -> Result<(), CliError> {
    // An unrepresentable instant means "no deadline" rather than a panic: the
    // value comes straight from `--timeout-ms`.
    let deadline = timeout_ms.filter(|ms| *ms > 0).and_then(|ms| {
        Instant::now()
            .checked_add(Duration::from_millis(ms))
            .map(|at| (at, ms))
    });

    sws_activate(transport)?;
    expect_ack(transport, Request::SetBreakpoint { slot, addr })?;

    loop {
        // The deadline is checked before every poll so the wait never reads
        // past it, and so expiry is reported with the slot already cleared.
        let left = match deadline {
            None => None,
            Some((at, ms)) => {
                let left = at.saturating_duration_since(Instant::now());
                if left.is_zero() {
                    return Err(missed(transport, slot, addr, ms));
                }
                Some(left)
            }
        };

        match poll_hit(transport, slot) {
            Ok(true) => return hit(transport, capture, addr, slot),
            Ok(false) => {}
            // A failed read leaves the slot armed, so clear it before reporting
            // the probe fault instead of orphaning the slot.
            Err(error) => return Err(clear_after_poll(transport, slot, error)),
        }

        // Sleeping at most the remaining time keeps expiry accurate.
        std::thread::sleep(left.map_or(poll_interval, |left| left.min(poll_interval)));
    }
}

/// Captures the hit, writes the report, then clears the slot and reboots.
///
/// The order is deliberate: the capture is one-shot and unrecoverable, so it is
/// rendered **before** the clear and the reboot. A failing capture still clears
/// the slot, because the armed state is terminal.
fn hit(
    transport: &mut dyn ProbeTransport,
    capture: &mut Capture<'_>,
    addr: u32,
    slot: u8,
) -> Result<(), CliError> {
    let report = match inspect_report(transport, capture.elf, capture.elf_path) {
        Ok(report) => report,
        Err(error) => return Err(capture_failed(transport, slot, error)),
    };

    let rendered = print_output(
        &Output::Trap {
            addr,
            slot,
            hit: true,
            inspect: Box::new(report),
        },
        capture.json,
        capture.out,
    );

    // The report is out of the process (or the write failed); either way the
    // slot must be cleared, and only a clear permits the reboot.
    if let Err(error) = disarm(transport, slot) {
        return Err(CliError::Device(format!(
            "trap: clearing the breakpoint slot {slot} failed after the capture: {error}; the \
             core is left stopped — run `probe-tlsr reset --soft`"
        )));
    }
    if let Err(error) = reboot(transport) {
        return Err(CliError::Device(format!(
            "trap: the soft reboot after the capture failed: {error}; run `probe-tlsr reset --soft`"
        )));
    }
    rendered
}

/// Reports the expired wait, after clearing the slot.
///
/// The clear runs first and its outcome is part of the message: an armed slot
/// would re-trap the next boot, so the operator must know whether it is clear.
fn missed(transport: &mut dyn ProbeTransport, slot: u8, addr: u32, ms: u64) -> CliError {
    let message = format!("trap: slot {slot} at 0x{addr:06x} was not hit within {ms} ms");
    match disarm(transport, slot) {
        Ok(()) => CliError::Timeout(format!("{message}; the breakpoint slot is cleared")),
        Err(error) => CliError::Timeout(format!("{message}; clearing it also failed: {error}")),
    }
}

/// Clears the slot after a failed poll, keeping the original fault when the
/// clear works and naming both facts when it does not.
fn clear_after_poll(transport: &mut dyn ProbeTransport, slot: u8, error: CliError) -> CliError {
    match disarm(transport, slot) {
        Ok(()) => error,
        Err(clear) => CliError::Device(format!(
            "{error}; clearing the breakpoint slot {slot} also failed: {clear}"
        )),
    }
}

/// Clears the slot after a failed capture, then reports both facts.
///
/// The core was stalled by the capture before it failed, so the message names
/// the documented recovery instead of pretending the core is still running.
fn capture_failed(transport: &mut dyn ProbeTransport, slot: u8, error: CliError) -> CliError {
    match disarm(transport, slot) {
        Ok(()) => CliError::Device(format!(
            "trap: the capture failed: {error}; the breakpoint slot {slot} is cleared and the core \
             is left stopped — run `probe-tlsr reset --soft`"
        )),
        Err(clear) => CliError::Device(format!(
            "trap: the capture failed: {error}; clearing the breakpoint slot {slot} also failed: \
             {clear}"
        )),
    }
}

/// One `ReadMem` of the slot's control word; `true` when bit 31 is set.
fn poll_hit(transport: &mut dyn ProbeTransport, slot: u8) -> Result<bool, CliError> {
    let word = read_word(transport, slot_reg(slot))?;
    Ok(word & BREAKPOINT_HIT != 0)
}

/// Clears `slot`: the probe's "clear this slot" value is `addr == 0`.
fn disarm(transport: &mut dyn ProbeTransport, slot: u8) -> Result<(), CliError> {
    expect_ack(transport, Request::SetBreakpoint { slot, addr: 0 })
}

/// Soft-reboots the core: the only way out of a terminal breakpoint hit.
fn reboot(transport: &mut dyn ProbeTransport) -> Result<(), CliError> {
    expect_ack(
        transport,
        Request::CpuCtrl {
            mode: CpuMode::SoftReboot,
        },
    )
}

/// The control register of `slot`: `0x0610 + slot * 4`.
fn slot_reg(slot: u8) -> u16 {
    BREAKPOINT_BASE + u16::from(slot) * 4
}
