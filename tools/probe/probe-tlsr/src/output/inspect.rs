//! The `inspect` report as text: registers, stacks, reset cause, panic body
//! and the optional disassembly block.
//!
//! `trap` reuses [`inspect_text`] verbatim under its hit line, so a crash dump
//! reads the same whichever command captured it.

use crate::commands::diag::{InspectReport, StackDump};

/// Human rendering of one inspect report: registers, stacks, reset cause,
/// panic body and, when available, the disassembly block.
pub(super) fn inspect_text(report: &InspectReport) -> String {
    let mut text = format!(
        "pc=0x{:08x}{}\nlr=0x{:08x}{}\nsp=0x{:08x}\npsr=0x{:08x}",
        report.regs.pc,
        annotation(report.pc_symbol.as_deref()),
        report.regs.lr,
        annotation(report.lr_symbol.as_deref()),
        report.regs.sp,
        report.regs.psr,
    );

    let general = [
        ("r0", report.regs.r0),
        ("r1", report.regs.r1),
        ("r2", report.regs.r2),
        ("r3", report.regs.r3),
        ("r4", report.regs.r4),
        ("r5", report.regs.r5),
        ("r6", report.regs.r6),
        ("r7", report.regs.r7),
        ("r8", report.regs.r8),
        ("r9", report.regs.r9),
        ("r10", report.regs.r10),
        ("r11", report.regs.r11),
        ("r12", report.regs.r12),
    ];
    for (name, value) in general {
        text.push_str(&format!("\n{name}=0x{value:08x}"));
    }

    text.push_str("\nshadow=");
    for (index, value) in report.shadow.iter().enumerate() {
        if index > 0 {
            text.push(' ');
        }
        text.push_str(&format!("0x{value:08x}"));
    }

    push_stack(&mut text, "stack", &report.stack);
    if let Some(irq) = &report.irq_stack {
        push_stack(&mut text, "irq_stack", irq);
    }

    if !report.stack_symbols.is_empty() {
        text.push_str("\nstack_symbols:");
        for symbol in &report.stack_symbols {
            text.push_str(&format!("\n0x{:08x} {}", symbol.addr, symbol.symbol));
        }
    }

    text.push_str(&format!(
        "\nreset sentinel=0x{:02x} ({}) breadcrumb={}",
        report.sentinel, report.sentinel_state, report.breadcrumb
    ));
    if let Some(panic) = &report.panic {
        text.push_str(&format!(
            "\npanic len={} message={}",
            panic.len, panic.message
        ));
    }
    if let Some(disassembly) = &report.disassembly {
        text.push_str("\ndisassembly:\n");
        text.push_str(disassembly);
    }

    text
}

/// Renders an optional symbol as a trailing ` name` annotation.
fn annotation(symbol: Option<&str>) -> String {
    match symbol {
        Some(symbol) => format!(" {symbol}"),
        None => String::new(),
    }
}

/// Appends one stack span: a header line, then 16 B hex-dump lines.
///
/// An empty span (the post-boot `SP == stack top` case) says so instead of
/// leaving a header with a dangling colon.
fn push_stack(text: &mut String, label: &str, stack: &StackDump) {
    if stack.len == 0 {
        text.push_str(&format!("\n{label} 0x{:08x} len=0 (empty)", stack.addr));
        return;
    }
    text.push_str(&format!(
        "\n{label} 0x{:08x} len={}:",
        stack.addr, stack.len
    ));
    for (line, words) in stack.words.chunks(4).enumerate() {
        text.push_str(&format!("\n0x{:08x}:", stack.addr + (line as u32) * 16));
        for word in words {
            text.push_str(&format!(" {word:08x}"));
        }
    }
}
