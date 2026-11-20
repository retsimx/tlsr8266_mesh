//! Fixtures shared by the output tests: a fully populated and a fully empty
//! `inspect` report, plus the firmware header.

use super::super::FlashInfo;
use crate::commands::diag::{InspectReport, PanicBody, Regs, StackDump, StackSymbol};

pub(super) fn flash_info() -> FlashInfo {
    FlashInfo {
        jedec_id: "ef4018".to_owned(),
        build_version: 3539,
        magic_ok: true,
    }
}

/// A full `inspect` report that exercises every nested shape: both stacks,
/// a panic body, symbolized addresses and a disassembly block.
pub(super) fn inspect_report() -> InspectReport {
    let mut shadow = [0u32; 14];
    shadow[0] = 0x11;
    shadow[13] = 0x22;
    InspectReport {
        regs: Regs {
            r0: 0,
            r1: 1,
            r2: 2,
            r3: 3,
            r4: 4,
            r5: 5,
            r6: 6,
            r7: 7,
            r8: 8,
            r9: 9,
            r10: 10,
            r11: 11,
            r12: 12,
            sp: 0x0080_bfc0,
            lr: 0x20,
            pc: 0x104,
            psr: 0x0100_0000,
        },
        shadow,
        stack: StackDump {
            addr: 0x0080_bfc0,
            len: 8,
            words: vec![0x20, 0xdead_beef],
        },
        irq_stack: Some(StackDump {
            addr: 0x0080_b580,
            len: 4,
            words: vec![0x104],
        }),
        sentinel: 0xa5,
        sentinel_state: "Running",
        breadcrumb: 3,
        panic: Some(PanicBody {
            len: 5,
            message: "crash".to_owned(),
        }),
        pc_symbol: Some("sym+0x4".to_owned()),
        lr_symbol: Some("sym".to_owned()),
        stack_symbols: vec![
            StackSymbol {
                addr: 0x0080_bfc0,
                symbol: "sym".to_owned(),
            },
            StackSymbol {
                addr: 0x0080_b580,
                symbol: "sym+0x4".to_owned(),
            },
        ],
        disassembly: Some("00000104 <sym>:\n 104:\tpush\t{r4, lr}".to_owned()),
    }
}

/// An `inspect` report with every optional part absent.
pub(super) fn empty_report() -> InspectReport {
    InspectReport {
        regs: Regs {
            r0: 0,
            r1: 0,
            r2: 0,
            r3: 0,
            r4: 0,
            r5: 0,
            r6: 0,
            r7: 0,
            r8: 0,
            r9: 0,
            r10: 0,
            r11: 0,
            r12: 0,
            sp: 0x0080_c000,
            lr: 0,
            pc: 0,
            psr: 0,
        },
        shadow: [0; 14],
        stack: StackDump {
            addr: 0x0080_c000,
            len: 0,
            words: Vec::new(),
        },
        irq_stack: None,
        sentinel: 0,
        sentinel_state: "CleanReboot",
        breadcrumb: 0,
        panic: None,
        pc_symbol: None,
        lr_symbol: None,
        stack_symbols: Vec::new(),
        disassembly: None,
    }
}
