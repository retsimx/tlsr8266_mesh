//! Command results and their human/JSON rendering.
//!
//! [`Output`] carries one literal tag per subcommand (the CLI spelling) and
//! stable keys; `--json` prints exactly one object, while human mode prints one
//! short block per success. The renderers live in [`human`] and [`inspect`].

mod human;
mod inspect;

pub use human::print_output;

use serde::Serialize;

use crate::commands::diag::InspectReport;

/// One JSON object per successful command, tagged with the CLI spelling.
#[derive(Debug, PartialEq, Serialize)]
#[serde(tag = "command")]
pub enum Output {
    #[serde(rename = "info")]
    Info {
        pc: u32,
        /// CPU status byte sampled before `--halt` is applied.
        cpu_status: u8,
        /// State derived from [`Self::Info::cpu_status`], so also pre-halt.
        cpu_state: &'static str,
        flash_info: Option<FlashInfo>,
    },
    #[serde(rename = "flash")]
    Flash {
        path: String,
        bytes: usize,
        sectors_erased: usize,
        pages_written: usize,
        target_xtal_mhz: Option<u8>,
        image_xtal_mhz: Option<u8>,
    },
    #[serde(rename = "flash-read")]
    FlashRead {
        addr: u32,
        len: usize,
        out: Option<String>,
        data_hex: Option<String>,
    },
    #[serde(rename = "flash-erase")]
    FlashErase {
        addr: u32,
        len: usize,
        sectors: usize,
    },
    #[serde(rename = "flash-write")]
    FlashWrite { addr: u32, len: usize, pages: usize },
    #[serde(rename = "read")]
    Read { addr: u16, data_hex: String },
    #[serde(rename = "write")]
    Write { addr: u16, len: usize },
    #[serde(rename = "analog-read")]
    AnalogRead { addr: u8, value: u8 },
    #[serde(rename = "analog-write")]
    AnalogWrite { addr: u8, value: u8 },
    #[serde(rename = "reset")]
    Reset { stop: bool, soft: bool },
    #[serde(rename = "inspect")]
    Inspect {
        /// Flattened so the JSON is `{"command":"inspect", …report fields…}`.
        /// Boxed because the report is far larger than every other variant.
        #[serde(flatten)]
        report: Box<InspectReport>,
    },
    /// One streamed log line; `rtt` writes one of these per emitted line.
    #[serde(rename = "rtt")]
    RttLine { line: String },
    /// A hardware breakpoint hit plus the capture taken at the hit.
    ///
    /// The report is nested (not flattened), so the JSON is
    /// `{"command":"trap", …, "inspect": {…}}` and the inner object keeps no
    /// `command` key. `hit` is always `true` today — the tag is only printed
    /// after the breakpoint fired — and is in the shape so the JSON states the
    /// outcome instead of implying it.
    #[serde(rename = "trap")]
    Trap {
        addr: u32,
        slot: u8,
        hit: bool,
        inspect: Box<InspectReport>,
    },
    #[serde(rename = "halt")]
    Halt,
}

/// Firmware header decoded from the first flash sector.
#[derive(Debug, PartialEq, Serialize)]
pub struct FlashInfo {
    /// JEDEC ID as lower-case hex without separators.
    pub jedec_id: String,
    /// Crystal selected by the header: 12 or 16 MHz.
    pub xtal_mhz: u8,
    /// Firmware version with the crystal bit masked off.
    pub build_version: u32,
    /// Whether the header carried the KNLT magic.
    pub magic_ok: bool,
}

#[cfg(test)]
mod tests;
