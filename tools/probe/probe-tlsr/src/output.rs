//! Command results and their human/JSON rendering.
//!
//! [`Output`] carries one literal tag per subcommand (the CLI spelling) and
//! stable keys; `--json` prints exactly one object, while human mode prints one
//! short block per success.

use std::io::Write;

use serde::Serialize;

use crate::CliError;

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

/// Writes the command result to `out`: one JSON object or a human block.
pub fn print_output(output: &Output, json: bool, out: &mut dyn Write) -> Result<(), CliError> {
    let text = if json {
        serde_json::to_string(output).map_err(|err| CliError::Io(format!("json: {err}")))?
    } else {
        human_output(output)
    };
    writeln!(out, "{text}").map_err(|err| CliError::Io(format!("stdout: {err}")))
}

/// Human rendering: `key=value` lines for `info`, a hex dump for memory reads,
/// one confirmation line for everything else.
fn human_output(output: &Output) -> String {
    match output {
        Output::Info {
            pc,
            cpu_status,
            cpu_state,
            flash_info,
        } => {
            let mut text =
                format!("pc=0x{pc:08x} cpu_status=0x{cpu_status:02x} cpu_state={cpu_state}");
            if let Some(flash) = flash_info {
                text.push_str(&format!(
                    "\nflash jedec_id={} xtal_mhz={} build_version={} magic_ok={}",
                    flash.jedec_id, flash.xtal_mhz, flash.build_version, flash.magic_ok
                ));
            }
            text
        }
        Output::Flash {
            path,
            bytes,
            sectors_erased,
            pages_written,
            ..
        } => format!(
            "flashed {bytes} bytes from {path}: {sectors_erased} sectors, {pages_written} pages"
        ),
        Output::FlashRead {
            addr,
            len,
            out,
            data_hex,
        } => match (out, data_hex) {
            (Some(out), _) => format!("read {len} bytes at 0x{addr:06x} into {out}"),
            (None, Some(hex)) => hex_dump(*addr, hex, 6),
            (None, None) => format!("read {len} bytes at 0x{addr:06x}"),
        },
        Output::FlashErase { addr, len, sectors } => {
            format!("erased {sectors} sectors covering 0x{addr:06x}+0x{len:x}")
        }
        Output::FlashWrite { addr, len, pages } => {
            format!("wrote {len} bytes at 0x{addr:06x} in {pages} pages")
        }
        Output::Read { addr, data_hex } => hex_dump(u32::from(*addr), data_hex, 4),
        Output::Write { addr, len } => format!("wrote {len} bytes at 0x{addr:04x}"),
        Output::AnalogRead { addr, value } => format!("analog 0x{addr:02x} = 0x{value:02x}"),
        Output::AnalogWrite { addr, value } => format!("analog 0x{addr:02x} <- 0x{value:02x}"),
        Output::Reset { stop, soft } => format!("reset (stop={stop} soft={soft})"),
        Output::Halt => "halted".to_owned(),
    }
}

/// Renders contiguous lower-case hex as 16 bytes per line, each prefixed with
/// its byte address in `width` hex digits.
fn hex_dump(base: u32, hex: &str, width: usize) -> String {
    let mut text = String::new();
    for (line, chunk) in hex.as_bytes().chunks(32).enumerate() {
        if line > 0 {
            text.push('\n');
        }
        let addr = base + (line * 16) as u32;
        text.push_str(&format!("0x{addr:0width$x}:", width = width));
        for pair in chunk.chunks(2) {
            text.push(' ');
            text.push_str(std::str::from_utf8(pair).unwrap_or("??"));
        }
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{json, Value};
    use std::collections::BTreeSet;

    fn flash_info() -> FlashInfo {
        FlashInfo {
            jedec_id: "ef4018".to_owned(),
            xtal_mhz: 12,
            build_version: 3539,
            magic_ok: true,
        }
    }

    fn serialized(output: &Output) -> Value {
        serde_json::to_value(output).expect("output serializes")
    }

    /// One representative value per `Output` variant and key combination, plus
    /// the exact JSON it must serialize to. Equality with this map pins the
    /// literal `command` tag, the key set and every value.
    fn output_cases() -> [(Output, Value); 14] {
        [
            (
                Output::Info {
                    pc: 0x1_10f0,
                    cpu_status: 0x05,
                    cpu_state: "stopped",
                    flash_info: Some(flash_info()),
                },
                json!({
                    "command": "info", "pc": 69872, "cpu_status": 5,
                    "cpu_state": "stopped",
                    "flash_info": {
                        "jedec_id": "ef4018", "xtal_mhz": 12,
                        "build_version": 3539, "magic_ok": true
                    }
                }),
            ),
            (
                Output::Info {
                    pc: 0,
                    cpu_status: 0,
                    cpu_state: "running",
                    flash_info: None,
                },
                json!({
                    "command": "info", "pc": 0, "cpu_status": 0,
                    "cpu_state": "running", "flash_info": null
                }),
            ),
            (
                Output::Flash {
                    path: "build/app.bin".to_owned(),
                    bytes: 78996,
                    sectors_erased: 20,
                    pages_written: 309,
                    target_xtal_mhz: Some(12),
                    image_xtal_mhz: None,
                },
                json!({
                    "command": "flash", "path": "build/app.bin", "bytes": 78996,
                    "sectors_erased": 20, "pages_written": 309,
                    "target_xtal_mhz": 12, "image_xtal_mhz": null
                }),
            ),
            (
                Output::Flash {
                    path: "raw.bin".to_owned(),
                    bytes: 4,
                    sectors_erased: 1,
                    pages_written: 1,
                    target_xtal_mhz: None,
                    image_xtal_mhz: None,
                },
                json!({
                    "command": "flash", "path": "raw.bin", "bytes": 4,
                    "sectors_erased": 1, "pages_written": 1,
                    "target_xtal_mhz": null, "image_xtal_mhz": null
                }),
            ),
            (
                Output::FlashRead {
                    addr: 0,
                    len: 12,
                    out: Some("backup.bin".to_owned()),
                    data_hex: None,
                },
                json!({
                    "command": "flash-read", "addr": 0, "len": 12,
                    "out": "backup.bin", "data_hex": null
                }),
            ),
            (
                Output::FlashRead {
                    addr: 0x20,
                    len: 2,
                    out: None,
                    data_hex: Some("aabb".to_owned()),
                },
                json!({
                    "command": "flash-read", "addr": 32, "len": 2,
                    "out": null, "data_hex": "aabb"
                }),
            ),
            (
                Output::FlashErase {
                    addr: 0x1000,
                    len: 0x2000,
                    sectors: 2,
                },
                json!({"command": "flash-erase", "addr": 4096, "len": 8192, "sectors": 2}),
            ),
            (
                Output::FlashWrite {
                    addr: 0x8000,
                    len: 256,
                    pages: 1,
                },
                json!({"command": "flash-write", "addr": 32768, "len": 256, "pages": 1}),
            ),
            (
                Output::Read {
                    addr: 0x8000,
                    data_hex: "deadbeef".to_owned(),
                },
                json!({"command": "read", "addr": 32768, "data_hex": "deadbeef"}),
            ),
            (
                Output::Write {
                    addr: 0x8000,
                    len: 4,
                },
                json!({"command": "write", "addr": 32768, "len": 4}),
            ),
            (
                Output::AnalogRead {
                    addr: 0x0c,
                    value: 0x42,
                },
                json!({"command": "analog-read", "addr": 12, "value": 66}),
            ),
            (
                Output::AnalogWrite {
                    addr: 0x0c,
                    value: 0xff,
                },
                json!({"command": "analog-write", "addr": 12, "value": 255}),
            ),
            (
                Output::Reset {
                    stop: false,
                    soft: true,
                },
                json!({"command": "reset", "stop": false, "soft": true}),
            ),
            (Output::Halt, json!({"command": "halt"})),
        ]
    }

    fn strings(tags: &[&str]) -> BTreeSet<String> {
        tags.iter().map(|tag| (*tag).to_owned()).collect()
    }

    #[test]
    fn output_variants_serialize_with_literal_tags() {
        for (output, expected) in output_cases() {
            assert_eq!(serialized(&output), expected, "{output:?}");
        }
    }

    #[test]
    fn every_output_variant_has_a_literal_tag() {
        let mut tags = BTreeSet::new();
        for (output, expected) in output_cases() {
            let value = serialized(&output);
            assert_eq!(value["command"], expected["command"], "{output:?}");
            tags.insert(
                value["command"]
                    .as_str()
                    .expect("tag is a string")
                    .to_owned(),
            );
        }

        let expected = strings(&[
            "analog-read",
            "analog-write",
            "flash",
            "flash-erase",
            "flash-read",
            "flash-write",
            "halt",
            "info",
            "read",
            "reset",
            "write",
        ]);
        assert_eq!(tags, expected);
    }

    #[test]
    fn json_success_writes_one_object_to_stdout() {
        let mut out = Vec::new();
        print_output(&Output::Halt, true, &mut out).expect("halt serializes");

        assert_eq!(
            String::from_utf8(out).expect("utf-8"),
            "{\"command\":\"halt\"}\n"
        );
    }

    #[test]
    fn human_info_prints_key_value_lines() {
        let running = Output::Info {
            pc: 0x0001_10f0,
            cpu_status: 0x00,
            cpu_state: "running",
            flash_info: None,
        };
        assert_eq!(
            human_output(&running),
            "pc=0x000110f0 cpu_status=0x00 cpu_state=running"
        );

        let stopped = Output::Info {
            pc: 0,
            cpu_status: 0x05,
            cpu_state: "stopped",
            flash_info: Some(flash_info()),
        };
        assert_eq!(
            human_output(&stopped),
            concat!(
                "pc=0x00000000 cpu_status=0x05 cpu_state=stopped\n",
                "flash jedec_id=ef4018 xtal_mhz=12 build_version=3539 magic_ok=true"
            )
        );
    }

    #[test]
    fn human_memory_reads_are_sixteen_byte_hex_dumps() {
        let read = Output::Read {
            addr: 0x8000,
            data_hex: "00112233445566778899aabbccddeeff0102".to_owned(),
        };
        assert_eq!(
            human_output(&read),
            concat!(
                "0x8000: 00 11 22 33 44 55 66 77 88 99 aa bb cc dd ee ff\n",
                "0x8010: 01 02"
            )
        );

        let dump = Output::FlashRead {
            addr: 0x20,
            len: 2,
            out: None,
            data_hex: Some("aabb".to_owned()),
        };
        assert_eq!(human_output(&dump), "0x000020: aa bb");

        let backup = Output::FlashRead {
            addr: 0x20,
            len: 2,
            out: Some("backup.bin".to_owned()),
            data_hex: None,
        };
        assert_eq!(
            human_output(&backup),
            "read 2 bytes at 0x000020 into backup.bin"
        );
    }

    #[test]
    fn human_control_commands_are_single_lines() {
        assert_eq!(human_output(&Output::Halt), "halted");
        assert_eq!(
            human_output(&Output::Reset {
                stop: false,
                soft: true,
            }),
            "reset (stop=false soft=true)"
        );
        assert_eq!(
            human_output(&Output::AnalogRead {
                addr: 0x0c,
                value: 0x42,
            }),
            "analog 0x0c = 0x42"
        );
        assert_eq!(
            human_output(&Output::AnalogWrite {
                addr: 0x0c,
                value: 0xff,
            }),
            "analog 0x0c <- 0xff"
        );
        assert_eq!(
            human_output(&Output::Write {
                addr: 0x8000,
                len: 4,
            }),
            "wrote 4 bytes at 0x8000"
        );
        assert_eq!(
            human_output(&Output::FlashErase {
                addr: 0x812,
                len: 0x2000,
                sectors: 3,
            }),
            "erased 3 sectors covering 0x000812+0x2000"
        );
        assert_eq!(
            human_output(&Output::FlashWrite {
                addr: 0x80f0,
                len: 600,
                pages: 4,
            }),
            "wrote 600 bytes at 0x0080f0 in 4 pages"
        );
    }
}
