//! Raw flash primitives: `flash-read`, `flash-erase` and `flash-write`.
//!
//! These commands stop the core ([`ensure_stopped`]) and leave it stopped; run
//! `reset` afterwards. Every span is validated before the first request, and
//! `flash-read --out` writes through a sibling temp file so a failed backup
//! cannot truncate an existing one.

use std::fs;
use std::path::{Path, PathBuf};

use probe_proto::{CpuMode, Request, FLASH_SECTOR_SIZE, FLASH_SIZE, MAX_DATA_PAYLOAD};

use super::flash::write_pages;
use super::{expect_ack, read_flash, read_mem, sws_activate, CPU_STATUS_ADDR};
use crate::output::Output;
use crate::transport::ProbeTransport;
use crate::CliError;

/// `flash-read`: dump a flash span, optionally straight into a file.
pub(super) fn flash_read(
    transport: &mut dyn ProbeTransport,
    addr: &str,
    len: &str,
    out: Option<&Path>,
) -> Result<Output, CliError> {
    let (addr, len) = parse_flash_span(addr, len)?;

    ensure_stopped(transport)?;

    // Both ends are validated, so the cursor arithmetic cannot wrap.
    let mut data = Vec::with_capacity(len);
    let mut cursor = addr as usize;
    let end = cursor + len;
    while cursor < end {
        let chunk = (end - cursor)
            .min(MAX_DATA_PAYLOAD)
            .min(FLASH_SIZE as usize - cursor);
        data.extend_from_slice(&read_flash(transport, cursor as u32, chunk as u16)?);
        cursor += chunk;
    }

    let out = match out {
        Some(path) => {
            write_backup(path, &data)?;
            Some(path.display().to_string())
        }
        None => None,
    };
    let data_hex = out.is_none().then(|| super::hex_string(&data));

    Ok(Output::FlashRead {
        addr,
        len,
        out,
        data_hex,
    })
}

/// Writes a backup through `<path>.tmp` and renames it into place, so an
/// existing backup survives a failed write.
fn write_backup(path: &Path, data: &[u8]) -> Result<(), CliError> {
    let mut tmp_name = path.as_os_str().to_owned();
    tmp_name.push(".tmp");
    let tmp = PathBuf::from(tmp_name);

    if let Err(err) = fs::write(&tmp, data) {
        let _ = fs::remove_file(&tmp);
        return Err(CliError::Io(format!("{}: {err}", tmp.display())));
    }
    fs::rename(&tmp, path).map_err(|err| {
        let _ = fs::remove_file(&tmp);
        CliError::Io(format!("{}: {err}", path.display()))
    })
}

/// `flash-erase`: erase the sectors covering `addr..addr + len`.
pub(super) fn flash_erase(
    transport: &mut dyn ProbeTransport,
    addr: &str,
    len: &str,
) -> Result<Output, CliError> {
    let (addr, len) = parse_flash_span(addr, len)?;

    ensure_stopped(transport)?;

    let start = addr - addr % FLASH_SECTOR_SIZE;
    let end = (addr as usize + len)
        .div_ceil(FLASH_SECTOR_SIZE as usize)
        .saturating_mul(FLASH_SECTOR_SIZE as usize)
        .min(FLASH_SIZE as usize) as u32;

    let mut sectors = 0;
    let mut cursor = start;
    while cursor < end {
        expect_ack(transport, Request::FlashEraseSector { addr: cursor })?;
        cursor += FLASH_SECTOR_SIZE;
        sectors += 1;
    }

    Ok(Output::FlashErase { addr, len, sectors })
}

/// `flash-write`: program a file at a flash address.
pub(super) fn flash_write(
    transport: &mut dyn ProbeTransport,
    addr: &str,
    path: &Path,
) -> Result<Output, CliError> {
    let data = crate::read_file(path)?;
    let (addr, len) = validate_flash_span(parse_flash_number(addr)?, data.len() as u64)?;

    ensure_stopped(transport)?;
    let pages = write_pages(transport, addr, &data)?;

    Ok(Output::FlashWrite { addr, len, pages })
}

/// Stops the core unless it already is (0x05) or is stalled (0x06).
fn ensure_stopped(transport: &mut dyn ProbeTransport) -> Result<(), CliError> {
    sws_activate(transport)?;
    let status = read_mem(transport, CPU_STATUS_ADDR, 1)?[0];
    if !matches!(status, 0x05 | 0x06) {
        expect_ack(
            transport,
            Request::CpuCtrl {
                mode: CpuMode::Stop,
            },
        )?;
    }
    Ok(())
}

/// Parses and validates a `flash-*` command span before any request.
fn parse_flash_span(addr: &str, len: &str) -> Result<(u32, usize), CliError> {
    validate_flash_span(parse_flash_number(addr)?, parse_flash_number(len)?)
}

/// Requires `len > 0` and `addr..addr + len` inside the flash.
fn validate_flash_span(addr: u64, len: u64) -> Result<(u32, usize), CliError> {
    if len == 0 {
        return Err(CliError::InvalidAddress {
            message: format!("flash span 0x{addr:x}+0 is empty"),
        });
    }
    let end = addr
        .checked_add(len)
        .ok_or_else(|| CliError::InvalidAddress {
            message: format!("flash span 0x{addr:x}+{len} overflows a 64-bit address"),
        })?;
    if addr >= u64::from(FLASH_SIZE) || end > u64::from(FLASH_SIZE) {
        return Err(CliError::InvalidAddress {
            message: format!("flash span 0x{addr:x}+{len} is outside the {FLASH_SIZE} byte flash"),
        });
    }
    let addr = u32::try_from(addr).expect("addr is below FLASH_SIZE and fits in u32");
    let len = usize::try_from(len).expect("len is at most FLASH_SIZE and fits in usize");
    Ok((addr, len))
}

/// Parses a flash address or length written as `0x..` hex or decimal.
fn parse_flash_number(text: &str) -> Result<u64, CliError> {
    let text = text.trim();
    let parsed = match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => u64::from_str_radix(hex, 16),
        None => text.parse(),
    };
    parsed.map_err(|_| CliError::InvalidAddress {
        message: format!("`{text}` is not a flash address or length"),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commands::run;
    use crate::commands::test_support::*;
    use crate::transport::MockTransport;
    use crate::Command;
    use probe_proto::Response;

    #[test]
    fn flash_read_chunks_at_1024() {
        let mut mock = MockTransport::new([
            Response::Ack,
            stopped(0x06),
            data(vec![0x11; 1024]),
            data(vec![0x22; 1024]),
            data(vec![0x33; 452]),
        ]);

        let output = flash_read(&mut mock, "0x100", "2500", None).expect("flash-read succeeds");

        let Output::FlashRead {
            addr,
            len,
            out,
            data_hex,
        } = output
        else {
            panic!("flash-read must return Output::FlashRead");
        };
        assert_eq!((addr, len, out), (0x100, 2500, None));
        let data_hex = data_hex.expect("hex body");
        assert_eq!(data_hex.len(), 2500 * 2);
        assert!(data_hex.starts_with("1111"), "{data_hex}");
        assert!(data_hex.ends_with("3333"), "{data_hex}");
        assert_eq!(
            mock.requests,
            [
                sws_activate_req(),
                read_mem_req(CPU_STATUS_ADDR, 1),
                read_flash_req(0x100, 1024),
                read_flash_req(0x500, 1024),
                read_flash_req(0x900, 452),
            ]
        );
    }

    #[test]
    fn flash_read_out_writes_the_exact_bytes() {
        let bytes: Vec<u8> = (0..12_u8).map(|byte| byte ^ 0xa5).collect();
        let file = TempFile::new("flash-read-out", &[]);
        let mut mock = MockTransport::new([Response::Ack, stopped(0x05), data(bytes.clone())]);

        let output =
            flash_read(&mut mock, "0x20", "12", Some(file.path())).expect("flash-read --out");

        assert_eq!(
            output,
            Output::FlashRead {
                addr: 0x20,
                len: 12,
                out: Some(file.path().display().to_string()),
                data_hex: None,
            }
        );
        assert_eq!(std::fs::read(file.path()).expect("backup reads"), bytes);
        let mut tmp = file.path().as_os_str().to_owned();
        tmp.push(".tmp");
        assert!(!Path::new(&tmp).exists(), "temp file must be renamed away");
    }

    #[test]
    fn flash_read_out_keeps_the_old_backup_when_the_write_fails() {
        let dest = TempFile::new("flash-read-keep", b"old backup");
        let mut tmp = dest.path().as_os_str().to_owned();
        tmp.push(".tmp");
        let tmp = PathBuf::from(tmp);
        // A directory at the temp path makes the backup write fail.
        std::fs::create_dir(&tmp).expect("temp path fixture");

        let mut mock = MockTransport::new([Response::Ack, stopped(0x05), data([0x5a; 4])]);

        let err = flash_read(&mut mock, "0x20", "4", Some(dest.path()))
            .expect_err("a blocked temp path must fail the backup");

        assert!(matches!(err, CliError::Io(_)), "{err:?}");
        assert_eq!(
            std::fs::read(dest.path()).expect("old backup still reads"),
            b"old backup"
        );
        std::fs::remove_dir(&tmp).expect("temp dir removes");
    }

    #[test]
    fn raw_flash_spans_are_validated_before_any_request() {
        let file = TempFile::new("flash-span", &[0u8; 2]);

        let mut mock = MockTransport::new([]);
        assert!(matches!(
            flash_read(&mut mock, "0x7ffff", "1024", None),
            Err(CliError::InvalidAddress { .. })
        ));
        assert!(mock.requests.is_empty());

        let mut mock = MockTransport::new([]);
        assert!(matches!(
            flash_read(&mut mock, "0x1000", "0", None),
            Err(CliError::InvalidAddress { .. })
        ));
        assert!(mock.requests.is_empty());

        let mut mock = MockTransport::new([]);
        assert!(matches!(
            flash_erase(&mut mock, "0x80000", "1"),
            Err(CliError::InvalidAddress { .. })
        ));
        assert!(mock.requests.is_empty());

        let mut mock = MockTransport::new([]);
        assert!(matches!(
            flash_write(&mut mock, "0x7ffff", file.path()),
            Err(CliError::InvalidAddress { .. })
        ));
        assert!(mock.requests.is_empty());
    }

    #[test]
    fn overflowing_flash_spans_are_rejected_before_any_request() {
        let mut mock = MockTransport::new([]);
        let err = flash_read(&mut mock, "1", "18446744073709551615", None)
            .expect_err("addr + len must not wrap");
        assert!(matches!(err, CliError::InvalidAddress { .. }), "{err:?}");
        assert!(mock.requests.is_empty());

        let mut mock = MockTransport::new([]);
        let err = flash_erase(&mut mock, "1", "18446744073709551615")
            .expect_err("addr + len must not wrap");
        assert!(matches!(err, CliError::InvalidAddress { .. }), "{err:?}");
        assert!(mock.requests.is_empty());
    }

    #[test]
    fn flash_read_of_the_last_flash_byte_succeeds() {
        let mut mock = MockTransport::new([Response::Ack, stopped(0x05), data([0x42])]);

        let output =
            flash_read(&mut mock, "0x7ffff", "1", None).expect("the last byte is in flash");

        assert_eq!(
            output,
            Output::FlashRead {
                addr: 0x7ffff,
                len: 1,
                out: None,
                data_hex: Some("42".to_owned()),
            }
        );
        assert_eq!(
            mock.requests,
            [
                sws_activate_req(),
                read_mem_req(CPU_STATUS_ADDR, 1),
                read_flash_req(0x7ffff, 1),
            ]
        );
    }

    #[test]
    fn flash_erase_aligns_to_sectors_within_flash() {
        let mut mock = MockTransport::new([
            Response::Ack,
            stopped(0x05),
            Response::Ack,
            Response::Ack,
            Response::Ack,
        ]);

        let output = flash_erase(&mut mock, "0x812", "0x2000").expect("flash-erase succeeds");

        assert_eq!(
            output,
            Output::FlashErase {
                addr: 0x812,
                len: 0x2000,
                sectors: 3,
            }
        );
        assert_eq!(
            mock.requests,
            [
                sws_activate_req(),
                read_mem_req(CPU_STATUS_ADDR, 1),
                erase_req(0x0000),
                erase_req(0x1000),
                erase_req(0x2000),
            ]
        );

        // A span ending unaligned at FLASH_SIZE still erases only its sector.
        let mut mock = MockTransport::new([Response::Ack, stopped(0x05), Response::Ack]);
        let output = flash_erase(&mut mock, "0x7f812", "0x7ee").expect("tail erase succeeds");
        assert_eq!(
            output,
            Output::FlashErase {
                addr: 0x7f812,
                len: 0x7ee,
                sectors: 1
            }
        );
        assert_eq!(
            mock.requests,
            [
                sws_activate_req(),
                read_mem_req(CPU_STATUS_ADDR, 1),
                erase_req(0x7f000),
            ]
        );
    }

    #[test]
    fn flash_write_splits_at_page_boundaries() {
        let payload: Vec<u8> = (0..600_u16).map(|byte| byte as u8).collect();
        let file = TempFile::new("flash-write", &payload);
        let mut mock = MockTransport::new([
            Response::Ack,
            stopped(0x06),
            Response::Ack,
            Response::Ack,
            Response::Ack,
            Response::Ack,
        ]);

        let output = flash_write(&mut mock, "0x80f0", file.path()).expect("flash-write succeeds");

        assert_eq!(
            output,
            Output::FlashWrite {
                addr: 0x80f0,
                len: 600,
                pages: 4,
            }
        );
        assert_eq!(
            mock.requests,
            [
                sws_activate_req(),
                read_mem_req(CPU_STATUS_ADDR, 1),
                write_page_req(0x80f0, &payload[..16]),
                write_page_req(0x8100, &payload[16..272]),
                write_page_req(0x8200, &payload[272..528]),
                write_page_req(0x8300, &payload[528..]),
            ]
        );
    }

    #[test]
    fn ensure_stopped_only_stops_a_running_core() {
        for (status, stops) in [
            (0x05_u8, false),
            (0x06, false),
            (0x00, true),
            (0x04, true),
            (0x07, true),
        ] {
            let mut responses = vec![Response::Ack, stopped(status)];
            if stops {
                responses.push(Response::Ack);
            }
            let mut mock = MockTransport::new(responses);

            ensure_stopped(&mut mock).expect("ensure_stopped succeeds");

            let mut expected = vec![sws_activate_req(), read_mem_req(CPU_STATUS_ADDR, 1)];
            if stops {
                expected.push(stop_req());
            }
            assert_eq!(mock.requests, expected, "status 0x{status:02x}");
        }
    }

    #[test]
    fn run_dispatches_to_the_flash_handlers() {
        let mut ctx = test_ctx([Response::Ack, stopped(0x05), data([0xaa, 0xbb])]);
        let mut out = Vec::new();

        let output = run(
            Command::FlashRead {
                addr: "0x20".to_owned(),
                len: "2".to_owned(),
                out: None,
            },
            &mut ctx,
            &mut out,
        )
        .expect("flash-read runs")
        .expect("flash-read returns an output");

        assert_eq!(
            output,
            Output::FlashRead {
                addr: 0x20,
                len: 2,
                out: None,
                data_hex: Some("aabb".to_owned()),
            }
        );
    }
}
