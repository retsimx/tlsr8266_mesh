//! `flash`: program an ELF or flat binary image.
//!
//! The image is erased and written from a stopped core. Without `--reset` the
//! core stays stopped when the command returns, and an aborted flash (crystal
//! mismatch or a failed erase/page write) also leaves it stopped.

use std::path::Path;

use probe_proto::{Request, FLASH_PAGE_SIZE, FLASH_SECTOR_SIZE, FLASH_SIZE};

use super::{
    expect_ack, read_flash, sws_activate, FLASH_HEADER_ADDR, FLASH_HEADER_LEN, KNLT_MAGIC,
    RESET_HOLD_MS, XTAL_BIT,
};
use crate::elf::ElfImage;
use crate::output::Output;
use crate::transport::ProbeTransport;
use crate::CliError;

/// ELF magic that selects the ELF parser instead of a raw binary image.
const ELF_MAGIC: [u8; 4] = *b"\x7fELF";

/// `flash`: load an image, guard the crystal, erase and program it.
///
/// A file starting with the ELF magic is flattened through [`ElfImage`];
/// anything else is a raw binary image. The 12-byte header read happens even
/// when the image cannot be guarded, so the request sequence stays uniform.
pub(super) fn flash(
    transport: &mut dyn ProbeTransport,
    path: &Path,
    force: bool,
    reset: bool,
) -> Result<Output, CliError> {
    let image = load_image(path)?;

    expect_ack(
        transport,
        Request::Reset {
            hold_ms: RESET_HOLD_MS,
            stop_cpu: true,
        },
    )?;
    sws_activate(transport)?;

    let target = read_flash(transport, FLASH_HEADER_ADDR, FLASH_HEADER_LEN)?;
    let target_xtal_mhz = header_xtal_mhz(&target);
    let image_xtal_mhz = header_xtal_mhz(&image);

    if let (Some(target_mhz), Some(image_mhz)) = (target_xtal_mhz, image_xtal_mhz) {
        if target_mhz != image_mhz && !force {
            return Err(CliError::CrystalMismatch {
                target_mhz,
                image_mhz,
            });
        }
    }

    let sectors_erased = erase_image(transport, image.len())?;
    let pages_written = write_pages(transport, 0, &image)?;

    if reset {
        expect_ack(
            transport,
            Request::Reset {
                hold_ms: RESET_HOLD_MS,
                stop_cpu: false,
            },
        )?;
    }

    Ok(Output::Flash {
        path: path.display().to_string(),
        bytes: image.len(),
        sectors_erased,
        pages_written,
        target_xtal_mhz,
        image_xtal_mhz,
    })
}

/// Reads `path` and returns the image to program: an ELF flat image or the raw
/// bytes.
fn load_image(path: &Path) -> Result<Vec<u8>, CliError> {
    let bytes = crate::read_file(path)?;
    let image = if bytes.starts_with(&ELF_MAGIC) {
        ElfImage::parse(&bytes)?.flat_image()?
    } else {
        bytes
    };

    if image.is_empty() {
        return Err(CliError::Device(format!("{} is empty", path.display())));
    }
    if image.len() > FLASH_SIZE as usize {
        return Err(CliError::Device(format!(
            "{} is {} bytes, larger than the {FLASH_SIZE} byte flash",
            path.display(),
            image.len()
        )));
    }
    Ok(image)
}

/// Decodes the crystal from a KNLT firmware header; `None` when the bytes are
/// not a firmware header, so a blank target or a raw blob skips the guard.
fn header_xtal_mhz(header: &[u8]) -> Option<u8> {
    if header.len() < usize::from(FLASH_HEADER_LEN) {
        return None;
    }
    let version = u32::from_le_bytes([header[4], header[5], header[6], header[7]]);
    let magic = u32::from_le_bytes([header[8], header[9], header[10], header[11]]);
    if magic != KNLT_MAGIC {
        return None;
    }
    Some(if version & XTAL_BIT == 0 { 12 } else { 16 })
}

/// Erases every 4 KB sector covering `0..len`.
fn erase_image(transport: &mut dyn ProbeTransport, len: usize) -> Result<usize, CliError> {
    let sectors = len.div_ceil(FLASH_SECTOR_SIZE as usize);
    for sector in 0..sectors {
        let addr = sector as u32 * FLASH_SECTOR_SIZE;
        expect_ack(transport, Request::FlashEraseSector { addr })?;
    }
    Ok(sectors)
}

/// Programs `data` at `addr` in chunks that never cross a 256 B page.
pub(super) fn write_pages(
    transport: &mut dyn ProbeTransport,
    addr: u32,
    data: &[u8],
) -> Result<usize, CliError> {
    let mut pages = 0;
    let mut cursor = addr;
    let mut offset = 0;
    while offset < data.len() {
        let chunk = page_chunk_len(cursor, data.len() - offset);
        expect_ack(
            transport,
            Request::FlashWritePage {
                addr: cursor,
                data: data[offset..offset + chunk].iter().copied().collect(),
            },
        )?;
        offset += chunk;
        cursor += chunk as u32;
        pages += 1;
    }
    Ok(pages)
}

/// Longest write that stays inside the 256 B page containing `cursor`.
fn page_chunk_len(cursor: u32, remaining: usize) -> usize {
    let to_page_end = (FLASH_PAGE_SIZE - cursor % FLASH_PAGE_SIZE) as usize;
    remaining.min(FLASH_PAGE_SIZE as usize).min(to_page_end)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commands::test_support::*;
    use probe_proto::Response;

    #[test]
    fn flash_xtal_match_erases_and_writes_page_bounded_chunks() {
        let image = firmware_image(0x0000_0dd3, 600);
        let file = TempFile::new("flash-match", &image);
        let mut mock = crate::transport::MockTransport::new([
            Response::Ack,       // Reset (hold, stop)
            Response::Ack,       // SwsActivate
            header(0x0000_0dd3), // target header, 12 MHz
            Response::Ack,       // erase sector 0x000
            Response::Ack,       // page 0x000
            Response::Ack,       // page 0x100
            Response::Ack,       // page 0x200
        ]);

        let output = flash(&mut mock, file.path(), false, false).expect("flash succeeds");

        assert_eq!(
            output,
            Output::Flash {
                path: file.path().display().to_string(),
                bytes: 600,
                sectors_erased: 1,
                pages_written: 3,
                target_xtal_mhz: Some(12),
                image_xtal_mhz: Some(12),
            }
        );
        assert_eq!(
            mock.requests,
            [
                Request::Reset {
                    hold_ms: 2,
                    stop_cpu: true,
                },
                sws_activate_req(),
                read_flash_req(0, 12),
                erase_req(0),
                write_page_req(0, &image[..256]),
                write_page_req(0x100, &image[256..512]),
                write_page_req(0x200, &image[512..]),
            ]
        );
    }

    #[test]
    fn flash_erases_every_sector_covering_the_image() {
        let image = firmware_image(0x0000_0dd3, 0x1200);
        let file = TempFile::new("flash-sectors", &image);
        let pages = image.len().div_ceil(FLASH_PAGE_SIZE as usize);
        let mut responses = vec![Response::Ack, Response::Ack, header(0x0000_0dd3)];
        responses.extend(std::iter::repeat_n(Response::Ack, 2 + pages));
        let mut mock = crate::transport::MockTransport::new(responses);

        let output = flash(&mut mock, file.path(), false, false).expect("flash succeeds");

        let erased: Vec<u32> = mock
            .requests
            .iter()
            .filter_map(|req| match req {
                Request::FlashEraseSector { addr } => Some(*addr),
                _ => None,
            })
            .collect();
        assert_eq!(erased, [0x0000, 0x1000]);
        for req in &mock.requests {
            if let Request::FlashWritePage { addr, data } = req {
                assert!(
                    probe_proto::flash_page_valid(*addr, data.len() as u16),
                    "page write 0x{addr:x}+{} crosses a page",
                    data.len()
                );
            }
        }
        assert!(
            matches!(
                output,
                Output::Flash {
                    sectors_erased: 2,
                    pages_written: 18,
                    ..
                }
            ),
            "{output:?}"
        );
    }

    #[test]
    fn flash_xtal_mismatch_aborts_before_erase() {
        let image = firmware_image(0x8000_0dd3, 600); // 16 MHz
        let file = TempFile::new("flash-mismatch", &image);
        let mut mock = crate::transport::MockTransport::new([
            Response::Ack,
            Response::Ack,
            header(0x0000_0dd3), // target is 12 MHz
        ]);

        let err = flash(&mut mock, file.path(), false, false).expect_err("mismatch must abort");

        assert_eq!(
            err,
            CliError::CrystalMismatch {
                target_mhz: 12,
                image_mhz: 16,
            }
        );
        assert_eq!(err.exit_code(), 3);
        assert!(!flash_requests(&mock.requests), "{:?}", mock.requests);
        assert_eq!(
            mock.requests,
            [
                Request::Reset {
                    hold_ms: 2,
                    stop_cpu: true,
                },
                sws_activate_req(),
                read_flash_req(0, 12),
            ]
        );
    }

    #[test]
    fn flash_force_overrides_the_crystal_guard() {
        let image = firmware_image(0x8000_0dd3, 256); // 16 MHz
        let file = TempFile::new("flash-force", &image);
        let mut mock = crate::transport::MockTransport::new([
            Response::Ack,
            Response::Ack,
            header(0x0000_0dd3),
            Response::Ack,
            Response::Ack,
        ]);

        let output = flash(&mut mock, file.path(), true, false).expect("--force proceeds");

        assert_eq!(
            output,
            Output::Flash {
                path: file.path().display().to_string(),
                bytes: 256,
                sectors_erased: 1,
                pages_written: 1,
                target_xtal_mhz: Some(12),
                image_xtal_mhz: Some(16),
            }
        );
        assert!(flash_requests(&mock.requests));
    }

    #[test]
    fn flash_skips_the_guard_for_blank_targets_and_non_firmware_images() {
        let blank = data([0xff; FLASH_HEADER_LEN as usize]);
        let cases = [
            (
                "blank-target",
                blank,
                firmware_image(0x0000_0dd3, 300),
                None,
                Some(12),
            ),
            (
                "non-knlt-image",
                header(0x0000_0dd3),
                vec![0x5a; 300],
                Some(12),
                None,
            ),
            (
                "short-image",
                header(0x0000_0dd3),
                vec![0x5a; 4],
                Some(12),
                None,
            ),
        ];

        for (name, target, image, target_xtal, image_xtal) in cases {
            let file = TempFile::new(name, &image);
            let sectors = image.len().div_ceil(FLASH_SECTOR_SIZE as usize);
            let pages = image.len().div_ceil(FLASH_PAGE_SIZE as usize);
            let mut responses = vec![Response::Ack, Response::Ack, target];
            responses.extend(std::iter::repeat_n(Response::Ack, sectors + pages));
            let mut mock = crate::transport::MockTransport::new(responses);

            let output = flash(&mut mock, file.path(), false, false)
                .unwrap_or_else(|err| panic!("{name}: {err:?}"));

            assert_eq!(
                output,
                Output::Flash {
                    path: file.path().display().to_string(),
                    bytes: image.len(),
                    sectors_erased: sectors,
                    pages_written: pages,
                    target_xtal_mhz: target_xtal,
                    image_xtal_mhz: image_xtal,
                },
                "{name}"
            );
            assert!(flash_requests(&mock.requests), "{name}");
        }
    }

    #[test]
    fn flash_reset_flag_appends_a_release_reset() {
        let image = firmware_image(0x0000_0dd3, 256);
        let file = TempFile::new("flash-reset", &image);
        let mut mock = crate::transport::MockTransport::new([
            Response::Ack,
            Response::Ack,
            header(0x0000_0dd3),
            Response::Ack,
            Response::Ack,
            Response::Ack,
        ]);

        flash(&mut mock, file.path(), false, true).expect("flash --reset succeeds");

        assert_eq!(
            mock.requests.last(),
            Some(&Request::Reset {
                hold_ms: 2,
                stop_cpu: false,
            })
        );
    }

    #[test]
    fn flash_rejects_empty_oversized_and_malformed_images_before_any_request() {
        let empty = TempFile::new("flash-empty", &[]);
        let mut mock = crate::transport::MockTransport::new([]);
        let err = flash(&mut mock, empty.path(), false, false).expect_err("empty image fails");
        assert!(
            matches!(&err, CliError::Device(message) if message.contains("empty")),
            "{err:?}"
        );
        assert!(mock.requests.is_empty());

        let oversized = TempFile::new("flash-oversized", &vec![0u8; FLASH_SIZE as usize + 1]);
        let mut mock = crate::transport::MockTransport::new([]);
        let err = flash(&mut mock, oversized.path(), false, false).expect_err("oversized fails");
        assert!(
            matches!(&err, CliError::Device(message) if message.contains("larger")),
            "{err:?}"
        );
        assert!(mock.requests.is_empty());

        // The ELF magic selects the parser, so a broken ELF fails as such.
        let broken = TempFile::new("flash-broken-elf", b"\x7fELFnot an elf");
        let mut mock = crate::transport::MockTransport::new([]);
        let err = flash(&mut mock, broken.path(), false, false).expect_err("bad ELF fails");
        assert!(matches!(err, CliError::Elf(_)), "{err:?}");
        assert!(mock.requests.is_empty());
    }
}
