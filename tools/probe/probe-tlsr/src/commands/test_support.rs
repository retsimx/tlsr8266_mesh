//! Fixtures shared by the command tests.
//!
//! Every command module asserts exact request sequences, so the request
//! builders, canned replies and temp files live here once instead of being
//! copied into each test module.

use std::path::{Path, PathBuf};

use probe_proto::{CpuMode, Request, Response};

use super::{FLASH_HEADER_LEN, HEADER_VERSION_OFFSET, KNLT_MAGIC, SWS_DIV};
use crate::Ctx;

pub fn sws_activate_req() -> Request {
    Request::SwsActivate { div: SWS_DIV }
}

pub fn read_mem_req(addr: u16, len: u16) -> Request {
    Request::ReadMem { addr, len }
}

pub fn read_flash_req(addr: u32, len: u16) -> Request {
    Request::FlashRead { addr, len }
}

pub fn write_mem_req(addr: u16, bytes: &[u8]) -> Request {
    Request::WriteMem {
        addr,
        data: bytes.iter().copied().collect(),
    }
}

pub fn erase_req(addr: u32) -> Request {
    Request::FlashEraseSector { addr }
}

pub fn write_page_req(addr: u32, bytes: &[u8]) -> Request {
    Request::FlashWritePage {
        addr,
        data: bytes.iter().copied().collect(),
    }
}

pub fn stop_req() -> Request {
    Request::CpuCtrl {
        mode: CpuMode::Stop,
    }
}

pub fn data(bytes: impl IntoIterator<Item = u8>) -> Response {
    Response::Data(bytes.into_iter().collect())
}

pub fn stopped(status: u8) -> Response {
    data([status])
}

/// A 12-byte header with `version` and the KNLT magic.
pub fn header(version: u32) -> Response {
    let mut bytes = vec![0u8; usize::from(FLASH_HEADER_LEN)];
    bytes[HEADER_VERSION_OFFSET..HEADER_VERSION_OFFSET + 4].copy_from_slice(&version.to_le_bytes());
    bytes[8..12].copy_from_slice(&KNLT_MAGIC.to_le_bytes());
    data(bytes)
}

pub fn jedec_id() -> Response {
    Response::JedecId([0xef, 0x40, 0x18])
}

pub fn pc_bytes() -> Response {
    data([0xf0, 0x10, 0x01, 0x00])
}

pub fn test_ctx(responses: impl IntoIterator<Item = Response>) -> Ctx {
    Ctx {
        transport: Box::new(crate::transport::MockTransport::new(responses)),
        elf: None,
        elf_path: None,
        json: false,
        timeout_ms: None,
    }
}

/// A firmware image with a KNLT header at `version`, filled with a
/// recognisable payload so page slices are easy to assert.
pub fn firmware_image(version: u32, len: usize) -> Vec<u8> {
    assert!(len >= usize::from(FLASH_HEADER_LEN));
    let mut image = vec![0x5a_u8; len];
    image[HEADER_VERSION_OFFSET..HEADER_VERSION_OFFSET + 4].copy_from_slice(&version.to_le_bytes());
    image[8..12].copy_from_slice(&KNLT_MAGIC.to_le_bytes());
    image
}

/// 12-byte firmware headers taken byte-for-byte from real images.
///
/// These are the first 12 bytes of `lightblemesh` builds. The version/crystal
/// word lives at bytes `2..6` because `sdk/cstartup_8266.S` emits a 2-byte
/// `tj __reset` at address 0. Do not "tidy" these back to offset 4.
pub const REAL_12MHZ_HEADER: [u8; 12] = [
    0x0e, 0x80, 0xd4, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x4b, 0x4e, 0x4c, 0x54,
];
pub const REAL_16MHZ_HEADER: [u8; 12] = [
    0x0e, 0x80, 0xd5, 0x0d, 0x00, 0x80, 0x00, 0x00, 0x4b, 0x4e, 0x4c, 0x54,
];

/// Temp file removed when the test ends (or unwinds).
pub struct TempFile(PathBuf);

impl TempFile {
    pub fn new(name: &str, bytes: &[u8]) -> Self {
        let path =
            std::env::temp_dir().join(format!("probe-tlsr-{name}-{}.bin", std::process::id()));
        std::fs::write(&path, bytes).expect("temp fixture writes");
        Self(path)
    }

    pub fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}

pub fn flash_requests(requests: &[Request]) -> bool {
    requests.iter().any(|req| {
        matches!(
            req,
            Request::FlashEraseSector { .. } | Request::FlashWritePage { .. }
        )
    })
}
