//! Fixtures shared by the command tests.
//!
//! Every command module asserts exact request sequences, so the request
//! builders, canned replies and temp files live here once instead of being
//! copied into each test module.

use std::path::{Path, PathBuf};

use probe_proto::{CpuMode, Request, Response};

use super::{FLASH_HEADER_LEN, KNLT_MAGIC, SWS_DIV};
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
    bytes[4..8].copy_from_slice(&version.to_le_bytes());
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
        json: false,
    }
}

/// A firmware image with a KNLT header at `version`, filled with a
/// recognisable payload so page slices are easy to assert.
pub fn firmware_image(version: u32, len: usize) -> Vec<u8> {
    assert!(len >= usize::from(FLASH_HEADER_LEN));
    let mut image = vec![0x5a_u8; len];
    image[4..8].copy_from_slice(&version.to_le_bytes());
    image[8..12].copy_from_slice(&KNLT_MAGIC.to_le_bytes());
    image
}

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
