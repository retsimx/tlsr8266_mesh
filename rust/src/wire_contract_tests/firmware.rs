//! `firmware` namespace: the 32-byte OTA image header and the flash layout the
//! OTA means to write.

use super::Parser;
use crate::config::FLASH_ADR_LIGHT_NEW_FW;
use crate::ota::flash_layout::{FLASH_ADR_OTA_READY_FLAG, FLASH_OTA_READY_FLAG};

/// The `ota.header` vector plus the two flash-layout constants, read without
/// asserting.
pub(super) struct FirmwareHeader {
    header: Vec<u8>,
    reset_opcode: Vec<u8>,
    ramcode_word: Vec<u8>,
    irq_opcode: Vec<u8>,
    ready_flag_address: u32,
    ready_flag: u8,
    staging_address: u32,
}

pub(super) fn read_firmware_header(p: &mut Parser) -> FirmwareHeader {
    FirmwareHeader {
        header: p.bytes("ota.header"),
        reset_opcode: p.bytes("ota.header.reset_opcode"),
        ramcode_word: p.bytes("ota.header.ramcode_word"),
        irq_opcode: p.bytes("ota.header.irq_opcode"),
        ready_flag_address: p.u32("ota.flash_adr_ota_ready_flag"),
        ready_flag: p.u8("ota.flash_ota_ready_flag"),
        staging_address: p.u32("ota.flash_adr_light_new_fw"),
    }
}

fn check_firmware_header(p: &mut Parser) {
    let vectors = read_firmware_header(p);
    let header = &vectors.header;

    assert_eq!(
        header.len(),
        32,
        "`ota.header` must be the first 32 bytes of the image"
    );

    assert_eq!(
        &header[0..2],
        vectors.reset_opcode.as_slice(),
        "`ota.header` must start with `ota.header.reset_opcode`"
    );

    let version = u32::from_le_bytes([header[2], header[3], header[4], header[5]]);
    assert_eq!(
        version & 0x8000_0000,
        0,
        "the header version word must have bit 31 clear"
    );
    assert_eq!(
        crate::version::BUILD_VERSION & 0x8000_0000,
        0,
        "the live BUILD_VERSION must have bit 31 clear"
    );

    assert_eq!(
        &header[6..8],
        &[0u8, 0u8],
        "header[6..8] must be the .org 0x8 gap"
    );
    assert_eq!(
        &header[8..12],
        &[0x4b, 0x4e, 0x4c, 0x54],
        "header magic 0x544c4e4b must land on flash as `KNLT`"
    );

    assert_eq!(
        &header[12..16],
        vectors.ramcode_word.as_slice(),
        "`ota.header` must carry `ota.header.ramcode_word`"
    );
    assert_eq!(
        &header[16..18],
        vectors.irq_opcode.as_slice(),
        "`ota.header` must carry `ota.header.irq_opcode`"
    );
    assert_eq!(
        &header[18..24],
        &[0u8; 6],
        "header[18..24] must be the .org 0x18 gap"
    );

    let bin_size = u32::from_le_bytes([header[24], header[25], header[26], header[27]]);
    assert!(
        bin_size > 0,
        "header[24..28] must carry a non-zero image size"
    );
    assert_eq!(
        bin_size % 4,
        0,
        "the image size word must stay word-aligned"
    );

    assert_eq!(
        FLASH_ADR_OTA_READY_FLAG, vectors.ready_flag_address,
        "`ota.flash_adr_ota_ready_flag` mismatch"
    );
    assert_eq!(
        FLASH_OTA_READY_FLAG, vectors.ready_flag,
        "`ota.flash_ota_ready_flag` mismatch"
    );
    assert_eq!(
        FLASH_ADR_LIGHT_NEW_FW, vectors.staging_address,
        "`ota.flash_adr_light_new_fw` mismatch"
    );
}

#[test]
fn fixture_firmware_header() {
    check_firmware_header(&mut Parser::new());
}
