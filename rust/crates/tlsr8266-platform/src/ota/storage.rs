//! Flash staging, read-back verification and boot-time recovery for OTA.
//!
//! Every routine here is a port of the firmware's OTA flash layer, with the
//! product's flash layout passed in as [`OtaFlashConfig`]. Nothing in this file
//! assumes a particular product: the defaults are the deployed F-3 layout.

use mesh_protocol::mesh_types::{OtaState, START_UP_FLAG};
use tlsr8266_hal::common::bit::ONES_32;
use tlsr8266_hal::drivers::flash::{flash_erase_sector, flash_read_page, flash_write_page};

use super::engine;

/// The deployed staging-area contract.
///
/// These values are shared with the host tooling and asserted against the
/// compatibility fixture; changing one is a wire/flash contract change.
pub mod flash_layout {
    /// Flash address of the "firmware staged and ready to copy" sentinel byte.
    pub const FLASH_ADR_OTA_READY_FLAG: u32 = 0x3F000;
    /// Sentinel written to [`FLASH_ADR_OTA_READY_FLAG`] once an image is staged.
    pub const FLASH_OTA_READY_FLAG: u8 = 0xa5;
    /// Upper bound on the staged image size, in KiB.
    pub const FW_SIZE_MAX_K: u32 = 128;
    /// Sectors erased before a staged image is copied out.
    pub const ERASE_SECTORS_FOR_OTA: u32 = FW_SIZE_MAX_K.div_ceil(4);
    /// Offset of the image-size word inside the staged header.
    pub const HEADER_BIN_SIZE_OFFSET: u8 = 0x18;
}

/// Flash address where a staged image starts.
///
/// Kept private: the OTA area is reached through the config, never by a literal
/// at a call site.
const STAGING_ADDR: u32 = 0x40000;
/// Size of one erasable sector.
const SECTOR_SIZE: u32 = 0x1000;
/// Offset of the boot flag word inside the staged header.
const HEADER_STARTUP_FLAG_OFFSET: u32 = 8;
/// The one boot-flag byte the staged header must carry.
const STARTUP_FLAG_BYTE: u8 = 0x4b;
/// Largest payload one chunk can carry (GATT bridge chunks are 16 bytes).
const MAX_CHUNK_BYTES: usize = 0x10;

/// Product-neutral description of the OTA staging area.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OtaFlashConfig {
    /// Address of the "staged and ready" sentinel byte.
    pub ready_flag_addr: u32,
    /// Value that arms the boot-time copy.
    pub ready_flag_value: u8,
    /// Offset of the image-size word inside the staged header.
    pub header_bin_size_offset: u8,
    /// Start of the staging area.
    pub staging_addr: u32,
    /// Upper bound on one staged image.
    pub max_image_bytes: u32,
    /// Size of one erasable sector.
    pub sector_size: u32,
    /// Boot flag word the staged header must carry.
    pub startup_flag: u32,
}

impl OtaFlashConfig {
    /// The deployed F-3 layout.
    pub const fn compatibility() -> Self {
        Self {
            ready_flag_addr: flash_layout::FLASH_ADR_OTA_READY_FLAG,
            ready_flag_value: flash_layout::FLASH_OTA_READY_FLAG,
            header_bin_size_offset: flash_layout::HEADER_BIN_SIZE_OFFSET,
            staging_addr: STAGING_ADDR,
            max_image_bytes: flash_layout::FW_SIZE_MAX_K * 1024,
            sector_size: SECTOR_SIZE,
            startup_flag: START_UP_FLAG,
        }
    }

    /// Sectors the staging area spans; zero when the config is unusable.
    const fn sector_count(self) -> u32 {
        if self.sector_size == 0 {
            0
        } else {
            self.max_image_bytes.div_ceil(self.sector_size)
        }
    }
}

impl Default for OtaFlashConfig {
    fn default() -> Self {
        Self::compatibility()
    }
}

/// Erase the whole staging area.
///
/// Sectors are erased last-to-first, matching the deployed order: a failure
/// leaves a non-erased sector behind, so [`is_ota_area_valid`] rejects the area
/// and the next boot erases again.
pub fn erase_staging_area() {
    let config = OtaFlashConfig::compatibility();
    let sectors = config.sector_count();
    for i in 0..sectors {
        let _ = flash_erase_sector(config.staging_addr + (sectors - 1 - i) * config.sector_size);
    }
}

/// Is the staging area in its erased state?
///
/// `Ok(true)` — every probed word reads `0xFFFFFFFF`; `Ok(false)` — it holds
/// staged data; `Err(())` — the bus failed.
// The project convention for fallible hardware calls is `Result<_, ()>`: the
// only failure is a bus timeout, so a bespoke error type would carry no
// information.
#[allow(clippy::result_unit_err)]
pub fn is_ota_area_valid() -> Result<bool, ()> {
    let config = OtaFlashConfig::compatibility();
    let mut buf = [0u8; 4];
    for i in 0..config.sector_count() {
        flash_read_page(
            config.staging_addr + i * config.sector_size,
            4,
            buf.as_mut_ptr(),
        )?;
        let word =
            buf[0] as u32 | (buf[1] as u32) << 8 | (buf[2] as u32) << 16 | (buf[3] as u32) << 24;
        if word != ONES_32 {
            return Ok(false);
        }
    }

    Ok(true)
}

/// Program one verified chunk at the write cursor.
///
/// Writes `data` at the cursor, reads it straight back and only advances the
/// cursor when the read-back matches. A failed verification returns
/// [`OtaState::Error`] and leaves the cursor where it was, which is what makes a
/// retransmitted packet rewrite the same address. Every call costs one write
/// plus one read of `data.len()` bytes in the staging area and contributes to
/// flash wear.
pub fn save_chunk_verified(data: &[u8]) -> OtaState {
    if data.len() > MAX_CHUNK_BYTES {
        // No transport carries this much; refuse instead of overrunning the
        // verification buffer.
        return OtaState::Error;
    }

    let config = OtaFlashConfig::compatibility();
    let addr = config.staging_addr + engine::current_flash_address();

    // A failed page program leaves the staging bytes unchanged, and the
    // read-back verification below reports `OtaState::Error` for it just as it
    // does for a corrupted write.
    let _ = flash_write_page(addr, data.len() as u32, data.as_ptr());

    let mut verify = [0u8; MAX_CHUNK_BYTES];
    if flash_read_page(addr, data.len() as u32, verify.as_mut_ptr()).is_err() {
        // A dead bus cannot prove the program landed, so the packet is reported
        // as a failed verification and retransmitted.
        return OtaState::Error;
    }

    if data == &verify[..data.len()] {
        engine::advance_flash_address(data.len() as u32);
        OtaState::Continue
    } else {
        OtaState::Error
    }
}

/// Read the staged image's advertised size from its header.
// See `is_ota_area_valid`: the only failure is a bus timeout.
#[allow(clippy::result_unit_err)]
pub fn read_staged_bin_size() -> Result<u32, ()> {
    let config = OtaFlashConfig::compatibility();
    let mut total = [0u8; 4];
    flash_read_page(
        config.staging_addr + config.header_bin_size_offset as u32,
        4,
        total.as_mut_ptr(),
    )?;
    Ok(u32::from_le_bytes(total))
}

/// Fix up the staged firmware flag and arm the boot-time ready sentinel.
///
/// Any failed read or write simply returns: the peer retransmits the final
/// header, so a half-armed image is never claimed as ready.
pub fn arm_ready_flag() {
    let config = OtaFlashConfig::compatibility();
    let flag_addr = config.staging_addr + HEADER_STARTUP_FLAG_OFFSET;

    let mut flag_new: u32 = 0;
    if flash_read_page(flag_addr, 4, core::ptr::addr_of_mut!(flag_new) as *mut u8).is_err() {
        return; // invalid flag
    }
    flag_new &= 0xffff_ff4b;
    if flag_new != config.startup_flag {
        return; // invalid flag
    }

    let mut flag0: u8 = 0;
    if flash_read_page(flag_addr, 1, core::ptr::addr_of_mut!(flag0)).is_err() {
        return; // invalid flag
    }
    if STARTUP_FLAG_BYTE != flag0 {
        flag0 = STARTUP_FLAG_BYTE;
        // A failure here is retried by the next staged transfer; the ready flag
        // below is only armed after this point.
        let _ = flash_write_page(flag_addr, 1, core::ptr::addr_of!(flag0));
    }

    // The staged image may come from a different product family, so force the
    // one boot-flag byte the startup code requires.
    //
    // A failed erase leaves the sentinel byte un-erased, so the flag write below
    // keeps the old value and the staged image is not armed this time.
    let _ = flash_erase_sector(config.ready_flag_addr);
    let flag: u32 = config.ready_flag_value as u32;
    // A failed flag write leaves the sentinel unarmed rather than claiming a
    // copy that never starts; the transfer is retried by the peer.
    let _ = flash_write_page(
        config.ready_flag_addr,
        4,
        core::ptr::addr_of!(flag) as *const u8,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;

    fn reset_fakes() {
        FakeFlash::reset();
        engine::reset_for_test();
    }

    #[test]
    fn staging_area_validates_writes_and_erase() {
        reset_fakes();

        assert_eq!(is_ota_area_valid(), Ok(true));
        FakeFlash::write_slice(STAGING_ADDR, &[0x00]);
        assert_eq!(is_ota_area_valid(), Ok(false));
        erase_staging_area();
        assert_eq!(is_ota_area_valid(), Ok(true));

        reset_fakes();
    }

    #[test]
    fn erase_covers_every_sector_last_to_first() {
        reset_fakes();

        erase_staging_area();

        let sectors = flash_layout::ERASE_SECTORS_FOR_OTA;
        assert_eq!(FakeFlash::erase_count(STAGING_ADDR), 1);
        assert_eq!(
            FakeFlash::erase_count(STAGING_ADDR + (sectors - 1) * SECTOR_SIZE),
            1
        );
        let history = FakeFlash::erase_history();
        assert_eq!(history.len() as u32, sectors);
        assert_eq!(history[0], STAGING_ADDR + (sectors - 1) * SECTOR_SIZE);
        assert_eq!(history[sectors as usize - 1], STAGING_ADDR);

        reset_fakes();
    }

    #[test]
    fn save_chunk_programs_and_verifies_then_advances_the_cursor() {
        reset_fakes();

        assert_eq!(save_chunk_verified(&[1, 2, 3, 4]), OtaState::Continue);
        let mut got = [0u8; 4];
        FakeFlash::read_slice(STAGING_ADDR, &mut got);
        assert_eq!(got, [1, 2, 3, 4]);
        assert_eq!(engine::current_flash_address(), 4);

        assert_eq!(save_chunk_verified(&[5, 6]), OtaState::Continue);
        FakeFlash::read_slice(STAGING_ADDR + 4, &mut got[..2]);
        assert_eq!(got[..2], [5, 6]);
        assert_eq!(engine::current_flash_address(), 6);

        reset_fakes();
    }

    #[test]
    fn failed_verification_keeps_the_cursor_for_retransmit() {
        reset_fakes();
        engine::set_current_flash_address(8);
        FakeFlash::fail_verify_next_write();

        assert_eq!(save_chunk_verified(&[9, 9, 9, 9]), OtaState::Error);
        assert_eq!(
            engine::current_flash_address(),
            8,
            "a failed verification must leave the retransmit address alone"
        );

        reset_fakes();
    }

    #[test]
    fn oversized_chunk_is_refused_without_touching_flash() {
        reset_fakes();

        assert_eq!(
            save_chunk_verified(&[0; MAX_CHUNK_BYTES + 1]),
            OtaState::Error
        );
        assert_eq!(engine::current_flash_address(), 0);

        reset_fakes();
    }

    #[test]
    fn bin_size_reads_the_header_word() {
        reset_fakes();
        FakeFlash::write_slice(
            STAGING_ADDR + flash_layout::HEADER_BIN_SIZE_OFFSET as u32,
            &0x1234_5678u32.to_le_bytes(),
        );
        assert_eq!(read_staged_bin_size(), Ok(0x1234_5678));

        reset_fakes();
    }

    #[test]
    fn ready_flag_is_armed_only_for_a_valid_staged_header() {
        reset_fakes();
        FakeFlash::write_slice(
            STAGING_ADDR + HEADER_STARTUP_FLAG_OFFSET,
            &START_UP_FLAG.to_le_bytes(),
        );

        arm_ready_flag();

        let mut ready = [0u8; 1];
        FakeFlash::read_slice(flash_layout::FLASH_ADR_OTA_READY_FLAG, &mut ready);
        assert_eq!(ready[0], flash_layout::FLASH_OTA_READY_FLAG);
        assert_eq!(
            FakeFlash::erase_count(flash_layout::FLASH_ADR_OTA_READY_FLAG),
            1,
            "the sentinel sector is erased before it is written"
        );
        let mut header = [0u8; 1];
        FakeFlash::read_slice(STAGING_ADDR + HEADER_STARTUP_FLAG_OFFSET, &mut header);
        assert_eq!(header[0], STARTUP_FLAG_BYTE);

        reset_fakes();
    }

    #[test]
    fn ready_flag_stays_unarmed_for_an_invalid_header() {
        reset_fakes();
        // Header word that the mask/compare rejects.
        FakeFlash::write_slice(
            STAGING_ADDR + HEADER_STARTUP_FLAG_OFFSET,
            &0xdead_beefu32.to_le_bytes(),
        );

        arm_ready_flag();

        let mut ready = [0u8; 1];
        FakeFlash::read_slice(flash_layout::FLASH_ADR_OTA_READY_FLAG, &mut ready);
        assert_eq!(ready[0], 0xff, "an unstaged sentinel stays erased");
        assert_eq!(
            FakeFlash::erase_count(flash_layout::FLASH_ADR_OTA_READY_FLAG),
            0
        );

        reset_fakes();
    }

    #[test]
    fn compatibility_config_pins_the_deployed_layout() {
        let config = OtaFlashConfig::compatibility();
        assert_eq!(config.ready_flag_addr, 0x3f000);
        assert_eq!(config.ready_flag_value, 0xa5);
        assert_eq!(config.staging_addr, 0x40000);
        assert_eq!(config.max_image_bytes, 128 * 1024);
        assert_eq!(config.sector_size, 4096);
        assert_eq!(config.startup_flag, 0x544c_4e4b);
        assert_eq!(config.header_bin_size_offset, 0x18);
        assert_eq!(config.sector_count(), 32);
    }
}
