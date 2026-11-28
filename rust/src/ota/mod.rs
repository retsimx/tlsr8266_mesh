//! Product-local OTA boot entry point.
//!
//! All OTA state, policy, staging, verification and recovery live in
//! `tlsr8266_platform::ota`. This module only supplies the by-value boot
//! configuration and the `.ram_code` entry point the reset stub calls.

pub use tlsr8266_platform::ota::flash_layout;

use tlsr8266_platform::ota::{ota_boot_copy, OtaBootConfig};

/// Boot copy configuration, materialised as immediates before any erase.
///
/// Deliberately a `const`, never a `static`: a static would land in `.rodata`,
/// which the copy itself erases, so the config must reach `ota_boot_copy` by
/// value before sector zero is gone.
pub const LIGHT_OTA_BOOT_CONFIG: OtaBootConfig = OtaBootConfig {
    ready_flag_addr: 0x3F000,
    ready_flag_value: 0xA5,
    header_bin_size_offset: flash_layout::HEADER_BIN_SIZE_OFFSET,
    staging_addr: crate::config::FLASH_ADR_LIGHT_NEW_FW,
    max_image_bytes: 128 * 1024,
    sector_size: crate::config::FLASH_SECTOR_SIZE as u32,
};

// The by-value boot configuration must agree with the published flash layout
// contract, or the copier would look for the sentinel in the wrong sector.
const _: () =
    assert!(LIGHT_OTA_BOOT_CONFIG.ready_flag_addr == flash_layout::FLASH_ADR_OTA_READY_FLAG);
const _: () = assert!(LIGHT_OTA_BOOT_CONFIG.ready_flag_value == flash_layout::FLASH_OTA_READY_FLAG);
const _: () = assert!(LIGHT_OTA_BOOT_CONFIG.max_image_bytes == flash_layout::FW_SIZE_MAX_K * 1024);

/// Apply a staged firmware image from the OTA area and reboot into it.
///
/// Runs at boot before any task or interrupt exists, so it takes no lock at all
/// and must stay fully RAM-resident: flash reads/writes need the RAM code path.
#[inline(never)]
#[link_section = ".ram_code"]
// Boot-only `.ram_code` entrypoint: it needs a staged image in flash and a
// reset to run, so it cannot execute on the host.
#[coverage(off)]
pub fn handle_ota_update() {
    ota_boot_copy(LIGHT_OTA_BOOT_CONFIG);
}
