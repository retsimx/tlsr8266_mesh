//! RAM-resident boot-time OTA image copier.
//!
//! Runs before any task or interrupt exists, in `.ram_code`: the copy erases the
//! sectors holding the running `.text` and `.rodata`, so every callee reachable
//! from here must itself be RAM-resident.
//!
//! # Why the entry point is a wrapper over a scalar core
//!
//! A by-value struct that crosses an **externally visible** function boundary is
//! lowered as a pointer to a materialised copy. Making the `.ram_code` copier
//! itself the externally visible symbol (`OPT_PRESERVE_ROOTS`, so `opt` keeps it)
//! therefore promoted the six-field config into `.rodata` and passed its address
//! — and `.rodata` is inside the flash image this copy erases. The published
//! signature must stay `ota_boot_copy(OtaBootConfig { .. })`, so the aggregate is
//! unpacked at the call site by an `#[inline(always)]` wrapper and only scalars
//! ever reach the RAM-resident core. Do not "simplify" this back into a single
//! by-value `.ram_code` function: the ELF gate rejects the `.rodata` pointer it
//! produces.
//!
//! Three codegen hazards are deliberately avoided, and an ELF gate enforces the
//! result:
//!
//! * a by-value aggregate across the RAM boundary (see above), avoided by the
//!   wrapper/core split;
//! * the 256-byte page buffer is [`MaybeUninit`], never `[0u8; 256]`, so LLVM
//!   never emits a `memclr` call into `.text`;
//! * erase boundaries are tracked with a saturating `next_erase_addr` counter,
//!   never `%` or `/`, so LLVM never emits a division helper call into `.text`.

use core::mem::MaybeUninit;

use tlsr8266_hal::drivers::flash::{
    flash_erase_sector, flash_read_page, flash_write_page, PAGE_SIZE,
};
use tlsr8266_hal::mcu::irq_i::irq_disable;
use tlsr8266_hal::pm::{ota_enable_clocks_for_copy, ram_sw_reboot};

/// Everything the copier needs, passed by value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OtaBootConfig {
    /// Address of the "staged and ready" sentinel byte.
    pub ready_flag_addr: u32,
    /// Value that arms the copy.
    pub ready_flag_value: u8,
    /// Offset of the image-size word inside the staged header.
    pub header_bin_size_offset: u8,
    /// Start of the staging area.
    pub staging_addr: u32,
    /// Upper bound on one staged image.
    pub max_image_bytes: u32,
    /// Size of one erasable sector.
    pub sector_size: u32,
}

/// Stage tag: the pre-erase header page read.
const STAGE_HEADER: u8 = 1;
/// Stage tag: the in-loop page read.
const STAGE_COPY: u8 = 2;

/// Boot-copy entry point. Everything it needs arrives in `config`, by value.
///
/// `#[inline(always)]` is load-bearing: the scalar fields are unpacked at the
/// call site, so the RAM-resident core never receives a pointer to an aggregate
/// configuration. On this target a by-value struct crossing an externally
/// visible boundary is materialised in `.rodata` and passed by pointer — and
/// `.rodata` is inside the flash region this copy erases.
///
/// A no-op unless the ready sentinel is armed. Every abort path before the first
/// erase returns with the sentinel still armed, so the next boot retries the
/// (idempotent) copy; every abort path after it reboots immediately, because the
/// running image may already be partly erased.
#[inline(always)]
pub fn ota_boot_copy(config: OtaBootConfig) {
    ota_boot_copy_fields(
        config.ready_flag_addr,
        config.ready_flag_value,
        config.header_bin_size_offset,
        config.staging_addr,
        config.max_image_bytes,
        config.sector_size,
    );
}

/// The actual copier: scalars only, no aggregate, no flash reads.
///
/// This is the externally visible `.ram_code` symbol the ELF gate tracks; the
/// public [`ota_boot_copy`] wrapper above exists solely to keep the aggregate
/// out of it.
#[export_name = "tlsr8266_ota_boot_copy"]
#[inline(never)]
#[link_section = ".ram_code"]
fn ota_boot_copy_fields(
    ready_flag_addr: u32,
    ready_flag_value: u8,
    header_bin_size_offset: u8,
    staging_addr: u32,
    max_image_bytes: u32,
    sector_size: u32,
) {
    if !ready_flag_armed(ready_flag_addr, ready_flag_value) {
        return;
    }

    // Configure the system clock so the flash delays work, then stop interrupts
    // for the whole copy.
    ota_enable_clocks_for_copy();
    irq_disable();

    let mut page = MaybeUninit::<[u8; PAGE_SIZE as usize]>::uninit();
    let page_ptr = page.as_mut_ptr() as *mut u8;

    // First page of the staged image. A dead bus leaves nothing trustworthy to
    // copy, so this returns before sector zero is erased.
    if read_page(staging_addr, page_ptr, STAGE_HEADER).is_err() {
        return;
    }

    let size_offset = header_bin_size_offset as usize;
    if size_offset + 4 > PAGE_SIZE as usize {
        // The header field cannot live in the first page.
        return;
    }

    // SAFETY: the read above returned `Ok`, so every one of the `PAGE_SIZE`
    // bytes behind `page` is initialised; the slice is a `[u8; PAGE_SIZE]` of
    // plain integers and is only read here.
    let header = unsafe { page.assume_init_ref() };
    let new_fw_size = u32::from_le_bytes([
        header[size_offset],
        header[size_offset + 1],
        header[size_offset + 2],
        header[size_offset + 3],
    ]);

    // Pre-erase guard: an impossible size must not cost the running image its
    // first sector.
    if new_fw_size == 0 || new_fw_size > max_image_bytes || sector_size == 0 {
        return;
    }

    let mut current_addr: u32 = 0;
    let mut next_erase_addr: u32 = 0;
    while current_addr < new_fw_size {
        // Clear the next sector when the copy reaches its boundary. The counter
        // is the division-free form of `current_addr % sector_size == 0`.
        if current_addr >= next_erase_addr {
            if erase_sector(current_addr).is_err() {
                reboot_and_retry_copy();
            }
            next_erase_addr = next_erase_addr.saturating_add(sector_size);
        }

        // Re-read the page from the staged image. From here on a failed read
        // leaves `page` untrustworthy, so the copy stops here and the sentinel
        // survives for the retry after the next reset.
        if read_page(
            staging_addr.saturating_add(current_addr),
            page_ptr,
            STAGE_COPY,
        )
        .is_err()
        {
            reboot_and_retry_copy();
        }

        if write_page(current_addr, page_ptr as *const u8).is_err() {
            reboot_and_retry_copy();
        }

        current_addr = current_addr.saturating_add(PAGE_SIZE);
    }

    // Disarm the sentinel with a one-byte scalar, never through the page
    // buffer. A stall here is harmless: the copy is complete, so the next boot
    // re-runs the same idempotent copy.
    let zero: u8 = 0;
    let _ = flash_write_page(ready_flag_addr, 1, core::ptr::addr_of!(zero));

    ram_sw_reboot();
}

/// Is the "staged and ready" sentinel armed?
///
/// Takes scalars, not the aggregate: this is inlined into the RAM-resident core,
/// which must never re-form the config struct for an ABI hand-off.
///
/// Always inlined: the copier is `.ram_code` and must not take a `.text` call
/// edge for a one-byte test.
#[inline(always)]
fn ready_flag_armed(ready_flag_addr: u32, ready_flag_value: u8) -> bool {
    #[cfg(target_os = "none")]
    {
        // SAFETY: (2) boot OTA sentinel read. `ready_flag_addr` is a fixed byte
        // address in the memory-mapped flash window; it is read through a raw
        // pointer before any staged code runs, no reference to it is created,
        // and the read is volatile so it cannot be folded away.
        let value = unsafe { core::ptr::read_volatile(ready_flag_addr as *const u8) };
        value == ready_flag_value
    }

    #[cfg(not(target_os = "none"))]
    {
        let mut value: u8 = 0;
        if flash_read_page(ready_flag_addr, 1, core::ptr::addr_of_mut!(value)).is_err() {
            return false;
        }
        value == ready_flag_value
    }
}

/// Read one page of the staged image into the copier's buffer.
#[inline(always)]
fn read_page(addr: u32, buf: *mut u8, stage: u8) -> Result<(), ()> {
    #[cfg(not(target_os = "none"))]
    if host_fault::take(stage) {
        return Err(());
    }
    let _ = stage;
    flash_read_page(addr, PAGE_SIZE, buf)
}

/// Erase the sector containing `addr`.
#[inline(always)]
fn erase_sector(addr: u32) -> Result<(), ()> {
    #[cfg(not(target_os = "none"))]
    if host_fault::take(host_fault::ERASE) {
        return Err(());
    }
    flash_erase_sector(addr)
}

/// Program one page of the running image.
#[inline(always)]
fn write_page(addr: u32, buf: *const u8) -> Result<(), ()> {
    #[cfg(not(target_os = "none"))]
    if host_fault::take(host_fault::WRITE) {
        return Err(());
    }
    flash_write_page(addr, PAGE_SIZE, buf)
}

/// Abort the copy by rebooting with the sentinel still armed.
///
/// The sentinel is the recovery contract: the next boot re-runs the idempotent
/// copy instead of resuming on a partly erased image. The parking stays in RAM
/// for the same reason the copier does — the running image below the cursor may
/// already be the new one.
#[inline(never)]
#[link_section = ".ram_code"]
fn reboot_and_retry_copy() -> ! {
    ram_sw_reboot()
}

/// Host-only fault injector for the copier's failure branches.
///
/// The platform crate may not name the register crate (and may not depend on
/// it), so the in-process register fake is unreachable from here. The three
/// flash failures are therefore injected at this crate's own call boundary; on
/// device this module does not exist and the copier calls the HAL directly.
#[cfg(not(target_os = "none"))]
mod host_fault {
    use core::sync::atomic::{AtomicU8, Ordering};

    /// Fail the first sector erase.
    pub(super) const ERASE: u8 = 4;
    /// Fail the first page program.
    pub(super) const WRITE: u8 = 8;

    static MASK: AtomicU8 = AtomicU8::new(0);

    /// Arm one or more one-shot failures.
    #[cfg(test)]
    pub(super) fn arm(mask: u8) {
        MASK.store(mask, Ordering::Relaxed);
    }

    /// Disarm every failure.
    #[cfg(test)]
    pub(super) fn clear() {
        MASK.store(0, Ordering::Relaxed);
    }

    /// Consume `stage` if it is armed.
    pub(super) fn take(stage: u8) -> bool {
        MASK.fetch_and(!stage, Ordering::Relaxed) & stage != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::panic::{catch_unwind, AssertUnwindSafe};

    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::clock::fake_clock;

    const READY_FLAG_ADDR: u32 = 0x3f000;
    const READY_FLAG: u8 = 0xa5;
    const STAGING: u32 = 0x40000;
    const SECTOR: u32 = 0x1000;
    const SIZE_OFFSET: u8 = 0x18;

    const CONFIG: OtaBootConfig = OtaBootConfig {
        ready_flag_addr: READY_FLAG_ADDR,
        ready_flag_value: READY_FLAG,
        header_bin_size_offset: SIZE_OFFSET,
        staging_addr: STAGING,
        max_image_bytes: 128 * 1024,
        sector_size: SECTOR,
    };

    fn reset() {
        FakeFlash::reset();
        fake_clock::reset();
        host_fault::clear();
    }

    fn arm_flag() {
        FakeFlash::write_slice(READY_FLAG_ADDR, &[READY_FLAG]);
    }

    fn flag_byte() -> u8 {
        let mut value = [0u8; 1];
        FakeFlash::read_slice(READY_FLAG_ADDR, &mut value);
        value[0]
    }

    /// Seed a staged image whose header advertises `size`, and return the bytes
    /// that were staged.
    fn stage_image(size: u32) -> Vec<u8> {
        let mut image: Vec<u8> = (0..size).map(|i| (i % 251) as u8).collect();
        if image.len() >= SIZE_OFFSET as usize + 4 {
            image[SIZE_OFFSET as usize..SIZE_OFFSET as usize + 4]
                .copy_from_slice(&size.to_le_bytes());
        } else {
            // A zero-size image still carries its (zero) size word.
            let mut word = [0u8; 4];
            word.copy_from_slice(&size.to_le_bytes());
            FakeFlash::write_slice(STAGING + SIZE_OFFSET as u32, &word);
        }
        if !image.is_empty() {
            FakeFlash::write_slice(STAGING, &image);
        }
        image
    }

    /// Run the copier, returning the panic message when it took the reboot path.
    ///
    /// Off target `ram_sw_reboot` latches the reset and then panics, so the
    /// panic message is the observable that proves the reboot primitive ran.
    fn run_copier() -> Option<String> {
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let result = catch_unwind(AssertUnwindSafe(|| ota_boot_copy(CONFIG)));
        std::panic::set_hook(previous);

        match result {
            Ok(()) => None,
            Err(payload) => Some(
                payload
                    .downcast_ref::<&str>()
                    .map(|text| (*text).to_owned())
                    .or_else(|| payload.downcast_ref::<String>().cloned())
                    .unwrap_or_else(|| "non-string panic".to_owned()),
            ),
        }
    }

    fn assert_rebooted(reason: Option<String>) {
        assert_eq!(
            reason.as_deref(),
            Some("ram_sw_reboot: host build observed the MCU reset latch"),
            "the copier must abort through the RAM reboot primitive"
        );
    }

    fn assert_returned(reason: Option<String>) {
        assert!(
            reason.is_none(),
            "this path must return, not reboot: {reason:?}"
        );
    }

    #[test]
    fn unarmed_flag_is_a_no_op() {
        reset();

        assert_returned(run_copier());

        assert!(FakeFlash::erase_history().is_empty());
        assert_eq!(flag_byte(), 0xff, "an unarmed sentinel is left alone");
    }

    #[test]
    fn pre_erase_header_read_failure_returns_with_the_flag_armed() {
        reset();
        arm_flag();
        stage_image(512);
        host_fault::arm(STAGE_HEADER);

        assert_returned(run_copier());

        assert_eq!(flag_byte(), READY_FLAG, "the retry flag must survive");
        assert!(
            FakeFlash::erase_history().is_empty(),
            "nothing may be erased before the header is readable"
        );
    }

    #[test]
    fn zero_size_does_not_erase_live_firmware() {
        reset();
        arm_flag();
        stage_image(0);

        assert_returned(run_copier());

        assert_eq!(flag_byte(), READY_FLAG);
        assert!(FakeFlash::erase_history().is_empty());
    }

    #[test]
    fn oversized_image_does_not_erase_live_firmware() {
        reset();
        arm_flag();
        stage_image(128 * 1024 + PAGE_SIZE);

        assert_returned(run_copier());

        assert_eq!(flag_byte(), READY_FLAG);
        assert!(FakeFlash::erase_history().is_empty());
    }

    #[test]
    fn erase_failure_reboots_with_the_flag_still_armed() {
        reset();
        arm_flag();
        stage_image(512);
        host_fault::arm(host_fault::ERASE);

        assert_rebooted(run_copier());

        assert_eq!(flag_byte(), READY_FLAG, "the retry flag must survive");
        assert!(
            FakeFlash::erase_history().is_empty(),
            "the failed erase never reached the device"
        );
    }

    #[test]
    fn post_erase_read_failure_reboots_with_the_flag_still_armed() {
        reset();
        arm_flag();
        stage_image(512);
        host_fault::arm(STAGE_COPY);

        assert_rebooted(run_copier());

        assert_eq!(flag_byte(), READY_FLAG, "the retry flag must survive");
        assert_eq!(
            FakeFlash::erase_history(),
            vec![0],
            "sector zero is erased before the copy read fails"
        );
    }

    #[test]
    fn write_failure_reboots_with_the_flag_still_armed() {
        reset();
        arm_flag();
        stage_image(512);
        host_fault::arm(host_fault::WRITE);

        assert_rebooted(run_copier());

        assert_eq!(flag_byte(), READY_FLAG, "the retry flag must survive");
        assert_eq!(
            FakeFlash::erase_history(),
            vec![0],
            "the erase happens before the page program fails"
        );
    }

    #[test]
    fn full_multi_sector_copy_clears_the_flag_and_reboots() {
        reset();
        arm_flag();
        let size = SECTOR + 2 * PAGE_SIZE + 16;
        let image = stage_image(size);

        assert_rebooted(run_copier());

        assert_eq!(flag_byte(), 0x00, "a complete copy disarms the sentinel");
        assert_eq!(
            FakeFlash::erase_history(),
            vec![0, SECTOR],
            "both destination sectors are erased in copy order"
        );

        let mut copied = vec![0u8; size as usize];
        FakeFlash::read_slice(0, &mut copied);
        assert_eq!(copied, image, "the staged image is copied byte-exact");
    }

    #[test]
    fn copy_is_idempotent_when_rerun_with_a_cleared_flag() {
        reset();
        arm_flag();
        let size = 512;
        stage_image(size);

        assert_rebooted(run_copier());
        assert_eq!(flag_byte(), 0x00);

        // A second boot must not copy again.
        let erases_before = FakeFlash::erase_history().len();
        assert_returned(run_copier());
        assert_eq!(FakeFlash::erase_history().len(), erases_before);
    }
}
