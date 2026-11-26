//! The OTA subsystem: every piece of OTA state and logic lives here.
//!
//! This module owns the staged-image flash routines ([`handle_ota_update`],
//! [`rf_ota_save_data`], [`rf_link_slave_data_ota_save`]), the runtime scalars
//! ([`OtaRuntimeState`] plus the module statics), the timeout countdown
//! ([`on_1s_tick`]) and the LED/reboot finish path.
//!
//! # Boundaries
//!
//! `ota` sits above `sdk` in the layering graph, so it may not name a single
//! SDK module. Two boot-registered seams carry the traffic instead:
//!
//! * [`OtaRadioOps`] — installed by `app.rs`; supplies the packet staging
//!   buffer, the GATT-OTA begin gate and the `PKT_TERMINATE` transmit.
//! * `sdk::ble_app::callbacks::OtaHooks` — points the other way, at
//!   [`on_gatt_ota_packet`], [`ota_status_snapshot_impl`],
//!   [`ota_request_termination_impl`] and [`ota_tick_timeout_1s_impl`].
//!
//! # Chunk validation
//!
//! [`fsm::evaluate_chunk`] decides what one staged packet means; the save path
//! only executes the decision. The decision table and its tests are pure.

use core::cell::Cell;
use core::mem::size_of_val;
use core::ptr::addr_of;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use critical_section::Mutex;

use crate::config::{FLASH_ADR_LIGHT_NEW_FW, FLASH_SECTOR_SIZE, OTA_LED};
use mesh_protocol::crc::crc16;
use mesh_protocol::mesh_types::{
    OtaState, OtaStatusSnapshot, RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS, START_UP_FLAG,
};
use mesh_protocol::packet_types::{Packet, PacketAttData};
use tlsr8266_hal::common::bit::ONES_32;
use tlsr8266_hal::common::sync::CriticalSectionMutex;
use tlsr8266_hal::common::sync::SimplifyLS;
use tlsr8266_hal::drivers::flash::{
    flash_erase_sector, flash_read_page, flash_write_page, PAGE_SIZE,
};
use tlsr8266_hal::mcu::clock::{
    clock_time, clock_time64, clock_time_exceed, sleep_us, CLOCK_SYS_CLOCK_1S,
};
use tlsr8266_hal::mcu::gpio::{gpio_set_func, gpio_set_output_en, gpio_write, AS_GPIO};
use tlsr8266_hal::mcu::irq_i::irq_disable;
use tlsr8266_hal::mcu::watchdog::wd_clear;
use tlsr8266_hal::pm::light_sw_reboot;

mod fsm;

use fsm::OtaChunkDecision;

/// The mutable OTA runtime scalars shared by the GATT write handler, the mesh
/// OTA command path and the system-timer interrupt.
///
/// Everything that blocks (flash erase/write/read, the LED delay loop, the RF
/// terminate transmission) is deliberately kept *outside* this type: the owners
/// above take the [`OTA_STATE`] lock just long enough to move these four values
/// in or out, never across blocking I/O. See [`rf_link_slave_data_ota_save`] for
/// the pattern.
struct OtaRuntimeState {
    /// Index of the last OTA packet accepted in the current transfer.
    ota_rcv_last_idx: u16,
    /// Number of packets currently staged in the shared response buffer.
    slave_ota_data_cache_idx: usize,
    /// `clock_time()` stamp taken when the transfer-finish flag was set, used by
    /// the reboot timeout.
    rf_slave_ota_finished_time: u32,
    /// Remaining `PKT_TERMINATE` notifications before the OTA reboot is allowed.
    terminate_cnt: u8,
}

impl OtaRuntimeState {
    /// Boot values, matching `App::default_const()` before the ownership split.
    pub const fn new() -> Self {
        Self {
            ota_rcv_last_idx: 0xffff,
            slave_ota_data_cache_idx: 0,
            rf_slave_ota_finished_time: 0,
            terminate_cnt: 0,
        }
    }
}

impl Default for OtaRuntimeState {
    fn default() -> Self {
        Self::new()
    }
}

/// Sole owner of the OTA runtime scalars.
static OTA_STATE: CriticalSectionMutex<OtaRuntimeState> =
    CriticalSectionMutex::new(OtaRuntimeState::new());

/// Transfer-completion flag, read by the BLE link layer through
/// [`ota_status_snapshot_impl`].
static RF_SLAVE_OTA_FINISHED_FLAG: CriticalSectionMutex<OtaState> =
    CriticalSectionMutex::new(OtaState::Continue);

/// Set while a *mesh* OTA is receiving, which selects the 8-byte payload size and
/// suppresses mesh forwarding.
static OTA_UPDATE_MESH_OPERATIONS_BLOCKED: AtomicBool = AtomicBool::new(false);

/// Set while a *BLE* OTA is receiving.
static OTA_UPDATE_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

/// Write cursor into the staging area, relative to [`FLASH_ADR_LIGHT_NEW_FW`].
static OTA_UPDATE_CURRENT_FLASH_ADDRESS: AtomicU32 = AtomicU32::new(0);

/// Set once the initiator asked for `PKT_TERMINATE` notifications.
static OTA_UPDATE_TERMINATION_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Remaining OTA supervision timeout, in seconds.
static OTA_UPDATE_TIMEOUT_SECONDS: AtomicU16 = AtomicU16::new(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);

/// Highest index accepted by the mesh OTA command path.
static LAST_INDEX: AtomicU16 = AtomicU16::new(u16::MAX);

/// Timestamp of the last `TMR0` interrupt at which the OTA timeout was
/// decremented.
///
/// `TMR0` is reprogrammed dynamically for embassy timers, so it no longer ticks
/// at a fixed 1 Hz. Stamping the tick here lets [`on_1s_tick`] count real
/// seconds.
static LAST_OTA_TIMEOUT_TICK: CriticalSectionMutex<u64> = CriticalSectionMutex::new(0);

/// Flash addresses and sizes shared by the staging path and the boot copier.
///
/// Part of the wire/flash contract: `rust/src/wire_contract_tests/mod.rs` asserts
/// these four constants against `tests/fixtures/wire_vectors.json`.
pub mod flash_layout {
    /// Flash address of the "firmware staged and ready to copy" sentinel byte.
    pub const FLASH_ADR_OTA_READY_FLAG: u32 = 0x3F000;
    /// Sentinel written to [`FLASH_ADR_OTA_READY_FLAG`] once an image is staged.
    pub const FLASH_OTA_READY_FLAG: u8 = 0xa5;
    /// Upper bound on the staged image size, in KiB.
    pub const FW_SIZE_MAX_K: u32 = 128;
    /// Sectors erased before a staged image is copied out.
    pub const ERASE_SECTORS_FOR_OTA: u32 = FW_SIZE_MAX_K.div_ceil(4);
}

/// Slots in the shared response buffer, and therefore the most GATT OTA packets
/// a single drain can stage.
///
/// Must match `tlsr8266_radio::types::BUFF_RESPONSE_PACKET_COUNT`. `ota` may not
/// name the radio crate (the layering gate forbids it), so the equality is
/// asserted at compile time in `app.rs`, which owns both sides.
pub const OTA_STAGING_SLOTS: usize = 16;

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
    // This function requires that *everything* be in ram
    // SAFETY: (2) boot-OTA flag read. `flash_layout::FLASH_ADR_OTA_READY_FLAG` is
    // a fixed byte address in the flash window, read as one `u8` before any code
    // from the staged image runs; no reference to it is created.
    unsafe {
        if *(flash_layout::FLASH_ADR_OTA_READY_FLAG as *const u8)
            != flash_layout::FLASH_OTA_READY_FLAG
        {
            // Nothing to do
            return;
        }
    }

    // First configure the system clock. This lets delays work as expected for reading/writing
    // flash
    tlsr8266_hal::pm::ota_enable_clocks_for_copy();
    irq_disable();

    // Read the first page of the new firmware
    let mut buff: [u8; PAGE_SIZE as usize] = [0; PAGE_SIZE as usize];
    if flash_read_page(FLASH_ADR_LIGHT_NEW_FW, PAGE_SIZE, buff.as_mut_ptr()).is_err() {
        // A dead bus leaves nothing trustworthy to copy: return before the ready
        // flag is cleared so the retry flag survives for the next reset.
        return;
    }

    // Get the size of the binary being copied
    // SAFETY: (2) boot-OTA image header. `buff` is the 256-byte page just read from
    // the staged image, so the 4-byte word at offset 0x18 (the `_bin_size_` field
    // of the header contract in `startup.rs`) is in bounds. The load needs a
    // 4-byte-aligned page; the 256-byte TC32 frame slot provides that alignment.
    let new_fw_size = unsafe { *(buff.as_ptr().offset(0x18) as *const u32) };

    // Fail-safe contract: a bus stall reboots *before* the ready flag is cleared,
    // so the retry flag survives and the copy is attempted again on the next
    // boot. Returning instead would resume `main_entrypoint` on a partly erased
    // image and leave recovery to an external power cycle. Clearing the flag
    // after a partial copy would leave a blank running image with no way back
    // except a SWire reflash.
    let mut current_addr = 0;
    while current_addr < new_fw_size {
        // Check if we need to clear the next sector
        if current_addr % FLASH_SECTOR_SIZE as u32 == 0 && flash_erase_sector(current_addr).is_err()
        {
            reboot_and_retry_copy();
        }

        // Read the page at this address from the new firmware. A failed read
        // leaves `buff` untrustworthy, so the copy stops here and the ready flag
        // survives for the retry after the next reset.
        if flash_read_page(
            FLASH_ADR_LIGHT_NEW_FW + current_addr,
            PAGE_SIZE,
            buff.as_mut_ptr(),
        )
        .is_err()
        {
            return;
        }

        // Write the page to the real firmware
        if flash_write_page(current_addr, PAGE_SIZE, buff.as_mut_ptr()).is_err() {
            reboot_and_retry_copy();
        }

        current_addr += PAGE_SIZE;
    }

    // Clear the OTA ready flag
    buff[0] = 0;
    // The copy already succeeded, so a stall here is harmless: the flag stays
    // armed and the next boot re-runs this same (idempotent) copy.
    let _ = flash_write_page(flash_layout::FLASH_ADR_OTA_READY_FLAG, 1, buff.as_mut_ptr());

    // Force reboot
    tlsr8266_hal::pm::mcu_trigger_reboot();

    #[allow(clippy::empty_loop)]
    loop {}
}

/// Abort the boot-time copy by rebooting with the ready flag still armed.
///
/// The flag is the recovery contract: the next boot re-runs the idempotent copy
/// instead of resuming `main_entrypoint` on a partly erased image, so no
/// external reset is needed. `-> !` also keeps the caller off the resume path on
/// host builds, where [`light_sw_reboot`] returns instead of holding the reset.
///
/// Safe to call from the `.ram_code` copier: both abort sites run after the flash
/// driver released chip select and returned, so the flash-resident reboot path
/// can be fetched. The parking loop stays in RAM for the same reason the copier
/// does — the running image above `current_addr` may already be the new image.
#[inline(never)]
#[link_section = ".ram_code"]
fn reboot_and_retry_copy() -> ! {
    light_sw_reboot();
    #[allow(clippy::empty_loop)]
    loop {}
}

/// Check if OTA area is valid; if not or on error, erase it.
pub fn check_ota_area_startup() {
    if let Ok(false) | Err(()) = is_ota_area_valid() {
        erase_ota_data();
    }
}

/// Validates the OTA firmware area by checking if it's in a clean state.
///
/// Performs hardware flash I/O to read OTA area sectors. Returns:
/// - `Ok(true)` if OTA area is valid (all sectors contain 0xFFFFFFFF)
/// - `Ok(false)` if OTA area is invalid (contains non-erased data)
/// - `Err(())` if hardware I/O error occurs
fn is_ota_area_valid() -> Result<bool, ()> {
    let mut buf: [u8; 4] = [0; 4];
    for i in 0..flash_layout::ERASE_SECTORS_FOR_OTA {
        flash_read_page(FLASH_ADR_LIGHT_NEW_FW + i * 0x1000, 4, buf.as_mut_ptr())?;
        let tmp =
            buf[0] as u32 | (buf[1] as u32) << 8 | (buf[2] as u32) << 16 | (buf[3] as u32) << 24;
        if tmp != ONES_32 {
            return Ok(false);
        }
    }

    Ok(true)
}

/// Erase every sector of the OTA staging area.
fn erase_ota_data() {
    for i in 0..flash_layout::ERASE_SECTORS_FOR_OTA {
        // A sector that fails to erase still reads non-erased, so
        // `is_ota_area_valid` rejects the area and the next boot erases again.
        let _ = flash_erase_sector(
            FLASH_ADR_LIGHT_NEW_FW + (flash_layout::ERASE_SECTORS_FOR_OTA - 1 - i) * 0x1000,
        );
    }
}

// --- radio ops: the staging/RF surface `ota` must not import ------------------

/// The hardware-facing operations `ota` needs but may not own.
///
/// Every slot is a plain `fn` pointer so the struct is `Copy` and can be read
/// out of the critical section in one go. Ownership runs `app.rs` → `ota`: the
/// providers live above `ota` in the layering graph, so the composition root
/// installs them once before `rf_drv_init(true)`. A shim that ran first would
/// panic in `radio_ops_or_panic` rather than lose an OTA packet silently.
#[derive(Copy, Clone)]
pub struct OtaRadioOps {
    /// May a GATT OTA begin now? Pairing gate plus status-read stop.
    pub try_begin_gatt_ota: fn() -> bool,
    /// Copy one packet into the shared staging buffer at `idx`; the provider owns
    /// the 16-slot wrap.
    pub stage_packet: fn(usize, &Packet),
    /// Read one staged packet back out at exactly `idx`.
    pub read_staged_packet: fn(usize) -> Packet,
    /// Queue one `PKT_TERMINATE`; `true` only when it was queued.
    pub try_send_terminate_packet: fn() -> bool,
}

#[cfg(not(test))]
static OTA_RADIO_OPS: Mutex<Cell<Option<OtaRadioOps>>> = Mutex::new(Cell::new(None));

#[cfg(test)]
static OTA_RADIO_OPS: Mutex<Cell<Option<OtaRadioOps>>> =
    Mutex::new(Cell::new(Some(TEST_RADIO_OPS)));

/// Registers the radio operations.
///
/// Called once from `app.rs` before `rf_drv_init(true)`.
#[inline(never)]
pub fn set_ota_radio_ops(ops: OtaRadioOps) {
    critical_section::with(|cs| OTA_RADIO_OPS.borrow(cs).set(Some(ops)));
}

/// Reads the registered radio operations, or panics if registration never
/// happened.
#[inline(never)]
fn radio_ops_or_panic() -> OtaRadioOps {
    match critical_section::with(|cs| OTA_RADIO_OPS.borrow(cs).get()) {
        Some(ops) => ops,
        None => panic!("OTA radio ops are not registered"),
    }
}

/// `#[inline(never)]` on every shim is load-bearing: `sdk/stack_analysis.py`'s
/// indirect-edge table binds the slot name to this symbol, and
/// `OPT_INLINE_THRESHOLD=50` would otherwise fold a one-call shim into its
/// caller and erase the symbol from the ELF.
#[inline(never)]
fn radio_begin_gatt_ota() -> bool {
    (radio_ops_or_panic().try_begin_gatt_ota)()
}

/// Stages one packet at `idx`, unwrapped.
///
/// The provider (`stage_ota_packet`, which owns the 16-slot `BUFF_RESPONSE`)
/// applies `idx % 16`, so the wrap has exactly one source of truth. The read
/// side keeps the legacy unwrapped indexing.
#[inline(never)]
fn radio_stage_pkt(idx: usize, p: &Packet) {
    (radio_ops_or_panic().stage_packet)(idx, p);
}

/// Reads a staged packet at exactly `idx`; the legacy read never wrapped.
#[inline(never)]
fn radio_read_pkt(idx: usize) -> Packet {
    (radio_ops_or_panic().read_staged_packet)(idx)
}

/// Queues one `PKT_TERMINATE`; `true` only when it really was queued.
#[inline(never)]
fn radio_terminate_tx() -> bool {
    (radio_ops_or_panic().try_send_terminate_packet)()
}

/// GATT OTA write handler.
///
/// Stages one packet in the shared response buffer; the cache cursor moves in
/// and out of [`OTA_STATE`] under a short critical section, so no lock is held
/// while the packet is copied. A full window drops the packet instead of
/// claiming a slot past [`OTA_STAGING_SLOTS`].
///
/// [`radio_begin_gatt_ota`] only reports whether a GATT OTA may start — the
/// OTA flags stay owned here, which is why there is no `start_gatt_ota` entry
/// point for the provider to call.
#[inline(never)]
pub fn on_gatt_ota_packet(data: &Packet) {
    if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue
        || OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get()
    {
        return;
    }

    if !OTA_UPDATE_IN_PROGRESS.get() {
        if !radio_begin_gatt_ota() {
            return;
        }

        OTA_UPDATE_IN_PROGRESS.set(true);
        // A new session starts over at index 0, so the mesh path's
        // monotonicity cursor must not survive the previous transfer.
        LAST_INDEX.store(u16::MAX, Ordering::Relaxed);
        tlsr8266_hal::pm::set_reboot_ota_active(true);
    }

    // Claim the next staging slot under a short critical section, then drop the
    // guard before touching the staging buffer (a second, independent lock).
    // The window holds at most `OTA_STAGING_SLOTS` packets: once it is full the
    // packet has nowhere to go, so it is dropped rather than indexing the
    // provider's buffer out of bounds.
    let cache_idx = {
        let mut st = OTA_STATE.lock();
        let cache_idx = st.slave_ota_data_cache_idx;
        if cache_idx >= OTA_STAGING_SLOTS {
            return;
        }
        st.slave_ota_data_cache_idx = cache_idx + 1;
        cache_idx
    };

    radio_stage_pkt(cache_idx, data);
}

/// Mesh OTA data command handler.
///
/// Returns the index the packet was staged under, so a caller can acknowledge
/// what it actually sent. The last accepted index comes back when the packet is
/// refused as stale, and `u16::MAX` when it never reached the staging path (the
/// mesh path does not own the transfer). A staged packet that then aborts the
/// transfer still answers with its own index: the finish funnel has already
/// cleared the cursor, so re-reading it would answer for the abort instead of
/// for this packet. Every [`OTA_STATE`] access is a short snapshot or store; the
/// persistence work happens in [`rf_link_slave_data_ota_save`].
pub fn rf_mesh_data_ota(pkt_data: &[u8], last: bool) -> u16 {
    if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue
        || OTA_UPDATE_IN_PROGRESS.get()
        || !OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get()
    {
        return u16::MAX;
    }

    let mut data: PacketAttData = PacketAttData::default();
    data.head.l2cap_len = if last { 7 } else { 10 + 7 };
    data.dat[0..pkt_data.len()].copy_from_slice(pkt_data);

    let index = data.dat[0] as u16 | ((data.dat[1] as u16) << 8);

    let last_index = LAST_INDEX.load(Ordering::Relaxed);
    if last_index == u16::MAX || last_index < index {
        LAST_INDEX.store(index, Ordering::Relaxed);

        radio_stage_pkt(0, &Packet { att_data: data });

        {
            let mut st = OTA_STATE.lock();
            st.slave_ota_data_cache_idx = 1;
        }

        rf_link_slave_data_ota_save();

        OTA_UPDATE_TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS); // refresh timeout

        if *RF_SLAVE_OTA_FINISHED_FLAG.lock() != OtaState::Continue {
            OTA_UPDATE_TIMEOUT_SECONDS.set(4);
        }

        // Report the index this packet was staged under. The finish funnel
        // clears the static on an abort so the *next* session starts at index 0;
        // re-reading it here would report the abort, not the packet.
        return index;
    }

    last_index
}

/// Save one OTA firmware packet to flash memory with verification.
///
/// Writes `data` at the current cursor, reads it straight back and only advances
/// the cursor when the read-back matches. A failed verification returns
/// [`OtaState::Error`] and leaves the cursor where it was, which is what makes a
/// retransmitted packet rewrite the same address.
///
/// Every call costs one write plus one read of `data.len()` bytes in the
/// staging area and contributes to flash wear.
fn rf_ota_save_data(data: &[u8]) -> OtaState {
    let addr = OTA_UPDATE_CURRENT_FLASH_ADDRESS.get() + FLASH_ADR_LIGHT_NEW_FW;

    // A failed page program leaves the staging bytes unchanged, and the
    // read-back verification below reports `OtaState::Error` for it just as it
    // does for a corrupted write.
    let _ = flash_write_page(addr, data.len() as u32, data.as_ptr());

    // Verification buffer; the largest packet this path carries is 16 bytes.
    let mut tmp = [0u8; 0x10];

    if flash_read_page(addr, data.len() as u32, tmp.as_mut_ptr()).is_err() {
        // A dead bus cannot prove the program landed, so the packet is reported
        // as a failed verification and retransmitted.
        return OtaState::Error;
    }

    if data == &tmp[..data.len()] {
        OTA_UPDATE_CURRENT_FLASH_ADDRESS
            .set(OTA_UPDATE_CURRENT_FLASH_ADDRESS.get() + data.len() as u32);
        OtaState::Continue
    } else {
        OtaState::Error
    }
}

/// Persist the staged packets.
///
/// # Critical-section invariant
///
/// [`OTA_STATE`] is locked twice, briefly: once to snapshot the staged-packet
/// count and the accepted-packet cursor, and once to publish the updated cursor.
/// The CRC pass, `flash_read_page` and `rf_ota_save_data` (which erase/write
/// flash) all run with **no** lock held.
fn rf_link_slave_data_ota_save() {
    let packet_len = if OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get() {
        8
    } else {
        16
    };

    // Short critical section: copy the two cursors out, then drop the guard.
    let (cache_len, mut ota_rcv_last_idx) = {
        let st = OTA_STATE.lock();
        (st.slave_ota_data_cache_idx, st.ota_rcv_last_idx)
    };
    // The provider owns the wrap, but the count is only ever read back through
    // `radio_read_pkt`, which does not: clamp it to the window that exists.
    let cache_len = cache_len.min(OTA_STAGING_SLOTS);

    for i in 0..cache_len {
        // Copy the staged packet out; the staging guard is a temporary that dies
        // at the end of this statement.
        let p = radio_read_pkt(i);

        // `l2cap_len` is peer-controlled: the payload length is `l2cap_len - 7`
        // and the trailing CRC pair sits at `n_data_len + 2..n_data_len + 4`, so
        // both bounds are established before any `dat[..]` index. `dat` is a
        // 23-byte array.
        let l2cap_len = p.head().l2cap_len as usize;
        let n_data_len = l2cap_len.saturating_sub(7);
        let frame_ok = l2cap_len >= 7 && n_data_len + 4 <= p.att_data().dat.len();

        // Every arm below assigns the finish state, so it is declared without an
        // initial value: a pre-loop `Continue` would be provably dead.
        let reset_flag: OtaState;

        if !frame_ok {
            // A malformed frame is a CRC-level failure: abort exactly like one.
            OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
            ota_rcv_last_idx = 0;
            reset_flag = OtaState::Error;
        } else if OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get()
            || crc16(&p.att_data().dat[0..n_data_len + 2])
                == p.att_data().dat[n_data_len + 2] as u16
                    | (p.att_data().dat[n_data_len + 3] as u16) << 8
        {
            OTA_UPDATE_TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS); // refresh timeout

            let cur_idx = p.att_data().dat[0] as u16 | (p.att_data().dat[1] as u16) << 8;
            // The decision table mirrors the four legacy branches exactly.
            // `RejectError` stands for four distinct legacy arms
            // (`HEAD:rust/src/ota.rs`) and only three of them zero the write
            // cursor:
            //   * a header at an unexpected index (`:330-334`) — zeroes;
            //   * index 0 on top of a non-empty cursor (`:337-345`) — zeroes;
            //   * an index gap, i.e. `cur_idx != last + 1` (`:360-366`) — zeroes;
            //   * the size-bound overrun on the `last + 1` path (`:348-358`) —
            //     leaves the cursor alone.
            // The other two legacy asymmetries are kept too: a failed
            // `WriteFirstChunk`/`WriteNextChunk` (flash verification error) does
            // not reset the cursor, so a retransmit rewrites the same address, and
            // the successful final-header path does not touch it either.
            let decision = fsm::evaluate_chunk(
                cur_idx,
                ota_rcv_last_idx,
                n_data_len,
                OTA_UPDATE_CURRENT_FLASH_ADDRESS.get(),
                packet_len,
                flash_layout::FW_SIZE_MAX_K * 1024,
            );
            reset_flag = match decision {
                OtaChunkDecision::WriteFirstChunk | OtaChunkDecision::WriteNextChunk => {
                    let flag = rf_ota_save_data(&p.att_data().dat[2..2 + packet_len as usize]);
                    ota_rcv_last_idx = cur_idx;
                    flag
                }
                OtaChunkDecision::VerifyFinalHeader { expected_index } => {
                    let mut total = [0u8; 4];
                    // Blocking flash read: no OTA_STATE guard is held here. A
                    // failed read gives no size to verify, so the transfer fails
                    // and the cursor returns to the start for a fresh attempt.
                    if flash_read_page(FLASH_ADR_LIGHT_NEW_FW + 0x18, 4, total.as_mut_ptr())
                        .is_err()
                    {
                        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
                        ota_rcv_last_idx = 0;
                        OtaState::Error
                    } else {
                        let ota_pkt_total = u32::from_le_bytes(total).div_ceil(packet_len) as u16;
                        if ota_pkt_total < 3 || ota_pkt_total != expected_index {
                            // invalid fw
                            OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
                            ota_rcv_last_idx = 0;
                            OtaState::Error
                        } else {
                            // ota ok, save, reboot
                            OtaState::Ok
                        }
                    }
                }
                OtaChunkDecision::RejectError => {
                    // error, ota failed. `ota_rcv_last_idx` is published as
                    // `cur_idx` for every data packet, but only the size-bound
                    // overrun leaves the write cursor alone (legacy
                    // `HEAD:rust/src/ota.rs:348-358`); the gap, index-0 and bad
                    // header arms all zero it (`:330-334`, `:337-345`, `:360-366`).
                    let size_bound_rejection = n_data_len != 0
                        && cur_idx != 0
                        && cur_idx == ota_rcv_last_idx.wrapping_add(1);
                    if !size_bound_rejection {
                        OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
                    }
                    ota_rcv_last_idx = if n_data_len == 0 { 0 } else { cur_idx };
                    OtaState::Error
                }
            };
        } else {
            // error, ota failed
            OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
            ota_rcv_last_idx = 0;
            reset_flag = OtaState::Error;
        }

        if reset_flag != OtaState::Continue {
            // Copy the flag out: the guard must not span the flag-set path,
            // which takes the same lock again.
            let finished = *RF_SLAVE_OTA_FINISHED_FLAG.lock();
            if finished == OtaState::Continue {
                rf_slave_ota_finished_flag_set(reset_flag);
            }

            break;
        }
    }

    // Short critical section: publish the cursors, then drop the guard.
    {
        let mut st = OTA_STATE.lock();
        st.ota_rcv_last_idx = ota_rcv_last_idx;
        st.slave_ota_data_cache_idx = 0;
    }
}

/// Fix up the firmware flag of a staged image and arm the OTA ready flag.
fn rf_ota_set_flag() {
    let fw_flag_telink = START_UP_FLAG;
    let mut flag_new = 0;
    // A failed read leaves the staged header untrusted, so the image is not
    // armed; the transfer is retried by the peer.
    if flash_read_page(
        FLASH_ADR_LIGHT_NEW_FW + 8,
        size_of_val(&flag_new) as u32,
        addr_of!(flag_new) as *mut u8,
    )
    .is_err()
    {
        return; // invalid flag
    }
    flag_new &= 0xffffff4b;
    if flag_new != fw_flag_telink {
        return; // invalid flag
    }

    let mut flag0 = 0;
    if flash_read_page(FLASH_ADR_LIGHT_NEW_FW + 8, 1, addr_of!(flag0) as *mut u8).is_err() {
        return; // invalid flag
    }
    if 0x4b != flag0 {
        flag0 = 0x4b;
        // The flag rewrite is re-attempted by the next staged transfer if it
        // fails; the ready flag below is only armed after this point.
        let _ = flash_write_page(FLASH_ADR_LIGHT_NEW_FW + 8, 1, addr_of!(flag0) as *mut u8);
        //Set FW flag, make sure valid. because the firmware may be from 8267 by mesh ota
    }

    // A failed erase leaves the sentinel byte un-erased, so the flag write below
    // keeps the old value and the staged image is not armed this time.
    let _ = flash_erase_sector(flash_layout::FLASH_ADR_OTA_READY_FLAG);
    let flag: u32 = flash_layout::FLASH_OTA_READY_FLAG as u32;
    // A failed flag write leaves the sentinel unarmed rather than claiming a
    // copy that never starts; the transfer is retried by the peer.
    let _ = flash_write_page(
        flash_layout::FLASH_ADR_OTA_READY_FLAG,
        4,
        addr_of!(flag) as *mut u8,
    );
}

/// Signal a successful OTA with the LED pattern.
fn rf_led_ota_ok() {
    tlsr8266_hal::pm::blink_ota_led_ok(OTA_LED as u32);
}

/// Signal a failed OTA by blinking the LED: 60 toggles of 100 ms.
///
/// The `sleep_us` delay loop runs with no lock held (there is nothing to lock).
fn rf_led_ota_error() {
    gpio_set_func(OTA_LED as u32, AS_GPIO);
    gpio_set_output_en(OTA_LED as u32, 1);
    let mut led_on = true;
    for _ in 0..60 {
        gpio_write(OTA_LED as u32, u32::from(led_on));
        led_on = !led_on;
        wd_clear();
        sleep_us(100 * 1000);
    }
}

/// Finish the OTA transfer: persist the flag / erase the staging area, give the
/// LED feedback, then reboot.
pub fn rf_link_slave_ota_finish_led_and_reboot(st: OtaState) {
    match st {
        OtaState::Error => {
            erase_ota_data();
            rf_led_ota_error();
        }
        OtaState::Ok => {
            //rf_ota_save_data(0);
            rf_ota_set_flag();
            rf_led_ota_ok();
        }
        _ => (),
    }
    irq_disable();
    light_sw_reboot();
}

/// Publish the transfer-finish flag and stamp the time it was set.
///
/// Also the single abort/finish funnel for the mesh-OTA monotonicity cursor: the
/// next session has to be able to send index 0 again, so the cursor goes back to
/// its boot value here.
fn rf_slave_ota_finished_flag_set(reset_flag: OtaState) {
    let finished_time = clock_time();
    *RF_SLAVE_OTA_FINISHED_FLAG.lock() = reset_flag;
    LAST_INDEX.store(u16::MAX, Ordering::Relaxed);

    let mut st = OTA_STATE.lock();
    st.rf_slave_ota_finished_time = finished_time;
}

/// Poll when an OTA transfer is busy (bridge/connection path).
///
/// Every [`OTA_STATE`] touch is a short snapshot or decrement, so the RF
/// terminate transmission and the LED/erase/reboot path never run under the
/// lock.
pub fn rf_link_slave_ota_finish_handle() {
    rf_link_slave_data_ota_save();

    // Copy the flag out: the guard must not span the handlers below, which take
    // the same lock again.
    let finished = *RF_SLAVE_OTA_FINISHED_FLAG.lock();
    if finished != OtaState::Continue {
        let mut reboot_flag = false;

        let terminate_idle = {
            let st = OTA_STATE.lock();
            st.terminate_cnt == 0
        };
        if terminate_idle && OTA_UPDATE_TERMINATION_REQUESTED.get() {
            // Blocking RF transmit: no OTA_STATE guard held. The counter is armed
            // only when the packet really was queued, so a full buffer neither
            // arms the countdown nor reboots.
            if radio_terminate_tx() {
                let mut st = OTA_STATE.lock();
                st.terminate_cnt = 6;
            }
        }

        {
            let mut st = OTA_STATE.lock();
            if st.terminate_cnt != 0 {
                st.terminate_cnt -= 1;
                if st.terminate_cnt == 0 {
                    reboot_flag = true;
                }
            }
        }

        let rf_slave_ota_finished_time = {
            let st = OTA_STATE.lock();
            st.rf_slave_ota_finished_time
        };

        if !OTA_UPDATE_TERMINATION_REQUESTED.get()
            && clock_time_exceed(rf_slave_ota_finished_time, 2000 * 1000)
        {
            OTA_UPDATE_TERMINATION_REQUESTED.set(true); // for ios: no last read command
        }

        if clock_time_exceed(rf_slave_ota_finished_time, 4000 * 1000) {
            reboot_flag = true;
        }

        if reboot_flag {
            // Copy the state out first: the guard must die before the erase /
            // LED / reboot path runs.
            let finished_state = *RF_SLAVE_OTA_FINISHED_FLAG.lock();
            rf_link_slave_ota_finish_led_and_reboot(finished_state);
            // have been rebooted
        }
    }
}

// --- hooks: the ble_app-facing control surface -------------------------------

/// Project the OTA flags for a layer that may not own them.
#[inline(never)]
pub fn ota_status_snapshot_impl() -> OtaStatusSnapshot {
    OtaStatusSnapshot {
        in_progress: OTA_UPDATE_IN_PROGRESS.get(),
        finish_status: *RF_SLAVE_OTA_FINISHED_FLAG.lock(),
        timeout_expired: OTA_UPDATE_TIMEOUT_SECONDS.get() == 0,
    }
}

/// Ask the finish path to send its `PKT_TERMINATE` notifications.
#[inline(never)]
pub fn ota_request_termination_impl() {
    OTA_UPDATE_TERMINATION_REQUESTED.set(true);
}

/// One second of OTA supervision timeout, driven by the connection event.
///
/// Returns `true` when the counter reads zero after this second.
#[inline(never)]
pub fn ota_tick_timeout_1s_impl() -> bool {
    if OTA_UPDATE_TIMEOUT_SECONDS.get() != 0 {
        OTA_UPDATE_TIMEOUT_SECONDS.dec();
    }
    OTA_UPDATE_TIMEOUT_SECONDS.get() == 0
}

/// Install these four entry points into the radio crate's OTA seam.
///
/// Host tests compile `tlsr8266_radio` as a dependency, so the radio never sees
/// this crate's `cfg(test)` and its OTA slot starts on the radio's own in-crate
/// stand-in. A test that drives an OTA-gated radio branch (the system-timer
/// event selection, the GATT terminate handshake) installs the real hooks here
/// first; production does the same thing in `app::main_task` through
/// [`tlsr8266_radio::callbacks::set_ota_hooks`].
#[cfg(test)]
pub fn install_hooks_for_test() {
    tlsr8266_radio::callbacks::set_ota_hooks(tlsr8266_radio::callbacks::OtaHooks {
        on_gatt_ota_write: on_gatt_ota_packet,
        status_snapshot: ota_status_snapshot_impl,
        request_termination: ota_request_termination_impl,
        tick_timeout_1s: ota_tick_timeout_1s_impl,
    });
}

// --- composition-root entry points ------------------------------------------

/// Start a mesh OTA: claim the mesh path and arm the boot-time image copier.
pub fn start_mesh_ota() {
    OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(true);
    // A new session starts over at index 0: clear the previous transfer's
    // monotonicity cursor so its final index cannot refuse the first packet.
    LAST_INDEX.store(u16::MAX, Ordering::Relaxed);
    tlsr8266_hal::pm::set_reboot_ota_active(true);
}

/// Is a mesh OTA receiving? Mesh forwarding is suppressed while it is.
pub fn is_mesh_ota_blocked() -> bool {
    OTA_UPDATE_MESH_OPERATIONS_BLOCKED.get()
}

/// Has the OTA supervision timeout run out?
pub fn is_timeout_expired() -> bool {
    OTA_UPDATE_TIMEOUT_SECONDS.get() == 0
}

/// Advance the mesh-OTA supervision timeout from the `TMR0` interrupt.
///
/// `TMR0` fires at embassy alarm deadlines, so the countdown is gated on elapsed
/// clock ticks rather than on the interrupt rate. Returns the finish state
/// exactly once, when the counter reaches zero during an OTA.
pub fn on_1s_tick() -> Option<OtaState> {
    let now = clock_time64();
    if !is_mesh_ota_blocked() {
        // Restamp while OTA is idle so the first interrupt after OTA begins
        // cannot decrement the timeout immediately.
        *LAST_OTA_TIMEOUT_TICK.lock() = now;
        return None;
    }

    let mut last = LAST_OTA_TIMEOUT_TICK.lock();
    if now.saturating_sub(*last) < CLOCK_SYS_CLOCK_1S as u64 {
        return None;
    }
    *last = now;
    drop(last);

    if OTA_UPDATE_TIMEOUT_SECONDS.get() == 0 {
        return None;
    }
    OTA_UPDATE_TIMEOUT_SECONDS.dec();
    if OTA_UPDATE_TIMEOUT_SECONDS.get() == 0 {
        // Copy the flag out before returning: the caller erases the OTA staging
        // area and burns the reboot latch, so the guard must not be alive there.
        Some(*RF_SLAVE_OTA_FINISHED_FLAG.lock())
    } else {
        None
    }
}

// --- host-test isolation -----------------------------------------------------

/// Restore every OTA scalar, the staging fake and the tick to its boot value.
///
/// Host tests run in one process, so the globals would otherwise leak OTA state
/// from one test into the next. There is no production cost.
#[cfg(test)]
pub fn reset_for_test() {
    *OTA_STATE.lock() = OtaRuntimeState::new();
    *RF_SLAVE_OTA_FINISHED_FLAG.lock() = OtaState::Continue;
    OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(false);
    OTA_UPDATE_IN_PROGRESS.set(false);
    OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(0);
    OTA_UPDATE_TERMINATION_REQUESTED.set(false);
    OTA_UPDATE_TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
    LAST_INDEX.store(u16::MAX, Ordering::Relaxed);
    *LAST_OTA_TIMEOUT_TICK.lock() = 0;
    fake_radio::reset();
}

/// Overwrites the transfer-completion flag.
#[cfg(test)]
pub fn set_finished_flag_for_test(state: OtaState) {
    *RF_SLAVE_OTA_FINISHED_FLAG.lock() = state;
}

/// Overwrites the supervision timeout, in seconds.
#[cfg(test)]
pub fn set_timeout_for_test(seconds: u16) {
    OTA_UPDATE_TIMEOUT_SECONDS.set(seconds);
}

/// Reads the supervision timeout back, in seconds.
#[cfg(test)]
pub fn timeout_seconds_for_test() -> u16 {
    OTA_UPDATE_TIMEOUT_SECONDS.get()
}

/// Overwrites the BLE-OTA in-progress flag.
#[cfg(test)]
pub fn set_in_progress_for_test(in_progress: bool) {
    OTA_UPDATE_IN_PROGRESS.set(in_progress);
}

/// Overwrites the mesh-OTA blocked flag.
#[cfg(test)]
pub fn set_mesh_ota_blocked_for_test(blocked: bool) {
    OTA_UPDATE_MESH_OPERATIONS_BLOCKED.set(blocked);
}

/// Did the finish path get asked for `PKT_TERMINATE` notifications?
#[cfg(test)]
pub fn is_termination_requested() -> bool {
    OTA_UPDATE_TERMINATION_REQUESTED.get()
}

/// Overwrites the terminate-request flag.
#[cfg(test)]
pub fn set_termination_requested_for_test(requested: bool) {
    OTA_UPDATE_TERMINATION_REQUESTED.set(requested);
}

/// Overwrites the staging write cursor.
#[cfg(test)]
pub fn set_current_flash_address_for_test(addr: u32) {
    OTA_UPDATE_CURRENT_FLASH_ADDRESS.set(addr);
}
/// Reads the staging write cursor back.
#[cfg(test)]
pub fn current_flash_address_for_test() -> u32 {
    OTA_UPDATE_CURRENT_FLASH_ADDRESS.get()
}

/// Allows (or forbids) a GATT OTA to begin, for the seam-wiring test.
#[cfg(test)]
pub fn set_gatt_ota_begin_for_test(ok: bool) {
    fake_radio::set_begin_ok(ok);
}

/// Overwrites the `TMR0` tick stamp that gates the countdown.
#[cfg(test)]
pub fn set_last_ota_timeout_tick_for_test(tick: u64) {
    *LAST_OTA_TIMEOUT_TICK.lock() = tick;
}

/// Reads the `TMR0` tick stamp back.
#[cfg(test)]
pub fn last_ota_timeout_tick_for_test() -> u64 {
    *LAST_OTA_TIMEOUT_TICK.lock()
}

/// The ota-local stand-in for the real staging/RF providers.
///
/// `OtaRadioOps`' real implementations live *above* `ota`
/// (`connection_management`, `app`), so naming them here — even in test code —
/// would invert the layering the seam exists to enforce. The fake therefore owns
/// its own 16-slot staging window and its own flags, which is all the OTA tests
/// need.
#[cfg(test)]
mod fake_radio {
    use super::*;

    /// Slots in the fake staging window; mirrors the production buffer depth.
    pub(super) const FAKE_OTA_SLOTS: usize = 16;

    /// Zero packet used to fill the fake staging window.
    const EMPTY_PACKET: Packet = Packet {
        att_data: PacketAttData {
            head: mesh_protocol::packet_types::PacketL2capHead {
                dma_len: 0,
                _type: 0,
                rf_len: 0,
                l2cap_len: 0,
                chan_id: 0,
            },
            att: 0,
            hl: 0,
            hh: 0,
            dat: [0; 23],
        },
    };

    /// Stand-in staging window.
    pub(super) static FAKE_STAGED: CriticalSectionMutex<[Packet; FAKE_OTA_SLOTS]> =
        CriticalSectionMutex::new([EMPTY_PACKET; FAKE_OTA_SLOTS]);
    /// What `try_begin_gatt_ota` reports.
    static FAKE_BEGIN_OK: AtomicBool = AtomicBool::new(false);
    /// What `try_send_terminate_packet` reports.
    static FAKE_TERMINATE_READY: AtomicBool = AtomicBool::new(false);
    /// How often `try_send_terminate_packet` was called.
    static FAKE_TERMINATE_CALLS: AtomicU16 = AtomicU16::new(0);

    pub(super) fn begin() -> bool {
        FAKE_BEGIN_OK.get()
    }

    pub(super) fn stage(idx: usize, p: &Packet) {
        FAKE_STAGED.lock()[idx % FAKE_OTA_SLOTS] = *p;
    }

    pub(super) fn read(idx: usize) -> Packet {
        FAKE_STAGED.lock()[idx]
    }

    pub(super) fn terminate() -> bool {
        FAKE_TERMINATE_CALLS.inc();
        FAKE_TERMINATE_READY.get()
    }

    pub(super) fn reset() {
        *FAKE_STAGED.lock() = [EMPTY_PACKET; FAKE_OTA_SLOTS];
        FAKE_BEGIN_OK.set(false);
        FAKE_TERMINATE_READY.set(false);
        FAKE_TERMINATE_CALLS.set(0);
    }

    pub(super) fn set_begin_ok(ok: bool) {
        FAKE_BEGIN_OK.set(ok);
    }

    pub(super) fn set_terminate_ready(ready: bool) {
        FAKE_TERMINATE_READY.set(ready);
    }

    pub(super) fn terminate_calls() -> u16 {
        FAKE_TERMINATE_CALLS.get()
    }
}

#[cfg(test)]
const TEST_RADIO_OPS: OtaRadioOps = OtaRadioOps {
    try_begin_gatt_ota: fake_radio::begin,
    stage_packet: fake_radio::stage,
    read_staged_packet: fake_radio::read,
    try_send_terminate_packet: fake_radio::terminate,
};

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// `reg_gpio_gpio_func`: per-port GPIO-function select register.
    const REG_GPIO_GPIO_FUNC: u32 = 0x586;
    /// `reg_gpio_oen`: per-port output-enable register (inverted logic).
    const REG_GPIO_OEN: u32 = 0x582;
    /// `reg_gpio_out`: per-port output-data register.
    const REG_GPIO_OUT: u32 = 0x583;
    /// `reg_tmr_ctrl`: the watchdog clear bit lives here.
    const REG_TMR_CTRL: u32 = 0x620;
    /// `reg_pwdn_ctrl`: the MCU reset latch `light_sw_reboot` writes.
    const REG_PWDN_CTRL: u32 = 0x6f;
    /// `FldPwdnCtrl::Reboot`: the bit that triggers the reset.
    const PWDN_CTRL_REBOOT: u32 = 0x20;

    /// The HAL host seams are process-wide singletons, so every test that
    /// observes them starts from a known-empty bank and tick count.
    fn reset_fakes() {
        FakeMmio::reset();
        fake_clock::reset();
        FakeFlash::reset();
    }

    /// The values written to the register at `offset`, oldest first.
    fn writes_to(offset: u32) -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == (offset | tlsr8266_pac::REG_BASE_ADDR))
            .map(|w| w.value)
            .collect()
    }

    /// A staged image whose firmware flag the Ok-finish path accepts, so
    /// `rf_ota_set_flag` really writes the ready sentinel.
    fn seed_valid_firmware_flag() {
        FakeFlash::write_slice(FLASH_ADR_LIGHT_NEW_FW + 8, &START_UP_FLAG.to_le_bytes());
    }

    /// The reboot latch fired exactly once: the observable of
    /// `rf_link_slave_ota_finish_led_and_reboot`.
    fn assert_reboot_latched() {
        assert_eq!(
            writes_to(REG_PWDN_CTRL),
            vec![PWDN_CTRL_REBOOT],
            "the finish path must latch exactly one MCU reset"
        );
    }

    /// The reboot latch never fired.
    fn assert_no_reboot() {
        assert!(
            writes_to(REG_PWDN_CTRL).is_empty(),
            "no finish path may latch a reboot here"
        );
    }

    /// The GATT write handler stages packets without holding `OTA_STATE` while
    /// it copies into the staging buffer, and advances the cursor once per
    /// packet.
    #[test]
    fn test_on_gatt_ota_packet_stages_packets_and_advances_cursor() {
        reset_for_test();
        fake_radio::set_begin_ok(true);

        let mut staged = PacketAttData::default();
        staged.dat[0] = 0xAB;
        let packet = Packet { att_data: staged };

        on_gatt_ota_packet(&packet);
        on_gatt_ota_packet(&packet);

        assert!(
            OTA_UPDATE_IN_PROGRESS.get(),
            "the first accepted write must claim the OTA"
        );
        assert_eq!(OTA_STATE.lock().slave_ota_data_cache_idx, 2);
        assert_eq!(radio_read_pkt(0).att_data().dat[0], 0xAB);
        assert_eq!(radio_read_pkt(1).att_data().dat[0], 0xAB);

        reset_for_test();
    }

    /// A rejected pair gate must leave the OTA claim and the cursor untouched.
    #[test]
    fn test_on_gatt_ota_packet_rejected_by_begin_gate_changes_nothing() {
        reset_for_test();
        fake_radio::set_begin_ok(false);

        on_gatt_ota_packet(&Packet {
            att_data: PacketAttData::default(),
        });

        assert!(
            !OTA_UPDATE_IN_PROGRESS.get(),
            "an unpaired write must not claim the OTA"
        );
        assert_eq!(OTA_STATE.lock().slave_ota_data_cache_idx, 0);
    }

    /// `terminate_cnt` is the last owner-move: the countdown and the reboot must
    /// happen outside the critical section.
    #[test]
    fn test_finish_handle_counts_down_terminate_then_reboots() {
        reset_fakes();
        reset_for_test();
        seed_valid_firmware_flag();
        set_finished_flag_for_test(OtaState::Ok);
        set_termination_requested_for_test(true);
        OTA_STATE.lock().terminate_cnt = 1;

        fake_clock::set_ticks(0);

        rf_link_slave_ota_finish_handle();

        assert_eq!(OTA_STATE.lock().terminate_cnt, 0);
        // The real reboot path ran with `Ok`: it armed the ready sentinel and
        // latched exactly one reset. (The old mock asserted the state argument;
        // the ready sentinel is written only on the `Ok` arm, `Error` erases the
        // staging area instead.)
        let mut ready = [0u8; 1];
        FakeFlash::read_slice(flash_layout::FLASH_ADR_OTA_READY_FLAG, &mut ready);
        assert_eq!(
            ready[0],
            flash_layout::FLASH_OTA_READY_FLAG,
            "the Ok finish must arm the boot-time OTA ready sentinel"
        );
        assert_reboot_latched();

        reset_for_test();
    }

    /// The terminate transmit only arms the countdown when the provider reports
    /// that the packet really was queued.
    #[test]
    fn test_finish_handle_does_not_arm_when_terminate_is_not_ready() {
        reset_fakes();
        reset_for_test();
        set_finished_flag_for_test(OtaState::Error);
        set_termination_requested_for_test(true);
        fake_radio::set_terminate_ready(false);

        fake_clock::set_ticks(0);

        rf_link_slave_ota_finish_handle();

        assert_eq!(
            fake_radio::terminate_calls(),
            1,
            "the provider must be asked exactly once"
        );
        assert_eq!(
            OTA_STATE.lock().terminate_cnt,
            0,
            "an unqueued terminate packet must not arm the countdown"
        );
        assert_no_reboot();
        // Nothing rebooted, so the error arm never erased the staging area.
        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_LIGHT_NEW_FW),
            0,
            "the error finish path must not run"
        );

        reset_for_test();
    }

    /// The terminate transmit arms the six-packet countdown when it is queued.
    #[test]
    fn test_finish_handle_arms_when_terminate_is_ready() {
        reset_fakes();
        reset_for_test();
        set_finished_flag_for_test(OtaState::Ok);
        set_termination_requested_for_test(true);
        fake_radio::set_terminate_ready(true);

        fake_clock::set_ticks(0);

        rf_link_slave_ota_finish_handle();

        assert_eq!(fake_radio::terminate_calls(), 1);
        // Armed to 6, then decremented once in this same call.
        assert_eq!(OTA_STATE.lock().terminate_cnt, 5);
        assert_no_reboot();

        reset_for_test();
    }

    /// The OTA scalars boot at the values the old `OtaManager::default_const()`
    /// used, and `reset_for_test()` restores them after a transfer.
    #[test]
    fn test_ota_runtime_state_defaults_and_reset_for_test() {
        let defaults = OtaRuntimeState::default();
        assert_eq!(defaults.ota_rcv_last_idx, 0xffff);
        assert_eq!(defaults.slave_ota_data_cache_idx, 0);
        assert_eq!(defaults.rf_slave_ota_finished_time, 0);
        assert_eq!(defaults.terminate_cnt, 0);

        {
            let mut st = OTA_STATE.lock();
            st.ota_rcv_last_idx = 7;
            st.slave_ota_data_cache_idx = 3;
            st.rf_slave_ota_finished_time = 42;
            st.terminate_cnt = 2;
        }

        reset_for_test();

        let st = OTA_STATE.lock();
        assert_eq!(st.ota_rcv_last_idx, 0xffff);
        assert_eq!(st.slave_ota_data_cache_idx, 0);
        assert_eq!(st.rf_slave_ota_finished_time, 0);
        assert_eq!(st.terminate_cnt, 0);
    }

    /// `reset_for_test()` also clears the radio fake and the counters the other
    /// modules' tests rely on.
    #[test]
    fn test_reset_for_test_clears_every_scalar_and_the_radio_fake() {
        fake_radio::set_begin_ok(true);
        fake_radio::set_terminate_ready(true);
        fake_radio::stage(
            0,
            &Packet {
                att_data: PacketAttData::default(),
            },
        );
        fake_radio::terminate();
        set_finished_flag_for_test(OtaState::Error);
        set_timeout_for_test(0);
        set_in_progress_for_test(true);
        set_mesh_ota_blocked_for_test(true);
        set_termination_requested_for_test(true);
        set_current_flash_address_for_test(64);
        set_last_ota_timeout_tick_for_test(99);
        LAST_INDEX.store(3, Ordering::Relaxed);

        reset_for_test();

        assert_eq!(fake_radio::terminate_calls(), 0);
        assert!(!fake_radio::begin());
        assert!(!fake_radio::terminate());
        assert_eq!(*RF_SLAVE_OTA_FINISHED_FLAG.lock(), OtaState::Continue);
        assert_eq!(
            timeout_seconds_for_test(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS
        );
        assert!(!ota_status_snapshot_impl().in_progress);
        assert!(!is_mesh_ota_blocked());
        assert!(!is_termination_requested());
        assert_eq!(current_flash_address_for_test(), 0);
        assert_eq!(last_ota_timeout_tick_for_test(), 0);
        assert_eq!(LAST_INDEX.load(Ordering::Relaxed), u16::MAX);

        fake_radio::reset();
    }

    /// Staging one mesh packet drives the real flash path: the payload is
    /// programmed into the OTA staging area and the write cursor only advances
    /// after the driver's read-back verification matches.
    ///
    /// This test cannot observe lock re-entrancy: the driver owns its fake
    /// device, so nothing here can take `OTA_STATE` on the flash side. That
    /// invariant is documented at the production call site instead — see the
    /// "Critical-section invariant" note on `rf_link_slave_data_ota_save`.
    #[test]
    fn test_ota_save_programs_and_verifies_staged_packet() {
        reset_fakes();
        reset_for_test();
        // Mesh OTA mode: skips the CRC check and uses 8-byte packets.
        set_mesh_ota_blocked_for_test(true);
        set_current_flash_address_for_test(0);

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7 + 8;
        staged.dat[0] = 0; // cur_idx low byte: first data packet
        staged.dat[1] = 0; // cur_idx high byte
        staged.dat[2..10].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        radio_stage_pkt(0, &Packet { att_data: staged });
        OTA_STATE.lock().slave_ota_data_cache_idx = 1;

        fake_clock::set_ticks(1234);

        rf_link_slave_data_ota_save();

        // The 8-byte payload is programmed at the start of the staging area.
        let mut staged_bytes = [0u8; 8];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW, &mut staged_bytes);
        assert_eq!(staged_bytes, [1, 2, 3, 4, 5, 6, 7, 8]);

        // `rf_ota_save_data` advances the cursor only when its read-back
        // verification matches, so address 8 proves write and verify both ran.
        assert_eq!(current_flash_address_for_test(), 8);

        {
            let st = OTA_STATE.lock();
            assert_eq!(
                st.slave_ota_data_cache_idx, 0,
                "the staging window is drained"
            );
            assert_eq!(
                st.ota_rcv_last_idx, 0,
                "the first data packet leaves the cursor at index 0"
            );
            // The write read back byte-exact, so verification succeeded and the
            // transfer stays in progress (no finish time, flag still `Continue`).
            assert_eq!(
                st.rf_slave_ota_finished_time, 0,
                "a verified mid-transfer packet must not finish the transfer"
            );
        }
        assert_eq!(*RF_SLAVE_OTA_FINISHED_FLAG.lock(), OtaState::Continue);

        reset_for_test();
    }

    /// A failed `WriteNextChunk` leaves the non-zero flash cursor alone, exactly
    /// like the legacy `else if cur_idx == ota_rcv_last_idx + 1` arm: the
    /// retransmitted packet rewrites the same address instead of restarting at
    /// the beginning of the staging area. This is the asymmetry that matters —
    /// a zeroed cursor here would silently overwrite the image head.
    #[test]
    fn test_failed_next_chunk_write_keeps_nonzero_cursor() {
        reset_fakes();
        reset_for_test();
        set_mesh_ota_blocked_for_test(true);
        // Mid-transfer: the image head is already written.
        set_current_flash_address_for_test(8);

        // Corrupt the programmed byte so the read-back verification fails.
        FakeFlash::fail_verify_next_write();

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7 + 8;
        staged.dat[0] = 1; // cur_idx = 1 == last (0) + 1
        staged.dat[1] = 0;
        staged.dat[2..10].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        radio_stage_pkt(0, &Packet { att_data: staged });
        {
            let mut st = OTA_STATE.lock();
            st.slave_ota_data_cache_idx = 1;
            st.ota_rcv_last_idx = 0;
        }

        fake_clock::set_ticks(1234);

        rf_link_slave_data_ota_save();

        assert_eq!(
            *RF_SLAVE_OTA_FINISHED_FLAG.lock(),
            OtaState::Error,
            "a failed next-chunk write must finish the transfer as an error"
        );
        assert_eq!(
            current_flash_address_for_test(),
            8,
            "the failed next-chunk path must leave the retransmit address alone"
        );
        assert_eq!(
            OTA_STATE.lock().slave_ota_data_cache_idx,
            0,
            "the staging window is drained even when the write fails"
        );

        reset_for_test();
    }

    /// The two `RejectError` arms that share the `last + 1` path differ in one
    /// respect only: the size-bound overrun leaves the write cursor untouched
    /// (legacy `HEAD:rust/src/ota.rs:348-358`) while a gap zeroes it (`:360-366`).
    #[test]
    fn test_size_bound_rejection_keeps_cursor_but_gap_rejection_zeroes_it() {
        reset_fakes();
        reset_for_test();

        // (a) BLE-bridge mode: 16-byte payloads. 131068 + 16 overruns the 128 KiB
        // staging area, so the packet is rejected at the size bound.
        set_current_flash_address_for_test(flash_layout::FW_SIZE_MAX_K * 1024 - 4);

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7 + 16;
        staged.dat[0] = 1; // cur_idx = 1 == last (0) + 1
        staged.dat[1] = 0;
        staged.dat[2..18].copy_from_slice(&[0x5A; 16]);
        // Bridge mode checks the trailing CRC: dat[18..20] = crc16(dat[0..18]).
        let crc = crc16(&staged.dat[0..18]).to_le_bytes();
        staged.dat[18..20].copy_from_slice(&crc);
        radio_stage_pkt(0, &Packet { att_data: staged });
        {
            let mut st = OTA_STATE.lock();
            st.slave_ota_data_cache_idx = 1;
            st.ota_rcv_last_idx = 0;
        }

        fake_clock::set_ticks(1234);

        rf_link_slave_data_ota_save();

        assert_eq!(
            *RF_SLAVE_OTA_FINISHED_FLAG.lock(),
            OtaState::Error,
            "a packet past the staging bound must finish as an error"
        );
        assert_eq!(
            current_flash_address_for_test(),
            flash_layout::FW_SIZE_MAX_K * 1024 - 4,
            "the size-bound rejection must not move the write cursor"
        );

        // (b) Same `last + 1` shape replaced by a gap: this one does zero it.
        set_finished_flag_for_test(OtaState::Continue);
        set_mesh_ota_blocked_for_test(true);
        set_current_flash_address_for_test(100);

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7 + 8;
        staged.dat[0] = 5; // cur_idx = 5, not last + 1
        staged.dat[1] = 0;
        radio_stage_pkt(0, &Packet { att_data: staged });
        {
            let mut st = OTA_STATE.lock();
            st.slave_ota_data_cache_idx = 1;
            st.ota_rcv_last_idx = 1;
        }

        rf_link_slave_data_ota_save();

        assert_eq!(*RF_SLAVE_OTA_FINISHED_FLAG.lock(), OtaState::Error);
        assert_eq!(
            current_flash_address_for_test(),
            0,
            "a gap rejection must reset the flash cursor"
        );

        reset_for_test();
    }

    /// A gap rejection zeroes the flash cursor, unlike the first-chunk write
    /// failure above.
    #[test]
    fn test_gap_rejection_zeroes_flash_cursor() {
        reset_fakes();
        reset_for_test();
        set_mesh_ota_blocked_for_test(true);
        set_current_flash_address_for_test(64);

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7 + 8;
        staged.dat[0] = 9; // cur_idx = 9, not last + 1
        staged.dat[1] = 0;
        radio_stage_pkt(0, &Packet { att_data: staged });
        {
            let mut st = OTA_STATE.lock();
            st.slave_ota_data_cache_idx = 1;
            st.ota_rcv_last_idx = 3;
        }

        fake_clock::set_ticks(1234);

        rf_link_slave_data_ota_save();

        assert_eq!(*RF_SLAVE_OTA_FINISHED_FLAG.lock(), OtaState::Error);
        assert_eq!(
            current_flash_address_for_test(),
            0,
            "an out-of-sequence packet must reset the flash cursor"
        );
        assert_eq!(OTA_STATE.lock().ota_rcv_last_idx, 9);

        reset_for_test();
    }

    /// A successful final header finishes as `Ok` and leaves the cursor alone.
    #[test]
    fn test_final_header_success_keeps_flash_cursor() {
        reset_fakes();
        reset_for_test();
        set_mesh_ota_blocked_for_test(true);
        set_current_flash_address_for_test(24);

        // The staged image advertises 3 packets of 8 bytes at header offset 0x18.
        FakeFlash::write_slice(FLASH_ADR_LIGHT_NEW_FW + 0x18, &24u32.to_le_bytes());

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7; // header-only packet
        staged.dat[0] = 3;
        staged.dat[1] = 0;
        radio_stage_pkt(0, &Packet { att_data: staged });
        {
            let mut st = OTA_STATE.lock();
            st.slave_ota_data_cache_idx = 1;
            st.ota_rcv_last_idx = 2;
        }

        fake_clock::set_ticks(1234);

        rf_link_slave_data_ota_save();

        assert_eq!(*RF_SLAVE_OTA_FINISHED_FLAG.lock(), OtaState::Ok);
        assert_eq!(
            current_flash_address_for_test(),
            24,
            "the successful final-header path must not touch the cursor"
        );

        reset_for_test();
    }

    /// A final header whose advertised size disagrees with the expected packet
    /// count is an invalid image: the transfer finishes as `Error`, the write
    /// cursor is zeroed and the staged window is drained.
    #[test]
    fn test_final_header_size_mismatch_fails_and_zeroes_cursor() {
        reset_fakes();
        reset_for_test();
        set_mesh_ota_blocked_for_test(true);
        set_current_flash_address_for_test(24);

        // 32 bytes / 8-byte packets = 4 packets, but the header arrives at
        // index 3, so the FSM reports `VerifyFinalHeader { expected_index: 3 }`.
        FakeFlash::write_slice(FLASH_ADR_LIGHT_NEW_FW + 0x18, &32u32.to_le_bytes());

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7; // header-only packet
        staged.dat[0] = 3;
        staged.dat[1] = 0;
        radio_stage_pkt(0, &Packet { att_data: staged });
        {
            let mut st = OTA_STATE.lock();
            st.slave_ota_data_cache_idx = 1;
            st.ota_rcv_last_idx = 2;
        }

        fake_clock::set_ticks(1234);

        rf_link_slave_data_ota_save();

        assert_eq!(
            *RF_SLAVE_OTA_FINISHED_FLAG.lock(),
            OtaState::Error,
            "a size/index mismatch must finish the transfer as an error"
        );
        assert_eq!(
            current_flash_address_for_test(),
            0,
            "the invalid final header must reset the write cursor"
        );
        assert_eq!(
            OTA_STATE.lock().slave_ota_data_cache_idx,
            0,
            "the staging window is drained on the failure path"
        );

        reset_for_test();
    }

    /// An image too small to hold three packets is invalid even when the
    /// advertised total matches the expected index: the `total < 3` guard is
    /// what rejects it.
    #[test]
    fn test_final_header_under_three_packets_fails() {
        reset_fakes();
        reset_for_test();
        set_mesh_ota_blocked_for_test(true);
        set_current_flash_address_for_test(8);

        // 8 bytes / 8-byte packets = 1 packet, which matches index 1 but is
        // below the three-packet minimum.
        FakeFlash::write_slice(FLASH_ADR_LIGHT_NEW_FW + 0x18, &8u32.to_le_bytes());

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 7; // header-only packet
        staged.dat[0] = 1;
        staged.dat[1] = 0;
        radio_stage_pkt(0, &Packet { att_data: staged });
        {
            let mut st = OTA_STATE.lock();
            st.slave_ota_data_cache_idx = 1;
            st.ota_rcv_last_idx = 0;
        }

        fake_clock::set_ticks(1234);

        rf_link_slave_data_ota_save();

        assert_eq!(
            *RF_SLAVE_OTA_FINISHED_FLAG.lock(),
            OtaState::Error,
            "an image shorter than three packets must finish as an error"
        );
        assert_eq!(
            current_flash_address_for_test(),
            0,
            "the undersized final header must reset the write cursor"
        );

        reset_for_test();
    }

    /// A blank staging area is valid; a single written byte invalidates it;
    /// erasing it restores validity. This is the boot dirty-area decision.
    #[test]
    fn test_is_ota_area_valid_tracks_writes_and_erase() {
        reset_fakes();
        reset_for_test();

        assert_eq!(
            is_ota_area_valid(),
            Ok(true),
            "a blank staging area must read as valid"
        );

        FakeFlash::write_slice(FLASH_ADR_LIGHT_NEW_FW, &[0x00]);
        assert_eq!(
            is_ota_area_valid(),
            Ok(false),
            "a written byte in the first sector must invalidate the area"
        );

        erase_ota_data();
        assert_eq!(
            is_ota_area_valid(),
            Ok(true),
            "erasing the staging area must make it valid again"
        );

        reset_for_test();
    }

    /// Erasing the staging area covers every sector, including the last one.
    #[test]
    fn test_erase_ota_data_erases_every_sector() {
        reset_fakes();
        reset_for_test();

        erase_ota_data();

        assert_eq!(
            FakeFlash::erase_count(FLASH_ADR_LIGHT_NEW_FW),
            1,
            "the first staging sector must be erased exactly once"
        );
        assert_eq!(
            FakeFlash::erase_count(
                FLASH_ADR_LIGHT_NEW_FW + (flash_layout::ERASE_SECTORS_FOR_OTA - 1) * 0x1000
            ),
            1,
            "the last staging sector must be erased exactly once"
        );

        reset_for_test();
    }

    /// A boot with a dirty staging sector must erase the whole OTA area.
    ///
    /// `check_ota_area_startup` is the boot-time recovery path: a half-written
    /// image at `FLASH_ADR_LIGHT_NEW_FW` (left behind by a reset mid-transfer)
    /// must never be copied out by `handle_ota_update`, so the area is erased
    /// before the ready flag is consulted.
    #[test]
    fn test_check_ota_area_startup_erases_dirty_staging_sector() {
        reset_fakes();
        reset_for_test();

        let sectors: Vec<u32> = (0..flash_layout::ERASE_SECTORS_FOR_OTA)
            .map(|i| FLASH_ADR_LIGHT_NEW_FW + i * 0x1000)
            .collect();
        let erases_before: Vec<u32> = sectors
            .iter()
            .map(|&sector| FakeFlash::erase_count(sector))
            .collect();

        FakeFlash::write_slice(FLASH_ADR_LIGHT_NEW_FW, &[0x00, 0x11, 0x22, 0x33]);
        assert_eq!(
            is_ota_area_valid(),
            Ok(false),
            "a written word in the first staging sector must invalidate the area"
        );

        check_ota_area_startup();

        for (sector, before) in sectors.into_iter().zip(erases_before) {
            assert_eq!(
                FakeFlash::erase_count(sector) - before,
                1,
                "boot recovery must erase staging sector {sector:#x} exactly once"
            );
        }
        assert_eq!(
            is_ota_area_valid(),
            Ok(true),
            "the staging area must validate after boot recovery"
        );

        let mut staged = [0u8; 4];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW, &mut staged);
        assert_eq!(
            staged, [0xFF; 4],
            "boot recovery must leave the staging bytes erased"
        );

        reset_for_test();
    }

    /// The failure indicator must genuinely alternate: `gpio_write` drives the pin
    /// high for any non-zero value, so the old `!`-on-`u32` toggle (`0xFFFF_FFFE`)
    /// held the pin high for the whole 6 s and looked identical to success.
    ///
    /// The GPIO helpers run for real against the fake register bank: each
    /// `gpio_write` reads the port's OUT byte back before setting or clearing its
    /// bit, so the write log carries the same alternating values the old write
    /// mock recorded, and the OEN/func registers are ordinary register state.
    #[test]
    fn test_rf_led_ota_error_alternates_pin_sixty_times() {
        // `OTA_LED` is port C, pin 0: the pin id's low byte is the bit mask and
        // the port number is the register offset, so (pin >> 8) << 3 == 0x10.
        const PIN_MASK: u8 = 0x01;
        const REG_OFFSET: u32 = 0x10;

        reset_fakes();
        reset_for_test();

        rf_led_ota_error();

        // `gpio_set_func` selects the GPIO function for the LED pin by writing
        // the pin mask into `reg_gpio_gpio_func`.
        let func_writes: Vec<_> = FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == (REG_GPIO_GPIO_FUNC + REG_OFFSET) | tlsr8266_pac::REG_BASE_ADDR)
            .collect();
        assert_eq!(
            func_writes.len(),
            1,
            "gpio_set_func must select the GPIO function exactly once"
        );
        assert_eq!(func_writes[0].value, u32::from(PIN_MASK));

        // Output enabled once, before the first write (OEN is active-low).
        let oen_writes = writes_to(REG_GPIO_OEN + REG_OFFSET);
        assert_eq!(
            oen_writes,
            vec![u32::from(0u8)],
            "the LED pin must be enabled as an output exactly once"
        );

        // Sixty writes that alternate high, low, high, low, ...
        let out_writes = writes_to(REG_GPIO_OUT + REG_OFFSET);
        let expected: Vec<u32> = (0..60)
            .map(|i| u32::from(if i % 2 == 0 { PIN_MASK } else { 0 }))
            .collect();
        assert_eq!(
            out_writes, expected,
            "the OTA error LED must be driven high and low alternately, sixty times"
        );
        // The real `wd_clear` pokes the CLR_WD bit of `reg_tmr_ctrl` once per
        // iteration; the write log is the observable that replaces the mock.
        assert_eq!(
            writes_to(REG_TMR_CTRL).len(),
            60,
            "the watchdog must be cleared once per blink"
        );
        // Cadence unchanged: 60 iterations of 100 ms, recorded by `sleep_us`.
        let expected_sleeps = vec![100 * 1000u32; 60];
        assert_eq!(fake_clock::sleeps(), expected_sleeps);
    }

    // ================================================================================
    // Tests for the rf_ota_save_data flash write-verify cycle
    // ================================================================================

    /// Helper function to reset global state for test isolation
    fn reset_ota_state() {
        set_current_flash_address_for_test(0);
        // The device is process-global: start every test from a blank NOR array
        // so a write-then-verify cycle really succeeds.
        FakeFlash::reset();
    }

    /// Tests rf_ota_save_data with basic functionality.
    ///
    /// Verifies that the function writes the packet, reads it back, and advances
    /// the write pointer.
    #[test]
    fn test_rf_ota_save_data_basic_functionality() {
        reset_ota_state();

        let test_data = [0x01, 0x02, 0x03, 0x04];
        let initial_address = current_flash_address_for_test();

        // Call the function under test
        let result = rf_ota_save_data(&test_data);

        // The write really landed and verified, so the packet is accepted.
        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        let mut got = [0u8; 4];
        FakeFlash::read_slice(expected_addr, &mut got);
        assert_eq!(got, test_data);
        assert_eq!(result, OtaState::Continue);
        assert_eq!(
            current_flash_address_for_test(),
            initial_address + test_data.len() as u32
        );
    }

    /// Tests rf_ota_save_data with empty data.
    ///
    /// Verifies that the function handles empty data gracefully.
    #[test]
    fn test_rf_ota_save_data_empty_data() {
        reset_ota_state();

        let empty_data: &[u8] = &[];
        let initial_address = current_flash_address_for_test();

        let result = rf_ota_save_data(empty_data);

        // Empty data should succeed since verification will compare empty slices
        assert_eq!(
            result,
            OtaState::Continue,
            "Should return Continue for empty data"
        );

        // Flash address should advance by the length of data (which is 0 for empty data)
        assert_eq!(
            current_flash_address_for_test(),
            initial_address + empty_data.len() as u32,
            "Flash address should advance by data length (0 for empty data)"
        );
    }

    /// Tests rf_ota_save_data with single byte.
    ///
    /// Verifies that the function correctly handles minimal data sizes.
    #[test]
    fn test_rf_ota_save_data_single_byte() {
        reset_ota_state();

        let single_byte = [0x42];
        let initial_address = current_flash_address_for_test();

        let result = rf_ota_save_data(&single_byte);

        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        let mut got = [0u8; 1];
        FakeFlash::read_slice(expected_addr, &mut got);
        assert_eq!(got, single_byte);
        assert_eq!(result, OtaState::Continue);
        assert_eq!(current_flash_address_for_test(), initial_address + 1);
    }

    /// Tests rf_ota_save_data with maximum typical packet size.
    ///
    /// Verifies that the function handles larger data packets correctly.
    #[test]
    fn test_rf_ota_save_data_max_packet_size() {
        reset_ota_state();

        // Create 16-byte test packet with sequential pattern
        let max_packet: Vec<u8> = (0..16).map(|i| (i & 0xFF) as u8).collect();
        let initial_address = current_flash_address_for_test();

        let result = rf_ota_save_data(&max_packet);

        let expected_addr = initial_address + FLASH_ADR_LIGHT_NEW_FW;
        let mut got = [0u8; 16];
        FakeFlash::read_slice(expected_addr, &mut got);
        assert_eq!(got, max_packet.as_slice());
        assert_eq!(result, OtaState::Continue);
        assert_eq!(
            current_flash_address_for_test(),
            initial_address + max_packet.len() as u32
        );
    }

    /// Tests rf_ota_save_data with different flash addresses.
    ///
    /// Verifies that the function correctly calculates flash addresses
    /// based on the current offset plus the base address.
    #[test]
    fn test_rf_ota_save_data_address_calculation() {
        reset_ota_state();

        // Set a specific offset for testing address calculation
        let test_offset = 0x1000;
        set_current_flash_address_for_test(test_offset);

        let test_data = [0xAA, 0xBB];

        let result = rf_ota_save_data(&test_data);

        // Verify address calculation: offset + base address
        let expected_flash_addr = test_offset + FLASH_ADR_LIGHT_NEW_FW;
        let mut got = [0u8; 2];
        FakeFlash::read_slice(expected_flash_addr, &mut got);
        assert_eq!(got, test_data);
        assert_eq!(result, OtaState::Continue);
    }

    /// Tests rf_ota_save_data operation sequence.
    ///
    /// Verifies the write-verify cycle: the read-back only matches when the
    /// write happened first, so a successful verification proves the ordering.
    #[test]
    fn test_rf_ota_save_data_operation_sequence() {
        reset_ota_state();

        let test_data = [0x77, 0x77, 0x77];
        let result = rf_ota_save_data(&test_data);

        assert_eq!(result, OtaState::Continue);
        let mut got = [0u8; 3];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW, &mut got);
        assert_eq!(got, test_data);
    }

    /// Tests rf_ota_save_data with multiple sequential calls.
    ///
    /// Verifies that multiple consecutive packets are handled correctly
    /// and that the flash address advances properly.
    #[test]
    fn test_rf_ota_save_data_sequential_packets() {
        reset_ota_state();

        let packets = [
            vec![0x00, 0x01, 0x02], // First packet (3 bytes)
            vec![0x03, 0x04],       // Second packet (2 bytes)
            vec![0x05],             // Third packet (1 byte)
        ];

        let initial_address = current_flash_address_for_test();

        // Process each packet sequentially
        for (i, packet) in packets.iter().enumerate() {
            let result = rf_ota_save_data(packet);

            assert_eq!(
                result,
                OtaState::Continue,
                "Packet {} should verify and continue",
                i
            );
        }

        // The packets are appended one after another in flash.
        let mut got = [0u8; 6];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW + initial_address, &mut got);
        assert_eq!(got, [0x00, 0x01, 0x02, 0x03, 0x04, 0x05]);
        assert_eq!(current_flash_address_for_test(), initial_address + 6);
    }

    /// Tests rf_ota_save_data with various data patterns.
    ///
    /// Verifies that the function correctly handles various data patterns
    /// that might cause issues (all zeros, all ones, alternating patterns).
    #[test]
    fn test_rf_ota_save_data_edge_case_patterns() {
        reset_ota_state();

        let test_patterns = [
            vec![0x00, 0x00, 0x00, 0x00], // All zeros
            vec![0xFF, 0xFF, 0xFF, 0xFF], // All ones
            vec![0xAA, 0x55, 0xAA, 0x55], // Alternating pattern
            vec![0x00, 0xFF, 0x00, 0xFF], // Alternating extreme values
        ];

        let mut offset = 0u32;
        for (i, pattern) in test_patterns.iter().enumerate() {
            let result = rf_ota_save_data(pattern);

            assert_eq!(
                result,
                OtaState::Continue,
                "Pattern {} should verify and continue",
                i
            );

            let mut got = vec![0u8; pattern.len()];
            FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW + offset, &mut got);
            assert_eq!(&got, pattern, "pattern {} must be stored verbatim", i);
            offset += pattern.len() as u32;
        }

        assert_eq!(current_flash_address_for_test(), offset);
    }

    // ================================================================================
    // Tests for the staging bounds and the index-cursor reset (C-13, #75)
    // ================================================================================

    /// The staging window is exactly [`OTA_STAGING_SLOTS`] deep: a twentieth GATT
    /// write is dropped instead of claiming a seventeenth slot, and the drain
    /// reads back at most sixteen packets and closes the window.
    #[test]
    fn test_gatt_staging_is_bounded_and_drain_is_clamped() {
        reset_fakes();
        reset_for_test();
        fake_radio::set_begin_ok(true);

        // Well-formed bridge frames: index `i` at `dat[0..2]`, 16 payload bytes at
        // `dat[2..18]` and the CRC over `dat[0..18]` at `dat[18..20]`.
        for i in 0..20u8 {
            let mut staged = PacketAttData::default();
            staged.head.l2cap_len = 7 + 16;
            staged.dat[0] = i;
            staged.dat[1] = 0;
            staged.dat[2..18].fill(i);
            let crc = crc16(&staged.dat[0..18]).to_le_bytes();
            staged.dat[18..20].copy_from_slice(&crc);

            on_gatt_ota_packet(&Packet { att_data: staged });
        }

        assert_eq!(
            OTA_STATE.lock().slave_ota_data_cache_idx,
            OTA_STAGING_SLOTS,
            "the window must stop at the provider's depth, not at 20"
        );

        fake_clock::set_ticks(1234);
        rf_link_slave_data_ota_save();

        // Each of the sixteen verified packets advances the cursor by 16 bytes;
        // a seventeenth read would either panic or write a seventeenth block.
        assert_eq!(
            current_flash_address_for_test(),
            OTA_STAGING_SLOTS as u32 * 16,
            "the drain must persist exactly the staged sixteen packets"
        );
        assert_eq!(
            OTA_STATE.lock().slave_ota_data_cache_idx,
            0,
            "the drain resets the staging cursor"
        );
        assert_eq!(
            *RF_SLAVE_OTA_FINISHED_FLAG.lock(),
            OtaState::Continue,
            "sixteen in-sequence packets are a healthy transfer"
        );

        reset_for_test();
    }

    /// A frame whose `l2cap_len` is smaller than the 7-byte header is rejected
    /// before the payload length is computed. The old `l2cap_len - 7` underflowed
    /// here, which panics in a debug build and reads nonsense on device.
    #[test]
    fn test_drain_rejects_l2cap_len_below_header_size() {
        reset_fakes();
        reset_for_test();
        set_current_flash_address_for_test(8);

        let mut staged = PacketAttData::default();
        staged.head.l2cap_len = 6;
        radio_stage_pkt(0, &Packet { att_data: staged });
        OTA_STATE.lock().slave_ota_data_cache_idx = 1;

        fake_clock::set_ticks(1234);
        rf_link_slave_data_ota_save();

        assert_eq!(
            *RF_SLAVE_OTA_FINISHED_FLAG.lock(),
            OtaState::Error,
            "a malformed frame must finish the transfer as an error"
        );
        assert_eq!(
            current_flash_address_for_test(),
            0,
            "the malformed-frame arm zeroes the flash cursor like a CRC failure"
        );
        assert_eq!(
            OTA_STATE.lock().slave_ota_data_cache_idx,
            0,
            "the staging window is drained on the failure path"
        );
        assert_eq!(
            OTA_STATE.lock().ota_rcv_last_idx,
            0,
            "the accepted-index cursor restarts with the transfer"
        );

        reset_for_test();
    }

    /// A frame whose payload plus CRC pair does not fit the 23-byte `dat` array
    /// takes the same error arm instead of indexing past the buffer.
    #[test]
    fn test_drain_rejects_payload_past_the_att_buffer() {
        reset_fakes();
        reset_for_test();
        set_current_flash_address_for_test(8);

        let mut staged = PacketAttData::default();
        // n_data_len = 20, so the CRC pair would sit at dat[22..24] and the last
        // index is one past the 23-byte array.
        staged.head.l2cap_len = 7 + 20;
        radio_stage_pkt(0, &Packet { att_data: staged });
        OTA_STATE.lock().slave_ota_data_cache_idx = 1;

        fake_clock::set_ticks(1234);
        rf_link_slave_data_ota_save();

        assert_eq!(
            *RF_SLAVE_OTA_FINISHED_FLAG.lock(),
            OtaState::Error,
            "an oversized payload must finish the transfer as an error"
        );
        assert_eq!(
            current_flash_address_for_test(),
            0,
            "the oversized-frame arm zeroes the flash cursor like a CRC failure"
        );
        assert_eq!(OTA_STATE.lock().slave_ota_data_cache_idx, 0);

        reset_for_test();
    }

    /// `LAST_INDEX` gates the mesh OTA index stream and both session boundaries
    /// put it back to `u16::MAX`: the start ([`start_mesh_ota`]) and the
    /// abort/finish funnel ([`rf_slave_ota_finished_flag_set`]). Without that,
    /// the last index of one transfer refused index 0 of the next for good.
    #[test]
    fn test_last_index_resets_on_session_start_and_abort() {
        reset_fakes();
        reset_for_test();
        // A stale cursor left behind by an earlier transfer.
        LAST_INDEX.store(7, Ordering::Relaxed);

        start_mesh_ota();
        assert_eq!(
            LAST_INDEX.load(Ordering::Relaxed),
            u16::MAX,
            "a session start must clear the previous transfer's index"
        );

        // Index 5 against `ota_rcv_last_idx == 0xffff` is a gap, so the drain
        // aborts through the finish funnel.
        let mut gap = [0u8; 10];
        gap[0] = 5;
        assert_eq!(
            rf_mesh_data_ota(&gap, false),
            5,
            "the response still reports the index the caller sent"
        );
        assert_eq!(*RF_SLAVE_OTA_FINISHED_FLAG.lock(), OtaState::Error);
        assert_eq!(
            LAST_INDEX.load(Ordering::Relaxed),
            u16::MAX,
            "an abort must clear the index cursor"
        );

        // Second session in the same boot: index 0 is accepted again.
        set_finished_flag_for_test(OtaState::Continue);
        start_mesh_ota();
        let mut first = [0u8; 10];
        first[2] = 0xA5;
        assert_eq!(
            rf_mesh_data_ota(&first, false),
            0,
            "the next session must accept index 0"
        );
        assert_eq!(*RF_SLAVE_OTA_FINISHED_FLAG.lock(), OtaState::Continue);

        let mut programmed = [0u8; 8];
        FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW, &mut programmed);
        assert_eq!(programmed[0], 0xA5, "index 0 is staged to flash");

        reset_for_test();
    }

    /// Tests rf_ota_save_data buffer boundary conditions.
    ///
    /// Verifies that the temporary buffer (16 bytes) is used correctly
    /// and doesn't cause issues with different data sizes.
    #[test]
    fn test_rf_ota_save_data_buffer_boundaries() {
        reset_ota_state();

        // Test sizes around the 16-byte buffer boundary
        let test_sizes = [1, 8, 15, 16]; // Various sizes up to buffer limit

        let mut offset = 0u32;
        for &size in &test_sizes {
            // Create test data
            let test_data: Vec<u8> = (0..size).map(|i| (0x80 | i) as u8).collect();

            let result = rf_ota_save_data(&test_data);

            assert_eq!(result, OtaState::Continue, "Size {} must verify", size);

            let mut got = vec![0u8; size];
            FakeFlash::read_slice(FLASH_ADR_LIGHT_NEW_FW + offset, &mut got);
            assert_eq!(&got, &test_data, "size {} must be stored verbatim", size);
            offset += size as u32;
        }

        assert_eq!(current_flash_address_for_test(), offset);
    }
}
