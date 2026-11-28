//! The transport-neutral OTA engine: runtime state, chunk decisions, timeouts
//! and the completion funnel.
//!
//! This is the single owner of OTA runtime state for every transport. The
//! per-transport adapters ([`super::gatt`], [`super::mesh`], [`super::uart`])
//! decode their own wire framing into an [`OtaChunk`] and call
//! [`process_chunk`]; nothing above or below this module owns an OTA flag.
//!
//! # Locking invariant
//!
//! Flash erase/write/read, the RF terminate transmit and every product
//! indication run with **no** [`CriticalSectionMutex`] held: the owners below
//! snapshot the scalars they need into locals, drop the guard, then do the work.
//! Each guard is held for a scalar move only.

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use mesh_protocol::mesh_types::{OtaState, RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS};
use tlsr8266_hal::common::sync::{CriticalSectionMutex, SimplifyLS};
use tlsr8266_hal::mcu::clock::{clock_time, clock_time64, clock_time_exceed, CLOCK_SYS_CLOCK_1S};
use tlsr8266_hal::mcu::irq_i::irq_disable;
use tlsr8266_hal::pm::mcu_sw_reboot;
use tlsr8266_radio::ConnectionPlan;

use crate::api::{PlatformEvent, Product, RebootIndication};
use crate::window;

use super::fsm::{self, OtaChunkDecision};
use super::storage::{self, OtaFlashConfig};

/// Which wire framing delivered a chunk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OtaTransport {
    /// GATT write: 16-byte payload, CRC-16 validated by the GATT decoder.
    Gatt,
    /// Mesh command: 8-byte payload, no per-chunk CRC on the wire.
    Mesh,
    /// UART frame: 8-byte payload, no per-chunk CRC on the wire.
    Uart,
}

impl OtaTransport {
    /// Payload width of one chunk on this transport.
    pub const fn packet_len(self) -> u32 {
        match self {
            OtaTransport::Gatt => 16,
            OtaTransport::Mesh | OtaTransport::Uart => 8,
        }
    }
}

/// One OTA chunk, already decoded from its wire framing.
///
/// `index` is the wire's 16-bit chunk index widened to `u32`; the decision table
/// compares it as `u16`, exactly like the deployed device did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OtaChunk<'a> {
    pub index: u32,
    pub payload: &'a [u8],
    pub final_chunk: bool,
}

/// A transport frame that could not be decoded into a chunk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OtaFrameError {
    /// The frame head or length is impossible (short head, oversized payload).
    MalformedFrame,
    /// The trailing CRC-16 does not match the frame.
    BadCrc,
}

/// What one [`process_chunk`] call did to the transfer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChunkOutcome {
    /// The transfer is still running; feed the next chunk.
    Continue,
    /// The transfer finished with this state; stop feeding chunks.
    Finished(OtaState),
}

/// The mutable OTA runtime scalars shared by every transport.
///
/// Everything that blocks (flash erase/write/read, the RF terminate transmit,
/// product indication) is deliberately kept *outside* this type: callers take
/// the lock just long enough to move these values in or out.
struct OtaRuntimeState {
    /// Index of the last OTA packet accepted in the current transfer.
    ota_rcv_last_idx: u16,
    /// Number of frames currently staged in the shared packet window.
    staged_count: usize,
    /// `clock_time()` stamp taken when the transfer-finish flag was set.
    finished_time: u32,
    /// Remaining `PKT_TERMINATE` notifications before the reboot is allowed.
    terminate_cnt: u8,
}

impl OtaRuntimeState {
    /// Boot values, matching the deployed OTA manager's constants.
    const fn new() -> Self {
        Self {
            ota_rcv_last_idx: 0xffff,
            staged_count: 0,
            finished_time: 0,
            terminate_cnt: 0,
        }
    }
}

/// Sole owner of the OTA runtime scalars.
static OTA_STATE: CriticalSectionMutex<OtaRuntimeState> =
    CriticalSectionMutex::new(OtaRuntimeState::new());

/// Transfer-completion flag.
static FINISHED: CriticalSectionMutex<OtaState> = CriticalSectionMutex::new(OtaState::Continue);

/// Set while a *GATT* OTA is receiving.
static IN_PROGRESS: AtomicBool = AtomicBool::new(false);

/// Set while a *mesh* OTA is receiving, which selects the 8-byte payload size and
/// suppresses mesh forwarding.
static MESH_BLOCKED: AtomicBool = AtomicBool::new(false);

/// Write cursor into the staging area, relative to the staging base address.
static CURRENT_FLASH_ADDRESS: AtomicU32 = AtomicU32::new(0);

/// Set once the initiator asked for `PKT_TERMINATE` notifications.
static TERMINATION_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Remaining OTA supervision timeout, in seconds.
static TIMEOUT_SECONDS: AtomicU16 = AtomicU16::new(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);

/// Highest index accepted by the mesh OTA command path.
static LAST_INDEX: AtomicU16 = AtomicU16::new(u16::MAX);

/// Stamp of the last `TMR0` interrupt at which the mesh timeout was decremented.
///
/// `TMR0` is reprogrammed dynamically for embassy timers, so it no longer ticks
/// at a fixed 1 Hz; stamping the tick here lets [`on_1s_tick`] count real
/// seconds.
static LAST_OTA_TIMEOUT_TICK: CriticalSectionMutex<u64> = CriticalSectionMutex::new(0);

/// Stamp of the last connection event at which the GATT timeout was decremented.
///
/// Owned here now, not by the radio's connection state machine: the radio only
/// supplies the current tick with the connection slot.
static GATT_OTA_TIMEOUT_TICK: AtomicU32 = AtomicU32::new(0);

// --- scalar access -----------------------------------------------------------------

/// Is a GATT OTA receiving?
pub fn is_gatt_ota_in_progress() -> bool {
    IN_PROGRESS.get()
}

/// Is a mesh OTA receiving? Mesh forwarding is suppressed while it is.
pub fn is_mesh_ota_blocked() -> bool {
    MESH_BLOCKED.get()
}

/// Has the OTA supervision timeout run out?
pub fn is_timeout_expired() -> bool {
    TIMEOUT_SECONDS.get() == 0
}

/// Current transfer-completion flag.
pub fn finish_status() -> OtaState {
    // Copy the flag out: the guard must not span the handlers that take it again.
    *FINISHED.lock()
}

/// Staging write cursor, relative to the staging base address.
pub(crate) fn current_flash_address() -> u32 {
    CURRENT_FLASH_ADDRESS.get()
}

/// Overwrite the staging write cursor.
pub(crate) fn set_current_flash_address(offset: u32) {
    CURRENT_FLASH_ADDRESS.set(offset);
}

/// Advance the staging write cursor by a verified chunk.
pub(crate) fn advance_flash_address(len: u32) {
    CURRENT_FLASH_ADDRESS.set(CURRENT_FLASH_ADDRESS.get().wrapping_add(len));
}

/// Number of frames staged in the window, clamped to the window depth.
pub(crate) fn staged_slot_count() -> usize {
    OTA_STATE
        .lock()
        .staged_count
        .min(crate::window::PACKET_WINDOW_SLOTS)
}

/// Publish the accepted-chunk index for the next decision.
fn set_accepted_index(index: u16) {
    OTA_STATE.lock().ota_rcv_last_idx = index;
}

/// Snapshot the accepted-chunk index.
fn accepted_index() -> u16 {
    OTA_STATE.lock().ota_rcv_last_idx
}

/// Publish the staged-frame count (the GATT adapter owns the increment).
pub(crate) fn set_staged_slot_count(count: usize) {
    OTA_STATE.lock().staged_count = count;
}

/// Claim the next staging slot, or `None` when the window is full.
///
/// The claim is a short scalar move under the lock; the caller stages the frame
/// after the guard is dropped, because the window is a second, independent lock.
pub(crate) fn claim_staged_slot() -> Option<usize> {
    let mut state = OTA_STATE.lock();
    let index = state.staged_count;
    if index >= crate::window::PACKET_WINDOW_SLOTS {
        return None;
    }
    state.staged_count = index + 1;
    Some(index)
}

// --- session lifecycle -------------------------------------------------------------

/// Publish the completion flag and stamp the moment it was set.
///
/// Also the single abort/finish funnel for the mesh monotonicity cursor and the
/// shared window claim: the next session has to be able to send index 0 again
/// and claim the window again, so both go back to their boot state here.
fn finish_session(state: OtaState) {
    let finished_time = clock_time();
    *FINISHED.lock() = state;
    LAST_INDEX.store(u16::MAX, Ordering::Relaxed);
    window::end_ota();

    OTA_STATE.lock().finished_time = finished_time;
}

/// Drop the shared window claim and reset every runtime scalar.
///
/// Host tests run in one process, so the globals would otherwise leak state from
/// one test into the next. There is no production cost.
#[cfg(not(target_os = "none"))]
pub fn reset_for_test() {
    window::end_ota();
    *OTA_STATE.lock() = OtaRuntimeState::new();
    *FINISHED.lock() = OtaState::Continue;
    IN_PROGRESS.set(false);
    MESH_BLOCKED.set(false);
    set_current_flash_address(0);
    TERMINATION_REQUESTED.set(false);
    TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);
    LAST_INDEX.store(u16::MAX, Ordering::Relaxed);
    *LAST_OTA_TIMEOUT_TICK.lock() = 0;
    GATT_OTA_TIMEOUT_TICK.store(0, Ordering::Relaxed);
}

// --- chunk processing --------------------------------------------------------------

/// Decide and execute what one decoded chunk means.
///
/// The transport adapter has already applied its own frame bounds and CRC policy
/// (the GATT decoder validates the trailing CRC-16; mesh and UART frames carry
/// none), so this function owns exactly the decision table the deployed
/// `rf_link_slave_data_ota_save` ran after its CRC check.
///
/// The deployed cursor-reset asymmetries are preserved verbatim:
///
/// * a failed flash verification leaves the write cursor alone, so a retransmit
///   rewrites the same address;
/// * the size-bound overrun on the `last + 1` path leaves it alone too;
/// * a gap, index 0 on a non-empty cursor, a bad final header, a malformed frame
///   and a bad CRC all zero it.
pub fn process_chunk<P: Product>(transport: OtaTransport, chunk: OtaChunk<'_>) -> ChunkOutcome {
    let packet_len = transport.packet_len();
    let n_data_len = if chunk.final_chunk {
        0
    } else {
        chunk.payload.len()
    };
    let cur_idx = chunk.index as u16;
    let flash = OtaFlashConfig::compatibility();

    // Accepted frames refresh the supervision timeout before the decision, as
    // the deployed save path did.
    TIMEOUT_SECONDS.set(RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS);

    let last_idx = accepted_index();
    let reset_flag = match fsm::evaluate_chunk(
        cur_idx,
        last_idx,
        n_data_len,
        CURRENT_FLASH_ADDRESS.get(),
        packet_len,
        flash.max_image_bytes,
    ) {
        OtaChunkDecision::WriteFirstChunk | OtaChunkDecision::WriteNextChunk => {
            // Blocking flash work: no OTA_STATE guard is held here, and the
            // verified write advances the cursor only when the read-back matched.
            let flag = storage::save_chunk_verified(chunk.payload);
            set_accepted_index(cur_idx);
            flag
        }
        OtaChunkDecision::VerifyFinalHeader { expected_index } => {
            match storage::read_staged_bin_size() {
                Err(()) => {
                    // No size to verify, so the transfer fails and the cursor
                    // returns to the start for a fresh attempt.
                    set_current_flash_address(0);
                    set_accepted_index(0);
                    OtaState::Error
                }
                Ok(size) => {
                    let total = size.div_ceil(packet_len) as u16;
                    if total < 3 || total != expected_index {
                        set_current_flash_address(0);
                        set_accepted_index(0);
                        OtaState::Error
                    } else {
                        // The successful final-header path does not move the
                        // write cursor.
                        OtaState::Ok
                    }
                }
            }
        }
        OtaChunkDecision::RejectError => {
            // Only the size-bound overrun leaves the write cursor alone; the
            // gap, index-0-on-nonzero-cursor and bad-header arms zero it.
            let size_bound_rejection =
                n_data_len != 0 && cur_idx != 0 && cur_idx == last_idx.wrapping_add(1);
            if !size_bound_rejection {
                set_current_flash_address(0);
            }
            set_accepted_index(if n_data_len == 0 { 0 } else { cur_idx });
            OtaState::Error
        }
    };

    if reset_flag != OtaState::Continue {
        // Copy the flag out first: the guard must not span the finish funnel,
        // which takes the same lock again.
        if finish_status() == OtaState::Continue {
            finish_session(reset_flag);
        }
    }

    let finished = finish_status();
    if finished == OtaState::Continue {
        ChunkOutcome::Continue
    } else {
        ChunkOutcome::Finished(finished)
    }
}

/// Abort the transfer because an inbound frame was malformed.
///
/// A malformed frame and a bad CRC take the same arm: cursor to zero, accepted
/// index to zero, transfer finished as an error. The type parameter keeps the
/// adapter call sites uniform even though the abort itself needs no product
/// hook.
pub fn on_frame_error<P: Product>() {
    set_current_flash_address(0);
    set_accepted_index(0);

    if finish_status() == OtaState::Continue {
        finish_session(OtaState::Error);
    }
}

// --- completion --------------------------------------------------------------------

/// Finish the transfer: persist or discard the staged image, tell the product,
/// then reboot.
///
/// The reboot is **unconditional**, matching the deployed finish funnel
/// (`rf_link_slave_ota_finish_led_and_reboot`): `Ok` and `Error` do their flash
/// and indication work, every other state does no flash or indicator work, and
/// the reset latch is written in all cases.
///
/// That unconditional reboot is load-bearing for the mesh supervision timeout. A
/// mesh OTA that expires before any chunk decided a finish state leaves the flag
/// at `Continue`; the deployed tick returned exactly that value and the deployed
/// funnel still rebooted. Returning instead would leave the link permanently
/// blocked with no recovery short of a power cycle. The `Continue` path
/// deliberately leaves the staging area dirty for the next boot's
/// [`super::check_ota_area_startup`] instead of erasing it here.
///
/// The platform never lights an indicator itself; the product hears
/// [`PlatformEvent::OtaCompleted`] / [`PlatformEvent::OtaFailed`] and is then
/// asked for its synchronous pre-reboot indication.
pub fn finish_and_reboot<P: Product>(state: OtaState) {
    match state {
        OtaState::Ok => {
            storage::arm_ready_flag();
            P::on_event(PlatformEvent::OtaCompleted);
            P::on_pre_reboot(RebootIndication::OtaSuccess);
        }
        OtaState::Error => {
            storage::erase_staging_area();
            P::on_event(PlatformEvent::OtaFailed);
            P::on_pre_reboot(RebootIndication::OtaFailure);
        }
        _ => {}
    }

    // The reset is latched only after the product has run its indication, and it
    // is latched for every state: see the unconditional-reboot note above.
    irq_disable();
    mcu_sw_reboot();
}

/// A lost link during a GATT transfer fails the transfer.
pub fn on_link_disconnected<P: Product>() {
    if IN_PROGRESS.get() {
        finish_and_reboot::<P>(OtaState::Error);
    }
}

// --- connection-slot servicing -----------------------------------------------------

/// One second of the GATT supervision timeout, gated on elapsed ticks.
///
/// The radio hands over the current system tick with every connection slot, so
/// this counts real seconds regardless of how often the slot runs.
fn tick_gatt_timeout(current_tick: u32) {
    let last = GATT_OTA_TIMEOUT_TICK.load(Ordering::Relaxed);
    if current_tick.wrapping_sub(last) > CLOCK_SYS_CLOCK_1S {
        GATT_OTA_TIMEOUT_TICK.store(current_tick, Ordering::Relaxed);
        if TIMEOUT_SECONDS.get() != 0 {
            TIMEOUT_SECONDS.dec();
        }
    }
}

/// Run the terminate handshake once the transfer has finished.
///
/// Returns `true` when the reboot deadline was reached; every step that can
/// block (the `PKT_TERMINATE` transmit) runs with no lock held, and the
/// countdown is armed only when the packet really was queued.
fn poll_finish_handshake() -> bool {
    if finish_status() == OtaState::Continue {
        return false;
    }

    let mut reboot_flag = false;

    let terminate_idle = OTA_STATE.lock().terminate_cnt == 0;
    if terminate_idle && TERMINATION_REQUESTED.get() {
        // Blocking RF transmit: no OTA_STATE guard held.
        if tlsr8266_radio::try_send_terminate_packet() {
            OTA_STATE.lock().terminate_cnt = 6;
        }
    }

    {
        let mut state = OTA_STATE.lock();
        if state.terminate_cnt != 0 {
            state.terminate_cnt -= 1;
            if state.terminate_cnt == 0 {
                reboot_flag = true;
            }
        }
    }

    let finished_time = OTA_STATE.lock().finished_time;
    if !TERMINATION_REQUESTED.get() && clock_time_exceed(finished_time, 2_000_000) {
        // The central asked nothing after the transfer: send the notifications
        // anyway so it can leave its own OTA dialog.
        TERMINATION_REQUESTED.set(true);
    }

    if clock_time_exceed(finished_time, 4_000_000) {
        reboot_flag = true;
    }

    reboot_flag
}

/// Service one BLE connection slot for the GATT OTA transfer.
///
/// This is the deployed `handle_ota_operations` plus the composition root's
/// finish dispatch:
///
/// * no GATT transfer — `Continue`, so the radio runs its normal bridge/mesh
///   work for this slot;
/// * transfer running — run the 1 s countdown gate, drain the staged frames,
///   run the terminate handshake, and either `Disconnect` (reboot latched) or
///   `SkipToNextEvent` (the radio must not run bridge/mesh work now).
pub fn poll_connection_slot<P: Product>(current_tick: u32) -> ConnectionPlan {
    if !IN_PROGRESS.get() {
        return ConnectionPlan::Continue;
    }

    tick_gatt_timeout(current_tick);

    // Persist everything staged since the last slot. Blocking flash work runs
    // inside the drain with no OTA_STATE guard held.
    super::gatt::drain_staged_slots::<P>();

    if poll_finish_handshake() {
        // Copy the state out: the guard must die before the erase/indication
        // path runs.
        let finished_state = finish_status();
        finish_and_reboot::<P>(finished_state);
        return ConnectionPlan::Disconnect;
    }

    if is_timeout_expired() {
        // `drain_staged_slots` can refresh the counter, so this check must run
        // after it.
        finish_and_reboot::<P>(OtaState::Error);
        return ConnectionPlan::Disconnect;
    }

    ConnectionPlan::SkipToNextEvent
}

// --- mesh supervision countdown ----------------------------------------------------

/// Advance the mesh-OTA supervision timeout from the `TMR0` interrupt.
///
/// `TMR0` fires at embassy alarm deadlines, so the countdown is gated on elapsed
/// clock ticks rather than on the interrupt rate. Restamping while no mesh OTA
/// is running keeps the first interrupt after a session starts from decrementing
/// immediately. Returns the finish state exactly once, when the counter reaches
/// zero during a transfer.
pub fn on_1s_tick<P: Product>() -> Option<OtaState> {
    let now = clock_time64();
    if !is_mesh_ota_blocked() {
        *LAST_OTA_TIMEOUT_TICK.lock() = now;
        return None;
    }

    {
        let mut last = LAST_OTA_TIMEOUT_TICK.lock();
        if now.saturating_sub(*last) < CLOCK_SYS_CLOCK_1S as u64 {
            return None;
        }
        *last = now;
    }

    if TIMEOUT_SECONDS.get() == 0 {
        return None;
    }
    TIMEOUT_SECONDS.dec();
    if TIMEOUT_SECONDS.get() == 0 {
        // Copy the flag out before returning: the caller runs the completion
        // funnel, which erases the staging area or arms the sentinel, so the
        // guard must not be alive there.
        //
        // The flag is returned **as-is, including `Continue`**, exactly like the
        // deployed tick. A transfer that expired before any chunk decided a
        // finish state therefore reboots without an erase or an indication, and
        // the next boot's `check_ota_area_startup` cleans the dirty staging
        // area. Forcing `Error` here would erase and blink a failure the peer
        // never reported.
        Some(finish_status())
    } else {
        None
    }
}

// --- host-test accessors -----------------------------------------------------------

/// Overwrites the transfer-completion flag.
#[cfg(not(target_os = "none"))]
pub fn set_finished_flag_for_test(state: OtaState) {
    *FINISHED.lock() = state;
}

/// Overwrites the supervision timeout, in seconds.
#[cfg(not(target_os = "none"))]
pub fn set_timeout_for_test(seconds: u16) {
    TIMEOUT_SECONDS.set(seconds);
}

/// Reads the supervision timeout back, in seconds.
#[cfg(not(target_os = "none"))]
pub fn timeout_seconds_for_test() -> u16 {
    TIMEOUT_SECONDS.get()
}

/// Overwrites the GATT-OTA in-progress flag.
#[cfg(not(target_os = "none"))]
pub fn set_in_progress_for_test(in_progress: bool) {
    IN_PROGRESS.set(in_progress);
}

/// Overwrites the mesh-OTA blocked flag.
#[cfg(not(target_os = "none"))]
pub fn set_mesh_ota_blocked_for_test(blocked: bool) {
    MESH_BLOCKED.set(blocked);
}

/// Did the finish path get asked for `PKT_TERMINATE` notifications?
#[cfg(not(target_os = "none"))]
pub fn is_termination_requested() -> bool {
    TERMINATION_REQUESTED.get()
}

/// Overwrites the terminate-request flag.
#[cfg(not(target_os = "none"))]
pub fn set_termination_requested_for_test(requested: bool) {
    TERMINATION_REQUESTED.set(requested);
}

/// Overwrites the staging write cursor.
#[cfg(not(target_os = "none"))]
pub fn set_current_flash_address_for_test(addr: u32) {
    CURRENT_FLASH_ADDRESS.set(addr);
}

/// Reads the staging write cursor back.
#[cfg(not(target_os = "none"))]
pub fn current_flash_address_for_test() -> u32 {
    CURRENT_FLASH_ADDRESS.get()
}

/// Overwrites the `TMR0` tick stamp that gates the countdown.
#[cfg(not(target_os = "none"))]
pub fn set_last_ota_timeout_tick_for_test(tick: u64) {
    *LAST_OTA_TIMEOUT_TICK.lock() = tick;
}

/// Reads the `TMR0` tick stamp back.
#[cfg(not(target_os = "none"))]
pub fn last_ota_timeout_tick_for_test() -> u64 {
    *LAST_OTA_TIMEOUT_TICK.lock()
}

/// Reads the mesh monotonicity cursor.
#[cfg(not(target_os = "none"))]
pub fn last_index_for_test() -> u16 {
    LAST_INDEX.load(Ordering::Relaxed)
}

/// Overwrites the mesh monotonicity cursor.
#[cfg(not(target_os = "none"))]
pub fn set_last_index_for_test(index: u16) {
    LAST_INDEX.store(index, Ordering::Relaxed);
}

// --- mesh session helpers used by the adapters -------------------------------------

/// Start a mesh OTA session: clear the previous session's monotonicity cursor.
pub(crate) fn begin_mesh_session() {
    MESH_BLOCKED.set(true);
    LAST_INDEX.store(u16::MAX, Ordering::Relaxed);
}

/// Mark a GATT session as running and clear the previous monotonicity cursor.
pub(crate) fn begin_gatt_session() {
    IN_PROGRESS.set(true);
    LAST_INDEX.store(u16::MAX, Ordering::Relaxed);
}

/// Read the mesh monotonicity cursor.
pub(crate) fn last_index() -> u16 {
    LAST_INDEX.load(Ordering::Relaxed)
}

/// Publish the mesh monotonicity cursor.
pub(crate) fn set_last_index(index: u16) {
    LAST_INDEX.store(index, Ordering::Relaxed);
}

/// Post-finish supervision timeout: four seconds to reboot.
pub(crate) fn set_post_finish_timeout() {
    TIMEOUT_SECONDS.set(4);
}

/// Ask the finish path to send its `PKT_TERMINATE` notifications.
pub fn request_termination() {
    TERMINATION_REQUESTED.set(true);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ota::test_product::TestProduct;
    use crate::window::{self, OtaClaimant, StagedOtaSlot};
    use mesh_protocol::crc::crc16;
    use mesh_protocol::mesh_types::START_UP_FLAG;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::analog::fake_analog::FakeAnalog;
    use tlsr8266_hal::mcu::clock::fake_clock;
    use tlsr8266_hal::pm::{REGA_RESET_SENTINEL, SENTINEL_CLEAN_REBOOT, SENTINEL_RUNNING};

    const STAGING: u32 = 0x40000;
    const READY_FLAG_ADDR: u32 = 0x3f000;
    const READY_FLAG: u8 = 0xa5;

    fn reset() {
        FakeFlash::reset();
        FakeAnalog::reset();
        reset_for_test();
        fake_clock::reset();
    }

    fn seed_valid_staged_header() {
        FakeFlash::write_slice(STAGING + 8, &START_UP_FLAG.to_le_bytes());
    }

    fn ready_flag() -> u8 {
        let mut value = [0u8; 1];
        FakeFlash::read_slice(READY_FLAG_ADDR, &mut value);
        value[0]
    }

    #[test]
    fn idle_connection_slot_continues() {
        reset();

        assert!(matches!(
            poll_connection_slot::<TestProduct>(1_000),
            ConnectionPlan::Continue
        ));

        reset();
    }

    #[test]
    fn running_connection_slot_skips_bridge_work() {
        reset();
        set_in_progress_for_test(true);

        assert!(matches!(
            poll_connection_slot::<TestProduct>(1_000),
            ConnectionPlan::SkipToNextEvent
        ));
        assert_eq!(finish_status(), OtaState::Continue);

        reset();
    }

    #[test]
    fn gatt_timeout_decrements_once_per_elapsed_second() {
        reset();
        set_in_progress_for_test(true);

        // The gate opens only once a whole second has elapsed since the stamp.
        let _ = poll_connection_slot::<TestProduct>(0);
        assert_eq!(
            timeout_seconds_for_test(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS
        );

        let _ = poll_connection_slot::<TestProduct>(CLOCK_SYS_CLOCK_1S);
        assert_eq!(
            timeout_seconds_for_test(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS
        );

        let _ = poll_connection_slot::<TestProduct>(2 * CLOCK_SYS_CLOCK_1S);
        assert_eq!(
            timeout_seconds_for_test(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS - 1
        );

        let _ = poll_connection_slot::<TestProduct>(4 * CLOCK_SYS_CLOCK_1S);
        assert_eq!(
            timeout_seconds_for_test(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS - 2
        );

        reset();
    }

    #[test]
    fn expired_timeout_disconnects_and_erases_the_staging_area() {
        reset();
        set_in_progress_for_test(true);
        set_timeout_for_test(1);
        FakeFlash::write_slice(STAGING, &[0x11, 0x22]);

        assert!(matches!(
            poll_connection_slot::<TestProduct>(2 * CLOCK_SYS_CLOCK_1S),
            ConnectionPlan::Disconnect
        ));

        assert_eq!(
            FakeFlash::erase_count(STAGING),
            1,
            "a timed-out transfer discards the staged image"
        );
        let mut staged = [0u8; 2];
        FakeFlash::read_slice(STAGING, &mut staged);
        assert_eq!(staged, [0xff; 2]);

        reset();
    }

    #[test]
    fn terminate_countdown_arms_at_six_and_reboots() {
        reset();
        seed_valid_staged_header();
        set_in_progress_for_test(true);
        set_finished_flag_for_test(OtaState::Ok);
        set_termination_requested_for_test(true);
        // The finished stamp is "now", so neither deadline fires yet.
        fake_clock::set_ticks(0);

        // The transmit reports `true` on the host fake, so the countdown arms at
        // six and decrements once per slot.
        for _ in 0..5 {
            assert!(matches!(
                poll_connection_slot::<TestProduct>(0),
                ConnectionPlan::SkipToNextEvent
            ));
        }
        assert!(matches!(
            poll_connection_slot::<TestProduct>(0),
            ConnectionPlan::Disconnect
        ));
        assert_eq!(
            ready_flag(),
            READY_FLAG,
            "the Ok finish arms the boot-time sentinel"
        );

        reset();
    }

    #[test]
    fn four_second_deadline_reboots_without_a_terminate_request() {
        reset();
        seed_valid_staged_header();
        set_in_progress_for_test(true);
        set_finished_flag_for_test(OtaState::Ok);
        set_termination_requested_for_test(false);
        fake_clock::set_ticks(0);

        // Before two seconds: still servicing.
        assert!(matches!(
            poll_connection_slot::<TestProduct>(0),
            ConnectionPlan::SkipToNextEvent
        ));
        // Past four seconds: the forced reboot deadline wins.
        fake_clock::set_ticks(4 * CLOCK_SYS_CLOCK_1S as u64 + 1);
        assert!(matches!(
            poll_connection_slot::<TestProduct>(0),
            ConnectionPlan::Disconnect
        ));
        assert_eq!(ready_flag(), READY_FLAG);

        reset();
    }

    #[test]
    fn two_second_deadline_arms_the_terminate_request() {
        reset();
        set_in_progress_for_test(true);
        set_finished_flag_for_test(OtaState::Error);
        set_termination_requested_for_test(false);
        set_timeout_for_test(30);
        fake_clock::set_ticks(0);

        let _ = poll_connection_slot::<TestProduct>(0);
        assert!(!is_termination_requested());

        fake_clock::set_ticks(2 * CLOCK_SYS_CLOCK_1S as u64 + 1);
        let _ = poll_connection_slot::<TestProduct>(0);
        assert!(
            is_termination_requested(),
            "the iOS fallback asks for the terminate notification"
        );

        reset();
    }

    #[test]
    fn failed_link_disconnect_discards_the_staged_image() {
        reset();
        set_in_progress_for_test(true);
        FakeFlash::write_slice(STAGING, &[0x33]);

        on_link_disconnected::<TestProduct>();

        let mut staged = [0u8; 1];
        FakeFlash::read_slice(STAGING, &mut staged);
        assert_eq!(staged, [0xff]);

        reset();
    }

    #[test]
    fn link_disconnect_without_a_transfer_does_nothing() {
        reset();
        FakeFlash::write_slice(STAGING, &[0x33]);

        on_link_disconnected::<TestProduct>();

        let mut staged = [0u8; 1];
        FakeFlash::read_slice(STAGING, &mut staged);
        assert_eq!(staged, [0x33]);

        reset();
    }

    #[test]
    fn mesh_ticks_count_real_seconds_and_finish_once() {
        reset();
        set_mesh_ota_blocked_for_test(true);
        set_timeout_for_test(2);
        set_last_ota_timeout_tick_for_test(0);
        fake_clock::set_ticks(0);

        assert_eq!(on_1s_tick::<TestProduct>(), None);
        assert_eq!(timeout_seconds_for_test(), 2);

        fake_clock::set_ticks(CLOCK_SYS_CLOCK_1S as u64);
        assert_eq!(on_1s_tick::<TestProduct>(), None);
        assert_eq!(timeout_seconds_for_test(), 1);

        fake_clock::set_ticks(2 * CLOCK_SYS_CLOCK_1S as u64);
        assert_eq!(
            on_1s_tick::<TestProduct>(),
            Some(OtaState::Continue),
            "the deployed tick returned the flag as-is, even with no decision"
        );
        assert_eq!(timeout_seconds_for_test(), 0);
        assert_eq!(
            FakeFlash::erase_count(STAGING),
            0,
            "a timeout with no decision leaves the staging area for the next boot"
        );

        // Once the counter is zero the countdown reports nothing more.
        fake_clock::set_ticks(3 * CLOCK_SYS_CLOCK_1S as u64);
        assert_eq!(on_1s_tick::<TestProduct>(), None);

        reset();
    }

    #[test]
    fn idle_mesh_tick_restamps_without_decrementing() {
        reset();
        fake_clock::set_ticks(5 * CLOCK_SYS_CLOCK_1S as u64);

        assert_eq!(on_1s_tick::<TestProduct>(), None);
        assert_eq!(
            last_ota_timeout_tick_for_test(),
            5 * CLOCK_SYS_CLOCK_1S as u64
        );
        assert_eq!(
            timeout_seconds_for_test(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS
        );

        reset();
    }

    /// The deployed finish funnel latched a reboot for **every** state: `Ok` and
    /// `Error` do their flash/indication work, anything else does neither, and
    /// the reset still fires.
    ///
    /// This is what keeps a stalled mesh transfer from blocking the link
    /// forever: the legacy timeout path returned `Continue` and the root still
    /// rebooted (no erase, no indication).
    #[test]
    fn unanswered_finish_state_reboots_without_erasing_the_staging_area() {
        reset();
        FakeFlash::write_slice(STAGING, &[0x11, 0x22]);
        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_RUNNING);

        finish_and_reboot::<TestProduct>(OtaState::Continue);

        assert_eq!(
            FakeFlash::erase_count(STAGING),
            0,
            "an unanswered finish must not erase the staging area"
        );
        assert_eq!(
            FakeAnalog::get(REGA_RESET_SENTINEL),
            SENTINEL_CLEAN_REBOOT,
            "the reset latch is written even without a decision"
        );

        reset();
    }

    /// Legacy `HEAD:rust/src/ota.rs:348-358`: a chunk that only overruns the
    /// staging bound on the `last + 1` path is rejected **without** moving the
    /// write cursor, so a retransmit rewrites the same address instead of
    /// restarting the image.
    #[test]
    fn size_bound_rejection_keeps_the_cursor() {
        reset();

        // Start a real image so the accepted index is 0.
        let first = [0xA5u8; 16];
        assert_eq!(
            process_chunk::<TestProduct>(
                OtaTransport::Gatt,
                OtaChunk {
                    index: 0,
                    payload: &first,
                    final_chunk: false,
                },
            ),
            ChunkOutcome::Continue
        );

        // Park the cursor one byte past the bound for one 16-byte chunk, then
        // send the in-sequence next chunk so the saturating bound check trips.
        let bound = storage::OtaFlashConfig::compatibility().max_image_bytes;
        let parked = bound - 16 + 1;
        set_current_flash_address_for_test(parked);
        let next = [0x5Au8; 16];

        assert_eq!(
            process_chunk::<TestProduct>(
                OtaTransport::Gatt,
                OtaChunk {
                    index: 1,
                    payload: &next,
                    final_chunk: false,
                },
            ),
            ChunkOutcome::Finished(OtaState::Error)
        );
        assert_eq!(
            current_flash_address_for_test(),
            parked,
            "the size-bound rejection must not reset the write cursor"
        );

        reset();
    }

    /// Legacy `HEAD:rust/src/ota.rs:607-612`: a final header whose advertised
    /// size rounds up to fewer than three chunks is an invalid image.
    ///
    /// This is the isolating form the deleted legacy test used: the accepted
    /// cursor is 0 and the header arrives at index 1, so
    /// `ota_pkt_total == expected_index == 1` and **only** the `< 3` floor can
    /// reject the image.
    #[test]
    fn final_header_under_three_chunks_is_rejected() {
        reset();

        // Accept one real 8-byte mesh chunk at index 0: accepted cursor 0.
        let first = [0xA5u8; 8];
        assert_eq!(
            process_chunk::<TestProduct>(
                OtaTransport::Mesh,
                OtaChunk {
                    index: 0,
                    payload: &first,
                    final_chunk: false,
                },
            ),
            ChunkOutcome::Continue
        );

        // One 8-byte chunk image: div_ceil(8, 8) == 1, which is exactly the
        // index the header will arrive at. The floor is the only rejection.
        FakeFlash::write_slice(STAGING + 0x18, &8u32.to_le_bytes());

        assert_eq!(
            process_chunk::<TestProduct>(
                OtaTransport::Mesh,
                OtaChunk {
                    index: 1,
                    payload: &[],
                    final_chunk: true,
                },
            ),
            ChunkOutcome::Finished(OtaState::Error)
        );
        assert_eq!(finish_status(), OtaState::Error);
        assert_eq!(
            current_flash_address_for_test(),
            0,
            "an undersized final header resets the write cursor"
        );

        reset();
    }

    /// Legacy root ordering (`rust/src/irq.rs:131-137`): the save pass refreshes
    /// the counter **before** the root checks it, so a drain that lands a chunk
    /// must suppress the post-drain error reboot.
    #[test]
    fn drain_refresh_suppresses_the_error_reboot() {
        reset();
        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_RUNNING);
        assert!(window::begin_ota(OtaClaimant::Gatt, || true).is_ok());
        set_in_progress_for_test(true);
        // The counter is already exhausted before the slot runs.
        set_timeout_for_test(0);

        // One well-formed bridge frame, staged exactly as the GATT adapter does.
        let mut slot = StagedOtaSlot::empty();
        slot.l2cap_len = 23;
        let mut frame = [0u8; 20];
        frame[2..18].fill(0x3C);
        let crc = crc16(&frame[0..18]).to_le_bytes();
        frame[18..20].copy_from_slice(&crc);
        slot.data[..20].copy_from_slice(&frame);
        window::stage_ota_slot(0, &slot);
        set_staged_slot_count(1);

        let plan = poll_connection_slot::<TestProduct>(0);

        assert!(
            matches!(plan, ConnectionPlan::SkipToNextEvent),
            "a refreshed counter must skip the slot, not disconnect"
        );
        assert_eq!(
            timeout_seconds_for_test(),
            RF_SLAVE_OTA_TIMEOUT_DEFAULT_SECONDS,
            "the drain refreshes the counter"
        );
        assert_eq!(
            FakeFlash::erase_count(STAGING),
            0,
            "the refreshed transfer must not be discarded"
        );
        assert_eq!(
            FakeAnalog::get(REGA_RESET_SENTINEL),
            SENTINEL_RUNNING,
            "no reboot may be latched"
        );
        assert_eq!(current_flash_address_for_test(), 16);

        window::end_ota();
        reset();
    }
}
