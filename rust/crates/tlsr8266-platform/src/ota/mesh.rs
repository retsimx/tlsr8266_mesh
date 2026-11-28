//! Mesh OTA adapter: the start/data/end command path.
//!
//! The mesh command handler hands over the 10-byte parameter block (index plus
//! the 8-byte chunk) and gets the accepted index back. The chunk is converted
//! straight into an [`OtaChunk`]; no `Packet` or `PacketAttData` is fabricated
//! anywhere on this path.

use mesh_protocol::mesh_types::OtaState;

use crate::api::Product;
use crate::config::FirmwareIdentity;
use crate::window::{self, OtaClaimant};

use super::engine::{self, OtaChunk, OtaTransport};

/// Parameter block of a mesh OTA data/end command: 2 index bytes plus the
/// 8-byte chunk.
const MESH_PARAMS_LEN: usize = 10;
/// Payload width of one mesh OTA chunk.
const MESH_CHUNK_BYTES: usize = 8;

/// Start a mesh OTA session.
///
/// Returns the firmware version bytes the caller stamps into the start response,
/// and whether the session was actually claimed. The version is reported whether
/// or not the claim succeeded, because the response always carries it.
pub fn start_ota<P: Product>(
    identity: FirmwareIdentity,
    abort_status: impl FnOnce() -> bool,
) -> ([u8; 4], bool) {
    // The downgrade decision is host-side by design (`flash_fw.py` refuses and
    // `--force` overrides); the device only reports its own version here.
    let version = identity.firmware_version.to_le_bytes();

    if engine::finish_status() != OtaState::Continue || engine::is_gatt_ota_in_progress() {
        return (version, false);
    }

    if window::begin_ota(OtaClaimant::Mesh, abort_status).is_err() {
        return (version, false);
    }

    engine::begin_mesh_session();
    tlsr8266_hal::pm::set_reboot_ota_active(true);
    (version, true)
}

/// Handle one mesh OTA data or end command.
///
/// Returns the index the packet was accepted under. A stale or duplicate index is
/// answered with the last accepted index and does **not** abort the transfer —
/// mesh retransmission relies on that. `u16::MAX` means the packet never reached
/// the staging path (no mesh session, or another transport owns the window).
pub fn ota_data<P: Product>(params: &[u8], final_chunk: bool) -> u16 {
    if engine::finish_status() != OtaState::Continue
        || engine::is_gatt_ota_in_progress()
        || !engine::is_mesh_ota_blocked()
        || window::active_ota_claimant() != Some(OtaClaimant::Mesh)
    {
        return u16::MAX;
    }

    // The deployed handler copied the parameter block into a default packet, so
    // a short block was zero-padded rather than refused. Keep that exactly.
    let mut block = [0u8; MESH_PARAMS_LEN];
    let copy_len = params.len().min(MESH_PARAMS_LEN);
    block[..copy_len].copy_from_slice(&params[..copy_len]);

    let index = block[0] as u16 | (block[1] as u16) << 8;

    let last_index = engine::last_index();
    if last_index == u16::MAX || last_index < index {
        engine::set_last_index(index);

        let chunk = OtaChunk {
            index: index as u32,
            payload: if final_chunk {
                &[]
            } else {
                &block[2..2 + MESH_CHUNK_BYTES]
            },
            final_chunk,
        };
        let _ = engine::process_chunk::<P>(OtaTransport::Mesh, chunk);

        // `process_chunk` already refreshed the timeout; shorten it once the
        // transfer has finished.
        if engine::finish_status() != OtaState::Continue {
            engine::set_post_finish_timeout();
        }

        // Report the index this packet was staged under. The finish funnel clears
        // the cursor on an abort so the *next* session starts at index 0;
        // re-reading it here would report the abort, not the packet.
        return index;
    }

    last_index
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ota::test_product::TestProduct;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::clock::fake_clock;

    const STAGING: u32 = 0x40000;
    const IDENTITY: FirmwareIdentity = FirmwareIdentity::new(0x1234, 0x0001, 0x1122_3344);

    fn reset() {
        FakeFlash::reset();
        engine::reset_for_test();
        fake_clock::reset();
    }

    fn start() {
        let (version, claimed) = start_ota::<TestProduct>(IDENTITY, || true);
        assert!(claimed);
        assert_eq!(version, IDENTITY.firmware_version.to_le_bytes());
    }

    /// One 10-byte parameter block: index plus 8 payload bytes.
    fn params(index: u16, fill: u8) -> [u8; MESH_PARAMS_LEN] {
        let mut block = [fill; MESH_PARAMS_LEN];
        block[0..2].copy_from_slice(&index.to_le_bytes());
        block
    }

    #[test]
    fn start_reports_the_identity_version_and_claims_the_window() {
        reset();

        let (version, claimed) = start_ota::<TestProduct>(IDENTITY, || true);

        assert!(claimed);
        assert_eq!(version, 0x1122_3344u32.to_le_bytes());
        assert!(engine::is_mesh_ota_blocked());
        assert_eq!(window::active_ota_claimant(), Some(OtaClaimant::Mesh));

        reset();
    }

    #[test]
    fn start_is_refused_while_a_gatt_transfer_runs() {
        reset();
        engine::set_in_progress_for_test(true);

        let (version, claimed) = start_ota::<TestProduct>(IDENTITY, || true);

        assert!(!claimed);
        assert_eq!(version, IDENTITY.firmware_version.to_le_bytes());
        assert!(!engine::is_mesh_ota_blocked());

        reset();
    }

    #[test]
    fn start_is_refused_when_the_status_abort_fails() {
        reset();
        assert!(window::start_status_window());

        let (version, claimed) = start_ota::<TestProduct>(IDENTITY, || false);

        assert!(!claimed);
        assert_eq!(version, IDENTITY.firmware_version.to_le_bytes());
        assert!(!engine::is_mesh_ota_blocked());
        assert_eq!(
            window::active_ota_claimant(),
            None,
            "a refused start must not claim the window"
        );

        // The status window survived, so a later start that can abort it works.
        let (_, claimed) = start_ota::<TestProduct>(IDENTITY, || true);
        assert!(claimed);
        assert_eq!(window::active_ota_claimant(), Some(OtaClaimant::Mesh));

        reset();
    }

    #[test]
    fn monotonic_chunks_are_accepted_and_programmed() {
        reset();
        start();

        assert_eq!(ota_data::<TestProduct>(&params(0, 0x11), false), 0);
        assert_eq!(ota_data::<TestProduct>(&params(1, 0x22), false), 1);
        assert_eq!(ota_data::<TestProduct>(&params(2, 0x33), false), 2);

        let mut chunk = [0u8; MESH_CHUNK_BYTES];
        FakeFlash::read_slice(STAGING, &mut chunk);
        assert_eq!(chunk, [0x11; MESH_CHUNK_BYTES]);
        FakeFlash::read_slice(STAGING + 16, &mut chunk);
        assert_eq!(chunk, [0x33; MESH_CHUNK_BYTES]);
        assert_eq!(engine::current_flash_address(), 24);
        assert_eq!(engine::finish_status(), OtaState::Continue);

        reset();
    }

    #[test]
    fn stale_retransmit_returns_the_last_index_without_aborting() {
        reset();
        start();

        assert_eq!(ota_data::<TestProduct>(&params(0, 0x11), false), 0);
        assert_eq!(ota_data::<TestProduct>(&params(1, 0x22), false), 1);
        // Duplicate of the packet just accepted.
        assert_eq!(ota_data::<TestProduct>(&params(1, 0x22), false), 1);
        // Older still.
        assert_eq!(ota_data::<TestProduct>(&params(0, 0x11), false), 1);

        assert_eq!(
            engine::finish_status(),
            OtaState::Continue,
            "a stale retransmit must not abort the transfer"
        );
        assert_eq!(engine::current_flash_address(), 16, "nothing was rewritten");

        reset();
    }

    #[test]
    fn final_header_finishes_ok_when_the_size_matches() {
        reset();
        start();

        // Chunk 3 lands on header offset 0x18, which is where the image-size
        // word lives: four 8-byte chunks advertise 32 bytes.
        for index in 0..4u16 {
            let mut block = params(index, 0x44);
            if index == 3 {
                block[2..6].copy_from_slice(&32u32.to_le_bytes());
            }
            assert_eq!(ota_data::<TestProduct>(&block, false), index);
        }

        let mut header = [0u8; MESH_PARAMS_LEN];
        header[0] = 4;
        assert_eq!(ota_data::<TestProduct>(&header, true), 4);

        assert_eq!(engine::finish_status(), OtaState::Ok);
        assert_eq!(
            engine::timeout_seconds_for_test(),
            4,
            "a finished transfer reboots within four seconds"
        );
        assert_eq!(window::active_ota_claimant(), None);

        reset();
    }

    #[test]
    fn final_header_size_mismatch_finishes_error() {
        reset();
        start();

        for index in 0..4u16 {
            let mut block = params(index, 0x44);
            if index == 3 {
                // 40 bytes advertises five chunks, but the header arrives at 4.
                block[2..6].copy_from_slice(&40u32.to_le_bytes());
            }
            assert_eq!(ota_data::<TestProduct>(&block, false), index);
        }

        let mut header = [0u8; MESH_PARAMS_LEN];
        header[0] = 4;
        assert_eq!(ota_data::<TestProduct>(&header, true), 4);

        assert_eq!(engine::finish_status(), OtaState::Error);
        assert_eq!(engine::current_flash_address(), 0);

        reset();
    }

    #[test]
    fn gap_rejection_resets_the_write_cursor() {
        reset();
        start();
        assert_eq!(ota_data::<TestProduct>(&params(0, 0x11), false), 0);

        assert_eq!(ota_data::<TestProduct>(&params(5, 0x55), false), 5);

        assert_eq!(engine::finish_status(), OtaState::Error);
        assert_eq!(engine::current_flash_address(), 0);

        reset();
    }

    /// The mesh filter is monotonic within a session: once a high index has been
    /// accepted, index 0 is a stale retransmit and is answered with the last
    /// accepted index instead of aborting. The `0xffff -> 0` wrap exists only in
    /// the chunk decision table, not in the mesh retransmit filter.
    #[test]
    fn high_index_makes_index_zero_stale_without_aborting() {
        reset();
        start();

        engine::set_last_index_for_test(0xfffe);
        assert_eq!(ota_data::<TestProduct>(&params(0, 0x11), false), 0xfffe);

        assert_eq!(
            engine::finish_status(),
            OtaState::Continue,
            "a stale index must not abort the transfer"
        );
        assert_eq!(
            engine::current_flash_address(),
            0,
            "a stale index must not touch the staging area"
        );

        reset();
    }

    #[test]
    fn mesh_data_is_refused_outside_a_mesh_session() {
        reset();

        assert_eq!(ota_data::<TestProduct>(&params(0, 0x11), false), u16::MAX);

        // A GATT claim is not a mesh claim.
        window::begin_ota(OtaClaimant::Gatt, || true).expect("gatt claim");
        assert_eq!(ota_data::<TestProduct>(&params(0, 0x11), false), u16::MAX);
        window::end_ota();

        reset();
    }

    #[test]
    fn short_parameter_block_is_zero_padded_like_legacy() {
        reset();
        start();

        // The deployed handler copied the block into a default packet, so a
        // one-byte block is index 0 with a zero-filled chunk, not a refusal.
        assert_eq!(ota_data::<TestProduct>(&[0x00], false), 0);
        assert_eq!(engine::finish_status(), OtaState::Continue);

        let mut chunk = [0xffu8; MESH_CHUNK_BYTES];
        FakeFlash::read_slice(STAGING, &mut chunk);
        assert_eq!(chunk, [0u8; MESH_CHUNK_BYTES]);
        assert_eq!(engine::current_flash_address(), MESH_CHUNK_BYTES as u32);

        reset();
    }

    /// Legacy `HEAD:rust/src/ota.rs` (abort funnel `:751-759`, session start
    /// `:887-889`): `LAST_INDEX` returns to `u16::MAX` at the session start and
    /// at the abort funnel, so the next session accepts index 0 again instead of
    /// treating it as a stale retransmit.
    #[test]
    fn last_index_resets_across_sessions() {
        reset();
        start();

        // Index 5 on an empty accepted cursor is a gap: the transfer aborts and
        // the finish funnel clears the monotonicity cursor.
        assert_eq!(ota_data::<TestProduct>(&params(5, 0x55), false), 5);
        assert_eq!(engine::finish_status(), OtaState::Error);
        assert_eq!(
            engine::last_index_for_test(),
            u16::MAX,
            "the abort funnel resets the monotonicity cursor"
        );

        // A second session in the same boot must accept index 0.
        engine::set_finished_flag_for_test(OtaState::Continue);
        start();
        assert_eq!(
            engine::last_index_for_test(),
            u16::MAX,
            "a session start resets the monotonicity cursor"
        );
        assert_eq!(
            ota_data::<TestProduct>(&params(0, 0xA5), false),
            0,
            "the next session must accept index 0"
        );
        assert_eq!(engine::finish_status(), OtaState::Continue);

        let mut programmed = [0u8; MESH_CHUNK_BYTES];
        FakeFlash::read_slice(STAGING, &mut programmed);
        assert_eq!(programmed, [0xA5; MESH_CHUNK_BYTES]);

        reset();
    }

    /// Legacy `HEAD:rust/src/ota.rs:887-889`: `start_ota` clears the
    /// monotonicity cursor, so a new session accepts index 0 even when the
    /// previous session left a real index behind.
    ///
    /// This isolates the **session-start** reset: the cursor is a non-`u16::MAX`
    /// value before `start()`, so removing the reset would leave `last == 7`
    /// and the monotonic filter (`7 < 0` is false) would answer index 0 with 7.
    #[test]
    fn session_start_resets_the_last_index_cursor() {
        reset();
        start();

        // End the session and leave a real index behind, as a previous transfer
        // would have.
        window::end_ota();
        engine::set_last_index_for_test(7);
        assert_eq!(engine::last_index_for_test(), 7);

        start();
        assert_eq!(
            engine::last_index_for_test(),
            u16::MAX,
            "the session start resets the monotonicity cursor"
        );
        assert_eq!(
            ota_data::<TestProduct>(&params(0, 0xA5), false),
            0,
            "index 0 must be accepted, not filtered as a stale retransmit"
        );
        assert_eq!(engine::finish_status(), OtaState::Continue);

        reset();
    }

    /// Legacy `HEAD:rust/src/ota.rs:613-616`: the successful final-header path
    /// deliberately leaves the write cursor untouched.
    #[test]
    fn final_header_success_keeps_the_flash_cursor() {
        reset();
        start();

        // Four 8-byte chunks; chunk 3 lands on header offset 0x18 and advertises
        // exactly 32 bytes, so the header's expected index (4) matches.
        for index in 0..4u16 {
            let mut block = params(index, 0x44);
            if index == 3 {
                block[2..6].copy_from_slice(&32u32.to_le_bytes());
            }
            assert_eq!(ota_data::<TestProduct>(&block, false), index);
        }

        // A distinctive non-zero cursor: the success path must not move it.
        engine::set_current_flash_address_for_test(24);

        let mut header = [0u8; MESH_PARAMS_LEN];
        header[0] = 4;
        assert_eq!(ota_data::<TestProduct>(&header, true), 4);

        assert_eq!(engine::finish_status(), OtaState::Ok);
        assert_eq!(
            engine::current_flash_address_for_test(),
            24,
            "the successful final header must leave the write cursor alone"
        );

        reset();
    }
}
