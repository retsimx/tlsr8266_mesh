//! GATT OTA adapter: stage inbound writes and drain them through the engine.
//!
//! The radio hands over an ATT write's L2CAP length plus its payload; it owns no
//! OTA policy. Bounds and CRC are validated here, exactly where the deployed
//! save loop validated them, and everything else is the shared engine.

use mesh_protocol::crc::crc16;
use mesh_protocol::mesh_types::OtaState;

use crate::api::{PlatformEvent, Product};
use crate::window::{self, OtaClaimant, StagedOtaSlot, OTA_SLOT_DATA_MAX, PACKET_WINDOW_SLOTS};

use super::engine::{self, ChunkOutcome, OtaChunk, OtaFrameError, OtaTransport};

/// Fixed part of an OTA frame head: the L2CAP length counts this much more than
/// the ATT payload.
const FRAME_HEAD_LEN: usize = 7;
/// Payload width of one GATT OTA chunk.
const GATT_CHUNK_BYTES: usize = 16;

/// Handle one GATT write to the OTA attribute.
///
/// Mirrors the deployed write handler: once the session has started, claim the
/// next staging slot and copy the frame into the shared window. A full window
/// drops the packet instead of claiming a slot past its depth.
pub fn on_write<P: Product>(l2cap_len: u16, payload: &[u8]) {
    if engine::finish_status() != OtaState::Continue || engine::is_mesh_ota_blocked() {
        return;
    }

    if !engine::is_gatt_ota_in_progress() {
        if !tlsr8266_radio::is_paired_login() {
            return;
        }

        if window::begin_ota(OtaClaimant::Gatt, || {
            tlsr8266_radio::abort_slave_read_status_if_busy();
            true
        })
        .is_err()
        {
            return;
        }

        engine::begin_gatt_session();
        tlsr8266_hal::pm::set_reboot_ota_active(true);
        P::on_event(PlatformEvent::OtaStarted);
    }

    // Claim the slot under a short critical section, then drop the guard before
    // touching the window (a second, independent lock).
    let Some(index) = engine::claim_staged_slot() else {
        return;
    };

    let mut slot = StagedOtaSlot::empty();
    slot.l2cap_len = l2cap_len;
    let copy_len = payload.len().min(OTA_SLOT_DATA_MAX);
    slot.data[..copy_len].copy_from_slice(&payload[..copy_len]);

    window::stage_ota_slot(index, &slot);
}

/// Handle a GATT read of the OTA attribute.
///
/// A read after the transfer finished is the initiator asking for the terminate
/// notifications, so it arms them.
pub fn on_read() {
    if engine::finish_status() != OtaState::Continue {
        engine::request_termination();
    }
}

/// Decode one staged frame into an engine chunk.
///
/// The bounds are the peer-controlled ones the deployed save path used:
/// `l2cap_len` must cover the fixed head, and the payload plus its trailing CRC
/// pair must fit the ATT buffer. `payload` is the full 16-byte chunk for a data
/// frame and empty for the final header.
pub fn decode_staged_slot(slot: &StagedOtaSlot) -> Result<OtaChunk<'_>, OtaFrameError> {
    let l2cap_len = slot.l2cap_len as usize;
    if l2cap_len < FRAME_HEAD_LEN {
        return Err(OtaFrameError::MalformedFrame);
    }

    let n_data_len = l2cap_len - FRAME_HEAD_LEN;
    if n_data_len + 4 > slot.data.len() {
        return Err(OtaFrameError::MalformedFrame);
    }

    let expected = slot.data[n_data_len + 2] as u16 | (slot.data[n_data_len + 3] as u16) << 8;
    if crc16(&slot.data[0..n_data_len + 2]) != expected {
        return Err(OtaFrameError::BadCrc);
    }

    let index = slot.data[0] as u32 | (slot.data[1] as u32) << 8;
    if n_data_len == 0 {
        Ok(OtaChunk {
            index,
            payload: &[],
            final_chunk: true,
        })
    } else {
        Ok(OtaChunk {
            index,
            // Exactly what the deployed save path wrote for a bridge frame: a
            // fixed 16-byte chunk, whatever `l2cap_len` said the payload was.
            payload: &slot.data[2..2 + GATT_CHUNK_BYTES],
            final_chunk: false,
        })
    }
}

/// Persist every frame staged since the last connection slot.
///
/// The stop conditions mirror the deployed save loop: the first chunk that
/// finishes the transfer (or fails to decode) ends the pass. The staging count
/// is cleared afterwards either way, so a failed frame is never replayed.
pub(crate) fn drain_staged_slots<P: Product>() {
    let count = engine::staged_slot_count();

    for index in 0..count.min(PACKET_WINDOW_SLOTS) {
        let slot = window::read_staged_ota_slot(index);
        match decode_staged_slot(&slot) {
            Ok(chunk) => {
                if let ChunkOutcome::Finished(_) =
                    engine::process_chunk::<P>(OtaTransport::Gatt, chunk)
                {
                    break;
                }
            }
            Err(_) => {
                engine::on_frame_error::<P>();
                break;
            }
        }
    }

    engine::set_staged_slot_count(0);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ota::test_product::TestProduct;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::clock::fake_clock;

    /// Staging base of the deployed layout.
    const STAGING: u32 = 0x40000;

    fn reset() {
        FakeFlash::reset();
        engine::reset_for_test();
        fake_clock::reset();
        tlsr8266_radio::set_paired_login(false);
    }

    /// Build one bridge frame: index, 16 payload bytes, CRC over the first 18.
    fn bridge_frame(index: u16, fill: u8) -> [u8; 20] {
        let mut frame = [fill; 20];
        frame[0..2].copy_from_slice(&index.to_le_bytes());
        let crc = crc16(&frame[0..18]).to_le_bytes();
        frame[18..20].copy_from_slice(&crc);
        frame
    }

    #[test]
    fn write_stages_and_drain_persists_verified_chunks() {
        reset();
        tlsr8266_radio::set_paired_login(true);

        on_write::<TestProduct>(23, &bridge_frame(0, 0xA5));
        on_write::<TestProduct>(23, &bridge_frame(1, 0x5A));
        assert!(engine::is_gatt_ota_in_progress());

        drain_staged_slots::<TestProduct>();

        let mut chunk = [0u8; 16];
        FakeFlash::read_slice(STAGING, &mut chunk);
        assert_eq!(chunk, [0xA5; 16], "the first frame is programmed verbatim");
        FakeFlash::read_slice(STAGING + 16, &mut chunk);
        assert_eq!(chunk, [0x5A; 16], "the second frame follows it");
        assert_eq!(engine::current_flash_address(), 32);
        assert_eq!(engine::finish_status(), OtaState::Continue);
        assert_eq!(engine::staged_slot_count(), 0);

        reset();
    }

    #[test]
    fn unpaired_write_is_refused_and_claims_nothing() {
        reset();
        tlsr8266_radio::set_paired_login(false);

        on_write::<TestProduct>(23, &bridge_frame(0, 1));

        assert!(!engine::is_gatt_ota_in_progress());
        assert_eq!(window::active_ota_claimant(), None);
        assert_eq!(engine::staged_slot_count(), 0);

        reset();
    }

    #[test]
    fn bad_crc_aborts_the_transfer_and_zeroes_the_cursor() {
        reset();
        tlsr8266_radio::set_paired_login(true);
        engine::set_current_flash_address(16);

        let mut frame = bridge_frame(0, 0x11);
        frame[18] ^= 0xff;
        on_write::<TestProduct>(23, &frame);

        drain_staged_slots::<TestProduct>();

        assert_eq!(engine::finish_status(), OtaState::Error);
        assert_eq!(engine::current_flash_address(), 0);
        assert_eq!(
            window::active_ota_claimant(),
            None,
            "the finish funnel releases the window claim"
        );

        reset();
    }

    #[test]
    fn short_frame_head_is_malformed() {
        reset();
        tlsr8266_radio::set_paired_login(true);

        // L2CAP length below the 7-byte fixed head.
        on_write::<TestProduct>(6, &[0; 8]);
        drain_staged_slots::<TestProduct>();

        assert_eq!(engine::finish_status(), OtaState::Error);
        assert_eq!(engine::current_flash_address(), 0);

        reset();
    }

    #[test]
    fn oversized_payload_is_malformed() {
        reset();
        tlsr8266_radio::set_paired_login(true);

        // n_data_len = 20, so the CRC pair would start past the 23-byte buffer.
        on_write::<TestProduct>(23 + 4, &[0; 20]);
        drain_staged_slots::<TestProduct>();

        assert_eq!(engine::finish_status(), OtaState::Error);

        reset();
    }

    #[test]
    fn full_window_stops_at_the_owned_depth() {
        reset();
        tlsr8266_radio::set_paired_login(true);
        let before = window::ota_dropped_count();

        for index in 0..(PACKET_WINDOW_SLOTS as u16 + 1) {
            on_write::<TestProduct>(23, &bridge_frame(index, 0x33));
        }

        assert_eq!(engine::staged_slot_count(), PACKET_WINDOW_SLOTS);
        assert_eq!(
            window::ota_dropped_count(),
            before,
            "a full staging window drops the write without claiming a slot"
        );

        reset();
    }

    #[test]
    fn read_after_finish_arms_the_terminate_request() {
        reset();
        engine::set_finished_flag_for_test(OtaState::Ok);

        on_read();
        assert!(engine::is_termination_requested());

        reset();
    }

    #[test]
    fn read_during_transfer_arms_nothing() {
        reset();

        on_read();
        assert!(!engine::is_termination_requested());

        reset();
    }

    #[test]
    fn decode_reads_index_and_final_header() {
        reset();

        let mut frame = StagedOtaSlot::empty();
        frame.l2cap_len = 23;
        frame.data[..20].copy_from_slice(&bridge_frame(7, 0x7E));
        let chunk = decode_staged_slot(&frame).expect("valid bridge frame");
        assert_eq!(chunk.index, 7);
        assert_eq!(chunk.payload, &[0x7E; 16]);
        assert!(!chunk.final_chunk);

        let mut header = StagedOtaSlot::empty();
        header.l2cap_len = 7;
        header.data[0..2].copy_from_slice(&9u16.to_le_bytes());
        let crc = crc16(&header.data[0..2]).to_le_bytes();
        header.data[2..4].copy_from_slice(&crc);
        let chunk = decode_staged_slot(&header).expect("valid final header");
        assert_eq!(chunk.index, 9);
        assert!(chunk.payload.is_empty());
        assert!(chunk.final_chunk);
    }
}
