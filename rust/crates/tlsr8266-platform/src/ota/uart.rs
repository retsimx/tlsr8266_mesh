//! UART OTA adapter (host-tested only).
//!
//! The host tooling's UART transport carries the same 10-byte frame the mesh
//! command carries — a 16-bit index and an 8-byte chunk, no per-chunk CRC — and a
//! 2-byte final header. This adapter decodes those frames into the shared engine
//! and is deliberately **not** wired to the production UART receiver yet.

use mesh_protocol::mesh_types::OtaState;

use crate::api::Product;
use crate::window::{self, OtaClaimant};

use super::engine::{self, OtaChunk, OtaFrameError, OtaTransport};

/// Bytes of one UART OTA data frame.
pub const UART_PACKET_LEN: usize = 10;
/// Payload width of one UART OTA chunk.
pub const UART_CHUNK_BYTES: usize = 8;
/// Bytes of the final header frame.
const UART_FINAL_LEN: usize = 2;

/// Decode one UART OTA frame.
///
/// A data frame is `[idx_lo, idx_hi, chunk[0..8]]`; the final header is
/// `[idx_lo, idx_hi]`. A short frame cannot be interpreted without reading bytes
/// the caller never supplied, so it is refused instead of zero-padded.
pub fn decode_packet(bytes: &[u8], final_chunk: bool) -> Result<OtaChunk<'_>, OtaFrameError> {
    let needed = if final_chunk {
        UART_FINAL_LEN
    } else {
        UART_PACKET_LEN
    };
    if bytes.len() < needed {
        return Err(OtaFrameError::MalformedFrame);
    }

    let index = bytes[0] as u32 | (bytes[1] as u32) << 8;
    if final_chunk {
        Ok(OtaChunk {
            index,
            payload: &[],
            final_chunk: true,
        })
    } else {
        Ok(OtaChunk {
            index,
            payload: &bytes[2..2 + UART_CHUNK_BYTES],
            final_chunk: false,
        })
    }
}

/// Start a UART OTA session.
pub fn try_start_session<P: Product>(abort_status: impl FnOnce() -> bool) -> bool {
    if engine::finish_status() != OtaState::Continue || engine::is_gatt_ota_in_progress() {
        return false;
    }

    if window::begin_ota(OtaClaimant::Uart, abort_status).is_err() {
        return false;
    }

    tlsr8266_hal::pm::set_reboot_ota_active(true);
    true
}

/// Handle one UART OTA frame and return the accepted index.
///
/// `u16::MAX` means the packet never reached the staging path.
pub fn on_packet<P: Product>(bytes: &[u8], final_chunk: bool) -> u16 {
    if engine::finish_status() != OtaState::Continue
        || engine::is_gatt_ota_in_progress()
        || window::active_ota_claimant() != Some(OtaClaimant::Uart)
    {
        return u16::MAX;
    }

    let Ok(chunk) = decode_packet(bytes, final_chunk) else {
        return u16::MAX;
    };
    let index = chunk.index as u16;

    let _ = engine::process_chunk::<P>(OtaTransport::Uart, chunk);

    // `process_chunk` already refreshed the timeout; shorten it once the
    // transfer has finished.
    if engine::finish_status() != OtaState::Continue {
        engine::set_post_finish_timeout();
    }

    index
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ota::storage::read_staged_bin_size;
    use crate::ota::test_product::TestProduct;
    use tlsr8266_hal::drivers::flash::fake_flash::FakeFlash;
    use tlsr8266_hal::mcu::clock::fake_clock;

    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../tests/fixtures/rust_parser.rs"
    ));

    const STAGING: u32 = 0x40000;

    fn reset() {
        FakeFlash::reset();
        engine::reset_for_test();
        fake_clock::reset();
    }

    fn vectors() -> Parser {
        Parser::parse(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../tests/wire_vectors.txt"
        )))
        .expect("wire vectors parse")
    }

    #[test]
    fn golden_uart_packets_decode_to_their_indices_and_chunks() {
        reset();
        let mut fixture = vectors();
        assert_eq!(
            fixture.u8("ota.uart.chunk_size").unwrap() as usize,
            UART_CHUNK_BYTES
        );

        for index in 0..8u32 {
            let key = format!("ota.uart.packet.{index}");
            let bytes = fixture.bytes(&key).unwrap();
            assert_eq!(bytes.len(), UART_PACKET_LEN);

            let chunk = decode_packet(&bytes, false).expect("golden packet decodes");
            assert_eq!(chunk.index, index);
            assert!(!chunk.final_chunk);
            assert_eq!(chunk.payload, &bytes[2..10]);
        }
    }

    #[test]
    fn full_golden_session_reaches_a_verified_finish() {
        reset();
        let mut fixture = vectors();
        assert!(try_start_session::<TestProduct>(|| true));

        for index in 0..8u32 {
            let key = format!("ota.uart.packet.{index}");
            let bytes = fixture.bytes(&key).unwrap();
            assert_eq!(on_packet::<TestProduct>(&bytes, false), index as u16);
        }

        let mut chunk = [0u8; UART_CHUNK_BYTES];
        FakeFlash::read_slice(STAGING, &mut chunk);
        assert_eq!(chunk, [0xde, 0xad, 0xd7, 0x0d, 0x00, 0x00, 0x00, 0x00]);
        FakeFlash::read_slice(STAGING + 7 * UART_CHUNK_BYTES as u32, &mut chunk);
        assert_eq!(chunk, [0x51, 0x05, 0x82, 0x09, 0x74, 0x94, 0x45, 0x92]);

        // Eight 8-byte chunks; the image header at offset 0x18 carries the size
        // word, so the golden chunk 3 already advertises 64 bytes.
        assert_eq!(read_staged_bin_size(), Ok(64));
        let mut header = [0u8; UART_FINAL_LEN];
        header.copy_from_slice(&8u16.to_le_bytes());
        assert_eq!(on_packet::<TestProduct>(&header, true), 8);

        assert_eq!(engine::finish_status(), OtaState::Ok);
        assert_eq!(engine::staged_slot_count(), 0);
        assert_eq!(window::active_ota_claimant(), None);

        reset();
    }

    #[test]
    fn short_frames_are_malformed() {
        reset();

        assert_eq!(
            decode_packet(&[0u8; UART_PACKET_LEN - 1], false),
            Err(OtaFrameError::MalformedFrame)
        );
        assert_eq!(
            decode_packet(&[0u8; UART_FINAL_LEN - 1], true),
            Err(OtaFrameError::MalformedFrame)
        );
    }

    #[test]
    fn final_header_is_index_only() {
        reset();

        let header = 7u16.to_le_bytes();
        let chunk = decode_packet(&header, true).expect("header decodes");
        assert_eq!(chunk.index, 7);
        assert!(chunk.final_chunk);
        assert!(chunk.payload.is_empty());
    }

    #[test]
    fn frames_outside_a_uart_session_are_refused() {
        reset();

        assert_eq!(
            on_packet::<TestProduct>(&[0u8; UART_PACKET_LEN], false),
            u16::MAX
        );

        // A mesh claim is not a UART claim.
        window::begin_ota(OtaClaimant::Mesh, || true).expect("mesh claim");
        assert_eq!(
            on_packet::<TestProduct>(&[0u8; UART_PACKET_LEN], false),
            u16::MAX
        );
        window::end_ota();

        reset();
    }

    #[test]
    fn session_start_is_refused_while_a_gatt_transfer_runs() {
        reset();
        engine::set_in_progress_for_test(true);

        assert!(!try_start_session::<TestProduct>(|| true));
        assert_eq!(window::active_ota_claimant(), None);

        reset();
    }
}
