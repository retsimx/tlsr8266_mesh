//! Pure, hardware-free OTA chunk validation.
//!
//! Ported verbatim from the firmware's `rust/src/ota/fsm.rs`. The save path used
//! to carry four hand-written `if`/`else` arms that decided the same four
//! outcomes this module returns; they live here instead so the index/size rules
//! have one source of truth and can be exercised without a flash device or a
//! clock.

/// What the save path should do with one staged OTA packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OtaChunkDecision {
    /// Index 0 with an empty write cursor: start a new staged image.
    WriteFirstChunk,
    /// The next expected index inside the size bound: append to the image.
    WriteNextChunk,
    /// Header-only packet at the expected index: verify the staged image size.
    VerifyFinalHeader { expected_index: u16 },
    /// Anything else is a protocol error and aborts the transfer.
    RejectError,
}

/// Decide what to do with one staged OTA packet.
///
/// `n_data_len == 0` is the final (header-only) packet. `current_flash_addr` is
/// the write cursor *before* this packet, `packet_len` the payload size (8 mesh
/// and UART, 16 GATT bridge) and `max_fw_bytes` the staging-area bound.
///
/// `wrapping_add(1)` reproduces the device's release-mode `u16 + 1`: an OTA that
/// ends at index `0xffff` continues at 0. Resetting the cursor for a fresh
/// transfer is a session concern, not this validator's job.
///
/// `current_flash_addr.saturating_add(packet_len)` keeps the bound total defined
/// for a corrupt or over-max cursor: the legacy release build wrapped the sum
/// back to a small value and let the packet through, saturating cannot.
pub fn evaluate_chunk(
    cur_idx: u16,
    last_idx: u16,
    n_data_len: usize,
    current_flash_addr: u32,
    packet_len: u32,
    max_fw_bytes: u32,
) -> OtaChunkDecision {
    if n_data_len == 0 {
        if cur_idx == last_idx.wrapping_add(1) {
            OtaChunkDecision::VerifyFinalHeader {
                expected_index: cur_idx,
            }
        } else {
            OtaChunkDecision::RejectError
        }
    } else if cur_idx == 0 {
        if current_flash_addr != 0 {
            OtaChunkDecision::RejectError
        } else {
            OtaChunkDecision::WriteFirstChunk
        }
    } else if cur_idx == last_idx.wrapping_add(1) {
        if current_flash_addr.saturating_add(packet_len) > max_fw_bytes {
            OtaChunkDecision::RejectError
        } else {
            OtaChunkDecision::WriteNextChunk
        }
    } else {
        OtaChunkDecision::RejectError
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Payload size of a mesh-OTA packet.
    const MESH_PKT: u32 = 8;
    /// Payload size of a BLE-bridge OTA packet.
    const BRIDGE_PKT: u32 = 16;
    /// Staging-area bound used by the production call site.
    const MAX_FW_BYTES: u32 = 128 * 1024;

    /// Index 0 with an empty cursor starts a transfer.
    #[test]
    fn test_first_chunk_accepted_with_zero_cursor() {
        assert_eq!(
            evaluate_chunk(0, 0xffff, 8, 0, MESH_PKT, MAX_FW_BYTES),
            OtaChunkDecision::WriteFirstChunk
        );
    }

    /// Index 0 on top of a partially written image is an error: the cursor must
    /// be empty before the first packet is accepted.
    #[test]
    fn test_first_chunk_rejected_with_nonzero_cursor() {
        assert_eq!(
            evaluate_chunk(0, 0xffff, 8, 8, MESH_PKT, MAX_FW_BYTES),
            OtaChunkDecision::RejectError
        );
    }

    /// The packet after the last accepted one is appended.
    #[test]
    fn test_next_chunk_accepted_at_last_plus_one() {
        assert_eq!(
            evaluate_chunk(7, 6, 8, 56, MESH_PKT, MAX_FW_BYTES),
            OtaChunkDecision::WriteNextChunk
        );
    }

    /// A skipped index aborts the transfer.
    #[test]
    fn test_next_chunk_rejected_on_gap() {
        assert_eq!(
            evaluate_chunk(9, 6, 8, 56, MESH_PKT, MAX_FW_BYTES),
            OtaChunkDecision::RejectError
        );
    }

    /// One byte past the staging bound is rejected.
    #[test]
    fn test_next_chunk_rejected_when_cursor_plus_packet_overflows_bound() {
        assert_eq!(
            evaluate_chunk(2, 1, 8, MAX_FW_BYTES - MESH_PKT + 1, MESH_PKT, MAX_FW_BYTES),
            OtaChunkDecision::RejectError
        );
    }

    /// Landing exactly on the bound is still a valid write.
    #[test]
    fn test_next_chunk_accepted_at_exact_bound() {
        assert_eq!(
            evaluate_chunk(2, 1, 8, MAX_FW_BYTES - MESH_PKT, MESH_PKT, MAX_FW_BYTES),
            OtaChunkDecision::WriteNextChunk
        );
    }

    /// A corrupt cursor past the staging bound cannot wrap the bound total:
    /// `u32::MAX` saturates instead of wrapping to 15, so the packet is still
    /// rejected. The legacy release build wrapped and would have written here.
    #[test]
    fn test_next_chunk_rejected_when_cursor_saturates_instead_of_wrapping() {
        assert_eq!(
            evaluate_chunk(42, 41, 8, u32::MAX, BRIDGE_PKT, MAX_FW_BYTES),
            OtaChunkDecision::RejectError
        );
    }

    /// The final header reports the index the image size must match.
    #[test]
    fn test_final_header_reports_expected_index() {
        assert_eq!(
            evaluate_chunk(
                42,
                41,
                0,
                MAX_FW_BYTES - BRIDGE_PKT,
                BRIDGE_PKT,
                MAX_FW_BYTES
            ),
            OtaChunkDecision::VerifyFinalHeader { expected_index: 42 }
        );
    }

    /// A transfer that ends at `0xffff` continues at index 0 on device; the
    /// validator reproduces that wrap explicitly.
    #[test]
    fn test_final_header_accepts_index_zero_after_max_index() {
        assert_eq!(
            evaluate_chunk(0, 0xffff, 0, 0, MESH_PKT, MAX_FW_BYTES),
            OtaChunkDecision::VerifyFinalHeader { expected_index: 0 }
        );
    }

    /// A final header out of sequence is an error.
    #[test]
    fn test_final_header_rejected_when_index_is_not_last_plus_one() {
        assert_eq!(
            evaluate_chunk(42, 40, 0, 0, MESH_PKT, MAX_FW_BYTES),
            OtaChunkDecision::RejectError
        );
    }

    /// A header-only packet is always the final-packet case, even at index 0:
    /// the `n_data_len == 0` test runs before the `cur_idx == 0` test, so an
    /// out-of-sequence header is rejected instead of starting a transfer.
    #[test]
    fn test_empty_payload_outranks_first_chunk() {
        assert_eq!(
            evaluate_chunk(0, 5, 0, 0, MESH_PKT, MAX_FW_BYTES),
            OtaChunkDecision::RejectError
        );
    }
}
