//! Hardware-free UART framing: command decode, ACK construction and the
//! echo-suppression fingerprint.
//!
//! Everything here is a pure function of a [`UartData`] frame, so the wire
//! contract is unit-testable on the host with no driver, no channel and no
//! mocked register. `uart::mod` owns the peripheral, the tasks and the seams;
//! this file is the only part of the subsystem that knows the frame layout, and
//! it names no sibling module.
//!
//! ## Frame layout (shared with `utilities/meshutils`)
//!
//! ```text
//! 0        counter (ack counter, or retry counter on our own frames)
//! 1        0xff marks an ack frame; 0 otherwise
//! 2        command (`UartMsg`)
//! 3..5     LightCtrl: mesh destination, little-endian
//! 5..18    LightCtrl: 13-byte payload handed to the mesh
//! 18       LightCtrl: retransmission count
//! 19       LightCtrl: non-zero requests a mesh ack
//! 42..44   CRC-16-MODBUS trailer
//! ```
//!
//! The mesh sequence number of an ACK frame lives at [`SNO_RANGE`] (`2..5`), so
//! the host can match the ack to the `sno` the mesh actually assigned.

use core::convert::TryFrom;
use core::ops::Range;

#[cfg(test)]
use mesh_protocol::uart::UART_CRC_OFFSET;
use mesh_protocol::uart::{compute_crc, UartData, UartMsg, UART_DATA_LEN};

/// Index of the frame's command byte (`UartMsg` opcode).
pub const MSG_TYPE_INDEX: usize = 2;

/// Byte span of the 3-byte mesh sequence number in an ACK frame.
pub const SNO_RANGE: Range<usize> = 2..5;

/// Byte span the UART CRC-16 covers.
///
/// Production delegates the check to `mesh_protocol::uart::verify_crc`/
/// `compute_crc`; the frame-building tests still need the span itself, and
/// deriving it from the crate's trailer offset keeps the two from drifting
/// apart.
#[cfg(test)]
pub(crate) const CRC_RANGE: Range<usize> = 0..UART_CRC_OFFSET;

/// Light control command: mesh destination address.
pub(crate) const CTRL_DESTINATION_RANGE: Range<usize> = 3..5;
/// Light control command: payload handed to the mesh.
pub(crate) const CTRL_PAYLOAD_RANGE: Range<usize> = 5..18;
/// Light control command: `dst`..`par` span echoed back as the dedupe key.
pub(crate) const CTRL_FINGERPRINT_RANGE: Range<usize> = 3..18;
/// Light control command: retransmission count.
pub(crate) const CTRL_RETRANSMIT_INDEX: usize = 18;
/// Light control command: non-zero requests a mesh ack.
pub(crate) const CTRL_ACK_FLAG_INDEX: usize = 19;
/// Outgoing mesh frame: span recorded when this device queues a frame.
pub(crate) const OUTGOING_FINGERPRINT_RANGE: Range<usize> = 7..22;

/// Number of payload bytes a light control command carries.
const CTRL_PAYLOAD_LEN: usize = CTRL_PAYLOAD_RANGE.end - CTRL_PAYLOAD_RANGE.start;
/// Number of bytes in a dedupe fingerprint.
const CTRL_FINGERPRINT_LEN: usize = CTRL_FINGERPRINT_RANGE.end - CTRL_FINGERPRINT_RANGE.start;

/// `dst`..`par` view of a frame, used to suppress echoes of sends.
pub type Fingerprint = [u8; CTRL_FINGERPRINT_LEN];

/// One command the host can send, decoded out of a validated frame.
///
/// [`UartCommand::HostAttached`] is a lifecycle event rather than a wire opcode
/// — [`decode_command`] never returns it — and is dispatched once, when the
/// first valid frame starts the bridge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UartCommand {
    /// The host started talking; enable the status reporting it expects.
    HostAttached,
    /// Build and transmit one mesh light command.
    LightCtrl {
        dst: u16,
        payload: [u8; 13],
        retransmit: u8,
        ack: bool,
    },
    /// The host asked for a full status refresh.
    EnableStatusReporting,
}

/// Decodes the command carried by a CRC-validated frame.
///
/// Returns `None` for any opcode this bridge does not dispatch, including an
/// ack frame (`data[1] == 0xff`) whose command byte is not set: such a frame is
/// consumed by the retry logic, never turned into a command.
pub fn decode_command(frame: &UartData) -> Option<UartCommand> {
    match UartMsg::try_from(frame.data[MSG_TYPE_INDEX]) {
        Ok(UartMsg::LightCtrl) => {
            let mut dst = [0u8; 2];
            dst.copy_from_slice(&frame.data[CTRL_DESTINATION_RANGE]);

            let mut payload = [0u8; CTRL_PAYLOAD_LEN];
            payload.copy_from_slice(&frame.data[CTRL_PAYLOAD_RANGE]);

            Some(UartCommand::LightCtrl {
                dst: u16::from_le_bytes(dst),
                payload,
                retransmit: frame.data[CTRL_RETRANSMIT_INDEX],
                ack: frame.data[CTRL_ACK_FLAG_INDEX] != 0,
            })
        }
        Ok(UartMsg::LightStatus) => Some(UartCommand::EnableStatusReporting),
        _ => None,
    }
}

/// Builds the ack frame for `frame`, carrying the mesh sequence number the
/// command handler returned at [`SNO_RANGE`].
///
/// Mirrors the host's ack counter (`data[0]`), sets the ack marker
/// (`data[1] == 0xff`) and stamps a fresh CRC trailer. The `sno` is the real
/// number the mesh assigned, not zeros, so the host can correlate the ack with
/// the mesh traffic it caused.
pub fn build_ack(frame: &UartData, sno: [u8; 3]) -> UartData {
    let mut ack = UartData {
        len: UART_DATA_LEN as u32,
        data: [0; UART_DATA_LEN],
    };

    // Ack the host's counter.
    ack.data[0] = frame.data[0];

    // Mark this frame as an ack.
    ack.data[1] = u8::from(UartMsg::Ack);

    // Set the mesh sequence number of the command we just queued.
    ack.data[SNO_RANGE].copy_from_slice(&sno);

    compute_crc(&mut ack);

    ack
}

/// Stamps the next retry counter into a copy of `frame` and recomputes the CRC.
///
/// The counter lives at `data[0]` and the CRC at `42..44`, both outside the
/// dedupe fingerprint span, so a retransmission never changes what the frame
/// looks like to the echo suppressor.
pub fn ack_retry_frame(frame: &UartData, ack_counter: u8) -> UartData {
    let mut retry = *frame;
    retry.data[0] = ack_counter;
    compute_crc(&mut retry);
    retry
}

/// `dst`..`par` view of a frame, as recorded when this device handles a
/// light-control command.
pub fn fingerprint(frame: &UartData) -> Fingerprint {
    let mut out = [0u8; CTRL_FINGERPRINT_LEN];
    out.copy_from_slice(&frame.data[CTRL_FINGERPRINT_RANGE]);
    out
}

/// Span recorded when this device queues an outgoing frame for retry.
///
/// A separate view from [`fingerprint`]: an outgoing `MeshMessage` frame
/// carries the 21-byte `AppCmdValue` from offset 3, so this 15-byte window
/// starts one byte earlier and ends one byte earlier than the control-command
/// span. The value is kept exactly as the pre-split UART manager stamped it —
/// changing it would change which echoes are suppressed.
pub fn outgoing_fingerprint(frame: &UartData) -> Fingerprint {
    let mut out = [0u8; CTRL_FINGERPRINT_LEN];
    out.copy_from_slice(&frame.data[OUTGOING_FINGERPRINT_RANGE]);
    out
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use core::convert::TryInto;
    use mesh_protocol::crc::crc16;
    use mesh_protocol::uart::verify_crc;

    /// A frame with a valid CRC and the given command byte.
    fn frame(cmd: UartMsg) -> UartData {
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        for (idx, byte) in msg.data.iter_mut().take(CRC_RANGE.end).enumerate() {
            *byte = idx as u8;
        }
        msg.data[MSG_TYPE_INDEX] = u8::from(cmd);
        compute_crc(&mut msg);
        msg
    }

    /// A light-control frame with every field distinct from the defaults, so an
    /// off-by-one in any of the decode offsets shows up as a wrong value.
    fn ctrl_frame() -> UartData {
        let mut msg = UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        };

        msg.data[0] = 0x21;
        msg.data[MSG_TYPE_INDEX] = u8::from(UartMsg::LightCtrl);
        msg.data[CTRL_DESTINATION_RANGE].copy_from_slice(&u16::to_le_bytes(0x1234));
        for (idx, byte) in msg.data[CTRL_PAYLOAD_RANGE].iter_mut().enumerate() {
            *byte = (idx as u8) + 1;
        }
        msg.data[CTRL_RETRANSMIT_INDEX] = 2;
        msg.data[CTRL_ACK_FLAG_INDEX] = 1;
        compute_crc(&mut msg);
        msg
    }

    fn empty_frame() -> UartData {
        UartData {
            len: UART_DATA_LEN as u32,
            data: [0; UART_DATA_LEN],
        }
    }

    #[test]
    fn test_uart_msg_try_from() {
        let cases = [
            (UartMsg::LightCtrl, 0x02),
            (UartMsg::LightStatus, 0x03),
            (UartMsg::MeshMessage, 0x04),
            (UartMsg::PanicMessage, 0x05),
            (UartMsg::PrintMessage, 0x06),
            (UartMsg::Ack, 0xff),
        ];

        for (variant, byte) in cases.iter() {
            assert_eq!(u8::from(*variant), *byte);
            assert_eq!(UartMsg::try_from(*byte).unwrap(), *variant);
        }

        assert!(UartMsg::try_from(0x11).is_err());
    }

    /// Every light-control field is decoded from its own offset.
    #[test]
    fn test_decode_command_light_ctrl_extracts_every_field() {
        let msg = ctrl_frame();

        let decoded = decode_command(&msg);

        assert_eq!(
            decoded,
            Some(UartCommand::LightCtrl {
                dst: 0x1234,
                payload: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
                retransmit: 2,
                ack: true,
            })
        );
    }

    /// A light-status request decodes to the reporting command, and a zeroed ack
    /// flag decodes to `ack: false`.
    #[test]
    fn test_decode_command_light_status_and_ack_flag() {
        let status = frame(UartMsg::LightStatus);
        assert_eq!(
            decode_command(&status),
            Some(UartCommand::EnableStatusReporting)
        );

        let mut no_ack = ctrl_frame();
        no_ack.data[CTRL_ACK_FLAG_INDEX] = 0;
        let Some(UartCommand::LightCtrl { ack, .. }) = decode_command(&no_ack) else {
            panic!("a light-control frame must decode");
        };
        assert!(!ack);
    }

    /// Unknown opcodes and an all-zero (short/empty) frame are rejected rather
    /// than dispatched: `decode_command` is total and never panics on a frame
    /// that passed the CRC check but carries no command we understand.
    #[test]
    fn test_decode_command_rejects_unknown_and_empty_frames() {
        assert_eq!(decode_command(&empty_frame()), None);

        let mut unknown = empty_frame();
        unknown.data[MSG_TYPE_INDEX] = 0x11;
        assert_eq!(decode_command(&unknown), None);

        // An ack frame from the host leaves the command byte clear.
        let ack = frame(UartMsg::Ack);
        assert_eq!(decode_command(&ack), None);
    }

    /// The ACK frame carries the mesh sequence number at `SNO_RANGE`, not zeros.
    #[test]
    fn test_build_ack_writes_sno_at_sno_range() {
        let msg = ctrl_frame();
        let sno = [0xAA, 0xBB, 0xCC];

        let ack = build_ack(&msg, sno);

        assert_eq!(ack.data[SNO_RANGE], sno);
        assert_eq!(ack.data[SNO_RANGE.start], 0xAA);
        assert_eq!(ack.data[SNO_RANGE.end - 1], 0xCC);
        assert!(verify_crc(&ack), "the ack must carry a valid CRC");
    }

    /// The ACK frame mirrors the host's counter, carries the ack marker and the
    /// standard frame length.
    #[test]
    fn test_build_ack_mirrors_counter_and_ack_marker() {
        let msg = ctrl_frame();

        let ack = build_ack(&msg, [1, 2, 3]);

        assert_eq!(ack.data[0], msg.data[0]);
        assert_eq!(ack.data[1], u8::from(UartMsg::Ack));
        assert_eq!(ack.len, UART_DATA_LEN as u32);
        assert_eq!(ack.data[SNO_RANGE], [1, 2, 3]);
    }

    /// The control-command fingerprint is exactly the `dst`..`par` span, so the
    /// echo suppressor compares the same bytes the mesh packet carries.
    #[test]
    fn test_fingerprint_is_the_destination_to_par_span() {
        let msg = ctrl_frame();
        let expected: [u8; 15] = msg.data[CTRL_FINGERPRINT_RANGE].try_into().unwrap();

        assert_eq!(fingerprint(&msg), expected);
        assert_eq!(&fingerprint(&msg)[0..2], &u16::to_le_bytes(0x1234));
    }

    /// A retransmission only rewrites the counter and the CRC, so the dedupe
    /// fingerprint of the outgoing frame is stable across retries.
    #[test]
    fn test_outgoing_fingerprint_is_stable_across_retries() {
        let mut msg = frame(UartMsg::MeshMessage);
        let before = outgoing_fingerprint(&msg);
        let expected: [u8; 15] = msg.data[OUTGOING_FINGERPRINT_RANGE].try_into().unwrap();
        assert_eq!(before, expected);

        let retry = ack_retry_frame(&msg, 9);

        assert_eq!(outgoing_fingerprint(&retry), before);
        assert_eq!(retry.data[0], 9);

        // The counter is the only stamped difference besides the CRC trailer.
        msg.data[0] = 9;
        compute_crc(&mut msg);
        assert_eq!(retry.data, msg.data);
    }

    /// The retry builder stamps the counter and refreshes the CRC trailer.
    #[test]
    fn test_ack_retry_frame_stamps_counter_and_crc() {
        let msg = frame(UartMsg::MeshMessage);

        let retry = ack_retry_frame(&msg, 7);

        assert_eq!(retry.data[0], 7);
        assert!(verify_crc(&retry));
        assert_eq!(
            retry.data[1..UART_CRC_OFFSET],
            msg.data[1..UART_CRC_OFFSET],
            "only the counter may change outside the trailer"
        );
    }

    /// Tests the compute_crc function.
    ///
    /// This test verifies that compute_crc correctly calculates and sets the
    /// CRC in the specified message.
    ///
    /// # Algorithm
    ///
    /// 1. Create a test message with known data
    /// 2. Call compute_crc on the message
    /// 3. Manually compute the expected CRC
    /// 4. Verify the CRC in the message matches the expected value
    #[test]
    fn test_compute_crc() {
        let mut msg = empty_frame();

        for (i, byte) in msg.data.iter_mut().enumerate().take(42) {
            *byte = i as u8;
        }

        compute_crc(&mut msg);

        let expected_crc = crc16(&msg.data[0..42]);
        let expected_crc_low = (expected_crc & 0xff) as u8;
        let expected_crc_high = ((expected_crc >> 8) & 0xff) as u8;

        assert_eq!(msg.data[42], expected_crc_low);
        assert_eq!(msg.data[43], expected_crc_high);
    }
}
