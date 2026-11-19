//! Frame envelope: `COBS([seq][tag][body][crc16_le]) 0x00`.

use crate::cobs::{cobs_decode, cobs_encode};
use crate::crc::crc16_ccitt;
use crate::msg::{ProtoError, Request, Response};
use crate::{MAX_BODY, MAX_DECODED_FRAME, MAX_WIRE_FRAME};

/// Which message space a frame carries.
///
/// Request `Ping` and response `Pong` both use tag `0x01`, and response `Ack`
/// uses tag `0x00`, so decoding needs an explicit direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Host-to-probe frame.
    Request,
    /// Probe-to-host frame.
    Response,
}

/// A decoded or to-be-encoded message.
// `Response` carries a fixed-size inline `Data` buffer by contract; the crate
// is no-heap, so there is no `Box` to move it behind.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Message {
    /// Host-to-probe request.
    Request(Request),
    /// Probe-to-host response.
    Response(Response),
}

/// Encodes `msg` as a complete wire frame in `out`, returning its length
/// including the trailing `0x00` delimiter.
///
/// The frame is `COBS([seq][tag][body][crc16_le]) 0x00`, where the CRC covers
/// `seq || tag || body`. `out` must hold the whole worst-case COBS encoding plus
/// the delimiter; returns [`ProtoError::InvalidArg`] if it does not, or if the
/// message is invalid, and [`ProtoError::InvalidAddrRange`] if its addresses are
/// unmapped.
///
/// `out` is written only on success, so a caller may retry with a larger buffer
/// after an error, and nothing outside `out` is ever written. No input panics.
pub fn encode_frame(seq: u8, msg: &Message, out: &mut [u8]) -> Result<usize, ProtoError> {
    // 1. Body first: this is where message validation runs, and it rejects
    //    before any of `out` is touched.
    let mut body = [0u8; MAX_BODY];
    let (tag, body_len) = match msg {
        Message::Request(request) => (request.tag(), request.encode_body(&mut body)?),
        Message::Response(response) => (response.tag(), response.encode_body(&mut body)?),
    };

    // 2. Decoded frame: seq, tag, body, little-endian CRC16.
    let mut frame = [0u8; MAX_DECODED_FRAME];
    frame[0] = seq;
    frame[1] = tag;
    frame[2..2 + body_len].copy_from_slice(&body[..body_len]);
    let crc_index = 2 + body_len;
    let crc = crc16_ccitt(&frame[..crc_index]);
    frame[crc_index..crc_index + 2].copy_from_slice(&crc.to_le_bytes());

    // 3. COBS into everything but the final byte, so the delimiter always has a
    //    slot once the encode succeeds. `cobs_encode` checks its capacity before
    //    writing anything, so a too-small `out` leaves the buffer untouched.
    if out.is_empty() {
        return Err(ProtoError::InvalidArg);
    }
    let cobs_budget = out.len() - 1;
    let cobs_len = cobs_encode(&frame[..crc_index + 2], &mut out[..cobs_budget])?;
    // The delimiter goes immediately after the COBS bytes, not at the end of
    // `out`: callers reuse this buffer, so `out[cobs_len]` may still hold the
    // tail of an earlier, longer frame. `cobs_encode` returned at most
    // `out.len() - 1`, so this index is always in bounds.
    out[cobs_len] = 0x00;
    Ok(cobs_len + 1)
}

/// Decodes a wire frame, accepting it with or without the trailing delimiter.
///
/// Returns the sequence byte and the decoded message. Structural faults yield
/// [`ProtoError::InvalidFrame`], CRC mismatches [`ProtoError::BadCrc`], and
/// message faults keep their body-codec mapping. No input panics.
pub fn decode_frame(direction: Direction, wire: &[u8]) -> Result<(u8, Message), ProtoError> {
    // 1. An optional final delimiter; any zero anywhere else is malformed
    //    (embedded zero, or bytes after the terminator).
    let payload = match wire.iter().position(|&byte| byte == 0) {
        Some(index) if index + 1 == wire.len() => &wire[..index],
        Some(_) => return Err(ProtoError::InvalidFrame),
        None => wire,
    };

    // 2. Size limits: a frame always carries at least one COBS code byte, and
    //    a longer payload than the worst case is structurally bogus.
    if payload.is_empty() || payload.len() > MAX_WIRE_FRAME - 1 {
        return Err(ProtoError::InvalidFrame);
    }

    // 3. COBS decode. `InvalidArg` here can only mean the peer frame decodes to
    //    more than `MAX_DECODED_FRAME` bytes: the destination is this crate's
    //    fixed buffer, not a caller argument, so report the frame as malformed.
    let mut frame = [0u8; MAX_DECODED_FRAME];
    let frame_len = cobs_decode(payload, &mut frame).map_err(|e| {
        if e == ProtoError::InvalidArg {
            ProtoError::InvalidFrame
        } else {
            e
        }
    })?;

    // 4. The shortest frame is `seq + tag + crc16`.
    if frame_len < 4 {
        return Err(ProtoError::InvalidFrame);
    }

    // 5. CRC covers `seq || tag || body`.
    let crc_index = frame_len - 2;
    let crc = u16::from_le_bytes([frame[crc_index], frame[crc_index + 1]]);
    if crc16_ccitt(&frame[..crc_index]) != crc {
        return Err(ProtoError::BadCrc);
    }

    // 6. Split and dispatch; `direction` is what disambiguates tag `0x01`.
    let (seq, tag) = (frame[0], frame[1]);
    let body = &frame[2..crc_index];
    let message = match direction {
        Direction::Request => Message::Request(Request::decode_body(tag, body)?),
        Direction::Response => Message::Response(Response::decode_body(tag, body)?),
    };
    Ok((seq, message))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::{CpuMode, MAX_DATA_PAYLOAD, MAX_WRITE_PAYLOAD};
    use heapless::Vec;

    fn write_payload(len: usize, fill: u8) -> Vec<u8, MAX_WRITE_PAYLOAD> {
        let bytes = [fill; MAX_WRITE_PAYLOAD];
        Vec::from_slice(&bytes[..len]).expect("payload fits")
    }

    fn data_payload(len: usize, fill: u8) -> Vec<u8, MAX_DATA_PAYLOAD> {
        let bytes = [fill; MAX_DATA_PAYLOAD];
        Vec::from_slice(&bytes[..len]).expect("payload fits")
    }

    fn request_samples() -> [Request; 15] {
        [
            Request::Ping,
            Request::Reset {
                hold_ms: 250,
                stop_cpu: true,
            },
            Request::SwsActivate { div: 8 },
            Request::ReadMem {
                addr: 0x8000,
                len: 64,
            },
            Request::WriteMem {
                addr: 0x8000,
                data: write_payload(MAX_WRITE_PAYLOAD, 0x5a),
            },
            Request::ReadAnalog { addr: 0x0c },
            Request::WriteAnalog {
                addr: 0x0c,
                val: 0xff,
            },
            Request::FlashJedecId,
            Request::FlashRead {
                addr: 0x7_f00,
                len: 256,
            },
            Request::FlashEraseSector { addr: 0x1000 },
            Request::FlashWritePage {
                addr: 0x2000,
                data: write_payload(MAX_WRITE_PAYLOAD, 0xa5),
            },
            Request::CpuCtrl {
                mode: CpuMode::SoftReboot,
            },
            Request::ReadRegs,
            Request::SetBreakpoint {
                slot: 2,
                addr: 0x0080_0000,
            },
            Request::RttPoll { ctrl_addr: 0xbef0 },
        ]
    }

    fn response_samples() -> [Response; 13] {
        [
            Response::Ack,
            Response::Pong {
                fw_ver: 0x0102,
                proto_ver: 1,
            },
            Response::ChipId {
                rev: 3,
                chip_id: 0x8266,
            },
            Response::Byte(0x42),
            Response::Data(data_payload(0, 0x00)),
            Response::Data(data_payload(MAX_DATA_PAYLOAD, 0xff)),
            Response::JedecId([0xef, 0x40, 0x18]),
            Response::Error(ProtoError::InvalidFrame),
            Response::Error(ProtoError::BadCrc),
            Response::Error(ProtoError::InvalidAddrRange),
            Response::Error(ProtoError::InvalidArg),
            Response::Error(ProtoError::SwsTimeout),
            Response::Error(ProtoError::FlashBusyTimeout),
        ]
    }

    /// Encodes a frame, decodes it back, and checks the message, the sequence
    /// byte, and that the delimiter is optional on input.
    ///
    /// The buffer starts dirty (`0xaa`, not zero), so the encode has to write
    /// the delimiter itself rather than rely on a pre-zeroed byte after the COBS
    /// output.
    fn assert_round_trip(direction: Direction, seq: u8, msg: &Message) {
        let mut wire = [0xaau8; MAX_WIRE_FRAME];
        let len = encode_frame(seq, msg, &mut wire).expect("frame encodes");
        assert!(len <= MAX_WIRE_FRAME, "frame fits the wire budget");
        assert_delimiter_terminated(&wire, len);

        let expected = Ok((seq, msg.clone()));
        assert_eq!(decode_frame(direction, &wire[..len]), expected);
        assert_eq!(
            decode_frame(direction, &wire[..len - 1]),
            Ok((seq, msg.clone())),
            "frame without its delimiter decodes identically"
        );
    }

    /// Asserts `wire[..len]` is delimiter-terminated and carries no other zero,
    /// so a stale byte before the terminator cannot hide in the frame.
    fn assert_delimiter_terminated(wire: &[u8], len: usize) {
        assert_eq!(wire[len - 1], 0x00, "frame ends with the delimiter");
        assert_eq!(
            wire[..len].iter().filter(|&&byte| byte == 0x00).count(),
            1,
            "the delimiter is the only zero byte in the frame"
        );
    }

    /// Builds a decoded frame straight from parts, so corruption tests can
    /// control the exact bytes that reach `decode_frame`.
    fn decoded_frame(seq: u8, tag: u8, body: &[u8]) -> ([u8; MAX_DECODED_FRAME], usize) {
        let mut frame = [0u8; MAX_DECODED_FRAME];
        frame[0] = seq;
        frame[1] = tag;
        frame[2..2 + body.len()].copy_from_slice(body);
        let crc_index = 2 + body.len();
        let crc = crc16_ccitt(&frame[..crc_index]);
        frame[crc_index..crc_index + 2].copy_from_slice(&crc.to_le_bytes());
        (frame, crc_index + 2)
    }

    /// COBS-encodes a decoded frame and appends the delimiter.
    fn wire_from(decoded: &[u8]) -> ([u8; MAX_WIRE_FRAME], usize) {
        let mut wire = [0u8; MAX_WIRE_FRAME];
        let len = cobs_encode(decoded, &mut wire).expect("fits");
        wire[len] = 0x00;
        (wire, len + 1)
    }

    #[test]
    fn every_request_round_trips() {
        for request in &request_samples() {
            assert_round_trip(Direction::Request, 0x5a, &Message::Request(request.clone()));
        }
    }

    #[test]
    fn every_response_round_trips() {
        for response in &response_samples() {
            assert_round_trip(
                Direction::Response,
                0xa5,
                &Message::Response(response.clone()),
            );
        }
    }

    #[test]
    fn maximum_data_frame_fits_the_wire_budget() {
        // `Data` caps at 1024 bytes, so the largest frame this crate can build
        // decodes to 1028 bytes: 1 seq + 1 tag + 1024 body + 2 CRC. COBS adds 5
        // code bytes (four full 254-byte blocks plus one), then the delimiter.
        let response = Response::Data(data_payload(MAX_DATA_PAYLOAD, 0xff));
        let msg = Message::Response(response);
        let mut wire = [0u8; MAX_WIRE_FRAME];
        let len = encode_frame(0, &msg, &mut wire).unwrap();

        assert_eq!(len, 1034);
        assert!(len < MAX_WIRE_FRAME);
        assert_eq!(
            decode_frame(Direction::Response, &wire[..len]),
            Ok((0, msg))
        );
    }

    #[test]
    fn delimiter_is_optional() {
        let msg = Message::Request(Request::WriteMem {
            addr: 0x8000,
            data: write_payload(64, 0x11),
        });
        let mut wire = [0u8; MAX_WIRE_FRAME];
        let len = encode_frame(9, &msg, &mut wire).unwrap();

        assert_eq!(wire[len - 1], 0x00);
        assert_eq!(
            decode_frame(Direction::Request, &wire[..len]),
            Ok((9, msg.clone()))
        );
        assert_eq!(
            decode_frame(Direction::Request, &wire[..len - 1]),
            Ok((9, msg))
        );
    }

    #[test]
    fn crc_covers_seq_and_body() {
        // `ReadMem { addr: 0x8000, len: 4 }`.
        let body = [0x00, 0x80, 0x04, 0x00];
        let (frame, frame_len) = decoded_frame(7, 0x04, &body);
        let (wire, wire_len) = wire_from(&frame[..frame_len]);
        assert!(decode_frame(Direction::Request, &wire[..wire_len]).is_ok());

        let mut bad_seq = frame;
        bad_seq[0] ^= 0x01;
        let (wire, wire_len) = wire_from(&bad_seq[..frame_len]);
        assert_eq!(
            decode_frame(Direction::Request, &wire[..wire_len]),
            Err(ProtoError::BadCrc)
        );

        let mut bad_body = frame;
        bad_body[4] ^= 0x01;
        let (wire, wire_len) = wire_from(&bad_body[..frame_len]);
        assert_eq!(
            decode_frame(Direction::Request, &wire[..wire_len]),
            Err(ProtoError::BadCrc)
        );

        // Dropping the final CRC byte leaves a frame whose CRC field no longer
        // matches its contents.
        let (wire, wire_len) = wire_from(&frame[..frame_len - 1]);
        assert!(matches!(
            decode_frame(Direction::Request, &wire[..wire_len]),
            Err(ProtoError::BadCrc) | Err(ProtoError::InvalidFrame)
        ));
    }

    #[test]
    fn structural_faults_are_invalid_frames() {
        // Empty input, and a lone delimiter (an empty COBS payload).
        assert_eq!(
            decode_frame(Direction::Request, &[]),
            Err(ProtoError::InvalidFrame)
        );
        assert_eq!(
            decode_frame(Direction::Request, &[0x00]),
            Err(ProtoError::InvalidFrame)
        );

        let msg = Message::Request(Request::Ping);
        let mut wire = [0u8; MAX_WIRE_FRAME];
        let len = encode_frame(1, &msg, &mut wire).unwrap();

        // An embedded zero in the middle of the payload.
        let mut embedded = [0u8; MAX_WIRE_FRAME + 1];
        embedded[..len].copy_from_slice(&wire[..len]);
        embedded.copy_within(1..len, 2);
        embedded[1] = 0x00;
        assert_eq!(
            decode_frame(Direction::Request, &embedded[..=len]),
            Err(ProtoError::InvalidFrame)
        );

        // Bytes after the delimiter.
        let mut trailing = [0u8; MAX_WIRE_FRAME + 1];
        trailing[..len].copy_from_slice(&wire[..len]);
        trailing[len] = 0xaa;
        assert_eq!(
            decode_frame(Direction::Request, &trailing[..=len]),
            Err(ProtoError::InvalidFrame)
        );

        // A payload longer than the largest possible COBS frame.
        let oversized = [0x55u8; MAX_WIRE_FRAME];
        assert_eq!(
            decode_frame(Direction::Request, &oversized),
            Err(ProtoError::InvalidFrame)
        );
    }

    #[test]
    fn direction_disambiguates_tag_0x01() {
        let msg = Message::Request(Request::Ping);
        let mut wire = [0u8; MAX_WIRE_FRAME];
        let len = encode_frame(1, &msg, &mut wire).unwrap();

        assert_eq!(
            decode_frame(Direction::Request, &wire[..len]),
            Ok((1, msg.clone()))
        );
        // As a response, tag `0x01` is `Pong`, which requires a 3-byte body, so
        // a body-less `Ping` frame cannot decode.
        assert_eq!(
            decode_frame(Direction::Response, &wire[..len]),
            Err(ProtoError::InvalidFrame)
        );
    }

    #[test]
    fn undersized_output_is_invalid_arg() {
        let msg = Message::Response(Response::Data(data_payload(MAX_DATA_PAYLOAD, 0xff)));
        let mut small = [0u8; 8];
        assert_eq!(
            encode_frame(0, &msg, &mut small),
            Err(ProtoError::InvalidArg)
        );
        assert_eq!(
            small, [0u8; 8],
            "a rejected encode writes nothing into the output buffer"
        );

        let mut empty = [0u8; 0];
        assert_eq!(
            encode_frame(0, &Message::Request(Request::Ping), &mut empty),
            Err(ProtoError::InvalidArg)
        );
    }

    #[test]
    fn failed_encode_leaves_output_untouched() {
        let msg = Message::Response(Response::Data(data_payload(MAX_DATA_PAYLOAD, 0xff)));

        // A maximum `Data` frame is exactly 1034 bytes on the wire: 1033 COBS
        // bytes plus the delimiter.
        let mut exact = [0u8; 1034];
        assert_eq!(encode_frame(0, &msg, &mut exact), Ok(1034));

        // One byte short: COBS alone would fit, but the delimiter would not, and
        // the encode is rejected before any byte of `out` is written.
        let mut short = [0u8; 1033];
        assert_eq!(
            encode_frame(0, &msg, &mut short),
            Err(ProtoError::InvalidArg)
        );
        assert_eq!(
            short, [0u8; 1033],
            "a rejected encode writes nothing into the output buffer"
        );
    }

    #[test]
    fn oversized_payload_is_an_invalid_frame() {
        // 1049 bytes of `0x01` COBS-decode to 1048 implicit zeros, which cannot
        // fit `MAX_DECODED_FRAME` (1044). The peer frame is malformed, so the
        // fault is structural rather than a bad caller argument.
        let oversized = [0x01u8; MAX_WIRE_FRAME - 1];
        assert_eq!(
            decode_frame(Direction::Request, &oversized),
            Err(ProtoError::InvalidFrame)
        );
    }

    #[test]
    fn message_validation_surfaces_through_the_frame() {
        let mut out = [0u8; MAX_WIRE_FRAME];

        assert_eq!(
            encode_frame(
                0,
                &Message::Request(Request::SwsActivate { div: 0 }),
                &mut out
            ),
            Err(ProtoError::InvalidArg)
        );
        assert_eq!(
            encode_frame(
                0,
                &Message::Request(Request::ReadMem {
                    addr: 0x1000,
                    len: 1,
                }),
                &mut out
            ),
            Err(ProtoError::InvalidAddrRange)
        );
    }
}
