//! Consistent Overhead Byte Stuffing (COBS).

use crate::msg::ProtoError;

/// Payload bytes carried by one full COBS block (code byte `0xFF`).
const MAX_BLOCK: usize = 254;

/// COBS-encodes `input` into `out`, returning the encoded length.
///
/// The encoded output excludes the trailing `0x00` delimiter, which the caller
/// appends. Returns [`ProtoError::InvalidArg`] if `out` cannot hold the
/// worst-case `input.len() + input.len() / 254 + 1` bytes.
///
/// Encoding is canonical: a code byte is the number of payload bytes up to (and
/// not including) the next zero plus one, a zero byte is represented only by
/// the code byte that terminates its block, and a `0xFF` code closes a full
/// block of 254 bytes. Empty input encodes to the single byte `[0x01]`.
pub fn cobs_encode(input: &[u8], out: &mut [u8]) -> Result<usize, ProtoError> {
    if out.len() < input.len() + input.len() / MAX_BLOCK + 1 {
        return Err(ProtoError::InvalidArg);
    }

    // `code_index` is where the current block's code byte lives; `idx` is the
    // first free byte after it. The size check above bounds both.
    let mut code_index = 0usize;
    let mut idx = 1usize;
    let mut code = 1u8;
    for &byte in input {
        if byte == 0 {
            out[code_index] = code;
            code_index = idx;
            idx += 1;
            code = 1;
        } else {
            out[idx] = byte;
            idx += 1;
            code += 1;
            if code == 0xFF {
                out[code_index] = code;
                code_index = idx;
                idx += 1;
                code = 1;
            }
        }
    }
    out[code_index] = code;
    Ok(idx)
}

/// COBS-decodes `input` into `out`, returning the decoded length.
///
/// Rejects malformed code bytes, truncated blocks, and non-zero data after the
/// final block with [`ProtoError::InvalidFrame`]; returns
/// [`ProtoError::InvalidArg`] if `out` is too small.
///
/// A `0x00` code byte is always malformed, as is a `0x00` byte inside block
/// data (canonical COBS output never contains a zero). Empty input is rejected
/// because a frame always contains at least one code byte.
pub fn cobs_decode(input: &[u8], out: &mut [u8]) -> Result<usize, ProtoError> {
    if input.is_empty() {
        return Err(ProtoError::InvalidFrame);
    }

    let mut idx = 0usize;
    let mut pos = 0usize;
    while pos < input.len() {
        let code = input[pos];
        if code == 0 {
            return Err(ProtoError::InvalidFrame);
        }
        pos += 1;

        let len = usize::from(code) - 1;
        if pos + len > input.len() {
            return Err(ProtoError::InvalidFrame);
        }
        let block = &input[pos..pos + len];
        if block.contains(&0) {
            return Err(ProtoError::InvalidFrame);
        }
        if idx + len > out.len() {
            return Err(ProtoError::InvalidArg);
        }
        out[idx..idx + len].copy_from_slice(block);
        idx += len;
        pos += len;

        // A block shorter than 0xFF is followed by an implicit zero unless it
        // is the last block.
        if code != 0xFF && pos < input.len() {
            if idx >= out.len() {
                return Err(ProtoError::InvalidArg);
            }
            out[idx] = 0;
            idx += 1;
        }
    }
    Ok(idx)
}

#[cfg(test)]
mod tests {
    use super::{cobs_decode, cobs_encode};
    use crate::msg::ProtoError;
    use crate::{MAX_DECODED_FRAME, MAX_WIRE_FRAME};

    /// Canonical encoded length: one code byte per block plus one byte per
    /// non-zero payload byte.
    fn expected_encoded_len(input: &[u8]) -> usize {
        let mut blocks = 1usize;
        let mut data = 0usize;
        let mut run = 0usize;
        for &byte in input {
            if byte == 0 {
                blocks += 1;
                run = 0;
            } else {
                data += 1;
                run += 1;
                if run == 254 {
                    blocks += 1;
                    run = 0;
                }
            }
        }
        blocks + data
    }

    fn worst_case(len: usize) -> usize {
        len + len / 254 + 1
    }

    fn assert_round_trip(payload: &[u8]) {
        let mut encoded = [0u8; MAX_WIRE_FRAME];
        let encoded_len = cobs_encode(payload, &mut encoded).expect("encode");
        assert_eq!(encoded_len, expected_encoded_len(payload));
        assert!(encoded_len <= worst_case(payload.len()));

        let mut decoded = [0u8; MAX_DECODED_FRAME];
        let decoded_len = cobs_decode(&encoded[..encoded_len], &mut decoded).expect("decode");
        assert_eq!(decoded_len, payload.len());
        assert_eq!(&decoded[..decoded_len], payload);
    }

    #[test]
    fn round_trips_every_length_and_pattern() {
        let mut payload = [0u8; MAX_DECODED_FRAME];
        for len in 0..=MAX_DECODED_FRAME {
            // All zero.
            payload[..len].fill(0);
            assert_round_trip(&payload[..len]);

            // All 0xFF (non-zero, so full blocks are exercised).
            payload[..len].fill(0xFF);
            assert_round_trip(&payload[..len]);

            // Alternating 0xAA / 0x00, so every other byte is implicit.
            for (i, byte) in payload[..len].iter_mut().enumerate() {
                *byte = if i % 2 == 0 { 0xAA } else { 0x00 };
            }
            assert_round_trip(&payload[..len]);

            // Every byte value, wrapping once past 256.
            for (i, byte) in payload[..len].iter_mut().enumerate() {
                *byte = (i % 256) as u8;
            }
            assert_round_trip(&payload[..len]);
        }
    }

    #[test]
    fn maximum_size_all_nonzero_payload() {
        let payload = [0xFFu8; MAX_DECODED_FRAME];
        let mut encoded = [0u8; MAX_WIRE_FRAME];
        let len = cobs_encode(&payload, &mut encoded).unwrap();

        // 4 full 254-byte blocks plus a final 28-byte block: 1044 + 5.
        assert_eq!(len, 1049);
        assert_eq!(len, worst_case(MAX_DECODED_FRAME));
        assert_eq!(
            (
                encoded[0],
                encoded[255],
                encoded[510],
                encoded[765],
                encoded[1020]
            ),
            (0xFF, 0xFF, 0xFF, 0xFF, 0x1D)
        );
        // With the delimiter the frame still fits MAX_WIRE_FRAME.
        assert!(len < MAX_WIRE_FRAME);
    }

    #[test]
    fn maximum_size_all_zero_payload() {
        let payload = [0u8; MAX_DECODED_FRAME];
        let mut encoded = [0u8; MAX_WIRE_FRAME];
        let len = cobs_encode(&payload, &mut encoded).unwrap();

        // Canonical COBS: every zero terminates a block, so 1044 zeros produce
        // 1044 one-byte blocks plus the final code byte, all `0x01`. (The
        // "5 code bytes" figure in the task brief describes the all-nonzero
        // block structure above, not this case.)
        assert_eq!(len, 1045);
        assert!(encoded[..len].iter().all(|&byte| byte == 0x01));
        assert!(len < MAX_WIRE_FRAME);

        let mut decoded = [0u8; MAX_DECODED_FRAME];
        assert_eq!(cobs_decode(&encoded[..len], &mut decoded), Ok(1044));
        assert_eq!(decoded, payload);
    }

    #[test]
    fn empty_input_encodes_to_one_code_byte() {
        let mut encoded = [0u8; 1];
        assert_eq!(cobs_encode(&[], &mut encoded), Ok(1));
        assert_eq!(encoded, [0x01]);

        let mut decoded = [0u8; 0];
        assert_eq!(cobs_decode(&[0x01], &mut decoded), Ok(0));
    }

    #[test]
    fn tiny_output_buffers_return_invalid_arg() {
        // One byte short of the 1049-byte worst case.
        let payload = [0xAAu8; MAX_DECODED_FRAME];
        let mut short = [0u8; 1048];
        assert_eq!(
            cobs_encode(&payload, &mut short),
            Err(ProtoError::InvalidArg)
        );

        // Typical small-input boundary: 10 bytes need 11 worst-case bytes.
        let mut tiny = [0u8; 10];
        assert_eq!(
            cobs_encode(&[0u8; 10], &mut tiny),
            Err(ProtoError::InvalidArg)
        );

        // Decoding a valid frame into a too-small buffer.
        let mut encoded = [0u8; MAX_WIRE_FRAME];
        let len = cobs_encode(&payload, &mut encoded).unwrap();
        assert_eq!(
            cobs_decode(&encoded[..len], &mut tiny),
            Err(ProtoError::InvalidArg)
        );
    }

    #[test]
    fn malformed_inputs_are_invalid_frames() {
        let mut out = [0u8; MAX_DECODED_FRAME];
        assert_eq!(cobs_decode(&[], &mut out), Err(ProtoError::InvalidFrame));
        assert_eq!(
            cobs_decode(&[0x00], &mut out),
            Err(ProtoError::InvalidFrame)
        );
        assert_eq!(
            cobs_decode(&[0x05, 0x01], &mut out),
            Err(ProtoError::InvalidFrame)
        );
        assert_eq!(
            cobs_decode(&[0x02, 0x00], &mut out),
            Err(ProtoError::InvalidFrame)
        );
    }

    #[test]
    fn max_block_boundary_round_trips() {
        let mut payload = [0xFFu8; 255];
        payload[254] = 0x00;

        // [0xFF][254 data][0x01][0x01]
        let mut encoded = [0u8; 257];
        let len = cobs_encode(&payload, &mut encoded).unwrap();
        assert_eq!(len, 257);
        assert_eq!(encoded[0], 0xFF);
        assert_eq!(encoded[255], 0x01);
        assert_eq!(encoded[256], 0x01);

        let mut decoded = [0u8; 255];
        assert_eq!(cobs_decode(&encoded[..len], &mut decoded), Ok(255));
        assert_eq!(decoded, payload);
    }
}
