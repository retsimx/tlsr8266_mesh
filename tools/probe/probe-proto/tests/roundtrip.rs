//! End-to-end framing tests through the crate's public API.
//!
//! Every request and response variant is encoded with `encode_frame`, decoded
//! with `decode_frame`, and compared for exact equality. Deterministic
//! corruption sweeps then flip every bit of a representative `WriteMem` frame
//! and a maximum-size `Data` frame, asserting that no mutation ever decodes.

use heapless::Vec;
use probe_proto::{
    decode_frame, encode_frame, Direction, Message, ProtoError, Request, Response,
    MAX_DATA_PAYLOAD, MAX_WIRE_FRAME, MAX_WRITE_PAYLOAD,
};

/// Builds a deterministic, non-uniform, all-non-zero payload.
///
/// Non-zero bytes keep every COBS block full (254 data bytes per `0xFF` code
/// byte), so the maximum `Data` frame is exactly 1034 bytes on the wire and
/// mutations cannot accidentally look like an implicit-zero block boundary.
fn patterned<const N: usize>(len: usize, seed: u8) -> Vec<u8, N> {
    let mut data = Vec::new();
    for index in 0..len {
        let byte = (index as u8).wrapping_mul(37).wrapping_add(seed) | 1;
        data.push(byte).expect("payload fits");
    }
    data
}

/// Encodes a message and returns the wire bytes plus their length.
///
/// The buffer starts dirty (`0xaa`), so every test here also pins that the
/// encoder writes its own delimiter instead of relying on a zeroed byte.
fn encoded(seq: u8, msg: &Message) -> ([u8; MAX_WIRE_FRAME], usize) {
    let mut wire = [0xaau8; MAX_WIRE_FRAME];
    let len = encode_frame(seq, msg, &mut wire).expect("frame encodes");
    (wire, len)
}

/// Encodes `msg`, decodes it with and without the trailing delimiter, and
/// checks the sequence byte and message survive unchanged.
fn assert_round_trip(direction: Direction, seq: u8, msg: &Message) {
    let (wire, len) = encoded(seq, msg);
    assert!(len <= MAX_WIRE_FRAME, "frame exceeds the wire budget");
    assert_eq!(wire[len - 1], 0x00, "frame ends with the delimiter");

    assert_eq!(
        decode_frame(direction, &wire[..len]),
        Ok((seq, msg.clone()))
    );
    assert_eq!(
        decode_frame(direction, &wire[..len - 1]),
        Ok((seq, msg.clone())),
        "the delimiter is optional on input"
    );
}

fn request_samples() -> [Request; 15] {
    [
        Request::Ping,
        Request::Reset {
            hold_ms: 1,
            stop_cpu: true,
        },
        Request::SwsActivate { div: 8 },
        Request::ReadMem {
            addr: 0x8000,
            len: 64,
        },
        Request::WriteMem {
            addr: 0x8000,
            data: patterned(MAX_WRITE_PAYLOAD, 0x5a),
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
            // 256 bytes starting at a page boundary stay inside one page.
            addr: 0x2000,
            data: patterned(MAX_WRITE_PAYLOAD, 0xa5),
        },
        Request::CpuCtrl {
            mode: probe_proto::CpuMode::SoftReboot,
        },
        Request::ReadRegs,
        Request::SetBreakpoint {
            slot: 3,
            addr: 0x00ff_ffff,
        },
        Request::RttPoll { ctrl_addr: 0xbef0 },
    ]
}

fn response_samples() -> [Response; 9] {
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
        Response::Data(patterned(0, 0x00)),
        Response::Data(patterned(1, 0x00)),
        Response::Data(patterned(MAX_DATA_PAYLOAD, 0x11)),
        Response::JedecId([0xef, 0x40, 0x18]),
        Response::Error(ProtoError::InvalidFrame),
    ]
}

#[test]
fn every_request_variant_round_trips() {
    for (index, request) in request_samples().into_iter().enumerate() {
        let seq = 0x40 + index as u8;
        assert_round_trip(Direction::Request, seq, &Message::Request(request));
    }
}

#[test]
fn every_response_variant_round_trips() {
    for (index, response) in response_samples().into_iter().enumerate() {
        let seq = 0xa0 + index as u8;
        assert_round_trip(Direction::Response, seq, &Message::Response(response));
    }

    // Every defined `Error` code uses its own `0x80 | code` tag.
    for code in 1..=7u8 {
        assert_round_trip(
            Direction::Response,
            code,
            &Message::Response(Response::Error(ProtoError::from_code(code))),
        );
    }
}

#[test]
fn delimiter_is_optional_on_every_frame() {
    // `assert_round_trip` checks both forms; this test makes the requirement
    // explicit and independent of the equality assertions above.
    let frames: [(Direction, u8, Message); 3] = [
        (Direction::Request, 0x11, Message::Request(Request::Ping)),
        (
            Direction::Request,
            0x22,
            Message::Request(Request::ReadMem {
                addr: 0x8000,
                len: 1,
            }),
        ),
        (Direction::Response, 0x33, Message::Response(Response::Ack)),
    ];
    for (direction, seq, msg) in frames {
        let (wire, len) = encoded(seq, &msg);
        assert_eq!(wire[len - 1], 0x00);
        assert!(decode_frame(direction, &wire[..len]).is_ok());
        assert!(decode_frame(direction, &wire[..len - 1]).is_ok());
    }
}

#[test]
fn dirty_buffer_encode_writes_its_own_delimiter() {
    // `out` is caller scratch, not guaranteed zeroed. A short frame must end at
    // `out[cobs_len]` with its own `0x00`, never rely on the byte already there.
    let short = Message::Response(Response::Error(ProtoError::SwsTimeout));
    let mut wire = [0xaau8; MAX_WIRE_FRAME];

    let len = encode_frame(7, &short, &mut wire).unwrap();
    assert_eq!(wire[len - 1], 0x00, "delimiter follows the frame");
    assert_eq!(
        wire[..len].iter().filter(|&&byte| byte == 0x00).count(),
        1,
        "the delimiter is the only zero byte in the frame"
    );
    assert_eq!(
        decode_frame(Direction::Response, &wire[..len]),
        Ok((7, short))
    );
}

#[test]
fn reused_buffer_keeps_the_delimiter_after_the_frame() {
    // Consumers (#20/#21) encode successive frames into one buffer. A shorter
    // frame must write its own delimiter over the previous frame's tail instead
    // of leaving a stale byte inside the returned frame.
    let mut wire = [0xaau8; MAX_WIRE_FRAME];

    let long = Message::Response(Response::JedecId([0xef, 0x40, 0x18]));
    let long_len = encode_frame(7, &long, &mut wire).unwrap();
    assert_eq!(wire[long_len - 1], 0x00);
    assert_eq!(
        decode_frame(Direction::Response, &wire[..long_len]),
        Ok((7, long))
    );

    let short = Message::Response(Response::Error(ProtoError::SwsTimeout));
    let len = encode_frame(7, &short, &mut wire).unwrap();
    assert!(
        len < long_len,
        "the short frame is shorter than the long one"
    );
    assert_eq!(wire[len - 1], 0x00, "delimiter follows the frame");
    assert_eq!(
        wire[..len].iter().filter(|&&byte| byte == 0x00).count(),
        1,
        "the delimiter is the only zero byte in the frame"
    );
    assert_eq!(
        decode_frame(Direction::Response, &wire[..len]),
        Ok((7, short))
    );
}

#[test]
fn maximum_data_frame_fits_the_wire_budget() {
    let msg = Message::Response(Response::Data(patterned(MAX_DATA_PAYLOAD, 0x11)));
    let (wire, len) = encoded(0, &msg);

    // 1024 body + seq + tag + CRC = 1028 decoded bytes, 5 COBS code bytes,
    // then the delimiter.
    assert_eq!(len, 1034);
    assert!(len <= MAX_WIRE_FRAME);
    let cobs_len = len - 1;
    assert!(
        cobs_len < MAX_WIRE_FRAME,
        "COBS output fits before the delimiter"
    );
    assert_eq!(wire[len - 1], 0x00, "the last byte is the delimiter");
    assert_eq!(
        decode_frame(Direction::Response, &wire[..len]),
        Ok((0, msg))
    );
}

/// Fails unless a mutated frame is rejected as a CRC mismatch or a structural
/// fault. `InvalidArg`/`InvalidAddrRange` also fail: the CRC is checked before
/// the body, so a mutated frame must never reach message validation.
fn assert_rejected(direction: Direction, wire: &[u8], context: &str) {
    let result = decode_frame(direction, wire);
    assert!(
        matches!(
            result,
            Err(ProtoError::BadCrc) | Err(ProtoError::InvalidFrame)
        ),
        "{context} decoded as {result:?}"
    );
}

/// Copies `wire`, corrupts exactly one byte of the copy, and asserts the result
/// is rejected.
fn assert_mutation_rejected(
    direction: Direction,
    wire: &[u8],
    index: usize,
    value: u8,
    context: &str,
) {
    let mut mutated = [0u8; MAX_WIRE_FRAME];
    mutated[..wire.len()].copy_from_slice(wire);
    mutated[index] = value;
    assert_rejected(direction, &mutated[..wire.len()], context);
}

/// Inverts every byte once, then flips each of its eight bits, one mutation at
/// a time, and asserts no mutated frame decodes. Each case starts from a fresh
/// copy of `wire`, so only the byte under test is ever corrupt.
fn corruption_sweep(direction: Direction, wire: &[u8], label: &str) -> usize {
    let mut mutations = 0;
    for index in 0..wire.len() {
        assert_mutation_rejected(
            direction,
            wire,
            index,
            wire[index] ^ 0xff,
            &format!("{label}: byte {index} ^ 0xff"),
        );
        mutations += 1;

        for bit in 0..8u8 {
            assert_mutation_rejected(
                direction,
                wire,
                index,
                wire[index] ^ (1u8 << bit),
                &format!("{label}: byte {index} bit {bit}"),
            );
            mutations += 1;
        }
    }
    mutations
}

#[test]
fn corruption_is_always_detected() {
    // Representative `WriteMem` frame: 64 patterned bytes at `0x8000`.
    let write_mem = Message::Request(Request::WriteMem {
        addr: 0x8000,
        data: patterned(64, 0x5c),
    });
    let (wire, write_len) = encoded(0x5a, &write_mem);
    assert_eq!(
        decode_frame(Direction::Request, &wire[..write_len]),
        Ok((0x5a, write_mem))
    );
    let write_mutations = corruption_sweep(Direction::Request, &wire[..write_len], "WriteMem");

    // Maximum `Data` frame: 1024 patterned bytes.
    let data = Message::Response(Response::Data(patterned(MAX_DATA_PAYLOAD, 0x91)));
    let (wire, data_len) = encoded(0x3c, &data);
    assert_eq!(data_len, 1034);
    assert_eq!(
        decode_frame(Direction::Response, &wire[..data_len]),
        Ok((0x3c, data))
    );
    let data_mutations = corruption_sweep(Direction::Response, &wire[..data_len], "Data(1024)");

    // One full-byte inversion plus eight single-bit flips per byte, each from
    // the pristine frame.
    assert_eq!(write_mutations, 9 * write_len);
    assert_eq!(data_mutations, 9 * data_len);
}

#[test]
fn structural_faults_are_rejected() {
    // No bytes at all.
    assert!(matches!(
        decode_frame(Direction::Request, &[]),
        Err(ProtoError::InvalidFrame)
    ));
    // A lone delimiter is an empty COBS payload.
    assert!(matches!(
        decode_frame(Direction::Request, &[0x00]),
        Err(ProtoError::InvalidFrame)
    ));

    let msg = Message::Request(Request::ReadMem {
        addr: 0x8000,
        len: 4,
    });
    let (wire, len) = encoded(0x33, &msg);
    assert!(decode_frame(Direction::Request, &wire[..len]).is_ok());

    // A delimiter inserted mid-payload: the first zero terminates the frame
    // early, leaving bytes after it.
    let mut embedded = [0u8; MAX_WIRE_FRAME + 1];
    embedded[..len].copy_from_slice(&wire[..len]);
    let split = len / 2;
    embedded.copy_within(split..len, split + 1);
    embedded[split] = 0x00;
    assert!(matches!(
        decode_frame(Direction::Request, &embedded[..=len]),
        Err(ProtoError::InvalidFrame)
    ));

    // Junk after the delimiter.
    let mut trailing = [0u8; MAX_WIRE_FRAME + 1];
    trailing[..len].copy_from_slice(&wire[..len]);
    trailing[len] = 0xaa;
    assert!(matches!(
        decode_frame(Direction::Request, &trailing[..=len]),
        Err(ProtoError::InvalidFrame)
    ));

    // The encoded payload with its last byte dropped (CRC tail truncated),
    // re-terminated so the fault is the truncation, not the missing delimiter.
    let mut truncated = [0u8; MAX_WIRE_FRAME];
    let truncated_len = len - 2;
    truncated[..truncated_len].copy_from_slice(&wire[..truncated_len]);
    truncated[truncated_len] = 0x00;
    assert!(matches!(
        decode_frame(Direction::Request, &truncated[..=truncated_len]),
        Err(ProtoError::BadCrc) | Err(ProtoError::InvalidFrame)
    ));

    // One byte over the largest legal encoded payload.
    let oversized = [0x55u8; MAX_WIRE_FRAME];
    assert_eq!(oversized.len(), MAX_WIRE_FRAME - 1 + 1);
    assert!(matches!(
        decode_frame(Direction::Request, &oversized),
        Err(ProtoError::InvalidFrame)
    ));
}

#[test]
fn direction_mismatch_never_decodes() {
    let msg = Message::Request(Request::Ping);
    let (wire, len) = encoded(1, &msg);
    assert_eq!(
        decode_frame(Direction::Request, &wire[..len]),
        Ok((1, msg.clone()))
    );
    assert!(
        matches!(
            decode_frame(Direction::Response, &wire[..len]),
            Err(ProtoError::InvalidFrame)
        ),
        "a Ping request decoded as a response"
    );
}

#[test]
fn validation_errors_pass_through_the_frame() {
    let mut out = [0u8; MAX_WIRE_FRAME];

    assert!(matches!(
        encode_frame(
            0,
            &Message::Request(Request::SwsActivate { div: 0 }),
            &mut out
        ),
        Err(ProtoError::InvalidArg)
    ));

    assert!(matches!(
        encode_frame(
            0,
            &Message::Request(Request::ReadMem {
                addr: 0x1000,
                len: 1,
            }),
            &mut out
        ),
        Err(ProtoError::InvalidAddrRange)
    ));

    // `FlashEraseSector` past the end of flash is a range fault; only a
    // misaligned in-range address is an argument fault.
    assert!(matches!(
        encode_frame(
            0,
            &Message::Request(Request::FlashEraseSector { addr: 0x80000 }),
            &mut out
        ),
        Err(ProtoError::InvalidAddrRange)
    ));
    assert!(matches!(
        encode_frame(
            0,
            &Message::Request(Request::FlashEraseSector { addr: 0x1001 }),
            &mut out
        ),
        Err(ProtoError::InvalidArg)
    ));
}
