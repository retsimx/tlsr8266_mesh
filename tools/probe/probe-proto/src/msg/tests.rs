//! Body codec tests for both message directions.

use super::*;
use heapless::Vec;

fn write_payload(data: &[u8]) -> Vec<u8, MAX_WRITE_PAYLOAD> {
    Vec::from_slice(data).expect("payload fits")
}

fn data_payload(len: usize) -> Vec<u8, MAX_DATA_PAYLOAD> {
    let mut data = Vec::new();
    for i in 0..len {
        data.push(i as u8).expect("payload fits");
    }
    data
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
            data: write_payload(&[0x5a; MAX_WRITE_PAYLOAD]),
        },
        Request::ReadAnalog { addr: 0x0c },
        Request::WriteAnalog {
            addr: 0x0c,
            val: 0xff,
        },
        Request::FlashJedecId,
        Request::FlashRead {
            addr: 0x7ff00,
            len: 256,
        },
        Request::FlashEraseSector { addr: 0x1000 },
        Request::FlashWritePage {
            addr: 0x2000,
            data: write_payload(&[0xa5; MAX_WRITE_PAYLOAD]),
        },
        Request::CpuCtrl {
            mode: CpuMode::SoftReboot,
        },
        Request::ReadRegs,
        Request::SetBreakpoint { slot: 2, addr: 0 },
        Request::RttPoll { ctrl_addr: 0xbef0 },
    ]
}

fn assert_request_roundtrip(request: &Request) {
    let mut buf = [0u8; 4 + MAX_WRITE_PAYLOAD];
    let len = request
        .encode_body(&mut buf)
        .expect("valid request encodes");
    let decoded = Request::decode_body(request.tag(), &buf[..len]).expect("body decodes");
    assert_eq!(&decoded, request);
}

fn assert_response_roundtrip(response: &Response) {
    let mut buf = [0u8; MAX_DATA_PAYLOAD];
    let len = response
        .encode_body(&mut buf)
        .expect("valid response encodes");
    let decoded = Response::decode_body(response.tag(), &buf[..len]).expect("body decodes");
    assert_eq!(&decoded, response);
}

#[test]
fn request_tags_match_the_table() {
    let expected: [u8; 15] = [
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    ];
    let mut mask = 0u32;
    for (sample, tag) in request_samples().iter().zip(expected) {
        assert_eq!(sample.tag(), tag);
        assert_eq!(mask & (1 << tag), 0, "duplicate request tag");
        mask |= 1 << tag;
    }
    // Tags 0x01..=0x0f exactly.
    assert_eq!(mask, 0xfffe);
}

#[test]
fn response_tags_match_the_table() {
    let expected: [u8; 7] = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x81];
    let samples: [Response; 7] = [
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
        Response::Data(data_payload(4)),
        Response::JedecId([0xef, 0x40, 0x18]),
        Response::Error(ProtoError::InvalidFrame),
    ];
    for (index, sample) in samples.iter().enumerate() {
        assert_eq!(sample.tag(), expected[index]);
        assert!(
            !expected[..index].contains(&sample.tag()),
            "duplicate response tag"
        );
    }
}

#[test]
fn every_request_roundtrips() {
    for request in &request_samples() {
        assert_request_roundtrip(request);
    }
    // `addr == 0` clears a breakpoint slot but is still a valid body.
    assert_request_roundtrip(&Request::SetBreakpoint { slot: 0, addr: 0 });
    assert_request_roundtrip(&Request::RttPoll { ctrl_addr: 0x8000 });
}

#[test]
fn every_response_roundtrips() {
    for response in [
        Response::Ack,
        Response::Pong {
            fw_ver: 0xbeef,
            proto_ver: 1,
        },
        Response::ChipId {
            rev: 7,
            chip_id: 0x1234,
        },
        Response::Byte(0),
        Response::Data(data_payload(0)),
        Response::Data(data_payload(1)),
        Response::Data(data_payload(MAX_DATA_PAYLOAD)),
        Response::JedecId([0xef, 0x40, 0x18]),
    ] {
        assert_response_roundtrip(&response);
    }
    for code in 1..=7u8 {
        assert_response_roundtrip(&Response::Error(ProtoError::from_code(code)));
    }
}

#[test]
fn encode_rejects_invalid_arguments() {
    let mut buf = [0u8; 4 + MAX_WRITE_PAYLOAD];
    let rejects: [(Request, ProtoError); 11] = [
        (Request::SwsActivate { div: 0 }, ProtoError::InvalidArg),
        (
            Request::Reset {
                hold_ms: 0,
                stop_cpu: false,
            },
            ProtoError::InvalidArg,
        ),
        (
            Request::ReadMem {
                addr: 0x8000,
                len: 0,
            },
            ProtoError::InvalidArg,
        ),
        (
            Request::ReadMem {
                addr: 0x8000,
                len: 1025,
            },
            ProtoError::InvalidArg,
        ),
        (
            Request::ReadMem {
                addr: 0x1000,
                len: 1,
            },
            ProtoError::InvalidAddrRange,
        ),
        (
            Request::FlashRead {
                addr: 0x80000,
                len: 1,
            },
            ProtoError::InvalidAddrRange,
        ),
        (
            Request::FlashWritePage {
                addr: 0xf0,
                data: write_payload(&[0; 32]),
            },
            ProtoError::InvalidArg,
        ),
        (
            Request::FlashEraseSector { addr: 100 },
            ProtoError::InvalidArg,
        ),
        (
            Request::SetBreakpoint { slot: 4, addr: 0 },
            ProtoError::InvalidArg,
        ),
        (
            Request::SetBreakpoint {
                slot: 0,
                addr: 0x0100_0000,
            },
            ProtoError::InvalidArg,
        ),
        (
            Request::RttPoll { ctrl_addr: 0x7fff },
            ProtoError::InvalidAddrRange,
        ),
    ];
    for (request, expected) in rejects {
        assert_eq!(request.encode_body(&mut buf), Err(expected));
    }
    assert_eq!(
        Request::RttPoll { ctrl_addr: 0xbef1 }.encode_body(&mut buf),
        Err(ProtoError::InvalidAddrRange)
    );
    assert_eq!(
        Request::WriteMem {
            addr: 0x8000,
            data: write_payload(&[]),
        }
        .encode_body(&mut buf),
        Err(ProtoError::InvalidArg)
    );
}

#[test]
fn flash_erase_sector_separates_range_from_alignment() {
    let mut buf = [0u8; 4];
    // Past the end of flash is a range fault, like `FlashRead`/`FlashWritePage`.
    assert_eq!(
        Request::FlashEraseSector { addr: 0x80000 }.encode_body(&mut buf),
        Err(ProtoError::InvalidAddrRange)
    );
    assert_eq!(
        Request::decode_body(0x0a, &[0x00, 0x00, 0x08, 0x00]),
        Err(ProtoError::InvalidAddrRange)
    );
    // Misalignment is an argument fault.
    assert_eq!(
        Request::FlashEraseSector { addr: 0x1001 }.encode_body(&mut buf),
        Err(ProtoError::InvalidArg)
    );
    assert_eq!(
        Request::decode_body(0x0a, &[0x01, 0x10, 0x00, 0x00]),
        Err(ProtoError::InvalidArg)
    );
    // In range and aligned still encodes.
    assert_eq!(
        Request::FlashEraseSector { addr: 0x1000 }.encode_body(&mut buf),
        Ok(4)
    );
}

#[test]
fn encode_rejects_a_short_output_buffer() {
    let request = Request::ReadMem {
        addr: 0x8000,
        len: 1,
    };
    let mut buf = [0u8; 3];
    assert_eq!(request.encode_body(&mut buf), Err(ProtoError::InvalidArg));

    let mut empty = [0u8; 0];
    assert_eq!(
        Request::Ping.encode_body(&mut empty),
        Ok(0),
        "an empty body fits an empty buffer"
    );
    assert_eq!(
        Response::Pong {
            fw_ver: 1,
            proto_ver: 1,
        }
        .encode_body(&mut empty),
        Err(ProtoError::InvalidArg)
    );
}

#[test]
fn decode_rejects_invalid_arguments() {
    // `div == 0`.
    assert_eq!(
        Request::decode_body(0x03, &[0]),
        Err(ProtoError::InvalidArg)
    );
    // `hold_ms == 0`.
    assert_eq!(
        Request::decode_body(0x02, &[0, 0, 0]),
        Err(ProtoError::InvalidArg)
    );
    // `ReadMem` zero length and 1025.
    assert_eq!(
        Request::decode_body(0x04, &[0x00, 0x80, 0x00, 0x00]),
        Err(ProtoError::InvalidArg)
    );
    assert_eq!(
        Request::decode_body(0x04, &[0x00, 0x80, 0x01, 0x04]),
        Err(ProtoError::InvalidArg)
    );
    // Unmapped SWire address `0x1000`.
    assert_eq!(
        Request::decode_body(0x04, &[0x00, 0x10, 0x01, 0x00]),
        Err(ProtoError::InvalidAddrRange)
    );
    // `RttPoll` just below SRAM and one byte past the window end.
    assert_eq!(
        Request::decode_body(0x0f, &[0xff, 0x7f]),
        Err(ProtoError::InvalidAddrRange)
    );
    assert_eq!(
        Request::decode_body(0x0f, &[0xf1, 0xbe]),
        Err(ProtoError::InvalidAddrRange)
    );
    assert_eq!(CpuMode::from_code(4), None);
    assert_eq!(CpuMode::from_code(0), Some(CpuMode::Stop));
    assert_eq!(CpuMode::from_code(3), Some(CpuMode::SoftReboot));
}

#[test]
fn body_lengths_are_strict() {
    let mut buf = [0u8; 8];
    let len = Request::ReadMem {
        addr: 0x8000,
        len: 4,
    }
    .encode_body(&mut buf)
    .unwrap();
    assert_eq!(len, 4);
    assert_eq!(
        Request::decode_body(0x04, &buf[..len - 1]),
        Err(ProtoError::InvalidFrame)
    );
    assert_eq!(
        Request::decode_body(0x04, &buf[..=len]),
        Err(ProtoError::InvalidFrame)
    );

    // Empty bodies reject any payload.
    for tag in [0x01u8, 0x08, 0x0d] {
        assert_eq!(
            Request::decode_body(tag, &[0]),
            Err(ProtoError::InvalidFrame)
        );
    }
    // Ping with a non-empty body.
    assert_eq!(
        Request::decode_body(0x01, &[0x00, 0x00]),
        Err(ProtoError::InvalidFrame)
    );
    // `stop_cpu` is boolean on the wire.
    assert_eq!(
        Request::decode_body(0x02, &[0x01, 0x00, 0x02]),
        Err(ProtoError::InvalidFrame)
    );
    // Unknown request tags.
    assert_eq!(
        Request::decode_body(0x10, &[]),
        Err(ProtoError::InvalidFrame)
    );
    assert_eq!(
        Request::decode_body(0x99, &[]),
        Err(ProtoError::InvalidFrame)
    );
    assert_eq!(
        Request::decode_body(0x00, &[]),
        Err(ProtoError::InvalidFrame)
    );

    // `WriteMem` / `FlashWritePage` need their fixed prefix.
    assert_eq!(
        Request::decode_body(0x05, &[0x00]),
        Err(ProtoError::InvalidFrame)
    );
    assert_eq!(
        Request::decode_body(0x0b, &[0x00, 0x00, 0x00]),
        Err(ProtoError::InvalidFrame)
    );

    // `Error` bodies are empty, and only codes 1..=7 exist.
    assert_eq!(
        Response::decode_body(0x81, &[0x01]),
        Err(ProtoError::InvalidFrame)
    );
    for tag in [0x80u8, 0x88, 0xff] {
        assert_eq!(
            Response::decode_body(tag, &[]),
            Err(ProtoError::InvalidFrame)
        );
    }
    assert_eq!(
        Response::decode_body(0x06, &[]),
        Err(ProtoError::InvalidFrame)
    );

    // `Data` above its capacity is structurally invalid.
    let oversized = [0u8; MAX_DATA_PAYLOAD + 1];
    assert_eq!(
        Response::decode_body(0x04, &oversized),
        Err(ProtoError::InvalidFrame)
    );
}

#[test]
fn error_codes_map_both_ways() {
    let all = [
        ProtoError::InvalidFrame,
        ProtoError::BadCrc,
        ProtoError::InvalidAddrRange,
        ProtoError::InvalidArg,
        ProtoError::SwsTimeout,
        ProtoError::FlashVerifyFailed,
        ProtoError::FlashBusyTimeout,
    ];
    for (index, error) in all.iter().enumerate() {
        let code = (index + 1) as u8;
        assert_eq!(error.code(), code);
        assert_eq!(ProtoError::from_code(code), *error);
        assert_eq!(Response::Error(*error).tag(), 0x80 | code);
        assert_eq!(
            Response::decode_body(0x80 | code, &[]),
            Ok(Response::Error(*error))
        );
    }

    assert_eq!(ProtoError::from_code(0), ProtoError::InvalidFrame);
    assert_eq!(ProtoError::from_code(8), ProtoError::InvalidFrame);
    assert_eq!(ProtoError::from_code(0xff), ProtoError::InvalidFrame);
}
