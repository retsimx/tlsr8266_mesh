use super::*;
use crate::commands::run;
use crate::commands::test_support::*;
use crate::transport::MockTransport;
use crate::Command;
use probe_proto::Response;

/// Memory commands must never reset or control the core.
fn assert_no_core_control(requests: &[Request]) {
    assert!(
        !requests
            .iter()
            .any(|req| matches!(req, Request::Reset { .. } | Request::CpuCtrl { .. })),
        "memory commands must not reset or control the core: {requests:?}"
    );
}

#[test]
fn read_accepts_wire_and_physical_addresses() {
    for spec in ["0x808000", "0x8000", "32768"] {
        let mut mock = MockTransport::new([Response::Ack, data([0xde, 0xad, 0xbe, 0xef])]);

        let output = read(&mut mock, None, spec, None).expect("read succeeds");

        assert_eq!(
            output,
            Output::Read {
                addr: 0x8000,
                data_hex: "deadbeef".to_owned(),
            },
            "{spec}"
        );
        assert_eq!(
            mock.requests,
            [sws_activate_req(), read_mem_req(0x8000, 4)],
            "{spec}"
        );
        assert_no_core_control(&mock.requests);
    }
}

#[test]
fn read_maps_the_low_mmio_window_to_wire_zero() {
    let mut mock = MockTransport::new([Response::Ack, data([0x12, 0x34, 0x56, 0x78])]);

    let output = read(&mut mock, None, "0x800000", None).expect("read succeeds");

    assert_eq!(
        output,
        Output::Read {
            addr: 0x0000,
            data_hex: "12345678".to_owned(),
        }
    );
    assert_eq!(mock.requests, [sws_activate_req(), read_mem_req(0x0000, 4)]);
    assert_no_core_control(&mock.requests);
}

#[test]
fn read_defaults_to_the_symbol_size_then_four_bytes() {
    let mut sized = MockTransport::new([Response::Ack, data([0u8; 6])]);
    let output = read_span(
        &mut sized,
        &AddrSpec::Symbol {
            value: 0x808000,
            size: 6,
        },
        None,
    )
    .expect("sized symbol read succeeds");

    assert_eq!(
        output,
        Output::Read {
            addr: 0x8000,
            data_hex: "000000000000".to_owned(),
        }
    );
    assert_eq!(
        sized.requests,
        [sws_activate_req(), read_mem_req(0x8000, 6)]
    );

    // A zero `st_size` is an unknown size, so it falls back to 4 bytes.
    let mut unbounded = MockTransport::new([Response::Ack, data([0x11, 0x22, 0x33, 0x44])]);
    let output = read_span(
        &mut unbounded,
        &AddrSpec::Symbol {
            value: 0x808010,
            size: 0,
        },
        None,
    )
    .expect("unsized symbol read succeeds");

    assert_eq!(
        output,
        Output::Read {
            addr: 0x8010,
            data_hex: "11223344".to_owned(),
        }
    );
    assert_eq!(
        unbounded.requests,
        [sws_activate_req(), read_mem_req(0x8010, 4)]
    );
}

#[test]
fn read_of_2500_bytes_chunks_at_1024() {
    let mut mock = MockTransport::new([
        Response::Ack,
        data(vec![0x11; 1024]),
        data(vec![0x22; 1024]),
        data(vec![0x33; 452]),
    ]);

    let output = read(&mut mock, None, "0x8000", Some("2500")).expect("read succeeds");

    let Output::Read { addr, data_hex } = output else {
        panic!("read must return Output::Read");
    };
    assert_eq!(addr, 0x8000);
    assert_eq!(data_hex.len(), 2500 * 2);
    assert!(data_hex.starts_with("1111"), "{data_hex}");
    assert!(data_hex.ends_with("3333"), "{data_hex}");
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            read_mem_req(0x8000, 1024),
            read_mem_req(0x8400, 1024),
            read_mem_req(0x8800, 452),
        ]
    );
    assert_no_core_control(&mock.requests);
}

#[test]
fn read_at_the_end_of_the_low_window_is_one_chunk() {
    let mut mock = MockTransport::new([Response::Ack, data([0xa5; 16])]);

    let output = read(&mut mock, None, "0x0ff0", Some("16")).expect("read succeeds");

    assert_eq!(
        output,
        Output::Read {
            addr: 0x0ff0,
            data_hex: "a5".repeat(16),
        }
    );
    assert_eq!(
        mock.requests,
        [sws_activate_req(), read_mem_req(0x0ff0, 16)]
    );
}

#[test]
fn spans_outside_one_window_fail_before_any_request() {
    let cases = [
        ("0x0ff0", "32"),    // crosses 0x0fff
        ("0xbff0", "64"),    // crosses 0xbfff
        ("0x1000", "4"),     // dead zone
        ("0x8000", "16385"), // one byte past the SRAM window
    ];
    for (spec, len) in cases {
        let mut mock = MockTransport::new([]);

        let err = read(&mut mock, None, spec, Some(len)).expect_err("span must be rejected");

        assert!(
            matches!(err, CliError::InvalidAddress { .. }),
            "{spec}+{len}: {err:?}"
        );
        assert!(mock.requests.is_empty(), "{spec}+{len} sent a request");
    }
}

#[test]
fn zero_length_reads_and_empty_writes_are_rejected_before_any_request() {
    let mut read_mock = MockTransport::new([]);
    let err =
        read(&mut read_mock, None, "0x8000", Some("0")).expect_err("zero length must be rejected");
    assert!(matches!(err, CliError::InvalidAddress { .. }), "{err:?}");
    assert!(read_mock.requests.is_empty());

    let mut write_mock = MockTransport::new([]);
    let err =
        write(&mut write_mock, None, "0x8000", "  _ ").expect_err("empty payload must be rejected");
    assert!(
        matches!(&err, CliError::Device(message) if message.contains("empty")),
        "{err:?}"
    );
    assert!(write_mock.requests.is_empty());
}

#[test]
fn short_data_replies_are_device_errors() {
    let mut mock = MockTransport::new([Response::Ack, data([0x00, 0x01, 0x02])]);

    let err = read(&mut mock, None, "0x8000", None).expect_err("short reply must fail");

    assert_eq!(
        err,
        CliError::Device("short read: got 3, want 4".to_owned())
    );
}

#[test]
fn flash_symbols_are_rejected_before_any_request() {
    let mut mock = MockTransport::new([]);

    let err = read_span(
        &mut mock,
        &AddrSpec::Symbol {
            value: 0x20,
            size: 0,
        },
        None,
    )
    .expect_err("a flash symbol must not become an MMIO read");

    assert!(matches!(err, CliError::InvalidAddress { .. }), "{err:?}");
    assert!(mock.requests.is_empty());
}

#[test]
fn symbols_need_an_elf() {
    let mut mock = MockTransport::new([]);
    let err = read(&mut mock, None, "__irq", None).expect_err("symbol needs --elf");
    assert_eq!(err, CliError::MissingElf);
    assert!(mock.requests.is_empty());

    let mut mock = MockTransport::new([]);
    let err = write(&mut mock, None, "__irq", "00").expect_err("symbol needs --elf");
    assert_eq!(err, CliError::MissingElf);
    assert!(mock.requests.is_empty());

    // Malformed hex is a bad address, not a missing ELF.
    let mut mock = MockTransport::new([]);
    let err = read(&mut mock, None, "0xzz", None).expect_err("bad hex must fail");
    assert!(matches!(err, CliError::InvalidAddress { .. }), "{err:?}");
    assert!(mock.requests.is_empty());
}

#[test]
fn oversized_symbol_lengths_name_the_16_bit_span() {
    let mut mock = MockTransport::new([]);
    let err = read_span(
        &mut mock,
        &AddrSpec::Symbol {
            value: 0x808000,
            size: 70_000,
        },
        None,
    )
    .expect_err("a symbol larger than u16::MAX needs an explicit length");

    assert!(
        matches!(&err, CliError::InvalidAddress { message } if message.contains("16-bit span")),
        "{err:?}"
    );
    assert!(mock.requests.is_empty());
}

#[test]
fn write_chunks_at_256_and_acks_each_chunk() {
    let payload: Vec<u8> = (0..600_u16).map(|byte| byte as u8).collect();
    let mut mock = MockTransport::new([Response::Ack, Response::Ack, Response::Ack, Response::Ack]);

    let output = write(&mut mock, None, "0x8000", &hex_string(&payload)).expect("write succeeds");

    assert_eq!(
        output,
        Output::Write {
            addr: 0x8000,
            len: 600,
        }
    );
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            write_mem_req(0x8000, &payload[..256]),
            write_mem_req(0x8100, &payload[256..512]),
            write_mem_req(0x8200, &payload[512..]),
        ]
    );
    assert_no_core_control(&mock.requests);
}

#[test]
fn write_clamps_the_last_chunk_to_the_window_end() {
    let payload = vec![0x5a; 272];
    let mut mock = MockTransport::new([Response::Ack, Response::Ack, Response::Ack]);

    let output = write(&mut mock, None, "0xbef0", &hex_string(&payload)).expect("write succeeds");

    assert_eq!(
        output,
        Output::Write {
            addr: 0xbef0,
            len: 272,
        }
    );
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            write_mem_req(0xbef0, &payload[..256]),
            write_mem_req(0xbff0, &payload[256..]),
        ]
    );

    let mut rejected = MockTransport::new([]);
    let err = write(&mut rejected, None, "0xbff0", &hex_string(&[0u8; 32]))
        .expect_err("write past 0xbfff must fail");
    assert!(matches!(err, CliError::InvalidAddress { .. }), "{err:?}");
    assert!(rejected.requests.is_empty());
}

#[test]
fn write_is_bounded_by_a_non_zero_symbol_size() {
    let sized = AddrSpec::Symbol {
        value: 0x808000,
        size: 4,
    };

    let mut mock = MockTransport::new([Response::Ack, Response::Ack]);
    let output = write_span(&mut mock, &sized, "0xde 0xad_be ef").expect("write succeeds");
    assert_eq!(
        output,
        Output::Write {
            addr: 0x8000,
            len: 4,
        }
    );
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            write_mem_req(0x8000, &[0xde, 0xad, 0xbe, 0xef]),
        ]
    );

    let mut rejected = MockTransport::new([]);
    let err =
        write_span(&mut rejected, &sized, "deadbeefff").expect_err("oversized payload must fail");
    assert!(
        matches!(&err, CliError::Device(message) if message.contains("holds 4")),
        "{err:?}"
    );
    assert!(rejected.requests.is_empty());
}

#[test]
fn hex_payloads_tolerate_prefixes_and_separators() {
    let cases = [
        ("0xdeadbeef", vec![0xde, 0xad, 0xbe, 0xef]),
        ("DEAD BEEF", vec![0xde, 0xad, 0xbe, 0xef]),
        ("0xde 0xad_be ef", vec![0xde, 0xad, 0xbe, 0xef]),
        ("0x00_0x11", vec![0x00, 0x11]),
    ];
    for (text, expected) in cases {
        assert_eq!(parse_hex_bytes(text), Ok(expected), "{text}");
    }

    for text in ["", "   ", "_", "0xzz", "abc"] {
        assert!(
            matches!(parse_hex_bytes(text), Err(CliError::Device(_))),
            "{text}"
        );
    }
}

#[test]
fn run_dispatches_to_the_memory_handlers() {
    let mut read_ctx = test_ctx([Response::Ack, data([0x01, 0x02, 0x03, 0x04])]);
    let mut out = Vec::new();
    let output = run(
        Command::Read {
            addr_or_symbol: "0x8000".to_owned(),
            len: Some("4".to_owned()),
        },
        &mut read_ctx,
        &mut out,
    )
    .expect("read runs")
    .expect("read returns an output");

    assert_eq!(
        output,
        Output::Read {
            addr: 0x8000,
            data_hex: "01020304".to_owned(),
        }
    );

    let mut write_ctx = test_ctx([Response::Ack, Response::Ack]);
    let mut out = Vec::new();
    let output = run(
        Command::Write {
            addr_or_symbol: "0x8000".to_owned(),
            hex: "0x01 0x02 0x03 0x04".to_owned(),
        },
        &mut write_ctx,
        &mut out,
    )
    .expect("write runs")
    .expect("write returns an output");

    assert_eq!(
        output,
        Output::Write {
            addr: 0x8000,
            len: 4,
        }
    );
}
