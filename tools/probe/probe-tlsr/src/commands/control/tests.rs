use super::*;
use crate::commands::run;
use crate::commands::test_support::*;
use crate::commands::WATCHDOG_CTRL_ADDR;
use crate::commands::{RESET_SENTINEL_ADDR, SENTINEL_CLEAN_REBOOT};
use crate::output::FlashInfo;
use crate::Command;

#[test]
fn info_on_running_target_issues_no_flash_requests() {
    let mut mock = crate::transport::MockTransport::new([Response::Ack, pc_bytes(), data([0x00])]);

    let output = info(&mut mock, false).expect("info succeeds");

    assert_eq!(
        output,
        Output::Info {
            pc: 0x0001_10f0,
            cpu_status: 0x00,
            cpu_state: "running",
            flash_info: None,
        }
    );
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            read_mem_req(PC_ADDR, 4),
            read_mem_req(CPU_STATUS_ADDR, 1),
        ]
    );
}

#[test]
fn info_on_stopped_or_stalled_target_reads_flash() {
    for (status, state) in [(0x05, "stopped"), (0x06, "stalled")] {
        let mut mock = crate::transport::MockTransport::new([
            Response::Ack,
            pc_bytes(),
            data([status]),
            jedec_id(),
            header(0x8000_0dd3),
        ]);

        let output = info(&mut mock, false).expect("info succeeds");

        assert_eq!(
            output,
            Output::Info {
                pc: 0x0001_10f0,
                cpu_status: status,
                cpu_state: state,
                flash_info: Some(FlashInfo {
                    jedec_id: "ef4018".to_owned(),
                    xtal_mhz: 16,
                    build_version: 0x0dd3,
                    magic_ok: true,
                }),
            }
        );
        assert_eq!(
            mock.requests,
            [
                sws_activate_req(),
                read_mem_req(PC_ADDR, 4),
                read_mem_req(CPU_STATUS_ADDR, 1),
                Request::FlashJedecId,
                read_flash_req(FLASH_HEADER_ADDR, FLASH_HEADER_LEN),
            ]
        );
    }
}

#[test]
fn info_decodes_the_real_12mhz_header() {
    let mut mock = crate::transport::MockTransport::new([
        Response::Ack,
        pc_bytes(),
        data([0x05]),
        jedec_id(),
        data(REAL_12MHZ_HEADER),
    ]);

    let output = info(&mut mock, false).expect("info succeeds");

    assert_eq!(
        output,
        Output::Info {
            pc: 0x0001_10f0,
            cpu_status: 0x05,
            cpu_state: "stopped",
            flash_info: Some(FlashInfo {
                jedec_id: "ef4018".to_owned(),
                xtal_mhz: 12,
                build_version: 0x0dd4,
                magic_ok: true,
            }),
        }
    );
}

#[test]
fn info_decodes_the_real_16mhz_header() {
    let mut mock = crate::transport::MockTransport::new([
        Response::Ack,
        pc_bytes(),
        data([0x05]),
        jedec_id(),
        data(REAL_16MHZ_HEADER),
    ]);

    let output = info(&mut mock, false).expect("info succeeds");

    assert_eq!(
        output,
        Output::Info {
            pc: 0x0001_10f0,
            cpu_status: 0x05,
            cpu_state: "stopped",
            flash_info: Some(FlashInfo {
                jedec_id: "ef4018".to_owned(),
                xtal_mhz: 16,
                build_version: 0x0dd5,
                magic_ok: true,
            }),
        }
    );
}

#[test]
fn info_halt_stalls_core_and_clears_watchdog_before_flash_reads() {
    let mut mock = crate::transport::MockTransport::new([
        Response::Ack,
        pc_bytes(),
        data([0x00]),
        // stall_core: status sample, stall, watchdog read, write-back
        data([0x08]),
        Response::Ack,
        data([0xb5]),
        Response::Ack,
        jedec_id(),
        header(0x0000_0dd3),
    ]);

    let output = info(&mut mock, true).expect("info --halt succeeds");

    assert_eq!(
        output,
        Output::Info {
            pc: 0x0001_10f0,
            // The status byte is sampled before `--halt` stalls the core.
            cpu_status: 0x00,
            cpu_state: "running",
            flash_info: Some(FlashInfo {
                jedec_id: "ef4018".to_owned(),
                xtal_mhz: 12,
                build_version: 0x0dd3,
                magic_ok: true,
            }),
        }
    );
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            read_mem_req(PC_ADDR, 4),
            read_mem_req(CPU_STATUS_ADDR, 1),
            read_mem_req(CPU_STATUS_ADDR, 1),
            Request::CpuCtrl {
                mode: CpuMode::Stall,
            },
            read_mem_req(WATCHDOG_CTRL_ADDR, 1),
            write_mem_req(WATCHDOG_CTRL_ADDR, &[0x35]),
            Request::FlashJedecId,
            read_flash_req(FLASH_HEADER_ADDR, FLASH_HEADER_LEN),
        ]
    );
}

#[test]
fn halt_stalls_core_and_clears_watchdog_bit_seven() {
    let mut mock = crate::transport::MockTransport::new([
        Response::Ack,
        data([0x08]),
        Response::Ack,
        data([0x80]),
        Response::Ack,
    ]);

    let output = halt(&mut mock).expect("halt succeeds");

    assert_eq!(output, Output::Halt);
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            read_mem_req(CPU_STATUS_ADDR, 1),
            Request::CpuCtrl {
                mode: CpuMode::Stall,
            },
            read_mem_req(WATCHDOG_CTRL_ADDR, 1),
            write_mem_req(WATCHDOG_CTRL_ADDR, &[0x00]),
        ]
    );
}

#[test]
fn halt_does_not_write_stall_to_an_already_stopped_core() {
    // 0x05 -> 0x06 releases the reset hold on this silicon: the core starts
    // running again, which makes any following MSPI read race XIP. `halt` on a
    // stopped core must only hold the watchdog.
    for status in [0x05_u8, 0x06] {
        let mut mock = crate::transport::MockTransport::new([
            Response::Ack,
            data([status]),
            data([0x80]),
            Response::Ack,
        ]);

        halt(&mut mock).expect("halt on a held core succeeds");

        assert_eq!(
            mock.requests,
            [
                sws_activate_req(),
                read_mem_req(CPU_STATUS_ADDR, 1),
                read_mem_req(WATCHDOG_CTRL_ADDR, 1),
                write_mem_req(WATCHDOG_CTRL_ADDR, &[0x00]),
            ],
            "status 0x{status:02x}"
        );
    }
}

#[test]
fn info_halt_on_a_stopped_core_keeps_it_stopped_before_the_flash_read() {
    let mut mock = crate::transport::MockTransport::new([
        Response::Ack,
        pc_bytes(),
        data([0x05]),
        data([0x05]),
        data([0x80]),
        Response::Ack,
        jedec_id(),
        header(0x0000_0dd3),
    ]);

    let output = info(&mut mock, true).expect("info --halt on a stopped core succeeds");

    assert_eq!(
        output,
        Output::Info {
            pc: 0x0001_10f0,
            cpu_status: 0x05,
            cpu_state: "stopped",
            flash_info: Some(FlashInfo {
                jedec_id: "ef4018".to_owned(),
                xtal_mhz: 12,
                build_version: 0x0dd3,
                magic_ok: true,
            }),
        }
    );
    assert!(
        !mock.requests.iter().any(|request| matches!(
            request,
            Request::CpuCtrl {
                mode: CpuMode::Stall
            }
        )),
        "a 0x05 core must not be switched to 0x06: {:?}",
        mock.requests
    );
}

#[test]
fn reset_soft_preserves_a_panic_sentinel() {
    let mut mock = crate::transport::MockTransport::new([
        Response::Ack,
        Response::Byte(0xe1),
        Response::Ack,
        Response::Ack,
    ]);

    reset(&mut mock, false, true).expect("reset --soft after a panic succeeds");

    assert!(
        !mock.requests.iter().any(|request| matches!(
            request,
            Request::WriteAnalog {
                addr: RESET_SENTINEL_ADDR,
                ..
            }
        )),
        "a pending panic must survive the reset: {:?}",
        mock.requests
    );
}

#[test]
fn reset_default_holds_for_two_ms_without_stopping() {
    let mut mock = crate::transport::MockTransport::new([Response::Ack]);

    let output = reset(&mut mock, false, false).expect("reset succeeds");

    assert_eq!(
        output,
        Output::Reset {
            stop: false,
            soft: false,
        }
    );
    assert_eq!(
        mock.requests,
        [Request::Reset {
            hold_ms: 2,
            stop_cpu: false,
        }],
        "a GP14 reset clears the analog sentinel itself: no SWire traffic"
    );
}

#[test]
fn reset_stop_leaves_the_core_stopped() {
    let mut mock = crate::transport::MockTransport::new([Response::Ack]);

    reset(&mut mock, true, false).expect("reset --stop succeeds");

    assert_eq!(
        mock.requests,
        [Request::Reset {
            hold_ms: 2,
            stop_cpu: true,
        }]
    );
}

#[test]
fn reset_soft_activates_swire_then_soft_reboots() {
    // `mark_clean_reboot` activates SWire itself, so the soft reboot must not
    // send a second `SwsActivate`.
    let mut mock = crate::transport::MockTransport::new([
        Response::Ack,
        Response::Byte(0xa5),
        Response::Ack,
        Response::Ack,
    ]);

    let output = reset(&mut mock, false, true).expect("reset --soft succeeds");

    assert_eq!(
        output,
        Output::Reset {
            stop: false,
            soft: true,
        }
    );
    assert_eq!(
        mock.requests,
        [
            sws_activate_req(),
            Request::ReadAnalog {
                addr: RESET_SENTINEL_ADDR,
            },
            Request::WriteAnalog {
                addr: RESET_SENTINEL_ADDR,
                val: SENTINEL_CLEAN_REBOOT,
            },
            Request::CpuCtrl {
                mode: CpuMode::SoftReboot,
            },
        ]
    );
}

#[test]
fn analog_read_returns_the_byte() {
    let mut mock = crate::transport::MockTransport::new([Response::Ack, Response::Byte(0x42)]);

    let output = analog_read(&mut mock, "0x0c").expect("analog-read succeeds");

    assert_eq!(
        output,
        Output::AnalogRead {
            addr: 0x0c,
            value: 0x42,
        }
    );
    assert_eq!(
        mock.requests,
        [sws_activate_req(), Request::ReadAnalog { addr: 0x0c }]
    );
}

#[test]
fn analog_write_accepts_hex_and_decimal() {
    let mut hex = crate::transport::MockTransport::new([Response::Ack, Response::Ack]);
    let output = analog_write(&mut hex, "0x0c", "0xff").expect("analog-write succeeds");
    assert_eq!(
        output,
        Output::AnalogWrite {
            addr: 0x0c,
            value: 0xff,
        }
    );
    assert_eq!(
        hex.requests,
        [
            sws_activate_req(),
            Request::WriteAnalog {
                addr: 0x0c,
                val: 0xff,
            },
        ]
    );

    let mut decimal = crate::transport::MockTransport::new([Response::Ack, Response::Ack]);
    analog_write(&mut decimal, "12", "255").expect("decimal arguments parse");
    assert_eq!(
        decimal.requests,
        [
            sws_activate_req(),
            Request::WriteAnalog { addr: 12, val: 255 },
        ]
    );
}

#[test]
fn cpu_state_mapping_is_pinned() {
    assert_eq!(cpu_state(0x00), "running");
    assert_eq!(cpu_state(0x04), "running");
    assert_eq!(cpu_state(0x08), "running");
    assert_eq!(cpu_state(0x05), "stopped");
    assert_eq!(cpu_state(0x06), "stalled");
    assert_eq!(cpu_state(0x07), "unknown");
}

#[test]
fn short_and_long_reads_are_device_errors() {
    let mut short = crate::transport::MockTransport::new([Response::Ack, data([0xf0, 0x10, 0x01])]);
    let err = info(&mut short, false).expect_err("short PC reply must fail");
    assert_eq!(
        err,
        CliError::Device("short read: got 3, want 4".to_owned())
    );

    let mut long =
        crate::transport::MockTransport::new([Response::Ack, data([0xf0, 0x10, 0x01, 0x00, 0x00])]);
    let err = info(&mut long, false).expect_err("long PC reply must fail");
    assert_eq!(err, CliError::Device("long read: got 5, want 4".to_owned()));
}

#[test]
fn unexpected_response_variants_are_device_errors() {
    let mut mock = crate::transport::MockTransport::new([Response::Ack, Response::Byte(0x00)]);
    let err = info(&mut mock, false).expect_err("Byte is not a Data reply");
    assert!(
        matches!(&err, CliError::Device(message) if message.contains("expected Data")),
        "{err:?}"
    );

    let mut mock =
        crate::transport::MockTransport::new([Response::Ack, data([0x08]), Response::Byte(0x00)]);
    let err = halt(&mut mock).expect_err("Byte is not an Ack reply");
    assert!(
        matches!(&err, CliError::Device(message) if message.contains("expected Ack")),
        "{err:?}"
    );
}

#[test]
fn invalid_analog_arguments_are_rejected_before_any_request() {
    let mut mock = crate::transport::MockTransport::new([]);
    let err = analog_read(&mut mock, "0x1ff").expect_err("out-of-range address fails");
    assert!(matches!(err, CliError::InvalidAddress { .. }), "{err:?}");

    let mut mock = crate::transport::MockTransport::new([]);
    let err = analog_write(&mut mock, "0x0c", "zz").expect_err("bad value fails");
    assert!(matches!(err, CliError::Device(_)), "{err:?}");

    assert!(mock.requests.is_empty());
}

#[test]
fn run_dispatches_to_the_control_handlers() {
    let mut ctx = test_ctx([Response::Ack]);
    let mut out = Vec::new();

    let output = run(
        Command::Reset {
            stop: false,
            soft: false,
        },
        &mut ctx,
        &mut out,
    )
    .expect("reset runs")
    .expect("reset returns an output");

    assert_eq!(
        output,
        Output::Reset {
            stop: false,
            soft: false,
        }
    );
}
