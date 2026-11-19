use super::*;
use crate::commands::run;
use crate::commands::test_support::*;
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
fn info_halt_stalls_core_and_clears_watchdog_before_flash_reads() {
    let mut mock = crate::transport::MockTransport::new([
        Response::Ack,
        pc_bytes(),
        data([0x00]),
        // halt_core: stall, read 0x0622, write it back with bit 7 cleared
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
            Request::CpuCtrl {
                mode: CpuMode::Stall,
            },
            read_mem_req(WATCHDOG_CTRL_ADDR, 1),
            write_mem_req(WATCHDOG_CTRL_ADDR, &[0x00]),
        ]
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
        }]
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
    let mut mock = crate::transport::MockTransport::new([Response::Ack, Response::Ack]);

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

    let mut mock = crate::transport::MockTransport::new([Response::JedecId([1, 2, 3])]);
    let err = halt(&mut mock).expect_err("JedecId is not an Ack reply");
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

    let output = run(
        Command::Reset {
            stop: false,
            soft: false,
        },
        &mut ctx,
    )
    .expect("reset runs");

    assert_eq!(
        output,
        Output::Reset {
            stop: false,
            soft: false,
        }
    );
}
