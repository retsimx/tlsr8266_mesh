#![no_std]
#![no_main]

mod flash;
mod swire;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1, USB};
use embassy_rp::pio::InterruptHandler;
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcAcmState};
use embassy_usb::driver::EndpointError;
use probe_proto::{
    Direction, MAX_DATA_PAYLOAD, MAX_WIRE_FRAME, Message, ProtoError, Request, Response,
    decode_frame, encode_frame,
};
use static_cell::StaticCell;

use crate::swire::SwireBus;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

type UsbDriver = embassy_rp::usb::Driver<'static, USB>;
type Cdc = CdcAcmClass<'static, UsbDriver>;

static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUFFER: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<CdcAcmState<'static>> = StaticCell::new();

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, UsbDriver>) {
    usb.run().await
}

#[embassy_executor::main(
    executor = "embassy_rp::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut bus = SwireBus::new(p.PIO0, p.PIO1, Irqs, p.PIN_14, p.PIN_15, p.PIN_17);

    // PID `0x000a` is a distinct non-MicroPython PID so `probe-tlsr`'s `0x2e8a`
    // VID auto-detect finds the probe without matching MicroPython's `0x0005`.
    let mut config = embassy_usb::Config::new(0x2e8a, 0x000a);
    config.manufacturer = Some("tlsr8266_mesh");
    config.product = Some("probe-pico");
    config.serial_number = Some("PROBE-PICO-1");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        UsbDriver::new(p.USB, Irqs),
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [],
        CONTROL_BUFFER.init([0; 64]),
    );
    let mut class = CdcAcmClass::new(&mut builder, CDC_STATE.init(CdcAcmState::new()), 64);
    match usb_task(builder.build()) {
        Ok(token) => spawner.spawn(token),
        // Without the USB task nothing can be served; halt instead of running
        // a device the host can never talk to.
        Err(_) => loop {
            core::hint::spin_loop();
        },
    }

    'session: loop {
        // A disconnect drops any partial frame with the connection.
        // Frames accumulate without their delimiter; `MAX_WIRE_FRAME` bounds the
        // longest valid COBS frame plus a byte, so a valid frame always fits.
        let mut acc = [0u8; MAX_WIRE_FRAME];
        let mut acc_len = 0usize;
        let mut resync = false;

        class.wait_connection().await;

        loop {
            let mut chunk = [0u8; 64];
            let n = match class.read_packet(&mut chunk).await {
                Ok(n) => n,
                Err(_) => continue 'session,
            };

            for &byte in &chunk[..n] {
                if resync {
                    // Discard everything up to and including the next delimiter.
                    if byte == 0 {
                        resync = false;
                    }
                    continue;
                }

                if byte == 0 {
                    if acc_len > 0 {
                        // A corrupt frame has no trustworthy `seq`, so it cannot
                        // be answered; the host times out and retries.
                        if let Ok((seq, Message::Request(req))) =
                            decode_frame(Direction::Request, &acc[..acc_len])
                        {
                            let response = execute(&mut bus, req).await;
                            if send_response(&mut class, seq, response).await.is_err() {
                                continue 'session;
                            }
                        }
                    }
                    acc_len = 0;
                } else if acc_len < acc.len() {
                    acc[acc_len] = byte;
                    acc_len += 1;
                } else {
                    // Overflow without a delimiter: the frame is unusable, so
                    // drop bytes until the next `0x00` resynchronises framing.
                    resync = true;
                    acc_len = 0;
                }
            }
        }
    }
}

async fn send_response(class: &mut Cdc, seq: u8, response: Response) -> Result<(), EndpointError> {
    let mut out = [0u8; MAX_WIRE_FRAME];
    let len = match encode_frame(seq, &Message::Response(response), &mut out) {
        Ok(len) => len,
        // Every `Response` this firmware builds fits `MAX_WIRE_FRAME`; a failed
        // encode is a bug, and a half-written frame is worse than silence.
        Err(_) => return Ok(()),
    };

    for chunk in out[..len].chunks(64) {
        class.write_packet(chunk).await?;
    }
    // Bulk transfers end with a short packet; when the frame length is a
    // multiple of 64 that has to be an explicit zero-length packet.
    if len % 64 == 0 {
        class.write_packet(&[]).await?;
    }
    Ok(())
}

/// `Data` from a buffer that is always within `MAX_DATA_PAYLOAD`.
fn data_response(buf: &[u8]) -> Response {
    match heapless::Vec::from_slice(buf) {
        Ok(data) => Response::Data(data),
        // Unreachable: every caller passes at most `MAX_DATA_PAYLOAD` bytes. An
        // impossible truncation must never look like an empty successful read.
        Err(_) => Response::Error(ProtoError::InvalidArg),
    }
}

async fn execute(bus: &mut SwireBus<'_>, req: Request) -> Response {
    match req {
        Request::Ping => Response::Pong {
            fw_ver: 1,
            proto_ver: probe_proto::PROTOCOL_VERSION,
        },
        Request::Reset { hold_ms, stop_cpu } => match bus.reset(hold_ms, stop_cpu).await {
            Ok(()) => Response::Ack,
            Err(err) => Response::Error(err),
        },
        Request::SwsActivate { div } => match bus.activate(div) {
            Ok((rev, chip_id)) => Response::ChipId { rev, chip_id },
            Err(err) => Response::Error(err),
        },
        Request::ReadMem { addr, len } => {
            let mut buf = [0u8; MAX_DATA_PAYLOAD];
            match bus.read_mem(addr, &mut buf[..usize::from(len)]) {
                Ok(n) => data_response(&buf[..n]),
                Err(err) => Response::Error(err),
            }
        }
        Request::WriteMem { addr, data } => match bus.write_mem(addr, &data) {
            Ok(()) => Response::Ack,
            Err(err) => Response::Error(err),
        },
        Request::ReadAnalog { addr } => match bus.read_analog(addr) {
            Ok(val) => Response::Byte(val),
            Err(err) => Response::Error(err),
        },
        Request::WriteAnalog { addr, val } => match bus.write_analog(addr, val) {
            Ok(()) => Response::Ack,
            Err(err) => Response::Error(err),
        },
        Request::FlashJedecId => match bus.flash_jedec_id() {
            Ok(id) => Response::JedecId(id),
            Err(err) => Response::Error(err),
        },
        Request::FlashRead { addr, len } => {
            let mut buf = [0u8; MAX_DATA_PAYLOAD];
            match bus.flash_read(addr, &mut buf[..usize::from(len)]) {
                Ok(n) => data_response(&buf[..n]),
                Err(err) => Response::Error(err),
            }
        }
        Request::FlashEraseSector { addr } => match bus.flash_erase_sector(addr) {
            Ok(()) => Response::Ack,
            Err(err) => Response::Error(err),
        },
        Request::FlashWritePage { addr, data } => match bus.flash_write_page(addr, &data) {
            Ok(()) => Response::Ack,
            Err(err) => Response::Error(err),
        },
        Request::CpuCtrl { mode } => match bus.cpu_ctrl(mode) {
            Ok(()) => Response::Ack,
            Err(err) => Response::Error(err),
        },
        Request::ReadRegs => {
            let mut buf = [0u8; 128];
            match bus.read_regs(&mut buf) {
                Ok(()) => data_response(&buf),
                Err(err) => Response::Error(err),
            }
        }
        Request::SetBreakpoint { slot, addr } => match bus.set_breakpoint(slot, addr) {
            Ok(()) => Response::Ack,
            Err(err) => Response::Error(err),
        },
        Request::RttPoll { ctrl_addr } => {
            let mut buf = [0u8; MAX_DATA_PAYLOAD];
            match bus.rtt_poll(ctrl_addr, &mut buf) {
                Ok(n) => data_response(&buf[..n]),
                Err(err) => Response::Error(err),
            }
        }
    }
}
