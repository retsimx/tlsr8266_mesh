//! TLSR8266 SWire bus: two PIO state machines (TX at 50 MHz, self-syncing RX)
//! plus the `GP14` reset line. Ported from `utilities/picow_swire/remote.py`.

use embassy_rp::Peri;
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{
    Config, Direction, Instance, InterruptHandler, Pio, PioPin, ShiftDirection, StateMachine,
};
use embassy_time::{Duration, Instant, Timer};
use fixed::FixedU32;
use fixed::types::extra::U8;
use pio::pio_asm;
use probe_proto::{CpuMode, ProtoError, breakpoint_reg_and_val};

/// Every FIFO poll gives the state machine this long before giving up.
const FIFO_TIMEOUT: Duration = Duration::from_millis(20);

/// Analog bus operations poll their done bit on the same 20 ms budget.
const ANALOG_TIMEOUT: Duration = FIFO_TIMEOUT;

/// Attempts for the `0x00b2` write/read-back loops (`reset`, `activate`).
const SWS_RETRIES: u32 = 10;

/// `2.5` in `U8` fixed point: 2.5 * 256 = 640. 125 MHz / 2.5 = 50 MHz.
const TX_CLKDIV: FixedU32<U8> = FixedU32::from_bits(640);
/// `1.0` in `U8` fixed point: 1 * 256.
const RX_CLKDIV: FixedU32<U8> = FixedU32::from_bits(256);

/// Word layout expected by `tx_pio`: bit 9 = "long word" marker (clear means the
/// short-low END/trigger pulse), bit 8 = cmd, bits 7..0 = byte, MSB first.
fn packed_word(cmd: bool, byte: u8) -> u32 {
    u32::from(byte) | (1 << 9) | (u32::from(cmd) << 8)
}

/// One SWire bus: PIO0 SM0 transmits, PIO1 SM0 receives, `GP14` resets the target.
pub struct SwireBus<'d> {
    tx: StateMachine<'d, PIO0, 0>,
    rx: StateMachine<'d, PIO1, 0>,
    tx_origin: u8,
    rx_origin: u8,
    reset: Output<'d>,
}

impl<'d> SwireBus<'d> {
    /// Create the bus, load both PIO programs, and idle the reset and TX lines HIGH.
    pub fn new(
        pio0: Peri<'d, PIO0>,
        pio1: Peri<'d, PIO1>,
        irqs: impl Binding<<PIO0 as Instance>::Interrupt, InterruptHandler<PIO0>>
        + Binding<<PIO1 as Instance>::Interrupt, InterruptHandler<PIO1>>,
        pin_reset: Peri<'d, impl Pin>,
        pin_rx: Peri<'d, impl PioPin + 'd>,
        pin_tx: Peri<'d, impl PioPin + 'd>,
    ) -> Self {
        let Pio {
            mut common,
            sm0: mut tx,
            ..
        } = Pio::new(pio0, irqs);
        let tx_pin = common.make_pio_pin(pin_tx);

        let tx_prog = pio_asm!(
            r#"
            pull block
            out x, 22
            out x, 1
            jmp !x, finish
            set y, 8
            nextbit:
            out x, 1
            jmp !x, zero
            set pins, 0 [19]
            set pins, 1 [1]
            jmp y--, nextbit
            jmp finish
            zero:
            set pins, 0 [4]
            set pins, 1 [16]
            jmp y--, nextbit
            finish:
            set pins, 0 [4]
            set pins, 1
            "#
        );
        let tx_prg = common.load_program(&tx_prog.program);
        let tx_origin = tx_prg.origin;

        let mut tx_cfg = Config::default();
        tx_cfg.use_program(&tx_prg, &[]);
        tx_cfg.set_set_pins(&[&tx_pin]);
        tx_cfg.shift_out.direction = ShiftDirection::Left;
        tx_cfg.shift_out.auto_fill = false;
        tx_cfg.clock_divider = TX_CLKDIV;
        tx.set_config(&tx_cfg);
        tx.set_pin_dirs(Direction::Out, &[&tx_pin]);
        // `pio` has no `set_init`; drive GP17 HIGH so SWire never sees a float
        // between the pin becoming an output and the first `pull block`.
        tx.set_pins(Level::High, &[&tx_pin]);
        tx.set_enable(true);

        let Pio {
            mut common,
            sm0: mut rx,
            ..
        } = Pio::new(pio1, irqs);
        let rx_pin = common.make_pio_pin(pin_rx);

        let rx_prog = pio_asm!(
            r#"
            .wrap_target
            ; Load the 8-bit counter before waiting for the trigger so `wait 0`
            ; (bit 7 falling edge) immediately follows `wait 1` (trigger HIGH).
            mov osr, !null
            out null, 24
            wait 0 pin 0
            wait 1 pin 0
            wait 0 pin 0
            bit_loop:
            mov x, !null
            count_low:
            jmp pin, is_high
            jmp x--, count_low
            is_high:
            mov y, !x
            count_high:
            jmp pin, still_high
            set x, 1
            in x, 1
            jmp next_bit
            still_high:
            jmp y--, count_high
            in null, 1
            wait 0 pin 0
            next_bit:
            out null, 1
            jmp !osre, bit_loop
            wait 1 pin 0
            ; Push only after `END` has returned HIGH (`wait 1 pin 0`) so the
            ; next `read_byte()` cannot race ahead of the target's END pulse.
            push noblock
            .wrap
            "#
        );
        let rx_prg = common.load_program(&rx_prog.program);
        let rx_origin = rx_prg.wrap.target;

        let mut rx_cfg = Config::default();
        rx_cfg.use_program(&rx_prg, &[]);
        rx_cfg.set_in_pins(&[&rx_pin]);
        rx_cfg.set_jmp_pin(&rx_pin);
        rx_cfg.shift_in.direction = ShiftDirection::Left;
        rx_cfg.shift_in.auto_fill = false;
        rx_cfg.shift_in.threshold = 8;
        rx_cfg.clock_divider = RX_CLKDIV;
        rx.set_config(&rx_cfg);
        rx.set_pin_dirs(Direction::In, &[&rx_pin]);

        Self {
            tx,
            rx,
            tx_origin,
            rx_origin,
            reset: Output::new(pin_reset, Level::High),
        }
    }

    /// Best-effort bounded push; never sends END itself (no recursion).
    fn push(&mut self, word: u32) -> Result<(), ProtoError> {
        let deadline = Instant::now() + FIFO_TIMEOUT;
        while !self.tx.tx().try_push(word) {
            if Instant::now() >= deadline {
                return Err(ProtoError::SwsTimeout);
            }
        }
        Ok(())
    }

    /// Push one 10-bit SWire word (`cmd` bit + byte) into the TX FIFO.
    pub fn send_word(&mut self, cmd: bool, byte: u8) -> Result<(), ProtoError> {
        if self.push(packed_word(cmd, byte)).is_err() {
            self.force_end();
            return Err(ProtoError::SwsTimeout);
        }
        Ok(())
    }

    /// Send SWire END (`cmd = 1`, `0xff`).
    pub fn send_end(&mut self) -> Result<(), ProtoError> {
        self.send_word(true, 0xff)
    }

    /// Unblock the target after a failed transfer; errors are ignored.
    fn force_end(&mut self) {
        let _ = self.push(packed_word(true, 0xff));
    }

    /// Clock one byte out of the target. The RX waveform is sampled by the RX
    /// program, so the result is returned as-is (no `!value` inversion).
    pub fn read_byte(&mut self) -> Result<u8, ProtoError> {
        // Wait for every queued command word to finish shifting (`tx_pio` sits
        // at `tx_origin` == `pull block` only after its trailing `set pins, 1`
        // wraps). Checking `empty()` before `get_addr()` avoids sticky-flag
        // edge cases and guarantees GP15 is idle HIGH before RX arms.
        let deadline = Instant::now() + FIFO_TIMEOUT;
        while !(self.tx.tx().empty() && self.tx.get_addr() == self.tx_origin) {
            if Instant::now() >= deadline {
                self.force_end();
                return Err(ProtoError::SwsTimeout);
            }
        }

        // `embassy_rp::StateMachine::restart()` clears shift counters/ISR/OSR
        // but does NOT reset the PC (unlike MicroPython's `sm.restart()`), so
        // jump explicitly to `.wrap_target` before arming RX.
        self.rx.set_enable(false);
        self.rx.clear_fifos();
        self.rx.restart();
        // SAFETY: `rx_origin` is the loaded `.wrap_target` address in 0..32
        // and the RX state machine is disabled above.
        unsafe { self.rx.exec_jmp(self.rx_origin) };
        self.rx.set_enable(true);

        // A zero word (bit 9 clear) is the short-low pulse the target answers.
        if self.push(0).is_err() {
            self.rx.set_enable(false);
            self.force_end();
            return Err(ProtoError::SwsTimeout);
        }

        let deadline = Instant::now() + FIFO_TIMEOUT;
        let value = loop {
            if let Some(v) = self.rx.rx().try_pull() {
                break Some(v);
            }
            if Instant::now() >= deadline {
                break None;
            }
        };
        self.rx.set_enable(false);

        match value {
            Some(v) => Ok((v & 0xff) as u8),
            None => {
                self.force_end();
                Err(ProtoError::SwsTimeout)
            }
        }
    }

    /// One `0x5a` write transaction: `cmd, addr_hi, addr_lo, 0x00`, payload, END.
    pub fn write_mem(&mut self, addr: u16, data: &[u8]) -> Result<(), ProtoError> {
        self.send_word(true, 0x5a)?;
        self.send_word(false, (addr >> 8) as u8)?;
        self.send_word(false, addr as u8)?;
        self.send_word(false, 0x00)?;
        for &byte in data {
            self.send_word(false, byte)?;
        }
        self.send_end()
    }

    /// One `0x5a` read transaction: `cmd, addr_hi, addr_lo, 0x80`, one
    /// `read_byte` per buffer byte, END. Returns the byte count consumed.
    pub fn read_mem(&mut self, addr: u16, buf: &mut [u8]) -> Result<usize, ProtoError> {
        self.send_word(true, 0x5a)?;
        self.send_word(false, (addr >> 8) as u8)?;
        self.send_word(false, addr as u8)?;
        self.send_word(false, 0x80)?;
        for slot in buf.iter_mut() {
            *slot = self.read_byte()?;
        }
        self.send_end()?;
        Ok(buf.len())
    }

    fn analog_xfer(&mut self, addr: u8, val: Option<u8>, op: u8) -> Result<(), ProtoError> {
        // Set-up as one fallible unit so a failure in any step still reaches the
        // `0x00ba = 0x00` clear below.
        let setup = self
            .write_mem(0x00b8, &[addr])
            .and_then(|()| match val {
                Some(val) => self.write_mem(0x00b9, &[val]),
                None => Ok(()),
            })
            .and_then(|()| self.write_mem(0x00ba, &[op]));

        let polled = match setup {
            Ok(()) => {
                let deadline = Instant::now() + ANALOG_TIMEOUT;
                let mut status = [0u8; 1];
                loop {
                    match self.read_mem(0x00ba, &mut status) {
                        Ok(_) if status[0] & 1 == 0 => break Ok(()),
                        Ok(_) if Instant::now() < deadline => {}
                        Ok(_) => break Err(ProtoError::SwsTimeout),
                        Err(err) => break Err(err),
                    }
                }
            }
            Err(err) => Err(err),
        };
        // A leftover `0x00ba` op keeps the analog bus busy, so every failed
        // access must still clear it before the error is returned.
        let cleared = self.write_mem(0x00ba, &[0x00]);
        polled?;
        cleared
    }

    /// Read one indirect analog register through `0x00b8..0x00b9`.
    pub fn read_analog(&mut self, addr: u8) -> Result<u8, ProtoError> {
        self.analog_xfer(addr, None, 0x50)?;
        let mut val = [0u8; 1];
        self.read_mem(0x00b9, &mut val)?;
        Ok(val[0])
    }

    /// Write one indirect analog register through `0x00b8..0x00b9`.
    pub fn write_analog(&mut self, addr: u8, val: u8) -> Result<(), ProtoError> {
        self.analog_xfer(addr, Some(val), 0x60)
    }

    /// Apply a CPU control action via `0x0602`/`0x0622`/`0x006f`.
    pub fn cpu_ctrl(&mut self, mode: CpuMode) -> Result<(), ProtoError> {
        match mode {
            CpuMode::Stop => self.write_mem(0x0602, &[0x05]),
            CpuMode::Stall => {
                self.write_mem(0x0602, &[0x06])?;
                let mut ctrl = [0u8; 1];
                self.read_mem(0x0622, &mut ctrl)?;
                // Timer 2's watchdog keeps ticking while the CPU is stalled and
                // resets the chip after 2 s; clearing `TMR_WD_EN` holds it.
                self.write_mem(0x0622, &[ctrl[0] & !0x80])
            }
            CpuMode::Run => self.write_mem(0x0602, &[0x08]),
            CpuMode::SoftReboot => self.write_mem(0x006f, &[0x20]),
        }
    }

    /// Read the 128-byte debug register block at `0x0680..=0x06ff`.
    pub fn read_regs(&mut self, buf: &mut [u8; 128]) -> Result<(), ProtoError> {
        self.read_mem(0x0680, buf).map(|_| ())
    }

    /// Program the 4-byte hardware breakpoint for `addr` in `slot`.
    pub fn set_breakpoint(&mut self, slot: u8, addr: u32) -> Result<(), ProtoError> {
        let (reg, val) = breakpoint_reg_and_val(slot, addr)?;
        self.write_mem(reg, &val.to_le_bytes())
    }

    /// Reset the target and, when `stop_cpu`, leave the core halted.
    pub async fn reset(&mut self, hold_ms: u16, stop_cpu: bool) -> Result<(), ProtoError> {
        self.reset_pulse(hold_ms).await;
        if !stop_cpu {
            return Ok(());
        }
        for _ in 0..SWS_RETRIES {
            self.write_mem(0x00b2, &[3])?;
            self.write_mem(0x0602, &[0x05])?;
            let mut div = [0u8; 1];
            let read = self.read_mem(0x00b2, &mut div);
            if matches!(read, Ok(1)) && div[0] == 3 {
                return Ok(());
            }
        }
        Err(ProtoError::SwsTimeout)
    }

    /// Set the SWire clock divider without touching `GP14`: this runs against a
    /// live target, and pulsing the reset line here would destroy its state.
    /// Returns `(rev, chip_id)` from `0x007d..=0x007f`.
    pub fn activate(&mut self, div: u8) -> Result<(u8, u16), ProtoError> {
        for _ in 0..SWS_RETRIES {
            self.write_mem(0x00b2, &[div])?;
            let mut readback = [0u8; 1];
            let read = self.read_mem(0x00b2, &mut readback);
            if matches!(read, Ok(1)) && readback[0] == div {
                let mut id = [0u8; 3];
                self.read_mem(0x007d, &mut id)?;
                return Ok((id[0], u16::from_le_bytes([id[1], id[2]])));
            }
        }
        Err(ProtoError::SwsTimeout)
    }

    /// Pulse `GP14` LOW for `hold_ms`, release it, then let the target settle.
    pub async fn reset_pulse(&mut self, hold_ms: u16) {
        // Words queued by an aborted frame would clock into the target the
        // moment it leaves reset. Flush both FIFOs first; `reset` goes through
        // here, so this covers every reset path.
        self.tx.clear_fifos();
        self.rx.clear_fifos();

        self.reset.set_low();
        Timer::after_millis(u64::from(hold_ms)).await;
        self.reset.set_high();
        Timer::after_millis(2).await;
    }
}
