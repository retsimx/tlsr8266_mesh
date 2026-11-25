use core::mem::size_of;
use core::ptr::addr_of;

use crate::mcu::clock::{clock_time, clock_time_exceed, CLOCK_SYS_CLOCK_HZ};
use crate::mcu::gpio::{gpio_set_func, gpio_set_input_en, AS_UART, GPIO_PIN_TYPE};
use crate::mcu::watchdog::wd_clear;
use crate::BIT;
use mesh_protocol::uart::{UartData, UART_DATA_LEN};
use tlsr8266_pac::{
    read_reg_dma_chn_en, read_reg_dma_chn_irq_msk, read_reg_dma_rx_rdy0, read_reg_irq_mask,
    read_reg_uart_status, write_reg_dma0_addr, write_reg_dma0_ctrl, write_reg_dma0_mode,
    write_reg_dma1_addr, write_reg_dma1_mode, write_reg_dma_chn_en, write_reg_dma_chn_irq_msk,
    write_reg_dma_rx_rdy0, write_reg_dma_tx_rdy0, write_reg_irq_mask, write_reg_uart_clk_div,
    write_reg_uart_ctrl0, write_reg_uart_ctrl1, write_reg_uart_rx_timeout,
    write_reg_uart_rx_timeout_cnt, write_reg_uart_status, FLD_DMA, FLD_IRQ, FLD_UART_CLK_DIV,
    FLD_UART_CTRL0, FLD_UART_STATUS,
};

/// UART interrupt mask flags for controlling the UART interrupt sources.
///
/// These bit flags can be used individually or combined to enable/disable
/// specific UART interrupt sources in the interrupt controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartIrqMask {
    /// Enable RX interrupts - triggered when data is received
    Rx = BIT!(0),
    /// Enable TX interrupts - triggered when data transmission completes
    Tx = BIT!(1),
}

impl UartIrqMask {
    /// Combines multiple interrupt mask flags
    pub const fn bits(&self) -> u8 {
        *self as u8
    }

    /// Combines RX and TX mask flags
    pub const fn all() -> u8 {
        UartIrqMask::Rx as u8 | UartIrqMask::Tx as u8
    }
}

/// Hardware control modes for UART communication.
///
/// These modes define various hardware flow control and parity options
/// for the UART peripheral on the TLSR8266.
#[allow(dead_code)] // full hardware mode table from the datasheet; a subset is used here
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum HardwareControl {
    /// Enable CTS hardware flow control with odd parity checking
    CtsWithOddParity = 0x0e,
    /// Enable CTS hardware flow control with even parity checking
    CtsWithEvenParity = 0x06,
    /// Enable CTS hardware flow control only (no parity)
    CtsOnly = 0x02,
    /// Enable odd parity checking without hardware flow control
    OddParity = 0x0C,
    /// Enable even parity checking without hardware flow control
    EvenParity = 0x04,
    /// No hardware flow control or parity checking
    NoControl = 0x00,
}

impl HardwareControl {
    /// Gets the raw value for register configuration
    pub const fn value(&self) -> u8 {
        *self as u8
    }
}

/// Driver for UART communication on the TLSR8266.
///
/// This driver is a pure synchronous peripheral: it owns the DMA buffers and the
/// TX busy flag, and does nothing else. Serialisation, wake-ups and async
/// coordination belong to the manager that owns the driver
/// (`uart_manager::UART_HW`), so this type has no application-layer imports and
/// no async methods.
pub struct UartDriver {
    /// Internal buffer used for transmitting data.
    /// This buffer should not be accessed directly by users.
    tx_data_buf: UartData,

    /// Buffer used to receive incoming UART data via DMA.
    /// Users should copy data from this buffer to their own memory,
    /// rather than using it directly, as it may be overwritten by new data.
    pub rx_data_buf: UartData,

    /// Flag indicating whether a transmission is currently in progress.
    is_tx_busy: bool,
}

impl UartDriver {
    /// Creates a new instance of the UART driver with default settings.
    ///
    /// This initializes the driver with empty TX/RX buffers and sets
    /// the busy flag to false. Note that this doesn't configure the UART
    /// hardware - call `init()` to do that.
    ///
    /// # Returns
    ///
    /// * A new UartDriver instance with default settings
    pub const fn default_const() -> Self {
        Self {
            tx_data_buf: UartData {
                len: 0,
                data: [0; UART_DATA_LEN],
            },
            rx_data_buf: UartData {
                len: 0,
                data: [0; UART_DATA_LEN],
            },
            is_tx_busy: false,
        }
    }

    /// Checks if the UART transmitter is currently busy.
    ///
    /// # Returns
    ///
    /// * `true` if a transmission is in progress
    /// * `false` if the transmitter is idle
    pub fn is_tx_busy(&self) -> bool {
        self.is_tx_busy
    }

    /// Initializes the UART hardware with default settings.
    ///
    /// This function:
    /// 1. Configures GPIO pins for UART TX and RX
    /// 2. Initializes DMA buffers for transmission and reception
    /// 3. Sets up the UART with 115200 baud rate (with 32MHz system clock)
    ///
    /// The default configuration uses:
    /// - No hardware flow control
    /// - RX and TX interrupts enabled
    /// - 69 clock divider, 3 bit width
    ///
    /// # Notes
    ///
    /// * Call this function before using any other UART functions
    pub fn init(&mut self) {
        // 1. Configure GPIO pins for UART functionality
        gpio_set_func(GPIO_PIN_TYPE::GPIO_UTX as u32, AS_UART);
        gpio_set_func(GPIO_PIN_TYPE::GPIO_URX as u32, AS_UART);

        // 2. Enable input mode for RX pin to receive data
        gpio_set_input_en(GPIO_PIN_TYPE::GPIO_URX as u32, 1);

        // 3. Initialize DMA buffers for receiving data
        self.init_buffers();

        // 4. Initialize UART with default settings (115200 baud at 32MHz clock)
        // Parameters: clkdiv=69, bit width=3, RX IRQ=on, TX IRQ=on, no hardware control
        let _ = self.uart_init(69, 3, true, true, HardwareControl::NoControl);
    }

    /// Initializes the UART hardware with specified settings.
    ///
    /// This function configures the UART peripheral with custom parameters for
    /// clock division, bit width, and interrupt settings. It calculates the baud rate
    /// based on the system clock and provided parameters.
    ///
    /// # Parameters
    ///
    /// * `uart_clkdiv` - UART clock divider value (affects baud rate)
    /// * `bwpc` - Bit width parameter (must be ≥3)
    /// * `enable_rx_irq` - Enable RX interrupts if true
    /// * `enable_tx_irq` - Enable TX interrupts if true
    /// * `hw_control` - Hardware control mode for flow control and parity
    ///
    /// # Returns
    ///
    /// * `Ok(())` if initialization succeeded
    /// * `Err(())` if invalid parameters were provided (e.g., bwpc < 3)
    ///
    /// # Notes
    ///
    /// * Baud rate calculation: BaudRate = CLOCK_SYS_CLOCK_HZ/((uart_clkdiv+1)*(bwpc+1))
    /// * For 32MHz system clock:
    ///   - 115200 baud: clkdiv=19, bwpc=13
    ///   - 9600 baud: clkdiv=237, bwpc=13
    fn uart_init(
        &mut self,
        uart_clkdiv: u16,
        bwpc: u8,
        enable_rx_irq: bool,
        enable_tx_irq: bool,
        hw_control: HardwareControl,
    ) -> Result<(), ()> {
        // Validate bit width parameter - must be at least 3
        if bwpc < 3 {
            return Err(());
        }

        // Calculate actual baud rate based on system clock and parameters
        let baudrate = (CLOCK_SYS_CLOCK_HZ / (uart_clkdiv as u32 + 1)) / (bwpc as u32 + 1);

        // --- Configure UART Registers ---

        // Set UART clock divider (bits 0-14) and enable clock (bit 15)
        write_reg_uart_clk_div(uart_clkdiv | FLD_UART_CLK_DIV::CLK_EN.bits());

        // Set bit width (bits 0-4) and enable RX/TX DMA (bits 4-5)
        write_reg_uart_ctrl0(
            FLD_UART_CTRL0::RX_DMA_EN.bits() | FLD_UART_CTRL0::TX_DMA_EN.bits() | bwpc,
        );

        // Configure UART timing parameters based on baud rate
        if baudrate > 100000 {
            // For high baud rates (e.g., 115200)
            write_reg_uart_rx_timeout(0xff); // Maximum timeout
            write_reg_uart_rx_timeout_cnt(3); // Higher safety margin
        } else {
            // For low baud rates (e.g., 9600)
            // One byte includes 12 bits at most (start + 8 data + parity + stop)
            write_reg_uart_rx_timeout((bwpc + 1) * 12); // Timeout based on bit width
            write_reg_uart_rx_timeout_cnt(1); // Lower safety margin
        }

        // Set hardware control function (flow control and parity options)
        write_reg_uart_ctrl1(hw_control.value());

        // --- Configure DMA for UART ---

        // Set DMA channel modes:
        // - DMA0 mode 0x01 for receiving data
        // - DMA1 mode 0x00 for transmitting data
        write_reg_dma0_mode(0x01);
        write_reg_dma1_mode(0x00);

        // Clear any pending UART interrupts
        Self::get_and_clear_irq_source();

        // --- Configure Interrupt Handling ---

        // Setup RX interrupts if enabled
        if enable_rx_irq {
            // Enable DMA channel 0 (RX) interrupt
            write_reg_dma_chn_irq_msk(read_reg_dma_chn_irq_msk() | FLD_DMA::ETH_RX.bits() as u8);
            // Enable DMA interrupt in global interrupt mask
            write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::DMA_EN.bits());
            // Enable DMA channel 0
            write_reg_dma_chn_en(read_reg_dma_chn_en() | BIT!(0));
        }

        // Setup TX interrupts if enabled
        if enable_tx_irq {
            // Enable DMA channel 1 (TX) interrupt
            write_reg_dma_chn_irq_msk(read_reg_dma_chn_irq_msk() | FLD_DMA::ETH_TX.bits() as u8);
            // Enable DMA interrupt in global interrupt mask
            write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::DMA_EN.bits());
            // Enable DMA channel 1
            write_reg_dma_chn_en(read_reg_dma_chn_en() | BIT!(1));
        }

        Ok(())
    }

    /// Gets and clears the UART interrupt source flags.
    ///
    /// This function retrieves the current interrupt status from the DMA RX ready
    /// register, clears the interrupt flags, and returns the relevant UART
    /// interrupt bits.
    ///
    /// # Returns
    ///
    /// * A bitmask indicating which UART interrupts were triggered:
    ///   - Bit 0: RX interrupt flag
    ///   - Bit 1: TX interrupt flag
    ///
    /// # Notes
    ///
    /// * This function should be called in the interrupt handler to determine
    ///   the interrupt source and clear the pending interrupt flags.
    /// * The returned value can be compared with UartIrqMask values to determine
    ///   which specific interrupts were triggered.
    #[inline(always)]
    pub fn get_and_clear_irq_source() -> u8 {
        // Read the current DMA RX ready status
        let irq_status = read_reg_dma_rx_rdy0();

        // Clear the interrupt flags by writing back the same value
        write_reg_dma_rx_rdy0(irq_status);

        // Return only the UART-relevant bits (mask with ALL)
        irq_status & UartIrqMask::all()
    }

    /// Clears the UART TX busy flag to indicate transmission completion.
    ///
    /// This function should be called after a transmission is complete to allow
    /// new transmissions to begin.
    ///
    /// # Notes
    ///
    /// * Typically called from the TX completion interrupt handler
    /// * Might be called from a higher-level driver function that determines
    ///   when transmission is complete
    #[inline(always)]
    pub fn clear_tx_busy_flag(&mut self) {
        self.is_tx_busy = false;
    }

    /// Arms a DMA1 transfer immediately, without waiting for the previous one.
    ///
    /// This is the raw "start sending" primitive: it marks the transmitter busy,
    /// copies `msg` into the TX buffer, points DMA1 at that buffer and triggers
    /// the transfer. It does **not** wait, rate-limit or serialise anything.
    ///
    /// # Caller contract
    ///
    /// The wait/serialise/wake-up policy lives with the owner of the driver
    /// (`uart_manager`). The caller must keep a late TX-complete interrupt from
    /// interleaving with the arm below, typically by calling this inside a
    /// critical section or under the manager's TX lock.
    ///
    /// # Parameters
    ///
    /// * `msg` - The UART data packet to transmit
    ///
    /// # Notes
    ///
    /// * Returns after initiating the transfer, not when the transfer completes
    /// * The busy flag stays set until cleared by
    ///   [`Self::clear_tx_busy_flag`]
    pub fn start_tx_unchecked(&mut self, msg: &UartData) {
        self.is_tx_busy = true;

        // Copy message to TX buffer
        self.tx_data_buf = *msg;

        // Configure DMA1 with address of TX buffer
        write_reg_dma1_addr(addr_of!(self.tx_data_buf) as u16);

        // Trigger DMA transfer to start transmission
        write_reg_dma_tx_rdy0(FLD_DMA::ETH_TX.bits() as u8);
    }

    /// Sends data synchronously over UART using DMA.
    ///
    /// This function operates in a critical section to prevent interrupts
    /// during the setup process. It waits for any previous transmission
    /// to complete before starting a new one, with watchdog kicks during waiting.
    ///
    /// # Parameters
    ///
    /// * `msg` - The UART data packet to transmit
    ///
    /// # Returns
    ///
    /// # Notes
    ///
    /// * This function returns after initiating the transfer, not when the transfer completes
    /// * Uses critical_section to prevent interrupts during the setup process
    /// * Maximum wait time for previous transmission is 10ms
    /// * Watchdog is kicked during waiting to prevent resets
    pub fn send(&mut self, msg: &UartData) {
        critical_section::with(|_| {
            // Get current time for timeout calculation
            let t_timeout = clock_time();

            // Wait for any previous transmission to complete (with timeout)
            // Kick watchdog while waiting to prevent system reset
            while self.is_tx_busy() && !clock_time_exceed(t_timeout, 10 * 1000) {
                wd_clear();
            }

            // Arm the transfer while still inside the critical section.
            self.start_tx_unchecked(msg);
        })
    }

    /// Sends a frame from the panic path, never waiting on a previous transfer.
    ///
    /// This is the sink behind `uart::send_sync_panic`, so it runs on the
    /// `panic_fmt -> rust_begin_unwind` call graph: no indexing, no `unwrap`,
    /// no formatting, and no waiting for a TX-complete interrupt that a
    /// panicking system may never service.
    ///
    /// If a transfer is in flight, DMA1 is disabled, its pending TX flag is
    /// cleared and the channel is re-enabled before the new transfer is armed.
    /// [`Self::start_tx_unchecked`] only writes the address and the trigger, so
    /// the channel-enable bit it depends on has to be restored here; without
    /// that the panic frame would sit in the buffer and never shift out.
    ///
    /// The 4.5 ms shift-out wait covers a full 44-byte frame at 115200 baud
    /// (3.82 ms) with margin. On the device it spins on the system tick while
    /// kicking the watchdog; off-target it goes through `sleep_us`, which
    /// advances the fake clock instead of blocking, so host tests never hang.
    ///
    /// # Parameters
    ///
    /// * `msg` - The UART data packet to transmit
    pub fn send_panic(&mut self, msg: &UartData) {
        if self.is_tx_busy() {
            // Drop the in-flight transfer: disable DMA1, clear its TX flag,
            // then re-enable the channel so the arm below can shift out.
            write_reg_dma_chn_en(read_reg_dma_chn_en() & !(BIT!(1) as u8));
            write_reg_dma_rx_rdy0(UartIrqMask::Tx as u8);
            write_reg_dma_chn_en(read_reg_dma_chn_en() | (BIT!(1) as u8));
            // The abandoned transfer is no longer in flight. `start_tx_unchecked`
            // below sets the flag again for the frame it arms, so this store
            // only closes out the transfer that was just dropped.
            self.is_tx_busy = false;
        }

        self.start_tx_unchecked(msg);

        // Wait for the frame to shift out before the panic handler returns.
        #[cfg(target_os = "none")]
        {
            let t0 = clock_time();
            while !clock_time_exceed(t0, 4_500) {
                wd_clear();
            }
        }
        #[cfg(not(target_os = "none"))]
        {
            crate::mcu::clock::sleep_us(4_500);
        }
    }

    /// Initializes the DMA buffer for UART data reception.
    ///
    /// This function configures DMA channel 0 with the address and size of the RX buffer,
    /// enabling automatic data reception without CPU intervention.
    ///
    /// # Notes
    ///
    /// * The buffer size is calculated as UartData size divided by 16
    /// * Buffer must be properly aligned for DMA operations
    /// * Called internally during UART initialization
    fn init_buffers(&mut self) {
        // Calculate buffer length in 16-byte units (DMA works with 16-byte blocks)
        let buf_len = size_of::<UartData>() / 16;

        // Configure DMA channel 0 with the address of the RX buffer
        write_reg_dma0_addr(addr_of!(self.rx_data_buf) as u16);

        // Set the size of the RX buffer for DMA transfers
        write_reg_dma0_ctrl(buf_len as u16);
    }

    /// Clears UART error conditions by checking and resetting the error flags.
    ///
    /// This static function reads the UART status register, checks for errors,
    /// and clears them if present.
    ///
    /// # Notes
    ///
    /// * Checks bit 7 of status register (FLD_UART_STATUS::ERR_FLAG) for error indication
    /// * Clears errors by setting bit 6 of the same register
    /// * Can be called periodically to ensure UART remains in a clean state
    pub fn clear_error() {
        // Read UART status register
        let state = read_reg_uart_status();

        // Check if error flag (bit 7) is set
        if state & FLD_UART_STATUS::ERR_FLAG.bits() != 0 {
            // Clear error by setting bit 6 while preserving other bits
            write_reg_uart_status(state | FLD_UART_STATUS::ERR_CLR.bits());
        }
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;

    use crate::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::FakeMmio;

    /// `reg_system_tick`: the free-running tick `clock_time()` reads.
    const REG_SYSTEM_TICK: u32 = 0x740;
    /// `reg_tmr_ctrl`: the watchdog clear bit lives here.
    const REG_TMR_CTRL: u32 = 0x620;
    /// `reg_gpio_gpio_func` plus the port C register offset (`(pin >> 8) << 3`).
    const REG_GPIO_PC_FUNC: u32 = 0x586 + 0x10;
    /// `reg_gpio_oen` plus the port C register offset.
    const REG_GPIO_PC_OEN: u32 = 0x582 + 0x10;
    /// `reg_dma_rx_rdy0`: DMA/UART interrupt flags, write-1-to-clear.
    const REG_DMA_RX_RDY0: u32 = 0x526;
    /// `reg_dma_tx_rdy0`: write to trigger the DMA1 transfer.
    const REG_DMA_TX_RDY0: u32 = 0x524;
    /// `reg_uart_clk_div`: 16-bit divider plus the clock-enable bit.
    const REG_UART_CLK_DIV: u32 = 0x094;
    /// `reg_uart_ctrl0`: bit width plus the RX/TX DMA enables.
    const REG_UART_CTRL0: u32 = 0x096;
    /// `reg_uart_ctrl1`: flow control and parity.
    const REG_UART_CTRL1: u32 = 0x097;
    /// `reg_uart_rx_timeout`: receive timeout value.
    const REG_UART_RX_TIMEOUT: u32 = 0x09a;
    /// `reg_uart_rx_timeout_cnt`: receive timeout safety margin.
    const REG_UART_RX_TIMEOUT_CNT: u32 = 0x09b;
    /// `reg_uart_status`: UART status flags including the error bit.
    const REG_UART_STATUS: u32 = 0x09d;
    /// `reg_dma0_addr`: receive buffer address for DMA0 (16-bit).
    const REG_DMA0_ADDR: u32 = 0x500;
    /// `reg_dma0_ctrl`: receive buffer size for DMA0 (16-bit).
    const REG_DMA0_CTRL: u32 = 0x502;
    /// `reg_dma0_mode`: receive channel mode.
    const REG_DMA0_MODE: u32 = 0x503;
    /// `reg_dma1_addr`: transmit buffer address for DMA1 (16-bit).
    const REG_DMA1_ADDR: u32 = 0x504;
    /// `reg_dma1_mode`: transmit channel mode.
    const REG_DMA1_MODE: u32 = 0x507;
    /// `reg_dma_chn_en`: per-channel DMA enables.
    const REG_DMA_CHN_EN: u32 = 0x520;
    /// `reg_dma_chn_irq_msk`: per-channel DMA interrupt masks.
    const REG_DMA_CHN_IRQ_MSK: u32 = 0x521;
    /// `reg_irq_mask`: global interrupt mask (32-bit).
    const REG_IRQ_MASK: u32 = 0x640;

    /// The HAL host seams are process-wide singletons, so every test that
    /// observes them starts from a known-empty bank and tick count.
    fn reset_fakes() {
        FakeMmio::reset();
        fake_clock::reset();
    }

    /// Values written to `addr`, oldest first.
    fn values_written_to(addr: u32) -> Vec<u32> {
        FakeMmio::writes()
            .into_iter()
            .filter(|write| write.addr == addr | tlsr8266_pac::REG_BASE_ADDR)
            .map(|write| write.value)
            .collect()
    }

    /// Number of DMA1 transmit triggers, i.e. `start_tx_unchecked` arms.
    fn dma1_arms() -> usize {
        FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == REG_DMA_TX_RDY0 | tlsr8266_pac::REG_BASE_ADDR)
            .count()
    }

    fn watchdog_clears() -> usize {
        FakeMmio::writes()
            .into_iter()
            .filter(|write| write.addr == REG_TMR_CTRL | tlsr8266_pac::REG_BASE_ADDR)
            .count()
    }

    /// Tests the default constructor for the UART driver.
    ///
    /// This test verifies that the UartDriver::default_const constructor:
    /// - Initializes transmit and receive buffers correctly
    /// - Sets the busy flag to false
    /// - Creates a ready-to-use driver instance
    #[test]
    fn test_uart_driver_default() {
        // Create a new driver instance using the default constructor
        let driver = UartDriver::default_const();

        // Verify transmit buffer is initialized empty
        assert_eq!(driver.tx_data_buf.len, 0);
        assert_eq!(driver.tx_data_buf.data, [0; UART_DATA_LEN]);

        // Verify receive buffer is initialized empty
        assert_eq!(driver.rx_data_buf.len, 0);
        assert_eq!(driver.rx_data_buf.data, [0; UART_DATA_LEN]);

        // Verify transmit busy flag is initially false
        assert!(!driver.is_tx_busy);
    }

    /// Tests the tx_busy accessor method.
    #[test]
    fn test_tx_busy() {
        // Create a driver with default settings
        let mut driver = UartDriver::default_const();

        // Initially, the busy flag should be false
        assert!(!driver.is_tx_busy());

        // Set the busy flag to true
        driver.is_tx_busy = true;

        // Now the busy flag should be true
        assert!(driver.is_tx_busy());
    }

    /// Tests the UART initialization function.
    ///
    /// This test verifies that the init method correctly:
    /// - Configures GPIO pins for UART functionality
    /// - Initializes DMA buffers for TX/RX
    /// - Programs the UART registers for 115200 baud
    /// - Enables both DMA channels and their interrupts
    ///
    /// # Notes
    ///
    /// * Assertions are the register contents the peripheral would hold after
    ///   `init()` returns
    #[test]
    fn test_uart_init() {
        reset_fakes();
        // Seed both UART pins as peripheral-selected (bits set) so the
        // `AS_UART` clears below are observable, and the OEN bank at 0.
        FakeMmio::set_reg8(REG_GPIO_PC_FUNC, 0xff);
        FakeMmio::set_reg8(REG_GPIO_PC_OEN, 0x00);

        // Create a new driver and initialize it
        let mut driver = UartDriver::default_const();
        driver.init();

        // Verify GPIO configuration: both UART pins leave the GPIO-function
        // register (their bits cleared for `AS_UART`), and the RX pin's input
        // enable sets its OEN bit.
        assert_eq!(
            FakeMmio::reg8(REG_GPIO_PC_FUNC),
            0x3f,
            "UTX (bit 6) and URX (bit 7) must both select the UART function"
        );
        assert_eq!(
            FakeMmio::reg8(REG_GPIO_PC_OEN),
            0x80,
            "the RX pin must have its input enable bit set"
        );

        // Verify UART register initialization
        assert_eq!(
            FakeMmio::reg16(REG_UART_CLK_DIV),
            69u16 | FLD_UART_CLK_DIV::CLK_EN.bits(),
            "clock divider 69 with the clock enabled"
        );
        assert_eq!(
            FakeMmio::reg8(REG_UART_CTRL0),
            FLD_UART_CTRL0::RX_DMA_EN.bits() | FLD_UART_CTRL0::TX_DMA_EN.bits() | 3,
            "bit width 3 with both DMA directions enabled"
        );
        assert_eq!(
            FakeMmio::reg8(REG_UART_CTRL1),
            0x00,
            "no hardware flow control or parity"
        );
        // A 115200 baud configuration uses the maximum timeout and margin 3.
        assert_eq!(FakeMmio::reg8(REG_UART_RX_TIMEOUT), 0xff);
        assert_eq!(FakeMmio::reg8(REG_UART_RX_TIMEOUT_CNT), 3);

        // Verify DMA configuration: DMA0 receives, DMA1 transmits.
        assert_eq!(FakeMmio::reg8(REG_DMA0_MODE), 0x01);
        assert_eq!(FakeMmio::reg8(REG_DMA1_MODE), 0x00);
        assert_eq!(
            values_written_to(REG_DMA_RX_RDY0),
            vec![0],
            "the pending interrupt flags were read and cleared once"
        );

        // Verify the interrupt/DMA enables accumulated from a zeroed bank:
        // both channel interrupts masked, the global DMA interrupt enabled and
        // both channels enabled.
        assert_eq!(
            FakeMmio::reg8(REG_DMA_CHN_IRQ_MSK),
            (FLD_DMA::ETH_RX.bits() | FLD_DMA::ETH_TX.bits()) as u8
        );
        assert_eq!(
            FakeMmio::reg32(REG_IRQ_MASK),
            FLD_IRQ::DMA_EN.bits(),
            "the global DMA interrupt must be unmasked"
        );
        assert_eq!(
            FakeMmio::reg8(REG_DMA_CHN_EN),
            (BIT!(0) | BIT!(1)) as u8,
            "both DMA channels must be enabled"
        );

        // DMA0 is pointed at the driver's receive buffer.
        assert_eq!(
            values_written_to(REG_DMA0_ADDR),
            vec![u32::from(addr_of!(driver.rx_data_buf) as u16)],
            "DMA0 must be pointed at the RX buffer"
        );
        assert_eq!(
            values_written_to(REG_DMA0_CTRL),
            vec![(size_of::<UartData>() / 16) as u32],
            "DMA0 must be told the RX buffer size in 16-byte units"
        );
    }

    /// Tests that the UART initialization function correctly validates the bit width parameter.
    ///
    /// This test verifies that the uart_init method returns Err(()) when:
    /// - The bit width parameter (bwpc) is less than the minimum required value of 3
    #[test]
    fn test_uart_init_invalid_bit_width() {
        // Create a new driver with default settings
        let mut driver = UartDriver::default_const();

        // Call uart_init with an invalid bit width (below minimum of 3)
        let result = driver.uart_init(69, 2, true, true, HardwareControl::NoControl);

        // Verify the function returns Err(()) for invalid parameters
        assert!(result.is_err());
    }

    /// Tests the UART IRQ source and flag clearing functionality with no flags set.
    #[test]
    fn test_get_and_clear_irq_source_no_flags() {
        // No interrupt flags set
        reset_fakes();
        FakeMmio::set_reg8(REG_DMA_RX_RDY0, 0);

        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, 0);

        // The raw value is written back to clear the flags.
        assert_eq!(values_written_to(REG_DMA_RX_RDY0), vec![0]);
    }

    /// Tests the UART IRQ source and flag clearing functionality with RX flag set.
    #[test]
    fn test_get_and_clear_irq_source_rx_flag() {
        // RX interrupt flag set
        reset_fakes();
        FakeMmio::set_reg8(REG_DMA_RX_RDY0, UartIrqMask::Rx.bits());

        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, UartIrqMask::Rx.bits());

        // The raw value is written back to clear the flags.
        assert_eq!(
            values_written_to(REG_DMA_RX_RDY0),
            vec![u32::from(UartIrqMask::Rx.bits())]
        );
    }

    /// Tests the UART IRQ source and flag clearing functionality with TX flag set.
    #[test]
    fn test_get_and_clear_irq_source_tx_flag() {
        // TX interrupt flag set
        reset_fakes();
        FakeMmio::set_reg8(REG_DMA_RX_RDY0, UartIrqMask::Tx.bits());

        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, UartIrqMask::Tx.bits());

        // The raw value is written back to clear the flags.
        assert_eq!(
            values_written_to(REG_DMA_RX_RDY0),
            vec![u32::from(UartIrqMask::Tx.bits())]
        );
    }

    /// Tests the UART IRQ source and flag clearing functionality with both RX and TX flags set.
    #[test]
    fn test_get_and_clear_irq_source_all_flags() {
        // Both RX and TX interrupt flags set
        reset_fakes();
        FakeMmio::set_reg8(REG_DMA_RX_RDY0, UartIrqMask::all());

        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, UartIrqMask::all());

        // The raw value is written back to clear the flags.
        assert_eq!(
            values_written_to(REG_DMA_RX_RDY0),
            vec![u32::from(UartIrqMask::all())]
        );
    }

    /// Tests the UART IRQ source and flag clearing functionality with masking of irrelevant bits.
    #[test]
    fn test_get_and_clear_irq_source_with_masking() {
        // Additional irrelevant bits set (should be masked out)
        reset_fakes();
        let raw_value = 0xF0 | UartIrqMask::Rx.bits();
        FakeMmio::set_reg8(REG_DMA_RX_RDY0, raw_value);

        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, UartIrqMask::Rx.bits()); // Should only have RX bit, others masked

        // The raw value is written back to clear the flags.
        assert_eq!(
            values_written_to(REG_DMA_RX_RDY0),
            vec![u32::from(raw_value)],
            "the whole register is written back, not just the UART bits"
        );
    }

    /// Tests the TX busy flag manipulation methods.
    #[test]
    fn test_tx_busy_flag_manipulation() {
        reset_fakes();

        let mut driver = UartDriver::default_const();

        // Initial state should be not busy
        assert!(!driver.is_tx_busy());

        // Arm a transfer: busy flag set, DMA1 configured and triggered
        let msg = UartData {
            len: 0,
            data: [0; UART_DATA_LEN],
        };
        driver.start_tx_unchecked(&msg);
        assert!(driver.is_tx_busy());
        assert_eq!(dma1_arms(), 1, "DMA1 must be triggered once");

        // Clear busy flag
        driver.clear_tx_busy_flag();
        assert!(!driver.is_tx_busy());
    }

    /// Tests the synchronous UART data transmission function when the driver is not busy.
    ///
    /// This test verifies that the send method correctly:
    /// - Sets the busy flag
    /// - Copies message data to the internal TX buffer
    /// - Triggers the DMA transfer
    /// - Does not wait (so the watchdog is never kicked)
    #[test]
    fn test_send_not_busy() {
        reset_fakes();

        // Create a test message
        let mut msg = UartData {
            len: 4,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0xAA;
        msg.data[1] = 0xBB;
        msg.data[2] = 0xCC;
        msg.data[3] = 0xDD;

        // Driver is not busy
        let mut driver = UartDriver::default_const();
        driver.send(&msg);

        // Verify result and busy flag
        assert!(driver.is_tx_busy);

        // The message was copied into the DMA buffer the driver points at.
        assert_eq!(driver.tx_data_buf.len, msg.len);
        assert_eq!(driver.tx_data_buf.data, msg.data);

        // Verify DMA configuration: `start_tx_unchecked` armed DMA1 once.
        assert_eq!(dma1_arms(), 1, "DMA1 must be triggered once");

        // No waiting should have happened: the loop body would kick the
        // watchdog, and with a frozen fake tick the loop could never satisfy its
        // timeout, so a zero kick count proves the `is_tx_busy()` guard
        // short-circuited `clock_time_exceed` entirely.
        assert_eq!(watchdog_clears(), 0, "no wait means no watchdog kick");
    }

    /// Tests the synchronous UART data transmission function when the driver is busy.
    ///
    /// This test verifies that the send method correctly:
    /// - Waits for any previous transmission to complete
    /// - Kicks the watchdog during the wait
    /// - Arms the transfer once the wait times out
    #[test]
    fn test_send_busy() {
        reset_fakes();

        // `send` spins on `clock_time_exceed(t_timeout, 10 ms)` while the tick
        // register (0x740) does not auto-advance, so the test scripts the tick:
        // the first read establishes the reference, and every later read jumps
        // 200_000 ticks (6.25 ms at 32 MHz). The 320_000-tick timeout therefore
        // trips on the second probe and the loop body runs exactly once.
        FakeMmio::on_read32(REG_SYSTEM_TICK, |call_idx| {
            1_000 + call_idx as u32 * 200_000
        });

        // Create a test message
        let mut msg = UartData {
            len: 4,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0xAA;
        msg.data[1] = 0xBB;
        msg.data[2] = 0xCC;
        msg.data[3] = 0xDD;

        // Driver is initially busy
        let mut driver = UartDriver::default_const();
        driver.is_tx_busy = true;

        driver.send(&msg);

        // Verify result and final busy state
        assert!(driver.is_tx_busy);

        // The message was still handed to the DMA engine.
        assert_eq!(driver.tx_data_buf.data, msg.data);
        assert_eq!(dma1_arms(), 1, "DMA1 must be triggered once");

        // Verify waiting occurred: one kick per loop iteration. With the tick
        // advancing 200_000 per read, the 320_000-tick timeout is reached on the
        // second probe, so exactly one iteration (one kick) may run — a second
        // kick would mean the loop body ran twice.
        assert_eq!(
            watchdog_clears(),
            1,
            "the busy-wait must kick the watchdog once before timing out"
        );
    }

    /// The panic sink clears a stuck DMA1 channel before arming the frame.
    ///
    /// With a transfer in flight, `send_panic` must disable DMA1 (bit 1 low),
    /// clear the pending TX flag, re-enable DMA1 (bit 1 high), clear the busy
    /// flag, arm the transfer and return — never spinning on the host.
    #[test]
    fn test_send_panic_busy_resets_dma1_and_arms() {
        reset_fakes();
        // Both channels enabled, as `init()` leaves them.
        FakeMmio::set_reg8(REG_DMA_CHN_EN, (BIT!(0) | BIT!(1)) as u8);

        let mut msg = UartData {
            len: 4,
            data: [0; UART_DATA_LEN],
        };
        msg.data[0] = 0xAA;
        msg.data[1] = 0xBB;
        msg.data[2] = 0xCC;
        msg.data[3] = 0xDD;

        let mut driver = UartDriver::default_const();
        driver.is_tx_busy = true;

        driver.send_panic(&msg);

        // DMA1 is disabled, then re-enabled with the RX channel untouched.
        assert_eq!(
            values_written_to(REG_DMA_CHN_EN),
            vec![
                u32::from(BIT!(0) as u8),
                u32::from((BIT!(0) | BIT!(1)) as u8)
            ],
            "DMA1 must be written low then high around the flag clear"
        );

        // The pending TX flag is cleared by writing the TX bit.
        assert_eq!(
            values_written_to(REG_DMA_RX_RDY0),
            vec![u32::from(UartIrqMask::Tx.bits())],
            "the pending TX flag must be cleared"
        );

        // The transfer is armed at the driver's TX buffer and triggered.
        assert_eq!(
            values_written_to(REG_DMA1_ADDR),
            vec![u32::from(addr_of!(driver.tx_data_buf) as u16)],
            "DMA1 must be pointed at the TX buffer"
        );
        assert_eq!(dma1_arms(), 1, "DMA1 must be triggered once");
        assert!(driver.is_tx_busy);
        assert_eq!(driver.tx_data_buf.data, msg.data);

        // Host path: one recorded 4.5 ms shift-out wait, no watchdog spin.
        assert_eq!(fake_clock::sleeps(), vec![4_500]);
        assert_eq!(watchdog_clears(), 0, "the host path must not spin");
    }

    /// An idle driver skips the DMA1 reset and still arms the transfer.
    #[test]
    fn test_send_panic_idle_skips_dma1_reset() {
        reset_fakes();
        FakeMmio::set_reg8(REG_DMA_CHN_EN, (BIT!(0) | BIT!(1)) as u8);

        let msg = UartData {
            len: 2,
            data: [0; UART_DATA_LEN],
        };

        let mut driver = UartDriver::default_const();
        assert!(!driver.is_tx_busy());

        driver.send_panic(&msg);

        assert!(
            values_written_to(REG_DMA_CHN_EN).is_empty(),
            "an idle driver must not touch the DMA channel enables"
        );
        assert!(
            values_written_to(REG_DMA_RX_RDY0).is_empty(),
            "an idle driver has no pending TX flag to clear"
        );
        assert_eq!(dma1_arms(), 1, "DMA1 must be triggered once");
        assert!(driver.is_tx_busy);
        assert_eq!(fake_clock::sleeps(), vec![4_500]);
    }

    /// Tests the DMA buffer initialization for UART reception.
    #[test]
    fn test_uart_buff_init() {
        reset_fakes();

        // Create a driver and call buffer initialization
        let mut driver = UartDriver::default_const();
        driver.init_buffers();

        // Calculate expected buffer size
        let expected_buf_len = size_of::<UartData>() / 16;

        // DMA0 is pointed at the driver's receive buffer and told its size.
        assert_eq!(
            values_written_to(REG_DMA0_ADDR),
            vec![u32::from(addr_of!(driver.rx_data_buf) as u16)],
            "DMA0 must be pointed at the RX buffer"
        );
        assert_eq!(
            values_written_to(REG_DMA0_CTRL),
            vec![u32::from(expected_buf_len as u16)],
            "DMA0 must be told the RX buffer size in 16-byte units"
        );
    }

    /// Tests the UART error clearing functionality when no error is present.
    ///
    /// This test verifies that the clear_error method:
    /// - Reads the UART status register
    /// - Detects that no error is present (bit 7 not set)
    /// - Performs no write when no error is detected
    #[test]
    fn test_clear_error_no_error() {
        reset_fakes();
        // No error present (bit 7 not set)
        FakeMmio::set_reg8(REG_UART_STATUS, 0x00);

        UartDriver::clear_error();

        // No write should occur.
        assert!(
            values_written_to(REG_UART_STATUS).is_empty(),
            "a clean status register must not be written"
        );
        assert_eq!(FakeMmio::reg8(REG_UART_STATUS), 0x00);
    }

    /// Tests the UART error clearing functionality when a basic error is present.
    #[test]
    fn test_clear_error_basic_error() {
        reset_fakes();
        // Error present (bit 7 set)
        FakeMmio::set_reg8(REG_UART_STATUS, FLD_UART_STATUS::ERR_FLAG.bits());

        UartDriver::clear_error();

        // The clear bit is set on top of the observed status.
        let expected = FLD_UART_STATUS::ERR_FLAG.bits() | FLD_UART_STATUS::ERR_CLR.bits();
        assert_eq!(
            values_written_to(REG_UART_STATUS),
            vec![u32::from(expected)]
        );
        assert_eq!(FakeMmio::reg8(REG_UART_STATUS), expected);
    }

    /// Tests the UART error clearing functionality when an error with other flags is present.
    #[test]
    fn test_clear_error_with_other_bits() {
        reset_fakes();
        // Error with other bits set (bits 0, 2 and 7 set)
        let status_with_error = FLD_UART_STATUS::ERR_FLAG.bits() | 0x05; // 0x05 for bits 0 and 2
        FakeMmio::set_reg8(REG_UART_STATUS, status_with_error);

        UartDriver::clear_error();

        // The other bits are preserved; only the clear bit is added.
        let expected = status_with_error | FLD_UART_STATUS::ERR_CLR.bits();
        assert_eq!(
            values_written_to(REG_UART_STATUS),
            vec![u32::from(expected)]
        );
        assert_eq!(FakeMmio::reg8(REG_UART_STATUS), expected);
    }

    /// Tests the UART initialization function with 9600 baud rate configuration.
    ///
    /// This test verifies that the uart_init method correctly:
    /// - Configures registers for a low baud rate (9600)
    /// - Sets the timeout from the bit width: (bwpc + 1) * 12 = 168
    /// - Sets the lower safety margin (1 instead of 3)
    #[test]
    fn test_uart_init_9600_baud() {
        reset_fakes();

        // Create a new driver and initialize UART with 9600 baud parameters
        // For 32MHz system clock, 9600 baud: clkdiv=237, bwpc=13
        let mut driver = UartDriver::default_const();
        let result = driver.uart_init(237, 13, true, true, HardwareControl::NoControl);

        // Verify initialization succeeded
        assert!(result.is_ok());

        // Verify UART register configuration
        assert_eq!(
            FakeMmio::reg16(REG_UART_CLK_DIV),
            237u16 | FLD_UART_CLK_DIV::CLK_EN.bits()
        );
        assert_eq!(
            FakeMmio::reg8(REG_UART_CTRL0),
            FLD_UART_CTRL0::RX_DMA_EN.bits() | FLD_UART_CTRL0::TX_DMA_EN.bits() | 13
        );

        // Verify specific low baud rate configuration
        assert_eq!(
            FakeMmio::reg8(REG_UART_RX_TIMEOUT),
            168,
            "timeout: (13 + 1) * 12 = 168"
        );
        assert_eq!(FakeMmio::reg8(REG_UART_RX_TIMEOUT_CNT), 1);

        // Verify hardware control setting
        assert_eq!(FakeMmio::reg8(REG_UART_CTRL1), 0x00);

        // Verify DMA configuration
        assert_eq!(FakeMmio::reg8(REG_DMA0_MODE), 0x01);
        assert_eq!(FakeMmio::reg8(REG_DMA1_MODE), 0x00);

        // Verify interrupt configuration
        assert_eq!(values_written_to(REG_DMA_RX_RDY0), vec![0]);
        assert_eq!(
            FakeMmio::reg8(REG_DMA_CHN_IRQ_MSK),
            (FLD_DMA::ETH_RX.bits() | FLD_DMA::ETH_TX.bits()) as u8
        );
        assert_eq!(FakeMmio::reg32(REG_IRQ_MASK), FLD_IRQ::DMA_EN.bits());
        assert_eq!(FakeMmio::reg8(REG_DMA_CHN_EN), (BIT!(0) | BIT!(1)) as u8);
    }
}
