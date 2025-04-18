use core::mem::size_of;
use core::ptr::addr_of;

use crate::{BIT, uprintln};
use crate::embassy::yield_now::yield_now;
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_HZ, clock_time, clock_time_exceed};
use crate::sdk::mcu::gpio::{AS_UART, GPIO_PIN_TYPE, gpio_set_func, gpio_set_input_en};
use crate::sdk::mcu::register::{FLD_DMA, FLD_IRQ, FLD_UART_CLK_DIV, FLD_UART_CTRL0, FLD_UART_CTRL1, FLD_UART_STATUS, 
                                read_reg8, read_reg_dma_chn_en, read_reg_dma_chn_irq_msk, read_reg_dma_rx_rdy0, 
                                read_reg_irq_mask, read_reg_uart_status, write_reg16, write_reg8, write_reg_dma0_addr, 
                                write_reg_dma0_ctrl, write_reg_dma0_mode, write_reg_dma1_addr, write_reg_dma1_mode,
                                write_reg_dma_chn_en, write_reg_dma_chn_irq_msk, write_reg_dma_rx_rdy0, 
                                write_reg_dma_tx_rdy0, write_reg_irq_mask, write_reg_uart_clk_div, 
                                write_reg_uart_ctrl0, write_reg_uart_ctrl1, write_reg_uart_rx_timeout, 
                                write_reg_uart_rx_timeout_cnt, write_reg_uart_status};
use crate::sdk::mcu::watchdog::wd_clear;

/// Maximum length of data that can be stored in a UART data buffer.
/// 
/// This value must be carefully chosen to balance memory usage against
/// maximum message size requirements. The TLSR8266 has limited memory,
/// so we keep this reasonably small.
pub const UART_DATA_LEN: usize = 44;      // data max 252

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

#[derive(Clone, Copy, Debug)]
/// UART data packet structure for transmission and reception.
/// 
/// This struct must be a multiple of 16 bytes in size to ensure proper
/// DMA alignment for efficient data transfer.
pub struct UartData {
    /// Length of valid data in the buffer (in bytes).
    /// Although the field is 32 bits, the maximum length is limited by UART_DATA_LEN.
    pub len: u32,
    /// Buffer containing the actual UART data.
    /// The maximum usable size is defined by UART_DATA_LEN.
    pub data: [u8; UART_DATA_LEN]
}

/// Driver for UART communication on the TLSR8266.
/// 
/// This driver provides functionality for initializing, transmitting, and receiving
/// data over UART using DMA for efficient transfers. It handles the low-level
/// register access and timing requirements of the UART peripheral.
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
    pub const fn default() -> Self {
        Self {
            tx_data_buf: UartData {len: 0, data: [0; UART_DATA_LEN]},
            rx_data_buf: UartData {len: 0, data: [0; UART_DATA_LEN]},
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
        self.uart_init(69, 3, true, true, HardwareControl::NoControl);
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
    /// * `true` if initialization succeeded
    /// * `false` if invalid parameters were provided (e.g., bwpc < 3)
    ///
    /// # Notes
    ///
    /// * Baud rate calculation: BaudRate = CLOCK_SYS_CLOCK_HZ/((uart_clkdiv+1)*(bwpc+1))
    /// * For 32MHz system clock:
    ///   - 115200 baud: clkdiv=19, bwpc=13
    ///   - 9600 baud: clkdiv=237, bwpc=13
    fn uart_init(&mut self, uart_clkdiv: u16, bwpc: u8, enable_rx_irq: bool, enable_tx_irq: bool, hw_control: HardwareControl) -> bool {
        // Validate bit width parameter - must be at least 3
        if bwpc < 3 {
            return false;
        }

        // Calculate actual baud rate based on system clock and parameters
        let baudrate = (CLOCK_SYS_CLOCK_HZ / (uart_clkdiv as u32 + 1)) / (bwpc as u32 + 1);

        // --- Configure UART Registers ---
        
        // Set UART clock divider (bits 0-14) and enable clock (bit 15)
        write_reg_uart_clk_div(uart_clkdiv | FLD_UART_CLK_DIV::CLK_EN.bits());
        
        // Set bit width (bits 0-4) and enable RX/TX DMA (bits 4-5)
        write_reg_uart_ctrl0(FLD_UART_CTRL0::RX_DMA_EN.bits() | FLD_UART_CTRL0::TX_DMA_EN.bits() | bwpc);

        // Configure UART timing parameters based on baud rate
        if baudrate > 100000 {          // For high baud rates (e.g., 115200)
            write_reg_uart_rx_timeout(0xff);  // Maximum timeout
            write_reg_uart_rx_timeout_cnt(3);  // Higher safety margin
        } else {                         // For low baud rates (e.g., 9600)
            // One byte includes 12 bits at most (start + 8 data + parity + stop)
            write_reg_uart_rx_timeout((bwpc+1)*12);  // Timeout based on bit width
            write_reg_uart_rx_timeout_cnt(1);       // Lower safety margin
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
        
        true
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

    /// Checks if the UART transmitter is currently busy.
    ///
    /// This is an internal helper function that checks the busy flag.
    ///
    /// # Returns
    ///
    /// * `true` if a transmission is in progress
    /// * `false` if the transmitter is idle
    ///
    /// # Notes
    ///
    /// * Internal method used by send
    #[inline(always)]
    fn uart_tx_is_busy(&self) -> bool {
        return self.is_tx_busy;
    }

    /// Sends data asynchronously over UART using DMA.
    ///
    /// This async function waits for any previous transmission to complete,
    /// then initiates a new DMA transfer. It yields to the async runtime while
    /// waiting, allowing other tasks to execute.
    ///
    /// # Parameters
    ///
    /// * `msg` - The UART data packet to transmit
    ///
    /// # Returns
    ///
    /// * `true` if transmission was initiated successfully
    ///
    /// # Notes
    ///
    /// * This function returns after initiating the transfer, not when the transfer completes
    /// * The busy flag remains set until cleared by `clear_tx_busy_flag`
    /// * Maximum wait time for previous transmission is 100ms
    pub async fn send_async(&mut self, msg: &UartData) -> bool {
        // Get current time for timeout calculation
        let t_timeout = clock_time();
        
        // Wait for any previous transmission to complete (with timeout)
        // Yields to async runtime while waiting to allow other tasks to run
        while self.is_tx_busy() && !clock_time_exceed(t_timeout, 100*1000) {
            yield_now().await;
        }

        // Mark transmitter as busy and copy message to TX buffer
        self.set_tx_busy_flag();
        self.tx_data_buf = *msg;

        // Configure DMA1 with address of TX buffer
        write_reg_dma1_addr(addr_of!(self.tx_data_buf) as u16);

        // Trigger DMA transfer to start transmission
        write_reg_dma_tx_rdy0(FLD_DMA::ETH_TX.bits() as u8);

        true
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
    /// * `true` if transmission was initiated successfully
    ///
    /// # Notes
    ///
    /// * This function returns after initiating the transfer, not when the transfer completes
    /// * Uses critical_section to prevent interrupts during the setup process
    /// * Maximum wait time for previous transmission is 10ms
    /// * Watchdog is kicked during waiting to prevent resets
    pub fn send(&mut self, msg: &UartData) -> bool {
        critical_section::with(|_| {
            // Get current time for timeout calculation
            let t_timeout = clock_time();
            
            // Wait for any previous transmission to complete (with timeout)
            // Kick watchdog while waiting to prevent system reset
            while self.is_tx_busy() && !clock_time_exceed(t_timeout, 10 * 1000) {
                wd_clear();
            }

            // Copy message to TX buffer
            self.set_tx_busy_flag();
            self.tx_data_buf = *msg;

            // Configure DMA1 with address of TX buffer
            write_reg_dma1_addr(addr_of!(self.tx_data_buf) as u16);

            // Trigger DMA transfer to start transmission
            write_reg_dma_tx_rdy0(FLD_DMA::ETH_TX.bits() as u8);

            true
        })
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

    /// Sets the UART TX busy flag to indicate transmission in progress.
    ///
    /// This is an internal helper function used before initiating a new
    /// transmission to prevent concurrent transmissions.
    ///
    /// # Notes
    ///
    /// * Internal method used by send and send_async
    fn set_tx_busy_flag(&mut self) {
        self.is_tx_busy = true;
    }

    /// Clears UART error conditions by checking and resetting the error flags.
    ///
    /// This static function reads the UART status register, checks for errors,
    /// and clears them if present.
    ///
    /// # Returns
    ///
    /// * `true` if an error was detected and cleared
    /// * `false` if no error was detected
    ///
    /// # Notes
    ///
    /// * Checks bit 7 of status register (FLD_UART_STATUS::ERR_FLAG) for error indication
    /// * Clears errors by setting bit 6 of the same register
    /// * Can be called periodically to ensure UART remains in a clean state
    pub fn clear_error() -> bool {
        // Read UART status register
        let state = read_reg_uart_status();
        
        // Check if error flag (bit 7) is set
        if state & FLD_UART_STATUS::ERR_FLAG.bits() != 0 {
            // Clear error by setting bit 6 while preserving other bits
            write_reg_uart_status(state | FLD_UART_STATUS::ERR_CLR.bits());
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;
    
    // Import mock versions of the functions from their original modules
    use crate::sdk::mcu::register::{
        mock_read_reg_dma_chn_en, mock_read_reg_dma_chn_irq_msk, 
        mock_read_reg_dma_rx_rdy0, mock_read_reg_irq_mask, mock_read_reg_uart_status,
        mock_write_reg_dma0_addr, mock_write_reg_dma0_ctrl, mock_write_reg_dma0_mode,
        mock_write_reg_dma1_addr, mock_write_reg_dma1_mode, mock_write_reg_dma_chn_en, 
        mock_write_reg_dma_chn_irq_msk, mock_write_reg_dma_rx_rdy0, mock_write_reg_dma_tx_rdy0, 
        mock_write_reg_irq_mask, mock_write_reg_uart_clk_div, mock_write_reg_uart_ctrl0, 
        mock_write_reg_uart_ctrl1, mock_write_reg_uart_rx_timeout, mock_write_reg_uart_rx_timeout_cnt,
        mock_write_reg_uart_status
    };
    use crate::sdk::mcu::gpio::{mock_gpio_set_func, mock_gpio_set_input_en};
    use crate::sdk::mcu::clock::{mock_clock_time, mock_clock_time_exceed};
    use crate::sdk::mcu::watchdog::mock_wd_clear;
    use crate::embassy::yield_now::mock_yield_now;
    
    /// Tests the default constructor for the UART driver.
    ///
    /// This test verifies that the UartDriver::default constructor:
    /// - Initializes transmit and receive buffers correctly
    /// - Sets the busy flag to false
    /// - Creates a ready-to-use driver instance
    ///
    /// # Notes
    ///
    /// * No mock functions needed since this only tests struct initialization
    #[test]
    fn test_uart_driver_default() {
        // Create a new driver instance using the default constructor
        let driver = UartDriver::default();
        
        // Verify transmit buffer is initialized empty
        assert_eq!(driver.tx_data_buf.len, 0);
        assert_eq!(driver.tx_data_buf.data, [0; UART_DATA_LEN]);
        
        // Verify receive buffer is initialized empty
        assert_eq!(driver.rx_data_buf.len, 0);
        assert_eq!(driver.rx_data_buf.data, [0; UART_DATA_LEN]);
        
        // Verify transmit busy flag is initially false
        assert_eq!(driver.is_tx_busy, false);
    }
    
    /// Tests the tx_busy accessor method.
    ///
    /// This test verifies that the tx_busy method correctly:
    /// - Returns the current state of the is_tx_busy flag
    ///
    /// # Notes
    ///
    /// * Tests both true and false states to ensure proper functionality
    #[test]
    fn test_tx_busy() {
        // Create a driver with default settings
        let mut driver = UartDriver::default();
        
        // Initially, the busy flag should be false
        assert_eq!(driver.is_tx_busy(), false);
        
        // Set the busy flag to true
        driver.is_tx_busy = true;
        
        // Now the busy flag should be true
        assert_eq!(driver.is_tx_busy(), true);
    }
    
    /// Tests the UART initialization function.
    ///
    /// This test verifies that the init method correctly:
    /// - Configures GPIO pins for UART functionality
    /// - Initializes DMA buffers for TX/RX
    /// - Sets up UART with appropriate baud rate and settings
    ///
    /// # Algorithm
    ///
    /// 1. Mock GPIO and register access functions
    /// 2. Call the init method
    /// 3. Verify all expected hardware configuration calls
    /// 4. Check that the correct parameters were used for each call
    ///
    /// # Notes
    ///
    /// * Validates the initialization sequence is correct
    /// * Checks that GPIO pins are properly configured
    /// * Confirms DMA is set up correctly
    #[test]
    #[mry::lock(
        gpio_set_func, gpio_set_input_en,
        write_reg_dma0_addr, write_reg_dma0_ctrl, 
        read_reg_dma_rx_rdy0, write_reg_dma_rx_rdy0,
        read_reg_dma_chn_irq_msk, write_reg_dma_chn_irq_msk,
        read_reg_irq_mask, write_reg_irq_mask,
        read_reg_dma_chn_en, write_reg_dma_chn_en, 
        write_reg_uart_clk_div, write_reg_uart_ctrl0,
        write_reg_uart_rx_timeout, write_reg_uart_rx_timeout_cnt,
        write_reg_uart_ctrl1, write_reg_dma0_mode, write_reg_dma1_mode
    )]
    fn test_uart_init() {
        // Setup mock responses for GPIO functions
        mock_gpio_set_func(GPIO_PIN_TYPE::GPIO_UTX as u32, AS_UART).returns(());
        mock_gpio_set_func(GPIO_PIN_TYPE::GPIO_URX as u32, AS_UART).returns(());
        mock_gpio_set_input_en(GPIO_PIN_TYPE::GPIO_URX as u32, 1).returns(());
        
        // Setup mock responses for DMA buffer initialization
        mock_write_reg_dma0_addr(mry::Any).returns(());
        mock_write_reg_dma0_ctrl(mry::Any).returns(());
        
        // Setup mock responses for UART register configuration
        mock_write_reg_uart_clk_div(69 | FLD_UART_CLK_DIV::CLK_EN.bits()).returns(());  // Clock divider
        mock_write_reg_uart_ctrl0(FLD_UART_CTRL0::RX_DMA_EN.bits() | FLD_UART_CTRL0::TX_DMA_EN.bits() | 3).returns(());  // Bit width
        mock_write_reg_uart_rx_timeout(0xff).returns(());          // Timeout
        mock_write_reg_uart_rx_timeout_cnt(3).returns(());         // Safety margin
        mock_write_reg_uart_ctrl1(0x00).returns(());               // Hardware control
        mock_write_reg_dma0_mode(0x01).returns(());                // DMA0 mode
        mock_write_reg_dma1_mode(0x00).returns(());                // DMA1 mode
        
        // Setup mock responses for interrupt clearing
        mock_read_reg_dma_rx_rdy0().returns(0);
        mock_write_reg_dma_rx_rdy0(0).returns(());
        
        // Setup mock responses for IRQ configuration
        mock_read_reg_dma_chn_irq_msk().returns(0);
        mock_write_reg_dma_chn_irq_msk(mry::Any).returns(());
        mock_read_reg_irq_mask().returns(0);
        mock_write_reg_irq_mask(mry::Any).returns(());
        mock_read_reg_dma_chn_en().returns(0);
        mock_write_reg_dma_chn_en(mry::Any).returns(());
        
        // Create a new driver and initialize it
        let mut driver = UartDriver::default();
        driver.init();
        
        // Verify GPIO configuration
        mock_gpio_set_func(GPIO_PIN_TYPE::GPIO_UTX as u32, AS_UART).assert_called(1);
        mock_gpio_set_func(GPIO_PIN_TYPE::GPIO_URX as u32, AS_UART).assert_called(1);
        mock_gpio_set_input_en(GPIO_PIN_TYPE::GPIO_URX as u32, 1).assert_called(1);
        
        // Verify UART register initialization
        mock_write_reg_uart_clk_div(69 | FLD_UART_CLK_DIV::CLK_EN.bits()).assert_called(1);  // Clock divider
        mock_write_reg_uart_ctrl0(FLD_UART_CTRL0::RX_DMA_EN.bits() | FLD_UART_CTRL0::TX_DMA_EN.bits() | 3).assert_called(1);  // Bit width with DMA enable
        mock_write_reg_uart_ctrl1(0x00).assert_called(1);          // Hardware control
        
        // Verify DMA configuration
        mock_write_reg_dma0_mode(0x01).assert_called(1);  // DMA0 mode (receive)
        mock_write_reg_dma1_mode(0x00).assert_called(1);  // DMA1 mode (transmit)
        
        // Verify IRQ configuration
        mock_read_reg_dma_rx_rdy0().assert_called(1);
        mock_write_reg_dma_rx_rdy0(0).assert_called(1);
        
        // Verify DMA channel and interrupt configuration 
        // (exact values depend on FLD_DMA and FLD_IRQ enums)
        mock_read_reg_dma_chn_irq_msk().assert_called(2);
        mock_write_reg_dma_chn_irq_msk(mry::Any).assert_called(2);
        mock_read_reg_irq_mask().assert_called(2);
        mock_write_reg_irq_mask(mry::Any).assert_called(2);
        mock_read_reg_dma_chn_en().assert_called(2);
        mock_write_reg_dma_chn_en(mry::Any).assert_called(2);
    }
    
    /// Tests that the UART initialization function correctly validates the bit width parameter.
    ///
    /// This test verifies that the uart_init method returns false when:
    /// - The bit width parameter (bwpc) is less than the minimum required value of 3
    ///
    /// # Algorithm
    ///
    /// 1. Create a driver instance
    /// 2. Call uart_init with an invalid bit width parameter (less than 3)
    /// 3. Verify the function returns false
    ///
    /// # Notes
    ///
    /// * No mocks are needed for this test since we're only validating the parameter check
    /// * The function should return early without making any register modifications
    #[test]
    fn test_uart_init_invalid_bit_width() {
        // Create a new driver with default settings
        let mut driver = UartDriver::default();
        
        // Call uart_init with an invalid bit width (below minimum of 3)
        let result = driver.uart_init(69, 2, true, true, HardwareControl::NoControl);
        
        // Verify the function returns false for invalid parameters
        assert_eq!(result, false);
    }
    
    /// Tests the UART IRQ source and flag clearing functionality with no flags set.
    ///
    /// This test verifies that the get_and_clear_irq_source method correctly:
    /// - Reads the DMA ready register to get interrupt status
    /// - Clears the interrupt flags by writing back the same value
    /// - Returns zero when no flags are set
    ///
    /// # Algorithm
    ///
    /// 1. Mock register access functions with no flags set
    /// 2. Call the get_and_clear_irq_source method
    /// 3. Verify the correct register is read
    /// 4. Verify the same value is written back to clear flags
    /// 5. Verify the returned value is zero
    #[test]
    #[mry::lock(read_reg_dma_rx_rdy0, write_reg_dma_rx_rdy0)]
    fn test_get_and_clear_irq_source_no_flags() {
        // No interrupt flags set
        mock_read_reg_dma_rx_rdy0().returns(0);
        mock_write_reg_dma_rx_rdy0(0).returns(());
        
        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, 0);
        
        mock_read_reg_dma_rx_rdy0().assert_called(1);
        mock_write_reg_dma_rx_rdy0(0).assert_called(1);
    }
    
    /// Tests the UART IRQ source and flag clearing functionality with RX flag set.
    ///
    /// This test verifies that the get_and_clear_irq_source method correctly:
    /// - Reads the DMA ready register with RX flag set
    /// - Clears the interrupt flags by writing back the same value
    /// - Returns the RX flag value
    ///
    /// # Algorithm
    ///
    /// 1. Mock register access functions with RX flag set
    /// 2. Call the get_and_clear_irq_source method
    /// 3. Verify the correct register is read
    /// 4. Verify the same value is written back to clear flags
    /// 5. Verify the returned value matches the RX flag
    #[test]
    #[mry::lock(read_reg_dma_rx_rdy0, write_reg_dma_rx_rdy0)]
    fn test_get_and_clear_irq_source_rx_flag() {
        // RX interrupt flag set
        mock_read_reg_dma_rx_rdy0().returns(UartIrqMask::Rx.bits());
        mock_write_reg_dma_rx_rdy0(UartIrqMask::Rx.bits()).returns(());
        
        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, UartIrqMask::Rx.bits());
        
        mock_read_reg_dma_rx_rdy0().assert_called(1);
        mock_write_reg_dma_rx_rdy0(UartIrqMask::Rx.bits()).assert_called(1);
    }
    
    /// Tests the UART IRQ source and flag clearing functionality with TX flag set.
    ///
    /// This test verifies that the get_and_clear_irq_source method correctly:
    /// - Reads the DMA ready register with TX flag set
    /// - Clears the interrupt flags by writing back the same value
    /// - Returns the TX flag value
    ///
    /// # Algorithm
    ///
    /// 1. Mock register access functions with TX flag set
    /// 2. Call the get_and_clear_irq_source method
    /// 3. Verify the correct register is read
    /// 4. Verify the same value is written back to clear flags
    /// 5. Verify the returned value matches the TX flag
    #[test]
    #[mry::lock(read_reg_dma_rx_rdy0, write_reg_dma_rx_rdy0)]
    fn test_get_and_clear_irq_source_tx_flag() {
        // TX interrupt flag set
        mock_read_reg_dma_rx_rdy0().returns(UartIrqMask::Tx.bits());
        mock_write_reg_dma_rx_rdy0(UartIrqMask::Tx.bits()).returns(());
        
        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, UartIrqMask::Tx.bits());
        
        mock_read_reg_dma_rx_rdy0().assert_called(1);
        mock_write_reg_dma_rx_rdy0(UartIrqMask::Tx.bits()).assert_called(1);
    }
    
    /// Tests the UART IRQ source and flag clearing functionality with both RX and TX flags set.
    ///
    /// This test verifies that the get_and_clear_irq_source method correctly:
    /// - Reads the DMA ready register with both RX and TX flags set
    /// - Clears the interrupt flags by writing back the same value
    /// - Returns the combined flag value
    ///
    /// # Algorithm
    ///
    /// 1. Mock register access functions with both flags set
    /// 2. Call the get_and_clear_irq_source method
    /// 3. Verify the correct register is read
    /// 4. Verify the same value is written back to clear flags
    /// 5. Verify the returned value matches the combined flags
    #[test]
    #[mry::lock(read_reg_dma_rx_rdy0, write_reg_dma_rx_rdy0)]
    fn test_get_and_clear_irq_source_all_flags() {
        // Both RX and TX interrupt flags set
        mock_read_reg_dma_rx_rdy0().returns(UartIrqMask::all());
        mock_write_reg_dma_rx_rdy0(UartIrqMask::all()).returns(());
        
        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, UartIrqMask::all());
        
        mock_read_reg_dma_rx_rdy0().assert_called(1);
        mock_write_reg_dma_rx_rdy0(UartIrqMask::all()).assert_called(1);
    }
    
    /// Tests the UART IRQ source and flag clearing functionality with masking of irrelevant bits.
    ///
    /// This test verifies that the get_and_clear_irq_source method correctly:
    /// - Reads the DMA ready register with extra irrelevant bits set
    /// - Clears the interrupt flags by writing back the same value
    /// - Returns only the relevant UART bits (masking out irrelevant bits)
    ///
    /// # Algorithm
    ///
    /// 1. Mock register access functions with both relevant and irrelevant bits set
    /// 2. Call the get_and_clear_irq_source method
    /// 3. Verify the correct register is read
    /// 4. Verify the same value is written back to clear flags
    /// 5. Verify the returned value has irrelevant bits masked out
    #[test]
    #[mry::lock(read_reg_dma_rx_rdy0, write_reg_dma_rx_rdy0)]
    fn test_get_and_clear_irq_source_with_masking() {
        // Additional irrelevant bits set (should be masked out)
        let raw_value = 0xF0 | UartIrqMask::Rx.bits();
        mock_read_reg_dma_rx_rdy0().returns(raw_value);
        mock_write_reg_dma_rx_rdy0(raw_value).returns(());
        
        let result = UartDriver::get_and_clear_irq_source();
        assert_eq!(result, UartIrqMask::Rx.bits()); // Should only have RX bit, others masked
        
        mock_read_reg_dma_rx_rdy0().assert_called(1);
        mock_write_reg_dma_rx_rdy0(raw_value).assert_called(1);
    }
    
    /// Tests the TX busy flag manipulation methods.
    ///
    /// This test verifies that the set_tx_busy_flag and clear_tx_busy_flag methods:
    /// - Properly set the is_tx_busy flag to true or false
    /// - uart_tx_is_busy returns the current state of the flag
    ///
    /// # Notes
    ///
    /// * Simple state manipulation test
    #[test]
    fn test_tx_busy_flag_manipulation() {
        let mut driver = UartDriver::default();
        
        // Initial state should be not busy
        assert_eq!(driver.uart_tx_is_busy(), false);
        
        // Set busy flag
        driver.set_tx_busy_flag();
        assert_eq!(driver.uart_tx_is_busy(), true);
        
        // Clear busy flag
        driver.clear_tx_busy_flag();
        assert_eq!(driver.uart_tx_is_busy(), false);
    }
    
    /// Tests the synchronous UART data transmission function when the driver is not busy.
    ///
    /// This test verifies that the send method correctly:
    /// - Sets the busy flag
    /// - Copies message data to the internal TX buffer
    /// - Configures DMA with the buffer address
    /// - Triggers the DMA transfer
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register access functions
    /// 2. Create test message data
    /// 3. Call send with test message
    /// 4. Verify DMA configuration and trigger
    /// 5. Verify no waiting was performed
    ///
    /// # Notes
    ///
    /// * Tests the case where the driver is not busy
    /// * No waiting or watchdog kicks should occur
    #[test]
    #[mry::lock(
        clock_time, clock_time_exceed, wd_clear,
        write_reg_dma1_addr, write_reg_dma_tx_rdy0
    )]
    fn test_send_not_busy() {
        // Setup mock responses for timing functions
        mock_clock_time().returns(1000);
        mock_clock_time_exceed(1000, 10 * 1000).returns(false);
        mock_wd_clear().returns(());
        
        // Setup mock responses for DMA configuration
        mock_write_reg_dma1_addr(mry::Any).returns(());
        mock_write_reg_dma_tx_rdy0(mry::Any).returns(());
        
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
        let mut driver = UartDriver::default();
        let result = driver.send(&msg);
        
        // Verify result and busy flag
        assert_eq!(result, true);
        assert_eq!(driver.is_tx_busy, true);
        
        // Verify DMA configuration
        mock_write_reg_dma1_addr(mry::Any).assert_called(1);
        mock_write_reg_dma_tx_rdy0(mry::Any).assert_called(1);
        
        // No waiting should have happened
        mock_clock_time().assert_called(1);
        mock_clock_time_exceed(mry::Any, mry::Any).assert_called(0);
        mock_wd_clear().assert_called(0);
    }
    
    /// Tests the synchronous UART data transmission function when the driver is busy.
    ///
    /// This test verifies that the send method correctly:
    /// - Waits for any previous transmission to complete
    /// - Kicks the watchdog during the wait
    /// - Sets the busy flag after waiting
    /// - Copies message data to the internal TX buffer
    /// - Configures DMA with the buffer address
    /// - Triggers the DMA transfer
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register access and timing functions
    /// 2. Create test message data and a busy driver
    /// 3. Configure mocks to simulate the busy condition clearing after some time
    /// 4. Call send with test message
    /// 5. Verify DMA configuration and trigger
    /// 6. Verify proper waiting and watchdog handling
    ///
    /// # Notes
    ///
    /// * Tests the case where the driver is initially busy but becomes available
    /// * Verifies watchdog is kicked during the busy-wait
    #[test]
    #[mry::lock(
        clock_time, clock_time_exceed, wd_clear,
        write_reg_dma1_addr, write_reg_dma_tx_rdy0
    )]
    fn test_send_busy() {
        // Setup mock responses for timing functions
        mock_clock_time().returns(2000);
        
        // Configure mock to return false on first call and true on subsequent calls
        // This simulates the busy flag becoming clear after waiting
        let mut call_count = 0;
        mock_clock_time_exceed(2000, 10 * 1000).returns_with(move |_, _| {
            call_count += 1;
            call_count > 1
        });
        
        mock_wd_clear().returns(());
        
        // Setup mock responses for DMA configuration
        mock_write_reg_dma1_addr(mry::Any).returns(());
        mock_write_reg_dma_tx_rdy0(mry::Any).returns(());
        
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
        let mut driver = UartDriver::default();
        driver.is_tx_busy = true;
        
        let result = driver.send(&msg);
        
        // Verify result and final busy state
        assert_eq!(result, true);
        assert_eq!(driver.is_tx_busy, true);
        
        // Verify DMA was configured
        mock_write_reg_dma1_addr(mry::Any).assert_called(1);
        mock_write_reg_dma_tx_rdy0(mry::Any).assert_called(1);
        
        // Verify waiting occurred
        mock_clock_time().assert_called(1);
        mock_clock_time_exceed(mry::Any, mry::Any).assert_called(2);
        mock_wd_clear().assert_called(1);
    }
    
    /// Tests the asynchronous UART data transmission function when the driver is not busy.
    ///
    /// This test verifies that the send_async method correctly:
    /// - Sets the busy flag
    /// - Copies message data to the internal TX buffer
    /// - Configures DMA with the buffer address
    /// - Triggers the DMA transfer
    /// - Returns without yielding to the async runtime
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register access functions
    /// 2. Create test message data
    /// 3. Call send_async with test message
    /// 4. Verify DMA configuration and trigger
    /// 5. Verify no yielding occurred
    ///
    /// # Notes
    ///
    /// * Tests the case where the driver is not busy
    /// * No yielding should occur
    #[test]
    #[mry::lock(
        clock_time, clock_time_exceed, yield_now,
        write_reg_dma1_addr, write_reg_dma_tx_rdy0
    )]
    fn test_send_async_not_busy() {
        // Setup mock responses for timing functions
        mock_clock_time().returns(1000);
        mock_clock_time_exceed(1000, 100 * 1000).returns(false);
        mock_yield_now().returns(());
        
        // Setup mock responses for DMA configuration
        mock_write_reg_dma1_addr(mry::Any).returns(());
        mock_write_reg_dma_tx_rdy0(mry::Any).returns(());
        
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
        let mut driver = UartDriver::default();
        let result = futures::executor::block_on(driver.send_async(&msg));
        
        // Verify result and busy flag
        assert_eq!(result, true);
        assert_eq!(driver.is_tx_busy, true);
        
        // Verify DMA configuration
        mock_write_reg_dma1_addr(mry::Any).assert_called(1);
        mock_write_reg_dma_tx_rdy0(mry::Any).assert_called(1);
        
        // No yielding should have happened
        mock_clock_time().assert_called(1);
        mock_clock_time_exceed(mry::Any, mry::Any).assert_called(0);
        mock_yield_now().assert_called(0);
    }
    
    /// Tests the asynchronous UART data transmission function when the driver is initially busy.
    ///
    /// This test verifies that the send_async method correctly:
    /// - Waits for any previous transmission to complete
    /// - Yields to the async runtime while waiting
    /// - Sets the busy flag after waiting
    /// - Copies message data to the internal TX buffer
    /// - Configures DMA with the buffer address
    /// - Triggers the DMA transfer
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register access and timing functions
    /// 2. Create test message data and a busy driver
    /// 3. Configure mocks to simulate the busy condition clearing after yielding
    /// 4. Call send_async with test message
    /// 5. Verify DMA configuration and trigger
    /// 6. Verify proper yielding during waiting
    ///
    /// # Notes
    ///
    /// * Tests the case where the driver is initially busy but becomes available
    /// * Verifies yield_now is called during the busy-wait
    #[test]
    #[mry::lock(
        clock_time, clock_time_exceed, yield_now,
        write_reg_dma1_addr, write_reg_dma_tx_rdy0
    )]
    fn test_send_async_busy() {
        // Setup mock responses for timing functions
        mock_clock_time().returns(2000);
        
        // Configure mock to return false on first call and true on subsequent calls
        // This simulates the busy flag becoming clear after waiting
        let mut call_count = 0;
        mock_clock_time_exceed(2000, 100 * 1000).returns_with(move |_, _| {
            call_count += 1;
            call_count > 1
        });
        
        mock_yield_now().returns(());
        
        // Setup mock responses for DMA configuration
        mock_write_reg_dma1_addr(mry::Any).returns(());
        mock_write_reg_dma_tx_rdy0(mry::Any).returns(());
        
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
        let mut driver = UartDriver::default();
        driver.is_tx_busy = true;
        
        let result = futures::executor::block_on(driver.send_async(&msg));
        
        // Verify result and final busy state
        assert_eq!(result, true);
        assert_eq!(driver.is_tx_busy, true);
        
        // Verify DMA was configured
        mock_write_reg_dma1_addr(mry::Any).assert_called(1);
        mock_write_reg_dma_tx_rdy0(mry::Any).assert_called(1);
        
        // Verify yielding occurred
        mock_clock_time().assert_called(1);
        mock_clock_time_exceed(mry::Any, mry::Any).assert_called(2);
        mock_yield_now().assert_called(1);
    }
    
    /// Tests the DMA buffer initialization for UART reception.
    ///
    /// This test verifies that the uart_buff_init method correctly:
    /// - Calculates the proper buffer size based on UartData structure
    /// - Configures DMA0 with the address of the RX buffer
    /// - Sets the DMA0 control register with the correct buffer size
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for DMA register functions
    /// 2. Create a driver instance and call uart_buff_init
    /// 3. Verify DMA0 was configured with the correct buffer address and size
    #[test]
    #[mry::lock(write_reg_dma0_addr, write_reg_dma0_ctrl)]
    fn test_uart_buff_init() {
        // Setup mock responses for DMA configuration
        mock_write_reg_dma0_addr(mry::Any).returns(());
        mock_write_reg_dma0_ctrl(mry::Any).returns(());
        
        // Create a driver and call buffer initialization
        let mut driver = UartDriver::default();
        driver.init_buffers();
        
        // Calculate expected buffer size
        let expected_buf_len = size_of::<UartData>() / 16;
        
        // Verify DMA was configured correctly
        mock_write_reg_dma0_addr(mry::Any).assert_called(1);
        mock_write_reg_dma0_ctrl(expected_buf_len as u16).assert_called(1);
    }
    
    /// Tests the UART error clearing functionality when no error is present.
    ///
    /// This test verifies that the clear_error method correctly:
    /// - Reads the UART status register
    /// - Detects that no error is present (bit 7 not set)
    /// - Does not attempt to clear anything
    /// - Returns false to indicate no error was detected
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register access with no error flag set
    /// 2. Call clear_error method
    /// 3. Verify the status register was read
    /// 4. Verify no write was performed
    /// 5. Verify the method returned false
    #[test]
    #[mry::lock(read_reg_uart_status, write_reg_uart_status)]
    fn test_clear_error_no_error() {
        // No error present (bit 7 not set)
        mock_read_reg_uart_status().returns(0x00);
        
        let result = UartDriver::clear_error();
        
        // Should return false when no error is detected
        assert_eq!(result, false);
        
        // Verify interactions
        mock_read_reg_uart_status().assert_called(1);
        mock_write_reg_uart_status(mry::Any).assert_called(0); // No write should occur
    }
    
    /// Tests the UART error clearing functionality when a basic error is present.
    ///
    /// This test verifies that the clear_error method correctly:
    /// - Reads the UART status register
    /// - Detects error condition (bit 7 set)
    /// - Clears the error by setting bit 6
    /// - Returns true to indicate an error was cleared
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register access with error flag set
    /// 2. Call clear_error method
    /// 3. Verify the status register was read
    /// 4. Verify the correct value was written back (with bit 6 set)
    /// 5. Verify the method returned true
    #[test]
    #[mry::lock(read_reg_uart_status, write_reg_uart_status)]
    fn test_clear_error_basic_error() {
        // Error present (bit 7 set)
        mock_read_reg_uart_status().returns(FLD_UART_STATUS::ERR_FLAG.bits());
        mock_write_reg_uart_status(FLD_UART_STATUS::ERR_FLAG.bits() | FLD_UART_STATUS::ERR_CLR.bits()).returns(());
        
        let result = UartDriver::clear_error();
        
        // Should return true when an error is cleared
        assert_eq!(result, true);
        
        // Verify interactions
        mock_read_reg_uart_status().assert_called(1);
        mock_write_reg_uart_status(FLD_UART_STATUS::ERR_FLAG.bits() | FLD_UART_STATUS::ERR_CLR.bits()).assert_called(1); // Should write with bit 6 set
    }
    
    /// Tests the UART error clearing functionality when an error with other flags is present.
    ///
    /// This test verifies that the clear_error method correctly:
    /// - Reads the UART status register
    /// - Detects error condition (bit 7 set) among other flag bits
    /// - Clears the error by setting bit 6 while preserving other bits
    /// - Returns true to indicate an error was cleared
    ///
    /// # Algorithm
    ///
    /// 1. Setup mock responses for register access with error flag and other bits set
    /// 2. Call clear_error method
    /// 3. Verify the status register was read
    /// 4. Verify the correct value was written back (with bit 6 set and other bits preserved)
    /// 5. Verify the method returned true
    #[test]
    #[mry::lock(read_reg_uart_status, write_reg_uart_status)]
    fn test_clear_error_with_other_bits() {
        // Error with other bits set (bits 0, 2 and 7 set)
        let status_with_error = FLD_UART_STATUS::ERR_FLAG.bits() | 0x05; // 0x05 for bits 0 and 2
        mock_read_reg_uart_status().returns(status_with_error);
        mock_write_reg_uart_status(status_with_error | FLD_UART_STATUS::ERR_CLR.bits()).returns(());
        
        let result = UartDriver::clear_error();
        
        // Should return true when an error is cleared
        assert_eq!(result, true);
        
        // Verify interactions
        mock_read_reg_uart_status().assert_called(1);
        mock_write_reg_uart_status(status_with_error | FLD_UART_STATUS::ERR_CLR.bits()).assert_called(1); // Should preserve other bits
    }
    
    /// Tests the UART initialization function with 9600 baud rate configuration.
    ///
    /// This test verifies that the uart_init method correctly:
    /// - Configures registers for a low baud rate (9600)
    /// - Sets the appropriate timeout value based on bit width
    /// - Sets the lower safety margin (1 instead of 3)
    ///
    /// # Algorithm
    ///
    /// 1. Mock GPIO and register access functions
    /// 2. Call uart_init with parameters for 9600 baud (clkdiv=237, bwpc=13)
    /// 3. Verify all expected hardware configuration calls
    /// 4. Check that timeout and safety margin are configured for low baud rate
    ///
    /// # Notes
    ///
    /// * Validates the specific timing configuration for baud rates below 100000
    /// * Verifies the timeout is calculated as (bwpc+1)*12
    /// * Verifies safety margin is set to 1
    #[test]
    #[mry::lock(
        read_reg_dma_rx_rdy0, write_reg_dma_rx_rdy0,
        read_reg_dma_chn_irq_msk, write_reg_dma_chn_irq_msk,
        read_reg_irq_mask, write_reg_irq_mask,
        read_reg_dma_chn_en, write_reg_dma_chn_en, 
        write_reg_uart_clk_div, write_reg_uart_ctrl0,
        write_reg_uart_rx_timeout, write_reg_uart_rx_timeout_cnt,
        write_reg_uart_ctrl1, write_reg_dma0_mode, write_reg_dma1_mode
    )]
    fn test_uart_init_9600_baud() {
        // Setup mock responses for UART register configuration
        mock_write_reg_uart_clk_div(237 | FLD_UART_CLK_DIV::CLK_EN.bits()).returns(());  // Clock divider for 9600 baud
        mock_write_reg_uart_ctrl0(FLD_UART_CTRL0::RX_DMA_EN.bits() | FLD_UART_CTRL0::TX_DMA_EN.bits() | 13).returns(());  // Bit width (13) with DMA enable
        
        // For 9600 baud, timeout should be (bwpc+1)*12 = (13+1)*12 = 168
        mock_write_reg_uart_rx_timeout(168).returns(());           // Timeout for low baud rate
        mock_write_reg_uart_rx_timeout_cnt(1).returns(());         // Lower safety margin (1)
        
        mock_write_reg_uart_ctrl1(0x00).returns(());               // Hardware control (NOCONTROL)
        mock_write_reg_dma0_mode(0x01).returns(());                // DMA0 mode (receive)
        mock_write_reg_dma1_mode(0x00).returns(());                // DMA1 mode (transmit)
        
        // Setup mock responses for interrupt clearing and configuration
        mock_read_reg_dma_rx_rdy0().returns(0);
        mock_write_reg_dma_rx_rdy0(0).returns(());
        
        mock_read_reg_dma_chn_irq_msk().returns(0);
        mock_write_reg_dma_chn_irq_msk(mry::Any).returns(());
        mock_read_reg_irq_mask().returns(0);
        mock_write_reg_irq_mask(mry::Any).returns(());
        mock_read_reg_dma_chn_en().returns(0);
        mock_write_reg_dma_chn_en(mry::Any).returns(());
        
        // Create a new driver and initialize UART with 9600 baud parameters
        // For 32MHz system clock, 9600 baud: clkdiv=237, bwpc=13
        let mut driver = UartDriver::default();
        let result = driver.uart_init(237, 13, true, true, HardwareControl::NoControl);
        
        // Verify initialization succeeded
        assert_eq!(result, true);
        
        // Verify UART register configuration
        mock_write_reg_uart_clk_div(237 | FLD_UART_CLK_DIV::CLK_EN.bits()).assert_called(1);  // Clock divider
        mock_write_reg_uart_ctrl0(FLD_UART_CTRL0::RX_DMA_EN.bits() | FLD_UART_CTRL0::TX_DMA_EN.bits() | 13).assert_called(1);  // Bit width with DMA enable
        
        // Verify specific low baud rate configuration
        mock_write_reg_uart_rx_timeout(168).assert_called(1);  // Timeout: (13+1)*12 = 168
        mock_write_reg_uart_rx_timeout_cnt(1).assert_called(1);    // Lower safety margin
        
        // Verify hardware control setting
        mock_write_reg_uart_ctrl1(0x00).assert_called(1);
        
        // Verify DMA configuration
        mock_write_reg_dma0_mode(0x01).assert_called(1);  // DMA0 mode
        mock_write_reg_dma1_mode(0x00).assert_called(1);  // DMA1 mode
        
        // Verify interrupt configuration
        mock_read_reg_dma_rx_rdy0().assert_called(1);
        mock_write_reg_dma_rx_rdy0(0).assert_called(1);
        mock_read_reg_dma_chn_irq_msk().assert_called(2);
        mock_write_reg_dma_chn_irq_msk(mry::Any).assert_called(2);
        mock_read_reg_irq_mask().assert_called(2);
        mock_write_reg_irq_mask(mry::Any).assert_called(2);
        mock_read_reg_dma_chn_en().assert_called(2);
        mock_write_reg_dma_chn_en(mry::Any).assert_called(2);
    }
}
