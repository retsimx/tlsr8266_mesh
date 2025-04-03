extern crate core;
extern crate paste;

use bitflags::bitflags;
use crate::{BIT, BIT_MASK_LEN, BIT_RNG};

//****************************************************
// TLSR8266 Register Documentation
//****************************************************
//
// This file defines the memory-mapped registers for the TLSR8266 SoC and provides
// safe read/write functions to interact with them.
//
// Register Map Overview:
// - All registers are memory-mapped starting at base address 0x800000
// - Registers are grouped functionally (GPIO, RF, DMA, etc.)
// - Access can be 8-bit, 16-bit, or 32-bit depending on the register
//
// Naming Convention:
// - reg_*: Register address definitions
// - read_*: Functions to read register values
// - write_*: Functions to write to registers
// - FLD_*: Enum defining bit fields within registers
//
// Usage Examples:
//
// 1. Basic register read/write:
//    ```
//    // Read the current RF channel
//    let channel = read_reg_rf_channel();
//    
//    // Set RF channel to 2
//    write_reg_rf_channel(2);
//    ```
//
// 2. Using bit fields:
//    ```
//    // Enable the AES module
//    let mut clk = read_reg_clk_en();
//    clk |= FLD_CLK_EN::AES_EN as u16;
//    write_reg_clk_en(clk);
//    ```
//
// 3. Working with GPIO:
//    ```
//    // Set pin PA0 as output
//    let mut oen = read_reg_gpio_pa_oen();
//    oen &= !(0x01);  // Clear bit 0 (0 = output)
//    write_reg_gpio_pa_oen(oen);
//    
//    // Set PA0 high
//    let mut out = read_reg_gpio_pa_out();
//    out |= 0x01;
//    write_reg_gpio_pa_out(out);
//    ```

/// Base address for all memory-mapped registers
pub const REG_BASE_ADDR: u32 = 0x800000;

//****************************************************
// Core read/write register functions
//****************************************************

/// Writes an 8-bit value to a register
/// 
/// # Arguments
/// * `addr` - Register address (without base address)
/// * `v` - 8-bit value to write
/// 
/// # Examples
/// ```
/// // Set the RF channel to 5
/// write_reg8(0x40d, 5);
/// ```
#[cfg_attr(test, mry::mry)]
pub fn write_reg8(addr: u32, v: u8) {
    unsafe { core::ptr::write_volatile((addr | REG_BASE_ADDR) as *mut u8, v) }
}

/// Writes a 16-bit value to a register
/// 
/// # Arguments
/// * `addr` - Register address (without base address)
/// * `v` - 16-bit value to write
#[cfg_attr(test, mry::mry)]
pub fn write_reg16(addr: u32, v: u16) {
    unsafe { core::ptr::write_volatile((addr | REG_BASE_ADDR) as *mut u16, v) }
}

/// Writes a 32-bit value to a register
/// 
/// # Arguments
/// * `addr` - Register address (without base address)
/// * `v` - 32-bit value to write
#[cfg_attr(test, mry::mry)]
pub fn write_reg32(addr: u32, v: u32) {
    unsafe { core::ptr::write_volatile((addr | REG_BASE_ADDR) as *mut u32, v) }
}

/// Reads an 8-bit value from a register
/// 
/// # Arguments
/// * `addr` - Register address (without base address)
/// 
/// # Returns
/// The 8-bit value read from the register
/// 
/// # Examples
/// ```
/// // Read the current RF channel
/// let channel = read_reg8(0x40d);
/// ```
#[cfg_attr(test, mry::mry)]
pub fn read_reg8(addr: u32) -> u8 {
    return unsafe { core::ptr::read_volatile((addr | REG_BASE_ADDR) as *mut u8) };
}

/// Reads a 16-bit value from a register
/// 
/// # Arguments
/// * `addr` - Register address (without base address)
/// 
/// # Returns
/// The 16-bit value read from the register
#[cfg_attr(test, mry::mry)]
pub fn read_reg16(addr: u32) -> u16 {
    unsafe { core::ptr::read_volatile((addr | REG_BASE_ADDR) as *mut u16) }
}

/// Reads a 32-bit value from a register
/// 
/// # Arguments
/// * `addr` - Register address (without base address)
/// 
/// # Returns
/// The 32-bit value read from the register
#[cfg_attr(test, mry::mry)]
pub fn read_reg32(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile((addr | REG_BASE_ADDR) as *mut u32) }
}

//****************************************************
// Register access macros
//****************************************************

#[macro_export]
macro_rules! regrw_idx {
    ( $x:ident, $a:expr, $s:literal ) => {
        paste::paste! {
            #[cfg_attr(test, mry::mry)]
            pub fn [<read_ $x>](i: u32) -> [<u $s>] {
                return [<read_reg $s>]($a + i);
            }

            #[cfg_attr(test, mry::mry)]
            pub fn [<write_ $x>](value: [<u $s>], i: u32) {
                [<write_reg $s>]($a + i, value);
            }
        }
    };
}

#[macro_export]
macro_rules! regrw {
    ( $x:ident, $a:expr, $s:expr ) => {
        paste::paste! {
            #[cfg_attr(test, mry::mry)]
            pub fn [<read_ $x>]() -> [<u $s>] {
                return [<read_reg $s>]($a);
            }

            #[cfg_attr(test, mry::mry)]
            pub fn [<write_ $x>](value: [<u $s>]) {
                [<write_reg $s>]($a, value);
            }
        }
    };
}

#[macro_export]
macro_rules! regrw_copy {
    ( $x:ident, $y:ident, $t:expr ) => {
        paste::paste! {
            #[cfg_attr(test, mry::mry)]
            pub fn [<read_ $x>]() -> [<u $t>] {
                [<read_ $y>]()
            }

            #[cfg_attr(test, mry::mry)]
            pub fn [<write_ $x>](value: [<u $t>]) {
                [<write_ $y>](value)
            }
        }
    };
}

//****************************************************
// Master SPI registers (0x0C - 0x0D)
//****************************************************
//
// This section defines registers that control the Master SPI (Serial Peripheral Interface)
// communication module, which allows the TLSR8266 to communicate with external SPI devices.
//
// Key Features:
// - Configurable clock polarity and phase
// - Support for standard SPI mode (master mode)
// - Chip select control
// - 8-bit data register for transmit/receive operations
// - Busy status flag for transaction timing
//
// Key Registers:
// - reg_master_spi_data: 8-bit data register for transmitting/receiving data
// - reg_master_spi_ctrl: Control register for SPI operation
//
// Usage Examples:
//
// 1. Basic SPI transfer operation:
//    ```
//    // Ensure CS is high initially (inactive)
//    mspi_high();
//    
//    // Pull CS low to begin transaction
//    mspi_low();
//    
//    // Write a byte and wait for completion
//    mspi_write(0x55);  // Example command byte
//    mspi_wait();       // Wait for transfer to complete
//    
//    // Read response byte
//    let response = mspi_read();
//    
//    // Release CS to end transaction
//    mspi_high();
//    ```
//
// 2. Controlling SPI chip select manually:
//    ```
//    // Set CS high (inactive)
//    let mut ctrl = read_reg_master_spi_ctrl();
//    ctrl |= FLD_MASTER_SPI::CS as u8;
//    write_reg_master_spi_ctrl(ctrl);
//    
//    // Set CS low (active)
//    ctrl &= !(FLD_MASTER_SPI::CS as u8);
//    write_reg_master_spi_ctrl(ctrl);
//    ```
//
// 3. Reading SPI busy status:
//    ```
//    // Check if SPI is busy
//    if (read_reg_master_spi_ctrl() & (FLD_MASTER_SPI::BUSY as u8)) != 0 {
//        // SPI is currently busy with a transfer
//    } else {
//        // SPI is ready for a new transfer
//    }
//    ```

regrw!(reg_master_spi_data, 0x0c, 8);
regrw!(reg_master_spi_ctrl, 0x0d, 8);

pub enum FLD_MASTER_SPI {
    CS = BIT!(0),
    SDO = BIT!(1),
    CONT = BIT!(2),
    RD = BIT!(3),
    BUSY = BIT!(4),
}

//****************************************************
// System reset and clock control registers (0x60 - 0x7F)
//****************************************************
//
// This section defines registers that control system resets and clock configurations
// of the TLSR8266. These registers are critical for managing power states, module
// functionality, and system initialization.
//
// Key Features:
// - Individual module reset control (soft resets)
// - Module clock gating for power saving
// - System clock source selection and dividers
// - Sleep/wakeup control registers
//
// Important Register Groups:
// - Reset registers (0x60-0x62): Control individual module resets
// - Clock enable registers (0x64-0x65): Enable/disable clocks to modules
// - Clock selection register (0x66): Select source and divider for system clock
// - Power management registers (0x6E-0x7F): Sleep and wakeup control
//
// Usage Examples:
//
// 1. Resetting a specific module (e.g., AES module):
//    ```
//    // Reset the AES module
//    let mut reset = read_reg_rst0_16();
//    reset |= FLD_RST::AES as u16;  // Set the AES reset bit
//    write_reg_rst0_16(reset);
//    
//    // Clear the reset bit to allow normal operation
//    reset &= !(FLD_RST::AES as u16);
//    write_reg_rst0_16(reset);
//    ```
//
// 2. Enabling/disabling a module clock (e.g., AES module):
//    ```
//    // Enable the AES module clock
//    let mut clk_en = read_reg_clk_en();
//    clk_en |= FLD_CLK_EN::AES_EN as u16;
//    write_reg_clk_en(clk_en);
//    
//    // Disable the AES module clock for power saving
//    clk_en &= !(FLD_CLK_EN::AES_EN as u16);
//    write_reg_clk_en(clk_en);
//    ```
//
// 3. Selecting system clock source:
//    ```
//    // Select 32MHz oscillator as clock source with divider of 1
//    let mut clk_sel = read_reg_clk_sel();
//    clk_sel = (clk_sel & 0xE0) | FHS_SEL::SEL_32M_OSC as u8;  // Set source
//    clk_sel = (clk_sel & 0x1F) | (0 << 0);  // Set divider to 0 (div by 1)
//    write_reg_clk_sel(clk_sel);
//    ```
//
// 4. Entering deep sleep mode:
//    ```
//    // Configure wakeup source (e.g., GPIO)
//    let mut wakeup = read_reg_wakeup_en();
//    wakeup |= FLD_WAKEUP_SRC::GPIO as u8;
//    write_reg_wakeup_en(wakeup);
//    
//    // Enter deep sleep mode
//    write_reg_pwdn_ctrl(FldPwdnCtrl::Sleep as u8);
//    ```

regrw!(reg_rst0, 0x60, 8);
regrw!(reg_rst0_16, 0x60, 16);
regrw!(reg_rst1, 0x61, 8);
regrw!(reg_rst2, 0x62, 8);
regrw!(reg_rst_clk0, 0x60, 32);

pub enum FLD_RST {
    SPI = BIT!(0),
    I2C = BIT!(1),
    USB = BIT!(2),
    USB_PHY = BIT!(3),
    MCU = BIT!(4),
    MAC = BIT!(5),
    AIF = BIT!(6),
    BB = BIT!(7),
    GPIO = BIT!(8),
    ALGM = BIT!(9),
    DMA = BIT!(10),
    UART = BIT!(11),
    PWM = BIT!(12),
    AES = BIT!(13),
    SWR_M = BIT!(14),
    SWR_S = BIT!(15),
    SBC = BIT!(16),
    AUD = BIT!(17),
    DFIFO = BIT!(18),
    ADC = BIT!(19),
    SOFT_MCU = BIT!(20),
    MCIC = BIT!(21),
    SOFT_MCIC = BIT!(22),
    RSV = BIT!(23),
}

impl FLD_RST {
    pub const ZB: FLD_RST = FLD_RST::BB;
}

regrw!(reg_clk_en, 0x64, 16);
regrw!(reg_clk_en1, 0x64, 8);
regrw!(reg_clk_en2, 0x65, 8);

#[repr(usize)]
pub enum FLD_CLK_EN {
    GPIO_EN = BIT!(0),
    ALGM_EN = BIT!(1),
    DMA_EN = BIT!(2),
    UART_EN = BIT!(3),
    PWM_EN = BIT!(4),
    AES_EN = BIT!(5),
    PLL_EN = BIT!(6),
    SWIRE_EN = BIT!(7),
    SBC_EN = BIT!(8),
    AUD_EN = BIT!(9),
    DIFIO_EN = BIT!(10),
    I2S = BIT_RNG!(11, 12),
    C32K = BIT_RNG!(13, 15),
    SPI_EN = BIT!(24),
    I2C_EN = BIT!(25),
    USB_EN = BIT!(26),
    USB_PHY_EN = BIT!(27),
    MCU_EN = BIT!(28),
    MAC_EN = BIT!(29),
    ADC_EN = BIT!(30), // ADC interface
    ZB_EN = BIT!(31),
}

pub enum FLD_CLK2_EN {
    SBC_EN = BIT!(0),
    AUD_EN = BIT!(1),
    DIFIO_EN = BIT!(2),
    I2S = BIT_RNG!(3, 4),
    C32K = BIT_RNG!(5, 7),
}

regrw!(reg_clk_sel, 0x66, 8);

pub enum FLD_CLK_SEL {
    DIV = BIT_RNG!(0, 4),
    SRC = BIT_RNG!(5, 7),
}

regrw!(reg_i2s_step, 0x67, 8);
pub enum FLD_I2S_STEP {
    STEP = BIT_RNG!(0, 6),
    CLK_EN = BIT!(7),
}

regrw!(reg_i2s_mod, 0x68, 8);

//****************************************************
// ADC control registers (0x69 - 0x6D)
//****************************************************
//
// This section defines registers that control the Analog-to-Digital Converter (ADC)
// functionality of the TLSR8266 SoC, used for sampling analog signals.
//
// Key Features:
// - Configurable sampling rate and resolution
// - Multiple input channel selection
// - Built-in audio processing capabilities
// - Power management for ADC components
// - Integration with DMA for continuous sampling
//
// Key Registers:
// - reg_adc_step_l/h: Controls ADC clock divider/step size
// - reg_adc_mod: Sets ADC operating mode and configuration
// - reg_adc_clk_en: Enables ADC clock
// - reg_adc_mod_h: Higher bits for ADC mode setting
//
// Usage Examples:
//
// 1. Setting up ADC for basic analog sampling:
//    ```
//    // Configure ADC clock divider for appropriate sampling rate
//    write_reg_adc_step_l(32);  // Set divider value
//    
//    // Configure ADC mode and enable clock
//    let mut adc_mod = 0;
//    adc_mod |= 0x300;  // Set appropriate mode bits
//    adc_mod |= FLD_ADC_MOD::CLK_EN as u16;  // Enable ADC clock
//    write_reg_adc_mod(adc_mod);
//    
//    // Now the ADC is ready to sample analog inputs
//    ```
//
// 2. Enabling ADC module in system clock:
//    ```
//    // Enable ADC clock in system clock control
//    let mut clk_en = read_reg_clk_en();
//    clk_en |= FLD_CLK_EN::ADC_EN as u16;
//    write_reg_clk_en(clk_en);
//    ```
//
// 3. Configuring digital microphone (DMIC) interface:
//    ```
//    // Set DMIC step size and enable clock
//    let mut dmic_step = (48 & 0x7F);  // Set divider value
//    dmic_step |= FLD_DMIC_STEP::CLK_EN as u8;  // Enable DMIC clock
//    write_reg_dmic_step(dmic_step);
//    
//    // Configure DMIC mode
//    write_reg_dmic_mod(0x15);  // Example mode configuration
//    ```

regrw!(reg_adc_step_l, 0x69, 8);
regrw!(reg_adc_mod_l, 0x6a, 8);
regrw!(reg_adc_mod, 0x6a, 16);
pub enum FLD_ADC_MOD {
    MOD = BIT_RNG!(0, 11),
    STEP_H = BIT_RNG!(12, 14),
    CLK_EN = BIT!(15),
}

regrw!(reg_adc_clk_en, 0x6b, 8);
regrw!(reg_adc_mod_h, 0x6b, 8);
pub enum FLD_ADC_MOD_H {
    H = BIT_RNG!(0, 3),
    H_STEP = BIT_RNG!(4, 6),
    H_CLK = BIT!(7),
}

regrw!(reg_dmic_step, 0x6c, 8);
pub enum FLD_DMIC_STEP {
    STEP = BIT_RNG!(0, 6),
    CLK_EN = BIT!(7),
}
regrw!(reg_dmic_mod, 0x6d, 8);

//****************************************************
// Power Management registers (0x6E - 0x7F)
//****************************************************
//
// This section defines registers that control the power management functionality
// of the TLSR8266 SoC, including sleep modes, wakeup sources, and power states.
//
// Key Features:
// - Multiple sleep modes (deep sleep, suspend)
// - Configurable wakeup sources (GPIO, timer, comparator)
// - Power control for individual peripherals
// - Battery monitoring and voltage detection
// - Wake-up edge/level detection for GPIO pins
//
// Key Registers:
// - reg_wakeup_en: Enables different wakeup sources
// - reg_pwdn_ctrl: Controls entering sleep/powerdown modes
// - reg_fhs_sel: Clock source selection during wakeup
// - rega_deepsleep_flag: Flag indicating reset from deep sleep
// - rega_wakeup_en_val0-2: GPIO wakeup configuration
// - raga_gpio_wkup_pol: Wakeup edge polarity settings
//
// Usage Examples:
//
// 1. Entering deep sleep mode with GPIO wakeup:
//    ```
//    // Enable GPIO as wakeup source
//    let mut wakeup = read_reg_wakeup_en();
//    wakeup |= FLD_WAKEUP_SRC::GPIO as u8;
//    write_reg_wakeup_en(wakeup);
//    
//    // Configure PA0 as wakeup pin (using analog register writes)
//    analog_write(rega_wakeup_en_val0, 0x01);  // Bit 0 for PA0
//    
//    // Set wakeup on rising edge
//    analog_write(raga_gpio_wkup_pol, 0x00);  // 0 for rising edge
//    
//    // Enter deep sleep mode
//    write_reg_pwdn_ctrl(FldPwdnCtrl::Sleep as u8);
//    ```
//
// 2. Checking if device has awakened from deep sleep:
//    ```
//    // Read the deep sleep flag from analog register
//    let sleep_flag = analog_read(rega_deepsleep_flag);
//    
//    if (sleep_flag & 0x40) != 0 {
//        // Device woke from deep sleep
//        // Restore previous settings
//        
//        // Clear the flag
//        analog_write(rega_deepsleep_flag, sleep_flag & !0x40);
//    } else {
//        // Normal power-on reset
//        // Perform initialization
//    }
//    ```
//
// 3. Setting up timer as wakeup source:
//    ```
//    // Configure system tick for wakeup
//    write_reg_system_wakeup_tick(read_reg_system_tick() + 32000);  // Wake after 1ms @ 32MHz
//    
//    // Enable timer as wakeup source
//    let mut wakeup = read_reg_wakeup_en();
//    wakeup |= PM_WAKEUP::TIMER as u8;
//    write_reg_wakeup_en(wakeup);
//    
//    // Enter sleep mode
//    write_reg_pwdn_ctrl(FldPwdnCtrl::Sleep as u8);
//    ```

regrw!(reg_wakeup_en, 0x6e, 8);
pub enum FLD_WAKEUP_SRC {
    I2C = BIT!(0),
    SPI = BIT!(1),
    USB = BIT!(2),
    GPIO = BIT!(3),
    I2C_SYN = BIT!(4),
    GPIO_RM = BIT!(5),
    USB_RESM = BIT!(6),
    RST_SYS = BIT!(7),
}

regrw!(reg_pwdn_ctrl, 0x6f, 8);
pub enum FldPwdnCtrl {
    Reboot = BIT!(5),
    Sleep = BIT!(7),
}

regrw!(reg_fhs_sel, 0x70, 8);
pub enum FLD_FHS_SELECT {
    SELECT = BIT_RNG!(0, 1),
}
pub enum FHS_SEL {
    SEL_192M_PLL = 0,
    //	SEL_48M_PLL = 1,
    SEL_32M_OSC = 1,
    //	SEL_16M_OSC = 3,
}

regrw!(reg_mcu_wakeup_mask, 0x78, 32);

//****************************************************
// Analog control registers (0xB8 - 0xBA)
//****************************************************
//
// This section defines registers that control analog peripheral settings through
// an indirect access mechanism. These registers provide access to analog modules
// such as ADC, PGA, DAC, and various power control settings.
//
// Key Features:
// - Indirect register access via address/data/control mechanism
// - Access to various analog settings not directly mapped in memory
// - Control of RF power, bias, and calibration settings
// - Management of analog power states and wake-up sources
//
// Register Usage:
// - reg_ana_addr: Sets the address of the analog register to access
// - reg_ana_data: Data to write to or read from the analog register
// - reg_ana_ctrl: Control register for analog access operations
//
// Usage Examples:
//
// 1. Reading an analog register:
//    ```
//    // Read analog register 0x3f (deep sleep flag)
//    let deep_sleep_flag = analog_read(0x3f);
//    
//    // Check if woke from deep sleep
//    if (deep_sleep_flag & 0x40) != 0 {
//        // Device woke from deep sleep
//    }
//    ```
//
// 2. Writing to an analog register:
//    ```
//    // Configure RF power settings
//    analog_write(0xa2, 0x28);  // Set power amplifier gain
//    analog_write(0x4, 0x11);   // Set bias current
//    ```
//
// 3. Implementing the analog_read function:
//    ```
//    // Write address to register
//    write_reg_ana_addr(addr);
//    
//    // Set control bits for read operation
//    write_reg_ana_ctrl(FLD_ANA::START as u8 | FLD_ANA::RSV as u8);
//    
//    // Wait for operation to complete
//    while (read_reg_ana_ctrl() & (FLD_ANA::BUSY as u8)) != 0 {}
//    
//    // Read the result
//    let data = read_reg_ana_data();
//    
//    // Clear control register
//    write_reg_ana_ctrl(0);
//    
//    return data;
//    ```

regrw!(reg_ana_ctrl32, 0xb8, 32);
regrw!(reg_ana_addr_data, 0xb8, 16);
regrw!(reg_ana_addr, 0xb8, 8);
regrw!(reg_ana_data, 0xb9, 8);
regrw!(reg_ana_ctrl, 0xba, 8);

pub enum FLD_ANA {
    BUSY = BIT!(0),
    RSV = BIT!(4),
    RW = BIT!(5),
    START = BIT!(6),
    CYC = BIT!(7),
}

//****************************************************
// RF transmission registers (0x400 - 0x4FF)
//****************************************************
//
// This section defines registers that control RF (Radio Frequency) functionality
// of the TLSR8266. These registers manage transmit/receive operations, channel
// selection, access codes, RF status, and more.
//
// Key RF Registers:
// - Transmission mode registers (0x400): Control packet format and modulation features
// - Channel control (0x40d): Sets the RF channel (frequency)
// - Access code (0x408): 32-bit code for filtering packets
// - RX sensitivity threshold (0x422): Controls receiver sensitivity
// - RX automatic calibration (0x426): Manages automatic calibration of receiver
// - RX mode (0x428): Sets receiver configuration (1M/2M mode, filtering)
// 
// Usage Examples:
// 
// 1. Setting RF channel:
//    ```
//    // Set to channel 5 (2405 MHz)
//    write_reg_rf_channel(5);
//    ```
// 
// 2. Configuring transmission mode:
//    ```
//    // Enable DMA and CRC for transmissions
//    let mut tx_mode = read_reg_rf_tx_mode();
//    tx_mode |= (FLD_RF_TX_MODE::DMA_EN as u16 | FLD_RF_TX_MODE::CRC_EN as u16);
//    write_reg_rf_tx_mode(tx_mode);
//    ```
//
// 3. Setting access code for packet filtering:
//    ```
//    // Set access code for mesh network
//    write_reg_rf_access_code(0x12345678);
//    ```

// RF transmission mode registers
regrw!(reg_rf_tx_mode1, 0x400, 8);
regrw!(reg_rf_tx_mode, 0x400, 16);
pub enum FLD_RF_TX_MODE {
    DMA_EN = BIT!(0),
    CRC_EN = BIT!(1),
    BANDWIDTH = BIT_RNG!(2, 3),
    OUTPUT = BIT!(4),
    TST_OUT = BIT!(5),
    TST_EN = BIT!(6),
    TST_MODE = BIT!(7),
    ZB_PN_EN = BIT!(8),
    ZB_FEC_EN = BIT!(9),
    ZB_INTL_EN = BIT!(10), // interleaving
    TX_1M2M_PN_EN = BIT!(11),
    TX_1M2M_FEC_EN = BIT!(12),
    TX_1M2M_INTL_EN = BIT!(13), // interleaving
}

// RF channel control register
regrw!(reg_rf_channel, 0x40d, 8);

regrw!(reg_rf_access_code, 0x408, 32);
regrw!(reg_rf_tx_buf_sta, 0x41c, 32);

// RF reception registers
regrw!(reg_rf_rx_sense_thr, 0x422, 8);
regrw!(reg_rf_rx_auto, 0x426, 8);
pub enum FLD_RF_RX_AUTO {
    IRR_GAIN = BIT!(0),
    RX_IRR_PHASE = BIT!(1),
    RX_DAC_I = BIT!(2),
    RX_DAC_Q = BIT!(3),
    RX_LNA_GAIN = BIT!(4),
    RX_MIX2_GAIN = BIT!(5),
    RX_PGA_GAIN = BIT!(6),
    RX_CAL_EN = BIT!(7),
}

regrw!(reg_rf_rx_sync, 0x427, 8);
pub enum FLD_RF_SYNC {
    FREQ_COMP_EN = BIT!(0),
    ADC_SYNC = BIT!(1),
    ADC_INP_SIGNED = BIT!(2),
    SWAP_ADC_IQ = BIT!(3),
    NOTCH_FREQ_SEL = BIT!(4),
    NOTCH_BAND_SEL = BIT!(5),
    NOTCH_EN = BIT!(6),
    DN_CONV_FREQ_SEL = BIT!(7),
}

regrw!(reg_rf_rx_mode, 0x428, 8);

bitflags! {
    pub struct FLD_RF_RX_MODE: u8 {
        const EN = BIT!(0);
        const MODE_1M = BIT!(1);
        const MODE_2M = BIT!(2);
        const LOW_IF = BIT!(3);
        const BYPASS_DCOC = BIT!(4);
        const MAN_FINE_TUNE = BIT!(5);
        const SINGLE_CAL = BIT!(6);
        const LOW_PASS_FILTER = BIT!(7);
    }
}

regrw!(reg_rf_rx_pilot, 0x42b, 8);
pub enum FLD_RF_PILOT {
    LEN = BIT_RNG!(0, 3),
    RF_ZB_SFD_CHK = BIT!(4),
    RF_1M_SFD_CHK = BIT!(5),
    RF_2M_SFD_CHK = BIT!(6),
    RF_ZB_OR_AUTO = BIT!(7),
}

regrw!(reg_rf_rx_chn_dc, 0x42c, 32);
regrw!(reg_rf_rx_q_chn_cal, 0x42f, 8);
pub enum FLD_RF_RX_DCQ_CAL {
    FLD_RF_RX_DCQ_HIGH = BIT_RNG!(0, 6),
    FLD_RF_RX_DCQ_CAL_START = BIT!(7),
}
regrw!(reg_rf_rx_pel, 0x434, 16);
regrw!(reg_rf_rx_pel_gain, 0x434, 32);
regrw!(reg_rf_rx_rssi_offset, 0x439, 8);

regrw!(reg_rf_rx_hdx, 0x43b, 8);
pub enum FLD_RF_RX_HDX {
    RX_HEADER_LEN = BIT_RNG!(0, 3),
    RT_TICK_LO_SEL = BIT!(4),
    RT_TICK_HI_SEL = BIT!(5),
    RT_TICK_FRAME = BIT!(6),
    PKT_LEN_OUTP_EN = BIT!(7),
}

regrw!(reg_rf_rx_gctl, 0x43c, 8);
pub enum FLD_RF_RX_GCTL {
    CIC_SAT_LO_EN = BIT!(0),
    CIC_SAT_HI_EN = BIT!(1),
    AUTO_PWR = BIT!(2),
    ADC_RST_VAL = BIT!(4),
    ADC_RST_EN = BIT!(5),
    PWR_CHG_DET_S = BIT!(6),
    PWR_CHG_DET_N = BIT!(7),
}
regrw!(reg_rf_rx_peak, 0x43d, 8);
pub enum FLD_RF_RX_PEAK {
    FLD_RX_PEAK_DET_SRC_EN = BIT_RNG!(0, 2),
    FLD_TX_PEAK_DET_EN = BIT!(3),
    FLD_PEAK_DET_NUM = BIT_RNG!(4, 5),
    FLD_PEAK_MAX_CNT_PRD = BIT_RNG!(6, 7),
}

regrw!(reg_rf_rx_status, 0x443, 8);
pub enum FLD_RF_RX_STATUS {
    RX_STATE = BIT_RNG!(0, 3),
    RX_STA_RSV = BIT_RNG!(4, 5),
    RX_INTR = BIT!(6),
    TX_INTR = BIT!(7),
}

regrw!(reg_rx_rnd_mode, 0x447, 8);
pub enum FLD_RX_RND_MODE {
    SRC = BIT!(0),
    MANU_MODE = BIT!(1),
    AUTO_RD = BIT!(2),
    FREE_MODE = BIT!(3),
    CLK_DIV = BIT_RNG!(4, 7),
}
regrw!(reg_rnd_number, 0x448, 16);

regrw!(reg_rf_crc, 0x44c, 32);

regrw!(reg_rf_rtt, 0x454, 32);
pub enum FLD_RF_RTT {
    CAL = BIT_RNG!(0, 7),
    CYC1 = BIT_RNG!(8, 15),
    LOCK = BIT_RNG!(16, 23),
    SD_DLY_40M = BIT_RNG!(24, 27),
    SD_DLY_BYPASS = BIT!(28),
}

regrw!(reg_rf_chn_rssi, 0x458, 8);

regrw_idx!(reg_rf_rx_gain_agc, 0x480, 32);

regrw!(reg_rf_rx_dci, 0x4cb, 8); 
regrw!(reg_rf_rx_dcq, 0x4cf, 8); 

// PLL control registers
regrw!(reg_pll_rx_coarse_tune, 0x4d0, 16);
regrw!(reg_pll_rx_coarse_div, 0x4d2, 8);
regrw!(reg_pll_rx_fine_tune, 0x4d4, 16);
regrw!(reg_pll_rx_fine_div, 0x4d6, 8);
regrw!(reg_pll_rx_fine_div_tune, 0x004d6, 16);
regrw!(reg_pll_tx_coarse_tune, 0x4d8, 16);
regrw!(reg_pll_tx_coarse_div, 0x4da, 8);
regrw!(reg_pll_tx_fine_tune, 0x4dc, 16);
regrw!(reg_pll_tx_fine_div, 0x4de, 8);

regrw!(reg_pll_rx_frac, 0x4e0, 32);
regrw!(reg_pll_tx_frac, 0x4e4, 32);

regrw!(reg_pll_tx_ctrl, 0x4e8, 8);
regrw!(reg_pll_ctrl16, 0x4e8, 16);
regrw!(reg_pll_ctrl, 0x4e8, 32);
pub enum FLD_PLL_CTRL {
    TX_CYC0 = BIT!(0),
    TX_SOF = BIT!(1),
    TX_CYC1 = BIT!(2),
    TX_PRE_EN = BIT!(3),
    TX_VCO_EN = BIT!(4),
    TX_PWDN_DIV = BIT!(5),
    TX_MOD_EN = BIT!(6),
    TX_MOD_TRAN_EN = BIT!(7),
    RX_CYC0 = BIT!(8),
    RX_SOF = BIT!(9),
    RX_CYC1 = BIT!(10),
    RX_PRES_EN = BIT!(11),
    RX_VCO_EN = BIT!(12),
    RX_PWDN_DIV = BIT!(13),
    RX_PEAK_EN = BIT!(14),
    RX_TP_CYC = BIT!(15),
    SD_RSTB = BIT!(16),
    SD_INTG_EN = BIT!(17),
    CP_TRI = BIT!(18),
    PWDN_INTG1 = BIT!(19),
    PWDN_INTG2 = BIT!(20),
    PWDN_INTG_DIV = BIT!(21),
    PEAK_DET_EN = BIT!(22),
    OPEN_LOOP_EN = BIT!(23),
    RX_TICK_EN = BIT!(24),
    TX_TICK_EN = BIT!(25),
    RX_ALWAYS_ON = BIT!(26),
    TX_ALWAYS_ON = BIT!(27),
    MANUAL_MODE_EN = BIT!(28),
    CAL_DONE_EN = BIT!(29),
    LOCK_EN = BIT!(30),
}

regrw!(reg_pll_rx_ctrl, 0x4e9, 8);
pub enum FLD_PLL_RX_CTRL {
    CYC0 = BIT!(0),
    SOF = BIT!(1),
    CYC1 = BIT!(2),
    PRES_EN = BIT!(3),
    VCO_EN = BIT!(4),
    PD_DIV = BIT!(5),
    PEAK_EN = BIT!(6),
    TP_CYC = BIT!(7),
}

regrw!(reg_pll_ctrl_a, 0x4eb, 8);
pub enum FLD_PLL_CTRL_A {
    RX_TICK_EN = BIT!(0),
    TX_TICK_EN = BIT!(1),
    RX_ALWAYS_ON = BIT!(2),
    TX_ALWAYS_ON = BIT!(3),
    MANUAL_MODE_EN = BIT!(4),
    CAL_DONE_EN = BIT!(5),
    LOCK_EN = BIT!(6),
}

// PLL polarity controls
regrw!(reg_pll_pol_ctrl, 0x4ec, 16);
pub enum FLD_PLL_POL_CTRL {
    TX_PRE_EN = BIT!(0),
    TX_VCO_EN = BIT!(1),
    TX_PD_DIV = BIT!(2),
    MOD_EN = BIT!(3),
    MOD_TRAN_EN = BIT!(4),
    RX_PRE_EN = BIT!(5),
    RX_VCO_EN = BIT!(6),
    RX_PD_DIV = BIT!(7),
    SD_RSTB = BIT!(8),
    SD_INTG_EN = BIT!(9),
    CP_TRI = BIT!(10),
    TX_SOF = BIT!(11),
    RX_SOF = BIT!(12),
}

regrw!(reg_rf_rx_cap, 0x4f0, 16); 
regrw!(reg_rf_tx_cap, 0x4f0, 16); 

//****************************************************
// DMA and MAC registers (0x500 - 0x57F)
//****************************************************
//
// This section defines registers that control the Direct Memory Access (DMA) controller,
// which enables efficient data transfers between memory and peripherals without CPU intervention.
//
// DMA Features:
// - Multiple independent DMA channels
// - Configurable buffer sizes and memory addressing modes
// - Support for ping-pong (double buffering) mode
// - FIFO support for continuous transfers
// - Automatic mode for periodic transfers
// - Byte or word transfer modes
// - Status flags and interrupt generation
//
// Key Registers:
// - reg_dma#_addr: Source/destination address for DMA channel #
// - reg_dma#_ctrl: Configuration register for DMA channel #
// - reg_dma_chn_en: Enables specific DMA channels
// - reg_dma_chn_irq_msk: Controls which channels generate interrupts
// - reg_dma_tx_rdy#/rx_rdy#: Ready flags for transfers
// - reg_dma_tx_rptr/wptr: Pointer registers for FIFO management
//
// Important Channel Assignments:
// - Channels 0-1: Ethernet RX/TX
// - Channels 2-3: RF (BLE) RX/TX
// - Channels 4-5: Available for other peripherals
//
// Usage Examples:
//
// 1. Setting up a basic DMA transfer for RF reception:
//    ```
//    // Set RF RX buffer address (where received data will be stored)
//    write_reg_dma_rf_rx_addr(0x8000);  // Memory address
//    
//    // Configure RF RX DMA properties
//    let mut ctrl = 0;
//    ctrl |= (64 & 0xFF);  // Set buffer size to 64 bytes
//    ctrl |= (FLD_DMA::WR_MEM as u16);  // Write to memory
//    // Enable ping-pong for continuous reception if needed
//    // ctrl |= (FLD_DMA::PINGPONG_EN as u16); 
//    write_reg_dma_rf_rx_ctrl(ctrl);
//    
//    // Enable the RF RX DMA channel
//    let mut en = read_reg_dma_chn_en();
//    en |= (FLD_DMA::RF_RX as u32);
//    write_reg_dma_chn_en(en);
//    ```
//
// 2. Enabling DMA interrupts:
//    ```
//    // Enable interrupt for RF RX completion
//    let mut irq_msk = read_reg_dma_chn_irq_msk();
//    irq_msk |= (FLD_DMA::RF_RX as u32);
//    write_reg_dma_chn_irq_msk(irq_msk);
//    
//    // Also enable DMA interrupts in the global interrupt mask
//    let mut irq = read_reg_irq_mask();
//    irq |= (FLD_IRQ::DMA_EN as u32);
//    write_reg_irq_mask(irq);
//    ```
//
// 3. Checking DMA transfer status:
//    ```
//    // Check if RF RX transfer is complete
//    let status = read_reg_dma_rx_rdy0();
//    if (status & (FLD_DMA::RF_RX as u32)) != 0 {
//        // RF data received and available in buffer
//        // Process received data
//        
//        // Clear the ready flag to prepare for next reception
//        write_reg_dma_rx_rdy0(FLD_DMA::RF_RX as u32);
//    }
//    ```

// DMA channel registers
regrw!(reg_dma0_addr, 0x500, 16);
regrw!(reg_dma0_ctrl, 0x502, 16);
regrw!(reg_dma1_addr, 0x504, 16);
regrw!(reg_dma1_ctrl, 0x506, 16);
regrw!(reg_dma2_addr, 0x508, 16);
regrw!(reg_dma2_ctrl, 0x50a, 16);
regrw!(reg_dma3_addr, 0x50c, 16);
regrw!(reg_dma3_ctrl, 0x50e, 16);
regrw!(reg_dma4_addr, 0x510, 16);
regrw!(reg_dma4_ctrl, 0x512, 16);
regrw!(reg_dma5_addr, 0x514, 16);
regrw!(reg_dma5_ctrl, 0x516, 16);

// DMA control registers
regrw!(reg_dma_chn_en, 0x520, 8);
regrw!(reg_dma_chn_irq_msk, 0x521, 8);
regrw!(reg_dma_tx_rdy0, 0x524, 8);
regrw!(reg_dma_tx_rdy1, 0x525, 8);
regrw!(reg_dma_rx_rdy0, 0x526, 8);
regrw_copy!(reg_dma_irq_src, reg_dma_rx_rdy0, 8);
regrw!(reg_dma_rx_rdy1, 0x527, 8);

regrw!(reg_dma_tx_rptr, 0x52a, 8);
regrw!(reg_dma_tx_wptr, 0x52b, 8);
regrw!(reg_dma_tx_fifo, 0x52c, 16);

//  The default channel assignment
regrw_copy!(reg_dma_eth_rx_addr, reg_dma0_addr, 16);
regrw_copy!(reg_dma_eth_rx_ctrl, reg_dma0_ctrl, 16);
regrw_copy!(reg_dma_eth_tx_addr, reg_dma1_addr, 16);

regrw_copy!(reg_dma_rf_rx_addr, reg_dma2_addr, 16);
regrw_copy!(reg_dma_rf_rx_ctrl, reg_dma2_ctrl, 16);
regrw_copy!(reg_dma_rf_tx_addr, reg_dma3_addr, 16);
regrw_copy!(reg_dma_rf_tx_ctrl, reg_dma3_ctrl, 16);

pub enum FLD_DMA {
    BUF_SIZE = BIT_RNG!(0, 7),
    WR_MEM = BIT!(8),
    PINGPONG_EN = BIT!(9),
    FIFO_EN = BIT!(10),
    AUTO_MODE = BIT!(11),
    BYTE_MODE = BIT!(12),

    RPTR_CLR = BIT!(4),
    RPTR_NEXT = BIT!(5),
    RPTR_SET = BIT!(6),
}

impl FLD_DMA {
    pub const ETH_RX: u32 = BIT!(0);
    pub const ETH_TX: u32 = BIT!(1);
    pub const RF_RX: u32 = BIT!(2);
    pub const RF_TX: u32 = BIT!(3);
}

//****************************************************
// AES registers (0x540 - 0x55F)
//****************************************************
//
// This section defines registers that control the AES (Advanced Encryption Standard)
// encryption hardware module. AES is used for mesh security, pairing, and OTA updates.
//
// The TLSR8266 includes hardware acceleration for AES-128 encryption/decryption:
// - 128-bit key support (16 bytes stored in key0-key15 registers)
// - Block cipher mode with 128-bit block size
// - Single operation cycle per encryption/decryption
//
// Register Usage:
// - reg_aes_ctrl: Control register for AES operations
// - reg_aes_data: 32-bit data register for input/output
// - reg_aes_key0...key15: 16 bytes for AES key storage
//
// Usage Examples:
//
// 1. Setting an AES key:
//    ```
//    // Write 16-byte key byte by byte
//    write_reg_aes_key0(key[0]);
//    write_reg_aes_key1(key[1]);
//    // ... and so on for all 16 bytes
//    ```
//
// 2. Encrypting data block:
//    ```
//    // Set data input
//    write_reg_aes_data(data_block);
//    
//    // Start encryption operation
//    write_reg_aes_ctrl(0x01);  // Enable AES encryption
//    
//    // Wait for completion
//    while (read_reg_aes_ctrl() & 0x01 != 0) {}
//    
//    // Read encrypted result
//    let encrypted = read_reg_aes_data();
//    ```

regrw!(reg_aes_ctrl, 0x540, 8);
regrw!(reg_aes_data, 0x548, 32);
regrw_idx!(reg_aes_key, 0x550, 8);
regrw!(reg_aes_key0, 0x550, 8);
regrw!(reg_aes_key1, 0x551, 8);
regrw!(reg_aes_key2, 0x552, 8);
regrw!(reg_aes_key3, 0x553, 8);
regrw!(reg_aes_key4, 0x554, 8);
regrw!(reg_aes_key5, 0x555, 8);
regrw!(reg_aes_key6, 0x556, 8);
regrw!(reg_aes_key7, 0x557, 8);
regrw!(reg_aes_key8, 0x558, 8);
regrw!(reg_aes_key9, 0x559, 8);
regrw!(reg_aes_key10, 0x55a, 8);
regrw!(reg_aes_key11, 0x55b, 8);
regrw!(reg_aes_key12, 0x55c, 8);
regrw!(reg_aes_key13, 0x55d, 8);
regrw!(reg_aes_key14, 0x55e, 8);
regrw!(reg_aes_key15, 0x55f, 8);

//****************************************************
// GPIO registers (0x580 - 0x5FF)
//****************************************************
//
// This section defines registers that control the General Purpose Input/Output (GPIO)
// pins on the TLSR8266. The SoC features multiple ports (A-F), each with up to 8 pins.
//
// For each GPIO port/pin:
// - in: Input value reading
// - ie: Input enable (1 = enable)
// - oen: Output enable (0 = output, 1 = input)
// - out: Output value setting
// - pol: Polarity (1 = invert)
// - ds: Drive strength
// - gpio: GPIO function selection
// - irq_en: Interrupt enable
//
// Each GPIO pin can be configured as:
// - Input: Set ie=1, oen=1
// - Output: Set oen=0
// - Pull-up/down: Using external registers
// - Special function: Using gpio register
//
// Additionally, GPIO pins can generate interrupts based on edge or level triggers.
//
// Usage Examples:
//
// 1. Configure PA0 as output and set high:
//    ```
//    // Set PA0 as output
//    let mut oen = read_reg_gpio_pa_oen();
//    oen &= !(0x01);  // Clear bit 0 (0 = output)
//    write_reg_gpio_pa_oen(oen);
//    
//    // Set PA0 high
//    let mut out = read_reg_gpio_pa_out();
//    out |= 0x01;  // Set bit 0
//    write_reg_gpio_pa_out(out);
//    ```
//
// 2. Read input value from PB2:
//    ```
//    // Enable PB2 as input
//    let mut ie = read_reg_gpio_pb_ie();
//    ie |= 0x04;  // Set bit 2 (1 = input enabled)
//    write_reg_gpio_pb_ie(ie);
//    
//    let mut oen = read_reg_gpio_pb_oen();
//    oen |= 0x04;  // Set bit 2 (1 = input mode)
//    write_reg_gpio_pb_oen(oen);
//    
//    // Read PB2 value
//    let value = read_reg_gpio_pb_in() & 0x04;  // Mask bit 2
//    if value != 0 {
//        // PB2 is high
//    } else {
//        // PB2 is low
//    }
//    ```
//
// 3. Configure interrupt on PC3 (falling edge):
//    ```
//    // Enable PC3 as input
//    let mut ie = read_reg_gpio_pc_ie();
//    ie |= 0x08;  // Set bit 3
//    write_reg_gpio_pc_ie(ie);
//    
//    // Set polarity to detect falling edge (1 = invert)
//    let mut pol = read_reg_gpio_pc_pol();
//    pol |= 0x08;  // Set bit 3
//    write_reg_gpio_pc_pol(pol);
//    
//    // Enable interrupt for PC3
//    let mut irq_en = read_reg_gpio_pc_irq_en();
//    irq_en |= 0x08;  // Set bit 3
//    write_reg_gpio_pc_irq_en(irq_en);
//    
//    // Enable GPIO interrupts in global interrupt mask
//    let mut irq_mask = read_reg_irq_mask();
//    irq_mask |= FLD_IRQ::GPIO_EN as u32;
//    write_reg_irq_mask(irq_mask);
//    ```

// Generic GPIO access registers
regrw_idx!(reg_gpio_in, 0x580, 8);
regrw_idx!(reg_gpio_ie, 0x581, 8);
regrw_idx!(reg_gpio_oen, 0x582, 8);
regrw_idx!(reg_gpio_out, 0x583, 8);
regrw_idx!(reg_gpio_pol, 0x584, 8);
regrw_idx!(reg_gpio_ds, 0x585, 8);
regrw_idx!(reg_gpio_gpio_func, 0x586, 8);

// Port A GPIO registers
regrw!(reg_gpio_pa_in, 0x580, 8);
regrw!(reg_gpio_pa_ie, 0x581, 8);
regrw!(reg_gpio_pa_oen, 0x582, 8);
regrw!(reg_gpio_pa_out, 0x583, 8);
regrw!(reg_gpio_pa_pol, 0x584, 8);
regrw!(reg_gpio_pa_ds, 0x585, 8);
regrw!(reg_gpio_pa_gpio, 0x586, 8);
regrw!(reg_gpio_pa_irq_en, 0x587, 8);
regrw!(reg_gpio_pa_setting1, 0x580, 32);
regrw!(reg_gpio_pa_setting2, 0x584, 32);

// Port B GPIO registers
regrw!(reg_gpio_pb_in, 0x588, 8);
regrw!(reg_gpio_pb_ie, 0x589, 8);
regrw!(reg_gpio_pb_oen, 0x58a, 8);
regrw!(reg_gpio_pb_out, 0x58b, 8);
regrw!(reg_gpio_pb_pol, 0x58c, 8);
regrw!(reg_gpio_pb_ds, 0x58d, 8);
regrw!(reg_gpio_pb_gpio, 0x58e, 8);
regrw!(reg_gpio_pb_irq_en, 0x58f, 8);
regrw!(reg_gpio_pb_setting1, 0x588, 32);
regrw!(reg_gpio_pb_setting2, 0x58c, 32);

// Port C GPIO registers
regrw!(reg_gpio_pc_in, 0x590, 8);
regrw!(reg_gpio_pc_ie, 0x591, 8);
regrw!(reg_gpio_pc_oen, 0x592, 8);
regrw!(reg_gpio_pc_out, 0x593, 8);
regrw!(reg_gpio_pc_pol, 0x594, 8);
regrw!(reg_gpio_pc_ds, 0x595, 8);
regrw!(reg_gpio_pc_gpio, 0x596, 8);
regrw!(reg_gpio_pc_irq_en, 0x597, 8);
regrw!(reg_gpio_pc_setting1, 0x590, 32);
regrw!(reg_gpio_pc_setting2, 0x594, 32);

// Port D GPIO registers
regrw!(reg_gpio_pd_in, 0x598, 8);
regrw!(reg_gpio_pd_ie, 0x599, 8);
regrw!(reg_gpio_pd_oen, 0x59a, 8);
regrw!(reg_gpio_pd_out, 0x59b, 8);
regrw!(reg_gpio_pd_pol, 0x59c, 8);
regrw!(reg_gpio_pd_ds, 0x59d, 8);
regrw!(reg_gpio_pd_gpio, 0x59e, 8);
regrw!(reg_gpio_pd_irq_en, 0x59f, 8);
regrw!(reg_gpio_pd_setting1, 0x598, 32);
regrw!(reg_gpio_pd_setting2, 0x59c, 32);

// Port E GPIO registers
regrw!(reg_gpio_pe_in, 0x5a0, 8);
regrw!(reg_gpio_pe_ie, 0x5a1, 8);
regrw!(reg_gpio_pe_oen, 0x5a2, 8);
regrw!(reg_gpio_pe_out, 0x5a3, 8);
regrw!(reg_gpio_pe_pol, 0x5a4, 8);
regrw!(reg_gpio_pe_ds, 0x5a5, 8);
regrw!(reg_gpio_pe_gpio, 0x5a6, 8);
regrw!(reg_gpio_pe_irq_en, 0x5a7, 8);
regrw!(reg_gpio_pe_setting1, 0x5a0, 32);
regrw!(reg_gpio_pe_setting2, 0x5a4, 32);

// Port F GPIO registers
regrw!(reg_gpio_pf_in, 0x5a8, 8);
regrw!(reg_gpio_pf_ie, 0x5a9, 8);
regrw!(reg_gpio_pf_oen, 0x5aa, 8);
regrw!(reg_gpio_pf_out, 0x5ab, 8);
regrw!(reg_gpio_pf_pol, 0x5ac, 8);
regrw!(reg_gpio_pf_ds, 0x5ad, 8);
regrw!(reg_gpio_pf_gpio, 0x5ae, 8);
regrw!(reg_gpio_pf_irq_en, 0x5af, 8);
regrw!(reg_gpio_pf_setting1, 0x5a8, 32);
regrw!(reg_gpio_pf_setting2, 0x5ac, 32);

// GPIO control and configuration registers
regrw!(reg_gpio_ctrl, 0x5a4, 32);

pub enum GPIO_CTRL {
    GPIO_WAKEUP_EN = BIT!(0),
    GPIO_IRQ_EN = BIT!(1),
    I2S_SLAVE_EN = BIT!(2),
    RMII_REFCLK_OUTPUT_EN = BIT!(3),
}

regrw!(reg_gpio_config_func, 0x5b0, 32);
regrw!(reg_gpio_config_func0, 0x5b0, 8);

pub enum GPIO_CFG_FUNC0 {
    FLD_I2S_REFCLK_DMIC = BIT!(0),
    FLD_I2S_BCK_BB_PEAK = BIT!(1),
    FLD_I2S_BCK_PWM1 = BIT!(2),
    FLD_I2S_LCK_UART_RX = BIT!(3),
    FLD_I2S_LCK_PWM2 = BIT!(4),
    FLD_I2S_DO_UART_TX = BIT!(5),
    FLD_I2S_DO_PWM3 = BIT!(6),
    FLD_I2S_DI_DMIC = BIT!(7),
}

regrw!(reg_gpio_config_func1, 0x5b1, 8);
pub enum GPIO_CFG_FUNC1 {
    FLD_RP_TX_CYC1 = BIT!(0),
    FLD_RN_BB_RSSI = BIT!(1),
    FLD_GP6_BB_SS2 = BIT!(2),
    FLD_GP7_RXADC_CLK = BIT!(3),
    FLD_RP_T0 = BIT!(4),
    FLD_RN_T1 = BIT!(5),
    FLD_GP6_TE = BIT!(6),
    FLD_GP7_MDC = BIT!(7),
}

regrw!(reg_gpio_config_func2, 0x5b2, 8);
pub enum GPIO_CFG_FUNC2 {
    FLD_GP8_RXADC_DAT = BIT!(0),
    FLD_GP9_BB_SS1 = BIT!(1),
    FLD_GP10_BBSS0 = BIT!(2),
    FLD_SWS_BB_GAIN4 = BIT!(3),
    FLD_DMIC_CK_BBCLK_BB = BIT!(4),
    FLD_DMIC_CK_REFCLK = BIT!(5),
    FLD_I2S_BCK_R0 = BIT!(6),
    FLD_I2S_LCK_R1 = BIT!(7),
}

regrw!(reg_gpio_config_func3, 0x5b3, 8);
enum GPIO_CFG_FUNC3 {
    FLD_CN_BB_GAIN3 = BIT!(0),
    FLD_CK_BB_GAIN2 = BIT!(1),
    FLD_DO_BB_GAIN1 = BIT!(2),
    FLD_DI_BB_GAIN0 = BIT!(3),
    //	FLD_I2S_LCK_PWM2	=	BIT!(4),//NOT SURE
    FLD_I2S_DO_RXDV = BIT!(5),
    FLD_I2S_DI_RXER = BIT!(6),
    FLD_I2S_DI_TXSD = BIT!(7),
}

regrw!(reg_gpio_config_func4, 0x5b4, 8);
pub enum GPIO_CFG_FUNC4 {
    FLD_DMIC_CK_RX_CLK = BIT!(0),
    FLD_I2S_DI_RX_DAT = BIT!(1),
}

regrw!(reg_gpio_wakeup_irq, 0x5b5, 8);
pub enum FLD_GPIO_WAKEUP_IRQ {
    WAKEUP_EN = BIT!(2),
    INTERRUPT_EN = BIT!(3),
}

// GPIO IRQ control
regrw_idx!(reg_gpio_irq_wakeup_en, 0x587, 8); // reg_irq_mask: FLD_IRQ_GPIO_EN
regrw_idx!(reg_gpio_irq_risc0_en, 0x5b8, 8); // reg_irq_mask: FLD_IRQ_GPIO_RISC0_EN
regrw_idx!(reg_gpio_irq_risc1_en, 0x5c0, 8); // reg_irq_mask: FLD_IRQ_GPIO_RISC1_EN
regrw_idx!(reg_gpio_irq_risc2_en, 0x5c8, 8); // reg_irq_mask: FLD_IRQ_GPIO_RISC2_EN

//****************************************************
// Timer registers (0x620 - 0x63F)
//****************************************************
//
// This section defines registers that control the hardware timers in the TLSR8266.
// The SoC features three general-purpose timers and a watchdog timer.
//
// Timer Features:
// - Three 32-bit timers (TMR0, TMR1, TMR2)
// - Configurable modes: 1-shot or continuous
// - Watchdog timer for system reset on software hang
// - Capture registers for reading/setting timer values
// - Status flags for timer events
// 
// Key Registers:
// - reg_tmr_ctrl: Master control for all timers
// - reg_tmr#_capt: Capture/compare value for timer # (0-2)
// - reg_tmr#_tick: Current value of timer # (0-2)
// - reg_tmr_sta: Status bits for all timers
//
// Usage Examples:
//
// 1. Setting up Timer0 in continuous mode:
//    ```
//    // Set Timer0 capture value (when to trigger)
//    write_reg_tmr0_capt(50000);  // 50,000 clock cycles
//    
//    // Configure and start Timer0 in continuous mode
//    let mut ctrl = read_reg_tmr_ctrl();
//    ctrl &= !(FLD_TMR::TMR0_MODE as u32);  // Clear mode bits (00 = continuous)
//    ctrl |= FLD_TMR::TMR0_EN as u32;       // Enable timer
//    write_reg_tmr_ctrl(ctrl);
//    ```
//
// 2. Checking if Timer0 has triggered:
//    ```
//    let status = read_reg_tmr_sta();
//    if (status & (FLD_TMR_STA::TMR0 as u8)) != 0 {
//        // Timer0 has triggered
//        
//        // Clear the timer status flag
//        write_reg_tmr_sta(FLD_TMR_STA::TMR0 as u8);
//    }
//    ```
//
// 3. Setting up the watchdog timer:
//    ```
//    // Set watchdog timeout (higher value = longer timeout)
//    let mut ctrl = read_reg_tmr_ctrl();
//    ctrl &= !(0x3FFF << 9);  // Clear watchdog bits
//    ctrl |= (1000 & 0x3FFF) << 9;  // Set timeout value 
//    ctrl |= FLD_TMR::TMR_WD_EN as u32;  // Enable watchdog
//    write_reg_tmr_ctrl(ctrl);
//    
//    // Periodically reset watchdog to prevent system reset
//    // (must be called regularly to "pet" the watchdog)
//    ctrl = read_reg_tmr_ctrl();
//    ctrl |= FLD_TMR::CLR_WD as u32;
//    write_reg_tmr_ctrl(ctrl);
//    ```

regrw!(reg_tmr_ctrl, 0x620, 32);
regrw!(reg_tmr_ctrl16, 0x620, 16);
regrw!(reg_tmr_ctrl8, 0x620, 8);

pub enum FLD_TMR {
    TMR0_EN = BIT!(0),
    TMR0_MODE = BIT_RNG!(1, 2),
    TMR1_EN = BIT!(3),
    TMR1_MODE = BIT_RNG!(4, 5),
    TMR2_EN = BIT!(6),
    TMR2_MODE = BIT_RNG!(7, 8),
    TMR_WD_CAPT = BIT_RNG!(9, 22),
    TMR_WD_EN = BIT!(23),
    TMR0_STA = BIT!(24),
    TMR1_STA = BIT!(25),
    TMR2_STA = BIT!(26),
    CLR_WD = BIT!(27),
}

pub const WATCHDOG_TIMEOUT_COEFF: u32 = 18; //  check register definiton, 0x622

regrw!(reg_tmr_sta, 0x623, 8);
pub enum FLD_TMR_STA {
    TMR0 = BIT!(0),
    TMR1 = BIT!(1),
    TMR2 = BIT!(2),
    WD = BIT!(3),
}

regrw!(reg_tmr0_capt, 0x624, 32);
regrw!(reg_tmr1_capt, 0x628, 32);
regrw!(reg_tmr2_capt, 0x62c, 32);
// #define reg_tmr_capt(i)			REG_ADDR32(0x624 + ((i) << 2))
regrw!(reg_tmr0_tick, 0x630, 32);
regrw!(reg_tmr1_tick, 0x634, 32);
regrw!(reg_tmr2_tick, 0x638, 32);
// #define reg_tmr_tick(i)			REG_ADDR32(0x630 + ((i) << 2))

//****************************************************
// Interrupt registers (0x640 - 0x64F)
//****************************************************
//
// This section defines registers that control the interrupt system of the TLSR8266.
// The SoC features a flexible interrupt controller with maskable interrupts,
// priority levels, and status tracking.
//
// Key Registers:
// - reg_irq_mask: Controls which interrupt sources are enabled
// - reg_irq_pri: Sets priority levels for different interrupts
// - reg_irq_src: Shows which interrupts are currently active
// - reg_irq_en: Global interrupt enable
//
// Interrupt Sources:
// - Timer interrupts (TMR0, TMR1, TMR2)
// - DMA and FIFO interrupts
// - USB-related interrupts
// - GPIO interrupts (can be mapped to specific GPIO pins)
// - System timer interrupts
// - RF/BLE-related interrupts
//
// Usage Examples:
//
// 1. Enabling the Timer0 interrupt:
//    ```
//    // Enable Timer0 interrupt in the interrupt mask
//    let mut mask = read_reg_irq_mask();
//    mask |= FLD_IRQ::TMR0_EN as u32;
//    write_reg_irq_mask(mask);
//    
//    // Enable global interrupts
//    write_reg_irq_en(1);
//    ```
//
// 2. Checking which interrupts are active:
//    ```
//    let active_irqs = read_reg_irq_src();
//    
//    if (active_irqs & (FLD_IRQ::TMR0_EN as u32)) != 0 {
//        // Timer0 interrupt is active
//        // Handle Timer0 interrupt
//    }
//    
//    if (active_irqs & (FLD_IRQ::GPIO_EN as u32)) != 0 {
//        // GPIO interrupt is active
//        // Check which GPIO triggered the interrupt
//    }
//    ```
//
// 3. Setting interrupt priorities:
//    ```
//    // Give Timer0 interrupt higher priority than GPIO interrupts
//    let mut pri = read_reg_irq_pri();
//    
//    // Clear priority bits for Timer0 and GPIO (bits 0-1 for Timer0, bits 18-19 for GPIO)
//    pri &= ~((0x3 << 0) | (0x3 << 18));
//    
//    // Set Timer0 to highest priority (00) and GPIO to lower priority (01)
//    pri |= (0x0 << 0) | (0x1 << 18);
//    
//    write_reg_irq_pri(pri);
//    ```

regrw!(reg_irq_mask, 0x640, 32);
regrw!(reg_irq_pri, 0x644, 32);
regrw!(reg_irq_src, 0x648, 32);
regrw!(reg_irq_src3, 0x64a, 8);
regrw!(reg_irq_en, 0x643, 8);

#[repr(usize)]
pub enum FLD_IRQ {
    TMR0_EN = BIT!(0),
    TMR1_EN = BIT!(1),
    TMR2_EN = BIT!(2),
    USB_PWDN_EN = BIT!(3),
    DMA_EN = BIT!(4),
    DAM_FIFO_EN = BIT!(5),
    SBC_MAC_EN = BIT!(6),
    HOST_CMD_EN = BIT!(7),

    EP0_SETUP_EN = BIT!(8),
    EP0_DAT_EN = BIT!(9),
    EP0_STA_EN = BIT!(10),
    SET_INTF_EN = BIT!(11),
    IRQ4_EN = BIT!(12),
    ZB_RT_EN = BIT!(13),
    SW_EN = BIT!(14),
    AN_EN = BIT!(15),

    USB_250US_EN = BIT!(16),
    USB_RST_EN = BIT!(17),
    GPIO_EN = BIT!(18),
    PM_EN = BIT!(19),
    SYSTEM_TIMER = BIT!(20),
    GPIO_RISC0_EN = BIT!(21),
    GPIO_RISC1_EN = BIT!(22),
    GPIO_RISC2_EN = BIT!(23),

    EN = BIT_RNG!(24, 31),
}

//****************************************************
// System Tick registers (0x740 - 0x74F)
//****************************************************
//
// This section defines registers that control the System Tick functionality,
// which provides a precise timing source for system operation and timing events.
//
// Key Features:
// - 32-bit counter with configurable period
// - System wake-up timer functionality
// - Interrupt generation on timer events
// - Low-power operation capability
//
// Register Usage:
// - reg_system_tick: Current system tick value (32-bit counter)
// - reg_system_tick_irq: Interrupt trigger value
// - reg_system_wakeup_tick: Value for system wakeup from sleep
// - reg_system_tick_mode: Configuration of tick operation mode
// - reg_system_tick_ctrl: Control register for starting/stopping
//
// Usage Examples:
//
// 1. Configure and start system tick timer:
//    ```
//    // Set the system tick interrupt period (trigger every 10,000 ticks)
//    write_reg_system_tick_irq(10000);
//    
//    // Start the system tick counter
//    write_reg_system_tick_ctrl(FLD_SYSTEM_TICK::START as u8);
//    ```
//
// 2. Using system tick for precise delays:
//    ```
//    // Get current tick value
//    let start_tick = read_reg_system_tick();
//    
//    // Wait for 1000 ticks to pass
//    while (read_reg_system_tick() - start_tick < 1000) {
//        // Optional: can yield here for power saving
//    }
//    ```
//
// 3. Setting up system wakeup from sleep:
//    ```
//    // Set wakeup time (wake after 50,000 ticks)
//    write_reg_system_wakeup_tick(read_reg_system_tick() + 50000);
//    
//    // Configure power management to use system tick for wakeup
//    // ... additional power management code ...
//    ```

regrw!(reg_system_tick, 0x740, 32);
regrw!(reg_system_tick_irq, 0x744, 32);
regrw!(reg_system_wakeup_tick, 0x748, 32);
regrw!(reg_system_tick_mode, 0x74c, 8);
regrw!(reg_system_tick_ctrl, 0x74f, 8);

pub enum FLD_SYSTEM_TICK {
    START = BIT!(0),
    STOP = BIT!(1),
}

impl FLD_SYSTEM_TICK {
    pub const RUNNING: FLD_SYSTEM_TICK = FLD_SYSTEM_TICK::STOP;
    pub const IRQ_EN: FLD_SYSTEM_TICK = FLD_SYSTEM_TICK::STOP;
}

//****************************************************
// PWM registers (0x780 - 0x7BF)
//****************************************************
//
// This section defines registers that control Pulse Width Modulation (PWM) functionality,
// which is essential for LED dimming, motor control, and other applications requiring
// variable power output.
//
// PWM Features:
// - Multiple independent PWM channels
// - Configurable frequency and duty cycle
// - Polarity and inversion control
// - Phase adjustment and cycle timing
// - Interrupt generation on PWM events
//
// Key Registers:
// - reg_pwm_enable: Enables individual PWM channels
// - reg_pwm_clk: Controls PWM clock source and divider
// - reg_pwm_mode: Sets PWM operating mode
// - reg_pwm_invert: Inverts PWM output
// - reg_pwm_phase: Sets phase delay of PWM
// - reg_pwm_cycle: Sets period and duty cycle of PWM
//
// Usage Examples:
//
// 1. Setting up a basic PWM channel for LED dimming (50% duty cycle):
//    ```
//    // Set PWM0 cycle and compare value for 50% duty cycle
//    write_reg_pwm_cycle(0, 0x00100010);  // Max = 0x0010, Cmp = 0x0010 (100% duty)
//                                         // or use separate writes:
//                                         // High 16 bits = max value, Low 16 bits = compare value
//    
//    // Enable PWM0
//    let mut enable = read_reg_pwm_enable();
//    enable |= 0x01;  // Enable bit 0 for PWM0
//    write_reg_pwm_enable(enable);
//    ```
//
// 2. Setting up PWM with a different duty cycle (25% on):
//    ```
//    // Set PWM1 cycle and compare value for 25% duty cycle
//    write_reg_pwm_cycle(1, 0x00200005);  // Max = 0x0020, Cmp = 0x0005 (25% duty)
//    
//    // Enable PWM1
//    let mut enable = read_reg_pwm_enable();
//    enable |= 0x02;  // Enable bit 1 for PWM1
//    write_reg_pwm_enable(enable);
//    ```
//
// 3. Setting polarity and inversion:
//    ```
//    // Invert PWM2 output
//    let mut invert = read_reg_pwm_invert();
//    invert |= 0x04;  // Set bit 2 for PWM2
//    write_reg_pwm_invert(invert);
//    
//    // Set PWM2 polarity
//    let mut pol = read_reg_pwm_pol();
//    pol |= 0x04;  // Set bit 2 for PWM2
//    write_reg_pwm_pol(pol);
//    ```

regrw!(reg_pwm_enable, 0x780, 8);
regrw!(reg_pwm_clk, 0x781, 8);
regrw!(reg_pwm_mode, 0x782, 8);
regrw!(reg_pwm_invert, 0x783, 8);
regrw!(reg_pwm_n_invert, 0x784, 8);
regrw!(reg_pwm_pol, 0x785, 8);

regrw_idx!(reg_pwm_phase, 0x788, 16);
regrw_idx!(reg_pwm_cycle, 0x794, 32);
regrw_idx!(reg_pwm_cmp, 0x794, 16);

#[repr(usize)]
pub enum FLD_PWM {
    CMP = BIT_RNG!(0, 15),
    MAX = BIT_RNG!(16, 31),
}

regrw_idx!(reg_pwm_pulse_num, 0x7ac, 16); // i == 0, 1
regrw!(reg_pwm_irq_mask, 0x7b0, 8);
regrw!(reg_pwm_irq_sta, 0x7b1, 8);

//****************************************************
// RF control registers (0xF00 - 0xFFF)
//****************************************************
//
// This section defines registers that control Radio Frequency (RF) operation at a higher level
// than the RF transmission registers. These registers manage the RF state machine, timing,
// interrupts, and events for Bluetooth Low Energy (BLE) communication.
//
// Key Features:
// - RF state machine control (TX/RX states)
// - Interrupt handling for RF events
// - RF timing and synchronization
// - BLE protocol scheduling
// - RF mode configuration
//
// Key Registers:
// - reg_rf_mode_control: Configures overall RF operating mode
// - reg_rf_irq_mask/status: Controls/reports RF interrupt sources
// - reg_rf_sched_tick: Controls scheduling of RF operations
// - reg_rf_sn: Sequence number for RF packets
// - reg_rf_tx_wail_settle_time: Timing for transmitter settling
//
// Usage Examples:
//
// 1. Setting up RF interrupts:
//    ```
//    // Enable both RX and TX interrupts
//    let mut irq_mask = read_reg_rf_irq_mask();
//    irq_mask |= (FLD_RF_IRQ_MASK::IRQ_RX as u16 | FLD_RF_IRQ_MASK::IRQ_TX as u16);
//    write_reg_rf_irq_mask(irq_mask);
//    ```
//
// 2. Checking RF status:
//    ```
//    // Check if we've received a packet
//    let status = read_reg_rf_irq_status();
//    if (status & (FLD_RF_IRQ_MASK::IRQ_RX as u16)) != 0 {
//        // A packet was received, process it
//        // ...
//        
//        // Clear the interrupt
//        let mut clear = 0;
//        clear |= (FLD_RF_IRQ_MASK::IRQ_RX as u32);
//        write_reg_rf_event_clear(clear);
//    }
//    ```
//
// 3. Reading RX state:
//    ```
//    // Check the current RX state machine state
//    let rx_state = read_reg_rf_txrx_state();
//    if (rx_state & 0x0F) == (FLD_RF_RX_STATE::DEMOD as u8) {
//        // Currently demodulating a packet
//    } else if (rx_state & 0x0F) == (FLD_RF_RX_STATE::IDLE as u8) {
//        // Receiver is idle
//    }
//    ```

// RF TX/RX state machine control register
regrw!(reg_rf_txrx_state, 0x00f02, 8);
regrw!(reg_rf_mode_control, 0xf00, 8);
regrw!(reg_rf_sn, 0xf01, 8);
regrw!(reg_rf_tx_wail_settle_time, 0xf04, 32);
regrw!(reg_rf_sys_timer_config, 0xf0a, 16);
regrw!(reg_rf_mode, 0xf16, 8);
regrw!(reg_rf_sched_tick, 0xf18, 32);

regrw!(reg_rf_irq_mask, 0xf1c, 16);
regrw!(reg_rf_irq_status, 0xf20, 16);

pub enum FLD_RF_IRQ_MASK {
    IRQ_RX = BIT!(0),
    IRQ_TX = BIT!(1),
    IRX_RX_TIMEOUT = BIT!(2),
    IRX_CMD_DONE = BIT!(5),
    IRX_RETRY_HIT = BIT!(7),
}

regrw!(reg_rf_event_clear, 0xf28, 32);
regrw!(reg_rf_timing_config, 0xf2c, 16);

// The value for FLD_RF_RX_STATE
pub enum FLD_RF_RX_STATE {
    IDLE = 0,
    SET_GAIN = 1,
    CIC_SETTLE = 2,
    LPF_SETTLE = 3,
    PE = 4,
    SYN_START = 5,
    GLOB_SYN = 6,
    GLOB_LOCK = 7,
    LOCAL_SYN = 8,
    LOCAL_LOCK = 9,
    ALIGN = 10,
    ADJUST = 11,
    DEMOD = 12, // de modulation
    FOOTER = 13,
}

///////////////////// PM register /////////////////////////

pub const  		rega_deepsleep_flag: u8 =		0x3f;		//0x34 - 0x39 (watch dog reset)
// #define		rega_deepsleep_flag: u8 =		0x34		//0x3a - 0x3b (power-on reset)
// pub const  		flag_deepsleep_wakeup	(analog_read(0x3f) & 0x40)

pub const  		rega_wakeup_en_val0: u8 =		0x41;
pub const  		rega_wakeup_en_val1: u8 =		0x42;
pub const  		rega_wakeup_en_val2: u8 =		0x43;
pub const  		raga_gpio_wkup_pol: u8 =		0x44;

pub const  		raga_pga_gain0: u8 =		0x86;
pub const  		raga_pga_gain1: u8 = 0x87;