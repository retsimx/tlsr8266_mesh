use crate::mcu::gpio::GPIO_PIN_TYPE::*;
use crate::BIT;
use crate::{BM_CLR, BM_SET};
use tlsr8266_pac::{
    read_reg_gpio_gpio_func, read_reg_gpio_oen, read_reg_gpio_out, write_reg_gpio_gpio_func,
    write_reg_gpio_oen, write_reg_gpio_out, write_reg_gpio_pa_setting1, write_reg_gpio_pa_setting2,
    write_reg_gpio_pb_setting1, write_reg_gpio_pb_setting2, write_reg_gpio_pc_setting1,
    write_reg_gpio_pc_setting2, write_reg_gpio_pd_setting1, write_reg_gpio_pd_setting2,
    write_reg_gpio_pe_setting1, write_reg_gpio_pe_setting2, write_reg_gpio_pf_setting1,
    write_reg_gpio_pf_setting2,
};

pub const RF_FAST_MODE_1M: u8 = 1;
pub const PM_PIN_PULL_DEFAULT: u8 = 1;

/// Pull-up/pull-down configuration constants for GPIO pins
///
/// These constants define various pull-up/pull-down resistor settings that can be
/// applied to GPIO pins to ensure defined logic levels when pins are not actively
/// driven.
///
/// * `GPIO_PULL_UP_0`: No pull-up (floating)
/// * `GPIO_PULL_UP_1M`: 1MΩ pull-up resistor
/// * `GPIO_PULL_UP_10K`: 10kΩ pull-up resistor
/// * `GPIO_PULL_DN_100K`: 100kΩ pull-down resistor
pub const GPIO_PULL_UP_0: u8 = 0;
pub const GPIO_PULL_UP_1M: u8 = 1;
pub const GPIO_PULL_UP_10K: u8 = 2;
pub const GPIO_PULL_DN_100K: u8 = 3;

/// GPIO pin direction enumeration.
///
/// Defines the possible directions for a GPIO pin:
/// * `IN`: Configures the pin as an input (value 0)
/// * `OUT`: Configures the pin as an output (value 1)
pub enum GpioDir {
    IN = 0,
    OUT = 1,
}

/// GPIO pin function type constants.
///
/// These constants define the possible functional modes for each GPIO pin:
/// * `AS_GPIO`: Standard GPIO function (input/output)
/// * `AS_MSPI`: SPI interface
/// * `AS_SWIRE`: Serial wire interface
/// * `AS_UART`: Serial UART interface
/// * `AS_PWM`: Pulse width modulation
/// * `AS_I2C`: I2C interface
/// * `AS_SPI`: SPI interface
/// * `AS_ETH_MAC`: Ethernet MAC interface
/// * `AS_I2S`: I2S audio interface
/// * `AS_SDM`: Sigma-delta modulation
/// * `AS_DMIC`: Digital microphone interface
/// * `AS_USB`: USB interface
/// * `AS_SWS`/`AS_SWM`: Serial wire debug
/// * `AS_TEST`: Test mode
/// * `AS_ADC`: Analog-to-digital converter
pub const AS_GPIO: u8 = 0; // Standard GPIO mode
pub const AS_MSPI: u8 = 1; // Master SPI interface
pub const AS_SWIRE: u8 = 2; // Serial wire interface
pub const AS_UART: u8 = 3; // Serial UART interface
pub const AS_PWM: u8 = 4; // Pulse width modulation
pub const AS_I2C: u8 = 5; // I2C interface
pub const AS_SPI: u8 = 6; // SPI interface
pub const AS_ETH_MAC: u8 = 7; // Ethernet MAC interface
pub const AS_I2S: u8 = 8; // I2S audio interface
pub const AS_SDM: u8 = 9; // Sigma-delta modulation
pub const AS_DMIC: u8 = 10; // Digital microphone interface
pub const AS_USB: u8 = 11; // USB interface
pub const AS_SWS: u8 = 12; // Serial wire debug (SWS)
pub const AS_SWM: u8 = 13; // Serial wire debug (SWM)
pub const AS_TEST: u8 = 14; // Test mode
pub const AS_ADC: u8 = 15; // Analog-to-digital converter

pub const PA0_INPUT_ENABLE: u8 = 1;
pub const PA1_INPUT_ENABLE: u8 = 0;
pub const PA2_INPUT_ENABLE: u8 = 1;
pub const PA3_INPUT_ENABLE: u8 = 1;
pub const PA4_INPUT_ENABLE: u8 = 1;
pub const PA5_INPUT_ENABLE: u8 = 0;
pub const PA6_INPUT_ENABLE: u8 = 1;
pub const PA7_INPUT_ENABLE: u8 = 1;
pub const PA0_OUTPUT_ENABLE: u8 = 0;
pub const PA1_OUTPUT_ENABLE: u8 = 1;
pub const PA2_OUTPUT_ENABLE: u8 = 0;
pub const PA3_OUTPUT_ENABLE: u8 = 0;
pub const PA4_OUTPUT_ENABLE: u8 = 0;
pub const PA5_OUTPUT_ENABLE: u8 = 1;
pub const PA6_OUTPUT_ENABLE: u8 = 0;
pub const PA7_OUTPUT_ENABLE: u8 = 0;
pub const PA0_DATA_STRENGTH: u8 = 1;
pub const PA1_DATA_STRENGTH: u8 = 1;
pub const PA2_DATA_STRENGTH: u8 = 1;
pub const PA3_DATA_STRENGTH: u8 = 1;
pub const PA4_DATA_STRENGTH: u8 = 1;
pub const PA5_DATA_STRENGTH: u8 = 1;
pub const PA6_DATA_STRENGTH: u8 = 1;
pub const PA7_DATA_STRENGTH: u8 = 1;
pub const PA0_DATA_OUT: u8 = 1;
//open SWS output to avoid MCU err
pub const PA1_DATA_OUT: u8 = 1; // PW1
pub const PA2_DATA_OUT: u8 = 0;
pub const PA3_DATA_OUT: u8 = 0;
pub const PA4_DATA_OUT: u8 = 0;
pub const PA5_DATA_OUT: u8 = 1; // PW2
pub const PA6_DATA_OUT: u8 = 0;
pub const PA7_DATA_OUT: u8 = 0;
pub const PA0_FUNC: u8 = AS_SWIRE;
pub const PA1_FUNC: u8 = AS_GPIO;
pub const PA2_FUNC: u8 = AS_MSPI;
pub const PA3_FUNC: u8 = AS_MSPI;
pub const PA4_FUNC: u8 = AS_GPIO;
pub const PA5_FUNC: u8 = AS_GPIO;
pub const PA6_FUNC: u8 = AS_GPIO;
pub const PA7_FUNC: u8 = AS_SWIRE;
pub const PULL_WAKEUP_SRC_PA0: u8 = PM_PIN_PULL_DEFAULT;
// SWS
pub const PULL_WAKEUP_SRC_PA1: u8 = 0;
// LIGHT_PWM must 0(float) or 3(pull_down_100K )
pub const PULL_WAKEUP_SRC_PA2: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PA3: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PA4: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PA5: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PA6: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PA7: u8 = PM_PIN_PULL_DEFAULT; // SWM

//////////////////////////////////////////////////
pub const PB0_INPUT_ENABLE: u8 = 1;
pub const PB1_INPUT_ENABLE: u8 = 1;
pub const PB2_INPUT_ENABLE: u8 = 1;
pub const PB3_INPUT_ENABLE: u8 = 1;
pub const PB4_INPUT_ENABLE: u8 = 1;
pub const PB5_INPUT_ENABLE: u8 = 1;
pub const PB6_INPUT_ENABLE: u8 = 1;
pub const PB7_INPUT_ENABLE: u8 = 1;
pub const PB0_OUTPUT_ENABLE: u8 = 0;
pub const PB1_OUTPUT_ENABLE: u8 = 0;
pub const PB2_OUTPUT_ENABLE: u8 = 0;
pub const PB3_OUTPUT_ENABLE: u8 = 0;
pub const PB4_OUTPUT_ENABLE: u8 = 0;
pub const PB5_OUTPUT_ENABLE: u8 = 0;
pub const PB6_OUTPUT_ENABLE: u8 = 0;
pub const PB7_OUTPUT_ENABLE: u8 = 0;
pub const PB0_DATA_STRENGTH: u8 = 1;
pub const PB1_DATA_STRENGTH: u8 = 1;
pub const PB2_DATA_STRENGTH: u8 = 1;
pub const PB3_DATA_STRENGTH: u8 = 1;
pub const PB4_DATA_STRENGTH: u8 = 1;
pub const PB5_DATA_STRENGTH: u8 = 1;
pub const PB6_DATA_STRENGTH: u8 = 1;
pub const PB7_DATA_STRENGTH: u8 = 1;
pub const PB0_DATA_OUT: u8 = 0;
pub const PB1_DATA_OUT: u8 = 0;
pub const PB2_DATA_OUT: u8 = 0;
pub const PB3_DATA_OUT: u8 = 0;
pub const PB4_DATA_OUT: u8 = 0;
pub const PB5_DATA_OUT: u8 = 0;
pub const PB6_DATA_OUT: u8 = 0;
pub const PB7_DATA_OUT: u8 = 0;
pub const PB0_FUNC: u8 = AS_GPIO;
pub const PB1_FUNC: u8 = AS_GPIO;
pub const PB2_FUNC: u8 = AS_MSPI;
pub const PB3_FUNC: u8 = AS_MSPI;
pub const PB4_FUNC: u8 = AS_GPIO;
pub const PB5_FUNC: u8 = AS_USB;
pub const PB6_FUNC: u8 = AS_USB;
pub const PB7_FUNC: u8 = AS_GPIO;
pub const PULL_WAKEUP_SRC_PB0: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PB1: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PB2: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PB3: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PB4: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PB5: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PB6: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PB7: u8 = PM_PIN_PULL_DEFAULT;

//////////////////////////////////////////////////
pub const PC0_INPUT_ENABLE: u8 = 1;
pub const PC1_INPUT_ENABLE: u8 = 1;
pub const PC2_INPUT_ENABLE: u8 = 1;
pub const PC3_INPUT_ENABLE: u8 = 1;
pub const PC4_INPUT_ENABLE: u8 = 1;
pub const PC5_INPUT_ENABLE: u8 = 1;
pub const PC6_INPUT_ENABLE: u8 = 1;
pub const PC7_INPUT_ENABLE: u8 = 1;
pub const PC0_OUTPUT_ENABLE: u8 = 0;
pub const PC1_OUTPUT_ENABLE: u8 = 0;
pub const PC2_OUTPUT_ENABLE: u8 = 0;
pub const PC3_OUTPUT_ENABLE: u8 = 0;
pub const PC4_OUTPUT_ENABLE: u8 = 0;
pub const PC5_OUTPUT_ENABLE: u8 = 0;
pub const PC6_OUTPUT_ENABLE: u8 = 0;
pub const PC7_OUTPUT_ENABLE: u8 = 0;
pub const PC0_DATA_STRENGTH: u8 = 1;
pub const PC1_DATA_STRENGTH: u8 = 1;
pub const PC2_DATA_STRENGTH: u8 = 1;
pub const PC3_DATA_STRENGTH: u8 = 1;
pub const PC4_DATA_STRENGTH: u8 = 1;
pub const PC5_DATA_STRENGTH: u8 = 1;
pub const PC6_DATA_STRENGTH: u8 = 1;
pub const PC7_DATA_STRENGTH: u8 = 1;
pub const PC0_DATA_OUT: u8 = 0;
pub const PC1_DATA_OUT: u8 = 0;
pub const PC2_DATA_OUT: u8 = 0;
pub const PC3_DATA_OUT: u8 = 0;
pub const PC4_DATA_OUT: u8 = 0;
pub const PC5_DATA_OUT: u8 = 0;
pub const PC6_DATA_OUT: u8 = 0;
pub const PC7_DATA_OUT: u8 = 0;
pub const PC0_FUNC: u8 = AS_GPIO;
pub const PC1_FUNC: u8 = AS_GPIO;
pub const PC2_FUNC: u8 = AS_GPIO;
pub const PC3_FUNC: u8 = AS_GPIO;
pub const PC4_FUNC: u8 = AS_GPIO;
pub const PC5_FUNC: u8 = AS_GPIO;
pub const PC6_FUNC: u8 = AS_GPIO;
pub const PC7_FUNC: u8 = AS_GPIO;
pub const PULL_WAKEUP_SRC_PC0: u8 = 0;
// LIGHT_PWM must 0(float) or 3(pull_down_100K )
pub const PULL_WAKEUP_SRC_PC1: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PC2: u8 = 0;
// LIGHT_PWM must 0(float) or 3(pull_down_100K )
pub const PULL_WAKEUP_SRC_PC3: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PC4: u8 = 0;
// LIGHT_PWM must 0(float) or 3(pull_down_100K )
pub const PULL_WAKEUP_SRC_PC5: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PC6: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PC7: u8 = PM_PIN_PULL_DEFAULT;

//////////////////////////////////////////////////
pub const PD0_INPUT_ENABLE: u8 = 1;
pub const PD1_INPUT_ENABLE: u8 = 1;
pub const PD2_INPUT_ENABLE: u8 = 1;
pub const PD3_INPUT_ENABLE: u8 = 1;
pub const PD4_INPUT_ENABLE: u8 = 1;
pub const PD5_INPUT_ENABLE: u8 = 1;
pub const PD6_INPUT_ENABLE: u8 = 1;
pub const PD7_INPUT_ENABLE: u8 = 1;
pub const PD0_OUTPUT_ENABLE: u8 = 0;
pub const PD1_OUTPUT_ENABLE: u8 = 0;
pub const PD2_OUTPUT_ENABLE: u8 = 0;
pub const PD3_OUTPUT_ENABLE: u8 = 0;
pub const PD4_OUTPUT_ENABLE: u8 = 0;
pub const PD5_OUTPUT_ENABLE: u8 = 0;
pub const PD6_OUTPUT_ENABLE: u8 = 0;
pub const PD7_OUTPUT_ENABLE: u8 = 0;
pub const PD0_DATA_STRENGTH: u8 = 1;
pub const PD1_DATA_STRENGTH: u8 = 1;
pub const PD2_DATA_STRENGTH: u8 = 1;
pub const PD3_DATA_STRENGTH: u8 = 1;
pub const PD4_DATA_STRENGTH: u8 = 1;
pub const PD5_DATA_STRENGTH: u8 = 1;
pub const PD6_DATA_STRENGTH: u8 = 1;
pub const PD7_DATA_STRENGTH: u8 = 1;
pub const PD0_DATA_OUT: u8 = 0;
pub const PD1_DATA_OUT: u8 = 0;
pub const PD2_DATA_OUT: u8 = 0;
pub const PD3_DATA_OUT: u8 = 0;
pub const PD4_DATA_OUT: u8 = 0;
pub const PD5_DATA_OUT: u8 = 0;
pub const PD6_DATA_OUT: u8 = 0;
pub const PD7_DATA_OUT: u8 = 0;
pub const PD0_FUNC: u8 = AS_GPIO;
pub const PD1_FUNC: u8 = AS_GPIO;
pub const PD2_FUNC: u8 = AS_GPIO;
pub const PD3_FUNC: u8 = AS_GPIO;
pub const PD4_FUNC: u8 = AS_GPIO;
pub const PD5_FUNC: u8 = AS_GPIO;
pub const PD6_FUNC: u8 = AS_GPIO;
pub const PD7_FUNC: u8 = AS_GPIO;
pub const PULL_WAKEUP_SRC_PD0: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PD1: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PD2: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PD3: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PD4: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PD5: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PD6: u8 = PM_PIN_PULL_DEFAULT;
pub const PULL_WAKEUP_SRC_PD7: u8 = PM_PIN_PULL_DEFAULT;

//////////////////////////////////////////////////
pub const PE0_INPUT_ENABLE: u8 = 1;
pub const PE1_INPUT_ENABLE: u8 = 1;
pub const PE2_INPUT_ENABLE: u8 = 1;
pub const PE3_INPUT_ENABLE: u8 = 1;
pub const PE4_INPUT_ENABLE: u8 = 1;
pub const PE5_INPUT_ENABLE: u8 = 1;
pub const PE6_INPUT_ENABLE: u8 = 1;
pub const PE7_INPUT_ENABLE: u8 = 1;
pub const PE0_OUTPUT_ENABLE: u8 = 0;
pub const PE1_OUTPUT_ENABLE: u8 = 0;
pub const PE2_OUTPUT_ENABLE: u8 = 0;
pub const PE3_OUTPUT_ENABLE: u8 = 0;
pub const PE4_OUTPUT_ENABLE: u8 = 0;
pub const PE5_OUTPUT_ENABLE: u8 = 0;
pub const PE6_OUTPUT_ENABLE: u8 = 0;
pub const PE7_OUTPUT_ENABLE: u8 = 0;
pub const PE0_DATA_STRENGTH: u8 = 1;
pub const PE1_DATA_STRENGTH: u8 = 1;
pub const PE2_DATA_STRENGTH: u8 = 1;
pub const PE3_DATA_STRENGTH: u8 = 1;
pub const PE4_DATA_STRENGTH: u8 = 1;
pub const PE5_DATA_STRENGTH: u8 = 1;
pub const PE6_DATA_STRENGTH: u8 = 1;
pub const PE7_DATA_STRENGTH: u8 = 1;
pub const PE0_DATA_OUT: u8 = 0;
pub const PE1_DATA_OUT: u8 = 0;
pub const PE2_DATA_OUT: u8 = 0;
pub const PE3_DATA_OUT: u8 = 0;
pub const PE4_DATA_OUT: u8 = 0;
pub const PE5_DATA_OUT: u8 = 0;
pub const PE6_DATA_OUT: u8 = 0;
pub const PE7_DATA_OUT: u8 = 0;
pub const PE0_FUNC: u8 = AS_GPIO;
pub const PE1_FUNC: u8 = AS_GPIO;
pub const PE2_FUNC: u8 = AS_GPIO;
pub const PE3_FUNC: u8 = AS_GPIO;
pub const PE4_FUNC: u8 = AS_GPIO;
pub const PE5_FUNC: u8 = AS_GPIO;
pub const PE6_FUNC: u8 = AS_GPIO;
pub const PE7_FUNC: u8 = AS_GPIO;
pub const PULL_WAKEUP_SRC_PE0: u8 = 0;
pub const PULL_WAKEUP_SRC_PE1: u8 = 0;
pub const PULL_WAKEUP_SRC_PE2: u8 = 0;
pub const PULL_WAKEUP_SRC_PE3: u8 = 0;
pub const PULL_WAKEUP_SRC_PE4: u8 = 0;
pub const PULL_WAKEUP_SRC_PE5: u8 = 0;
pub const PULL_WAKEUP_SRC_PE6: u8 = 0;
pub const PULL_WAKEUP_SRC_PE7: u8 = 0;

//////////////////////////////////////////////////
pub const PF0_INPUT_ENABLE: u8 = 1;
pub const PF1_INPUT_ENABLE: u8 = 1;
pub const PF0_OUTPUT_ENABLE: u8 = 0;
pub const PF1_OUTPUT_ENABLE: u8 = 0;
pub const PF0_DATA_STRENGTH: u8 = 1;
pub const PF1_DATA_STRENGTH: u8 = 1;
pub const PF0_DATA_OUT: u8 = 0;
pub const PF1_DATA_OUT: u8 = 0;
pub const PF0_FUNC: u8 = AS_GPIO;
pub const PF1_FUNC: u8 = AS_GPIO;
pub const PULL_WAKEUP_SRC_PF0: u8 = 0;
pub const PULL_WAKEUP_SRC_PF1: u8 = 0;

pub const PM_PIN_PULLUP_1M: u8 = 1;
pub const PM_PIN_PULLUP_10K: u8 = 2;
pub const PM_PIN_PULLDOWN_100K: u8 = 3;
pub const PM_PIN_UP_DOWN_FLOAT: u8 = 0xff;

/// Enumeration of all GPIO pins in the microcontroller.
///
/// This enumeration provides type-safe access to all GPIO pins.
/// Each pin is encoded with:
/// - Port number (high byte: 0-5 for ports A-F)
/// - Pin bit mask (low byte: bit position 0-7)
///
/// The encoding format allows for efficient pin manipulation by separating
/// the port number and bit position, which can be extracted using bit operations.
///
/// The type and variant names are the datasheet pin names, so the datasheet
/// spelling is kept as-is; only the irreducible `#[allow]` for that spelling is
/// applied.
#[allow(non_camel_case_types)]
pub enum GPIO_PIN_TYPE {
    GPIO_PA0 = BIT!(0), // Port A, pin 0
    GPIO_PA1 = BIT!(1), // Port A, pin 1
    GPIO_PA2 = BIT!(2), // Port A, pin 2
    GPIO_PA3 = BIT!(3), // Port A, pin 3
    GPIO_PA4 = BIT!(4), // Port A, pin 4
    GPIO_PA5 = BIT!(5), // Port A, pin 5
    GPIO_PA6 = BIT!(6), // Port A, pin 6
    GPIO_PA7 = BIT!(7), // Port A, pin 7

    GPIO_PB0 = 0x100 | BIT!(0), // Port B, pin 0
    GPIO_PB1 = 0x100 | BIT!(1), // Port B, pin 1
    GPIO_PB2 = 0x100 | BIT!(2), // Port B, pin 2
    GPIO_PB3 = 0x100 | BIT!(3), // Port B, pin 3
    GPIO_PB4 = 0x100 | BIT!(4), // Port B, pin 4
    GPIO_PB5 = 0x100 | BIT!(5), // Port B, pin 5
    GPIO_PB6 = 0x100 | BIT!(6), // Port B, pin 6
    GPIO_PB7 = 0x100 | BIT!(7), // Port B, pin 7

    GPIO_PC0 = 0x200 | BIT!(0), // Port C, pin 0
    GPIO_PC1 = 0x200 | BIT!(1), // Port C, pin 1
    GPIO_PC2 = 0x200 | BIT!(2), // Port C, pin 2
    GPIO_PC3 = 0x200 | BIT!(3), // Port C, pin 3
    GPIO_PC4 = 0x200 | BIT!(4), // Port C, pin 4
    GPIO_PC5 = 0x200 | BIT!(5), // Port C, pin 5
    GPIO_PC6 = 0x200 | BIT!(6), // Port C, pin 6
    GPIO_PC7 = 0x200 | BIT!(7), // Port C, pin 7

    GPIO_PD0 = 0x300 | BIT!(0), // Port D, pin 0
    GPIO_PD1 = 0x300 | BIT!(1), // Port D, pin 1
    GPIO_PD2 = 0x300 | BIT!(2), // Port D, pin 2
    GPIO_PD3 = 0x300 | BIT!(3), // Port D, pin 3
    GPIO_PD4 = 0x300 | BIT!(4), // Port D, pin 4
    GPIO_PD5 = 0x300 | BIT!(5), // Port D, pin 5
    GPIO_PD6 = 0x300 | BIT!(6), // Port D, pin 6
    GPIO_PD7 = 0x300 | BIT!(7), // Port D, pin 7

    GPIO_PE0 = 0x400 | BIT!(0), // Port E, pin 0
    GPIO_PE1 = 0x400 | BIT!(1), // Port E, pin 1
    GPIO_PE2 = 0x400 | BIT!(2), // Port E, pin 2
    GPIO_PE3 = 0x400 | BIT!(3), // Port E, pin 3
    GPIO_PE4 = 0x400 | BIT!(4), // Port E, pin 4
    GPIO_PE5 = 0x400 | BIT!(5), // Port E, pin 5
    GPIO_PE6 = 0x400 | BIT!(6), // Port E, pin 6
    GPIO_PE7 = 0x400 | BIT!(7), // Port E, pin 7

    GPIO_PF0 = 0x500 | BIT!(0), // Port F, pin 0
    GPIO_PF1 = 0x500 | BIT!(1), // Port F, pin 1

    GPIO_MAX_COUNT = 56, // Total number of GPIO pins
}

/// Functional aliases for GPIO pins.
///
/// This implementation block defines named aliases for GPIO pins based on their
/// default or commonly used peripheral functions. These aliases make code more
/// readable by referring to pins by their functional role rather than just
/// their port/pin designations.
///
/// The aliases are grouped by functionality:
/// - Serial wire debug: GPIO_SWS, GPIO_SWM
/// - PWM channels: GPIO_PWMxyz (various PWM channels and complementary outputs)
/// - SPI/MSPI: GPIO_MSDI, GPIO_MSDO, GPIO_MSCN, GPIO_MCLK
/// - USB: GPIO_DM, GPIO_DP
/// - UART: GPIO_UTX, GPIO_URX
/// - I2C/SPI: GPIO_CN, GPIO_DI, GPIO_DO, GPIO_CK
/// - Digital microphone: GPIO_DMIC_CK, GPIO_DMIC_DI
/// - General purpose: GPIO_GPx (numbered general purpose pins)
impl GPIO_PIN_TYPE {
    // Serial wire debug interface
    pub const GPIO_SWS: GPIO_PIN_TYPE = GPIO_PA0; // Serial wire signal
    pub const GPIO_SWM: GPIO_PIN_TYPE = GPIO_PA7; // Serial wire mode

    // PWM channels
    pub const GPIO_PWM3A1: GPIO_PIN_TYPE = GPIO_PA1; // PWM3 channel A
    pub const GPIO_PWM3NA4: GPIO_PIN_TYPE = GPIO_PA4; // PWM3 channel A complementary
    pub const GPIO_PWM4A5: GPIO_PIN_TYPE = GPIO_PA5; // PWM4 channel A
    pub const GPIO_PWM4NA6: GPIO_PIN_TYPE = GPIO_PA6; // PWM4 channel A complementary
    pub const GPIO_PWM5B0: GPIO_PIN_TYPE = GPIO_PB0; // PWM5 channel B
    pub const GPIO_PWM5NB1: GPIO_PIN_TYPE = GPIO_PB1; // PWM5 channel B complementary
    pub const GPIO_PWM0NB7: GPIO_PIN_TYPE = GPIO_PB7; // PWM0 channel B complementary
    pub const GPIO_PWM0C0: GPIO_PIN_TYPE = GPIO_PC0; // PWM0 channel C
    pub const GPIO_PWM1NC1: GPIO_PIN_TYPE = GPIO_PC1; // PWM1 channel C complementary
    pub const GPIO_PWM1NC2: GPIO_PIN_TYPE = GPIO_PC2; // PWM1 channel C complementary
    pub const GPIO_PWM1C3: GPIO_PIN_TYPE = GPIO_PC3; // PWM1 channel C
    pub const GPIO_PWM2C4: GPIO_PIN_TYPE = GPIO_PC4; // PWM2 channel C
    pub const GPIO_PWM2NC5: GPIO_PIN_TYPE = GPIO_PC5; // PWM2 channel C complementary
    pub const GPIO_PWM3D2: GPIO_PIN_TYPE = GPIO_PD2; // PWM3 channel D
    pub const GPIO_PWM4D3: GPIO_PIN_TYPE = GPIO_PD3; // PWM4 channel D

    // MSPI (Master SPI) interface
    pub const GPIO_MSDI: GPIO_PIN_TYPE = GPIO_PA2; // MSPI data in
    pub const GPIO_MCLK: GPIO_PIN_TYPE = GPIO_PA3; // MSPI clock
    pub const GPIO_MSDO: GPIO_PIN_TYPE = GPIO_PB2; // MSPI data out
    pub const GPIO_MSCN: GPIO_PIN_TYPE = GPIO_PB3; // MSPI chip select

    // USB interface
    pub const GPIO_DM: GPIO_PIN_TYPE = GPIO_PB5; // USB D-
    pub const GPIO_DP: GPIO_PIN_TYPE = GPIO_PB6; // USB D+

    // UART interface
    pub const GPIO_UTX: GPIO_PIN_TYPE = GPIO_PC6; // UART TX
    pub const GPIO_URX: GPIO_PIN_TYPE = GPIO_PC7; // UART RX

    // I2C/SPI interface
    pub const GPIO_CN: GPIO_PIN_TYPE = GPIO_PE6; // I2C/SPI chip select
    pub const GPIO_DI: GPIO_PIN_TYPE = GPIO_PE7; // I2C/SPI data in
    pub const GPIO_DO: GPIO_PIN_TYPE = GPIO_PF0; // I2C/SPI data out
    pub const GPIO_CK: GPIO_PIN_TYPE = GPIO_PF1; // I2C/SPI clock

    // Digital microphone interface
    pub const GPIO_DMIC_CK: GPIO_PIN_TYPE = GPIO_PE1; // DMIC clock
    pub const GPIO_DMIC_DI: GPIO_PIN_TYPE = GPIO_PE2; // DMIC data in

    // General purpose (GP) pins
    pub const GPIO_GP0: GPIO_PIN_TYPE = GPIO_PB7;
    pub const GPIO_GP1: GPIO_PIN_TYPE = GPIO_PC1;
    pub const GPIO_GP2: GPIO_PIN_TYPE = GPIO_PC3;
    pub const GPIO_GP3: GPIO_PIN_TYPE = GPIO_PC5;
    pub const GPIO_GP4: GPIO_PIN_TYPE = GPIO_PC6;
    pub const GPIO_GP5: GPIO_PIN_TYPE = GPIO_PC7;
    pub const GPIO_GP6: GPIO_PIN_TYPE = GPIO_PD0;
    pub const GPIO_GP7: GPIO_PIN_TYPE = GPIO_PD1;
    pub const GPIO_GP8: GPIO_PIN_TYPE = GPIO_PD2;
    pub const GPIO_GP9: GPIO_PIN_TYPE = GPIO_PD3;
    pub const GPIO_GP10: GPIO_PIN_TYPE = GPIO_PD4;
    pub const GPIO_GP11: GPIO_PIN_TYPE = GPIO_PD5;
    pub const GPIO_GP12: GPIO_PIN_TYPE = GPIO_PD6;
    pub const GPIO_GP13: GPIO_PIN_TYPE = GPIO_PD7;
    pub const GPIO_GP14: GPIO_PIN_TYPE = GPIO_PE0;
    pub const GPIO_GP15: GPIO_PIN_TYPE = GPIO_PE3;
    pub const GPIO_GP16: GPIO_PIN_TYPE = GPIO_PE4;
    pub const GPIO_GP17: GPIO_PIN_TYPE = GPIO_PE5;
    pub const GPIO_GP18: GPIO_PIN_TYPE = GPIO_PA4;
    pub const GPIO_GP19: GPIO_PIN_TYPE = GPIO_PA6;
    pub const GPIO_GP20: GPIO_PIN_TYPE = GPIO_PB1;
    pub const GPIO_GP21: GPIO_PIN_TYPE = GPIO_PB4;
}

/// Initializes all GPIO pins based on the predefined constants.
///
/// This function configures the entire GPIO system according to the predefined constants
/// for each pin port (PA through PF). Only the digital properties are programmed; the
/// vendor SDK's analog pull-up/pull-down step never ran and is not compiled.
///
/// # Step 1: Configure digital GPIO properties
/// For each port (PA-PF), configures:
/// - Input enable status (1=enabled, 0=disabled)
/// - Output enable status (inverted in register: 0=enabled, 1=disabled)
/// - Initial output values (1=high, 0=low)
/// - Drive strength (typically 1=strong drive)
/// - Function selection (GPIO vs peripheral functions)
///
/// # Step 2: Configure analog properties (not performed)
/// The vendor SDK also programs each pin's pull-up/pull-down resistor through
/// analog registers `0x0a..0x14`. That tail sits behind an early `return` in the
/// ported SDK code and has never run on device, so it is not part of the
/// compiled path; see the note at the end of the function body.
///
/// # Notes
///
/// - Register bits 8-15: Input enable flags
/// - Register bits 16-23: Output enable flags (inverted logic)
/// - Register bits 24-31: Initial output values
/// - Setting2 registers bits 8-15: Drive strength configuration
/// - Setting2 registers bits 16-23: Function selection (GPIO vs peripherals)
/// - The special-function combination configuration is not ported from the vendor SDK
#[coverage(off)]
pub fn gpio_init() {
    // --- Configure Port A (PA0-PA7) GPIO settings ---
    write_reg_gpio_pa_setting1(
        ((PA0_INPUT_ENABLE as u32) << 8)     // PA0 input enable
            | ((PA1_INPUT_ENABLE as u32) << 9)     // PA1 input enable
            | ((PA2_INPUT_ENABLE as u32) << 10)    // PA2 input enable
            | ((PA3_INPUT_ENABLE as u32) << 11)    // PA3 input enable
            | ((PA4_INPUT_ENABLE as u32) << 12)    // PA4 input enable
            | ((PA5_INPUT_ENABLE as u32) << 13)    // PA5 input enable
            | ((PA6_INPUT_ENABLE as u32) << 14)    // PA6 input enable
            | ((PA7_INPUT_ENABLE as u32) << 15)    // PA7 input enable
            | ((if PA0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)  // PA0 output enable (inverted)
            | ((if PA1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)  // PA1 output enable (inverted)
            | ((if PA2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)  // PA2 output enable (inverted)
            | ((if PA3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)  // PA3 output enable (inverted)
            | ((if PA4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)  // PA4 output enable (inverted)
            | ((if PA5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)  // PA5 output enable (inverted)
            | ((if PA6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)  // PA6 output enable (inverted)
            | ((if PA7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)  // PA7 output enable (inverted)
            | ((PA0_DATA_OUT as u32) << 24)  // PA0 initial output value
            | ((PA1_DATA_OUT as u32) << 25)  // PA1 initial output value
            | ((PA2_DATA_OUT as u32) << 26)  // PA2 initial output value
            | ((PA3_DATA_OUT as u32) << 27)  // PA3 initial output value
            | ((PA4_DATA_OUT as u32) << 28)  // PA4 initial output value
            | ((PA5_DATA_OUT as u32) << 29)  // PA5 initial output value
            | ((PA6_DATA_OUT as u32) << 30)  // PA6 initial output value
            | ((PA7_DATA_OUT as u32) << 31), // PA7 initial output value
    );

    write_reg_gpio_pa_setting2(
        ((PA0_DATA_STRENGTH as u32) << 8)    // PA0 drive strength
            | ((PA1_DATA_STRENGTH as u32) << 9)    // PA1 drive strength
            | ((PA2_DATA_STRENGTH as u32) << 10)   // PA2 drive strength
            | ((PA3_DATA_STRENGTH as u32) << 11)   // PA3 drive strength
            | ((PA4_DATA_STRENGTH as u32) << 12)   // PA4 drive strength
            | ((PA5_DATA_STRENGTH as u32) << 13)   // PA5 drive strength
            | ((PA6_DATA_STRENGTH as u32) << 14)   // PA6 drive strength
            | ((PA7_DATA_STRENGTH as u32) << 15)   // PA7 drive strength
            | (if PA0_FUNC == AS_GPIO { BIT!(16) } else { 0 })  // PA0 function: 1=GPIO, 0=peripheral
            | (if PA1_FUNC == AS_GPIO { BIT!(17) } else { 0 })  // PA1 function: 1=GPIO, 0=peripheral
            | (if PA2_FUNC == AS_GPIO { BIT!(18) } else { 0 })  // PA2 function: 1=GPIO, 0=peripheral
            | (if PA3_FUNC == AS_GPIO { BIT!(19) } else { 0 })  // PA3 function: 1=GPIO, 0=peripheral
            | (if PA4_FUNC == AS_GPIO { BIT!(20) } else { 0 })  // PA4 function: 1=GPIO, 0=peripheral
            | (if PA5_FUNC == AS_GPIO { BIT!(21) } else { 0 })  // PA5 function: 1=GPIO, 0=peripheral
            | (if PA6_FUNC == AS_GPIO { BIT!(22) } else { 0 })  // PA6 function: 1=GPIO, 0=peripheral
            | (if PA7_FUNC == AS_GPIO { BIT!(23) } else { 0 }), // PA7 function: 1=GPIO, 0=peripheral
    );

    // --- Configure Port B (PB0-PB7) GPIO settings ---
    write_reg_gpio_pb_setting1(
        ((PB0_INPUT_ENABLE as u32) << 8)     // PB0 input enable
            | ((PB1_INPUT_ENABLE as u32) << 9)     // PB1 input enable
            | ((PB2_INPUT_ENABLE as u32) << 10)    // PB2 input enable
            | ((PB3_INPUT_ENABLE as u32) << 11)    // PB3 input enable
            | ((PB4_INPUT_ENABLE as u32) << 12)    // PB4 input enable
            | ((PB5_INPUT_ENABLE as u32) << 13)    // PB5 input enable
            | ((PB6_INPUT_ENABLE as u32) << 14)    // PB6 input enable
            | ((PB7_INPUT_ENABLE as u32) << 15)    // PB7 input enable
            | ((if PB0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)  // PB0 output enable (inverted)
            | ((if PB1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)  // PB1 output enable (inverted)
            | ((if PB2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)  // PB2 output enable (inverted)
            | ((if PB3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)  // PB3 output enable (inverted)
            | ((if PB4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)  // PB4 output enable (inverted)
            | ((if PB5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)  // PB5 output enable (inverted)
            | ((if PB6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)  // PB6 output enable (inverted)
            | ((if PB7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)  // PB7 output enable (inverted)
            | ((PB0_DATA_OUT as u32) << 24)  // PB0 initial output value
            | ((PB1_DATA_OUT as u32) << 25)  // PB1 initial output value
            | ((PB2_DATA_OUT as u32) << 26)  // PB2 initial output value
            | ((PB3_DATA_OUT as u32) << 27)  // PB3 initial output value
            | ((PB4_DATA_OUT as u32) << 28)  // PB4 initial output value
            | ((PB5_DATA_OUT as u32) << 29)  // PB5 initial output value
            | ((PB6_DATA_OUT as u32) << 30)  // PB6 initial output value
            | ((PB7_DATA_OUT as u32) << 31), // PB7 initial output value
    );

    write_reg_gpio_pb_setting2(
        ((PB0_DATA_STRENGTH as u32) << 8)    // PB0 drive strength
            | ((PB1_DATA_STRENGTH as u32) << 9)    // PB1 drive strength
            | ((PB2_DATA_STRENGTH as u32) << 10)   // PB2 drive strength
            | ((PB3_DATA_STRENGTH as u32) << 11)   // PB3 drive strength
            | ((PB4_DATA_STRENGTH as u32) << 12)   // PB4 drive strength
            | ((PB5_DATA_STRENGTH as u32) << 13)   // PB5 drive strength
            | ((PB6_DATA_STRENGTH as u32) << 14)   // PB6 drive strength
            | ((PB7_DATA_STRENGTH as u32) << 15)   // PB7 drive strength
            | (if PB0_FUNC == AS_GPIO { BIT!(16) } else { 0 })  // PB0 function: 1=GPIO, 0=peripheral
            | (if PB1_FUNC == AS_GPIO { BIT!(17) } else { 0 })  // PB1 function: 1=GPIO, 0=peripheral
            | (if PB2_FUNC == AS_GPIO { BIT!(18) } else { 0 })  // PB2 function: 1=GPIO, 0=peripheral
            | (if PB3_FUNC == AS_GPIO { BIT!(19) } else { 0 })  // PB3 function: 1=GPIO, 0=peripheral
            | (if PB4_FUNC == AS_GPIO { BIT!(20) } else { 0 })  // PB4 function: 1=GPIO, 0=peripheral
            | (if PB5_FUNC == AS_GPIO { BIT!(21) } else { 0 })  // PB5 function: 1=GPIO, 0=peripheral
            | (if PB6_FUNC == AS_GPIO { BIT!(22) } else { 0 })  // PB6 function: 1=GPIO, 0=peripheral
            | (if PB7_FUNC == AS_GPIO { BIT!(23) } else { 0 }), // PB7 function: 1=GPIO, 0=peripheral
    );

    // --- Configure Port C (PC0-PC7) GPIO settings ---
    write_reg_gpio_pc_setting1(
        ((PC0_INPUT_ENABLE as u32) << 8)     // PC0 input enable
            | ((PC1_INPUT_ENABLE as u32) << 9)     // PC1 input enable
            | ((PC2_INPUT_ENABLE as u32) << 10)    // PC2 input enable
            | ((PC3_INPUT_ENABLE as u32) << 11)    // PC3 input enable
            | ((PC4_INPUT_ENABLE as u32) << 12)    // PC4 input enable
            | ((PC5_INPUT_ENABLE as u32) << 13)    // PC5 input enable
            | ((PC6_INPUT_ENABLE as u32) << 14)    // PC6 input enable
            | ((PC7_INPUT_ENABLE as u32) << 15)    // PC7 input enable
            | ((if PC0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)  // PC0 output enable (inverted)
            | ((if PC1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)  // PC1 output enable (inverted)
            | ((if PC2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)  // PC2 output enable (inverted)
            | ((if PC3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)  // PC3 output enable (inverted)
            | ((if PC4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)  // PC4 output enable (inverted)
            | ((if PC5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)  // PC5 output enable (inverted)
            | ((if PC6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)  // PC6 output enable (inverted)
            | ((if PC7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)  // PC7 output enable (inverted)
            | ((PC0_DATA_OUT as u32) << 24)  // PC0 initial output value
            | ((PC1_DATA_OUT as u32) << 25)  // PC1 initial output value
            | ((PC2_DATA_OUT as u32) << 26)  // PC2 initial output value
            | ((PC3_DATA_OUT as u32) << 27)  // PC3 initial output value
            | ((PC4_DATA_OUT as u32) << 28)  // PC4 initial output value
            | ((PC5_DATA_OUT as u32) << 29)  // PC5 initial output value
            | ((PC6_DATA_OUT as u32) << 30)  // PC6 initial output value
            | ((PC7_DATA_OUT as u32) << 31), // PC7 initial output value
    );

    write_reg_gpio_pc_setting2(
        ((PC0_DATA_STRENGTH as u32) << 8)    // PC0 drive strength
            | ((PC1_DATA_STRENGTH as u32) << 9)    // PC1 drive strength
            | ((PC2_DATA_STRENGTH as u32) << 10)   // PC2 drive strength
            | ((PC3_DATA_STRENGTH as u32) << 11)   // PC3 drive strength
            | ((PC4_DATA_STRENGTH as u32) << 12)   // PC4 drive strength
            | ((PC5_DATA_STRENGTH as u32) << 13)   // PC5 drive strength
            | ((PC6_DATA_STRENGTH as u32) << 14)   // PC6 drive strength
            | ((PC7_DATA_STRENGTH as u32) << 15)   // PC7 drive strength
            | (if PC0_FUNC == AS_GPIO { BIT!(16) } else { 0 })  // PC0 function: 1=GPIO, 0=peripheral
            | (if PC1_FUNC == AS_GPIO { BIT!(17) } else { 0 })  // PC1 function: 1=GPIO, 0=peripheral
            | (if PC2_FUNC == AS_GPIO { BIT!(18) } else { 0 })  // PC2 function: 1=GPIO, 0=peripheral
            | (if PC3_FUNC == AS_GPIO { BIT!(19) } else { 0 })  // PC3 function: 1=GPIO, 0=peripheral
            | (if PC4_FUNC == AS_GPIO { BIT!(20) } else { 0 })  // PC4 function: 1=GPIO, 0=peripheral
            | (if PC5_FUNC == AS_GPIO { BIT!(21) } else { 0 })  // PC5 function: 1=GPIO, 0=peripheral
            | (if PC6_FUNC == AS_GPIO { BIT!(22) } else { 0 })  // PC6 function: 1=GPIO, 0=peripheral
            | (if PC7_FUNC == AS_GPIO { BIT!(23) } else { 0 }), // PC7 function: 1=GPIO, 0=peripheral
    );

    // --- Configure Port D (PD0-PD7) GPIO settings ---
    write_reg_gpio_pd_setting1(
        ((PD0_INPUT_ENABLE as u32) << 8)     // PD0 input enable
            | ((PD1_INPUT_ENABLE as u32) << 9)     // PD1 input enable
            | ((PD2_INPUT_ENABLE as u32) << 10)    // PD2 input enable
            | ((PD3_INPUT_ENABLE as u32) << 11)    // PD3 input enable
            | ((PD4_INPUT_ENABLE as u32) << 12)    // PD4 input enable
            | ((PD5_INPUT_ENABLE as u32) << 13)    // PD5 input enable
            | ((PD6_INPUT_ENABLE as u32) << 14)    // PD6 input enable
            | ((PD7_INPUT_ENABLE as u32) << 15)    // PD7 input enable
            | ((if PD0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)  // PD0 output enable (inverted)
            | ((if PD1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)  // PD1 output enable (inverted)
            | ((if PD2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)  // PD2 output enable (inverted)
            | ((if PD3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)  // PD3 output enable (inverted)
            | ((if PD4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)  // PD4 output enable (inverted)
            | ((if PD5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)  // PD5 output enable (inverted)
            | ((if PD6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)  // PD6 output enable (inverted)
            | ((if PD7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)  // PD7 output enable (inverted)
            | ((PD0_DATA_OUT as u32) << 24)  // PD0 initial output value
            | ((PD1_DATA_OUT as u32) << 25)  // PD1 initial output value
            | ((PD2_DATA_OUT as u32) << 26)  // PD2 initial output value
            | ((PD3_DATA_OUT as u32) << 27)  // PD3 initial output value
            | ((PD4_DATA_OUT as u32) << 28)  // PD4 initial output value
            | ((PD5_DATA_OUT as u32) << 29)  // PD5 initial output value
            | ((PD6_DATA_OUT as u32) << 30)  // PD6 initial output value
            | ((PD7_DATA_OUT as u32) << 31), // PD7 initial output value
    );

    write_reg_gpio_pd_setting2(
        ((PD0_DATA_STRENGTH as u32) << 8)    // PD0 drive strength
            | ((PD1_DATA_STRENGTH as u32) << 9)    // PD1 drive strength
            | ((PD2_DATA_STRENGTH as u32) << 10)   // PD2 drive strength
            | ((PD3_DATA_STRENGTH as u32) << 11)   // PD3 drive strength
            | ((PD4_DATA_STRENGTH as u32) << 12)   // PD4 drive strength
            | ((PD5_DATA_STRENGTH as u32) << 13)   // PD5 drive strength
            | ((PD6_DATA_STRENGTH as u32) << 14)   // PD6 drive strength
            | ((PD7_DATA_STRENGTH as u32) << 15)   // PD7 drive strength
            | (if PD0_FUNC == AS_GPIO { BIT!(16) } else { 0 })  // PD0 function: 1=GPIO, 0=peripheral
            | (if PD1_FUNC == AS_GPIO { BIT!(17) } else { 0 })  // PD1 function: 1=GPIO, 0=peripheral
            | (if PD2_FUNC == AS_GPIO { BIT!(18) } else { 0 })  // PD2 function: 1=GPIO, 0=peripheral
            | (if PD3_FUNC == AS_GPIO { BIT!(19) } else { 0 })  // PD3 function: 1=GPIO, 0=peripheral
            | (if PD4_FUNC == AS_GPIO { BIT!(20) } else { 0 })  // PD4 function: 1=GPIO, 0=peripheral
            | (if PD5_FUNC == AS_GPIO { BIT!(21) } else { 0 })  // PD5 function: 1=GPIO, 0=peripheral
            | (if PD6_FUNC == AS_GPIO { BIT!(22) } else { 0 })  // PD6 function: 1=GPIO, 0=peripheral
            | (if PD7_FUNC == AS_GPIO { BIT!(23) } else { 0 }), // PD7 function: 1=GPIO, 0=peripheral
    );

    // --- Configure Port E (PE0-PE7) GPIO settings ---
    write_reg_gpio_pe_setting1(
        ((PE0_INPUT_ENABLE as u32) << 8)     // PE0 input enable
            | ((PE1_INPUT_ENABLE as u32) << 9)     // PE1 input enable
            | ((PE2_INPUT_ENABLE as u32) << 10)    // PE2 input enable
            | ((PE3_INPUT_ENABLE as u32) << 11)    // PE3 input enable
            | ((PE4_INPUT_ENABLE as u32) << 12)    // PE4 input enable
            | ((PE5_INPUT_ENABLE as u32) << 13)    // PE5 input enable
            | ((PE6_INPUT_ENABLE as u32) << 14)    // PE6 input enable
            | ((PE7_INPUT_ENABLE as u32) << 15)    // PE7 input enable
            | ((if PE0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)  // PE0 output enable (inverted)
            | ((if PE1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)  // PE1 output enable (inverted)
            | ((if PE2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)  // PE2 output enable (inverted)
            | ((if PE3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)  // PE3 output enable (inverted)
            | ((if PE4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)  // PE4 output enable (inverted)
            | ((if PE5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)  // PE5 output enable (inverted)
            | ((if PE6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)  // PE6 output enable (inverted)
            | ((if PE7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)  // PE7 output enable (inverted)
            | ((PE0_DATA_OUT as u32) << 24)  // PE0 initial output value
            | ((PE1_DATA_OUT as u32) << 25)  // PE1 initial output value
            | ((PE2_DATA_OUT as u32) << 26)  // PE2 initial output value
            | ((PE3_DATA_OUT as u32) << 27)  // PE3 initial output value
            | ((PE4_DATA_OUT as u32) << 28)  // PE4 initial output value
            | ((PE5_DATA_OUT as u32) << 29)  // PE5 initial output value
            | ((PE6_DATA_OUT as u32) << 30)  // PE6 initial output value
            | ((PE7_DATA_OUT as u32) << 31), // PE7 initial output value
    );

    write_reg_gpio_pe_setting2(
        ((PE0_DATA_STRENGTH as u32) << 8)    // PE0 drive strength
            | ((PE1_DATA_STRENGTH as u32) << 9)    // PE1 drive strength
            | ((PE2_DATA_STRENGTH as u32) << 10)   // PE2 drive strength
            | ((PE3_DATA_STRENGTH as u32) << 11)   // PE3 drive strength
            | ((PE4_DATA_STRENGTH as u32) << 12)   // PE4 drive strength
            | ((PE5_DATA_STRENGTH as u32) << 13)   // PE5 drive strength
            | ((PE6_DATA_STRENGTH as u32) << 14)   // PE6 drive strength
            | ((PE7_DATA_STRENGTH as u32) << 15)   // PE7 drive strength
            | (if PE0_FUNC == AS_GPIO { BIT!(16) } else { 0 })  // PE0 function: 1=GPIO, 0=peripheral
            | (if PE1_FUNC == AS_GPIO { BIT!(17) } else { 0 })  // PE1 function: 1=GPIO, 0=peripheral
            | (if PE2_FUNC == AS_GPIO { BIT!(18) } else { 0 })  // PE2 function: 1=GPIO, 0=peripheral
            | (if PE3_FUNC == AS_GPIO { BIT!(19) } else { 0 })  // PE3 function: 1=GPIO, 0=peripheral
            | (if PE4_FUNC == AS_GPIO { BIT!(20) } else { 0 })  // PE4 function: 1=GPIO, 0=peripheral
            | (if PE5_FUNC == AS_GPIO { BIT!(21) } else { 0 })  // PE5 function: 1=GPIO, 0=peripheral
            | (if PE6_FUNC == AS_GPIO { BIT!(22) } else { 0 })  // PE6 function: 1=GPIO, 0=peripheral
            | (if PE7_FUNC == AS_GPIO { BIT!(23) } else { 0 }), // PE7 function: 1=GPIO, 0=peripheral
    );

    // --- Configure Port F (PF0-PF1) GPIO settings ---
    // Note: Port F only has 2 pins (PF0-PF1)
    write_reg_gpio_pf_setting1(
        ((PF0_INPUT_ENABLE as u32) << 8)     // PF0 input enable
            | ((PF1_INPUT_ENABLE as u32) << 9)     // PF1 input enable
            | ((if PF0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)  // PF0 output enable (inverted)
            | ((if PF1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)  // PF1 output enable (inverted)
            | ((PF0_DATA_OUT as u32) << 24)  // PF0 initial output value
            | ((PF1_DATA_OUT as u32) << 25), // PF1 initial output value
    );

    write_reg_gpio_pf_setting2(
        ((PF0_DATA_STRENGTH as u32) << 8)    // PF0 drive strength
            | ((PF1_DATA_STRENGTH as u32) << 9)    // PF1 drive strength
            | (if PF0_FUNC == AS_GPIO { BIT!(16) } else { 0 })  // PF0 function: 1=GPIO, 0=peripheral
            | (if PF1_FUNC == AS_GPIO { BIT!(17) } else { 0 }), // PF1 function: 1=GPIO, 0=peripheral
    );

    // The vendor SDK also programs the analog pin pull-up/pull-down registers
    // (`0x0a..0x14`) at the end of `gpio_init`. That tail has been unreachable
    // since an early `return` was added (af45079, 2023-01-27), so it has never
    // run on device, and re-enabling it would change pad behaviour. It is
    // therefore removed instead of compiled, which leaves the emitted firmware
    // unchanged; the original sequence is in git history at af45079^.
}

/// Sets the function type for a GPIO pin.
///
/// This function configures a pin to work either as a standard GPIO or as a peripheral function.
///
/// # Parameters
///
/// * `pin` - The GPIO pin identifier (e.g. GPIO_PA0, GPIO_PB3, etc.)
/// * `func` - The function type (AS_GPIO, AS_MSPI, AS_UART, etc.)
///
/// # Algorithm
///
/// 1. Extract the pin bit from the pin identifier
/// 2. If function is GPIO:
///    - Set the corresponding bit in the GPIO function register
/// 3. Otherwise (peripheral function):
///    - Clear the corresponding bit in the GPIO function register
///
/// # Notes
///
/// * Pin format: 0xPBB where P is port (0-5 for A-F) and BB is bit mask
/// * The implementation shifts the port ID to get the register offset
pub fn gpio_set_func(pin: u32, func: u8) {
    // Extract the bit position from the pin identifier (lower 8 bits)
    let bit: u8 = (pin & 0xff) as u8;

    // Calculate register offset based on port number
    let reg_offset = (pin >> 8) << 3;

    // Read current register value
    let current_value = read_reg_gpio_gpio_func(reg_offset);

    // Prepare the new register value based on the function type
    let new_value = if func == AS_GPIO {
        // For GPIO function, set the bit in the function register
        current_value | bit // Set bit to enable GPIO function
    } else {
        // For peripheral function, clear the bit in the function register
        current_value & !bit // Clear bit to enable peripheral function
    };

    // Write the new value to the register
    write_reg_gpio_gpio_func(new_value, reg_offset);
}

/// Enables or disables output mode for a GPIO pin.
///
/// This function configures whether a GPIO pin is set to output mode.
/// Note the inverted logic: setting a bit to 0 in the register enables output.
///
/// # Parameters
///
/// * `pin` - The GPIO pin identifier (e.g. GPIO_PA0, GPIO_PB3, etc.)
/// * `value` - The output mode (1=enable output, 0=disable output)
///
/// # Algorithm
///
/// 1. Extract the pin bit from the pin identifier
/// 2. Calculate the register offset based on the port number
/// 3. If value is 0 (disable output):
///    - Set the corresponding bit in the OEN (Output Enable Negative) register
/// 4. If value is 1 (enable output):
///    - Clear the corresponding bit in the OEN register
///
/// # Notes
///
/// * Pin format: 0xPBB where P is port (0-5 for A-F) and BB is bit mask
/// * IMPORTANT: The hardware uses inverted logic for output enable
///   - OEN=0: Output enabled
///   - OEN=1: Output disabled
pub fn gpio_set_output_en(pin: u32, value: u32) {
    // Extract the bit position from the pin identifier
    let bit = (pin & 0xff) as u8;

    // Calculate the register offset based on port number
    let reg_offset = (pin >> 8) << 3;

    // Read current register value
    let mut val = read_reg_gpio_oen(reg_offset);

    if value == 0 {
        // Disable output: Set the bit (inverted logic)
        BM_SET!(val, bit); // Set bit to disable output (inverted logic)
    } else {
        // Enable output: Clear the bit (inverted logic)
        BM_CLR!(val, bit); // Clear bit to enable output (inverted logic)
    }

    write_reg_gpio_oen(val, reg_offset);
}

/// Enables or disables input mode for a GPIO pin.
///
/// This function configures whether a GPIO pin can receive input signals.
/// Note that this reuses the OEN register, though the logic does not align with
/// output enable function (possibly an error or hardware-specific behavior).
///
/// # Parameters
///
/// * `pin` - The GPIO pin identifier (e.g. GPIO_PA0, GPIO_PB3, etc.)
/// * `value` - The input mode (1=enable input, 0=disable input)
///
/// # Algorithm
///
/// 1. Extract the pin bit from the pin identifier
/// 2. Calculate the register offset based on the port number
/// 3. If value is non-zero (enable input):
///    - Set the corresponding bit in the OEN register
/// 4. If value is 0 (disable input):
///    - Clear the corresponding bit in the OEN register
///
/// # Notes
///
/// * Pin format: 0xPBB where P is port (0-5 for A-F) and BB is bit mask
/// * This function exhibits different behavior from gpio_set_output_en
///   despite manipulating the same register, which may indicate a hardware-specific
///   design or a potential issue in the implementation
pub fn gpio_set_input_en(pin: u32, value: u32) {
    // Extract the bit position from the pin identifier
    let bit = (pin & 0xff) as u8;

    // Calculate the register offset based on port number
    let reg_offset = (pin >> 8) << 3;

    // Read current register value
    let mut val = read_reg_gpio_oen(reg_offset);

    if value != 0 {
        // Enable input
        BM_SET!(val, bit); // Set bit to enable input
    } else {
        // Disable input
        BM_CLR!(val, bit); // Clear bit to disable input
    }

    write_reg_gpio_oen(val, reg_offset);
}

/// Writes a value to a GPIO pin output.
///
/// This function sets the output state of a GPIO pin to either high (1) or low (0).
/// Note that the pin must be configured as an output for this to have an effect.
///
/// # Parameters
///
/// * `pin` - The GPIO pin identifier (e.g. GPIO_PA0, GPIO_PB3, etc.)
/// * `value` - The output value (non-zero=high/1, 0=low/0)
///
/// # Algorithm
///
/// 1. Extract the pin bit from the pin identifier
/// 2. Calculate the register offset based on the port number
/// 3. If value is non-zero:
///    - Set the corresponding bit in the OUT register to drive the pin high
/// 4. If value is 0:
///    - Clear the corresponding bit in the OUT register to drive the pin low
///
/// # Notes
///
/// * Pin format: 0xPBB where P is port (0-5 for A-F) and BB is bit mask
/// * This function only affects pins configured as outputs
/// * Writing to an input pin has no immediate effect but the value is stored
///   and will be applied if the pin is later switched to output mode
pub fn gpio_write(pin: u32, value: u32) {
    // Extract the bit position from the pin identifier
    let bit = (pin & 0xff) as u8;

    // Calculate the register offset based on port number
    let reg_offset = (pin >> 8) << 3;

    // Read current register value
    let mut val = read_reg_gpio_out(reg_offset);

    if value != 0 {
        // Set pin to high (1)
        BM_SET!(val, bit); // Set bit to drive the pin high
    } else {
        // Set pin to low (0)
        BM_CLR!(val, bit); // Clear bit to drive the pin low
    }

    write_reg_gpio_out(val, reg_offset);
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// Base addresses of the three per-port GPIO registers these tests touch.
    const GPIO_OEN: u32 = 0x582;
    const GPIO_OUT: u32 = 0x583;
    const GPIO_GPIO_FUNC: u32 = 0x586;

    /// `FakeMmio` is process-wide and the suite runs with `--test-threads=1`,
    /// so every test starts from a zeroed bank.
    fn reset_fakes() {
        FakeMmio::reset();
    }

    /// Tests setting a GPIO pin to GPIO function mode.
    ///
    /// This test verifies that the gpio_set_func function correctly:
    /// - Reads the current function register value
    /// - Sets the appropriate bit for GPIO mode
    /// - Writes the updated register value back
    ///
    /// # Algorithm
    ///
    /// 1. Seed the port A function register with 0
    /// 2. Call gpio_set_func with GPIO_PA0 and AS_GPIO
    /// 3. Verify the register was written with the bit set
    #[test]
    fn test_gpio_set_func_as_gpio() {
        reset_fakes();
        FakeMmio::set_reg8(GPIO_GPIO_FUNC, 0x00);

        // Call the function (set PA0 as GPIO)
        gpio_set_func(GPIO_PA0 as u32, AS_GPIO);

        // Verify function register was written with bit 0 set (PA0's bit mask is 0x01)
        assert_eq!(FakeMmio::reg8(GPIO_GPIO_FUNC), 0x01);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: GPIO_GPIO_FUNC | REG_BASE_ADDR,
                width: 1,
                value: 0x01,
            }]
        );
    }

    /// Tests setting a GPIO pin to peripheral function mode.
    ///
    /// This test verifies that the gpio_set_func function correctly:
    /// - Reads the current function register value
    /// - Clears the appropriate bit for peripheral mode
    /// - Writes the updated register value back
    ///
    /// # Algorithm
    ///
    /// 1. Seed the function register with PA0's bit already set
    /// 2. Call gpio_set_func with GPIO_PA0 and a peripheral function (AS_UART)
    /// 3. Verify the register was written with the bit cleared
    #[test]
    fn test_gpio_set_func_as_peripheral() {
        reset_fakes();
        // PA0's bit set (bit mask 0x01)
        FakeMmio::set_reg8(GPIO_GPIO_FUNC, 0x01);
        FakeMmio::clear_writes();

        // Call the function (set PA0 as UART)
        gpio_set_func(GPIO_PA0 as u32, AS_UART);

        // Verify function register was written with PA0's bit cleared
        assert_eq!(FakeMmio::reg8(GPIO_GPIO_FUNC), 0x00);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: GPIO_GPIO_FUNC | REG_BASE_ADDR,
                width: 1,
                value: 0x00,
            }]
        );
    }

    /// Tests setting a GPIO pin to peripheral function mode for a different port.
    ///
    /// This test verifies that port selection logic works correctly in gpio_set_func:
    /// - Uses correct register offset for Port B
    /// - Clears the appropriate bit for the selected pin
    ///
    /// # Algorithm
    ///
    /// 1. Seed the port B function register with PB2's bit set
    /// 2. Call gpio_set_func with GPIO_PB2 and AS_MSPI
    /// 3. Verify the register was written with the bit cleared at the port B offset
    #[test]
    fn test_gpio_set_func_port_b() {
        reset_fakes();
        // Port B offset = 8 (calculated from (0x100 >> 8) << 3 = 0x08)
        // PB2's bit mask is 0x04
        const PORT_B_OFFSET: u32 = 8;
        FakeMmio::set_reg8(GPIO_GPIO_FUNC + PORT_B_OFFSET, 0x04);
        FakeMmio::clear_writes();

        // Call the function (set PB2 as MSPI)
        gpio_set_func(GPIO_PB2 as u32, AS_MSPI);

        // Verify function register was written with bit 2 cleared at port B offset
        assert_eq!(FakeMmio::reg8(GPIO_GPIO_FUNC + PORT_B_OFFSET), 0x00);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: (GPIO_GPIO_FUNC + PORT_B_OFFSET) | REG_BASE_ADDR,
                width: 1,
                value: 0x00,
            }]
        );
    }

    /// Tests enabling output for a GPIO pin.
    ///
    /// This test verifies that the gpio_set_output_en function correctly:
    /// - Reads the current output enable register value
    /// - Clears the appropriate bit (inverted logic: 0=enabled)
    /// - Writes the updated register value back
    ///
    /// # Algorithm
    ///
    /// 1. Seed the OEN register with all outputs disabled (all 1s)
    /// 2. Call gpio_set_output_en with GPIO_PA3 and value=1 (enable)
    /// 3. Verify the register was written with the correct bit cleared
    #[test]
    fn test_gpio_set_output_en_enable() {
        reset_fakes();
        // PA3 is port A (offset 0); all outputs disabled (all 1s)
        FakeMmio::set_reg8(GPIO_OEN, 0xFF);
        FakeMmio::clear_writes();

        // Call the function (enable output for PA3)
        gpio_set_output_en(GPIO_PA3 as u32, 1);

        // PA3's bit mask is 0x08: 0xFF with bit 3 cleared = 0xF7
        assert_eq!(FakeMmio::reg8(GPIO_OEN), 0xF7);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: GPIO_OEN | REG_BASE_ADDR,
                width: 1,
                value: 0xF7,
            }]
        );
    }

    /// Tests disabling output for a GPIO pin.
    ///
    /// This test verifies that the gpio_set_output_en function correctly:
    /// - Reads the current output enable register value
    /// - Sets the appropriate bit (inverted logic: 1=disabled)
    /// - Writes the updated register value back
    ///
    /// # Algorithm
    ///
    /// 1. Seed the port B OEN register with PB5 already enabled
    /// 2. Call gpio_set_output_en with GPIO_PB5 and value=0 (disable)
    /// 3. Verify the register was written with the correct bit set
    #[test]
    fn test_gpio_set_output_en_disable() {
        reset_fakes();
        // Port B offset = 8, bit 5 already cleared (enabled)
        const PORT_B_OFFSET: u32 = 8;
        FakeMmio::set_reg8(GPIO_OEN + PORT_B_OFFSET, 0xDF);
        FakeMmio::clear_writes();

        // Call the function (disable output for PB5)
        gpio_set_output_en(GPIO_PB5 as u32, 0);

        // 0xDF with bit 5 set = 0xFF
        assert_eq!(FakeMmio::reg8(GPIO_OEN + PORT_B_OFFSET), 0xFF);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: (GPIO_OEN + PORT_B_OFFSET) | REG_BASE_ADDR,
                width: 1,
                value: 0xFF,
            }]
        );
    }

    /// Tests enabling input for a GPIO pin.
    ///
    /// This test verifies that the gpio_set_input_en function correctly:
    /// - Reads the current register value
    /// - Sets the appropriate bit for input enable
    /// - Writes the updated register value back
    ///
    /// # Algorithm
    ///
    /// 1. Seed the port C register with bit 6 initially cleared
    /// 2. Call gpio_set_input_en with GPIO_PC6 and value=1 (enable)
    /// 3. Verify the register was written with the bit set
    #[test]
    fn test_gpio_set_input_en_enable() {
        reset_fakes();
        // Port C offset = 16, bit 6 initially cleared
        const PORT_C_OFFSET: u32 = 16;
        FakeMmio::set_reg8(GPIO_OEN + PORT_C_OFFSET, 0xBF);
        FakeMmio::clear_writes();

        // Call the function (enable input for PC6)
        gpio_set_input_en(GPIO_PC6 as u32, 1);

        // 0xBF with bit 6 set = 0xFF
        assert_eq!(FakeMmio::reg8(GPIO_OEN + PORT_C_OFFSET), 0xFF);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: (GPIO_OEN + PORT_C_OFFSET) | REG_BASE_ADDR,
                width: 1,
                value: 0xFF,
            }]
        );
    }

    /// Tests disabling input for a GPIO pin.
    ///
    /// This test verifies that the gpio_set_input_en function correctly:
    /// - Reads the current register value
    /// - Clears the appropriate bit for input disable
    /// - Writes the updated register value back
    ///
    /// # Algorithm
    ///
    /// 1. Seed the port D register with bit 2 initially set
    /// 2. Call gpio_set_input_en with GPIO_PD2 and value=0 (disable)
    /// 3. Verify the register was written with the bit cleared
    #[test]
    fn test_gpio_set_input_en_disable() {
        reset_fakes();
        // Port D offset = 24, bit 2 initially set
        const PORT_D_OFFSET: u32 = 24;
        FakeMmio::set_reg8(GPIO_OEN + PORT_D_OFFSET, 0x04);
        FakeMmio::clear_writes();

        // Call the function (disable input for PD2)
        gpio_set_input_en(GPIO_PD2 as u32, 0);

        // 0x04 with bit 2 cleared = 0x00
        assert_eq!(FakeMmio::reg8(GPIO_OEN + PORT_D_OFFSET), 0x00);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: (GPIO_OEN + PORT_D_OFFSET) | REG_BASE_ADDR,
                width: 1,
                value: 0x00,
            }]
        );
    }

    /// Tests writing a 1 (high) to a GPIO pin.
    ///
    /// This test verifies that the gpio_write function correctly:
    /// - Reads the current output register value
    /// - Sets the appropriate bit for high output
    /// - Writes the updated register value back
    ///
    /// # Algorithm
    ///
    /// 1. Seed the OUT register with bit 5 cleared
    /// 2. Call gpio_write with GPIO_PA5 and value=1 (high)
    /// 3. Verify the register was written with the bit set
    #[test]
    fn test_gpio_write_high() {
        reset_fakes();
        // PA5's bit initially cleared
        FakeMmio::set_reg8(GPIO_OUT, 0xDF);
        FakeMmio::clear_writes();

        // Call the function (write high to PA5)
        gpio_write(GPIO_PA5 as u32, 1);

        // 0xDF with bit 5 set = 0xFF
        assert_eq!(FakeMmio::reg8(GPIO_OUT), 0xFF);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: GPIO_OUT | REG_BASE_ADDR,
                width: 1,
                value: 0xFF,
            }]
        );
    }

    /// Tests writing a 0 (low) to a GPIO pin.
    ///
    /// This test verifies that the gpio_write function correctly:
    /// - Reads the current output register value
    /// - Clears the appropriate bit for low output
    /// - Writes the updated register value back
    ///
    /// # Algorithm
    ///
    /// 1. Seed the port B OUT register with bit 1 set
    /// 2. Call gpio_write with GPIO_PB1 and value=0 (low)
    /// 3. Verify the register was written with the bit cleared
    #[test]
    fn test_gpio_write_low() {
        reset_fakes();
        // Port B offset = 8, bit 1 initially set
        const PORT_B_OFFSET: u32 = 8;
        FakeMmio::set_reg8(GPIO_OUT + PORT_B_OFFSET, 0x02);
        FakeMmio::clear_writes();

        // Call the function (write low to PB1)
        gpio_write(GPIO_PB1 as u32, 0);

        // 0x02 with bit 1 cleared = 0x00
        assert_eq!(FakeMmio::reg8(GPIO_OUT + PORT_B_OFFSET), 0x00);
        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: (GPIO_OUT + PORT_B_OFFSET) | REG_BASE_ADDR,
                width: 1,
                value: 0x00,
            }]
        );
    }
}
