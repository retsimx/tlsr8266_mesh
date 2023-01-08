use ::{BIT, regrw_idx};
use sdk::mcu::analog::{analog_read__attribute_ram_code, analog_write__attribute_ram_code};
use sdk::mcu::gpio::GPIO_PIN_TYPE::*;
use sdk::mcu::register::{write_reg_gpio_gpio_func, read_reg_gpio_gpio_func, write_reg_gpio_pa_setting1, write_reg_gpio_pa_setting2, write_reg_gpio_pb_setting1, write_reg_gpio_pb_setting2, write_reg_gpio_pc_setting1, write_reg_gpio_pc_setting2, write_reg_gpio_pd_setting1, write_reg_gpio_pd_setting2, write_reg_gpio_pe_setting1, write_reg_gpio_pe_setting2, write_reg_gpio_pf_setting1, write_reg_gpio_pf_setting2};

pub static RF_FAST_MODE_1M: u8 = 1;
pub static PM_PIN_PULL_DEFAULT: u8 = 1;

pub static GPIO_PULL_UP_0: u8 = 0;
pub static GPIO_PULL_UP_1M: u8 = 1;
pub static GPIO_PULL_UP_10K: u8 = 2;
pub static GPIO_PULL_DN_100K: u8 = 3;

pub enum GPIO_DIR {
    IN = 0,
    OUT = 1,
}

pub static AS_GPIO: u8 = 0;
pub static AS_MSPI: u8 = 1;
pub static AS_SWIRE: u8 = 2;
pub static AS_UART: u8 = 3;
pub static AS_PWM: u8 = 4;
pub static AS_I2C: u8 = 5;
pub static AS_SPI: u8 = 6;
pub static AS_ETH_MAC: u8 = 7;
pub static AS_I2S: u8 = 8;
pub static AS_SDM: u8 = 9;
pub static AS_DMIC: u8 = 10;
pub static AS_USB: u8 = 11;
pub static AS_SWS: u8 = 12;
pub static AS_SWM: u8 = 13;
pub static AS_TEST: u8 = 14;
pub static AS_ADC: u8 = 15;

pub static PA0_INPUT_ENABLE: u8 = 1;
pub static PA1_INPUT_ENABLE: u8 = 1;
pub static PA2_INPUT_ENABLE: u8 = 1;
pub static PA3_INPUT_ENABLE: u8 = 1;
pub static PA4_INPUT_ENABLE: u8 = 1;
pub static PA5_INPUT_ENABLE: u8 = 1;
pub static PA6_INPUT_ENABLE: u8 = 1;
pub static PA7_INPUT_ENABLE: u8 = 1;
pub static PA0_OUTPUT_ENABLE: u8 = 0;
pub static PA1_OUTPUT_ENABLE: u8 = 0;
pub static PA2_OUTPUT_ENABLE: u8 = 0;
pub static PA3_OUTPUT_ENABLE: u8 = 0;
pub static PA4_OUTPUT_ENABLE: u8 = 0;
pub static PA5_OUTPUT_ENABLE: u8 = 0;
pub static PA6_OUTPUT_ENABLE: u8 = 0;
pub static PA7_OUTPUT_ENABLE: u8 = 0;
pub static PA0_DATA_STRENGTH: u8 = 1;
pub static PA1_DATA_STRENGTH: u8 = 1;
pub static PA2_DATA_STRENGTH: u8 = 1;
pub static PA3_DATA_STRENGTH: u8 = 1;
pub static PA4_DATA_STRENGTH: u8 = 1;
pub static PA5_DATA_STRENGTH: u8 = 1;
pub static PA6_DATA_STRENGTH: u8 = 1;
pub static PA7_DATA_STRENGTH: u8 = 1;
pub static PA0_DATA_OUT: u8 = 1;
//open SWS output to avoid MCU err
pub static PA1_DATA_OUT: u8 = 0;
pub static PA2_DATA_OUT: u8 = 0;
pub static PA3_DATA_OUT: u8 = 0;
pub static PA4_DATA_OUT: u8 = 0;
pub static PA5_DATA_OUT: u8 = 0;
pub static PA6_DATA_OUT: u8 = 0;
pub static PA7_DATA_OUT: u8 = 0;
pub static PA0_FUNC: u8 = AS_SWIRE;
pub static PA1_FUNC: u8 = AS_GPIO;
pub static PA2_FUNC: u8 = AS_MSPI;
pub static PA3_FUNC: u8 = AS_MSPI;
pub static PA4_FUNC: u8 = AS_GPIO;
pub static PA5_FUNC: u8 = AS_GPIO;
pub static PA6_FUNC: u8 = AS_GPIO;
pub static PA7_FUNC: u8 = AS_SWIRE;
pub static PULL_WAKEUP_SRC_PA0: u8 = PM_PIN_PULL_DEFAULT;
// SWS
pub static PULL_WAKEUP_SRC_PA1: u8 = 0;
// LIGHT_PWM must 0(float) or 3(pull_down_100K )
pub static PULL_WAKEUP_SRC_PA2: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PA3: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PA4: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PA5: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PA6: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PA7: u8 = PM_PIN_PULL_DEFAULT;    // SWM

//////////////////////////////////////////////////
pub static PB0_INPUT_ENABLE: u8 = 1;
pub static PB1_INPUT_ENABLE: u8 = 1;
pub static PB2_INPUT_ENABLE: u8 = 1;
pub static PB3_INPUT_ENABLE: u8 = 1;
pub static PB4_INPUT_ENABLE: u8 = 1;
pub static PB5_INPUT_ENABLE: u8 = 1;
pub static PB6_INPUT_ENABLE: u8 = 1;
pub static PB7_INPUT_ENABLE: u8 = 1;
pub static PB0_OUTPUT_ENABLE: u8 = 0;
pub static PB1_OUTPUT_ENABLE: u8 = 0;
pub static PB2_OUTPUT_ENABLE: u8 = 0;
pub static PB3_OUTPUT_ENABLE: u8 = 0;
pub static PB4_OUTPUT_ENABLE: u8 = 0;
pub static PB5_OUTPUT_ENABLE: u8 = 0;
pub static PB6_OUTPUT_ENABLE: u8 = 0;
pub static PB7_OUTPUT_ENABLE: u8 = 0;
pub static PB0_DATA_STRENGTH: u8 = 1;
pub static PB1_DATA_STRENGTH: u8 = 1;
pub static PB2_DATA_STRENGTH: u8 = 1;
pub static PB3_DATA_STRENGTH: u8 = 1;
pub static PB4_DATA_STRENGTH: u8 = 1;
pub static PB5_DATA_STRENGTH: u8 = 1;
pub static PB6_DATA_STRENGTH: u8 = 1;
pub static PB7_DATA_STRENGTH: u8 = 1;
pub static PB0_DATA_OUT: u8 = 0;
pub static PB1_DATA_OUT: u8 = 0;
pub static PB2_DATA_OUT: u8 = 0;
pub static PB3_DATA_OUT: u8 = 0;
pub static PB4_DATA_OUT: u8 = 0;
pub static PB5_DATA_OUT: u8 = 0;
pub static PB6_DATA_OUT: u8 = 0;
pub static PB7_DATA_OUT: u8 = 0;
pub static PB0_FUNC: u8 = AS_GPIO;
pub static PB1_FUNC: u8 = AS_GPIO;
pub static PB2_FUNC: u8 = AS_MSPI;
pub static PB3_FUNC: u8 = AS_MSPI;
pub static PB4_FUNC: u8 = AS_GPIO;
pub static PB5_FUNC: u8 = AS_USB;
pub static PB6_FUNC: u8 = AS_USB;
pub static PB7_FUNC: u8 = AS_GPIO;
pub static PULL_WAKEUP_SRC_PB0: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PB1: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PB2: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PB3: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PB4: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PB5: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PB6: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PB7: u8 = PM_PIN_PULL_DEFAULT;

//////////////////////////////////////////////////
pub static PC0_INPUT_ENABLE: u8 = 1;
pub static PC1_INPUT_ENABLE: u8 = 1;
pub static PC2_INPUT_ENABLE: u8 = 1;
pub static PC3_INPUT_ENABLE: u8 = 1;
pub static PC4_INPUT_ENABLE: u8 = 1;
pub static PC5_INPUT_ENABLE: u8 = 1;
pub static PC6_INPUT_ENABLE: u8 = 1;
pub static PC7_INPUT_ENABLE: u8 = 1;
pub static PC0_OUTPUT_ENABLE: u8 = 0;
pub static PC1_OUTPUT_ENABLE: u8 = 0;
pub static PC2_OUTPUT_ENABLE: u8 = 0;
pub static PC3_OUTPUT_ENABLE: u8 = 0;
pub static PC4_OUTPUT_ENABLE: u8 = 0;
pub static PC5_OUTPUT_ENABLE: u8 = 0;
pub static PC6_OUTPUT_ENABLE: u8 = 0;
pub static PC7_OUTPUT_ENABLE: u8 = 0;
pub static PC0_DATA_STRENGTH: u8 = 1;
pub static PC1_DATA_STRENGTH: u8 = 1;
pub static PC2_DATA_STRENGTH: u8 = 1;
pub static PC3_DATA_STRENGTH: u8 = 1;
pub static PC4_DATA_STRENGTH: u8 = 1;
pub static PC5_DATA_STRENGTH: u8 = 1;
pub static PC6_DATA_STRENGTH: u8 = 1;
pub static PC7_DATA_STRENGTH: u8 = 1;
pub static PC0_DATA_OUT: u8 = 0;
pub static PC1_DATA_OUT: u8 = 0;
pub static PC2_DATA_OUT: u8 = 0;
pub static PC3_DATA_OUT: u8 = 0;
pub static PC4_DATA_OUT: u8 = 0;
pub static PC5_DATA_OUT: u8 = 0;
pub static PC6_DATA_OUT: u8 = 0;
pub static PC7_DATA_OUT: u8 = 0;
pub static PC0_FUNC: u8 = AS_GPIO;
pub static PC1_FUNC: u8 = AS_GPIO;
pub static PC2_FUNC: u8 = AS_GPIO;
pub static PC3_FUNC: u8 = AS_GPIO;
pub static PC4_FUNC: u8 = AS_GPIO;
pub static PC5_FUNC: u8 = AS_GPIO;
pub static PC6_FUNC: u8 = AS_GPIO;
pub static PC7_FUNC: u8 = AS_GPIO;
pub static PULL_WAKEUP_SRC_PC0: u8 = 0;
// LIGHT_PWM must 0(float) or 3(pull_down_100K )
pub static PULL_WAKEUP_SRC_PC1: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PC2: u8 = 0;
// LIGHT_PWM must 0(float) or 3(pull_down_100K )
pub static PULL_WAKEUP_SRC_PC3: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PC4: u8 = 0;
// LIGHT_PWM must 0(float) or 3(pull_down_100K )
pub static PULL_WAKEUP_SRC_PC5: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PC6: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PC7: u8 = PM_PIN_PULL_DEFAULT;

//////////////////////////////////////////////////
pub static PD0_INPUT_ENABLE: u8 = 1;
pub static PD1_INPUT_ENABLE: u8 = 1;
pub static PD2_INPUT_ENABLE: u8 = 1;
pub static PD3_INPUT_ENABLE: u8 = 1;
pub static PD4_INPUT_ENABLE: u8 = 1;
pub static PD5_INPUT_ENABLE: u8 = 1;
pub static PD6_INPUT_ENABLE: u8 = 1;
pub static PD7_INPUT_ENABLE: u8 = 1;
pub static PD0_OUTPUT_ENABLE: u8 = 0;
pub static PD1_OUTPUT_ENABLE: u8 = 0;
pub static PD2_OUTPUT_ENABLE: u8 = 0;
pub static PD3_OUTPUT_ENABLE: u8 = 0;
pub static PD4_OUTPUT_ENABLE: u8 = 0;
pub static PD5_OUTPUT_ENABLE: u8 = 0;
pub static PD6_OUTPUT_ENABLE: u8 = 0;
pub static PD7_OUTPUT_ENABLE: u8 = 0;
pub static PD0_DATA_STRENGTH: u8 = 1;
pub static PD1_DATA_STRENGTH: u8 = 1;
pub static PD2_DATA_STRENGTH: u8 = 1;
pub static PD3_DATA_STRENGTH: u8 = 1;
pub static PD4_DATA_STRENGTH: u8 = 1;
pub static PD5_DATA_STRENGTH: u8 = 1;
pub static PD6_DATA_STRENGTH: u8 = 1;
pub static PD7_DATA_STRENGTH: u8 = 1;
pub static PD0_DATA_OUT: u8 = 0;
pub static PD1_DATA_OUT: u8 = 0;
pub static PD2_DATA_OUT: u8 = 0;
pub static PD3_DATA_OUT: u8 = 0;
pub static PD4_DATA_OUT: u8 = 0;
pub static PD5_DATA_OUT: u8 = 0;
pub static PD6_DATA_OUT: u8 = 0;
pub static PD7_DATA_OUT: u8 = 0;
pub static PD0_FUNC: u8 = AS_GPIO;
pub static PD1_FUNC: u8 = AS_GPIO;
pub static PD2_FUNC: u8 = AS_GPIO;
pub static PD3_FUNC: u8 = AS_GPIO;
pub static PD4_FUNC: u8 = AS_GPIO;
pub static PD5_FUNC: u8 = AS_GPIO;
pub static PD6_FUNC: u8 = AS_GPIO;
pub static PD7_FUNC: u8 = AS_GPIO;
pub static PULL_WAKEUP_SRC_PD0: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PD1: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PD2: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PD3: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PD4: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PD5: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PD6: u8 = PM_PIN_PULL_DEFAULT;
pub static PULL_WAKEUP_SRC_PD7: u8 = PM_PIN_PULL_DEFAULT;

//////////////////////////////////////////////////
pub static PE0_INPUT_ENABLE: u8 = 1;
pub static PE1_INPUT_ENABLE: u8 = 1;
pub static PE2_INPUT_ENABLE: u8 = 1;
pub static PE3_INPUT_ENABLE: u8 = 1;
pub static PE4_INPUT_ENABLE: u8 = 1;
pub static PE5_INPUT_ENABLE: u8 = 1;
pub static PE6_INPUT_ENABLE: u8 = 1;
pub static PE7_INPUT_ENABLE: u8 = 1;
pub static PE0_OUTPUT_ENABLE: u8 = 0;
pub static PE1_OUTPUT_ENABLE: u8 = 0;
pub static PE2_OUTPUT_ENABLE: u8 = 0;
pub static PE3_OUTPUT_ENABLE: u8 = 0;
pub static PE4_OUTPUT_ENABLE: u8 = 0;
pub static PE5_OUTPUT_ENABLE: u8 = 0;
pub static PE6_OUTPUT_ENABLE: u8 = 0;
pub static PE7_OUTPUT_ENABLE: u8 = 0;
pub static PE0_DATA_STRENGTH: u8 = 1;
pub static PE1_DATA_STRENGTH: u8 = 1;
pub static PE2_DATA_STRENGTH: u8 = 1;
pub static PE3_DATA_STRENGTH: u8 = 1;
pub static PE4_DATA_STRENGTH: u8 = 1;
pub static PE5_DATA_STRENGTH: u8 = 1;
pub static PE6_DATA_STRENGTH: u8 = 1;
pub static PE7_DATA_STRENGTH: u8 = 1;
pub static PE0_DATA_OUT: u8 = 0;
pub static PE1_DATA_OUT: u8 = 0;
pub static PE2_DATA_OUT: u8 = 0;
pub static PE3_DATA_OUT: u8 = 0;
pub static PE4_DATA_OUT: u8 = 0;
pub static PE5_DATA_OUT: u8 = 0;
pub static PE6_DATA_OUT: u8 = 0;
pub static PE7_DATA_OUT: u8 = 0;
pub static PE0_FUNC: u8 = AS_GPIO;
pub static PE1_FUNC: u8 = AS_GPIO;
pub static PE2_FUNC: u8 = AS_GPIO;
pub static PE3_FUNC: u8 = AS_GPIO;
pub static PE4_FUNC: u8 = AS_GPIO;
pub static PE5_FUNC: u8 = AS_GPIO;
pub static PE6_FUNC: u8 = AS_GPIO;
pub static PE7_FUNC: u8 = AS_GPIO;
pub static PULL_WAKEUP_SRC_PE0: u8 = 0;
pub static PULL_WAKEUP_SRC_PE1: u8 = 0;
pub static PULL_WAKEUP_SRC_PE2: u8 = 0;
pub static PULL_WAKEUP_SRC_PE3: u8 = 0;
pub static PULL_WAKEUP_SRC_PE4: u8 = 0;
pub static PULL_WAKEUP_SRC_PE5: u8 = 0;
pub static PULL_WAKEUP_SRC_PE6: u8 = 0;
pub static PULL_WAKEUP_SRC_PE7: u8 = 0;


//////////////////////////////////////////////////
pub static PF0_INPUT_ENABLE: u8 = 1;
pub static PF1_INPUT_ENABLE: u8 = 1;
pub static PF0_OUTPUT_ENABLE: u8 = 0;
pub static PF1_OUTPUT_ENABLE: u8 = 0;
pub static PF0_DATA_STRENGTH: u8 = 1;
pub static PF1_DATA_STRENGTH: u8 = 1;
pub static PF0_DATA_OUT: u8 = 0;
pub static PF1_DATA_OUT: u8 = 0;
pub static PF0_FUNC: u8 = AS_GPIO;
pub static PF1_FUNC: u8 = AS_GPIO;
pub static PULL_WAKEUP_SRC_PF0: u8 = 0;
pub static PULL_WAKEUP_SRC_PF1: u8 = 0;


pub static PM_PIN_PULLUP_1M: u8 = 1;
pub static PM_PIN_PULLUP_10K: u8 = 2;
pub static PM_PIN_PULLDOWN_100K: u8 = 3;
pub static PM_PIN_UP_DOWN_FLOAT: u8 = 0xff;

pub enum GPIO_PIN_TYPE {
    GPIO_PA0 = 0x000 | BIT!(0),
    GPIO_PA1 = 0x000 | BIT!(1),
    GPIO_PA2 = 0x000 | BIT!(2),
    GPIO_PA3 = 0x000 | BIT!(3),
    GPIO_PA4 = 0x000 | BIT!(4),
    GPIO_PA5 = 0x000 | BIT!(5),
    GPIO_PA6 = 0x000 | BIT!(6),
    GPIO_PA7 = 0x000 | BIT!(7),
    GPIO_PB0 = 0x100 | BIT!(0),
    GPIO_PB1 = 0x100 | BIT!(1),
    GPIO_PB2 = 0x100 | BIT!(2),
    GPIO_PB3 = 0x100 | BIT!(3),
    GPIO_PB4 = 0x100 | BIT!(4),
    GPIO_PB5 = 0x100 | BIT!(5),
    GPIO_PB6 = 0x100 | BIT!(6),
    GPIO_PB7 = 0x100 | BIT!(7),
    GPIO_PC0 = 0x200 | BIT!(0),
    GPIO_PC1 = 0x200 | BIT!(1),
    GPIO_PC2 = 0x200 | BIT!(2),
    GPIO_PC3 = 0x200 | BIT!(3),
    GPIO_PC4 = 0x200 | BIT!(4),
    GPIO_PC5 = 0x200 | BIT!(5),
    GPIO_PC6 = 0x200 | BIT!(6),
    GPIO_PC7 = 0x200 | BIT!(7),
    GPIO_PD0 = 0x300 | BIT!(0),
    GPIO_PD1 = 0x300 | BIT!(1),
    GPIO_PD2 = 0x300 | BIT!(2),
    GPIO_PD3 = 0x300 | BIT!(3),
    GPIO_PD4 = 0x300 | BIT!(4),
    GPIO_PD5 = 0x300 | BIT!(5),
    GPIO_PD6 = 0x300 | BIT!(6),
    GPIO_PD7 = 0x300 | BIT!(7),
    GPIO_PE0 = 0x400 | BIT!(0),
    GPIO_PE1 = 0x400 | BIT!(1),
    GPIO_PE2 = 0x400 | BIT!(2),
    GPIO_PE3 = 0x400 | BIT!(3),
    GPIO_PE4 = 0x400 | BIT!(4),
    GPIO_PE5 = 0x400 | BIT!(5),
    GPIO_PE6 = 0x400 | BIT!(6),
    GPIO_PE7 = 0x400 | BIT!(7),
    GPIO_PF0 = 0x500 | BIT!(0),
    GPIO_PF1 = 0x500 | BIT!(1),

    GPIO_MAX_COUNT = 56,
}

impl GPIO_PIN_TYPE {
    pub const GPIO_SWS: GPIO_PIN_TYPE = GPIO_PA0;
    pub const GPIO_PWM3A1: GPIO_PIN_TYPE = GPIO_PA1;
    pub const GPIO_MSDI: GPIO_PIN_TYPE = GPIO_PA2;
    pub const GPIO_MCLK: GPIO_PIN_TYPE = GPIO_PA3;
    pub const GPIO_PWM3NA4: GPIO_PIN_TYPE = GPIO_PA4;
    pub const GPIO_GP18: GPIO_PIN_TYPE = GPIO_PA4;
    pub const GPIO_PWM4A5: GPIO_PIN_TYPE = GPIO_PA5;
    pub const GPIO_PWM4NA6: GPIO_PIN_TYPE = GPIO_PA6;
    pub const GPIO_GP19: GPIO_PIN_TYPE = GPIO_PA6;
    pub const GPIO_SWM: GPIO_PIN_TYPE = GPIO_PA7;
    pub const GPIO_PWM5B0: GPIO_PIN_TYPE = GPIO_PB0;
    pub const GPIO_PWM5NB1: GPIO_PIN_TYPE = GPIO_PB1;
    pub const GPIO_GP20: GPIO_PIN_TYPE = GPIO_PB1;
    pub const GPIO_MSDO: GPIO_PIN_TYPE = GPIO_PB2;
    pub const GPIO_MSCN: GPIO_PIN_TYPE = GPIO_PB3;
    pub const GPIO_GP21: GPIO_PIN_TYPE = GPIO_PB4;
    pub const GPIO_DM: GPIO_PIN_TYPE = GPIO_PB5;
    pub const GPIO_DP: GPIO_PIN_TYPE = GPIO_PB6;
    pub const GPIO_PWM0NB7: GPIO_PIN_TYPE = GPIO_PB7;
    pub const GPIO_GP0: GPIO_PIN_TYPE = GPIO_PB7;
    pub const GPIO_PWM0C0: GPIO_PIN_TYPE = GPIO_PC0;
    pub const GPIO_PWM1NC1: GPIO_PIN_TYPE = GPIO_PC1;
    pub const GPIO_GP1: GPIO_PIN_TYPE = GPIO_PC1;
    pub const GPIO_PWM1NC2: GPIO_PIN_TYPE = GPIO_PC2;
    pub const GPIO_PWM1C3: GPIO_PIN_TYPE = GPIO_PC3;
    pub const GPIO_GP2: GPIO_PIN_TYPE = GPIO_PC3;
    pub const GPIO_PWM2C4: GPIO_PIN_TYPE = GPIO_PC4;
    pub const GPIO_PWM2NC5: GPIO_PIN_TYPE = GPIO_PC5;
    pub const GPIO_GP3: GPIO_PIN_TYPE = GPIO_PC5;
    pub const GPIO_GP4: GPIO_PIN_TYPE = GPIO_PC6;
    pub const GPIO_UTX: GPIO_PIN_TYPE = GPIO_PC6;
    pub const GPIO_GP5: GPIO_PIN_TYPE = GPIO_PC7;
    pub const GPIO_URX: GPIO_PIN_TYPE = GPIO_PC7;
    pub const GPIO_GP6: GPIO_PIN_TYPE = GPIO_PD0;
    pub const GPIO_GP7: GPIO_PIN_TYPE = GPIO_PD1;
    pub const GPIO_PWM3D2: GPIO_PIN_TYPE = GPIO_PD2;
    pub const GPIO_GP8: GPIO_PIN_TYPE = GPIO_PD2;
    pub const GPIO_PWM4D3: GPIO_PIN_TYPE = GPIO_PD3;
    pub const GPIO_GP9: GPIO_PIN_TYPE = GPIO_PD3;
    pub const GPIO_GP10: GPIO_PIN_TYPE = GPIO_PD4;
    pub const GPIO_GP11: GPIO_PIN_TYPE = GPIO_PD5;
    pub const GPIO_GP12: GPIO_PIN_TYPE = GPIO_PD6;
    pub const GPIO_GP13: GPIO_PIN_TYPE = GPIO_PD7;
    pub const GPIO_GP14: GPIO_PIN_TYPE = GPIO_PE0;
    pub const GPIO_DMIC_CK: GPIO_PIN_TYPE = GPIO_PE1;
    pub const GPIO_DMIC_DI: GPIO_PIN_TYPE = GPIO_PE2;
    pub const GPIO_GP15: GPIO_PIN_TYPE = GPIO_PE3;
    pub const GPIO_GP16: GPIO_PIN_TYPE = GPIO_PE4;
    pub const GPIO_GP17: GPIO_PIN_TYPE = GPIO_PE5;
    pub const GPIO_CN: GPIO_PIN_TYPE = GPIO_PE6;
    pub const GPIO_DI: GPIO_PIN_TYPE = GPIO_PE7;
    pub const GPIO_DO: GPIO_PIN_TYPE = GPIO_PF0;
    pub const GPIO_CK: GPIO_PIN_TYPE = GPIO_PF1;
}

pub fn gpio_init() {
    write_reg_gpio_pa_setting1(
        ((PA0_INPUT_ENABLE as u32) << 8) | ((PA1_INPUT_ENABLE as u32) << 9) | ((PA2_INPUT_ENABLE as u32) << 10) | ((PA3_INPUT_ENABLE as u32) << 11) |
            ((PA4_INPUT_ENABLE as u32) << 12) | ((PA5_INPUT_ENABLE as u32) << 13) | ((PA6_INPUT_ENABLE as u32) << 14) | ((PA7_INPUT_ENABLE as u32) << 15) |
            ((if PA0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16) | ((if PA1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17) | ((if PA2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18) | ((if PA3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19) |
            ((if PA4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20) | ((if PA5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21) | ((if PA6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22) | ((if PA7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23) |
            ((PA0_DATA_OUT as u32) << 24) | ((PA1_DATA_OUT as u32) << 25) | ((PA2_DATA_OUT as u32) << 26) | ((PA3_DATA_OUT as u32) << 27) |
            ((PA4_DATA_OUT as u32) << 28) | ((PA5_DATA_OUT as u32) << 29) | ((PA6_DATA_OUT as u32) << 30) | ((PA7_DATA_OUT as u32) << 31)
    );

    write_reg_gpio_pa_setting2(
        ((PA0_DATA_STRENGTH as u32) << 8) | ((PA1_DATA_STRENGTH as u32) << 9) | ((PA2_DATA_STRENGTH as u32) << 10) | ((PA3_DATA_STRENGTH as u32) << 11) |
            ((PA4_DATA_STRENGTH as u32) << 12) | ((PA5_DATA_STRENGTH as u32) << 13) | ((PA6_DATA_STRENGTH as u32) << 14) | ((PA7_DATA_STRENGTH as u32) << 15) |
            (if PA0_FUNC == AS_GPIO { BIT!(16) } else { 0 }) | (if PA1_FUNC == AS_GPIO { BIT!(17) } else { 0 }) | (if PA2_FUNC == AS_GPIO { BIT!(18) } else { 0 }) | (if PA3_FUNC == AS_GPIO { BIT!(19) } else { 0 }) |
            (if PA4_FUNC == AS_GPIO { BIT!(20) } else { 0 }) | (if PA5_FUNC == AS_GPIO { BIT!(21) } else { 0 }) | (if PA6_FUNC == AS_GPIO { BIT!(22) } else { 0 }) | (if PA7_FUNC == AS_GPIO { BIT!(23) } else { 0 })
    );

    write_reg_gpio_pb_setting1(
        ((PB0_INPUT_ENABLE as u32) << 8) | ((PB1_INPUT_ENABLE as u32) << 9) | ((PB2_INPUT_ENABLE as u32) << 10) | ((PB3_INPUT_ENABLE as u32) << 11) |
            ((PB4_INPUT_ENABLE as u32) << 12) | ((PB5_INPUT_ENABLE as u32) << 13) | ((PB6_INPUT_ENABLE as u32) << 14) | ((PB7_INPUT_ENABLE as u32) << 15) |
            ((if PB0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16) | ((if PB1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17) | ((if PB2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18) | ((if PB3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19) |
            ((if PB4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20) | ((if PB5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21) | ((if PB6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22) | ((if PB7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23) |
            ((PB0_DATA_OUT as u32) << 24) | ((PB1_DATA_OUT as u32) << 25) | ((PB2_DATA_OUT as u32) << 26) | ((PB3_DATA_OUT as u32) << 27) |
            ((PB4_DATA_OUT as u32) << 28) | ((PB5_DATA_OUT as u32) << 29) | ((PB6_DATA_OUT as u32) << 30) | ((PB7_DATA_OUT as u32) << 31)
    );

    write_reg_gpio_pb_setting2(
        ((PB0_DATA_STRENGTH as u32) << 8) | ((PB1_DATA_STRENGTH as u32) << 9) | ((PB2_DATA_STRENGTH as u32) << 10) | ((PB3_DATA_STRENGTH as u32) << 11) |
            ((PB4_DATA_STRENGTH as u32) << 12) | ((PB5_DATA_STRENGTH as u32) << 13) | ((PB6_DATA_STRENGTH as u32) << 14) | ((PB7_DATA_STRENGTH as u32) << 15) |
            (if PB0_FUNC == AS_GPIO { BIT!(16) } else { 0 }) | (if PB1_FUNC == AS_GPIO { BIT!(17) } else { 0 }) | (if PB2_FUNC == AS_GPIO { BIT!(18) } else { 0 }) | (if PB3_FUNC == AS_GPIO { BIT!(19) } else { 0 }) |
            (if PB4_FUNC == AS_GPIO { BIT!(20) } else { 0 }) | (if PB5_FUNC == AS_GPIO { BIT!(21) } else { 0 }) | (if PB6_FUNC == AS_GPIO { BIT!(22) } else { 0 }) | (if PB7_FUNC == AS_GPIO { BIT!(23) } else { 0 })
    );

    write_reg_gpio_pc_setting1(
        ((PC0_INPUT_ENABLE as u32) << 8) | ((PC1_INPUT_ENABLE as u32) << 9) | ((PC2_INPUT_ENABLE as u32) << 10) | ((PC3_INPUT_ENABLE as u32) << 11) |
            ((PC4_INPUT_ENABLE as u32) << 12) | ((PC5_INPUT_ENABLE as u32) << 13) | ((PC6_INPUT_ENABLE as u32) << 14) | ((PC7_INPUT_ENABLE as u32) << 15) |
            ((if PC0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16) | ((if PC1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17) | ((if PC2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18) | ((if PC3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19) |
            ((if PC4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20) | ((if PC5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21) | ((if PC6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22) | ((if PC7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23) |
            ((PC0_DATA_OUT as u32) << 24) | ((PC1_DATA_OUT as u32) << 25) | ((PC2_DATA_OUT as u32) << 26) | ((PC3_DATA_OUT as u32) << 27) |
            ((PC4_DATA_OUT as u32) << 28) | ((PC5_DATA_OUT as u32) << 29) | ((PC6_DATA_OUT as u32) << 30) | ((PC7_DATA_OUT as u32) << 31)
    );

    write_reg_gpio_pc_setting2(
        ((PC0_DATA_STRENGTH as u32) << 8) | ((PC1_DATA_STRENGTH as u32) << 9) | ((PC2_DATA_STRENGTH as u32) << 10) | ((PC3_DATA_STRENGTH as u32) << 11) |
            ((PC4_DATA_STRENGTH as u32) << 12) | ((PC5_DATA_STRENGTH as u32) << 13) | ((PC6_DATA_STRENGTH as u32) << 14) | ((PC7_DATA_STRENGTH as u32) << 15) |
            (if PC0_FUNC == AS_GPIO { BIT!(16) } else { 0 }) | (if PC1_FUNC == AS_GPIO { BIT!(17) } else { 0 }) | (if PC2_FUNC == AS_GPIO { BIT!(18) } else { 0 }) | (if PC3_FUNC == AS_GPIO { BIT!(19) } else { 0 }) |
            (if PC4_FUNC == AS_GPIO { BIT!(20) } else { 0 }) | (if PC5_FUNC == AS_GPIO { BIT!(21) } else { 0 }) | (if PC6_FUNC == AS_GPIO { BIT!(22) } else { 0 }) | (if PC7_FUNC == AS_GPIO { BIT!(23) } else { 0 })
    );

    write_reg_gpio_pd_setting1(
        ((PD0_INPUT_ENABLE as u32) << 8) | ((PD1_INPUT_ENABLE as u32) << 9) | ((PD2_INPUT_ENABLE as u32) << 10) | ((PD3_INPUT_ENABLE as u32) << 11) |
            ((PD4_INPUT_ENABLE as u32) << 12) | ((PD5_INPUT_ENABLE as u32) << 13) | ((PD6_INPUT_ENABLE as u32) << 14) | ((PD7_INPUT_ENABLE as u32) << 15) |
            ((if PD0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16) | ((if PD1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17) | ((if PD2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18) | ((if PD3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19) |
            ((if PD4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20) | ((if PD5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21) | ((if PD6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22) | ((if PD7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23) |
            ((PD0_DATA_OUT as u32) << 24) | ((PD1_DATA_OUT as u32) << 25) | ((PD2_DATA_OUT as u32) << 26) | ((PD3_DATA_OUT as u32) << 27) |
            ((PD4_DATA_OUT as u32) << 28) | ((PD5_DATA_OUT as u32) << 29) | ((PD6_DATA_OUT as u32) << 30) | ((PD7_DATA_OUT as u32) << 31)
    );

    write_reg_gpio_pd_setting2(
        ((PD0_DATA_STRENGTH as u32) << 8) | ((PD1_DATA_STRENGTH as u32) << 9) | ((PD2_DATA_STRENGTH as u32) << 10) | ((PD3_DATA_STRENGTH as u32) << 11) |
            ((PD4_DATA_STRENGTH as u32) << 12) | ((PD5_DATA_STRENGTH as u32) << 13) | ((PD6_DATA_STRENGTH as u32) << 14) | ((PD7_DATA_STRENGTH as u32) << 15) |
            (if PD0_FUNC == AS_GPIO { BIT!(16) } else { 0 }) | (if PD1_FUNC == AS_GPIO { BIT!(17) } else { 0 }) | (if PD2_FUNC == AS_GPIO { BIT!(18) } else { 0 }) | (if PD3_FUNC == AS_GPIO { BIT!(19) } else { 0 }) |
            (if PD4_FUNC == AS_GPIO { BIT!(20) } else { 0 }) | (if PD5_FUNC == AS_GPIO { BIT!(21) } else { 0 }) | (if PD6_FUNC == AS_GPIO { BIT!(22) } else { 0 }) | (if PD7_FUNC == AS_GPIO { BIT!(23) } else { 0 })
    );

    write_reg_gpio_pe_setting1(
        ((PE0_INPUT_ENABLE as u32) << 8) | ((PE1_INPUT_ENABLE as u32) << 9) | ((PE2_INPUT_ENABLE as u32) << 10) | ((PE3_INPUT_ENABLE as u32) << 11) |
            ((PE4_INPUT_ENABLE as u32) << 12) | ((PE5_INPUT_ENABLE as u32) << 13) | ((PE6_INPUT_ENABLE as u32) << 14) | ((PE7_INPUT_ENABLE as u32) << 15) |
            ((if PE0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16) | ((if PE1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17) | ((if PE2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18) | ((if PE3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19) |
            ((if PE4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20) | ((if PE5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21) | ((if PE6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22) | ((if PE7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23) |
            ((PE0_DATA_OUT as u32) << 24) | ((PE1_DATA_OUT as u32) << 25) | ((PE2_DATA_OUT as u32) << 26) | ((PE3_DATA_OUT as u32) << 27) |
            ((PE4_DATA_OUT as u32) << 28) | ((PE5_DATA_OUT as u32) << 29) | ((PE6_DATA_OUT as u32) << 30) | ((PE7_DATA_OUT as u32) << 31)
    );

    write_reg_gpio_pe_setting2(
        ((PE0_DATA_STRENGTH as u32) << 8) | ((PE1_DATA_STRENGTH as u32) << 9) | ((PE2_DATA_STRENGTH as u32) << 10) | ((PE3_DATA_STRENGTH as u32) << 11) |
            ((PE4_DATA_STRENGTH as u32) << 12) | ((PE5_DATA_STRENGTH as u32) << 13) | ((PE6_DATA_STRENGTH as u32) << 14) | ((PE7_DATA_STRENGTH as u32) << 15) |
            (if PE0_FUNC == AS_GPIO { BIT!(16) } else { 0 }) | (if PE1_FUNC == AS_GPIO { BIT!(17) } else { 0 }) | (if PE2_FUNC == AS_GPIO { BIT!(18) } else { 0 }) | (if PE3_FUNC == AS_GPIO { BIT!(19) } else { 0 }) |
            (if PE4_FUNC == AS_GPIO { BIT!(20) } else { 0 }) | (if PE5_FUNC == AS_GPIO { BIT!(21) } else { 0 }) | (if PE6_FUNC == AS_GPIO { BIT!(22) } else { 0 }) | (if PE7_FUNC == AS_GPIO { BIT!(23) } else { 0 })
    );

    write_reg_gpio_pf_setting1(
        ((PF0_INPUT_ENABLE as u32) << 8) | ((PF1_INPUT_ENABLE as u32) << 9) |
            ((if PF0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16) | ((if PF1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17) |
            ((PF0_DATA_OUT as u32) << 24) | ((PF1_DATA_OUT as u32) << 25)
    );

    write_reg_gpio_pf_setting2(
        ((PF0_DATA_STRENGTH as u32) << 8) | ((PF1_DATA_STRENGTH as u32) << 9) |
            (if PF0_FUNC == AS_GPIO { BIT!(16) } else { 0 }) | (if PF1_FUNC == AS_GPIO { BIT!(17) } else { 0 })
    );

    /*  do later
    reg_gpio_config_func = ((if PA0_FUNC==AS_DMIC||PA4_FUNC==AS_DMIC) { BITS(0,7)} else {0}) | ((PA1_FUNC==AS_PWM { BIT!(2)} else {0}) |
        ((if PA2_FUNC==AS_UART||PA3_FUNC==AS_UART) { BITS(3,5)} else {0}) |
        ((PA2_FUNC==AS_PWM { BIT!(4)} else {0}) | ((PA3_FUNC==AS_PWM { BIT!(6)} else {0}) |
        (((PB0_FUNC==AS_SDM||PB1_FUNC==AS_SDM||PB6_FUNC==AS_SDM||PB7_FUNC==AS_SDM) { BIT_RNG(12,15)} else {0}) |
        ((if PA0_FUNC==AS_I2S||PA1_FUNC==AS_I2S||PA2_FUNC==AS_I2S||PA3_FUNC==AS_I2S||PA4_FUNC==AS_I2S) { (BIT_RNG(21,23)|BIT_RNG(29,30))} else {0});
    */


    let areg = analog_read__attribute_ram_code(0x0a) & 0x0f;

    analog_write__attribute_ram_code(0x0a, areg | (PULL_WAKEUP_SRC_PA0 << 4) |
        (PULL_WAKEUP_SRC_PA1 << 6));

    analog_write__attribute_ram_code(0x0b, PULL_WAKEUP_SRC_PA2 |
        (PULL_WAKEUP_SRC_PA3 << 2) |
        (PULL_WAKEUP_SRC_PA4 << 4) |
        (PULL_WAKEUP_SRC_PA5 << 6));

    analog_write__attribute_ram_code(0x0c, PULL_WAKEUP_SRC_PA6 |
        (PULL_WAKEUP_SRC_PA7 << 2) |
        (PULL_WAKEUP_SRC_PB0 << 4) |
        (PULL_WAKEUP_SRC_PB1 << 6));

    analog_write__attribute_ram_code(0x0d, PULL_WAKEUP_SRC_PB2 |
        (PULL_WAKEUP_SRC_PB3 << 2) |
        (PULL_WAKEUP_SRC_PB4 << 4) |
        (PULL_WAKEUP_SRC_PB5 << 6));

    analog_write__attribute_ram_code(0x0e, PULL_WAKEUP_SRC_PB6 |
        (PULL_WAKEUP_SRC_PB7 << 2) |
        (PULL_WAKEUP_SRC_PC0 << 4) |
        (PULL_WAKEUP_SRC_PC1 << 6));

    analog_write__attribute_ram_code(0x0f, PULL_WAKEUP_SRC_PC2 |
        (PULL_WAKEUP_SRC_PC3 << 2) |
        (PULL_WAKEUP_SRC_PC4 << 4) |
        (PULL_WAKEUP_SRC_PC5 << 6));

    analog_write__attribute_ram_code(0x10, PULL_WAKEUP_SRC_PC6 |
        (PULL_WAKEUP_SRC_PC7 << 2) |
        (PULL_WAKEUP_SRC_PD0 << 4) |
        (PULL_WAKEUP_SRC_PD1 << 6));

    analog_write__attribute_ram_code(0x11, PULL_WAKEUP_SRC_PD2 |
        (PULL_WAKEUP_SRC_PD3 << 2) |
        (PULL_WAKEUP_SRC_PD4 << 4) |
        (PULL_WAKEUP_SRC_PD5 << 6));

    analog_write__attribute_ram_code(0x12, PULL_WAKEUP_SRC_PD6 |
        (PULL_WAKEUP_SRC_PD7 << 2) |
        (PULL_WAKEUP_SRC_PE0 << 4) |
        (PULL_WAKEUP_SRC_PE1 << 6));

    analog_write__attribute_ram_code(0x13, PULL_WAKEUP_SRC_PE2 |
        (PULL_WAKEUP_SRC_PE3 << 2) |
        (PULL_WAKEUP_SRC_PE4 << 4) |
        (PULL_WAKEUP_SRC_PE5 << 6));

    analog_write__attribute_ram_code(0x14, PULL_WAKEUP_SRC_PE6 |
        (PULL_WAKEUP_SRC_PE7 << 2) |
        (PULL_WAKEUP_SRC_PF0 << 4) |
        (PULL_WAKEUP_SRC_PF1 << 6));
}

pub fn gpio_set_func(pin: u32, func: u8){
	let	bit : u8 = (pin & 0xff) as u8;
	if func == AS_GPIO {
        write_reg_gpio_gpio_func(read_reg_gpio_gpio_func(((pin>>8)<<3)) | bit, ((pin>>8)<<3));
		return;
	}else{
        write_reg_gpio_gpio_func(read_reg_gpio_gpio_func((pin>>8)<<3) & !bit, ((pin>>8)<<3));
	}
}