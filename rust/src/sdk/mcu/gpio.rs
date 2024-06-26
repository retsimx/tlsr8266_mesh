use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::gpio::GPIO_PIN_TYPE::*;
use crate::sdk::mcu::register::{
    read_reg_gpio_gpio_func, read_reg_gpio_oen, read_reg_gpio_out, write_reg_gpio_gpio_func,
    write_reg_gpio_oen, write_reg_gpio_out, write_reg_gpio_pa_setting1, write_reg_gpio_pa_setting2,
    write_reg_gpio_pb_setting1, write_reg_gpio_pb_setting2, write_reg_gpio_pc_setting1,
    write_reg_gpio_pc_setting2, write_reg_gpio_pd_setting1, write_reg_gpio_pd_setting2,
    write_reg_gpio_pe_setting1, write_reg_gpio_pe_setting2, write_reg_gpio_pf_setting1,
    write_reg_gpio_pf_setting2,
};
use crate::{regrw_idx, BIT};
use crate::{BM_CLR, BM_SET};

pub const RF_FAST_MODE_1M: u8 = 1;
pub const PM_PIN_PULL_DEFAULT: u8 = 1;

pub const GPIO_PULL_UP_0: u8 = 0;
pub const GPIO_PULL_UP_1M: u8 = 1;
pub const GPIO_PULL_UP_10K: u8 = 2;
pub const GPIO_PULL_DN_100K: u8 = 3;

pub enum GPIO_DIR {
    IN = 0,
    OUT = 1,
}

pub const AS_GPIO: u8 = 0;
pub const AS_MSPI: u8 = 1;
pub const AS_SWIRE: u8 = 2;
pub const AS_UART: u8 = 3;
pub const AS_PWM: u8 = 4;
pub const AS_I2C: u8 = 5;
pub const AS_SPI: u8 = 6;
pub const AS_ETH_MAC: u8 = 7;
pub const AS_I2S: u8 = 8;
pub const AS_SDM: u8 = 9;
pub const AS_DMIC: u8 = 10;
pub const AS_USB: u8 = 11;
pub const AS_SWS: u8 = 12;
pub const AS_SWM: u8 = 13;
pub const AS_TEST: u8 = 14;
pub const AS_ADC: u8 = 15;

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
        ((PA0_INPUT_ENABLE as u32) << 8)
            | ((PA1_INPUT_ENABLE as u32) << 9)
            | ((PA2_INPUT_ENABLE as u32) << 10)
            | ((PA3_INPUT_ENABLE as u32) << 11)
            | ((PA4_INPUT_ENABLE as u32) << 12)
            | ((PA5_INPUT_ENABLE as u32) << 13)
            | ((PA6_INPUT_ENABLE as u32) << 14)
            | ((PA7_INPUT_ENABLE as u32) << 15)
            | ((if PA0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)
            | ((if PA1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)
            | ((if PA2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)
            | ((if PA3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)
            | ((if PA4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)
            | ((if PA5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)
            | ((if PA6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)
            | ((if PA7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)
            | ((PA0_DATA_OUT as u32) << 24)
            | ((PA1_DATA_OUT as u32) << 25)
            | ((PA2_DATA_OUT as u32) << 26)
            | ((PA3_DATA_OUT as u32) << 27)
            | ((PA4_DATA_OUT as u32) << 28)
            | ((PA5_DATA_OUT as u32) << 29)
            | ((PA6_DATA_OUT as u32) << 30)
            | ((PA7_DATA_OUT as u32) << 31),
    );

    write_reg_gpio_pa_setting2(
        ((PA0_DATA_STRENGTH as u32) << 8)
            | ((PA1_DATA_STRENGTH as u32) << 9)
            | ((PA2_DATA_STRENGTH as u32) << 10)
            | ((PA3_DATA_STRENGTH as u32) << 11)
            | ((PA4_DATA_STRENGTH as u32) << 12)
            | ((PA5_DATA_STRENGTH as u32) << 13)
            | ((PA6_DATA_STRENGTH as u32) << 14)
            | ((PA7_DATA_STRENGTH as u32) << 15)
            | (if PA0_FUNC == AS_GPIO { BIT!(16) } else { 0 })
            | (if PA1_FUNC == AS_GPIO { BIT!(17) } else { 0 })
            | (if PA2_FUNC == AS_GPIO { BIT!(18) } else { 0 })
            | (if PA3_FUNC == AS_GPIO { BIT!(19) } else { 0 })
            | (if PA4_FUNC == AS_GPIO { BIT!(20) } else { 0 })
            | (if PA5_FUNC == AS_GPIO { BIT!(21) } else { 0 })
            | (if PA6_FUNC == AS_GPIO { BIT!(22) } else { 0 })
            | (if PA7_FUNC == AS_GPIO { BIT!(23) } else { 0 }),
    );

    write_reg_gpio_pb_setting1(
        ((PB0_INPUT_ENABLE as u32) << 8)
            | ((PB1_INPUT_ENABLE as u32) << 9)
            | ((PB2_INPUT_ENABLE as u32) << 10)
            | ((PB3_INPUT_ENABLE as u32) << 11)
            | ((PB4_INPUT_ENABLE as u32) << 12)
            | ((PB5_INPUT_ENABLE as u32) << 13)
            | ((PB6_INPUT_ENABLE as u32) << 14)
            | ((PB7_INPUT_ENABLE as u32) << 15)
            | ((if PB0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)
            | ((if PB1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)
            | ((if PB2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)
            | ((if PB3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)
            | ((if PB4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)
            | ((if PB5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)
            | ((if PB6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)
            | ((if PB7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)
            | ((PB0_DATA_OUT as u32) << 24)
            | ((PB1_DATA_OUT as u32) << 25)
            | ((PB2_DATA_OUT as u32) << 26)
            | ((PB3_DATA_OUT as u32) << 27)
            | ((PB4_DATA_OUT as u32) << 28)
            | ((PB5_DATA_OUT as u32) << 29)
            | ((PB6_DATA_OUT as u32) << 30)
            | ((PB7_DATA_OUT as u32) << 31),
    );

    write_reg_gpio_pb_setting2(
        ((PB0_DATA_STRENGTH as u32) << 8)
            | ((PB1_DATA_STRENGTH as u32) << 9)
            | ((PB2_DATA_STRENGTH as u32) << 10)
            | ((PB3_DATA_STRENGTH as u32) << 11)
            | ((PB4_DATA_STRENGTH as u32) << 12)
            | ((PB5_DATA_STRENGTH as u32) << 13)
            | ((PB6_DATA_STRENGTH as u32) << 14)
            | ((PB7_DATA_STRENGTH as u32) << 15)
            | (if PB0_FUNC == AS_GPIO { BIT!(16) } else { 0 })
            | (if PB1_FUNC == AS_GPIO { BIT!(17) } else { 0 })
            | (if PB2_FUNC == AS_GPIO { BIT!(18) } else { 0 })
            | (if PB3_FUNC == AS_GPIO { BIT!(19) } else { 0 })
            | (if PB4_FUNC == AS_GPIO { BIT!(20) } else { 0 })
            | (if PB5_FUNC == AS_GPIO { BIT!(21) } else { 0 })
            | (if PB6_FUNC == AS_GPIO { BIT!(22) } else { 0 })
            | (if PB7_FUNC == AS_GPIO { BIT!(23) } else { 0 }),
    );

    write_reg_gpio_pc_setting1(
        ((PC0_INPUT_ENABLE as u32) << 8)
            | ((PC1_INPUT_ENABLE as u32) << 9)
            | ((PC2_INPUT_ENABLE as u32) << 10)
            | ((PC3_INPUT_ENABLE as u32) << 11)
            | ((PC4_INPUT_ENABLE as u32) << 12)
            | ((PC5_INPUT_ENABLE as u32) << 13)
            | ((PC6_INPUT_ENABLE as u32) << 14)
            | ((PC7_INPUT_ENABLE as u32) << 15)
            | ((if PC0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)
            | ((if PC1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)
            | ((if PC2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)
            | ((if PC3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)
            | ((if PC4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)
            | ((if PC5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)
            | ((if PC6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)
            | ((if PC7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)
            | ((PC0_DATA_OUT as u32) << 24)
            | ((PC1_DATA_OUT as u32) << 25)
            | ((PC2_DATA_OUT as u32) << 26)
            | ((PC3_DATA_OUT as u32) << 27)
            | ((PC4_DATA_OUT as u32) << 28)
            | ((PC5_DATA_OUT as u32) << 29)
            | ((PC6_DATA_OUT as u32) << 30)
            | ((PC7_DATA_OUT as u32) << 31),
    );

    write_reg_gpio_pc_setting2(
        ((PC0_DATA_STRENGTH as u32) << 8)
            | ((PC1_DATA_STRENGTH as u32) << 9)
            | ((PC2_DATA_STRENGTH as u32) << 10)
            | ((PC3_DATA_STRENGTH as u32) << 11)
            | ((PC4_DATA_STRENGTH as u32) << 12)
            | ((PC5_DATA_STRENGTH as u32) << 13)
            | ((PC6_DATA_STRENGTH as u32) << 14)
            | ((PC7_DATA_STRENGTH as u32) << 15)
            | (if PC0_FUNC == AS_GPIO { BIT!(16) } else { 0 })
            | (if PC1_FUNC == AS_GPIO { BIT!(17) } else { 0 })
            | (if PC2_FUNC == AS_GPIO { BIT!(18) } else { 0 })
            | (if PC3_FUNC == AS_GPIO { BIT!(19) } else { 0 })
            | (if PC4_FUNC == AS_GPIO { BIT!(20) } else { 0 })
            | (if PC5_FUNC == AS_GPIO { BIT!(21) } else { 0 })
            | (if PC6_FUNC == AS_GPIO { BIT!(22) } else { 0 })
            | (if PC7_FUNC == AS_GPIO { BIT!(23) } else { 0 }),
    );

    write_reg_gpio_pd_setting1(
        ((PD0_INPUT_ENABLE as u32) << 8)
            | ((PD1_INPUT_ENABLE as u32) << 9)
            | ((PD2_INPUT_ENABLE as u32) << 10)
            | ((PD3_INPUT_ENABLE as u32) << 11)
            | ((PD4_INPUT_ENABLE as u32) << 12)
            | ((PD5_INPUT_ENABLE as u32) << 13)
            | ((PD6_INPUT_ENABLE as u32) << 14)
            | ((PD7_INPUT_ENABLE as u32) << 15)
            | ((if PD0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)
            | ((if PD1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)
            | ((if PD2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)
            | ((if PD3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)
            | ((if PD4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)
            | ((if PD5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)
            | ((if PD6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)
            | ((if PD7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)
            | ((PD0_DATA_OUT as u32) << 24)
            | ((PD1_DATA_OUT as u32) << 25)
            | ((PD2_DATA_OUT as u32) << 26)
            | ((PD3_DATA_OUT as u32) << 27)
            | ((PD4_DATA_OUT as u32) << 28)
            | ((PD5_DATA_OUT as u32) << 29)
            | ((PD6_DATA_OUT as u32) << 30)
            | ((PD7_DATA_OUT as u32) << 31),
    );

    write_reg_gpio_pd_setting2(
        ((PD0_DATA_STRENGTH as u32) << 8)
            | ((PD1_DATA_STRENGTH as u32) << 9)
            | ((PD2_DATA_STRENGTH as u32) << 10)
            | ((PD3_DATA_STRENGTH as u32) << 11)
            | ((PD4_DATA_STRENGTH as u32) << 12)
            | ((PD5_DATA_STRENGTH as u32) << 13)
            | ((PD6_DATA_STRENGTH as u32) << 14)
            | ((PD7_DATA_STRENGTH as u32) << 15)
            | (if PD0_FUNC == AS_GPIO { BIT!(16) } else { 0 })
            | (if PD1_FUNC == AS_GPIO { BIT!(17) } else { 0 })
            | (if PD2_FUNC == AS_GPIO { BIT!(18) } else { 0 })
            | (if PD3_FUNC == AS_GPIO { BIT!(19) } else { 0 })
            | (if PD4_FUNC == AS_GPIO { BIT!(20) } else { 0 })
            | (if PD5_FUNC == AS_GPIO { BIT!(21) } else { 0 })
            | (if PD6_FUNC == AS_GPIO { BIT!(22) } else { 0 })
            | (if PD7_FUNC == AS_GPIO { BIT!(23) } else { 0 }),
    );

    write_reg_gpio_pe_setting1(
        ((PE0_INPUT_ENABLE as u32) << 8)
            | ((PE1_INPUT_ENABLE as u32) << 9)
            | ((PE2_INPUT_ENABLE as u32) << 10)
            | ((PE3_INPUT_ENABLE as u32) << 11)
            | ((PE4_INPUT_ENABLE as u32) << 12)
            | ((PE5_INPUT_ENABLE as u32) << 13)
            | ((PE6_INPUT_ENABLE as u32) << 14)
            | ((PE7_INPUT_ENABLE as u32) << 15)
            | ((if PE0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)
            | ((if PE1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)
            | ((if PE2_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 18)
            | ((if PE3_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 19)
            | ((if PE4_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 20)
            | ((if PE5_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 21)
            | ((if PE6_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 22)
            | ((if PE7_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 23)
            | ((PE0_DATA_OUT as u32) << 24)
            | ((PE1_DATA_OUT as u32) << 25)
            | ((PE2_DATA_OUT as u32) << 26)
            | ((PE3_DATA_OUT as u32) << 27)
            | ((PE4_DATA_OUT as u32) << 28)
            | ((PE5_DATA_OUT as u32) << 29)
            | ((PE6_DATA_OUT as u32) << 30)
            | ((PE7_DATA_OUT as u32) << 31),
    );

    write_reg_gpio_pe_setting2(
        ((PE0_DATA_STRENGTH as u32) << 8)
            | ((PE1_DATA_STRENGTH as u32) << 9)
            | ((PE2_DATA_STRENGTH as u32) << 10)
            | ((PE3_DATA_STRENGTH as u32) << 11)
            | ((PE4_DATA_STRENGTH as u32) << 12)
            | ((PE5_DATA_STRENGTH as u32) << 13)
            | ((PE6_DATA_STRENGTH as u32) << 14)
            | ((PE7_DATA_STRENGTH as u32) << 15)
            | (if PE0_FUNC == AS_GPIO { BIT!(16) } else { 0 })
            | (if PE1_FUNC == AS_GPIO { BIT!(17) } else { 0 })
            | (if PE2_FUNC == AS_GPIO { BIT!(18) } else { 0 })
            | (if PE3_FUNC == AS_GPIO { BIT!(19) } else { 0 })
            | (if PE4_FUNC == AS_GPIO { BIT!(20) } else { 0 })
            | (if PE5_FUNC == AS_GPIO { BIT!(21) } else { 0 })
            | (if PE6_FUNC == AS_GPIO { BIT!(22) } else { 0 })
            | (if PE7_FUNC == AS_GPIO { BIT!(23) } else { 0 }),
    );

    write_reg_gpio_pf_setting1(
        ((PF0_INPUT_ENABLE as u32) << 8)
            | ((PF1_INPUT_ENABLE as u32) << 9)
            | ((if PF0_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 16)
            | ((if PF1_OUTPUT_ENABLE != 0 { 0 } else { 1 }) << 17)
            | ((PF0_DATA_OUT as u32) << 24)
            | ((PF1_DATA_OUT as u32) << 25),
    );

    write_reg_gpio_pf_setting2(
        ((PF0_DATA_STRENGTH as u32) << 8)
            | ((PF1_DATA_STRENGTH as u32) << 9)
            | (if PF0_FUNC == AS_GPIO { BIT!(16) } else { 0 })
            | (if PF1_FUNC == AS_GPIO { BIT!(17) } else { 0 }),
    );
    return;

    /*  do later
    reg_gpio_config_func = ((if PA0_FUNC==AS_DMIC||PA4_FUNC==AS_DMIC) { BITS(0,7)} else {0}) | ((PA1_FUNC==AS_PWM { BIT!(2)} else {0}) |
        ((if PA2_FUNC==AS_UART||PA3_FUNC==AS_UART) { BITS(3,5)} else {0}) |
        ((PA2_FUNC==AS_PWM { BIT!(4)} else {0}) | ((PA3_FUNC==AS_PWM { BIT!(6)} else {0}) |
        (((PB0_FUNC==AS_SDM||PB1_FUNC==AS_SDM||PB6_FUNC==AS_SDM||PB7_FUNC==AS_SDM) { BIT_RNG(12,15)} else {0}) |
        ((if PA0_FUNC==AS_I2S||PA1_FUNC==AS_I2S||PA2_FUNC==AS_I2S||PA3_FUNC==AS_I2S||PA4_FUNC==AS_I2S) { (BIT_RNG(21,23)|BIT_RNG(29,30))} else {0});
    */

    let areg = analog_read(0x0a) & 0x0f;

    analog_write(
        0x0a,
        areg | (PULL_WAKEUP_SRC_PA0 << 4) | (PULL_WAKEUP_SRC_PA1 << 6),
    );

    analog_write(
        0x0b,
        PULL_WAKEUP_SRC_PA2
            | (PULL_WAKEUP_SRC_PA3 << 2)
            | (PULL_WAKEUP_SRC_PA4 << 4)
            | (PULL_WAKEUP_SRC_PA5 << 6),
    );

    analog_write(
        0x0c,
        PULL_WAKEUP_SRC_PA6
            | (PULL_WAKEUP_SRC_PA7 << 2)
            | (PULL_WAKEUP_SRC_PB0 << 4)
            | (PULL_WAKEUP_SRC_PB1 << 6),
    );

    analog_write(
        0x0d,
        PULL_WAKEUP_SRC_PB2
            | (PULL_WAKEUP_SRC_PB3 << 2)
            | (PULL_WAKEUP_SRC_PB4 << 4)
            | (PULL_WAKEUP_SRC_PB5 << 6),
    );

    analog_write(
        0x0e,
        PULL_WAKEUP_SRC_PB6
            | (PULL_WAKEUP_SRC_PB7 << 2)
            | (PULL_WAKEUP_SRC_PC0 << 4)
            | (PULL_WAKEUP_SRC_PC1 << 6),
    );

    analog_write(
        0x0f,
        PULL_WAKEUP_SRC_PC2
            | (PULL_WAKEUP_SRC_PC3 << 2)
            | (PULL_WAKEUP_SRC_PC4 << 4)
            | (PULL_WAKEUP_SRC_PC5 << 6),
    );

    analog_write(
        0x10,
        PULL_WAKEUP_SRC_PC6
            | (PULL_WAKEUP_SRC_PC7 << 2)
            | (PULL_WAKEUP_SRC_PD0 << 4)
            | (PULL_WAKEUP_SRC_PD1 << 6),
    );

    analog_write(
        0x11,
        PULL_WAKEUP_SRC_PD2
            | (PULL_WAKEUP_SRC_PD3 << 2)
            | (PULL_WAKEUP_SRC_PD4 << 4)
            | (PULL_WAKEUP_SRC_PD5 << 6),
    );

    analog_write(
        0x12,
        PULL_WAKEUP_SRC_PD6
            | (PULL_WAKEUP_SRC_PD7 << 2)
            | (PULL_WAKEUP_SRC_PE0 << 4)
            | (PULL_WAKEUP_SRC_PE1 << 6),
    );

    analog_write(
        0x13,
        PULL_WAKEUP_SRC_PE2
            | (PULL_WAKEUP_SRC_PE3 << 2)
            | (PULL_WAKEUP_SRC_PE4 << 4)
            | (PULL_WAKEUP_SRC_PE5 << 6),
    );

    analog_write(
        0x14,
        PULL_WAKEUP_SRC_PE6
            | (PULL_WAKEUP_SRC_PE7 << 2)
            | (PULL_WAKEUP_SRC_PF0 << 4)
            | (PULL_WAKEUP_SRC_PF1 << 6),
    );
}

pub fn gpio_set_func(pin: u32, func: u8) {
    let bit: u8 = (pin & 0xff) as u8;
    if func == AS_GPIO {
        write_reg_gpio_gpio_func(
            read_reg_gpio_gpio_func(((pin >> 8) << 3)) | bit,
            ((pin >> 8) << 3),
        );
        return;
    } else {
        write_reg_gpio_gpio_func(
            read_reg_gpio_gpio_func((pin >> 8) << 3) & !bit,
            ((pin >> 8) << 3),
        );
    }
}

pub fn gpio_set_output_en(mut pin: u32, value: u32) {
    let bit = (pin & 0xff) as u8;
    pin = (pin >> 8) << 3;
    if value == 0 {
        let mut val = read_reg_gpio_oen(pin);
        BM_SET!(val, bit);
        write_reg_gpio_oen(val, pin);
    } else {
        let mut val = read_reg_gpio_oen(pin);
        BM_CLR!(val, bit);
        write_reg_gpio_oen(val, pin);
    }
}

pub fn gpio_set_input_en(mut pin: u32, value: u32) {
    let bit = (pin & 0xff) as u8;
    pin = (pin >> 8) << 3;
    if value != 0 {
        let mut val = read_reg_gpio_oen(pin);
        BM_SET!(val, bit);
        write_reg_gpio_oen(val, pin);
    } else {
        let mut val = read_reg_gpio_oen(pin);
        BM_CLR!(val, bit);
        write_reg_gpio_oen(val, pin);
    }
}

pub fn gpio_write(mut pin: u32, value: u32) {
    let bit = (pin & 0xff) as u8;
    pin = (pin >> 8) << 3;
    if value != 0 {
        let mut val = read_reg_gpio_out(pin);
        BM_SET!(val, bit);
        write_reg_gpio_out(val, pin);
    } else {
        let mut val = read_reg_gpio_out(pin);
        BM_CLR!(val, bit);
        write_reg_gpio_out(val, pin);
    }
}
