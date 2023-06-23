use crate::sdk::mcu::gpio::GPIO_PIN_TYPE;
// General stuff

// 0~max_mesh_name_len bytes  (strlen(adv_data) + strlen(MESH_NAME) + sizeof(LlAdvPrivateT))<=31
pub const MESH_NAME: &str = "out_of_mesh";
// max 16 bytes
pub const MESH_PWD: &str = "123";
// max 16 bytes
pub const MESH_PWD_ENCODE_SK: &str = "0123456789ABCDEF";
// max 16 bytes, random data from master
pub const MESH_LTK: [u8; 16] = [
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
];

pub const DEVICE_NAME: &[u8] = b"Telink tLight";

pub const OUT_OF_MESH: &str = "out_of_mesh";

pub const MAX_LUM_BRIGHTNESS_VALUE: u16 = 0x7fff;

pub const VENDOR_ID: u16 = 0x0211;

pub const PAIR_VALID_FLAG: u8 = 0xFA;
pub const PANIC_VALID_FLAG: u8 = 0xDE;

// don't use GPIO_PB0(pwm5) of 8266, because its default status is ouput 1 as GPIO.
pub const PWM_G: GPIO_PIN_TYPE = GPIO_PIN_TYPE::GPIO_PC0;
pub const PWM_B: GPIO_PIN_TYPE = GPIO_PIN_TYPE::GPIO_PC2;
// pub const PWM_W: GPIO_PIN_TYPE = GPIO_PIN_TYPE::GPIO_PA1;

pub const PWMID_G: u32 = 0;
pub const PWMID_B: u32 = 1;
// pub const PWMID_W: u32 = 3;

pub const OTA_LED: GPIO_PIN_TYPE = PWM_G;

pub const FLASH_SECTOR_SIZE: u16 = 4096;

// 512K flash
pub const FLASH_ADR_MAC: u32 = 0x76000;
pub const FLASH_ADR_PAIRING: u32 = 0x77000;
pub const FLASH_ADR_LUM: u32 = 0x78000;
pub const FLASH_ADR_DEV_GRP_ADR: u32 = 0x79000;
pub const FLASH_ADR_RESET_CNT: u32 = 0x7A000;
pub const FLASH_ADR_PANIC_INFO: u32 = 0x7B000;
pub const FLASH_ADR_LIGHT_NEW_FW: u32 = 0x40000;
