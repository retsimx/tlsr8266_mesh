use core::ptr::null_mut;
use crate::{app, BIT};
use crate::common::REGA_LIGHT_OFF;
use crate::sdk::common::compat::{load_tbl_cmd_set, TBLCMDSET};
use crate::sdk::light::RecoverStatus;
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::register::write_reg_pwdn_ctrl;
use crate::state::State;

pub fn usb_dp_pullup_en(en: bool) {
    let mut dat: u8 = analog_read(0x00);
    if en {
        dat &= !BIT!(4);
    } else {
        dat |= BIT!(4);
    }

    analog_write(0x00, dat);
}

pub enum PM_WAKEUP {
    // WAKEUP_SRC_ANA 0 -- 2  not supported
    CORE = BIT!(5),
    TIMER = BIT!(6),
    COMP = BIT!(7),
    PAD = BIT!(8),
}

//
// enum{
//     // WAKEUP_SRC_ANA 0 -- 2  not supported
//     WAKEUP_PC3_GRP0 = BIT(0),
//     WAKEUP_PC4_GRP0 = BIT(1),
//     WAKEUP_PC5_GRP0 = BIT(2),
//     WAKEUP_PD0_GRP0 = BIT(3),
//     WAKEUP_PD1_GRP1 = BIT(4),
//     WAKEUP_PD2_GRP1 = BIT(5),
//     WAKEUP_PD3_GRP1 = BIT(6),
//     WAKEUP_PD4_GRP1 = BIT(7),
//     WAKEUP_PD5_GRP2 = BIT(8),
//     WAKEUP_PD6_GRP2 = BIT(9),
//     WAKEUP_PD7_GRP2 = BIT(10),
//     WAKEUP_PA0_GRP2 = BIT(11),
//     WAKEUP_PA1_GRP3 = BIT(12),
//     WAKEUP_PA2_GRP3 = BIT(13),
//     WAKEUP_PA3_GRP3 = BIT(14),
//     WAKEUP_PA4_GRP3 = BIT(15),
//     WAKEUP_PA7_GRP4 = BIT(16),
//     WAKEUP_PC6_GRP4 = BIT(17),
//     WAKEUP_PC7_GRP4 = BIT(18),
//     WAKEUP_PE0_GRP4 = BIT(19),
//     WAKEUP_PE1_GRP5 = BIT(20),
//     WAKEUP_PE2_GRP5 = BIT(21),
//     WAKEUP_PA5_GRP5 = BIT(22),
//     WAKEUP_PA6_GRP5 = BIT(23),
// };
// /*wakeup-level*/
// enum{
//     WAKEUP_GRP0_POS_EDG = 0,
//     WAKEUP_GRP1_POS_EDG = 0,
//     WAKEUP_GRP2_POS_EDG = 0,
//     WAKEUP_GRP3_POS_EDG = 0,
//     WAKEUP_GRP4_POS_EDG = 0,
//     WAKEUP_GRP5_POS_EDG = 0,
//
//     WAKEUP_GRP0_NEG_EDG = BIT(0),
//     WAKEUP_GRP1_NEG_EDG = BIT(1),
//     WAKEUP_GRP2_NEG_EDG = BIT(2),
//     WAKEUP_GRP3_NEG_EDG = BIT(3),
//     WAKEUP_GRP4_NEG_EDG = BIT(4),
//     WAKEUP_GRP5_NEG_EDG = BIT(5),
//
// };
//
//
//
// ///////////////////////////////////////////////////////////////////////
// ////////////////////////////battery dectect////////////////////////////
// ////////////////////////////////////////////////////////////////////////
// /*test data
//  * standard      test
//  * 0.98v <--->1.022v ~ 1.024v
//  * 1.1v  <--->1.144v ~ 1.150v
//  * 1.18v <--->1.214v ~ 1.218v
//  * 1.25v <--->1.285v ~ 1.289v
//  * 1.3v  <--->1.355v ~ 1.358v
//  * 1.6v  <--->1.701v ~ 1.708v
//  * */
// enum  COMP_CHANNALE {
//     COMP_ANA3 = 0x00,	COMP_GPIO_GP11 = 0x00,
//     COMP_ANA4 = 0x02,	COMP_GPIO_GP12 = 0x02,
//     COMP_ANA5 = 0x04,	COMP_GPIO_SWS = 0x04,
//     COMP_ANA6 = 0x06,	COMP_GPIO_CN = 0x06,
//     COMP_ANA7 = 0x08,	COMP_GPIO_CK = 0x08,
//     COMP_ANA8 = 0x0a,	COMP_GPIO_DO = 0x0a,
//     COMP_ANA9 = 0x0c,	COMP_GPIO_DI = 0x0c,
//     COMP_ANA10 = 0x0e, 	COMP_GPIO_MSCN = 0x0e,
//     COMP_ANA11 = 0x10,	COMP_GPIO_MCLK = 0x10,
//     COMP_ANA12 = 0x12,	COMP_GPIO_MSDO = 0x12,
//     COMP_ANA13 = 0x14,	COMP_GPIO_MSDI = 0x14,
//     COMP_ANA14 = 0x16,	COMP_GPIO_DMIC_CK = 0x16,	COMP_GPIO_I2S_REFCLK = 0x16,
//     COMP_ANA15 = 0x18,	COMP_GPIO_I2S_BCK = 0x18,
//     COMP_AVDD =  0x1a
// };
//
// enum{
//     V0P98,
//     V1P1,
//     V1P18,
//     V1P25,
//     V1P3,
//     V1P66,
// };
//
// #define SCALING_SELECT_QUARTER 		0x00//25%
// #define SCALING_SELECT_HALF 		0x20//50%
// #define SCALING_SELECT_3QUARTER 	0x40//75%
// #define SCALING_SELECT_FULL 		0x60//100%
//
// #define REF_VOLTAGE_SEL_0			0x00//float
// #define REF_VOLTAGE_SEL_1			0x01//981mv
// #define REF_VOLTAGE_SEL_2			0x02//937mv
// #define REF_VOLTAGE_SEL_3			0x03//885mv
// #define REF_VOLTAGE_SEL_4			0x04//832mv
// #define REF_VOLTAGE_SEL_5			0x05//ana3
// #define REF_VOLTAGE_SEL_6			0x06//ain9
// #define REF_VOLTAGE_SEL_7			0x07//avddh
//
// #ifndef		VBAT_LOW_LEVLE
// #define		VBAT_LOW_LEVLE		V0P98
// #endif
//
// #ifndef		VBAT_CHANNEL
// #if BATTERY_DETECTION_WITH_LDO_SET
// #define		VBAT_CHANNEL		COMP_AVDD
// #else
// #define		VBAT_CHANNEL		COMP_ANA8
// #endif
// #endif
//
// #define		V0P98_REF			REF_VOLTAGE_SEL_1
// #define		V0P98_SCALE			SCALING_SELECT_FULL
//
// #define		V1P1_REF			REF_VOLTAGE_SEL_4
// #define		V1P1_SCALE			SCALING_SELECT_3QUARTER
//
// #define		V1P18_REF			REF_VOLTAGE_SEL_3
// #define		V1P18_SCALE			SCALING_SELECT_3QUARTER
//
// #define		V1P25_REF			REF_VOLTAGE_SEL_2
// #define		V1P25_SCALE			SCALING_SELECT_3QUARTER
//
// #define		V1P3_REF			REF_VOLTAGE_SEL_1
// #define		V1P3_SCALE			SCALING_SELECT_3QUARTER
//
// #define		V1P66_REF			REF_VOLTAGE_SEL_4
// #define		V1P66_SCALE			SCALING_SELECT_HALF
//
// #define		VBAT_LOW_SCALE		(VBAT_LOW_LEVLE==V0P98 ? V0P98_SCALE  : V1P1_SCALE )
// #define		VBAT_LOW_REF		(VBAT_LOW_LEVLE==V0P98 ? V0P98_REF : V1P1_REF)
//
// int battery_detection_with_ldo (u8 chn, int set);
// int battery_direct_detection (u8 chn, int set);
// int battery_low_by_set ( u8 chn, u8 v_ref, u8 v_scale ) ;
// int battery_low ();
// void battery_by_comp_init();
//
// enum {
//     WAKEUP_STATUS_COMP   = BIT(0),  //wakeup by comparator
//     WAKEUP_STATUS_TIMER  = BIT(1),
//     WAKEUP_STATUS_CORE   = BIT(2),
//     WAKEUP_STATUS_PAD    = BIT(3),
// };
//
// #define SUSPEND_MODE	0
// #define DEEPSLEEP_MODE	1
//
// #define WAKEUP_LEVEL_L 	0
// #define WAKEUP_LEVEL_H 	1
//
// // usually, we don't use gpio wakeup in suspend mode.
// // If you do need it,  pls turn on this micro, add set  wakeup pin before calling cpu_sleep_wakeup
//
// // like:
// // reg_gpio_f_wakeup_en = SUSPEND_WAKEUP_SRC_PWM0;
// // reg_gpio_f_pol = SUSPEND_WAKEUP_SRC_PWM0;
// // cpu_sleep_wakeup(1, 50, 0, 0)
// #define PM_SUSPEND_WAKEUP_BY_GPIO_ENABLE		0
//
// void pm_init(void);
// extern void cpu_set_system_tick (u32 tick);
// static inline int cpu_get_32k_tick (void) {
// return 1;
// }

//
// extern const u16 wakeup_src_pin[];
//
//
// #endif

pub const TCMD_UNDER_RD: u8 = 0x80;
pub const TCMD_UNDER_WR: u8 = 0x40;
pub const TCMD_UNDER_BOTH: u8 = 0xc0;
pub const TCMD_MASK: u8 = 0x3f;

pub const TCMD_WRITE: u8 = 0x3;
pub const TCMD_WAIT: u8 = 0x7;
pub const TCMD_WAREG: u8 = 0x8;

const tbl_cpu_wakeup_init: [TBLCMDSET; 0x13] = [
    TBLCMDSET {
        adr: 0x60,
        dat: 0,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x61,
        dat: 0,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x62,
        dat: 0,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x63,
        dat: 0xff,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x64,
        dat: 0xff,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x74,
        dat: 0x53,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x7c,
        dat: 0xf7,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x74,
        dat: 0,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x67,
        dat: 0,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x66,
        dat: 0,
        cmd: 0xc3, // 0b11000011
    },
    TBLCMDSET {
        adr: 0x73,
        dat: 0,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x620,
        dat: 1,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x74f,
        dat: 1,
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x81,
        dat: 0xc0,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x20,
        dat: 0xd0,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x2d,
        dat: 0x0f,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x81,
        dat: 0xc0,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x2c,
        dat: 0,
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x05,
        dat: 0x62,
        cmd: 0xc8,
    },
];

pub fn cpu_wakeup_init() {
    load_tbl_cmd_set(tbl_cpu_wakeup_init.as_ptr(), 0x13);
}

// recover status before software reboot
fn light_sw_reboot_callback(state: *mut State) {
    if state != null_mut() && unsafe { (*state).rf_slave_ota_busy } {
        // rf_slave_ota_busy means mesh ota master busy also.
        analog_write(
            REGA_LIGHT_OFF,
            if app().light_manager.is_light_off() {
                RecoverStatus::LightOff as u8
            } else {
                0
            },
        );
    }
}

#[link_section = ".ram_code"]
fn light_sw_reboot_ll()
{
    // In the original code it calls this to reset, but I can't get this function working.
    // unsafe { cpu_sleep_wakeup(1, 0x40, ((CLOCK_SYS_CLOCK_1US) * 10000) + read_reg_system_tick()); }

    // Instead, let's just reset the MCU as described in the docs
    write_reg_pwdn_ctrl(0x20);

    loop {}
}

pub fn light_sw_reboot(state: *mut State)
{
    irq_disable();
    light_sw_reboot_callback(state);
    light_sw_reboot_ll();
    return;
}