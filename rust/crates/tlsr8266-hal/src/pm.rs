use core::sync::atomic::{AtomicBool, Ordering};

use crate::mcu::analog::{analog_read, analog_write};
use crate::mcu::clock::sleep_us;
use crate::mcu::cmd_table::{load_tbl_cmd_set, TBLCMDSET};
use crate::mcu::gpio::{gpio_set_func, gpio_set_output_en, gpio_write, AS_GPIO};
use crate::mcu::irq_i::irq_disable;
use crate::mcu::watchdog::wd_clear;
use crate::BIT;
use tlsr8266_pac::{
    write_reg_clk_en1, write_reg_pwdn_ctrl, write_reg_rst1, write_reg_system_tick_ctrl, FldPwdnCtrl,
};

// Analog-retention register addresses and their sentinel values. These live in the
// SDK because `light_sw_reboot` owns the retention writes; `common` and
// `diagnostics` re-export them for application callers.

/// Analog register holding the light on/off state seen across a software reboot.
pub const REGA_LIGHT_OFF: u8 = 0x3a;
/// Analog register holding why the previous boot ended.
pub const REGA_RESET_SENTINEL: u8 = 0x3b;
/// Analog register holding the last breadcrumb written before the reset.
pub const REGA_BREADCRUMB: u8 = 0x3c;

/// The previous boot ended cleanly (power-on or software reboot).
pub const SENTINEL_CLEAN_REBOOT: u8 = 0x00;
/// The firmware is running; a watchdog reset leaves this value behind.
pub const SENTINEL_RUNNING: u8 = 0xA5;
/// The firmware panicked; `FLASH_ADR_PANIC_INFO` holds the message.
pub const SENTINEL_PANIC: u8 = 0xE1;

/// Bit flags written to [`REGA_LIGHT_OFF`] so the next boot can recover the
/// state the previous one was in.
pub enum RecoverStatus {
    LightOff = BIT!(0),
    MeshOtaMaster100 = BIT!(1),
}

/// Last published light on/off state, so the reboot path can record whether the
/// lamp was off without reaching into `LightManager`.
static REBOOT_LIGHT_OFF: AtomicBool = AtomicBool::new(true);

/// Set once an OTA update is under way, so the reboot path can tell it must
/// leave the recovery payload behind instead of reading `ota`'s staging state.
static REBOOT_OTA_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Publishes that an OTA update is in progress for `light_sw_reboot_callback`.
pub fn set_reboot_ota_active(active: bool) {
    REBOOT_OTA_ACTIVE.store(active, Ordering::Relaxed);
}

/// Publishes the current on/off state for `light_sw_reboot_callback`.
pub fn set_reboot_light_off_state(is_off: bool) {
    REBOOT_LIGHT_OFF.store(is_off, Ordering::Relaxed);
}

pub fn usb_dp_pullup_en(en: bool) {
    let mut dat: u8 = analog_read(0x00);
    if en {
        dat &= !BIT!(4);
    } else {
        dat |= BIT!(4);
    }

    analog_write(0x00, dat);
}

/// Wakeup source bits of `reg_wakeup_en`, named as they are in the datasheet.
#[allow(non_camel_case_types)]
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

const TBL_CPU_WAKEUP_INIT: [TBLCMDSET; 0x13] = [
    TBLCMDSET {
        adr: 0x60,
        dat: 0, // reg_rst0: release module resets (0 = not in reset)
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x61,
        dat: 0, // reg_rst1: release module resets (0 = not in reset)
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x62,
        dat: 0, // reg_rst2: release module resets (0 = not in reset)
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x63,
        dat: 0xff, // reg_clk_en0 (CLKEN0): enable all peripheral clocks
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x64,
        dat: 0xff, // reg_clk_en1 (CLKEN1): enable GPIO/DMA/UART/PWM/AES/SWIRE clocks
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x74,
        dat: 0x53, // reg_id_wr_en = REG_ID_WR_EN_UNLOCK (0x53): unlock reg_func_id writes
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x7c,
        dat: 0xf7, // reg_func_id = 0b1111_0111: enable chip functions, RF_ENABLE (bit6) set,
        // RISC32_ENABLE (bit3) cleared
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x74,
        dat: 0, // reg_id_wr_en = 0: re-lock reg_func_id after the modification above
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x67,
        dat: 0, // reg_i2s_step = 0: I2S clock disabled
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x66,
        dat: 0, // reg_clk_sel = 0: clock divider/source cleared (configured later by clock_init)
        cmd: 0xc3, // 0b11000011
    },
    TBLCMDSET {
        adr: 0x73,
        dat: 0, // reg_clk_mux_sel: 32k from internal RC, USB PHY clock from 48M PLL, LPR div 0
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x620,
        dat: 1, // reg_tmr_ctrl8: TMR0_EN (start timer 0)
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x74f,
        dat: 1, // reg_system_tick_ctrl: START (start the system tick)
        cmd: 0xc3,
    },
    TBLCMDSET {
        adr: 0x81,
        dat: 0xc0, // rega_xtal_csel: crystal oscillator supply/bias/freq trim
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x20,
        dat: 0xd0, // ana 0x20 (3V PM/wakeup domain): exact bitfields undocumented; sits just
        // before the pad wakeup polarity/enable registers 0x21-0x2b (DS Table 2-2)
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x2d,
        dat: 0x0f, // ana 0x2d: retention/scratch byte (rega_pkt_seq_no); survives sleep
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x81,
        dat: 0xc0, // rega_xtal_csel: same crystal oscillator setting, re-applied
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x2c,
        dat: 0, // ana 0x2c: retention/scratch byte (cleared)
        cmd: 0xc8,
    },
    TBLCMDSET {
        adr: 0x05,
        dat: 0x62, // rega_pwdn_setting1 = 0b0110_0010: power down 32K_XTAL (bit1),
        // BGIREF_3V (bit5), COMP (bit6); keep 32K_RC, 32M_RC, 32M_XTAL, LDO, TEMPSEN
        cmd: 0xc8,
    },
];

pub fn cpu_wakeup_init() {
    load_tbl_cmd_set(&TBL_CPU_WAKEUP_INIT);
}

// recover status before software reboot
fn light_sw_reboot_callback() {
    if REBOOT_OTA_ACTIVE.load(Ordering::Relaxed) {
        // rf_slave_ota_busy means mesh ota master busy also.
        analog_write(
            REGA_LIGHT_OFF,
            if REBOOT_LIGHT_OFF.load(Ordering::Relaxed) {
                RecoverStatus::LightOff as u8
            } else {
                0
            },
        );
    }
}

/// Enable peripheral clocks and the system tick for the boot-time OTA flash copy.
#[inline(always)]
pub fn ota_enable_clocks_for_copy() {
    write_reg_rst1(0);
    write_reg_clk_en1(0xff);
    write_reg_system_tick_ctrl(0x01);
}

/// Trigger an immediate hardware MCU reboot.
#[inline(always)]
pub fn mcu_trigger_reboot() {
    write_reg_pwdn_ctrl(FldPwdnCtrl::Reboot.bits());
}

#[link_section = ".ram_code"]
fn light_sw_reboot_ll() {
    // In the original code it calls this to reset, but I can't get this function working.
    // unsafe { cpu_sleep_wakeup(1, 0x40, ((CLOCK_SYS_CLOCK_1US) * 10000) + read_reg_system_tick()); }

    // Instead, let's just reset the MCU as described in the docs
    mcu_trigger_reboot();

    // Don't loop forever off-target - just return instead
    // The reboot write above has already fired: park without returning so no
    // caller resumes before the watchdog resets the part.
    #[cfg(target_os = "none")]
    #[allow(clippy::empty_loop)]
    {
        loop {}
    }
}

pub fn light_sw_reboot() {
    irq_disable();

    // A pending panic keeps its sentinel so the next boot can still report it
    if analog_read(REGA_RESET_SENTINEL) != SENTINEL_PANIC {
        analog_write(REGA_RESET_SENTINEL, SENTINEL_CLEAN_REBOOT);
    }

    light_sw_reboot_callback();
    light_sw_reboot_ll();
}

/// Blinks the OTA LED six times at 1 Hz to signal success. `pin` is passed by
/// the caller so this module needs no `crate::config` dependency.
pub fn blink_ota_led_ok(pin: u32) {
    gpio_set_func(pin, AS_GPIO);
    gpio_set_output_en(pin, 1);
    let mut led_on = true;
    for _ in 0..6 {
        gpio_write(pin, u32::from(led_on));
        led_on = !led_on;
        wd_clear();
        sleep_us(1000 * 1000);
    }
}

#[cfg(test)]
#[coverage(off)]
mod tests {
    use super::*;
    use crate::mcu::analog::fake_analog::FakeAnalog;
    use crate::mcu::clock::fake_clock;
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::REG_BASE_ADDR;

    /// Registers these tests read back directly.
    const IRQ_EN: u32 = 0x643;
    const PWDN_CTRL: u32 = 0x6f;
    const TMR_CTRL: u32 = 0x620;
    const GPIO_GPIO_FUNC: u32 = 0x586;
    const GPIO_OEN: u32 = 0x582;
    const GPIO_OUT: u32 = 0x583;

    /// `FakeMmio`, `FakeAnalog` and `fake_clock` are process-wide singletons and
    /// the suite runs with `--test-threads=1`, so every test starts from zero.
    /// The reboot mirrors are reset too because they are module statics.
    fn reset_fakes() {
        FakeMmio::reset();
        FakeAnalog::reset();
        fake_clock::reset();
        set_reboot_ota_active(false);
        set_reboot_light_off_state(false);
    }

    /// Tests usb_dp_pullup_en function with enable=true
    ///
    /// Verifies that when enabling USB DP pullup, BIT(4) is cleared
    /// in the analog register value.
    #[test]
    fn test_usb_dp_pullup_en_enable() {
        reset_fakes();
        // Setup: analog register 0x00 starts with BIT(4) set
        let initial_value = 0xFF;
        FakeAnalog::set(0x00, initial_value);

        // Execute
        usb_dp_pullup_en(true);

        // Verify: the analog register holds the value with BIT(4) cleared
        assert_eq!(FakeAnalog::get(0x00), initial_value & !BIT!(4));
    }

    /// Tests usb_dp_pullup_en function with enable=false
    ///
    /// Verifies that when disabling USB DP pullup, BIT(4) is set
    /// in the analog register value.
    #[test]
    fn test_usb_dp_pullup_en_disable() {
        reset_fakes();
        // Setup: analog register 0x00 starts with BIT(4) cleared
        let initial_value = 0x00;
        FakeAnalog::set(0x00, initial_value);

        // Execute
        usb_dp_pullup_en(false);

        // Verify: the analog register holds the value with BIT(4) set
        assert_eq!(FakeAnalog::get(0x00), initial_value | BIT!(4));
    }

    /// Tests cpu_wakeup_init function
    ///
    /// Verifies that cpu_wakeup_init runs the whole static wakeup command table:
    /// the register writes, the analog writes and the absence of any wait.
    #[test]
    fn test_cpu_wakeup_init() {
        reset_fakes();

        // Execute
        cpu_wakeup_init();

        // Every write command in `TBL_CPU_WAKEUP_INIT`, in table order.
        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: 0x60 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00
                },
                RegWrite {
                    addr: 0x61 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00
                },
                RegWrite {
                    addr: 0x62 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00
                },
                RegWrite {
                    addr: 0x63 | REG_BASE_ADDR,
                    width: 1,
                    value: 0xff
                },
                RegWrite {
                    addr: 0x64 | REG_BASE_ADDR,
                    width: 1,
                    value: 0xff
                },
                RegWrite {
                    addr: 0x74 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x53
                },
                RegWrite {
                    addr: 0x7c | REG_BASE_ADDR,
                    width: 1,
                    value: 0xf7
                },
                RegWrite {
                    addr: 0x74 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00
                },
                RegWrite {
                    addr: 0x67 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00
                },
                RegWrite {
                    addr: 0x66 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00
                },
                RegWrite {
                    addr: 0x73 | REG_BASE_ADDR,
                    width: 1,
                    value: 0x00
                },
                RegWrite {
                    addr: TMR_CTRL | REG_BASE_ADDR,
                    width: 1,
                    value: 0x01
                },
                RegWrite {
                    addr: 0x74f | REG_BASE_ADDR,
                    width: 1,
                    value: 0x01
                },
            ],
            "every write command must reach its register, in table order"
        );

        // Every analog-write command, as the analog bank now holds them.
        assert_eq!(FakeAnalog::get(0x81), 0xc0);
        assert_eq!(FakeAnalog::get(0x20), 0xd0);
        assert_eq!(FakeAnalog::get(0x2d), 0x0f);
        assert_eq!(FakeAnalog::get(0x2c), 0x00);
        assert_eq!(FakeAnalog::get(0x05), 0x62);

        // The table has no wait command
        assert!(fake_clock::sleeps().is_empty());
    }

    /// Tests light_sw_reboot_callback when no OTA update is active
    ///
    /// Verifies that when the OTA mirror is clear, no analog_write occurs.
    #[test]
    fn test_light_sw_reboot_callback_no_ota() {
        reset_fakes();
        // Setup: OTA mirror clear
        set_reboot_ota_active(false);
        set_reboot_light_off_state(false);
        // A value no callback path writes, so a stray write is visible
        FakeAnalog::set(REGA_LIGHT_OFF, 0x77);

        // Execute
        light_sw_reboot_callback();

        // Verify: the callback writes nothing when no OTA is in progress
        assert_eq!(
            FakeAnalog::get(REGA_LIGHT_OFF),
            0x77,
            "no OTA in progress must leave the recovery payload alone"
        );
    }

    /// Tests light_sw_reboot_callback when OTA is active and light is off
    ///
    /// Verifies that when the OTA mirror is set and the published state says the
    /// light is off, analog_write is called with RecoverStatus::LightOff.
    #[test]
    fn test_light_sw_reboot_callback_ota_light_off() {
        reset_fakes();
        // Setup: OTA mirror set, light is off
        set_reboot_ota_active(true);
        set_reboot_light_off_state(true);

        // Execute
        light_sw_reboot_callback();

        // Verify: the recovery payload records "light off"
        assert_eq!(
            FakeAnalog::get(REGA_LIGHT_OFF),
            RecoverStatus::LightOff as u8
        );
    }

    /// Tests light_sw_reboot_callback when OTA is active and light is on
    ///
    /// Verifies that when the OTA mirror is set and the published state says the
    /// light is on, analog_write is called with 0.
    #[test]
    fn test_light_sw_reboot_callback_ota_light_on() {
        reset_fakes();
        // Setup: OTA mirror set, light is on
        set_reboot_ota_active(true);
        set_reboot_light_off_state(false);
        FakeAnalog::set(REGA_LIGHT_OFF, 0x77);

        // Execute
        light_sw_reboot_callback();

        // Verify: the recovery payload records "light on" (zero)
        assert_eq!(FakeAnalog::get(REGA_LIGHT_OFF), 0);
    }

    /// The mirror is the callback's only input: clearing it after a call stops
    /// the next one from writing, so the decision is made per call and not
    /// latched by an earlier OTA update.
    #[test]
    fn test_light_sw_reboot_callback_follows_mirror() {
        reset_fakes();
        set_reboot_light_off_state(true);

        set_reboot_ota_active(true);
        light_sw_reboot_callback();
        assert_eq!(
            FakeAnalog::get(REGA_LIGHT_OFF),
            RecoverStatus::LightOff as u8
        );

        set_reboot_ota_active(false);
        // Overwrite the payload so a second write would be visible
        FakeAnalog::set(REGA_LIGHT_OFF, 0x77);
        light_sw_reboot_callback();
        assert_eq!(
            FakeAnalog::get(REGA_LIGHT_OFF),
            0x77,
            "clearing the mirror must stop the next callback from writing"
        );
    }

    /// The recovery payload written to `REGA_LIGHT_OFF` is a wire contract
    /// shared with the next boot and the host tools.
    #[test]
    fn test_recover_status_bit_values() {
        assert_eq!(RecoverStatus::LightOff as u8, 1);
        assert_eq!(RecoverStatus::MeshOtaMaster100 as u8, 2);
    }

    /// `ota_enable_clocks_for_copy` must release the peripheral resets, enable
    /// the peripheral clocks and start the system tick with those exact values:
    /// the boot-time flash copy cannot run without them.
    #[test]
    fn test_ota_enable_clocks_for_copy_writes_clock_registers() {
        reset_fakes();

        ota_enable_clocks_for_copy();

        assert_eq!(
            FakeMmio::writes(),
            vec![
                RegWrite {
                    addr: 0x61 | REG_BASE_ADDR,
                    width: 1,
                    value: 0,
                },
                RegWrite {
                    addr: 0x64 | REG_BASE_ADDR,
                    width: 1,
                    value: 0xff,
                },
                RegWrite {
                    addr: 0x74f | REG_BASE_ADDR,
                    width: 1,
                    value: 0x01,
                },
            ]
        );
    }

    /// `mcu_trigger_reboot` must fire the hardware reboot bit, not a magic
    /// number that could drift away from the PAC definition.
    #[test]
    fn test_mcu_trigger_reboot_writes_reboot_bit() {
        reset_fakes();

        mcu_trigger_reboot();

        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: PWDN_CTRL | REG_BASE_ADDR,
                width: 1,
                value: FldPwdnCtrl::Reboot.bits() as u32,
            }]
        );
        assert_eq!(FldPwdnCtrl::Reboot.bits(), 0x20);
    }

    /// `light_sw_reboot_ll` reboots through the shared helper, so the register
    /// effect is the same one `mcu_trigger_reboot` produces.
    #[test]
    fn test_light_sw_reboot_ll_triggers_reboot() {
        reset_fakes();

        light_sw_reboot_ll();

        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: PWDN_CTRL | REG_BASE_ADDR,
                width: 1,
                value: FldPwdnCtrl::Reboot.bits() as u32,
            }]
        );
    }

    /// `blink_ota_led_ok` configures the pin as an output and genuinely
    /// alternates it high/low over six cycles, 1 s apart, with the watchdog
    /// cleared between writes.
    ///
    /// The assertion is made on the real GPIO/clock leaves: the OUT register
    /// write log is the level sequence shifted into the pin's bit
    /// (`[0x02, 0x00, 0x02, 0x00, 0x02, 0x00]`), the watchdog control register
    /// is written once per cycle, and `fake_clock` records six 1 s sleeps.
    ///
    /// The old `led_onoff: u32` toggle produced `1, 0xFFFF_FFFE, 1, ...`, all of
    /// which `gpio_write` treats as "high", so the old sequence was six set
    /// writes and the LED never blinked.
    #[test]
    fn test_blink_ota_led_ok_alternates_pin_six_times() {
        reset_fakes();

        const PIN: u32 = 0x02; // port A, pin id low byte is the bit mask itself
        const PIN_MASK: u8 = 0x02;

        blink_ota_led_ok(PIN);

        // The pin was switched to GPIO function and enabled as an output.
        assert_eq!(FakeMmio::reg8(GPIO_GPIO_FUNC), PIN_MASK);
        assert_eq!(FakeMmio::reg8(GPIO_OEN), 0x00);

        // Six OUT-register writes that alternate high, low, high, low, high, low.
        let out_writes: Vec<u32> = FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == GPIO_OUT | REG_BASE_ADDR)
            .map(|w| w.value)
            .collect();
        assert_eq!(
            out_writes,
            vec![PIN_MASK as u32, 0, PIN_MASK as u32, 0, PIN_MASK as u32, 0],
            "the OTA LED must be driven high and low alternately, six times"
        );

        // The watchdog was cleared once per cycle.
        let wd_writes = FakeMmio::writes()
            .into_iter()
            .filter(|w| w.addr == TMR_CTRL | REG_BASE_ADDR)
            .count();
        assert_eq!(wd_writes, 6, "the watchdog must be cleared once per cycle");

        // Six one-second waits were requested.
        assert_eq!(fake_clock::sleeps(), vec![1000 * 1000; 6]);
    }

    /// Tests light_sw_reboot function
    ///
    /// Verifies that light_sw_reboot masks interrupts, records a clean reboot
    /// and then fires the hardware reboot bit.
    #[test]
    fn test_light_sw_reboot() {
        reset_fakes();
        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_RUNNING);

        // Execute
        light_sw_reboot();

        // Verify: interrupts were masked
        assert_eq!(FakeMmio::reg8(IRQ_EN), 0);

        // Verify: the running sentinel was replaced by the clean-reboot one
        assert_eq!(FakeAnalog::get(REGA_RESET_SENTINEL), SENTINEL_CLEAN_REBOOT);

        // Verify: the reboot bit was fired
        assert_eq!(FakeMmio::reg8(PWDN_CTRL), FldPwdnCtrl::Reboot.bits());
    }

    /// A watchdog or user reboot overwrites the running sentinel with "clean",
    /// so the next boot does not report a phantom watchdog reset.
    #[test]
    fn test_light_sw_reboot_sets_clean_sentinel() {
        reset_fakes();
        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_RUNNING);

        // Execute
        light_sw_reboot();

        // Verify: the running sentinel was cleared exactly once
        assert_eq!(FakeAnalog::get(REGA_RESET_SENTINEL), SENTINEL_CLEAN_REBOOT);
    }

    /// A panic reboot keeps the panic sentinel intact so the next boot can
    /// stream the panic message left in flash.
    #[test]
    fn test_light_sw_reboot_preserves_panic_sentinel() {
        reset_fakes();
        FakeAnalog::set(REGA_RESET_SENTINEL, SENTINEL_PANIC);

        // Execute
        light_sw_reboot();

        // Verify: no clean-reboot write, which would hide the panic
        assert_eq!(
            FakeAnalog::get(REGA_RESET_SENTINEL),
            SENTINEL_PANIC,
            "a panic sentinel must survive the reboot"
        );
    }

    /// Tests PM_WAKEUP enum values
    ///
    /// Verifies that PM_WAKEUP enum variants have the correct bit values.
    #[test]
    fn test_pm_wakeup_enum_values() {
        assert_eq!(PM_WAKEUP::CORE as u32, BIT!(5));
        assert_eq!(PM_WAKEUP::TIMER as u32, BIT!(6));
        assert_eq!(PM_WAKEUP::COMP as u32, BIT!(7));
        assert_eq!(PM_WAKEUP::PAD as u32, BIT!(8));
    }
}
