use core::sync::atomic::{AtomicBool, Ordering};

use crate::sdk::common::sync::SimplifyLS;
use crate::sdk::mcu::analog::{analog_read, analog_write};
use crate::sdk::mcu::clock::sleep_us;
use crate::sdk::mcu::cmd_table::{load_tbl_cmd_set, TBLCMDSET};
use crate::sdk::mcu::gpio::{gpio_set_func, gpio_set_output_en, gpio_write, AS_GPIO};
use crate::sdk::mcu::irq_i::irq_disable;
use crate::sdk::mcu::watchdog::wd_clear;
use crate::BIT;
use critical_section;
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
/// leave the recovery payload behind instead of reading `ota_management`.
static REBOOT_OTA_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Publishes that an OTA update is in progress for `light_sw_reboot_callback`.
pub fn set_reboot_ota_active(active: bool) {
    REBOOT_OTA_ACTIVE.store(active, Ordering::Relaxed);
}

/// Publishes the current on/off state for `light_sw_reboot_callback`.
pub fn set_reboot_light_off_state(is_off: bool) {
    REBOOT_LIGHT_OFF.store(is_off, Ordering::Relaxed);
}

#[cfg_attr(test, mry::mry)]
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
    load_tbl_cmd_set(&tbl_cpu_wakeup_init);
}

// recover status before software reboot
#[cfg_attr(test, mry::mry)]
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
#[cfg_attr(test, mry::mry)]
fn light_sw_reboot_ll() {
    // In the original code it calls this to reset, but I can't get this function working.
    // unsafe { cpu_sleep_wakeup(1, 0x40, ((CLOCK_SYS_CLOCK_1US) * 10000) + read_reg_system_tick()); }

    // Instead, let's just reset the MCU as described in the docs
    mcu_trigger_reboot();

    // Don't loop forever in tests - just return instead
    #[cfg(not(test))]
    {
        loop {}
    }
}

#[cfg_attr(test, mry::mry)]
pub fn light_sw_reboot() {
    irq_disable();

    // A pending panic keeps its sentinel so the next boot can still report it
    if analog_read(REGA_RESET_SENTINEL) != SENTINEL_PANIC {
        analog_write(REGA_RESET_SENTINEL, SENTINEL_CLEAN_REBOOT);
    }

    light_sw_reboot_callback();
    light_sw_reboot_ll();
    return;
}

/// Blinks the OTA LED six times at 1 Hz to signal success. `pin` is passed by
/// the caller so this module needs no `crate::config` dependency.
#[cfg_attr(test, mry::mry)]
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
    use crate::sdk::mcu::analog::{mock_analog_read, mock_analog_write};
    use crate::sdk::mcu::clock::mock_sleep_us;
    use crate::sdk::mcu::cmd_table::mock_load_tbl_cmd_set;
    use crate::sdk::mcu::gpio::mock_gpio_set_func;
    use crate::sdk::mcu::irq_i::mock_irq_disable;
    use crate::sdk::mcu::watchdog::mock_wd_clear;
    use mry::Any;
    use std::sync::{Arc, Mutex};
    use tlsr8266_pac::fake_mmio::{FakeMmio, RegWrite};
    use tlsr8266_pac::{
        mock_read_reg_gpio_oen, mock_read_reg_gpio_out, mock_write_reg_gpio_oen,
        mock_write_reg_gpio_out, mock_write_reg_pwdn_ctrl, read_reg_gpio_oen, read_reg_gpio_out,
        write_reg_gpio_oen, write_reg_gpio_out, REG_BASE_ADDR,
    };

    /// Tests usb_dp_pullup_en function with enable=true
    ///
    /// Verifies that when enabling USB DP pullup, BIT(4) is cleared
    /// in the analog register value.
    #[test]
    #[mry::lock(analog_read, analog_write)]
    fn test_usb_dp_pullup_en_enable() {
        // Setup: analog_read returns a value with BIT(4) set
        let initial_value = 0xFF;
        mock_analog_read(0x00).returns(initial_value);
        mock_analog_write(0x00, initial_value & !BIT!(4)).returns(());

        // Execute
        usb_dp_pullup_en(true);

        // Verify: analog_write was called with BIT(4) cleared
        mock_analog_write(0x00, Any).assert_called(1);
    }

    /// Tests usb_dp_pullup_en function with enable=false
    ///
    /// Verifies that when disabling USB DP pullup, BIT(4) is set
    /// in the analog register value.
    #[test]
    #[mry::lock(analog_read, analog_write)]
    fn test_usb_dp_pullup_en_disable() {
        // Setup: analog_read returns a value with BIT(4) cleared
        let initial_value = 0x00;
        mock_analog_read(0x00).returns(initial_value);
        mock_analog_write(0x00, initial_value | BIT!(4)).returns(());

        // Execute
        usb_dp_pullup_en(false);

        // Verify: analog_write was called with BIT(4) set
        mock_analog_write(0x00, Any).assert_called(1);
    }

    /// Tests cpu_wakeup_init function
    ///
    /// Verifies that cpu_wakeup_init calls load_tbl_cmd_set
    /// with the correct table.
    #[test]
    #[mry::lock(load_tbl_cmd_set)]
    fn test_cpu_wakeup_init() {
        // Since load_tbl_cmd_set takes a reference to a slice,
        // we just verify it's called once
        mock_load_tbl_cmd_set().returns(0);

        // Execute
        cpu_wakeup_init();

        // Verify: load_tbl_cmd_set was called exactly once
        mock_load_tbl_cmd_set().assert_called(1);
    }

    /// Tests light_sw_reboot_callback when no OTA update is active
    ///
    /// Verifies that when the OTA mirror is clear, no analog_write occurs.
    #[test]
    #[mry::lock(analog_write)]
    fn test_light_sw_reboot_callback_no_ota() {
        // Setup: OTA mirror clear
        set_reboot_ota_active(false);
        // Light state must not leak in from another test
        set_reboot_light_off_state(false);

        mock_analog_write(Any, Any).returns(());

        // Execute
        light_sw_reboot_callback();

        // Verify: the callback writes nothing when no OTA is in progress
        mock_analog_write(Any, Any).assert_called(0);
    }

    /// Tests light_sw_reboot_callback when OTA is active and light is off
    ///
    /// Verifies that when the OTA mirror is set and the published state says the
    /// light is off, analog_write is called with RecoverStatus::LightOff.
    #[test]
    #[mry::lock(analog_write)]
    fn test_light_sw_reboot_callback_ota_light_off() {
        // Setup: OTA mirror set, light is off
        set_reboot_ota_active(true);
        set_reboot_light_off_state(true);

        mock_analog_write(REGA_LIGHT_OFF, RecoverStatus::LightOff as u8).returns(());

        // Execute
        light_sw_reboot_callback();

        // Verify: analog_write called with correct parameters
        mock_analog_write(REGA_LIGHT_OFF, RecoverStatus::LightOff as u8).assert_called(1);

        // The mirror is process-global: leave it clear for later tests.
        set_reboot_ota_active(false);
    }

    /// Tests light_sw_reboot_callback when OTA is active and light is on
    ///
    /// Verifies that when the OTA mirror is set and the published state says the
    /// light is on, analog_write is called with 0.
    #[test]
    #[mry::lock(analog_write)]
    fn test_light_sw_reboot_callback_ota_light_on() {
        // Setup: OTA mirror set, light is on
        set_reboot_ota_active(true);
        set_reboot_light_off_state(false);

        mock_analog_write(REGA_LIGHT_OFF, 0).returns(());

        // Execute
        light_sw_reboot_callback();

        // Verify: analog_write called with correct parameters
        mock_analog_write(REGA_LIGHT_OFF, 0).assert_called(1);

        // The mirror is process-global: leave it clear for later tests.
        set_reboot_ota_active(false);
    }

    /// The mirror is the callback's only input: clearing it after a call stops
    /// the next one from writing, so the decision is made per call and not
    /// latched by an earlier OTA update.
    #[test]
    #[mry::lock(analog_write)]
    fn test_light_sw_reboot_callback_follows_mirror() {
        set_reboot_light_off_state(true);
        mock_analog_write(REGA_LIGHT_OFF, RecoverStatus::LightOff as u8).returns(());

        set_reboot_ota_active(true);
        light_sw_reboot_callback();
        mock_analog_write(REGA_LIGHT_OFF, RecoverStatus::LightOff as u8).assert_called(1);

        set_reboot_ota_active(false);
        light_sw_reboot_callback();
        mock_analog_write(Any, Any).assert_called(1);

        // Leave the mirror clear for any later test in this binary.
        set_reboot_ota_active(false);
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
        FakeMmio::reset();

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
        FakeMmio::reset();

        mcu_trigger_reboot();

        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: 0x6f | REG_BASE_ADDR,
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
        FakeMmio::reset();

        light_sw_reboot_ll();

        assert_eq!(
            FakeMmio::writes(),
            vec![RegWrite {
                addr: 0x6f | REG_BASE_ADDR,
                width: 1,
                value: FldPwdnCtrl::Reboot.bits() as u32,
            }]
        );
    }

    /// `blink_ota_led_ok` configures the pin as an output and genuinely
    /// alternates it high/low over six cycles, 1 s apart, with the watchdog
    /// cleared between writes.
    ///
    /// `gpio_write`/`gpio_set_output_en` carry no `mry` annotation, so the
    /// assertion is made one level down: the OUT register they drive. Each
    /// `gpio_write` reads the (mocked) zeroed OUT register, sets or clears the
    /// pin bit, and writes it back, so the recorded write sequence is the level
    /// sequence shifted into the pin's bit: `[0x02, 0x00, 0x02, 0x00, 0x02, 0x00]`.
    ///
    /// The old `led_onoff: u32` toggle produced `1, 0xFFFF_FFFE, 1, ...`, all of
    /// which `gpio_write` treats as "high", so the old sequence was six set
    /// writes and the LED never blinked.
    #[test]
    #[mry::lock(
        gpio_set_func,
        read_reg_gpio_oen,
        write_reg_gpio_oen,
        read_reg_gpio_out,
        write_reg_gpio_out,
        wd_clear,
        sleep_us
    )]
    fn test_blink_ota_led_ok_alternates_pin_six_times() {
        const PIN: u32 = 0x02; // port A, pin id low byte is the bit mask itself
        const PIN_MASK: u8 = 0x02;

        mock_gpio_set_func(Any, Any).returns(());
        mock_read_reg_gpio_oen(Any).returns(0);
        mock_write_reg_gpio_oen(Any, Any).returns(());
        mock_read_reg_gpio_out(Any).returns(0);

        // Record every OUT register write in call order.
        let writes: Arc<Mutex<Vec<u8>>> = Arc::new(Mutex::new(Vec::new()));
        let recorded_writes = Arc::clone(&writes);
        mock_write_reg_gpio_out(Any, Any).returns_with(move |value: u8, _offset: u32| {
            recorded_writes.lock().unwrap().push(value);
        });

        mock_wd_clear().returns(());
        mock_sleep_us(Any).returns(());

        blink_ota_led_ok(PIN);

        mock_gpio_set_func(PIN, AS_GPIO).assert_called(1);
        // Output enabled once, before the first write.
        mock_write_reg_gpio_oen(Any, Any).assert_called(1);
        // Six writes that alternate high, low, high, low, high, low.
        assert_eq!(
            *writes.lock().unwrap(),
            vec![PIN_MASK, 0, PIN_MASK, 0, PIN_MASK, 0],
            "the OTA LED must be driven high and low alternately, six times"
        );
        mock_write_reg_gpio_out(PIN_MASK, Any).assert_called(3);
        mock_write_reg_gpio_out(0, Any).assert_called(3);
        mock_write_reg_gpio_out(Any, Any).assert_called(6);
        mock_wd_clear().assert_called(6);
        mock_sleep_us(1000 * 1000).assert_called(6);
    }

    /// Tests light_sw_reboot function
    ///
    /// Verifies that light_sw_reboot calls irq_disable,
    /// light_sw_reboot_callback, and light_sw_reboot_ll in sequence.
    #[test]
    #[mry::lock(
        irq_disable,
        light_sw_reboot_callback,
        light_sw_reboot_ll,
        analog_read,
        analog_write
    )]
    fn test_light_sw_reboot() {
        mock_irq_disable().returns(0);
        mock_light_sw_reboot_callback().returns(());
        mock_light_sw_reboot_ll().returns(());
        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_RUNNING);
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_CLEAN_REBOOT).returns(());

        // Execute
        light_sw_reboot();

        // Verify: All functions called in correct order
        mock_irq_disable().assert_called(1);
        mock_light_sw_reboot_callback().assert_called(1);
        mock_light_sw_reboot_ll().assert_called(1);
    }

    /// A watchdog or user reboot overwrites the running sentinel with "clean",
    /// so the next boot does not report a phantom watchdog reset.
    #[test]
    #[mry::lock(
        irq_disable,
        light_sw_reboot_callback,
        light_sw_reboot_ll,
        analog_read,
        analog_write
    )]
    fn test_light_sw_reboot_sets_clean_sentinel() {
        mock_irq_disable().returns(0);
        mock_light_sw_reboot_callback().returns(());
        mock_light_sw_reboot_ll().returns(());
        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_RUNNING);
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_CLEAN_REBOOT).returns(());

        // Execute
        light_sw_reboot();

        // Verify: the running sentinel was cleared exactly once
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_CLEAN_REBOOT).assert_called(1);
    }

    /// A panic reboot keeps the panic sentinel intact so the next boot can
    /// stream the panic message left in flash.
    #[test]
    #[mry::lock(
        irq_disable,
        light_sw_reboot_callback,
        light_sw_reboot_ll,
        analog_read,
        analog_write
    )]
    fn test_light_sw_reboot_preserves_panic_sentinel() {
        mock_irq_disable().returns(0);
        mock_light_sw_reboot_callback().returns(());
        mock_light_sw_reboot_ll().returns(());
        mock_analog_read(REGA_RESET_SENTINEL).returns(SENTINEL_PANIC);
        mock_analog_write(Any, Any).returns(());

        // Execute
        light_sw_reboot();

        // Verify: no clean-reboot write, which would hide the panic
        mock_analog_write(REGA_RESET_SENTINEL, SENTINEL_CLEAN_REBOOT).assert_called(0);
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

    /// Tests TCMD constants
    ///
    /// Verifies that TCMD constants have the correct values.
    #[test]
    fn test_tcmd_constants() {
        assert_eq!(TCMD_UNDER_RD, 0x80);
        assert_eq!(TCMD_UNDER_WR, 0x40);
        assert_eq!(TCMD_UNDER_BOTH, 0xc0);
        assert_eq!(TCMD_MASK, 0x3f);
        assert_eq!(TCMD_WRITE, 0x3);
        assert_eq!(TCMD_WAIT, 0x7);
        assert_eq!(TCMD_WAREG, 0x8);
    }
}
