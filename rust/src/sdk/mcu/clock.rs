use crate::sdk::mcu::register::{
    read_reg_system_tick, write_reg_clk_sel, write_reg_rst_clk0, write_reg_tmr_ctrl, FLD_CLK_EN,
    FLD_CLK_SEL, FLD_TMR, WATCHDOG_TIMEOUT_COEFF,
};
use crate::{BIT, BIT_LOW_BIT, MASK_VAL};

pub const CLOCK_SYS_CLOCK_HZ: u32 = 32000000;

pub const CLK_SBC_ENABLE: u8 = 1;
pub const CLK_AUD_ENABLE: u8 = 1;
pub const CLK_DFIFO_ENABLE: u8 = 1;
pub const CLK_USB_ENABLE: bool = true;
pub const WATCHDOG_INIT_TIMEOUT: u32 = 2000;

pub const CLOCK_PLL_CLOCK: u32 = 192000000;

pub const CLOCK_SYS_CLOCK_1S: u32 = CLOCK_SYS_CLOCK_HZ;
pub const CLOCK_SYS_CLOCK_1MS: u32 = (CLOCK_SYS_CLOCK_1S / 1000);
pub const CLOCK_SYS_CLOCK_1US: u32 = (CLOCK_SYS_CLOCK_1S / 1000000);
pub const CLOCK_SYS_CLOCK_4S: u32 = CLOCK_SYS_CLOCK_1S << 2;
pub const CLOCK_MAX_MS: u32 = (u32::MAX / CLOCK_SYS_CLOCK_1MS);
pub const CLOCK_MAX_US: u32 = (u32::MAX / CLOCK_SYS_CLOCK_1US);

enum CLOCK_SEL {
    SEL_32M_RC = 0,
    SEL_HS_DIV = 1,
    SEL_16M_PAD = 2,
    SEL_32M_PAD = 3,
    SEL_SPI = 4,
    SEL_40M_INTERNAL = 5,
    SEL_32K_RC = 6,
}

pub fn clock_init() {
    write_reg_rst_clk0(
        0xff000000
            | (if CLK_USB_ENABLE {
                FLD_CLK_EN::USB_EN.bits()
            } else {
                0
            }),
    );

    write_reg_clk_sel(MASK_VAL!(
        FLD_CLK_SEL::DIV.bits() as u32,
        (CLOCK_PLL_CLOCK / CLOCK_SYS_CLOCK_1S) as u32,
        FLD_CLK_SEL::SRC.bits() as u32,
        CLOCK_SEL::SEL_HS_DIV as u32
    ) as u8);

    //reg_clk_en = 0xff | CLK_EN_TYPE;
    write_reg_tmr_ctrl(MASK_VAL!(
        FLD_TMR::TMR0_EN.bits(),
        1,
        FLD_TMR::TMR_WD_CAPT.bits(),
        WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF,
        FLD_TMR::TMR_WD_EN.bits(),
        1
    ))
}

#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn clock_time() -> u32 {
    read_reg_system_tick()
}

#[inline(always)]
pub fn clock_time_exceed(reference: u32, span_us: u32) -> bool {
    return (clock_time() - reference) as u32 > span_us * CLOCK_SYS_CLOCK_1US;
}

#[inline(never)]
#[cfg_attr(test, mry::mry)]
#[link_section = ".ram_code"]
pub fn sleep_us(us: u32) {
    let t = clock_time();
    while !clock_time_exceed(t, us) {}
}
