use crate::sdk::mcu::clock::CLOCK_SYS_CLOCK_1MS;
use crate::sdk::mcu::register::WATCHDOG_TIMEOUT_COEFF;
use crate::sdk::mcu::register::{read_reg_tmr_ctrl, write_reg_tmr2_tick, write_reg_tmr_ctrl, FLD_TMR};
use crate::CLR_FLD;
use crate::{
    BIT, BIT_LOW_BIT, BM_CLR, BM_MASK_VAL, BM_SET, BM_SET_MASK_FLD, MASK_VAL, SET_FLD, SET_FLD_V,
};

//  watchdog use timer 2
pub fn wd_set_interval(ms: u32) //  in ms
{
    assert!((ms * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF) > 0);
    let mut val = read_reg_tmr_ctrl();
    SET_FLD_V!(
        val,
        FLD_TMR::TMR_WD_CAPT as u32,
        (ms * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF),
        FLD_TMR::TMR2_MODE as u32,
        0
    );
    write_reg_tmr_ctrl(val);
    write_reg_tmr2_tick(0);
}

#[inline(always)]
pub fn wd_start() {
    let mut val = read_reg_tmr_ctrl();
    SET_FLD!(val, FLD_TMR::TMR_WD_EN as u32);
    write_reg_tmr_ctrl(val);
}

#[inline(always)]
pub fn wd_stop() {
    let mut val = read_reg_tmr_ctrl();
    CLR_FLD!(val, FLD_TMR::TMR_WD_EN as u32);
    write_reg_tmr_ctrl(val);
}

#[inline(always)]
pub fn wd_clear() {
    let mut val = read_reg_tmr_ctrl();
    SET_FLD!(val, FLD_TMR::CLR_WD as u32);
    write_reg_tmr_ctrl(val);
}
