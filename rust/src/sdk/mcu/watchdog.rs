use sdk::mcu::register::{read_reg_tmr_ctrl, write_reg_tmr_ctrl, write_reg_tmr2_tick, FLD_TMR};
use sdk::mcu::clock::CLOCK_SYS_CLOCK_1MS;
use sdk::mcu::register::WATCHDOG_TIMEOUT_COEFF;
use {SET_FLD_V, BM_SET_MASK_FLD, BM_MASK_VAL, MASK_VAL, BIT_LOW_BIT, BIT, BM_SET, SET_FLD, BM_CLR};
use CLR_FLD;

//  watchdog use timer 2
pub fn wd_set_interval(ms: u32)	//  in ms
{
	assert!((ms * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF) > 0);
    let mut val = read_reg_tmr_ctrl();
	SET_FLD_V!(val, FLD_TMR::TMR_WD_CAPT as u32, (ms * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF), FLD_TMR::TMR2_MODE as u32, 0);
    write_reg_tmr_ctrl(val);
	write_reg_tmr2_tick(0);
}

#[inline(always)]
pub fn wd_start()
{
    let mut val = read_reg_tmr_ctrl();
	SET_FLD!(val, FLD_TMR::TMR_WD_EN as u32);
    write_reg_tmr_ctrl(val);
}

#[inline(always)]
pub fn wd_stop()
{
    let mut val = read_reg_tmr_ctrl();
	CLR_FLD!(val, FLD_TMR::TMR_WD_EN as u32);
    write_reg_tmr_ctrl(val);
}

#[inline(always)]
pub fn wd_clear()
{
    let mut val = read_reg_tmr_ctrl();
    SET_FLD!(val, FLD_TMR::CLR_WD as u32);
    write_reg_tmr_ctrl(val);
}