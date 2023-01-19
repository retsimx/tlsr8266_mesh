use crate::sdk::mcu::register::{
    read_reg_pwm_enable, write_reg_pwm_cmp, write_reg_pwm_cycle, write_reg_pwm_enable, FLD_PWM,
};
use crate::{BIT, BIT_LOW_BIT, MASK_VAL};

pub fn pwm_set_cmp(id: u32, cmp: u16) {
    write_reg_pwm_cmp(cmp, id << 2)
}

pub fn pwm_set_duty(id: u32, max_tick: u16, cmp_tick: u16) {
    write_reg_pwm_cycle(
        MASK_VAL!(
            FLD_PWM::CMP as u32,
            cmp_tick as u32,
            FLD_PWM::MAX as u32,
            max_tick as u32
        ),
        id << 2,
    );
}

pub fn pwm_start(id: u32) {
    write_reg_pwm_enable(read_reg_pwm_enable() | BIT!(id));
}
