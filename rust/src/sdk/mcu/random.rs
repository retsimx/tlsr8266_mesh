use crate::sdk::mcu::clock::clock_time;
use crate::sdk::mcu::register::read_reg_rnd_number;

#[inline(always)]
pub fn rand() -> u16 {
    return clock_time() as u16 ^ read_reg_rnd_number();
}