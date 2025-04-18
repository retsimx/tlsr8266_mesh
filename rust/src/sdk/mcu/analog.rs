use critical_section;
use crate::sdk::mcu::register::{
    read_reg_ana_ctrl, read_reg_ana_data, write_reg_ana_addr, write_reg_ana_ctrl,
    write_reg_ana_data, FLD_ANA,
};

#[inline(always)]
#[cfg_attr(test, mry::mry)]
pub fn analog_wait() {
    while read_reg_ana_ctrl() & (FLD_ANA::BUSY.bits()) != 0 {}
}

#[inline(never)]
#[link_section = ".ram_code"]
#[cfg_attr(test, mry::mry)]
pub fn analog_read(addr: u8) -> u8 {
    critical_section::with(|_| {
        write_reg_ana_addr(addr);
        write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RSV).bits());
        //   Can't use one line setting "reg_ana_ctrl32 = ((FLD_ANA_START | FLD_ANA_RSV) << 16) | addr;"
        //   This will fail because of time sequence and more over size is bigger
        analog_wait();
        let data = read_reg_ana_data();
        write_reg_ana_ctrl(0); // finish

        data
    })
}

#[inline(never)]
#[link_section = ".ram_code"]
#[cfg_attr(test, mry::mry)]
pub fn analog_write(addr: u8, v: u8) {
    critical_section::with(|_| {
        write_reg_ana_addr(addr);
        write_reg_ana_data(v);
        write_reg_ana_ctrl((FLD_ANA::START | FLD_ANA::RW).bits());

        //	 Can't use one line setting "reg_ana_ctrl32 = ((FLD_ANA_START | FLD_ANA_RW) << 16) | (v << 8) | addr;"
        //   This will fail because of time sequence and more over size is bigger
        analog_wait();
        write_reg_ana_ctrl(0); // finish
    });
}
