use crate::sdk::mcu::register::{
    read_reg_master_spi_ctrl, read_reg_master_spi_data, write_reg_master_spi_ctrl,
    write_reg_master_spi_data, FLD_MASTER_SPI,
};

#[inline(always)]
pub fn mspi_high() {
    write_reg_master_spi_ctrl(FLD_MASTER_SPI::CS.bits());
}

#[inline(always)]
pub fn mspi_low() {
    write_reg_master_spi_ctrl(0);
}

#[inline(always)]
pub fn mspi_write(c: u8) {
    write_reg_master_spi_data(c);
}

#[inline(always)]
pub fn mspi_wait() {
    while read_reg_master_spi_ctrl() & (FLD_MASTER_SPI::BUSY.bits()) != 0 {}
}

#[inline(always)]
pub fn mspi_ctrl_write(c: u8) {
    write_reg_master_spi_ctrl(c);
}

#[inline(always)]
pub fn mspi_get() -> u8 {
    read_reg_master_spi_data()
}

#[inline(always)]
pub fn mspi_read() -> u8 {
    mspi_write(0); // dummy, issue clock
    mspi_wait();
    return mspi_get();
}
