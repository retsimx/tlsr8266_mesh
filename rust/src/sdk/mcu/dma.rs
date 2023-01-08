use sdk::mcu::register::{write_reg_dma_chn_irq_msk};

pub fn dma_init() {
    write_reg_dma_chn_irq_msk(0)
}