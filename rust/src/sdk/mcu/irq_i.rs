use crate::sdk::mcu::register::{read_reg_irq_en, write_reg_irq_en, write_reg_irq_mask, FLD_IRQ};

pub const IRQ_TIMER1_ENABLE: bool = true;
pub const IRQ_GPIO_ENABLE: bool = false;
pub const IRQ_RF_RTX_ENABLE: bool = true;

pub const IRQ_INIT_VALUE: u32 = ((if IRQ_TIMER1_ENABLE {
    FLD_IRQ::TMR1_EN.bits()
} else {
    0
}) | (if IRQ_GPIO_ENABLE {
    FLD_IRQ::GPIO_RISC0_EN.bits()
} else {
    0
}) | (if IRQ_RF_RTX_ENABLE {
    FLD_IRQ::ZB_RT_EN.bits()
} else {
    0
}));

pub fn irq_enable() -> u8 {
    let r = read_reg_irq_en(); // don't worry,  the compiler will optimize the return value if not used
    write_reg_irq_en(1);
    return r;
}

pub fn irq_disable() -> u8 {
    let r = read_reg_irq_en(); // don't worry,  the compiler will optimize the return value if not used
    write_reg_irq_en(0);
    return r;
}

pub fn irq_restore(en: u8) {
    write_reg_irq_en(en);
}

pub fn irq_init() {
    write_reg_irq_mask(IRQ_INIT_VALUE);
}
