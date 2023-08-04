extern crate core;
extern crate paste;

use core::ptr::addr_of;

use crate::{BIT, BIT_MASK_LEN, BIT_RNG};

pub const REG_BASE_ADDR: u32 = 0x800000;

#[cfg_attr(test, mry::mry)]
pub fn write_reg8(addr: u32, v: u8) {
    unsafe { core::ptr::write_volatile((addr | REG_BASE_ADDR) as *mut u8, v) }
}

#[cfg_attr(test, mry::mry)]
pub fn write_reg16(addr: u32, v: u16) {
    unsafe { core::ptr::write_volatile((addr | REG_BASE_ADDR) as *mut u16, v) }
}

#[cfg_attr(test, mry::mry)]
pub fn write_reg32(addr: u32, v: u32) {
    unsafe { core::ptr::write_volatile((addr | REG_BASE_ADDR) as *mut u32, v) }
}

#[cfg_attr(test, mry::mry)]
pub fn read_reg8(addr: u32) -> u8 {
    return unsafe { core::ptr::read_volatile((addr | REG_BASE_ADDR) as *mut u8) };
}

#[cfg_attr(test, mry::mry)]
pub fn read_reg16(addr: u32) -> u16 {
    unsafe { core::ptr::read_volatile((addr | REG_BASE_ADDR) as *mut u16) }
}

#[cfg_attr(test, mry::mry)]
pub fn read_reg32(addr: u32) -> u32 {
    unsafe { core::ptr::read_volatile((addr | REG_BASE_ADDR) as *mut u32) }
}

#[macro_export]
macro_rules! regrw_idx {
    ( $x:ident, $a:expr, $s:literal ) => {
        paste::paste! {
            pub fn [<read_ $x>](i: u32) -> [<u $s>] {
                return [<read_reg $s>]($a + i);
            }

            pub fn [<write_ $x>](value: [<u $s>], i: u32) {
                [<write_reg $s>]($a + i, value);
            }
        }
    };
}

#[macro_export]
macro_rules! regrw {
    ( $x:ident, $a:expr, $s:expr ) => {
        paste::paste! {
            pub fn [<read_ $x>]() -> [<u $s>] {
                return [<read_reg $s>]($a);
            }

            pub fn [<write_ $x>](value: [<u $s>]) {
                [<write_reg $s>]($a, value);
            }
        }
    };
}

#[macro_export]
macro_rules! regrw_copy {
    ( $x:ident, $y:ident, $t:expr ) => {
        paste::paste! {
            pub fn [<read_ $x>]() -> [<u $t>] {
                [<read_ $y>]()
            }

            pub fn [<write_ $x>](value: [<u $t>]) {
                [<write_ $y>](value)
            }
        }
    };
}

/****************************************************
master spi regs struct: begin  addr : 0x0c
*****************************************************/
regrw!(reg_master_spi_data, 0x0c, 8);
regrw!(reg_master_spi_ctrl, 0x0d, 8);

pub enum FLD_MASTER_SPI {
    CS = BIT!(0),
    SDO = BIT!(1),
    CONT = BIT!(2),
    RD = BIT!(3),
    BUSY = BIT!(4),
}

/****************************************************
sys regs struct: begin  addr : 0x60
*****************************************************/
regrw!(reg_rst0, 0x60, 8);
regrw!(reg_rst0_16, 0x60, 16);
regrw!(reg_rst1, 0x61, 8);
regrw!(reg_rst2, 0x62, 8);
regrw!(reg_rst_clk0, 0x60, 32);
pub enum FLD_RST {
    SPI = BIT!(0),
    I2C = BIT!(1),
    USB = BIT!(2),
    USB_PHY = BIT!(3),
    MCU = BIT!(4),
    MAC = BIT!(5),
    AIF = BIT!(6),
    BB = BIT!(7),
    GPIO = BIT!(8),
    ALGM = BIT!(9),
    DMA = BIT!(10),
    UART = BIT!(11),
    PWM = BIT!(12),
    AES = BIT!(13),
    SWR_M = BIT!(14),
    SWR_S = BIT!(15),
    SBC = BIT!(16),
    AUD = BIT!(17),
    DFIFO = BIT!(18),
    ADC = BIT!(19),
    SOFT_MCU = BIT!(20),
    MCIC = BIT!(21),
    SOFT_MCIC = BIT!(22),
    RSV = BIT!(23),
}

impl FLD_RST {
    pub const ZB: FLD_RST = FLD_RST::BB;
}

#[repr(usize)]
pub enum FLD_CLK_EN {
    GPIO_EN = BIT!(0),
    ALGM_EN = BIT!(1),
    DMA_EN = BIT!(2),
    UART_EN = BIT!(3),
    PWM_EN = BIT!(4),
    AES_EN = BIT!(5),
    PLL_EN = BIT!(6),
    SWIRE_EN = BIT!(7),
    SBC_EN = BIT!(8),
    AUD_EN = BIT!(9),
    DIFIO_EN = BIT!(10),
    I2S = BIT_RNG!(11, 12),
    C32K = BIT_RNG!(13, 15),
    SPI_EN = BIT!(24),
    I2C_EN = BIT!(25),
    USB_EN = BIT!(26),
    USB_PHY_EN = BIT!(27),
    MCU_EN = BIT!(28),
    MAC_EN = BIT!(29),
    ADC_EN = BIT!(30), // ADC interface
    ZB_EN = BIT!(31),
}

regrw!(reg_clk_en, 0x64, 16);
regrw!(reg_clk_en1, 0x64, 8);

regrw!(reg_clk_en2, 0x65, 8);

pub enum FLD_CLK2_EN {
    SBC_EN = BIT!(0),
    AUD_EN = BIT!(1),
    DIFIO_EN = BIT!(2),
    I2S = BIT_RNG!(3, 4),
    C32K = BIT_RNG!(5, 7),
}

regrw!(reg_clk_sel, 0x66, 8);

pub enum FLD_CLK_SEL {
    DIV = BIT_RNG!(0, 4),
    SRC = BIT_RNG!(5, 7),
}

regrw!(reg_i2s_step, 0x67, 8);
pub enum FLD_I2S_STEP {
    STEP = BIT_RNG!(0, 6),
    CLK_EN = BIT!(7),
}

regrw!(reg_i2s_mod, 0x68, 8);

// pub fn SET_SDM_CLOCK_MHZ(f_mhz: 32)	{
//     write_reg_i2s_step(FLD_I2S::CLK_EN as 32 | f_mhz);
//     write_reg_i2s_mod(0xc0);
// }

/****************************************************
    ADC: 0x69
*****************************************************/
regrw!(reg_adc_step_l, 0x69, 8);
regrw!(reg_adc_mod_l, 0x6a, 8);
regrw!(reg_adc_mod, 0x6a, 16);
pub enum FLD_ADC_MOD {
    MOD = BIT_RNG!(0, 11),
    STEP_H = BIT_RNG!(12, 14),
    CLK_EN = BIT!(15),
}

regrw!(reg_adc_clk_en, 0x6b, 8);
regrw!(reg_adc_mod_h, 0x6b, 8);
pub enum FLD_ADC_MOD_H {
    H = BIT_RNG!(0, 3),
    H_STEP = BIT_RNG!(4, 6),
    H_CLK = BIT!(7),
}

regrw!(reg_dmic_step, 0x6c, 8);
pub enum FLD_DMIC_STEP {
    STEP = BIT_RNG!(0, 6),
    CLK_EN = BIT!(7),
}
regrw!(reg_dmic_mod, 0x6d, 8);

regrw!(reg_wakeup_en, 0x6e, 8);
pub enum FLD_WAKEUP_SRC {
    I2C = BIT!(0),
    SPI = BIT!(1),
    USB = BIT!(2),
    GPIO = BIT!(3),
    I2C_SYN = BIT!(4),
    GPIO_RM = BIT!(5),
    USB_RESM = BIT!(6),
    RST_SYS = BIT!(7),
}

regrw!(reg_pwdn_ctrl, 0x6f, 8);
pub enum FldPwdnCtrl {
    Reboot = BIT!(5),
    Sleep = BIT!(7),
}

regrw!(reg_fhs_sel, 0x70, 8);
pub enum FLD_FHS_SELECT {
    SELECT = BIT_RNG!(0, 1),
}
pub enum FHS_SEL {
    SEL_192M_PLL = 0,
    //	SEL_48M_PLL = 1,
    SEL_32M_OSC = 1,
    //	SEL_16M_OSC = 3,
}

regrw!(reg_mcu_wakeup_mask, 0x78, 32);

//////  analog controls 0xb8 ///////
regrw!(reg_ana_ctrl32, 0xb8, 32);
regrw!(reg_ana_addr_data, 0xb8, 16);
regrw!(reg_ana_addr, 0xb8, 8);
regrw!(reg_ana_data, 0xb9, 8);
regrw!(reg_ana_ctrl, 0xba, 8);

pub enum FLD_ANA {
    BUSY = BIT!(0),
    RSV = BIT!(4),
    RW = BIT!(5),
    START = BIT!(6),
    CYC = BIT!(7),
}

/****************************************************
   RF : begin  addr : 0x4e8
*****************************************************/
regrw!(reg_rf_tx_mode1, 0x400, 8);
regrw!(reg_rf_tx_mode, 0x400, 16);
pub enum FLD_RF_TX_MODE {
    DMA_EN = BIT!(0),
    CRC_EN = BIT!(1),
    BANDWIDTH = BIT_RNG!(2, 3),
    OUTPUT = BIT!(4),
    TST_OUT = BIT!(5),
    TST_EN = BIT!(6),
    TST_MODE = BIT!(7),
    ZB_PN_EN = BIT!(8),
    ZB_FEC_EN = BIT!(9),
    ZB_INTL_EN = BIT!(10), // interleaving
    TX_1M2M_PN_EN = BIT!(11),
    TX_1M2M_FEC_EN = BIT!(12),
    TX_1M2M_INTL_EN = BIT!(13), // interleaving
}
regrw!(reg_rf_access_code, 0x408, 32);
regrw!(reg_rf_tx_buf_sta, 0x41c, 32);

regrw!(reg_rf_rx_sense_thr, 0x422, 8);
regrw!(reg_rf_rx_auto, 0x426, 8);
pub enum FLD_RF_RX_AUTO {
    IRR_GAIN = BIT!(0),
    RX_IRR_PHASE = BIT!(1),
    RX_DAC_I = BIT!(2),
    RX_DAC_Q = BIT!(3),
    RX_LNA_GAIN = BIT!(4),
    RX_MIX2_GAIN = BIT!(5),
    RX_PGA_GAIN = BIT!(6),
    RX_CAL_EN = BIT!(7),
}

regrw!(reg_rf_rx_sync, 0x427, 8);
pub enum FLD_RF_SYNC {
    FREQ_COMP_EN = BIT!(0),
    ADC_SYNC = BIT!(1),
    ADC_INP_SIGNED = BIT!(2),
    SWAP_ADC_IQ = BIT!(3),
    NOTCH_FREQ_SEL = BIT!(4),
    NOTCH_BAND_SEL = BIT!(5),
    NOTCH_EN = BIT!(6),
    DN_CONV_FREQ_SEL = BIT!(7),
}

regrw!(reg_rf_rx_mode, 0x428, 8);
pub enum FLD_RF_RX_MODE {
    EN = BIT!(0),
    MODE_1M = BIT!(1),
    MODE_2M = BIT!(2),
    LOW_IF = BIT!(3),
    BYPASS_DCOC = BIT!(4),
    MAN_FINE_TUNE = BIT!(5),
    SINGLE_CAL = BIT!(6),
    LOW_PASS_FILTER = BIT!(7),
}

regrw!(reg_rf_rx_pilot, 0x42b, 8);
pub enum FLD_RF_PILOT {
    LEN = BIT_RNG!(0, 3),
    RF_ZB_SFD_CHK = BIT!(4),
    RF_1M_SFD_CHK = BIT!(5),
    RF_2M_SFD_CHK = BIT!(6),
    RF_ZB_OR_AUTO = BIT!(7),
}

regrw!(reg_rf_rx_chn_dc, 0x42c, 32);
regrw!(reg_rf_rx_q_chn_cal, 0x42f, 8);
pub enum FLD_RF_RX_DCQ_CAL {
    FLD_RF_RX_DCQ_HIGH = BIT_RNG!(0, 6),
    FLD_RF_RX_DCQ_CAL_START = BIT!(7),
}
regrw!(reg_rf_rx_pel, 0x434, 16);
regrw!(reg_rf_rx_pel_gain, 0x434, 32);
regrw!(reg_rf_rx_rssi_offset, 0x439, 8);

regrw!(reg_rf_rx_hdx, 0x43b, 8);
pub enum FLD_RF_RX_HDX {
    RX_HEADER_LEN = BIT_RNG!(0, 3),
    RT_TICK_LO_SEL = BIT!(4),
    RT_TICK_HI_SEL = BIT!(5),
    RT_TICK_FRAME = BIT!(6),
    PKT_LEN_OUTP_EN = BIT!(7),
}

regrw!(reg_rf_rx_gctl, 0x43c, 8);
pub enum FLD_RF_RX_GCTL {
    CIC_SAT_LO_EN = BIT!(0),
    CIC_SAT_HI_EN = BIT!(1),
    AUTO_PWR = BIT!(2),
    ADC_RST_VAL = BIT!(4),
    ADC_RST_EN = BIT!(5),
    PWR_CHG_DET_S = BIT!(6),
    PWR_CHG_DET_N = BIT!(7),
}
regrw!(reg_rf_rx_peak, 0x43d, 8);
pub enum FLD_RF_RX_PEAK {
    FLD_RX_PEAK_DET_SRC_EN = BIT_RNG!(0, 2),
    FLD_TX_PEAK_DET_EN = BIT!(3),
    FLD_PEAK_DET_NUM = BIT_RNG!(4, 5),
    FLD_PEAK_MAX_CNT_PRD = BIT_RNG!(6, 7),
}

regrw!(reg_rf_rx_status, 0x443, 8);
pub enum FLD_RF_RX_STATUS {
    RX_STATE = BIT_RNG!(0, 3),
    RX_STA_RSV = BIT_RNG!(4, 5),
    RX_INTR = BIT!(6),
    TX_INTR = BIT!(7),
}

regrw!(reg_rf_crc, 0x44c, 32);

regrw!(reg_rf_irq_mask, 0xf1c, 16);
regrw!(reg_rf_irq_status, 0xf20, 16);

pub enum FLD_RF_IRQ_MASK {
    IRQ_RX = BIT!(0),
    IRQ_TX = BIT!(1),
    IRX_RX_TIMEOUT = BIT!(2),
    IRX_CMD_DONE = BIT!(5),
    IRX_RETRY_HIT = BIT!(7),
}

// The value for FLD_RF_RX_STATE
pub enum FLD_RF_RX_STATE {
    IDLE = 0,
    SET_GAIN = 1,
    CIC_SETTLE = 2,
    LPF_SETTLE = 3,
    PE = 4,
    SYN_START = 5,
    GLOB_SYN = 6,
    GLOB_LOCK = 7,
    LOCAL_SYN = 8,
    LOCAL_LOCK = 9,
    ALIGN = 10,
    ADJUST = 11,
    DEMOD = 12, // de modulation
    FOOTER = 13,
}

regrw!(reg_rx_rnd_mode, 0x447, 8);
pub enum FLD_RX_RND_MODE {
    SRC = BIT!(0),
    MANU_MODE = BIT!(1),
    AUTO_RD = BIT!(2),
    FREE_MODE = BIT!(3),
    CLK_DIV = BIT_RNG!(4, 7),
}
regrw!(reg_rnd_number, 0x448, 16);

regrw!(reg_rf_rtt, 0x454, 32);
pub enum FLD_RF_RTT {
    CAL = BIT_RNG!(0, 7),
    CYC1 = BIT_RNG!(8, 15),
    LOCK = BIT_RNG!(16, 23),
    SD_DLY_40M = BIT_RNG!(24, 27),
    SD_DLY_BYPASS = BIT!(28),
}

regrw!(reg_rf_chn_rssi, 0x458, 8);

regrw_idx!(reg_rf_rx_gain_agc, 0x480, 32);

regrw!(reg_rf_rx_dci, 0x4cb, 8); //  different from the document, why
regrw!(reg_rf_rx_dcq, 0x4cf, 8); //  different from the document, why

regrw!(reg_pll_rx_coarse_tune, 0x4d0, 16);
regrw!(reg_pll_rx_coarse_div, 0x4d2, 8);
regrw!(reg_pll_rx_fine_tune, 0x4d4, 16);
regrw!(reg_pll_rx_fine_div, 0x4d6, 8);
regrw!(reg_pll_tx_coarse_tune, 0x4d8, 16);
regrw!(reg_pll_tx_coarse_div, 0x4da, 8);
regrw!(reg_pll_tx_fine_tune, 0x4dc, 16);
regrw!(reg_pll_tx_fine_div, 0x4de, 8);

regrw!(reg_pll_rx_frac, 0x4e0, 32);
regrw!(reg_pll_tx_frac, 0x4e4, 32);

regrw!(reg_pll_tx_ctrl, 0x4e8, 8);
regrw!(reg_pll_ctrl16, 0x4e8, 16);
regrw!(reg_pll_ctrl, 0x4e8, 32);
pub enum FLD_PLL_CTRL {
    TX_CYC0 = BIT!(0),
    TX_SOF = BIT!(1),
    TX_CYC1 = BIT!(2),
    TX_PRE_EN = BIT!(3),
    TX_VCO_EN = BIT!(4),
    TX_PWDN_DIV = BIT!(5),
    TX_MOD_EN = BIT!(6),
    TX_MOD_TRAN_EN = BIT!(7),
    RX_CYC0 = BIT!(8),
    RX_SOF = BIT!(9),
    RX_CYC1 = BIT!(10),
    RX_PRES_EN = BIT!(11),
    RX_VCO_EN = BIT!(12),
    RX_PWDN_DIV = BIT!(13),
    RX_PEAK_EN = BIT!(14),
    RX_TP_CYC = BIT!(15),
    SD_RSTB = BIT!(16),
    SD_INTG_EN = BIT!(17),
    CP_TRI = BIT!(18),
    PWDN_INTG1 = BIT!(19),
    PWDN_INTG2 = BIT!(20),
    PWDN_INTG_DIV = BIT!(21),
    PEAK_DET_EN = BIT!(22),
    OPEN_LOOP_EN = BIT!(23),
    RX_TICK_EN = BIT!(24),
    TX_TICK_EN = BIT!(25),
    RX_ALWAYS_ON = BIT!(26),
    TX_ALWAYS_ON = BIT!(27),
    MANUAL_MODE_EN = BIT!(28),
    CAL_DONE_EN = BIT!(29),
    LOCK_EN = BIT!(30),
}
regrw!(reg_pll_rx_ctrl, 0x4e9, 8);
pub enum FLD_PLL_RX_CTRL {
    CYC0 = BIT!(0),
    SOF = BIT!(1),
    CYC1 = BIT!(2),
    PRES_EN = BIT!(3),
    VCO_EN = BIT!(4),
    PD_DIV = BIT!(5),
    PEAK_EN = BIT!(6),
    TP_CYC = BIT!(7),
}

regrw!(reg_pll_ctrl_a, 0x4eb, 8);
pub enum FLD_PLL_CTRL_A {
    RX_TICK_EN = BIT!(0),
    TX_TICK_EN = BIT!(1),
    RX_ALWAYS_ON = BIT!(2),
    TX_ALWAYS_ON = BIT!(3),
    MANUAL_MODE_EN = BIT!(4),
    CAL_DONE_EN = BIT!(5),
    LOCK_EN = BIT!(6),
}
// pll polarity
regrw!(reg_pll_pol_ctrl, 0x4ec, 16);
pub enum FLD_PLL_POL_CTRL {
    TX_PRE_EN = BIT!(0),
    TX_VCO_EN = BIT!(1),
    TX_PD_DIV = BIT!(2),
    MOD_EN = BIT!(3),
    MOD_TRAN_EN = BIT!(4),
    RX_PRE_EN = BIT!(5),
    RX_VCO_EN = BIT!(6),
    RX_PD_DIV = BIT!(7),
    SD_RSTB = BIT!(8),
    SD_INTG_EN = BIT!(9),
    CP_TRI = BIT!(10),
    TX_SOF = BIT!(11),
    RX_SOF = BIT!(12),
}

regrw!(reg_rf_rx_cap, 0x4f0, 16); //  ����
regrw!(reg_rf_tx_cap, 0x4f0, 16); //  ����

/****************************************************
dma mac regs struct: begin  addr : 0x500
*****************************************************/
regrw!(reg_dma0_addr, 0x500, 16);
regrw!(reg_dma0_ctrl, 0x502, 16);
regrw!(reg_dma1_addr, 0x504, 16);
regrw!(reg_dma1_ctrl, 0x506, 16);
regrw!(reg_dma2_addr, 0x508, 16);
regrw!(reg_dma2_ctrl, 0x50a, 16);
regrw!(reg_dma3_addr, 0x50c, 16);
regrw!(reg_dma3_ctrl, 0x50e, 16);
regrw!(reg_dma4_addr, 0x510, 16);
regrw!(reg_dma4_ctrl, 0x512, 16);
regrw!(reg_dma5_addr, 0x514, 16);
regrw!(reg_dma5_ctrl, 0x516, 16);
regrw!(reg_dma_tx_rptr, 0x52a, 8);
regrw!(reg_dma_tx_wptr, 0x52b, 8);
regrw!(reg_dma_tx_fifo, 0x52c, 16);

pub enum FLD_DMA {
    BUF_SIZE = BIT_RNG!(0, 7),
    WR_MEM = BIT!(8),
    PINGPONG_EN = BIT!(9),
    FIFO_EN = BIT!(10),
    AUTO_MODE = BIT!(11),
    BYTE_MODE = BIT!(12),

    RPTR_CLR = BIT!(4),
    RPTR_NEXT = BIT!(5),
    RPTR_SET = BIT!(6),
}

impl FLD_DMA {
    pub const ETH_RX: u32 = BIT!(0);
    pub const ETH_TX: u32 = BIT!(1);
    pub const RF_RX: u32 = BIT!(2);
    pub const RF_TX: u32 = BIT!(3);
}

regrw!(reg_dma_chn_en, 0x520, 8);
regrw!(reg_dma_chn_irq_msk, 0x521, 8);
regrw!(reg_dma_tx_rdy0, 0x524, 8);
regrw!(reg_dma_tx_rdy1, 0x525, 8);
regrw!(reg_dma_rx_rdy0, 0x526, 8);
regrw_copy!(reg_dma_irq_src, reg_dma_rx_rdy0, 8);
regrw!(reg_dma_rx_rdy1, 0x527, 8);

//  The default channel assignment
regrw_copy!(reg_dma_eth_rx_addr, reg_dma0_addr, 16);
regrw_copy!(reg_dma_eth_rx_ctrl, reg_dma0_ctrl, 16);
regrw_copy!(reg_dma_eth_tx_addr, reg_dma1_addr, 16);

regrw_copy!(reg_dma_rf_rx_addr, reg_dma2_addr, 16);
regrw_copy!(reg_dma_rf_rx_ctrl, reg_dma2_ctrl, 16);
regrw_copy!(reg_dma_rf_tx_addr, reg_dma3_addr, 16);
regrw_copy!(reg_dma_rf_tx_ctrl, reg_dma3_ctrl, 16);

regrw!(reg_aes_ctrl, 0x540, 8);
regrw!(reg_aes_data, 0x548, 32);
regrw_idx!(reg_aes_key, 0x550, 8);
regrw!(reg_aes_key0, 0x550, 8);
regrw!(reg_aes_key1, 0x551, 8);
regrw!(reg_aes_key2, 0x552, 8);
regrw!(reg_aes_key3, 0x553, 8);
regrw!(reg_aes_key4, 0x554, 8);
regrw!(reg_aes_key5, 0x555, 8);
regrw!(reg_aes_key6, 0x556, 8);
regrw!(reg_aes_key7, 0x557, 8);
regrw!(reg_aes_key8, 0x558, 8);
regrw!(reg_aes_key9, 0x559, 8);
regrw!(reg_aes_key10, 0x55a, 8);
regrw!(reg_aes_key11, 0x55b, 8);
regrw!(reg_aes_key12, 0x55c, 8);
regrw!(reg_aes_key13, 0x55d, 8);
regrw!(reg_aes_key14, 0x55e, 8);
regrw!(reg_aes_key15, 0x55f, 8);

/****************************************************
gpio regs struct: begin  0x580
*****************************************************/
regrw_idx!(reg_gpio_in, 0x580, 8);
regrw_idx!(reg_gpio_ie, 0x581, 8);
regrw_idx!(reg_gpio_oen, 0x582, 8);
regrw_idx!(reg_gpio_out, 0x583, 8);
regrw_idx!(reg_gpio_pol, 0x584, 8);
regrw_idx!(reg_gpio_ds, 0x585, 8);
regrw_idx!(reg_gpio_gpio_func, 0x586, 8);

regrw_idx!(reg_gpio_irq_wakeup_en, 0x587, 8); // reg_irq_mask: FLD_IRQ_GPIO_EN
regrw_idx!(reg_gpio_irq_risc0_en, 0x5b8, 8); // reg_irq_mask: FLD_IRQ_GPIO_RISC0_EN
regrw_idx!(reg_gpio_irq_risc1_en, 0x5c0, 8); // reg_irq_mask: FLD_IRQ_GPIO_RISC1_EN
regrw_idx!(reg_gpio_irq_risc2_en, 0x5c8, 8); // reg_irq_mask: FLD_IRQ_GPIO_RISC2_EN

regrw!(reg_gpio_pa_in, 0x580, 8);
regrw!(reg_gpio_pa_ie, 0x581, 8);
regrw!(reg_gpio_pa_oen, 0x582, 8);
regrw!(reg_gpio_pa_out, 0x583, 8);
regrw!(reg_gpio_pa_pol, 0x584, 8);
regrw!(reg_gpio_pa_ds, 0x585, 8);
regrw!(reg_gpio_pa_gpio, 0x586, 8);
regrw!(reg_gpio_pa_irq_en, 0x587, 8);
regrw!(reg_gpio_pb_in, 0x588, 8);
regrw!(reg_gpio_pb_ie, 0x589, 8);
regrw!(reg_gpio_pb_oen, 0x58a, 8);
regrw!(reg_gpio_pb_out, 0x58b, 8);
regrw!(reg_gpio_pb_pol, 0x58c, 8);
regrw!(reg_gpio_pb_ds, 0x58d, 8);
regrw!(reg_gpio_pb_gpio, 0x58e, 8);
regrw!(reg_gpio_pb_irq_en, 0x58f, 8);

regrw!(reg_gpio_pc_in, 0x590, 8);
regrw!(reg_gpio_pc_ie, 0x591, 8);
regrw!(reg_gpio_pc_oen, 0x592, 8);
regrw!(reg_gpio_pc_out, 0x593, 8);
regrw!(reg_gpio_pc_pol, 0x594, 8);
regrw!(reg_gpio_pc_ds, 0x595, 8);
regrw!(reg_gpio_pc_gpio, 0x596, 8);
regrw!(reg_gpio_pc_irq_en, 0x597, 8);

regrw!(reg_gpio_pd_in, 0x598, 8);
regrw!(reg_gpio_pd_ie, 0x599, 8);
regrw!(reg_gpio_pd_oen, 0x59a, 8);
regrw!(reg_gpio_pd_out, 0x59b, 8);
regrw!(reg_gpio_pd_pol, 0x59c, 8);
regrw!(reg_gpio_pd_ds, 0x59d, 8);
regrw!(reg_gpio_pd_gpio, 0x59e, 8);
regrw!(reg_gpio_pd_irq_en, 0x59f, 8);

regrw!(reg_gpio_pe_in, 0x5a0, 8);
regrw!(reg_gpio_pe_ie, 0x5a1, 8);
regrw!(reg_gpio_pe_oen, 0x5a2, 8);
regrw!(reg_gpio_pe_out, 0x5a3, 8);
regrw!(reg_gpio_pe_pol, 0x5a4, 8);
regrw!(reg_gpio_pe_ds, 0x5a5, 8);
regrw!(reg_gpio_pe_gpio, 0x5a6, 8);
regrw!(reg_gpio_pe_irq_en, 0x5a7, 8);

regrw!(reg_gpio_pf_in, 0x5a8, 8);
regrw!(reg_gpio_pf_ie, 0x5a9, 8);
regrw!(reg_gpio_pf_oen, 0x5aa, 8);
regrw!(reg_gpio_pf_out, 0x5ab, 8);
regrw!(reg_gpio_pf_pol, 0x5ac, 8);
regrw!(reg_gpio_pf_ds, 0x5ad, 8);
regrw!(reg_gpio_pf_gpio, 0x5ae, 8);
regrw!(reg_gpio_pf_irq_en, 0x5af, 8);

regrw!(reg_gpio_pa_setting1, 0x580, 32);
regrw!(reg_gpio_pa_setting2, 0x584, 32);
regrw!(reg_gpio_pb_setting1, 0x588, 32);
regrw!(reg_gpio_pb_setting2, 0x58c, 32);
regrw!(reg_gpio_pc_setting1, 0x590, 32);
regrw!(reg_gpio_pc_setting2, 0x594, 32);
regrw!(reg_gpio_pd_setting1, 0x598, 32);
regrw!(reg_gpio_pd_setting2, 0x59c, 32);
regrw!(reg_gpio_pe_setting1, 0x5a0, 32);
regrw!(reg_gpio_pe_setting2, 0x5a4, 32);
regrw!(reg_gpio_pf_setting1, 0x5a8, 32);
regrw!(reg_gpio_pf_setting2, 0x5ac, 32);

regrw!(reg_gpio_ctrl, 0x5a4, 32);

pub enum GPIO_CTRL {
    GPIO_WAKEUP_EN = BIT!(0),
    GPIO_IRQ_EN = BIT!(1),
    I2S_SLAVE_EN = BIT!(2),
    RMII_REFCLK_OUTPUT_EN = BIT!(3),
}

regrw!(reg_gpio_config_func, 0x5b0, 32);
regrw!(reg_gpio_config_func0, 0x5b0, 8);

pub enum GPIO_CFG_FUNC0 {
    FLD_I2S_REFCLK_DMIC = BIT!(0),
    FLD_I2S_BCK_BB_PEAK = BIT!(1),
    FLD_I2S_BCK_PWM1 = BIT!(2),
    FLD_I2S_LCK_UART_RX = BIT!(3),
    FLD_I2S_LCK_PWM2 = BIT!(4),
    FLD_I2S_DO_UART_TX = BIT!(5),
    FLD_I2S_DO_PWM3 = BIT!(6),
    FLD_I2S_DI_DMIC = BIT!(7),
}

regrw!(reg_gpio_config_func1, 0x5b1, 8);
pub enum GPIO_CFG_FUNC1 {
    FLD_RP_TX_CYC1 = BIT!(0),
    FLD_RN_BB_RSSI = BIT!(1),
    FLD_GP6_BB_SS2 = BIT!(2),
    FLD_GP7_RXADC_CLK = BIT!(3),
    FLD_RP_T0 = BIT!(4),
    FLD_RN_T1 = BIT!(5),
    FLD_GP6_TE = BIT!(6),
    FLD_GP7_MDC = BIT!(7),
}

regrw!(reg_gpio_config_func2, 0x5b2, 8);
pub enum GPIO_CFG_FUNC2 {
    FLD_GP8_RXADC_DAT = BIT!(0),
    FLD_GP9_BB_SS1 = BIT!(1),
    FLD_GP10_BBSS0 = BIT!(2),
    FLD_SWS_BB_GAIN4 = BIT!(3),
    FLD_DMIC_CK_BBCLK_BB = BIT!(4),
    FLD_DMIC_CK_REFCLK = BIT!(5),
    FLD_I2S_BCK_R0 = BIT!(6),
    FLD_I2S_LCK_R1 = BIT!(7),
}

regrw!(reg_gpio_config_func3, 0x5b3, 8);
enum GPIO_CFG_FUNC3 {
    FLD_CN_BB_GAIN3 = BIT!(0),
    FLD_CK_BB_GAIN2 = BIT!(1),
    FLD_DO_BB_GAIN1 = BIT!(2),
    FLD_DI_BB_GAIN0 = BIT!(3),
    //	FLD_I2S_LCK_PWM2	=	BIT!(4),//NOT SURE
    FLD_I2S_DO_RXDV = BIT!(5),
    FLD_I2S_DI_RXER = BIT!(6),
    FLD_I2S_DI_TXSD = BIT!(7),
}

regrw!(reg_gpio_config_func4, 0x5b4, 8);
pub enum GPIO_CFG_FUNC4 {
    FLD_DMIC_CK_RX_CLK = BIT!(0),
    FLD_I2S_DI_RX_DAT = BIT!(1),
}

regrw!(reg_gpio_wakeup_irq, 0x5b5, 8);
pub enum FLD_GPIO_WAKEUP_IRQ {
    WAKEUP_EN = BIT!(2),
    INTERRUPT_EN = BIT!(3),
}

/****************************************************
timer regs struct: begin  0x620
*****************************************************/
regrw!(reg_tmr_ctrl, 0x620, 32);
regrw!(reg_tmr_ctrl16, 0x620, 16);
regrw!(reg_tmr_ctrl8, 0x620, 8);

pub enum FLD_TMR {
    TMR0_EN = BIT!(0),
    TMR0_MODE = BIT_RNG!(1, 2),
    TMR1_EN = BIT!(3),
    TMR1_MODE = BIT_RNG!(4, 5),
    TMR2_EN = BIT!(6),
    TMR2_MODE = BIT_RNG!(7, 8),
    TMR_WD_CAPT = BIT_RNG!(9, 22),
    TMR_WD_EN = BIT!(23),
    TMR0_STA = BIT!(24),
    TMR1_STA = BIT!(25),
    TMR2_STA = BIT!(26),
    CLR_WD = BIT!(27),
}

pub const WATCHDOG_TIMEOUT_COEFF: u32 = 18; //  check register definiton, 0x622

regrw!(reg_tmr_sta, 0x623, 8);
pub enum FLD_TMR_STA {
    TMR0 = BIT!(0),
    TMR1 = BIT!(1),
    TMR2 = BIT!(2),
    WD = BIT!(3),
}

regrw!(reg_tmr0_capt, 0x624, 32);
regrw!(reg_tmr1_capt, 0x628, 32);
regrw!(reg_tmr2_capt, 0x62c, 32);
// #define reg_tmr_capt(i)			REG_ADDR32(0x624 + ((i) << 2))
regrw!(reg_tmr0_tick, 0x630, 32);
regrw!(reg_tmr1_tick, 0x634, 32);
regrw!(reg_tmr2_tick, 0x638, 32);
// #define reg_tmr_tick(i)			REG_ADDR32(0x630 + ((i) << 2))

/****************************************************
interrupt regs struct: begin  0x640
*****************************************************/
regrw!(reg_irq_mask, 0x640, 32);
regrw!(reg_irq_pri, 0x644, 32);
regrw!(reg_irq_src, 0x648, 32);
regrw!(reg_irq_src3, 0x64a, 8);
#[repr(usize)]
pub enum FLD_IRQ {
    TMR0_EN = BIT!(0),
    TMR1_EN = BIT!(1),
    TMR2_EN = BIT!(2),
    USB_PWDN_EN = BIT!(3),
    DMA_EN = BIT!(4),
    DAM_FIFO_EN = BIT!(5),
    SBC_MAC_EN = BIT!(6),
    HOST_CMD_EN = BIT!(7),

    EP0_SETUP_EN = BIT!(8),
    EP0_DAT_EN = BIT!(9),
    EP0_STA_EN = BIT!(10),
    SET_INTF_EN = BIT!(11),
    IRQ4_EN = BIT!(12),
    ZB_RT_EN = BIT!(13),
    SW_EN = BIT!(14),
    AN_EN = BIT!(15),

    USB_250US_EN = BIT!(16),
    USB_RST_EN = BIT!(17),
    GPIO_EN = BIT!(18),
    PM_EN = BIT!(19),
    SYSTEM_TIMER = BIT!(20),
    GPIO_RISC0_EN = BIT!(21),
    GPIO_RISC1_EN = BIT!(22),
    GPIO_RISC2_EN = BIT!(23),

    EN = BIT_RNG!(24, 31),
}

regrw!(reg_irq_en, 0x643, 8);

regrw!(reg_system_tick, 0x740, 32);
regrw!(reg_system_tick_irq, 0x744, 32);
regrw!(reg_system_wakeup_tick, 0x748, 32);
regrw!(reg_system_tick_mode, 0x74c, 8);
regrw!(reg_system_tick_ctrl, 0x74f, 8);

pub enum FLD_SYSTEM_TICK {
    START = BIT!(0),
    STOP = BIT!(1),
}

impl FLD_SYSTEM_TICK {
    pub const RUNNING: FLD_SYSTEM_TICK = FLD_SYSTEM_TICK::STOP;
    pub const IRQ_EN: FLD_SYSTEM_TICK = FLD_SYSTEM_TICK::STOP;
}

/****************************************************
PWM regs define:  begin  0x780
*****************************************************/
regrw!(reg_pwm_enable, 0x780, 8);
regrw!(reg_pwm_clk, 0x781, 8);
regrw!(reg_pwm_mode, 0x782, 8);
regrw!(reg_pwm_invert, 0x783, 8);
regrw!(reg_pwm_n_invert, 0x784, 8);
regrw!(reg_pwm_pol, 0x785, 8);

regrw_idx!(reg_pwm_phase, 0x788, 16);
regrw_idx!(reg_pwm_cycle, 0x794, 32);
regrw_idx!(reg_pwm_cmp, 0x794, 16);

#[repr(usize)]
pub enum FLD_PWM {
    CMP = BIT_RNG!(0, 15),
    MAX = BIT_RNG!(16, 31),
}

regrw_idx!(reg_pwm_pulse_num, 0x7ac, 16); // i == 0, 1
regrw!(reg_pwm_irq_mask, 0x7b0, 8);
regrw!(reg_pwm_irq_sta, 0x7b1, 8);

regrw!(reg_rf_mode_control, 0xf00, 8);
regrw!(reg_rf_sn, 0xf01, 8);
regrw!(reg_rf_tx_wail_settle_time, 0xf04, 32);
regrw!(reg_rf_mode, 0xf16, 8);
regrw!(reg_rf_sched_tick, 0xf18, 32);

///////////////////// PM register /////////////////////////

pub const  		rega_deepsleep_flag: u8 =		0x3f;		//0x34 - 0x39 (watch dog reset)
// #define		rega_deepsleep_flag: u8 =		0x34		//0x3a - 0x3b (power-on reset)
// pub const  		flag_deepsleep_wakeup	(analog_read(0x3f) & 0x40)

pub const  		rega_wakeup_en_val0: u8 =		0x41;
pub const  		rega_wakeup_en_val1: u8 =		0x42;
pub const  		rega_wakeup_en_val2: u8 =		0x43;
pub const  		raga_gpio_wkup_pol: u8 =		0x44;

pub const  		raga_pga_gain0: u8 =		0x86;
pub const  		raga_pga_gain1: u8 = 0x87;