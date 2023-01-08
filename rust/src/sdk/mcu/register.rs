use ::{BIT, BIT_RNG, BIT_MASK_LEN};
extern crate core;
extern crate paste;

pub static REG_BASE_ADDR: u32 =	0x800000;

#[macro_export]
macro_rules! regrw_idx {
    ( $x:ident, $a:expr, $s:ty ) => {
        paste::paste! {
            pub fn [<read_ $x>](i: u32) -> $s {
                unsafe {
                    return core::ptr::read_volatile(((REG_BASE_ADDR + $a) + i) as *mut $s)
                }
            }

            pub fn [<write_ $x>](value: $s, i: u32) {
                unsafe {
                    core::ptr::write_volatile(((REG_BASE_ADDR + $a) + i) as *mut $s, value)
                }
            }
        }
    };
}

#[macro_export]
macro_rules! regrw {
    ( $x:ident, $a:expr, $s:ty ) => {
        paste::paste! {
            pub fn [<read_ $x>]() -> $s {
                unsafe {
                    return core::ptr::read_volatile((REG_BASE_ADDR + $a) as *mut $s)
                }
            }

            pub fn [<write_ $x>](value: $s) {
                unsafe {
                    core::ptr::write_volatile((REG_BASE_ADDR + $a) as *mut $s, value)
                }
            }
        }
    };
}

#[macro_export]
macro_rules! regrw_copy {
    ( $x:ident, $y:ident, $t:ty ) => {
        paste::paste! {
            pub fn [<read_ $x>]() -> $t {
                [<read_ $y>]()
            }

            pub fn [<write_ $x>](value: $t) {
                [<write_ $y>](value)
            }
        }
    };
}

/****************************************************
 master spi regs struct: begin  addr : 0x0c
 *****************************************************/
regrw!(reg_master_spi_data, 0x0c, u8);
regrw!(reg_master_spi_ctrl, 0x0d, u8);

pub enum FLD_MASTER_SPI {
    CS = 		BIT!(0),
    SDO = 		BIT!(1),
    CONT = 		BIT!(2),
    RD = 		BIT!(3),
    BUSY = 		BIT!(4),
}

/****************************************************
 sys regs struct: begin  addr : 0x60
 *****************************************************/
regrw!(reg_rst0, 0x60, u8);
regrw!(reg_rst0_16, 0x60, u16);
regrw!(reg_rst1, 0x61, u8);
regrw!(reg_rst2, 0x62, u8);
regrw!(reg_rst_clk0, 0x60, u32);
pub enum FLD_RST {
    SPI = 				BIT!(0),
    I2C = 				BIT!(1),
    USB = 				BIT!(2),
    USB_PHY = 			BIT!(3),
    MCU = 				BIT!(4),
    MAC =				BIT!(5),
    AIF = 				BIT!(6),
    BB = 				BIT!(7),
    GPIO = 				BIT!(8),
    ALGM = 				BIT!(9),
    DMA =				BIT!(10),
    UART = 				BIT!(11),
    PWM = 				BIT!(12),
    AES = 				BIT!(13),
    SWR_M =				BIT!(14),
    SWR_S =				BIT!(15),
    SBC =				BIT!(16),
    AUD =				BIT!(17),
    DFIFO =				BIT!(18),
    ADC =				BIT!(19),
    SOFT_MCU =			BIT!(20),
    MCIC = 				BIT!(21),
    SOFT_MCIC =			BIT!(22),
    RSV =				BIT!(23)
}

impl FLD_RST {
    pub const ZB: FLD_RST = FLD_RST::BB;
}

pub enum FLD_CLK_EN {
    GPIO_EN = 			BIT!(0),
    ALGM_EN = 			BIT!(1),
    DMA_EN = 			BIT!(2),
    UART_EN = 			BIT!(3),
    PWM_EN = 			BIT!(4),
    AES_EN = 			BIT!(5),
    PLL_EN = 			BIT!(6),
    SWIRE_EN = 			BIT!(7),
    SBC_EN =			BIT!(8),
    AUD_EN =			BIT!(9),
    DIFIO_EN = 			BIT!(10),
    I2S =				BIT_RNG!(11,12),
    C32K =				BIT_RNG!(13,15),
    SPI_EN =			BIT!(24),
    I2C_EN =			BIT!(25),
    USB_EN =			BIT!(26),
    USB_PHY_EN =		BIT!(27),
    MCU_EN =			BIT!(28),
    MAC_EN =			BIT!(29),
    ADC_EN =			BIT!(30),	// ADC interface
    ZB_EN =				BIT!(31),
}

regrw!(reg_clk_en, 0x64, u16);
regrw!(reg_clk_en1, 0x64, u8);

regrw!(reg_clk_en2, 0x65, u8);

pub enum FLD_CLK2_EN {
    SBC_EN =			BIT!(0),
    AUD_EN =			BIT!(1),
    DIFIO_EN = 		    BIT!(2),
    I2S =				BIT_RNG!(3,4),
    C32K =				BIT_RNG!(5,7),
}

regrw!(reg_clk_sel, 0x66, u8);

pub enum FLD_CLK_SEL {
    DIV = 			BIT_RNG!(0,4),
    SRC =			BIT_RNG!(5,7),
}

regrw!(reg_i2s_step, 0x67, u8);
pub enum FLD_I2S_STEP {
    STEP = 				BIT_RNG!(0,6),
    CLK_EN =			BIT!(7),
}

regrw!(reg_i2s_mod, 0x68, u8);


// pub fn SET_SDM_CLOCK_MHZ(f_mhz: u32)	{
//     write_reg_i2s_step(FLD_I2S::CLK_EN as u32 | f_mhz);
//     write_reg_i2s_mod(0xc0);
// }

//////  analog controls 0xb8 ///////
regrw!(reg_ana_ctrl32, 0xb8, u32);
regrw!(reg_ana_addr_data, 0xb8, u16);
regrw!(reg_ana_addr, 0xb8, u8);
regrw!(reg_ana_data, 0xb9, u8);
regrw!(reg_ana_ctrl, 0xba, u8);

pub enum FLD_ANA {
    BUSY  = 			BIT!(0),
    RSV	=				BIT!(4),
    RW  = 				BIT!(5),
    START  = 			BIT!(6),
    CYC  = 				BIT!(7),
}

/****************************************************
 dma mac regs struct: begin  addr : 0x500
 *****************************************************/
regrw!(reg_dma0_addr, 0x500, u16);
regrw!(reg_dma0_ctrl, 0x502, u16);
regrw!(reg_dma1_addr, 0x504, u16);
regrw!(reg_dma1_ctrl, 0x506, u16);
regrw!(reg_dma2_addr, 0x508, u16);
regrw!(reg_dma2_ctrl, 0x50a, u16);
regrw!(reg_dma3_addr, 0x50c, u16);
regrw!(reg_dma3_ctrl, 0x50e, u16);
regrw!(reg_dma4_addr, 0x510, u16);
regrw!(reg_dma4_ctrl, 0x512, u16);
regrw!(reg_dma5_addr, 0x514, u16);
regrw!(reg_dma5_ctrl, 0x516, u16);
regrw!(reg_dma_tx_rptr, 0x52a, u8);
regrw!(reg_dma_tx_wptr, 0x52b, u8);
regrw!(reg_dma_tx_fifo, 0x52c, u16);

enum FLD_DMA {
    BUF_SIZE =			BIT_RNG!(0,7),
    WR_MEM =			BIT!(8),
    PINGPONG_EN =		BIT!(9),
    FIFO_EN =			BIT!(10),
    AUTO_MODE =			BIT!(11),
    BYTE_MODE =			BIT!(12),

    RPTR_CLR =			BIT!(4),
    RPTR_NEXT =			BIT!(5),
    RPTR_SET =			BIT!(6),
}

impl FLD_DMA {
    pub const ETH_RX: u32 = BIT!(0);
    pub const ETH_TX: u32 = BIT!(1);
    pub const RF_RX: u32 = BIT!(2);
    pub const RF_TX: u32 = BIT!(3);
}

regrw!(reg_dma_chn_en, 0x520, u8);
regrw!(reg_dma_chn_irq_msk, 0x521, u8);
regrw!(reg_dma_tx_rdy0, 0x524, u8);
regrw!(reg_dma_tx_rdy1, 0x525, u8);
regrw!(reg_dma_rx_rdy0, 0x526, u8);
regrw_copy!(reg_dma_irq_src, reg_dma_rx_rdy0, u8);
regrw!(reg_dma_rx_rdy1, 0x527, u8);

//  The default channel assignment
regrw_copy!(reg_dma_eth_rx_addr, reg_dma0_addr, u16);
regrw_copy!(reg_dma_eth_rx_ctrl, reg_dma0_ctrl, u16);
regrw_copy!(reg_dma_eth_tx_addr, reg_dma1_addr, u16);

regrw_copy!(reg_dma_rf_rx_addr, reg_dma2_addr, u16);
regrw_copy!(reg_dma_rf_rx_ctrl, reg_dma2_ctrl, u16);
regrw_copy!(reg_dma_rf_tx_addr, reg_dma3_addr, u16);
regrw_copy!(reg_dma_rf_tx_ctrl, reg_dma3_ctrl, u16);

regrw!(reg_aes_ctrl, 0x540, u8);
regrw!(reg_aes_data, 0x548, u32);
// #define reg_aes_key(key_id)     reg_aes_key##key_id
regrw!(reg_aes_key0, 0x550, u8);
regrw!(reg_aes_key1, 0x551, u8);
regrw!(reg_aes_key2, 0x552, u8);
regrw!(reg_aes_key3, 0x553, u8);
regrw!(reg_aes_key4, 0x554, u8);
regrw!(reg_aes_key5, 0x555, u8);
regrw!(reg_aes_key6, 0x556, u8);
regrw!(reg_aes_key7, 0x557, u8);
regrw!(reg_aes_key8, 0x558, u8);
regrw!(reg_aes_key9, 0x559, u8);
regrw!(reg_aes_key10, 0x55a, u8);
regrw!(reg_aes_key11, 0x55b, u8);
regrw!(reg_aes_key12, 0x55c, u8);
regrw!(reg_aes_key13, 0x55d, u8);
regrw!(reg_aes_key14, 0x55e, u8);
regrw!(reg_aes_key15, 0x55f, u8);

/****************************************************
 gpio regs struct: begin  0x580
 *****************************************************/
regrw_idx!(reg_gpio_in, 0x580, u8);
regrw_idx!(reg_gpio_ie, 0x581, u8);
regrw_idx!(reg_gpio_oen, 0x582, u8);
regrw_idx!(reg_gpio_out, 0x583, u8);
regrw_idx!(reg_gpio_pol, 0x584, u8);
regrw_idx!(reg_gpio_ds, 0x585, u8);
regrw_idx!(reg_gpio_gpio_func, 0x586, u8);

regrw_idx!(reg_gpio_irq_wakeup_en, 0x587, u8);  // reg_irq_mask: FLD_IRQ_GPIO_EN
regrw_idx!(reg_gpio_irq_risc0_en, 0x5b8, u8);   // reg_irq_mask: FLD_IRQ_GPIO_RISC0_EN
regrw_idx!(reg_gpio_irq_risc1_en, 0x5c0, u8);   // reg_irq_mask: FLD_IRQ_GPIO_RISC1_EN
regrw_idx!(reg_gpio_irq_risc2_en, 0x5c8, u8);   // reg_irq_mask: FLD_IRQ_GPIO_RISC2_EN

regrw!(reg_gpio_pa_in, 0x580, u8);
regrw!(reg_gpio_pa_ie, 0x581, u8);
regrw!(reg_gpio_pa_oen, 0x582, u8);
regrw!(reg_gpio_pa_out, 0x583, u8);
regrw!(reg_gpio_pa_pol, 0x584, u8);
regrw!(reg_gpio_pa_ds, 0x585, u8);
regrw!(reg_gpio_pa_gpio, 0x586, u8);
regrw!(reg_gpio_pa_irq_en, 0x587, u8);
regrw!(reg_gpio_pb_in, 0x588, u8);
regrw!(reg_gpio_pb_ie, 0x589, u8);
regrw!(reg_gpio_pb_oen, 0x58a, u8);
regrw!(reg_gpio_pb_out, 0x58b, u8);
regrw!(reg_gpio_pb_pol, 0x58c, u8);
regrw!(reg_gpio_pb_ds, 0x58d, u8);
regrw!(reg_gpio_pb_gpio, 0x58e, u8);
regrw!(reg_gpio_pb_irq_en, 0x58f, u8);


regrw!(reg_gpio_pc_in, 0x590, u8);
regrw!(reg_gpio_pc_ie, 0x591, u8);
regrw!(reg_gpio_pc_oen, 0x592, u8);
regrw!(reg_gpio_pc_out, 0x593, u8);
regrw!(reg_gpio_pc_pol, 0x594, u8);
regrw!(reg_gpio_pc_ds, 0x595, u8);
regrw!(reg_gpio_pc_gpio, 0x596, u8);
regrw!(reg_gpio_pc_irq_en, 0x597, u8);

regrw!(reg_gpio_pd_in, 0x598, u8);
regrw!(reg_gpio_pd_ie, 0x599, u8);
regrw!(reg_gpio_pd_oen, 0x59a, u8);
regrw!(reg_gpio_pd_out, 0x59b, u8);
regrw!(reg_gpio_pd_pol, 0x59c, u8);
regrw!(reg_gpio_pd_ds, 0x59d, u8);
regrw!(reg_gpio_pd_gpio, 0x59e, u8);
regrw!(reg_gpio_pd_irq_en, 0x59f, u8);

regrw!(reg_gpio_pe_in, 0x5a0, u8);
regrw!(reg_gpio_pe_ie, 0x5a1, u8);
regrw!(reg_gpio_pe_oen, 0x5a2, u8);
regrw!(reg_gpio_pe_out, 0x5a3, u8);
regrw!(reg_gpio_pe_pol, 0x5a4, u8);
regrw!(reg_gpio_pe_ds, 0x5a5, u8);
regrw!(reg_gpio_pe_gpio, 0x5a6, u8);
regrw!(reg_gpio_pe_irq_en, 0x5a7, u8);

regrw!(reg_gpio_pf_in, 0x5a8, u8);
regrw!(reg_gpio_pf_ie, 0x5a9, u8);
regrw!(reg_gpio_pf_oen, 0x5aa, u8);
regrw!(reg_gpio_pf_out, 0x5ab, u8);
regrw!(reg_gpio_pf_pol, 0x5ac, u8);
regrw!(reg_gpio_pf_ds, 0x5ad, u8);
regrw!(reg_gpio_pf_gpio, 0x5ae, u8);
regrw!(reg_gpio_pf_irq_en, 0x5af, u8);

regrw!(reg_gpio_pa_setting1, 0x580, u32);
regrw!(reg_gpio_pa_setting2, 0x584, u32);
regrw!(reg_gpio_pb_setting1, 0x588, u32);
regrw!(reg_gpio_pb_setting2, 0x58c, u32);
regrw!(reg_gpio_pc_setting1, 0x590, u32);
regrw!(reg_gpio_pc_setting2, 0x594, u32);
regrw!(reg_gpio_pd_setting1, 0x598, u32);
regrw!(reg_gpio_pd_setting2, 0x59c, u32);
regrw!(reg_gpio_pe_setting1, 0x5a0, u32);
regrw!(reg_gpio_pe_setting2, 0x5a4, u32);
regrw!(reg_gpio_pf_setting1, 0x5a8, u32);
regrw!(reg_gpio_pf_setting2, 0x5ac, u32);

regrw!(reg_gpio_ctrl, 0x5a4, u32);

pub enum GPIO_CTRL {
    GPIO_WAKEUP_EN = BIT!(0),
    GPIO_IRQ_EN	   = BIT!(1),
    I2S_SLAVE_EN   = BIT!(2),
    RMII_REFCLK_OUTPUT_EN = BIT!(3),
}

regrw!(reg_gpio_config_func, 0x5b0, u32);
regrw!(reg_gpio_config_func0, 0x5b0, u8);

pub enum GPIO_CFG_FUNC0 {
    FLD_I2S_REFCLK_DMIC	=	BIT!(0),
    FLD_I2S_BCK_BB_PEAK	=	BIT!(1),
    FLD_I2S_BCK_PWM1	=	BIT!(2),
    FLD_I2S_LCK_UART_RX	=	BIT!(3),
    FLD_I2S_LCK_PWM2	=	BIT!(4),
    FLD_I2S_DO_UART_TX	=	BIT!(5),
    FLD_I2S_DO_PWM3		=	BIT!(6),
    FLD_I2S_DI_DMIC		=	BIT!(7),
}

regrw!(reg_gpio_config_func1, 0x5b1, u8);
pub enum GPIO_CFG_FUNC1 {
    FLD_RP_TX_CYC1		=	BIT!(0),
    FLD_RN_BB_RSSI		=	BIT!(1),
    FLD_GP6_BB_SS2		=	BIT!(2),
    FLD_GP7_RXADC_CLK	=	BIT!(3),
    FLD_RP_T0			=	BIT!(4),
    FLD_RN_T1			=	BIT!(5),
    FLD_GP6_TE			=	BIT!(6),
    FLD_GP7_MDC			=	BIT!(7),
}

regrw!(reg_gpio_config_func2, 0x5b2, u8);
pub enum GPIO_CFG_FUNC2 {
    FLD_GP8_RXADC_DAT	=	BIT!(0),
    FLD_GP9_BB_SS1		=	BIT!(1),
    FLD_GP10_BBSS0		=	BIT!(2),
    FLD_SWS_BB_GAIN4	=	BIT!(3),
    FLD_DMIC_CK_BBCLK_BB	=	BIT!(4),
    FLD_DMIC_CK_REFCLK	=	BIT!(5),
    FLD_I2S_BCK_R0		=	BIT!(6),
    FLD_I2S_LCK_R1		=	BIT!(7),
}

regrw!(reg_gpio_config_func3, 0x5b3, u8);
enum GPIO_CFG_FUNC3 {
    FLD_CN_BB_GAIN3		=	BIT!(0),
    FLD_CK_BB_GAIN2		=	BIT!(1),
    FLD_DO_BB_GAIN1		=	BIT!(2),
    FLD_DI_BB_GAIN0		=	BIT!(3),
    //	FLD_I2S_LCK_PWM2	=	BIT!(4),//NOT SURE
    FLD_I2S_DO_RXDV		=	BIT!(5),
    FLD_I2S_DI_RXER		=	BIT!(6),
    FLD_I2S_DI_TXSD		=	BIT!(7),
}

regrw!(reg_gpio_config_func4, 0x5b4, u8);
pub enum GPIO_CFG_FUNC4 {
    FLD_DMIC_CK_RX_CLK	=	BIT!(0),
    FLD_I2S_DI_RX_DAT	=	BIT!(1),
}

regrw!(reg_gpio_wakeup_irq, 0x5b5, u8);
pub enum FLD_GPIO_WAKEUP_IRQ {
    WAKEUP_EN		=	BIT!(2),
    INTERRUPT_EN	=	BIT!(3),
}

/****************************************************
 timer regs struct: begin  0x620
 *****************************************************/
regrw!(reg_tmr_ctrl, 0x620, u32);
regrw!(reg_tmr_ctrl16, 0x620, u16);
regrw!(reg_tmr_ctrl8, 0x620, u8);

pub enum FLD_TMR {
    TMR0_EN =				BIT!(0),
    TMR0_MODE =				BIT_RNG!(1,2),
    TMR1_EN = 				BIT!(3),
    TMR1_MODE =				BIT_RNG!(4,5),
    TMR2_EN =				BIT!(6),
    TMR2_MODE = 			BIT_RNG!(7,8),
    TMR_WD_CAPT = 			BIT_RNG!(9,22),
    TMR_WD_EN =				BIT!(23),
    TMR0_STA =				BIT!(24),
    TMR1_STA =				BIT!(25),
    TMR2_STA =				BIT!(26),
    CLR_WD =				BIT!(27),
}

pub static WATCHDOG_TIMEOUT_COEFF: u32 =	18;		//  check register definiton, 0x622

regrw!(reg_tmr_sta, 0x623, u8);
pub enum FLD_TMR_STA {
    TMR0 =			BIT!(0),
    TMR1 =			BIT!(1),
    TMR2 =			BIT!(2),
    WD =			BIT!(3),
}

regrw!(reg_tmr0_capt, 0x624, u32);
regrw!(reg_tmr1_capt, 0x628, u32);
regrw!(reg_tmr2_capt, 0x62c, u32);
// #define reg_tmr_capt(i)			REG_ADDR32(0x624 + ((i) << 2))
regrw!(reg_tmr0_tick, 0x630, u32);
regrw!(reg_tmr1_tick, 0x634, u32);
regrw!(reg_tmr2_tick, 0x638, u32);
// #define reg_tmr_tick(i)			REG_ADDR32(0x630 + ((i) << 2))

/****************************************************
 interrupt regs struct: begin  0x640
 *****************************************************/
regrw!(reg_irq_mask, 0x640, u32);
regrw!(reg_irq_pri, 0x644, u32);
regrw!(reg_irq_src, 0x648, u32);
regrw!(reg_irq_src3, 0x64a, u8);
pub enum FLD_IRQ {
    TMR0_EN =			BIT!(0),
    TMR1_EN =			BIT!(1),
    TMR2_EN =			BIT!(2),
    USB_PWDN_EN =		BIT!(3),
    DMA_EN =			BIT!(4),
    DAM_FIFO_EN =		BIT!(5),
    SBC_MAC_EN =		BIT!(6),
    HOST_CMD_EN =		BIT!(7),

    EP0_SETUP_EN =		BIT!(8),
    EP0_DAT_EN =		BIT!(9),
    EP0_STA_EN =		BIT!(10),
    SET_INTF_EN =		BIT!(11),
    IRQ4_EN =			BIT!(12),
    ZB_RT_EN =			BIT!(13),
    SW_EN =				BIT!(14),
    AN_EN =				BIT!(15),

    USB_250US_EN =		BIT!(16),
    USB_RST_EN =		BIT!(17),
    GPIO_EN =			BIT!(18),
    PM_EN =				BIT!(19),
    SYSTEM_TIMER =		BIT!(20),
    GPIO_RISC0_EN =		BIT!(21),
    GPIO_RISC1_EN =		BIT!(22),
    GPIO_RISC2_EN = 	BIT!(23),

    EN =				BIT_RNG!(24,31),
}

regrw!(reg_irq_en, 0x643, u8);

regrw!(reg_system_tick, 0x740, u32);
regrw!(reg_system_tick_irq, 0x744, u32);
regrw!(reg_system_wakeup_tick, 0x748, u32);
regrw!(reg_system_tick_mode, 0x74c, u8);
regrw!(reg_system_tick_ctrl, 0x74f, u8);

pub enum FLD_SYSTEM_TICK {
    START	=		BIT!(0),
    STOP	=		BIT!(1)
}

impl FLD_SYSTEM_TICK {
    pub const RUNNING: FLD_SYSTEM_TICK = FLD_SYSTEM_TICK::STOP;
    pub const IRQ_EN: FLD_SYSTEM_TICK = FLD_SYSTEM_TICK::STOP;
}

/****************************************************
 PWM regs define:  begin  0x780
 *****************************************************/
regrw!(reg_pwm_enable, 0x780, u8);
regrw!(reg_pwm_clk, 0x781, u8);
regrw!(reg_pwm_mode, 0x782, u8);
regrw!(reg_pwm_invert, 0x783, u8);
regrw!(reg_pwm_n_invert, 0x784, u8);
regrw!(reg_pwm_pol, 0x785, u8);

regrw_idx!(reg_pwm_phase, 0x788, u16);
regrw_idx!(reg_pwm_cycle, 0x794, u32);
regrw_idx!(reg_pwm_cmp, 0x794, u16);

pub enum FLD_PWM {
    CMP  = 				BIT_RNG!(0,15),
    MAX  = 				BIT_RNG!(16,31),
}

regrw_idx!(reg_pwm_pulse_num, 0x7ac, u16);   // i == 0, 1
regrw!(reg_pwm_irq_mask, 0x7b0, u8);
regrw!(reg_pwm_irq_sta, 0x7b1, u8);
