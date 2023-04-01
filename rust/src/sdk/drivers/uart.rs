use core::cmp::min;
use core::mem::size_of;
use core::ptr::addr_of;
use crate::{app, BIT, blinken};
use crate::embassy::yield_now::yield_now;
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_HZ, clock_time, clock_time_exceed};
use crate::sdk::mcu::gpio::{AS_UART, GPIO_PIN_TYPE, gpio_set_func, gpio_set_input_en, gpio_set_output_en};
use crate::sdk::mcu::register::{FLD_DMA, FLD_IRQ, read_reg8, read_reg_dma_chn_en, read_reg_dma_chn_irq_msk, read_reg_dma_rx_rdy0, read_reg_irq_mask, write_reg16, write_reg8, write_reg_dma0_addr, write_reg_dma0_ctrl, write_reg_dma1_addr, write_reg_dma_chn_en, write_reg_dma_chn_irq_msk, write_reg_dma_rx_rdy0, write_reg_dma_tx_rdy0, write_reg_irq_mask, write_reg_rst0};
use crate::sdk::mcu::watchdog::wd_clear;

pub const UART_DATA_LEN: usize = 44;      // data max 252

pub enum UARTIRQMASK {
	RX      = BIT!(0),
	TX      = BIT!(1),
	ALL     = UARTIRQMASK::RX as isize | UARTIRQMASK::TX as isize,
}

enum HARDWARECONTROL {
    CTSWODDPARITY = 0x0e,
    CTSWEVENPARITY = 0x06,
    CTSONLY = 0x02,
    ODDPARITY = 0x0C,
    EVENPARITY = 0x04,
    NOCONTROL = 0x00,
}

#[derive(Clone, Copy, Debug)]
// This struct must be a multiple of 16 bytes in size
pub struct uart_data_t {
    pub len: u32,        // data max 252
    pub data: [u8; UART_DATA_LEN]
}


pub struct UartDriver {
    txdata_buf: uart_data_t, // not for user

    // data max 252, user must copy rxdata to other Ram,but not use directly
    pub rxdata_buf: uart_data_t,

    uart_tx_busy_flag: bool,
}

impl UartDriver {
    pub const fn default() -> Self {
        Self {
            txdata_buf: uart_data_t{len: 0, data: [0; UART_DATA_LEN]}, // not for user

            // data max 252, user must copy rxdata to other Ram,but not use directly
            rxdata_buf: uart_data_t{len: 0, data: [0; UART_DATA_LEN]},

            uart_tx_busy_flag: false,
        }
    }

    pub fn init(&mut self) {
        gpio_set_func(GPIO_PIN_TYPE::GPIO_UTX as u32, AS_UART);
        gpio_set_func(GPIO_PIN_TYPE::GPIO_URX as u32, AS_UART);

        gpio_set_input_en(GPIO_PIN_TYPE::GPIO_URX as u32, 1);

        self.uart_buff_init();

        // CLK32M_UART115200
        self.uart_init(69, 3, true, true, HARDWARECONTROL::NOCONTROL);
    }

    /*******************************************************
    *
    *	@brief	uart initiate, set uart clock divider, bitwidth and the uart work mode
    *
    *	@param	uartCLKdiv - uart clock divider
    *			bwpc - bitwidth, should be set to larger than 2
    *			en_rx_irq - '1' enable rx irq; '0' disable.
    *			en_tx_irq - enable tx irq; '0' disable.
    *			hdwC - enum variable of hardware control functions
    *
    *	@return	'1' set success; '0' set error probably bwpc smaller than 3.
    *
    *		BaudRate = sclk/((uartCLKdiv+1)*(bwpc+1))
    *		SYCLK = 16Mhz
            115200		9			13
            9600		103			15
    *
    *		SYCLK = 32Mhz
    *		115200		19			13
            9600		237			13
    */

    fn uart_init(&mut self, uart_clkdiv: u16, bwpc: u8, en_rx_irq: bool, en_tx_irq: bool, hdwC: HARDWARECONTROL) -> bool{
        if bwpc < 3 {
            return false;
        }

        let baudrate = (CLOCK_SYS_CLOCK_HZ / (uart_clkdiv as u32 + 1)) / (bwpc as u32 + 1);

        write_reg16(0x800094, uart_clkdiv | 0x8000); //set uart clk divider and enable clock divider
        write_reg8(0x800096,0x30 | bwpc); //set bit width and enable rx/tx DMA

        if baudrate > 100000 {          // 115200
            write_reg8(0x80009a,0xff);
            write_reg8(0x80009b,3);
        }
        else
        {
            write_reg8(0x80009a,(bwpc+1)*12);//one byte includes 12 bits at most
            write_reg8(0x80009b,1);//For save
        }

        //write_reg8(0x800097,0x00);//No clts and rts
        write_reg8(0x800097,hdwC as u8);//set hardware control function

        //receive DMA and buffer details

        write_reg8(0x800503,0x01);//set DMA 0 mode to 0x01 for receive
        write_reg8(0x800507,0x00); //DMA1 mode to send
        Self::uart_irqsource_get(); //clear uart irq
        if en_rx_irq {
            write_reg_dma_chn_irq_msk(read_reg_dma_chn_irq_msk() | FLD_DMA::ETH_RX as u8); //open dma1 interrupt mask
            write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::DMA_EN as u32); //open dma interrupt mask

            write_reg_dma_chn_en(read_reg_dma_chn_en() | BIT!(0));
        }
        if en_tx_irq {
            write_reg_dma_chn_irq_msk(read_reg_dma_chn_irq_msk() | FLD_DMA::ETH_TX as u8); //open dma1 interrupt mask
            write_reg_irq_mask(read_reg_irq_mask() | FLD_IRQ::DMA_EN as u32); //open dma interrupt mask

            write_reg_dma_chn_en(read_reg_dma_chn_en() | BIT!(1));
        }
        return true;
    }

    /******************************************************************************
    *
    *	@brief		get the uart IRQ source and clear the IRQ status, need to be called in the irq process function
    *
    *	@return		uart_irq_src- enum variable of uart IRQ source, 'UARTRXIRQ' or 'UARTTXIRQ'
    *
    */
    #[inline(always)]
    pub fn uart_irqsource_get() -> u8 {
        let irq_s = read_reg_dma_rx_rdy0();
        write_reg_dma_rx_rdy0(irq_s);//CLR irq source
        return irq_s & UARTIRQMASK::ALL as u8;
    }

    #[inline(always)]
    pub fn uart_clr_tx_busy_flag(&mut self) {
        self.uart_tx_busy_flag = false;
    }

    #[inline(always)]
    fn uart_tx_is_busy(&self) -> bool {
        return self.uart_tx_busy_flag;
    }

    /********************************************************************************
    *	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start
    *			the DMA send function
    *
    *	@param	sendBuff - send data buffer
    *
    *	@return	'1' send success; '0' DMA busy
    */

    pub async fn uart_send_async(&mut self, msg: &uart_data_t) -> bool {
        let t_timeout = clock_time();
        while self.uart_tx_is_busy() && !clock_time_exceed(t_timeout, 400*1000) {
            wd_clear();

            yield_now().await;
        }

        self.uart_set_tx_busy_flag();
        self.txdata_buf = *msg;

        write_reg_dma1_addr(addr_of!(self.txdata_buf) as u16); // packet data, start address is sendBuff+1

        // STARTTX;
        write_reg_dma_tx_rdy0(FLD_DMA::ETH_TX as u8); //trigger dma

        return true;
    }

    pub fn uart_send(&mut self, msg: &uart_data_t) -> bool {
        let t_timeout = clock_time();
        while self.uart_tx_is_busy() && !clock_time_exceed(t_timeout, 400*1000) {
            wd_clear();

            app().uart_manager.check_irq();
        }

        self.uart_set_tx_busy_flag();
        self.txdata_buf = *msg;

        write_reg_dma1_addr(addr_of!(self.txdata_buf) as u16); // packet data, start address is sendBuff+1

        // STARTTX;
        write_reg_dma_tx_rdy0(FLD_DMA::ETH_TX as u8); //trigger dma

        return true;
    }

    /****************************************************************************************
    *
    *	@brief	data receive buffer initiate function. DMA would move received uart data to the address space, uart packet length
    *			needs to be no larger than (recBuffLen - 4).
    *
    *	@param	*recAddr:	receive buffer's address info.
    *			recBuffLen:	receive buffer's length, the maximum uart packet length should be smaller than (recBuffLen - 4)
    *
    *	@return	none
    */

    fn uart_buff_init(&mut self){
        let buf_len = size_of::<uart_data_t>() / 16;
        write_reg_dma0_addr(addr_of!(self.rxdata_buf) as u16);  //set receive buffer address
        write_reg_dma0_ctrl(buf_len as u16);    //set receive buffer size
    }

    fn uart_set_tx_busy_flag(&mut self) {
        self.uart_tx_busy_flag = true;
    }

    pub fn uart_error_clr() -> bool {
        let state = read_reg8(0x80009d);
        if state & 0x80 != 0 {
            write_reg8(0x80009d, state | 0x40);
            return true;
        }
        return false;
    }
}
