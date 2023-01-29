use std::cmp::min;
use std::mem::size_of;
use std::ptr::addr_of;
use crate::{BIT, blinken};
use crate::embassy::yield_now::yield_now;
use crate::sdk::mcu::clock::{CLOCK_SYS_CLOCK_HZ, clock_time, clock_time_exceed};
use crate::sdk::mcu::gpio::{AS_UART, GPIO_PIN_TYPE, gpio_set_func};
use crate::sdk::mcu::register::{read_reg8, read_reg_dma_rx_rdy0, write_reg16, write_reg8, write_reg_dma0_addr, write_reg_dma0_ctrl, write_reg_dma_rx_rdy0};
use crate::sdk::mcu::watchdog::wd_clear;

const UART_DATA_LEN: usize = 44;      // data max 252

enum UARTIRQMASK {
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

#[derive(Clone)]
// This struct must be a multiple of 16 bytes in size
pub struct uart_data_t {
    pub len: u32,        // data max 252
    pub data: [u8; UART_DATA_LEN]
}


pub struct UartManager {
    pub txdata_user: uart_data_t,
    txdata_buf: uart_data_t, // not for user

    pub rxdata_user: uart_data_t,
    // data max 252, user must copy rxdata to other Ram,but not use directly
    rxdata_buf: uart_data_t,

    pub uart_rx_true: bool,
    uart_tx_busy_flag: bool,
    uart_continue_delay_time: u32,
    baudrate_set: u32
}

impl UartManager {
    pub const fn default() -> Self {
        Self {
            txdata_user: uart_data_t{len: 0, data: [0; UART_DATA_LEN]},
            txdata_buf: uart_data_t{len: 0, data: [0; UART_DATA_LEN]}, // not for user

            rxdata_user: uart_data_t{len: 0, data: [0; UART_DATA_LEN]},
            // data max 252, user must copy rxdata to other Ram,but not use directly
            rxdata_buf: uart_data_t{len: 0, data: [0; UART_DATA_LEN]},

            uart_rx_true: false,
            uart_tx_busy_flag: false,
            uart_continue_delay_time: 0,
            baudrate_set: 0
        }
    }

    pub fn init(&mut self) {
        gpio_set_func(GPIO_PIN_TYPE::GPIO_UTX as u32, AS_UART);
        gpio_set_func(GPIO_PIN_TYPE::GPIO_URX as u32, AS_UART);

        // CLK32M_UART115200
        self.uart_init(69, 3, true, true, HARDWARECONTROL::NOCONTROL);

        self.uart_buff_init();
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
        self.baudrate_set = baudrate;

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
            write_reg8(0x800521, read_reg8(0x800521) | 0x01); //open dma1 interrupt mask
            write_reg8(0x800640, read_reg8(0x800640) | 0x10); //open dma interrupt mask
            //irq_enable(); // write_reg8(0x800643,0x01);//enable intterupt
        }
        if en_tx_irq {
            write_reg8(0x800521, read_reg8(0x800521) | 0x02); //open dma1 interrupt mask
            write_reg8(0x800640, read_reg8(0x800640) | 0x10); //open dma interrupt mask
            //irq_enable(); // write_reg8(0x800643,0x01);//enable intterupt
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
    fn uart_irqsource_get() -> u8 {
        let irq_s = read_reg_dma_rx_rdy0();
        write_reg_dma_rx_rdy0(irq_s);//CLR irq source
        return irq_s & UARTIRQMASK::ALL as u8;
    }

    pub fn check_irq(&mut self) {
        let irq_s = Self::uart_irqsource_get();
        if irq_s & UARTIRQMASK::RX as u8 != 0 {
            self.uart_rx_true = true;
        }

        if irq_s & UARTIRQMASK::TX as u8 != 0 {
            self.uart_clr_tx_busy_flag();
        }
    }

    fn uart_clr_tx_busy_flag(&mut self) {
        self.uart_tx_busy_flag = false;
        self.uart_continue_delay_time = 0;
    }

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

    pub async fn uart_send_async(&mut self) -> bool {
        let t_timeout = clock_time();
        while self.uart_tx_is_busy() && !clock_time_exceed(t_timeout, 400*1000) {
            wd_clear();

            yield_now().await;

            if self.uart_continue_delay_time != 0 && clock_time_exceed(self.uart_continue_delay_time, if self.baudrate_set > 100000 { 800 } else { 9000 }) {
                self.uart_continue_delay_time = 0;    // minimum delay :  115200 delay 600us;  9600 delay 7200us
                self.uart_tx_busy_flag = false;
            }
        }

        self.uart_set_tx_busy_flag();
        self.txdata_buf = self.txdata_user.clone();
        write_reg16(0x800504,addr_of!(self.txdata_buf) as u16); // packet data, start address is sendBuff+1

        // STARTTX;
        write_reg8(0x800524,0x02); //trigger dma

        return true;
    }

    pub fn uart_send(&mut self) -> bool {
        let t_timeout = clock_time();
        while self.uart_tx_is_busy() && !clock_time_exceed(t_timeout, 400*1000) {
            wd_clear();

            if self.uart_continue_delay_time != 0 && clock_time_exceed(self.uart_continue_delay_time, if self.baudrate_set > 100000 { 800 } else { 9000 }) {
                self.uart_continue_delay_time = 0;    // minimum delay :  115200 delay 600us;  9600 delay 7200us
                self.uart_tx_busy_flag = false;
            }
        }

        self.uart_set_tx_busy_flag();
        self.txdata_buf = self.txdata_user.clone();
        write_reg16(0x800504,addr_of!(self.txdata_buf) as u16); // packet data, start address is sendBuff+1

        // STARTTX;
        write_reg8(0x800524,0x02); //trigger dma

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
        self.uart_continue_delay_time = 0;
    }

    pub fn uart_error_clr() -> bool {
        let state = read_reg8(0x80009d);
        if state & 0x80 != 0 {
            write_reg8(0x80009d, state | 0x40);
            return true;
        }
        return false;
    }

    pub fn data_ready(&mut self) -> u8 {
        if self.uart_rx_true {
            self.uart_rx_true = false;
            self.rxdata_user = self.rxdata_buf.clone();

            return 1
        }

        return 0
    }

    pub async fn printf(&mut self, msg: &str) {
        let len = min(43, msg.len());
        self.txdata_user.len = len as u32 + 1;
        self.txdata_user.data[0..len].copy_from_slice(&msg.as_bytes()[0..len]);
        self.txdata_user.data[len] = '\n' as u8;
        self.uart_send_async().await;
    }
}
