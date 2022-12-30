/********************************************************************************************************
 * @file     master_dongle.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/light_ll/light_ll.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "../../proj_lib/ble_ll/service.h"
#include "../common/rtc.h"
#include "../../proj/drivers/flash.h"
#include "../common/common.h"
#include "../common/scene.h"
#include "../../proj_lib/ble_ll/blueLight.h"
#include "../../proj_lib/ble_ll/att.h"
#include "../common/crc.h"
#include "master_dongle.h"

FLASH_ADDRESS_EXTERN;

#if MODE_MASTER_SLAVE
//rf_packet_att_data_t	ota_buffer[8];
u8 *	p_firmware = 0;
int		host_ota_start = 0;
u8		ota_hl = 0;

#define EP_BO                   5
#define rega_mode			    0x3a

const	u8	mac_default[] = {0, 0, 0, 0, 0, 0};
int					conn_interval = 32;				// 32 * 1.25 = 40 ms
int					conn_timeout = 2000;			// 2 second
int					conn_start = 1;
rf_custom_master_dat_t 	rf_custom_dat;

int host_write_master (int n)
{
	static u32 no_cmd;
	no_cmd++;

	u8 cmd = buff_command[0];

	if (cmd == 0 && n > 0)		// data
	{
		ble_master_ll_data (buff_command + 1, n - 1);
	}
	else if (cmd == 1)	// set mac
	{
		ble_master_set_slave_mac (buff_command + 1);
	}
	else if (cmd == 2)	// start master
	{
		conn_start = 1;
	}
	else if (cmd == 3)	// stop master
	{
		conn_start = 0;
		ble_master_stop ();
	}
	else if (cmd == 4)	// set connection parameters
	{
		ble_set_debug_adv_channel (buff_command[1]);
		//ble_master_set_channel_mask (buff_command + 6);
		conn_interval = buff_command[2] + buff_command[3]*256;
		conn_timeout = buff_command[4] + buff_command[5]*256;
		ble_master_update_adv (buff_command + 2);
	}
	else if (cmd == 5)	// set ota handle and start
	{
		host_ota_start = 1;
		ota_hl = buff_command[1] ? buff_command[1] : 0x18;
	}
	else if (cmd == 6)	//
	{
		//master_smp_func_init ();  // skip to save RAM
	}
	else if (cmd == 7)	// set ota handle and start
	{
		host_ota_start = 1;
		ota_hl = 0x0a;
	}

	return 0;
}

extern u8 master_push_fifo (u8 *p);
extern u8 master_fifo_ready();
void proc_ota ()
{
	if (! ble_master_status ())
	{
		host_ota_start = 0;
	}
	//ble_master_add_tx_packet;
	static u32 n_firmware = 0;
	static u32 ota_adr = 0;
	static u32	tick_page;
	static u32	tick_10s;
	if (host_ota_start == 1)
	{
		n_firmware = *(u32 *)(flash_adr_ota_master+0x18);
		if(n_firmware > 0x30000){
            host_ota_start = 0;
            ble_event_callback (FLG_SYS_LINK_LOST, 0, 0);
            return ;
		}else{
    		host_ota_start = 2;
    		ota_adr = 0;
    		tick_page = clock_time ();
    		tick_10s = 0;
		}
	}
	else if (host_ota_start == 2)
	{
		if(clock_time_exceed(tick_page, 10*1000*1000)){
			tick_page = clock_time ();
			tick_10s ++;
			if (tick_10s > 100)
			{
				host_ota_start = 0;
				return;
			}
		}
		int idx = (ota_adr >> 4) & 7;

		// use share memory with slave mode
		extern u32			buff_response[48][9];
		rf_packet_att_data_t *p = (rf_packet_att_data_t *)buff_response[idx];//&ota_buffer[idx];

		int nlen = ota_adr < n_firmware ? 16 : 0;

		p->type = 2;
		p->l2cap = 7 + nlen;
		p->chanid = 0x04;
		p->att = ATT_OP_WRITE_CMD;
		p->hl = ota_hl;
		p->hh = 0x0;
		p->dat[0] = ota_adr>>4;
		p->dat[1] = ota_adr>>12;
		if(nlen == 16){
			memcpy(p->dat + 2, p_firmware + ota_adr, 16);
		}else{
			memset(p->dat + 2, 0, 16);
		}
		u16 crc = crc16(p->dat, 2+nlen);
		p->dat[nlen + 2] = crc;
		p->dat[nlen + 3] = crc >> 8;
		p->rf_len = p->l2cap + 4;
		p->dma_len = p->l2cap + 6;
		
		if (master_fifo_ready()){
			if (master_push_fifo((u8 *)p))
			{
	            if(0 == ota_adr){
	                sleep_us(300*1000);     // wait for slave to unprotect flash.
	            }
			
				ota_adr += 16;
				if (nlen == 0)
				{
					host_ota_start = 0;
				}
			}
		}
	}
}

extern u8 pair_login_ok;

int	rf_link_slave_data_ota(void *ph);

enum{
    CMD_UART_OTA_FW_VERSION = 0xff00,
	CMD_UART_OTA_START      = 0xff01,
	CMD_UART_OTA_END        = 0xff02,
	CMD_UART_OTA_CODE       = 0xff03,
};

typedef struct{
    u8  len;
    u16 sno;
    u8	code[16];    //size must 16 
    u16 crc;
}uart_pkt_ota_t;

uart_pkt_ota_t uart_pkt_ota_pc;
u8  uart_ota_pc_tx_busy;

#define     UART_OTA_RX_EN              0   // only 8267 can enable now

#define     UART_OTA_PC_TEST_EN         0
#if UART_OTA_PC_TEST_EN   // for PC demo
u8  uart_ota_pc_start = 0;

#define CMD_UART_OTA_START_LEN  (sizeof(uart_pkt_ota_pc.sno))
#define CMD_UART_OTA_END_LEN    (sizeof(uart_pkt_ota_pc.sno)+sizeof(uart_pkt_ota_pc.crc))
#define CMD_UART_OTA_CODE_LEN   (sizeof(uart_pkt_ota_t)- sizeof(uart_pkt_ota_pc.len))
#define UART_CODE_LEN           (sizeof(uart_pkt_ota_pc.code))

STATIC_ASSERT(UART_CODE_LEN == 16);

void proc_uart_ota_pc()  // demo for PC
{
    static u32 n_firmware = 0;
    static u32 ota_adr = 0;
    if(!uart_ota_pc_start || uart_ota_pc_tx_busy){
        return ;
    }
    
    uart_ota_pc_tx_busy = 1;
    uart_pkt_ota_t *p = &uart_pkt_ota_pc;
    if(1 == uart_ota_pc_start){
        n_firmware = *(u32 *)(flash_adr_ota_master+0x18);
        ota_adr = 0;
        uart_ota_pc_start = 2;
        p->len = CMD_UART_OTA_START_LEN;
        p->sno = CMD_UART_OTA_START;
    }else if(2 == uart_ota_pc_start){
        int nlen = ota_adr < n_firmware ? UART_CODE_LEN : 0;

    	if(nlen == UART_CODE_LEN){
    		memcpy(p->code, p_firmware + ota_adr, UART_CODE_LEN);
            p->len = CMD_UART_OTA_CODE_LEN;
    	}else{  // nlen == 0
    		memset(p->code, 0, UART_CODE_LEN);
            p->len = CMD_UART_OTA_END_LEN;
    	}
        p->sno = ota_adr / UART_CODE_LEN;
		u16 crc = crc16((u8 *)&p->sno, sizeof(uart_pkt_ota_pc.sno)+nlen);
	    p->code[0+nlen] = (u8)crc;
	    p->code[1+nlen] = crc >> 8;
	    
        if(0 == ota_adr){
            sleep_us(300*1000);     // wait for slave to unprotect flash.
        }
	
		ota_adr += UART_CODE_LEN;
		if (nlen == 0){
			uart_ota_pc_start = 0;
		}    	
    }        
}
#endif

#if UART_OTA_RX_EN
u8  uart_ota_rx_start = 0;
u32 uart_ota_rx_start_tick = 0;
uart_pkt_ota_t uart_pkt_ota_rx;
rf_packet_att_data_t    uart2pkt_att;

void proc_uart_ota_rx ()
{
    if(uart_ota_pc_tx_busy){
	    rf_packet_att_data_t *p_pkt_att = &uart2pkt_att;
        memcpy(&uart_pkt_ota_rx, &uart_pkt_ota_pc, sizeof(uart_pkt_ota_rx));
        memcpy(p_pkt_att->dat, &uart_pkt_ota_rx.sno, uart_pkt_ota_rx.len);
        uart_ota_pc_tx_busy = 0;
        
	    u16 sno2cmd =  p_pkt_att->dat[0] | (p_pkt_att->dat[1]<<8);
    	if(sno2cmd == CMD_UART_OTA_FW_VERSION){
    	    // reserve
    	}else if(sno2cmd == CMD_UART_OTA_START){
            pair_login_ok = 1;
            uart_ota_rx_start = 1;
            uart_ota_rx_start_tick = clock_time();
    	    // flash erase
    	}else if(sno2cmd == CMD_UART_OTA_END){
    	    // reserve
    	}else{              // CODE
        	if (uart_ota_rx_start){
            	p_pkt_att->type = 2;
            	p_pkt_att->l2cap = 3 + uart_pkt_ota_rx.len;
            	p_pkt_att->chanid = 0x04;
            	p_pkt_att->att = ATT_OP_WRITE_CMD;
            	p_pkt_att->hl = 0x18;
            	p_pkt_att->hh = 0x0;
            	p_pkt_att->rf_len = p_pkt_att->l2cap + 4;
            	p_pkt_att->dma_len = p_pkt_att->l2cap + 6;
                
                rf_link_slave_data_ota(p_pkt_att);
            }
    	}
	}
}
#endif

void switch_mode_master2slave(u8 m_master)
{
	gpio_setup_up_down_resistor(GPIO_DP, PM_PIN_PULLDOWN_100K);
    if(m_master){
        analog_write (rega_mode, SAVE_MODE_MASTER);
    }else{
        analog_write (rega_mode, SAVE_MODE_SLAVE);
    }
    light_sw_reboot();
}

void proc_ui (void)
{
	if (!clock_time_exceed (0, 200000))
	{
		return;
	}

	u8 b_sel = !gpio_read (GPIO_BUTTON_MASTER);
	u8 b_ok = !gpio_read (GPIO_BUTTON_SLAVE);
	if (!b_sel && !b_ok)
	{
		return;
	}

	if(b_sel && !mode_master){
	    switch_mode_master2slave(1);
	}else if(b_ok && mode_master){
	    switch_mode_master2slave(0);
	}
}

void master_dongle_main_loop()
{
	proc_host ();

	if (! ble_master_status () && conn_start)		// APP(master) emulation
	{
		ble_master_start (
				conn_interval,					//6 * 1.25 = 7.5 ms interval
				conn_timeout,					// 1 second time out
				&rf_custom_dat,
				CLOCK_SYS_CLOCK_HZ/1000000		// 32M: 32 tick per us
				);
	}

    proc_ota ();                // update firmware  to other slave lights.
	if(!host_ota_start){
	    #if UART_OTA_PC_TEST_EN
        proc_uart_ota_pc();     // update firmware to itself (local) from local for test.
        #endif
        #if UART_OTA_RX_EN
        proc_uart_ota_rx();     // update firmware to itself (local) from UART.
        #endif
    }
}

void master_dongle_button_init()
{
    gpio_write(GPIO_BUTTON_MASTER, 1);        // pull up
    gpio_write(GPIO_BUTTON_SLAVE, 1);        // pull up
}

u8 master_dongle_get_mode()
{
    u8 mode = analog_read (rega_mode);
    if(SAVE_MODE_MASTER == mode){
	    mode = 1;			//  1: master 0: slave
	}else if(SAVE_MODE_SLAVE == mode){
        mode = 0;
	}else{
	    mode = 0;	// default is slave(gateway) when power on
	}

    return mode;
}

void master_dongle_user_init()
{
	blc_readFlashSize_autoConfigCustomFlashSector();
	usb_log_init ();
    host_init();

    extern void ble_master_init ();
	ble_master_init ();
	//master_smp_func_init ();

	/////////////// setup LED /////////////////////////////////////////
	//gpio_set_func (GPIO_LED, AS_GPIO);
	#if (MCU_CORE_TYPE == MCU_CORE_8267)
	REG_ADDR8(0x5b1) = 0x0;     // set default  function Mux to PWM
	REG_ADDR8(0x5b4) |= 0x3;    // set default  function Mux to PWM for PE0/PE1
	#endif

	rf_set_power_level_index (RF_POWER_8dBm);

	//ble_set_debug_adv_channel (38);
	p_firmware = (u8 *)flash_adr_ota_master;

	ble_master_set_slave_mac (mac_default);
	ble_master_set_golden ();

#if(STACK_CHECK_ENABLE)
    stack_check_init();
#endif

}
#endif
