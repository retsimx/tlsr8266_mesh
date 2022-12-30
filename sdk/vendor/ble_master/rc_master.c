/********************************************************************************************************
 * @file     rc_master.c 
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
#include "../../proj_lib/ble_ll/ble_ll.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "../../proj/drivers/keyboard.h"
#include "../common/rf_frame.h"
#include "../common/tl_audio.h"
#include "trace.h"
#include "../../proj/drivers/usbhw_i.h"
#include "../common/common.h"

#ifndef			DEBUG_USB_MODE
#define			DEBUG_USB_MODE					0
#endif

#define			EP_BO							5
FLASH_ADDRESS_EXTERN;

void	ble_master_update_adv (u8 * p);
void rc_led_en (int en, int fre);
void airmouse_enable(int en);
void	ble_master_set_golden ();
extern u8 master_push_fifo (u8 *p);
extern u8 master_fifo_ready();

extern u32 		master_ota_break_tick;
extern int		host_ota_start;

rf_packet_att_data_t	ota_buffer[8];
u8 *	p_firmware = 0;
int		host_ota_start = 0;
u8		ota_hl = 0;

///////////////////////////////////////////////////////////////////////////
rf_custom_master_dat_t 	rf_custom_dat;
extern kb_data_t	kb_event;
u32					tick_kb;
int					sys_kb = 0;
int					sys_am = 0;
u8					mouse_button = 0;

int					conn_interval = 24;				// 6 * 1.25 = 7.5 ms
int					conn_timeout = 1000;			// 1 second
int					conn_start = 1;

/////////////////// rf interrupt handler ///////////////////////////
void irq_timer0(void){
	static u8 a_irq_timer0;
	a_irq_timer0++;
}

void irq_timer1(void){
	static u8 a_irq_timer1;
	a_irq_timer1++;

#if UART_ENABLE
    uart_ErrorCLR();
#endif
}

void gpio_irq_user_handle(void){
	return;
}

void gpio_risc0_user_handle(void){
	return;
}

void gpio_risc1_user_handle(void){
	return;
}

void gpio_risc2_user_handle(void){
	return;
}

_attribute_ram_code_ void irq_handler(void)
{
	irq_ble_master_handler ();
}

#define NOTIFY_RSP_BUF_NUM		64
u8 notify_rsp_buf[NOTIFY_RSP_BUF_NUM][48];
u8 notify_rsp_buf_num = NOTIFY_RSP_BUF_NUM;
u8 notify_rsp_buf_rptr = 0;
u8 notify_rsp_buf_wptr = 0;
static u32 notify_rsp_tick;

void notify_rsp_buf_init()
{
    memset4(notify_rsp_buf, 0, sizeof(notify_rsp_buf));
    notify_rsp_buf_wptr = notify_rsp_buf_rptr = 0;
}

_attribute_ram_code_ void notify_rsp_add2buf(u8 status, u8 *p, u8 rssi)
{
	u8 r = irq_disable();
	if ((((notify_rsp_buf_wptr+1) % notify_rsp_buf_num) != notify_rsp_buf_rptr))
	{
		u8 *pd = notify_rsp_buf[(notify_rsp_buf_wptr % notify_rsp_buf_num)];
		pd[0] = status;
		if(status != FLG_SYS_LINK_NOTIFY_DATA){
			if((status == FLG_SYS_DEVICE_FOUND) || (status == FLG_SYS_BEACON_FOUND)){
				u8 res_num = (u8)(notify_rsp_buf_wptr - notify_rsp_buf_rptr) % notify_rsp_buf_num;
				for(u8 i=0; i<res_num; i++){
					u8 buf_index = (notify_rsp_buf_rptr + i) % notify_rsp_buf_num;
					if((status == notify_rsp_buf[buf_index][0])&&(notify_rsp_buf[buf_index][1] == p[0]+8) && !memcmp(&notify_rsp_buf[buf_index][3], p+1, 6)){// compare type length and mac
						irq_restore(r);
						return;
					}
				}
			}
			int n = 0;
			if(p){
				n = *p;
			}
			
			pd[1] = n + 8;
			pd[2] = status;
			memcpy(pd+3, p+1, n+5);
			pd[n+8] = rssi;
		}
		else{
			memcpy(pd+1, p+10, 32);
			pd[33] = p[0];  // rssi
		}
		notify_rsp_buf_wptr = (notify_rsp_buf_wptr+1) % notify_rsp_buf_num;
    }
	irq_restore(r);
}

int notify_rsp_buf2hci(u8 *data)
{
	if(data[0] != FLG_SYS_LINK_NOTIFY_DATA){
		for (int i=0; i<data[1]; i++)
		{
			reg_usb_ep8_dat = data[i+1];
		}
		reg_usb_ep8_ctrl = BIT(7);
	}
	else{
		reg_usb_ep8_dat = 35;
		reg_usb_ep8_dat = 0;
		for (int i=0; i<33; i++)
		{
			reg_usb_ep8_dat = data[i+1];
		}
		reg_usb_ep8_ctrl = BIT(7);
	}
	return 1;
}

void notify_rsp_update ()
{
    if(clock_time_exceed(notify_rsp_tick,10*1000)){
        notify_rsp_tick = clock_time();
    	if (notify_rsp_buf_wptr != notify_rsp_buf_rptr)
    	{
    		if (notify_rsp_buf2hci (notify_rsp_buf[notify_rsp_buf_rptr % notify_rsp_buf_num]))
    		{
    			u8 r = irq_disable();
                notify_rsp_buf_rptr = (notify_rsp_buf_rptr+1) % notify_rsp_buf_num;
				irq_restore(r);
    		}
    	}
	}
}

void	ble_master_data_callback (u8 *p)
{
	static	u32 bdbg_data;
	bdbg_data++;
    notify_rsp_tick = clock_time();

	//////////////////////////////////////
	int n = p[9] + 8;   // p[9]:rf_len;  8 = 2 + 3(crc) + 2(dc) + 1(rssi)
	rf_packet_att_cmd_t *p_cmd = (rf_packet_att_cmd_t *)(p+4);
	if(27 == p_cmd->rf_len && 0x1b == p_cmd->opcode){
	    static u8 notify_rsp_last[32];
	    if(memcmp(notify_rsp_last, &p_cmd->l2capLen, 27)){
	        memcpy(notify_rsp_last, &p_cmd->l2capLen,27);
	        notify_rsp_add2buf(FLG_SYS_LINK_NOTIFY_DATA, p, p[0]);
            return ;
	    }
	}
	reg_usb_ep8_dat = n;
	reg_usb_ep8_dat = 0;
	for (int i=0; i<p[9]+5; i++)
	{
		reg_usb_ep8_dat = p[10+i];
	}
	reg_usb_ep8_dat = p[0]; // rssi
	reg_usb_ep8_ctrl = BIT(7);
	///////////////////////////////////////
}

#if(__PROJECT_BLE_MASTER__)
_attribute_ram_code_
#endif
void ble_event_callback (u8 status, u8 *p, u8 rssi)
{
	static u32 bdbg_event;
	bdbg_event++;

	if(FLG_SYS_LINK_LOST == status){
		host_ota_start = 0;			// must, if not, it can not reconnect, when host_ota_start not zero. 
	}
	notify_rsp_add2buf(status, p, rssi);
}

#if 0
typedef struct{
    u8 sno[3];
    u8 src[2];
    u8 dst[2];
    u8 op;
    u16 vendor_id;
    u8 par[10];
}app_cmd_value_t1;

typedef struct{
	u32 dma_len;
	u8	type;
	u8  rf_len;
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 handle;
	u8 handle1;
	app_cmd_value_t1 app_cmd_v;
}rf_packet_ll_app_t1;

void mesh_online_st_auto_add(u8 *p_buff_command)
{
    // packet 
    rf_packet_ll_app_t1  pkt_app_data = {0};
    u8 online_st_add[10] =      {0x00,0x01,0x64,0xff,0x00,0x01,0x64,0xff,};
    u8 par_len = 10;
    u8 op = 0x52;
    u16 dst_adr = 0;
    u8 *par = online_st_add;
    memset(&pkt_app_data, 0, sizeof(pkt_app_data));
    pkt_app_data.type = 0x02;
    pkt_app_data.rf_len = 17 + par_len;
    pkt_app_data.dma_len = pkt_app_data.rf_len + 2;
    pkt_app_data.l2capLen = pkt_app_data.rf_len - 4;
    pkt_app_data.chanId = 0x04;
    pkt_app_data.opcode = op;
    pkt_app_data.handle= 0x15;
    pkt_app_data.handle1 = 0x00;
    
    u32 sno_test = clock_time();
    memcpy(pkt_app_data.app_cmd_v.sno, &sno_test, 3);
    //memcpy(pkt_app_data.app_cmd_v.src, &device_address, 2);
    memcpy(pkt_app_data.app_cmd_v.dst, &dst_adr, 2);
    pkt_app_data.app_cmd_v.op = 0xDC;//(cmd & 0x3F) | 0xC0;
    pkt_app_data.app_cmd_v.vendor_id = 0x0211;
    memcpy(pkt_app_data.app_cmd_v.par, par, par_len);
    
    mesh_pkt_t *p_pkt = (mesh_pkt_t *)&pkt_app_data;
    u8 sub_cmd = p_buff_command[1];
    int num = p_buff_command[2];
    if(0 == sub_cmd){ // offline
        memset(p_pkt->par, 0, sizeof(p_pkt->par));
    }else if(1 == sub_cmd){ // add
        memcpy(p_pkt->par, online_st_add, sizeof(p_pkt->par));
    }
    
    for(int i = 0; i < (num); i+=2){
        u32 tick_online_st = clock_time();
        int timeout_flag = 0;
        while(!timeout_flag){
            if(master_fifo_ready()){
                p_pkt->par[0] = 0x80+i;
                if(i+1 < num){
                    p_pkt->par[4] = 0x80+i+1;
                }else{
                    memset(p_pkt->par+4, 0, 4);
                }
                ble_master_ll_data (&p_pkt->type, p_pkt->rf_len + 2);
                sno_test++;
                memcpy(p_pkt->sno, &sno_test, 3);
                break;
            }else{
                timeout_flag = clock_time_exceed(tick_online_st, 1000*1000);
            }
        }
        
        if(timeout_flag){
            break;
        }
    }

}
#endif

//////////////////////////////////////////////////////////
//	USB interfuace BI/BO
//////////////////////////////////////////////////////////
u8	buff_command[64];
int host_write ()
{
	static u32 no_cmd;
	no_cmd++;
	int n = reg_usb_ep_ptr (EP_BO);
	reg_usb_ep_ptr(EP_BO) = 0;
	if(n){
		memset(buff_command, 0, sizeof(buff_command));	// init
	}
	else{
		return 0;
	}
	
	for (int i=0; i<n; i++)
	{
		buff_command[i] = reg_usb_ep_dat(EP_BO);
	}

	u8 cmd = buff_command[0];

	if (cmd == 0 && n > 0)		// data
	{
	    notify_rsp_buf_init();
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
		master_smp_func_init ();
	}
	else if (cmd == 7)	// set ota handle and start
	{
		host_ota_start = 1;
		ota_hl = 0x0a;
	}
	#if 0
	else if (cmd == 8)	// set ota handle and start
	{
        mesh_online_st_auto_add(buff_command);
	}
	#endif

	return 0;
}

void proc_host ()
{
	static u32	tick_bulk_out;
	//////////// host interface change  //////////////////////////////////
	if (reg_usb_irq & BIT(EP_BO)) {
		host_write ();
		reg_usb_irq = BIT(EP_BO);
		reg_usb_ep_ctrl(EP_BO) = BIT(0);
	}
	else if (reg_usb_ep_ctrl(EP_BO) & FLD_USB_EP_BUSY)
	{
		tick_bulk_out = clock_time ();
	}
	else if (clock_time_exceed (tick_bulk_out, 1000000))
	{
		reg_usb_ep_ctrl(EP_BO) = BIT(0);
	}

}

//////////////////////////////////////////////////////////
// debug mode
//////////////////////////////////////////////////////////
void sys_enable_led ()
{
	if (sys_kb && sys_am)
	{
		rc_led_en (1, 2);
	}
	else if (!sys_kb && !sys_am)
	{
		rc_led_en (0, 2);
	}
	else
	{
		rc_led_en (1, 4);
	}
}

void sys_enable_kb (int en)
{
	sys_kb = en;
	//gpio_write (GPIO_PC5, en);
	sys_enable_led ();
}

int debug_am;
void sys_enable_airmouse (int en)
{
	debug_am++;
	sys_am = en;
	airmouse_enable (en);
	sys_enable_led ();
}

/////////////////////////////////////////////////////////////
//	work with PC software
/////////////////////////////////////////////////////////////
void proc_suspend (void)
{
	static u32 tick_10ms;
	if (sys_kb || sys_am || !clock_time_exceed (tick_kb, 2000000))		// mic or airmouse on
	{
		while (!clock_time_exceed (tick_10ms, 10000));
		tick_10ms = clock_time ();
	}
	else
	{
		usb_dp_pullup_en (0);
		cpu_sleep_wakeup (1, PM_WAKEUP_PAD , 0) ;
	}
}

#define		reg_debug_cmd		REG_ADDR16(0x8008)
void	proc_debug () {
    return ;    // can not use 0x8004~0x800d
    
	u16	udat = reg_debug_cmd;
	u8	cmd = udat >> 8;
	u8  adr = udat;
	u8	dat;
	if (cmd == 0xff) {			//read command
		dat = analog_read (adr);
		reg_debug_cmd = dat | 0x5500;
	}
	else if (cmd <= 0x20 || (cmd>=0x80 && cmd <=0xc0)) {	//write command
		analog_write (cmd, adr);
		dat = analog_read (cmd);
		reg_debug_cmd = dat | 0x6600;
	}
	else if (cmd == 0xfe) {	//set channel mask
		//chn_mask_fix = adr;
		//reg_debug_cmd = adr | 0x6700;
	}
	else if (cmd == 0xfd) {	//set tx power
		rf_set_power_level_index (adr & 15);
		reg_debug_cmd = adr | 0x6800;
	}

}

void proc_ota ()
{
	if (! ble_master_status ())
	{
		host_ota_start = 0;
		return;
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
		if(master_ota_break_tick){
			if(clock_time_exceed(master_ota_break_tick, 1000*1000)){
				master_ota_break_tick = 0;
			}else{
				return ;
			}
		}
	
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
		rf_packet_att_data_t *p = &ota_buffer[idx];

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
			if (master_push_fifo ((u8 *)p))
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

typedef void (*fp_rf_led_ota_ok)(void);
typedef void (*fp_rf_led_ota_error)(void);

u16 		                ota_pkt_cnt = 0;
fp_rf_led_ota_ok 			p_vendor_rf_led_ota_ok 			= 0;
fp_rf_led_ota_error			p_vendor_rf_led_ota_error 		= 0;

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

int dbg_am, dbg_kb;
_attribute_ram_code_ void main_loop(void)
{
	static u32 dbg_st, dbg_m_loop;
	dbg_m_loop ++;

	//proc_debug ();

#if(STACK_CHECK_ENABLE)
    stack_check();
#endif
    notify_rsp_update ();

#if DEBUG_USB_MODE

	proc_host ();
#if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(1);
#endif

	if (! ble_master_status () && conn_start)		// APP(master) emulation
	{
		dbg_st++;

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
#else

	int det_kb = proc_keyboard (1);
	if (det_kb)
	{
		mouse_button = kb_event.cnt == 1 && kb_event.keycode[0] == VK_LEFTB;
		if (kb_event.cnt || kb_event.ctrl_key)		//non empty or release key
		{
			tick_kb = clock_time ();
		}
	}

	if (sys_am || mouse_button)
	{
		proc_mouse (mouse_button);
	}

	if ((sys_kb || sys_am) && det_kb)
	{
		dbg_kb++;
		rf_custom_dat.ctype = CUSTOM_DATA_KEYBOARD;
		memcpy (rf_custom_dat.cmd, (void *)&kb_event, sizeof(kb_data_t));
		ble_master_write (&rf_custom_dat.ctype);
	}
	else if (!ble_master_command_busy () && sys_am && km_data_get(rf_custom_dat.cmd))
	{
		dbg_am++;
		rf_custom_dat.ctype = CUSTOM_DATA_MOUSE;
		ble_master_write (&rf_custom_dat.ctype);
	}

	if (!sys_am && !sys_kb)
	{
		ble_master_stop ();
	}
	else if (! ble_master_status ())		// APP(master) emulation
	{
		dbg_st++;

		ble_master_pairing_enable (1);

		ble_master_start (
				conn_interval,					//6 * 1.25 = 7.5 ms interval
				conn_timeout,					//1 second time out
				&rf_custom_dat,
				CLOCK_SYS_CLOCK_HZ/1000000		// 32M: 32 tick per us
				);
	}

	proc_suspend ();
#endif
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
const	u8	mac_default[] = {0, 0, 0, 0, 0, 0};

void  user_init(void)
{
    #if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(0); //battery check must do before OTA relative operation
    #endif
	blc_readFlashSize_autoConfigCustomFlashSector();
	// for app ota   
    flash_get_id();

    erase_ota_data_handle();

	usb_log_init ();
	/////////// ID initialization for host control software //////////
	u16 dongle_usb_id = 0x82bd;
	if(REG_ADDR16(0x7e) == 0x5327){
		dongle_usb_id = 0x82bc; // fix write error
	}
	REG_ADDR8(0x74) = 0x53;
	REG_ADDR16(0x7e) = dongle_usb_id;
	REG_ADDR8(0x74) = 0x00;

	//enable I2C function, enable internal 10K pullup
	reg_gpio_pe_gpio &= ~ BIT(7);
	reg_gpio_pf_gpio &= ~ BIT(1);
	analog_write(20, analog_read(20) | (GPIO_PULL_UP_10K<<2) | (GPIO_PULL_UP_10K<<6));	//  CK, DI, pullup 10K
	// enable gyro power
	gpio_write(GPIO_PD7, 0);
	gpio_set_output_en(GPIO_PD7, 1);
	//airmouse_enable (1);

	/////////// rc initialization /////////////////////////////////////
	ble_master_init ();
	//master_smp_func_init ();

	/////////// enable USB device /////////////////////////////////////
	#if (MCU_CORE_TYPE == MCU_CORE_8267)
	usb_dp_pullup_en (0);    // fix 8267 A0 error:1.5K pull up
	#else
	usb_dp_pullup_en (1);
	#endif
	reg_usb_ep_ctrl(EP_BO) = BIT(0);

	/////////////// setup LED /////////////////////////////////////////
	gpio_set_func (GPIO_LED, AS_GPIO);
	#if (MCU_CORE_TYPE == MCU_CORE_8267)
	REG_ADDR8(0x5b1) = 0x0;     // set default  function Mux to PWM
	REG_ADDR8(0x5b4) |= 0x3;    // set default  function Mux to PWM for PE0/PE1
	#endif
	//reg_pwm_pol =  BIT(PWMID_LED);
	reg_pwm_clk = 255;			//clock by 256
	pwm_set (PWMID_LED, 0x2000, 0x1000);
	pwm_start (PWMID_LED);

	////////// set up wakeup source: driver pin of keyboard  //////////
	u8 pin[] = KB_DRIVE_PINS;
	for (int i=0; i<sizeof (pin); i++)
	{
		cpu_set_gpio_wakeup (pin[i], 1, 1);
	}
	////////// set up wakeup source: driver pin of keyboard  //////////
	rf_set_power_level_index (RF_POWER_8dBm);

	ble_set_debug_adv_channel (38);
	p_firmware = (u8 *)flash_adr_ota_master;

#if	DEBUG_USB_MODE
	ble_master_set_slave_mac (mac_default);
	ble_master_set_golden ();
#endif

#if(STACK_CHECK_ENABLE)
    stack_check_init();
#endif

}
