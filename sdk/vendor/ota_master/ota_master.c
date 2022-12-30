/********************************************************************************************************
 * @file     ota_master.c 
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
#include "../../proj/drivers/keyboard.h"
#include "../../vendor/common/crc.h"

#define				OTA_PKT_BUFF_SIZE				64
__attribute__((aligned(4))) unsigned char  		ota_rx_buff[OTA_PKT_BUFF_SIZE*4] = {0};
int					ota_rx_wptr;

#define			ACCESS_CODE_PAIRING				0x95d6d695

//////////////////// rf packets //////////////////////////////

rf_packet_ll_rc_data_t	pkt_ota_command = {
		sizeof (rf_packet_ll_rc_data_t) - 4,	// dma_len
		FLG_BLE_LIGHT_DATA,						// type
		sizeof (rf_packet_ll_rc_data_t) - 6,	// rf_len
		FLG_RF_RC_DATA, 						// u16

		0xa0b1c2d3,								// source id

		0x12, 0x0, 0x0, 0x0,					// rsv[3], sno
		0x0, 0x0,									// , ttc
		0xffff,									// group
		0x0,									// dest id
		{LGT_CMD_PAIRING_LIGHT_OTA_EN},
};

rf_packet_ll_ota_data_t pkt_ota_dat = {
		sizeof (rf_packet_ll_ota_data_t) - 4,	// dma_len
		FLG_BLE_LIGHT_DATA,						// type
		sizeof (rf_packet_ll_ota_data_t) - 6,	// rf_len
		FLG_RF_OTA_DATA, 						// u16
		0xa0a0a0a0,								// adr
};

u8 *	p_response = 0;

FLASH_ADDRESS_EXTERN;

u8 *	p_firmware = 0;
int		n_firmware = 0;

//////////////////////////////////////////////////////////
// debug mode
//////////////////////////////////////////////////////////
void proc_power_level ()
{
	u8	cmd = read_reg8 (0x800004);
	if ((cmd & 0xf0) == 0xe0) {
		rf_set_power_level_index (cmd & 15);
		write_reg8 (0x800004, cmd & 0x1f);
	}
}

void rc_led_en (int en, int fre)
{
	if (en)
	{
		pwm_set (PWMID_LED, 0x3ff * fre, 0x200 * fre);
		gpio_set_func (GPIO_LED, !AS_GPIO);
	}
	else
	{
		//pwm_set (PWMID_LED, 0x4000 * 2, 0x2000);
		gpio_set_func (GPIO_LED, AS_GPIO);
	}
}

/////////////////// rf interrupt handler ///////////////////////////
_attribute_ram_code_ void irq_ota_master_rx(void)
{
	static u32 irq_ota_rx_no = 0;
	//log_event (TR_T_rf_irq_rx);

	u8 * raw_pkt = (u8 *) (ota_rx_buff + ota_rx_wptr * OTA_PKT_BUFF_SIZE);
	ota_rx_wptr = (ota_rx_wptr + 1) & 3;
	reg_dma_rf_rx_addr = (u16)(u32) (ota_rx_buff + ota_rx_wptr * OTA_PKT_BUFF_SIZE); //set next buffer
	reg_rf_irq_status = FLD_RF_IRQ_RX;

	if	( raw_pkt[0] >= 15 && RF_PACKET_LENGTH_OK(raw_pkt) && RF_PACKET_CRC_OK(raw_pkt) )
	{
		rf_packet_ll_rc_data_t * p = (rf_packet_ll_rc_data_t *) (raw_pkt + 8);
		if (p->flag == FLG_RF_OTA_DATA)
		{
			////// prepare data ///////////////////
			rf_set_tx_rx_off ();
			sleep_us (10);
			if (p->src_id > n_firmware || p->src_id > 0xfc00)
			{
				pkt_ota_dat.adr = U32_MAX;
				rc_led_en (1, 1);
			}
			else
			{
				memcpy (pkt_ota_dat.dat, p_firmware + p->src_id, 32);
				pkt_ota_dat.adr = p->src_id;
			}
			pkt_ota_dat.crc = crc16 ((unsigned char *)(&pkt_ota_dat.adr), 36);

			rf_start_stx ((void *)&pkt_ota_dat, clock_time() + 40 * 32);
			sleep_us (600);
			rf_set_rxmode ();
		}
		else if ((p->type & 0x3)  == FLG_BLE_LIGHT_DATA_RSP)
		{
			irq_ota_rx_no++;
			rf_packet_ll_rc_data_t *p_rsp = (rf_packet_ll_rc_data_t *) p;
			if (p_rsp->cmd[0] == LGT_CMD_PAIRING_LIGHT_RESPONSE)
			{
				p_response = (u8 *)p;
				pkt_ota_command.dst_id = p_rsp->src_id;
			}

		}
	}
	raw_pkt[0] = 1;
}

_attribute_ram_code_ void irq_ota_master_tx(void)
{
	static u32 no_tx_data;
	no_tx_data++;
	reg_rf_irq_status = FLD_RF_IRQ_TX;
}

_attribute_ram_code_ void irq_handler(void)
{
	u16  src_rf = reg_rf_irq_status;
	if(src_rf & FLD_RF_IRQ_RX){
		irq_ota_master_rx();
	}

	if(src_rf & FLD_RF_IRQ_TX){
		irq_ota_master_tx();
	}

}

/////////////////////////////////////////////////////////////
//	work with PC software
/////////////////////////////////////////////////////////////
u32			timeout_us;
int			stop_on_response = 0;
u8			ota_command_busy = 0;
u32			tick_timeout;
int			ota_command_rsp = 0;
int			ota_download_start = 0;

void		set_ota_command (u8 cmd)
{
	ota_command_busy = pkt_ota_command.cmd[0] = cmd;
	pkt_ota_command.sno ++;
	stop_on_response = 0;
	tick_timeout = clock_time ();

	if (cmd == LGT_CMD_PAIRING_LIGHT_OFF)
	{
		rc_led_en (1, 0x40);
		timeout_us = 20000000;
	}
	else if (cmd == LGT_CMD_PAIRING_LIGHT_ON)
	{
		rc_led_en (0, 0x40);
		timeout_us = 400000;
	}
	else if (cmd == LGT_CMD_PAIRING_LIGHT_SEL)
	{
		rc_led_en (1, 0x20);
		timeout_us = 2000000;
		stop_on_response = 1;
		pkt_ota_command.dst_id = 0;
	}
	else if (ota_command_rsp && cmd == LGT_CMD_PAIRING_LIGHT_OTA_EN)
	{
		rc_led_en (1, 0x10);
		timeout_us = 2000000;
		stop_on_response = 1;
	}
	else
	{
		rc_led_en (0, 0x40);
		timeout_us = 0;
	}
	ota_command_rsp = 0;
}

void proc_ota_command ()
{
	static u32		dbg_cmd;
	static	u32		tick_cmd;
	const u8		chn[4] = {2, 12, 23, 34};	//8, 30, 52, 74

	if (ota_command_busy && clock_time_exceed (tick_timeout, timeout_us))
	{
		ota_command_busy = 0;
		set_ota_command (0);
	}
	if (!ota_command_busy || !clock_time_exceed (tick_cmd, 25000))
	{
		return;
	}
	tick_cmd = clock_time ();

	dbg_cmd++;
	u32 ac = ACCESS_CODE_PAIRING;
	for (int i=0; i<4; i++)
	{
		rf_set_ble_access_code ((u8 *)&ac);
		rf_set_ble_crc_adv ();
		rf_set_ble_channel (chn[i]);//listen channel: 4
		//rf_set_ble_channel (chn[0]);//listen channel: 4

		rf_start_stx (&pkt_ota_command, clock_time () + 100 * 32);
		u32 t = clock_time ();
		sleep_us (700);

		if (stop_on_response)
		{
			rf_set_rxmode ();
			p_response = NULL;
			while (!p_response && !clock_time_exceed (t, 4000));
			rf_set_tx_rx_off ();
			if (p_response)
			{
				if (pkt_ota_command.cmd[0] == LGT_CMD_PAIRING_LIGHT_SEL)
				{
					pkt_ota_command.cmd[0] = LGT_CMD_PAIRING_LIGHT_CONFIRM;
				}
				else
				{
					if (pkt_ota_command.cmd[0] == LGT_CMD_PAIRING_LIGHT_OTA_EN)
					{
						ota_download_start = 1;
						rf_set_ble_channel (0);
						rf_set_rxmode ();
					}
					ota_command_busy = 0;
					stop_on_response = 0;
					ota_command_rsp = 1;
				}
				return;
			}
		}
	}
}

void proc_ui (void)
{
	static u8	button_sel, button_ok, button_pressed;
	static u32	tick_button, tick_pressed;
	static int	ui_state = 0;

	if (!clock_time_exceed (tick_button, 20000))
	{
		return;
	}
	tick_button = clock_time ();

	u8 b_sel = !gpio_read (GPIO_PC5) || !gpio_read(GPIO_PD5);
	u8 b_ok = !gpio_read (GPIO_PC6) || !gpio_read(GPIO_PD4);
	u8 c_sel = button_sel != b_sel;
	u8 c_ok = button_ok != b_ok;
	if (!c_sel && !c_ok && !button_pressed)
	{
		return;
	}

	if (b_sel || b_ok)
	{
		if (!button_pressed)
		{
			button_pressed = 1;
			tick_pressed = clock_time ();
		}
		if (ui_state == 0 && clock_time_exceed (tick_pressed, 1500000) && button_sel)
		{
			ui_state = 1;				// 1 长按开始慢闪
			set_ota_command (LGT_CMD_PAIRING_LIGHT_OFF);
		}
		if (clock_time_exceed (tick_pressed, 700000) && button_ok)
		{
			ui_state = 0;				// 5 取消升级
			set_ota_command (0);
		}
	}
	else if (button_pressed)			// button released
	{
		button_pressed = 0;
		if (ui_state == 1 && button_sel)
		{
			ui_state = 2;				// 2 松开，停止闪烁
			set_ota_command (LGT_CMD_PAIRING_LIGHT_ON);
		}
		else if ((ui_state >= 2 && ui_state <= 4) && button_sel)
		{
			ui_state = 3;				// 3 再按下，快闪
			set_ota_command (LGT_CMD_PAIRING_LIGHT_SEL);
		}
		else if (ui_state == 3 && button_ok)
		{
			ui_state = 4;			// 4 开始升级
			set_ota_command (LGT_CMD_PAIRING_LIGHT_OTA_EN);
		}
	}

	button_sel = b_sel;
	button_ok = b_ok;
}

u32  dbg_m_loop;

void main_loop(void)
{
	dbg_m_loop ++;

	proc_power_level ();

	proc_ui ();
#if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(1);
#endif

	proc_ota_command ();
}


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
void  user_init(void)
{
    #if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(0); //battery check must do before OTA relative operation
    #endif
	blc_readFlashSize_autoConfigCustomFlashSector();
	/////////// ID initialization for light control software //////////
	REG_ADDR8(0x74) = 0x53;
	REG_ADDR16(0x7e) = 0x08ee;
	REG_ADDR8(0x74) = 0x00;

	/////////////// setup LED /////////////////////////////////////////
	gpio_set_func (GPIO_LED, AS_GPIO);
	//reg_pwm_pol =  BIT(PWMID_LED);
	reg_pwm_clk = 255;			//clock by 256
	pwm_set (PWMID_LED, 0x2000, 0x1000);
	pwm_start (PWMID_LED);

	/////////// rc initialization /////////////////////////////////////
	rf_link_master_init ();

	/////////// enable USB device /////////////////////////////////////
	usb_dp_pullup_en (1);

	reg_dma_rf_rx_addr = (u16)(u32) (ota_rx_buff);
	reg_dma2_ctrl = FLD_DMA_WR_MEM | (OTA_PKT_BUFF_SIZE>>4);   // rf rx buffer enable & size
	reg_dma_chn_irq_msk = 0;
	reg_irq_mask |= FLD_IRQ_ZB_RT_EN;    //enable RF & timer1 interrupt
	reg_rf_irq_mask = FLD_RF_IRQ_RX | FLD_RF_IRQ_TX;

	rf_set_power_level_index (RF_POWER_8dBm);

	n_firmware = *(u32 *)0x10018;
	p_firmware = (u8 *)flash_adr_ota_master;

}
