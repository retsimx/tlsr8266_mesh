/********************************************************************************************************
 * @file     rc_keyboard.c 
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
#include "../../proj/drivers/keyboard.h"

#define				ACTIVE_INTERVAL					24000			//24ms

#define				rega_sno						0x34

void sys_enable_airmouse (int en);
void sys_enable_kb (int en);

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
		pwm_set (PWMID_LED, 0x3fff * fre, 0x2000 * fre);
		gpio_set_func (GPIO_LED, !AS_GPIO);
	}
	else
	{
		//pwm_set (PWMID_LED, 0x4000 * 2, 0x2000);
		gpio_set_func (GPIO_LED, AS_GPIO);
	}
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
int		rc_pairing_mode = 0;
int		rc_pairing_power = 0;

extern int			mouse_enable;

extern kb_data_t	kb_event;
kb_data_t			kb_event_last;

int proc_keyboard (int read)
{
	static u32		rc_long_pressed, rc_repeat_key;
	static u32		tick_key_pressed, rc_tick_active, dtick_kb;

	u32 t = clock_time ();

	kb_event.keycode[0] = 0;
	kb_event.keycode[1] = 0;

	int det_key = kb_scan_key (0, read);

	if (det_key)
	{
		memcpy (&kb_event_last, &kb_event, sizeof (kb_data_t));
	}

	///////////////////////////////////////////////////////////////////////////////////////
	//			key pressed or released
	///////////////////////////////////////////////////////////////////////////////////////
	if (det_key) 	{

		/////////////////////////// key pressed  /////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
		rc_tick_active = clock_time ();
		if (kb_event.cnt || kb_event.ctrl_key) {

			if (rc_pairing_mode)
			{
				if ((kb_event.keycode[0] == VK_UP || kb_event.keycode[0] == VK_DOWN ||
					 kb_event.keycode[0] == VK_LEFT || kb_event.keycode[0] == VK_RIGHT))
				{
					rc_pairing_mode = 0;
					rc_led_en (0, 2);
				}
			}
			else {
				if (kb_event.keycode[0] == VK_MMODE)
				{
					sys_enable_airmouse (1);
				}
				else if (kb_event.keycode[0] == VK_MIC)
				{
					sys_enable_kb (1);
				}
				else if (kb_event.keycode[0] == VK_W_BACK)
				{

				}
			}
		}

		///////////////////////////   key released  ///////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
		else {
			rc_repeat_key = 0;
			/////////////////////////// pairing /////////////////////////////////////////
			if (rc_pairing_mode == 1)
			{
				rc_pairing_mode = 2;
			}
			////////////////// end of pairing /////////////////////////////////////
			else if (!rc_long_pressed)
			{
			}
			rc_long_pressed = 0;
		}

		tick_key_pressed = clock_time ();
	}
	//////////////////////////////////////////////////////////////////////////////////////////
	//				no key event
	//////////////////////////////////////////////////////////////////////////////////////////
	else if (kb_event_last.cnt || kb_event_last.ctrl_key)
	{
		//	long pressed
		rc_tick_active = clock_time ();			//key active
		if (!rc_pairing_mode && clock_time_exceed(tick_key_pressed, 1500000))		// long pressed
		{
				if (	(kb_event_last.keycode[0] == VK_MEDIA && kb_event_last.keycode[1] == VK_VOL_DN) ||
						(kb_event_last.keycode[1] == VK_MEDIA && kb_event_last.keycode[0] == VK_VOL_DN)	)		//pairing mode with PA low
				{
					rc_pairing_mode = 1;
					rc_pairing_power = 0;
					rc_led_en (1, 2);
				}
				else if ((kb_event_last.keycode[0] == VK_MEDIA && kb_event_last.keycode[1] == VK_VOL_UP) ||
						 (kb_event_last.keycode[1] == VK_MEDIA && kb_event_last.keycode[0] == VK_VOL_UP)   )		//pairing mode with PA low
				{
					rc_pairing_mode = 1;
					rc_pairing_power = 1;
					rc_led_en (1, 2);
				}
		}
		if (clock_time_exceed (tick_key_pressed, 500000))		//repeat key mode
		{
		}

		if (!rc_long_pressed && clock_time_exceed(tick_key_pressed, 700000))
		{
			rc_long_pressed = 1;
			if (kb_event_last.keycode[0] == VK_MMODE)
			{
				sys_enable_airmouse (0);
			}	//VK_MIC
			else if (kb_event_last.keycode[0] == VK_MIC)
			{
				sys_enable_kb (0);
			}
		}

		if (clock_time_exceed (rc_tick_active, 200000))	//200 ms
		{
			rc_tick_active = clock_time ();
			det_key = 1;
			memcpy (&kb_event, &kb_event_last, sizeof (kb_data_t));
		}
	}

	/////////////////////// poll rc_repeat_key rc_pairing_mode //////////////////////
	if (rc_pairing_mode>1 && clock_time_exceed (tick_key_pressed, 5000000))
	{
		rc_pairing_mode = 0;		//time out pairing mode
		rc_led_en (0, 1);
	}

	dtick_kb = clock_time () - t;

	return det_key;
}
