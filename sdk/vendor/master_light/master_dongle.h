/********************************************************************************************************
 * @file     master_dongle.h 
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

extern int	ble_master_ll_data (u8 *p, int n);
extern int ble_master_add_tx_packet (u32 p);
extern int     ble_master_status ();
extern void ble_master_set_slave_mac (const u8 *p);
extern void	ble_master_stop ();
extern void ble_set_debug_adv_channel (u8 chn);
extern void	ble_master_update_adv (u8 * p);
extern void		master_smp_func_init ();
extern void irq_ble_master_handler(void);
extern int	ble_master_start (u32 n_1p25ms, u32 timeout, void *p_data, int tick);

void proc_ui (void);
void master_dongle_main_loop();
void master_dongle_button_init();
u8 master_dongle_get_mode();
void master_dongle_user_init();
void	ble_master_set_golden ();


extern u8	buff_command[64];

enum{
    SAVE_MODE_MASTER = 1,
    SAVE_MODE_SLAVE  = 2,
};

