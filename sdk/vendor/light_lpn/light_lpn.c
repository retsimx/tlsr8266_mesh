/********************************************************************************************************
 * @file     light_lpn.c 
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
#include "../../proj_lib/ble_ll/service.h"
#include "../../proj/drivers/flash.h"
#include "../common/common.h"
#include "../../proj/drivers/adc.h"

#define			ACTIVE_INTERVAL				32000

FLASH_ADDRESS_EXTERN;

#define		rega_sno						0x34
#define		DEUBG_UART_HEX_EN				0

#define		DEUBG_UART_TEXT_EN				0

#if		DEUBG_UART_HEX_EN
#define		USBUART(d)		reg_usb_ep8_dat = d
#else
#define		USBUART(d)
#endif

u16 		switch_group[SW_GRP_SIZE];
const u16   switch_const_grp[SW_GRP_SIZE] = {0x8001,0x8002,0x8003,0x8004};
u16 	sw_grp_next_pos=0;


u8          key_wakeup_flag = 0;
u8			mode_config;
u8			RFtx_flag;

u16 		ota_pkt_cnt = 0;
u32 		active_time = 0;
int			led_dbg;
u32			buff_response[48][9];
u8 			write_master_data = 0;
u32			led_event_pending;
u16			cmd_delay_ms = 0;
u16			cmd_left_delay_ms = 0;
u32 		irq_timer1_cb_time = 0;
static u8 	cmd_last_c[11];
static int	led_count = 0;

fp_rf_led_ota_ok 			p_vendor_rf_led_ota_ok 			= 0;
fp_rf_led_ota_error			p_vendor_rf_led_ota_error 		= 0;
fp_irq_timer1 				p_vendor_irq_timer1 			= 0;
fp_proc_led 				p_vendor_proc_led 				= 0;
fp_rf_link_data_callback	p_vendor_rf_link_data_callback	= 0;
fp_user_init 				p_vendor_user_init				= 0;

static int	rc_repeat_key = 0;
static int	rc_key_pressed;
static int	rc_long_pressed;

extern kb_data_t	kb_event;

u8          led_onoff_st = 0;
static u8	led_lum = 100;//5~100
static u8	led_rgb = 0;//0~64/51

#define		reg_debug_cmd		REG_ADDR16(0x8008)

// send_to_master 16bytes data, will be reset after send to master
extern u8 	send_to_master[];

extern u8* 	slave_p_mac;
extern u16 	slave_group;
extern u32	slave_first_connected_tick;
extern u8 	pair_login_ok;
extern u8 	not_need_login;
extern u8 	slave_link_connected;
extern u8	rf_link_key[];
extern u8 	max_relay_num;
extern ll_adv_private_t adv_pri_data;
extern u8	adv_private_data_len;
extern ll_adv_rsp_private_t adv_rsp_pri_data;


////////////////////////////////////////////////////////////////////////////
u32 config_start_time = 0;


u16			cmd_dst = 0xffff;
u16			cmd_dst_last = 0xffff;
u8			cmd_op_para[16];
u8			cmd_sno;
u8			cmd_busy = 0;

//////////////////////////////////////////////////////
#define LED_MASK							0x07
#define	config_led_event(on,off,n,sel)		(on | (off<<8) | (n<<16) | (sel<<24))

#define	LED_EVENT_FLASH_4HZ_10S				config_led_event(2,2,40,LED_MASK)
#define	LED_EVENT_FLASH_STOP				config_led_event(1,1,1,LED_MASK)
#define	LED_EVENT_FLASH_2HZ_2S				config_led_event(4,4,4,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_2S				config_led_event(8,8,2,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_3S				config_led_event(8,8,3,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_4S				config_led_event(8,8,4,LED_MASK)
#define	LED_EVENT_FLASH_4HZ					config_led_event(2,2,0,LED_MASK)
#define	LED_EVENT_FLASH_1HZ					config_led_event(8,8,0,LED_MASK)
#define	LED_EVENT_FLASH_4HZ_1T				config_led_event(2,2,1,LED_MASK)
#define	LED_EVENT_FLASH_4HZ_3T				config_led_event(2,2,3,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_1T				config_led_event(8,8,1,LED_MASK)

extern void light_set_tick_per_us(int tick);
extern void	rf_link_slave_pairing_enable(int en);
extern void	rf_link_slave_set_buffer (u32 *p, u8 n);
extern int mesh_send_user_command ();
extern int rf_link_get_op_para(u8 *p, u8 *op, u8 *op_len, u8 *para, u8 *para_len, u8 mesh_flag);
extern void	rf_link_slave_enable (int en);
void global_reset_new_key_wakeup();

void retrieve_switch_grp_address(void){
	u16 switch_grp_tmp[SW_GRP_SIZE];
	u16 switch_grp_cmp[SW_GRP_SIZE];
	memset(switch_grp_cmp ,0xff,sizeof(switch_grp_cmp));
	
	if(*(u32*)flash_adr_alarm == 0xffffffff){ // when the sector is blank ,use the initial cfg
		memcpy(switch_group , switch_const_grp ,sizeof(switch_group));
		flash_write_page(flash_adr_alarm,sizeof(switch_group),(u8 *)switch_group);
		sw_grp_next_pos = sizeof(switch_group);
		return ;
	}
	else{	// get the value from the flash ,until the read data is blank 
		foreach(i,FLASH_SECTOR_SIZE/sizeof(switch_group)){
			flash_read_page(flash_adr_alarm+i*sizeof(switch_group),sizeof(switch_group),(u8 *)switch_grp_tmp);
			if(!memcmp(switch_grp_tmp,switch_grp_cmp,sizeof(switch_grp_cmp))){
				sw_grp_next_pos = i*sizeof(switch_group);
				break;
			}
			else{
				memcpy(switch_group,switch_grp_tmp,sizeof(switch_group));
			}
		}
	}
}

/////////////////// rf interrupt handler ///////////////////////////
_attribute_ram_code_ void irq_handler(void)
{
	irq_light_slave_handler ();
}

void irq_timer0(void){
#if IRQ_TIMER0_ENABLE
	static u8 a_irq_timer0;
	a_irq_timer0++;
#endif
}
void irq_timer1(void){
	static u8 a_irq_timer1;
	a_irq_timer1++;
	if(p_vendor_irq_timer1){
		p_vendor_irq_timer1();
		//return;
	}
}

void	cfg_led_event (u32 e)
{
	led_event_pending = e;
}

//////////////////////////////////////////////////////
void get_online_st_par(u8 *p_st_out)
{
    p_st_out[0] = led_onoff_st;
	p_st_out[1] = 0xFF;   // rsv
}

void device_status_update(){
    // packet
    u8 st_val_par[MESH_NODE_ST_PAR_LEN] = {0};
    memset(st_val_par, 0xFF, sizeof(st_val_par));
    get_online_st_par(st_val_par);
    // end
    
    ll_device_status_update(st_val_par, sizeof(st_val_par));
}

void rf_link_light_event_callback (u8 status)
{
    extern void mesh_node_init ();
	if(status == LGT_CMD_SET_MESH_INFO){
        cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
	}else if(status == LGT_CMD_SET_DEV_ADDR){
        mesh_node_init();
        device_status_update();
        cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
    }else if(status == LGT_CMD_DEL_PAIR){
        cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
    }
	else if(status == LGT_CMD_FRIEND_SHIP_OK){
		cfg_led_event(LED_EVENT_FLASH_4HZ_3T);
	}
	else if(status == LGT_CMD_FRIEND_SHIP_DISCONNECT){
		cfg_led_event(LED_EVENT_FLASH_4HZ_1T);
	}
#if(MESH_PAIR_ENABLE)
	else if(status == LGT_CMD_MESH_PAIR_TIMEOUT){
		cfg_led_event(LED_EVENT_FLASH_2HZ_2S);
	}
#endif
}

void led_onoff(u8 on){
    gpio_set_func (GPIO_LED, AS_GPIO);
    gpio_set_output_en (GPIO_LED, 1);
#if PANEL_ENABLE
    gpio_write(GPIO_LED, !on); 
#else
    gpio_write(GPIO_LED, on);
#endif
}

int is_led_busy()
{
    return (led_count || led_event_pending);
}

void proc_led()
{
	if(p_vendor_proc_led){
		p_vendor_proc_led();
		return;
	}
	static	u32 led_ton;
	static	u32 led_toff;
	static	int led_sel;						//
	static	u32 led_tick;
	static	int led_no;
	static	int led_is_on;

	if(!is_led_busy()) {
		return;  //led flash finished
	}

	if (led_event_pending)
	{
		// new event
		led_ton = (led_event_pending & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
		led_toff = ((led_event_pending>>8) & 0xff) * 64000 * CLOCK_SYS_CLOCK_1US;
		led_count = (led_event_pending>>16) & 0xff;
		led_sel = led_event_pending>>24;

		led_event_pending = 0;
		led_tick = clock_time () + 30000000 * CLOCK_SYS_CLOCK_1US;
		led_no = 0;
		led_is_on = 0;
	}

	if( 1 ){
		if( (u32)(clock_time() - led_tick) >= (led_is_on ? led_ton : led_toff) ){
			led_tick = clock_time ();
			
			led_is_on = !led_is_on;
			if (led_is_on)
			{
				led_no++;
				led_dbg++;
				if (led_no - 1 == led_count)
				{
					led_count = led_no = 0;
					return ;
				}
			}
			
			int led_off = (!led_is_on || !led_ton) && led_toff;
			int led_on = led_is_on && led_ton;
			
			if( led_off || led_on  ){
				if (led_sel & BIT(0))
				{
					led_onoff(led_on);
				}
            }
        }
	}

}

void light_event_now(u8 event)
{
    rf_link_light_event_callback(event);
#if 1
    while(is_led_busy()){
        proc_led();
        wd_clear();
    }
#endif
}

void friend_ship_establish_ok_cb_lpn()
{
    //friend_send_current_subsc_list();
    #if 1   // for test
    light_event_now(LGT_CMD_FRIEND_SHIP_OK);
    #endif
}

void friend_ship_disconnect_cb_lpn()
{
    #if 1   // for test
    light_event_now(LGT_CMD_FRIEND_SHIP_DISCONNECT);
    #endif
}

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


void proc_suspend (int en, int active)
{
    if (!en || active)
    {
    	active_time = clock_time ();
    }
    
    if (mode_config == 2)
    {
    	/*u8 r = */irq_disable();
    	mode_config++;
    	analog_write (rega_sno+6, mode_config);			//reboot with ble mode on
    	light_sw_reboot();
    }
	extern u8 mesh_user_cmd_idx;
	if(clock_time_exceed(active_time, 1000000) || ((!(!en || active)) && (RFtx_flag && (!mesh_user_cmd_idx)))){
		RFtx_flag = 0;
		u8 time_flag =  get_proc_st_lpn() ? PM_WAKEUP_TIMER : 0;
		u32 wakeup_tick = time_flag ? (clock_time() + 1000*CLOCK_SYS_CLOCK_1MS) : 0;
		u8 r = irq_disable();
#if (0 == DEEP_SLEEP_EN)   //suspend
		//cpu_sleep_wakeup(0, PM_WAKEUP_PAD | PM_WAKEUP_CORE, 0) ;
		//analog_write (0x06, 0xff);
		led_onoff(0);
		usb_dp_pullup_en (0);
		cpu_sleep_wakeup(0, PM_WAKEUP_CORE | time_flag, wakeup_tick) ;
		usb_dp_pullup_en (1);       // recover  to initial status
		//analog_write (0x06, 0x00);
		//cpu_sleep_wakeup(0, PM_WAKEUP_PAD, 0) ;
		active_time = clock_time ();
		
        key_wakeup_flag = is_wakeup_by_gpio();
		irq_restore (r);
#else
		//u8 r = irq_disable();
		// BIT(0):	0 power   1 no power
		// BIT(1):	1  timer
		// BIT(2):	1 io
#if(MCU_CORE_TYPE == MCU_CORE_8258)
		usb_dp_pullup_en (0);
        #if (!PM_DEEPSLEEP_RETENTION_ENABLE)
		analog_write (rega_sno + 6, ((led_rgb << 2) & 0xFC));
		analog_write (rega_sno+7, cmd_sno);
		#endif
		u8 wakeup_flag = BIT(7) |led_lum ;
		analog_write (rega_sno + 8, wakeup_flag);

        #if PM_DEEPSLEEP_RETENTION_ENABLE
        global_reset_new_key_wakeup();
        #endif
#else
		u8 wakeup_flag = analog_read (rega_sno + 3);
		usb_dp_pullup_en (0);
		//analog_write (rega_sno, rf_link_get_user_data_sno());
		analog_write (rega_sno, cmd_sno);
		analog_write (rega_sno+1, cmd_dst);
		analog_write (rega_sno+2, cmd_dst>>8);
		wakeup_flag |= BIT(2) | BIT(0);
		wakeup_flag &= ~ BIT(1);
		analog_write (rega_sno + 3, wakeup_flag);
		analog_write (rega_sno + 4, led_lum);
		analog_write (rega_sno + 5, led_rgb);
#endif
		cpu_sleep_wakeup(PM_DEEPSLEEP_RETENTION_ENABLE ? DEEPSLEEP_MODE_RET_SRAM : DEEPSLEEP_MODE, PM_WAKEUP_PAD | time_flag, wakeup_tick) ;
		active_time = clock_time ();
		//while(1);
		r = r;                  // clean compile warnning. will be optimized, not waste code size.
#endif
	}
	else if (en)
	{
		u8 r = irq_disable();
		// if enter retention deep, LED indication(GPIO) would not keep.
		cpu_sleep_wakeup (0, PM_WAKEUP_TIMER, clock_time () + ACTIVE_INTERVAL * CLOCK_SYS_CLOCK_1US) ;
		irq_restore (r);
	}
	else if(SW_Low_Power && mode_config && (!DEBUG_SUSPEND))
	{
		BLE_low_power_handle(mode_config, ACTIVE_INTERVAL);
	}
	else
	{
		sleep_us (ACTIVE_INTERVAL);
	}
}

void gpio_irq_user_handle(){
	return;
}

void gpio_risc0_user_handle(){
	return;
}

void gpio_risc1_user_handle(){
	return;
}
//#if(IRQ_GPIO0_ENABLE)
void gpio_risc2_user_handle(void){
#if PANEL_ENABLE
	static u8 i2c_irq_cnt;
	static u8 i2c_data[16][4] = {0};
    u8 i2c_try_cnt = 0;
	//sleep_us(10*1000);
    while(i2c_try_cnt < 200){
    	i2c_burst_read(0x5a, 0x51, i2c_data[i2c_irq_cnt], 4);
    	if((i2c_data[i2c_irq_cnt][3] == 0x00)){
            sleep_us(500);
            ++i2c_try_cnt;
    	}else{
    	    break;
    	}
	}
	static u8 i2c_data_err = 0;
	static u8 i2c_data_ok = 0;
	static u8 a_iic = 0;
	a_iic = i2c_data[i2c_irq_cnt][0];
	if(i2c_data[i2c_irq_cnt][0] + i2c_data[i2c_irq_cnt][1] + i2c_data[i2c_irq_cnt][2] == i2c_data[i2c_irq_cnt][3]){
        if(i2c_data[i2c_irq_cnt][0] > 0 && i2c_data[i2c_irq_cnt][0] < 6){
            cfg_led_event (LED_EVENT_FLASH_4HZ_1T);
        }
		++i2c_data_ok;
	}else{
		++i2c_data_err;
	}

	static u8 i2c_onoff[2] = {0};
	static u8 cnt_flag = 0;
    u8 cmd[11] = {0};
	if(i2c_data[i2c_irq_cnt][0] == 0x03){
        cmd[0] = LGT_CMD_LIGHT_ON;
        light_slave_tx_command(cmd, 0xFFFF);
		i2c_onoff[0]++;
	}else if(i2c_data[i2c_irq_cnt][0] == 0x05){
        cmd[0] = LGT_CMD_LIGHT_OFF;
        light_slave_tx_command(cmd, 0xFFFF);
		i2c_onoff[1]++;
	}
	
	if(++i2c_irq_cnt == 16){
		i2c_irq_cnt = 0;
	}

	active_time = clock_time();
	reg_irq_src = FLD_IRQ_GPIO_RISC2_EN;
#endif
}

void light_panel_config(void){
#if (PANEL_ENABLE)
	// wakeup src
	gpio_set_func(I2C_WAKUP_PIN, AS_GPIO);		//gpio
	gpio_set_output_en(I2C_WAKUP_PIN, 0);		//output disable
	gpio_write(I2C_WAKUP_PIN, 1);				//pull-up enable
	gpio_set_interrupt(I2C_WAKUP_PIN, 1);

	//enable I2C function, enable internal 10K pullup
	reg_gpio_pe_gpio &= ~ BIT(7);
	reg_gpio_pf_gpio &= ~ BIT(1);
	analog_write(20, analog_read(20) | (GPIO_PULL_UP_10K<<2) | (GPIO_PULL_UP_10K<<6));
	i2c_init();
	
    gpio_set_wakeup(I2C_WAKUP_PIN, 0, 1);
    gpio_core_wakeup_enable_all (1);
	active_time = clock_time();
	gpio_set_func(PANEL_LED, AS_GPIO);			  //gpio
	gpio_set_output_en(PANEL_LED, 1);
	gpio_write(PANEL_LED, 1);
	reg_irq_mask |= (IRQ_GPIO0_ENABLE?FLD_IRQ_GPIO_RISC2_EN:0);
#endif
}

void light_hw_timer1_config(void){
 	//enable timer1 interrupt
	reg_irq_mask |= FLD_IRQ_TMR1_EN;
	reg_tmr1_tick = 0;
	reg_tmr1_capt = CLOCK_MCU_RUN_CODE_1US * IRQ_TIME1_INTERVAL * 1000;
	reg_tmr_ctrl |= FLD_TMR1_EN;
}

int light_slave_tx_command(u8 *p_cmd, int para)
{
	static u32 dbg_sc;
	dbg_sc++;
	RFtx_flag = 1;
	memcpy(cmd_last_c, p_cmd, 11);
	cmd_dst_last = para;
	reg_usb_ep8_dat = p_cmd[0];
	reg_usb_ep8_dat = para;

	memset(cmd_op_para, 0, 11);

	cmd_op_para[0] = p_cmd[0];
    cmd_dst = para;
    
	if (p_cmd[0] == LGT_CMD_LIGHT_ON || p_cmd[0] == LGT_CMD_LIGHT_OFF)
	{
		cmd_op_para[0] = LGT_CMD_LIGHT_ONOFF;
		cmd_op_para[3] = p_cmd[0] == LGT_CMD_LIGHT_ON;
	}
	else if (p_cmd[0] == LGT_CMD_LUM_UP || p_cmd[0] == LGT_CMD_LUM_DOWN)
	{
	    if(p_cmd[0] == LGT_CMD_LUM_UP){
            led_lum = (led_lum == 5)? 10: ((led_lum >= 90)? 100 : (led_lum+10));
        }else{
            led_lum = (led_lum <= 10)? 5 : (led_lum-10);
        }
		cmd_op_para[0] = LGT_CMD_LIGHT_SET;
	    cmd_op_para[3] = led_lum;
	}
	else if (p_cmd[0] == LGT_CMD_LEFT_KEY || p_cmd[0] == LGT_CMD_RIGHT_KEY)
	{
	    if(p_cmd[0] == LGT_CMD_LEFT_KEY){
	        led_rgb++;
	        if(led_rgb >= RGB_MAP_MAX){
	            led_rgb = 0;
	        }
        }else{
	        if(led_rgb == 0){
	            led_rgb = RGB_MAP_MAX-1;
	        }else{
	            led_rgb--;
	        }
        }
		cmd_op_para[0] = LGT_CMD_LIGHT_RC_SET_RGB;
	    cmd_op_para[3] = led_rgb;
	}
	cmd_op_para[0] |= 0xc0;
	cmd_op_para[1] = VENDOR_ID & 0xFF;
	cmd_op_para[2] = VENDOR_ID >> 8;

	//rf_link_user_command((u8 *) &slave_tx_cmd);

	mesh_push_user_command((int)(cmd_sno++), cmd_dst, cmd_op_para, 11);
	//mesh_send_user_command();

	return 1;
}

void tx_command_repeat ()
{
	light_slave_tx_command (cmd_last_c, cmd_dst_last);
	reg_usb_ep8_dat = cmd_last_c[0];
	reg_usb_ep8_dat = cmd_dst_last;
}

#if 1
void cfg_sw_grp(void )
{
	if(sw_grp_next_pos > FLASH_SECTOR_SIZE-sizeof(switch_group)){	//when the write overcome  the sector 
		flash_erase_sector(flash_adr_alarm);
		sleep_us(100);
		flash_write_page(flash_adr_alarm,sizeof(switch_group),(u8 *)switch_group);
		sw_grp_next_pos = sizeof(switch_group);
	}
	else{
		flash_write_page(flash_adr_alarm+sw_grp_next_pos , sizeof(switch_group),(u8 *)switch_group);
		sw_grp_next_pos += sizeof(switch_group);
	}
}

void rf_link_data_callback (u8 *p)
{
	if(p_vendor_rf_link_data_callback){
		p_vendor_rf_link_data_callback(p);
		return;
	}

	u8 op = 0;
    u8 op_cmd[8] = {0};
    u8 op_cmd_len = 0;
    u8 params[16] = {0};
    u8 params_len = 0;
    rf_packet_att_value_t *pp = (rf_packet_att_value_t*)(((ll_packet_l2cap_data_t*)(p))->value);
    rf_link_get_op_para(p, op_cmd, &op_cmd_len, params, &params_len, 1);

    if(op_cmd_len == op_type_1){
    }else if(op_cmd_len == op_type_2){
    }else if(op_cmd_len == op_type_3){
        u16 vendor_id = op_cmd[2] << 8 | op_cmd[1];
        op = op_cmd[0] & 0x3F;
        if(vendor_id == VENDOR_ID){
    	    active_time = clock_time();
    	    #if 1
    	    if (op == LGT_CMD_CONFIG_DEV_ADDR){
        		u16 val = (params[0] | (params[1] << 8));
    			extern u8 rf_link_add_dev_addr(u16 deviceaddress);
    			if(!dev_addr_with_mac_flag(params) || dev_addr_with_mac_match(params)){
    				if(rf_link_add_dev_addr(val)){
						mesh_pair_proc_get_mac_flag();
					}
    			}
			}
            #endif

            mesh_pkt_t *pkt = CONTAINER_OF(pp->sno,mesh_pkt_t,sno);
			if(0 != lpn_rx_friendship_cmd_proc(pkt)){
			    return ;
			}
			
            if(op == LGT_CMD_LIGHT_ONOFF){
                if(led_onoff_st != params[0]){
                    fs_proc_lpn.poll_report_online_st_now = 1;
                }
                
                if(params[0] == LIGHT_ON_PARAM){
            		led_onoff(1);
            		led_onoff_st = 1;
        		}else if(params[0] == LIGHT_OFF_PARAM){
            		led_onoff(0);
            		led_onoff_st = 0;
        		}
            }
        	else if (op == LGT_CMD_SWITCH_CONFIG){
        	    cfg_led_event (config_led_event(4, 4, params[0], LED_MASK));
        	}
            else if (op == LGT_CMD_LIGHT_CONFIG_GRP){
        		u16 val = (params[1] | (params[2] << 8));
        		if(params[0] == LIGHT_DEL_GRP_PARAM){
        			extern u8 rf_link_del_group(u16 group);
        			if(rf_link_del_group(val)){
                        fs_proc_lpn.poll_type = POLL_TYPE_GROUP_OW;
                        //cfg_led_event (config_led_event(4, 4, params[0], LED_MASK));
        			}
        		}else if(params[0] == LIGHT_ADD_GRP_PARAM){
        			extern u8 rf_link_add_group(u16 group);
        			if(rf_link_add_group(val)){
                        fs_proc_lpn.poll_type = POLL_TYPE_GROUP_OW;
                        //cfg_led_event (config_led_event(4, 4, params[0], LED_MASK));
        			}
        		}
        	}else if (op == LGT_CMD_KICK_OUT){
        		//if(is_mesh_cmd_need_delay(p, params, pp->val[op_cmd_len+params_len])){
        			//return ;  // no need delay for lowpower node
        		//}
        	    irq_disable();
        	    void kick_out(u8 par);
        	    kick_out(params[0]);
        	    light_sw_reboot();
			}else if(op == LGT_CMD_SET_SW_GRP){
				if(memcmp(switch_group , params, sizeof(switch_group))){	//if the switch grp is not equal to the data ,then update
					memcpy(switch_group,params,sizeof(switch_group));
					cfg_sw_grp();
					cfg_led_event (config_led_event(4, 4, params[0], LED_MASK));
				}
				else{	
					//cfg_led_event (config_led_event(4, 4, params[0], LED_MASK));
					return ;
				}
			}
    	}
	}
}

/*@param: p: p is pointer to response
**@param: p_cmd_rx: is pointer to request command*/
int rf_link_response_callback (u8 *p, u8 *p_cmd_rx)
{
    // mac-app[5] low 2 bytes used as ttc && hop-count 
    rf_packet_att_value_t *ppp = (rf_packet_att_value_t*)(p);
    rf_packet_att_value_t *p_req = (rf_packet_att_value_t*)(p_cmd_rx);
    bool dst_unicast = is_unicast_addr(p_req->dst);
	memcpy(ppp->dst, p_req->src, 2);
	memcpy(ppp->src, &device_address, 2);
	set_sub_addr2rsp((device_addr_sub_t *)ppp->src, p_req->dst, dst_unicast);
	//memcpy(ppp->dst, (u8*)&slave_group, 2);

	u8 params[10] = {0};
	memcpy(params, ppp->val+3, sizeof(params));
	memset(ppp->val+3, 0, 10);
	    
	ppp->val[1] = VENDOR_ID & 0xFF;
	ppp->val[2] = VENDOR_ID >> 8;

	ppp->val[18] = max_relay_num;

	if(ppp->val[15] == GET_DEV_ADDR){
	    ppp->val[0] = LGT_CMD_DEV_ADDR_RSP | 0xc0;
		if(dev_addr_with_mac_flag(params)){
			return dev_addr_with_mac_rsp(params, ppp->val + 3);
	    }else{
		    memcpy(ppp->val + 3, &device_address, 2);
	    }
	}else if(ppp->val[15] == GET_USER_NOTIFY){
		#if 1
        /*user can get parameters from APP.
             params[0] is relay times.
             params[1 -- 9] is parameters from APP if haved been set by user.
             
             dst_unicast == 1 means destination address is unicast address.
             */
        if(dst_unicast){
        	// params[0 -- 9] is valid
        }else{
        	// only params[0 -- 4] is valid
        }
        #endif
        
	    ppp->val[0] = LGT_CMD_USER_NOTIFY_RSP | 0xc0;
	    foreach(i, 10){//params[0]
            ppp->val[i+3] = i;
	    }
	    ppp->val[3] = (u8)device_address;
	}else if(ppp->val[15] == GET_GROUP1){
        u8 idx = 0;
	    ppp->val[0] = LGT_CMD_LIGHT_GRP_RSP1 | 0xc0;
	    foreach(i, MAX_GROUP_NUM){
            ppp->val[i+3] = 0xFF;
	        if(group_address[i]){
    	        ppp->val[idx+3] = group_address[i];
    	        ++idx;
    	    }
	    }
	}else{
	    return 0;
	}
	
	return 1;
}

void light_init_default(void){
	extern u8 advData[3];
	extern u8 max_mesh_name_len;
	u8 len = (sizeof(advData) + sizeof(ll_adv_private_t));
	if(len >= 31){
		// error
		max_mesh_name_len = 0;
	}else{
		max_mesh_name_len = 31 - len;
		max_mesh_name_len = max_mesh_name_len < 16 ? max_mesh_name_len : 16;
	}

    // get fw version @flash 0x02,0x03,0x04,0x05
    mesh_get_fw_version();
    
	user_data_len =0 ; //disable add the userdata after the adv_pridata
	//usb_log_init ();
	light_set_tick_per_us (CLOCK_SYS_CLOCK_1US);

	extern u8 pair_config_valid_flag;
	pair_config_valid_flag = PAIR_VALID_FLAG;

	extern u8 pair_config_mesh_name[17];
	u8 meshName[] = {MESH_NAME};
	memset(pair_config_mesh_name, 0, sizeof(pair_config_mesh_name));
	memcpy(pair_config_mesh_name, meshName, strlen((char *)meshName) > max_mesh_name_len? max_mesh_name_len : strlen((char *)meshName));
	
	extern u8 pair_config_mesh_pwd[17];
	u8 msehPwd[] = {MESH_PWD};
	memset(pair_config_mesh_pwd, 0, sizeof(pair_config_mesh_pwd));
	memcpy(pair_config_mesh_pwd, msehPwd, strlen((char *)msehPwd) > 16? 16:strlen((char *)msehPwd)); 
	
	u8 service_uuid[16] = TELINK_SPP_UUID_SERVICE;
	u8 data_s2c_uuid[16] = TELINK_SPP_DATA_SERVER2CLIENT;
	u8 data_c2s_uuid[16] = TELINK_SPP_DATA_CLIENT2SERVER;
	u8 data_ota_uuid[16] = TELINK_SPP_DATA_OTA;
	u8 data_pair_uuid[16] = TELINK_SPP_DATA_PAIR;
	extern void setSppUUID(u8 *p_service_uuid, u8 *p_data_s2c_uuid, u8 *p_data_c2s_uuid, u8 *p_data_ota_uuid, u8 *p_data_pair_uuid);
	setSppUUID(service_uuid, data_s2c_uuid, data_c2s_uuid, data_ota_uuid, data_pair_uuid);

	p_adv_pri_data = (u8*)(&adv_pri_data);
	adv_private_data_len = sizeof(ll_adv_private_t);

	rf_link_slave_pairing_enable (1);
	
#if((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
	rf_set_power_level_index (MY_RF_POWER_INDEX);
#else
	rf_set_power_level_index (RF_POWER_8dBm);
#endif
	
	rf_link_slave_set_buffer (buff_response[0], 48);

	//rf_link_set_debug_adv_channel (38);

	extern void rf_link_set_max_bridge (int num);
	rf_link_set_max_bridge (BRIDGE_MAX_CNT);
	vendor_id_init(VENDOR_ID);
	
	usb_dp_pullup_en (1);
#if(MESH_PAIR_ENABLE)
	mesh_pair_init();
#endif

	//extern void	rf_link_set_max_relay (u8 num);
	//rf_link_set_max_relay (2);
}

void light_set_master_data(void){
	// write_master_data means need to send some data to master,just padding data into send_to_master[]
	if(write_master_data){
		write_master_data = 0;
		memcpy(send_to_master, (u8*)(&slave_group), 2);
	}
}

void light_auth_check(void){
	if(security_enable && !pair_login_ok && slave_first_connected_tick && clock_time_exceed(slave_first_connected_tick, AUTH_TIME*1000*1000)){
		//rf_link_slave_disconnect(); // must login in 60s after connected, if need
		slave_first_connected_tick = 0;
	}
}

void light_user_func(void){
	light_set_master_data();
	
	light_auth_check();
}

#if(!PANEL_ENABLE)
void proc_keyboard ()
{
	static u32		tick_key_pressed, tick_key_repeat;
	static u8		kb_last[2];
	kb_event.keycode[0] = 0;
	kb_event.keycode[1] = 0;

	int det_key = kb_scan_key (0, 1);
	static u8 key_released = 0;

	///////////////////////////////////////////////////////////////////////////////////////
	//			key change:pressed or released
	///////////////////////////////////////////////////////////////////////////////////////
	if (det_key) 	{
		/////////////////////////// key pressed  /////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
		if (kb_event.keycode[0]) {
			if(key_released && mode_config){
				mode_config = 0;
				rf_link_slave_enable (0);
				set_proc_st_lpn(ST_LPN_REQUEST);
			}
		    active_time = clock_time();
            if ((kb_event.keycode[0] == RC_KEY_A_ON && kb_event.keycode[1] == RC_KEY_1_OFF) ||
                (kb_event.keycode[1] == RC_KEY_A_ON && kb_event.keycode[0] == RC_KEY_1_OFF))
		    {
            	mode_config = 1;
            	cfg_led_event (LED_EVENT_FLASH_4HZ_3T);		//enterring configuration mode
			}else if ((kb_event.keycode[0] == RC_KEY_A_ON && kb_event.keycode[1] == RC_KEY_4_OFF) ||
                (kb_event.keycode[1] == RC_KEY_A_ON && kb_event.keycode[0] == RC_KEY_4_OFF))
		    {
                irq_disable();
                extern int factory_reset();
                extern void rf_led_ota_ok(void);
                factory_reset();
                rf_led_ota_ok();
                light_sw_reboot();
		    }else{
		        u8 cmd[11] = {0};
		        int group = 0;
    			if (kb_event.keycode[0] == RC_KEY_1_ON){
    			    cmd[0] = LGT_CMD_LIGHT_ON;
    			    group = switch_group[0];
    			}else if (kb_event.keycode[0] == RC_KEY_2_ON){
    			    cmd[0] = LGT_CMD_LIGHT_ON;
    			    group = switch_group[1];
    			}else if (kb_event.keycode[0] == RC_KEY_3_ON){
    			    cmd[0] = LGT_CMD_LIGHT_ON;
    			    group = switch_group[2];
    			}else if (kb_event.keycode[0] == RC_KEY_4_ON){
    			    cmd[0] = LGT_CMD_LIGHT_ON;
    			    group = switch_group[3];
    			}else if (kb_event.keycode[0] == RC_KEY_A_ON){
    			    cmd[0] = LGT_CMD_LIGHT_ON;
    			    group = 0xffff;
    			}else if (kb_event.keycode[0] == RC_KEY_1_OFF){
    			    cmd[0] = LGT_CMD_LIGHT_OFF;
    			    group = switch_group[0];
    			}else if (kb_event.keycode[0] == RC_KEY_2_OFF){
    			    cmd[0] = LGT_CMD_LIGHT_OFF;
    			    group = switch_group[1];
    			}else if (kb_event.keycode[0] == RC_KEY_3_OFF){
    			    cmd[0] = LGT_CMD_LIGHT_OFF;
    			    group = switch_group[2];
    			}else if (kb_event.keycode[0] == RC_KEY_4_OFF){
    			    cmd[0] = LGT_CMD_LIGHT_OFF;
    			    group = switch_group[3];
    			}else if (kb_event.keycode[0] == RC_KEY_A_OFF){
    			    cmd[0] = LGT_CMD_LIGHT_OFF;
    			    group = 0xffff;
    			}else if (kb_event.keycode[0] == RC_KEY_UP){
    			    cmd[0] = LGT_CMD_LUM_UP;
    			    group = 0xFFFF;
    			}else if (kb_event.keycode[0] == RC_KEY_DN){
    			    cmd[0] = LGT_CMD_LUM_DOWN;
    			    group = 0xFFFF;
    			}else if (kb_event.keycode[0] == RC_KEY_L){
    			    cmd[0] = LGT_CMD_LEFT_KEY;
    			    group = 0xFFFF;
    			}else if (kb_event.keycode[0] == RC_KEY_R){
    			    cmd[0] = LGT_CMD_RIGHT_KEY;
    			    group = 0xFFFF;
    			}else{
    				// invalid key
    				memset4(&kb_event, 0, sizeof(kb_event));
					key_released = 0;
    				return;
    			}

	            cfg_led_event (LED_EVENT_FLASH_4HZ_1T);
                light_slave_tx_command(cmd, group);
			}
			key_released = 0;
		}

		///////////////////////////   key released  ///////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
		else {
			rc_repeat_key = 0;
			key_released = 1;
			if (mode_config)
			{
				mode_config = 2;
			}
		}

		tick_key_repeat = tick_key_pressed = clock_time ();
		kb_last[0] = kb_event.keycode[0];
		kb_last[1] = kb_event.keycode[1];

		//reg_usb_ep8_dat = kb_last[0];			//debug purpose, output to usb UART
	}
	//////////////////////////////////////////////////////////////////////////////////////////
	//				no key change event
	//////////////////////////////////////////////////////////////////////////////////////////
	else if (kb_last[0])
	{
		//	long pressed
		active_time = clock_time();
		if (clock_time_exceed(tick_key_pressed, 1500000))		// long pressed
		{

		}
		if (clock_time_exceed (tick_key_pressed, 500000))		//repeat key mode
		{
			if (kb_last[0] == VK_UP)
			{
				rc_repeat_key = LGT_CMD_LUM_UP;
			}
			else if (kb_last[0] == VK_DOWN)
			{
				rc_repeat_key = LGT_CMD_LUM_DOWN;
			}
			else if (kb_last[0] == VK_LEFT)
			{
				rc_repeat_key = LGT_CMD_LEFT_KEY;
			}
			else if (kb_last[0] == VK_RIGHT)
			{
				rc_repeat_key = LGT_CMD_RIGHT_KEY;
			}
		}

		if (!rc_long_pressed && clock_time_exceed(tick_key_pressed, 700000))
		{
			rc_long_pressed = 1;
			if (kb_last[0] == VK_1)		//group 1
			{
				
			}
		}

	}else{
		key_released = 1;
		static u8 connect_flag = 0;
        if(mode_config){
            if(slave_link_connected == 0){
            	if(connect_flag || clock_time_exceed(config_start_time, 60*1000*1000)){
            	    if(is_default_mesh()){
                		config_start_time = clock_time();
                		mode_config = 0;
                		connect_flag = 0;
                		rf_link_slave_enable (0);
            		}else{
            		    light_sw_reboot();  // reboot and start send request.
            		}
            	}
        	    if((0 == led_count) && (!led_event_pending)){
                    cfg_led_event (LED_EVENT_FLASH_1HZ_1T);
                }
            }else if(connect_flag == 0){
            	connect_flag = 1;
            }
        }
	}

	if (rc_repeat_key && clock_time_exceed (tick_key_repeat, ACTIVE_INTERVAL * BRIDGE_MAX_CNT))	// should be longer than interval of command
	{
		tick_key_repeat = clock_time ();
		tx_command_repeat ();
	}

	rc_key_pressed = kb_last[0];
	/////////////////////////////////////////////////////////////////////////////////
}
#endif
////////////////////////////////////////////////////////////////////////////
void main_loop(void)
{
	static u32  dbg_m_loop;
	dbg_m_loop++;
	
	if(is_receive_ota_window()){
		return ;
	}

#if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(1); // should be before key board check
#endif
    //flash_protect_debug();
    if((!key_wakeup_flag) && friendship_proc_lpn()){
	    return ;
	}
	
#if(!PANEL_ENABLE)
	proc_keyboard ();
#endif

	cmd_busy = mesh_send_user_command ();

	light_user_func ();
	
	rf_link_slave_proc ();

	proc_led ();

	//proc_debug ();

#if(STACK_CHECK_ENABLE)
    stack_check();
#endif

	proc_suspend (!(DEBUG_SUSPEND || mode_config), rc_key_pressed || cmd_busy);


#if DEUBG_UART_TEXT_EN
	printf ("main loop: %d\r\n", dbg_m_loop);
	sleep_us (100000);
#endif

#if DEUBG_UART_HEX_EN
	USBUART (dbg_m_loop>>24);
	USBUART (dbg_m_loop>>16);
	USBUART (dbg_m_loop>>8);
	USBUART (dbg_m_loop);
#endif

#if ADC_SET_CHN_ENABLE
	static u32 adc_tmp_check_time;
	if(clock_time_exceed(adc_tmp_check_time, 40*1000)){
        adc_tmp_check_time = clock_time();
        static u32 T_adc_mv;
		#if(MCU_CORE_TYPE == MCU_CORE_8258)
       	T_adc_mv = adc_sample_and_get_result();
		#else		
		static u32 T_adc_val_tmp;		
        T_adc_val_tmp = adc_val_get();
		T_adc_mv = (T_adc_val_tmp * adc_ref_get()) >> 14 ;
		#endif
    }
#endif
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// BIT(0):  0 power   1 no power
// BIT(1):  1  timer
// BIT(2):  1 io

#if (PM_DEEPSLEEP_RETENTION_ENABLE)
_attribute_ram_code_ 
#endif
void user_init_peripheral(int retention_flag)
{
    if (mode_config){
        cfg_led_event (LED_EVENT_FLASH_4HZ_3T);
    }else{
        rf_link_slave_enable (0);
        //cfg_led_event (LED_EVENT_FLASH_1HZ_1T);
    }
    
#if ADC_SET_CHN_ENABLE
    #if(MCU_CORE_TYPE == MCU_CORE_8258)
	adc_drv_init();  
	#else
	adc_set_chn_init();
	#endif
#endif 

    //light_hw_timer0_config();
    //light_hw_timer1_config();

#if (PANEL_ENABLE)
    light_panel_config();
#else
    ////////// set up wakeup source: driver pin of keyboard  //////////
    u32 pin[] = KB_DRIVE_PINS;
    //#if (0 == DEEP_SLEEP_EN)   //suspend
    for (int i=0; i<sizeof (pin)/sizeof(u32); i++)
    {
        //cpu_set_gpio_wakeup (pin[i], 0, 1);
        gpio_set_wakeup (pin[i], 1, 1);         // level : 1 (high); 0 (low)
    }
    gpio_core_wakeup_enable_all (1);
    //#else   //deep sleep
    for (int i=0; i<sizeof (pin)/sizeof(u32); i++)
    {
        cpu_set_gpio_wakeup (pin[i], 1, 1);     // level : 1 (high); 0 (low)
    }
    //#endif
#endif

    key_wakeup_flag = is_wakeup_by_gpio();
    
    lpn_debug_gpio_init();
}

void  user_init(void)
{
    #if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(0); //battery check must do before OTA relative operation
    #endif
	blc_readFlashSize_autoConfigCustomFlashSector();
    flash_get_id();
    sw_flag = 1;
    
	SW_Low_Power = 1;// note: if use app to get sw's onlinestatus, it must 0
    pkt_need_relay = 0;

    erase_ota_data_handle();
    
	retrieve_switch_grp_address();
    
	extern void set_vendor_function(void);
	set_vendor_function();

	if(p_vendor_user_init){
		p_vendor_user_init();
		return;
	}
#if((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
	mode_config = analog_read (rega_sno + 6);			//mode_config
	light_init_default();
	cmd_sno = analog_read (rega_sno+7);
	u8 wakeup_flag = analog_read (rega_sno + 8);
	if(wakeup_flag & BIT(7)){
		led_lum = wakeup_flag & 0x7F;
		led_rgb = mode_config >> 2;
	}
	mode_config = mode_config & 0x03;
	analog_write (rega_sno + 6, 0);
#else
	u8 wakeup_flag = analog_read (rega_sno + 3);

	mode_config = analog_read (rega_sno + 6);			//mode_config
	analog_write (rega_sno + 6, 0);
	//while (1);

	light_init_default();
	
    //rf_link_set_user_data_sno (analog_read (rega_sno));
	cmd_sno = analog_read (rega_sno);
	cmd_dst = analog_read (rega_sno + 1) + analog_read (rega_sno + 2) * 256;
	if(cmd_dst == 0){
		cmd_dst = 0xFFFF;
	}
	
	if (wakeup_flag & BIT(2))
	{
		led_lum = analog_read (rega_sno + 4);
		led_rgb = analog_read (rega_sno + 5);
	}else if(0 == wakeup_flag){
        // power on
	}
#endif
    
#if (SW_NO_PAIR_ENABLE)
	extern u8 sw_no_pair;
	sw_no_pair = 1;
#endif
	rf_link_slave_init (40000);

	extern void vendor_set_adv_data(void);
	vendor_set_adv_data();

#if(STACK_CHECK_ENABLE)
    stack_check_init();
#endif
    
    device_status_update();

#if (SW_NO_PAIR_ENABLE)
	mesh_security_enable (0);
#else
	mesh_security_enable (1);
#endif

   	 if(!security_enable){
		//not_need_login = pair_login_ok = 1;// 1 means not need login when no-security mode.
   	 }

	extern u32 switch_rf_tx_once_time;
	switch_rf_tx_once_time = TX_ONE_RF_US;       // us
	
	if(0 == mode_config){
	    lpn_proc_st_init();
	}

    user_init_peripheral(0);
}
#endif

#if (PM_DEEPSLEEP_RETENTION_ENABLE)
_attribute_ram_code_ void user_init_deepRetn(void)
{
    lpn_debug_set_current_pin(1);
    blc_ll_initBasicMCU();   //mandatory
	rf_set_power_level_index (MY_RF_POWER_INDEX);
    blc_ll_recoverDeepRetention();
    user_init_peripheral(1);
    // should enable IRQ here, because it may use irq here, for example BLE connect.
    // irq_enable();
}

void global_reset_new_key_wakeup()
{
    rc_key_pressed = 0;
    rc_long_pressed = rc_repeat_key = 0;
    memset(&kb_event, 0, sizeof(kb_event));
    global_var_no_ret_init_kb();
}
#endif

