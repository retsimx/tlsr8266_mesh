/********************************************************************************************************
 * @file     motionsensor.c 
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

#define			DEBUG_SUSPEND				0
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

u8			mode_config;

u32			mode_motion_tick = 0;

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

ll_packet_l2cap_data_t	cmd_delay;
extern kb_data_t	kb_event;

static u8	led_lum = 100;//5~100
static u8	led_mode = 0;

#define		reg_debug_cmd		REG_ADDR16(0x8008)

// send_to_master 16bytes data, will be reset after send to master
extern u8 	send_to_master[];

extern u8* 	slave_p_mac;
extern u16 	slave_group;
extern u32	slave_first_connected_tick;
extern u8 	pair_login_ok;
extern u8 	not_need_login;
extern u8   security_enable;
extern u8 	slave_link_connected;
extern u8	rf_link_key[];
extern u8 	max_relay_num;
extern ll_adv_private_t adv_pri_data;
extern u8	adv_private_data_len;
extern ll_adv_rsp_private_t adv_rsp_pri_data;

////////////////////////////////////////////////////////////////////////////
u32 config_start_time = 0;


u16			cmd_dst = 0x8003;
u16			cmd_dst_last = 0xffff;
u8			cmd_op_para[16];
u8			cmd_sno;
u8			cmd_busy = 0;

//////////////////////////////////////////////////////
#define LED_MASK							0x07
#define	config_led_event(on,off,n,sel)		(on | (off<<8) | (n<<16) | (sel<<24))

#define	LED_EVENT_FLASH_4HZ_64S				config_led_event(2,2,255,LED_MASK)
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
	if(irq_timer1_cb_time && clock_time_exceed(irq_timer1_cb_time, cmd_left_delay_ms*1000)){
		cmd_left_delay_ms = 0;
		rf_link_data_callback((u8*)(&cmd_delay));
		cmd_delay_ms = irq_timer1_cb_time = 0;
	}
}

void	cfg_led_event (u32 e)
{
	led_event_pending = e;
}

//////////////////////////////////////////////////////

void device_status_update(){
    // packet
    u8 st_val_par[MESH_NODE_ST_PAR_LEN] = {0};
    memset(st_val_par, 0xFF, sizeof(st_val_par));
    st_val_par[0] = 0x00;
	st_val_par[1] = 0xFF;   // rsv
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
#if(MESH_PAIR_ENABLE)
	else if(status == LGT_CMD_MESH_PAIR_TIMEOUT){
		cfg_led_event(LED_EVENT_FLASH_2HZ_2S);
	}
#endif
}

void led_onoff(u8 on){
    gpio_set_func (GPIO_LED, AS_GPIO);
    gpio_set_output_en (GPIO_LED, 1);
#if 1
    gpio_write(GPIO_LED, !on); 
#else
    gpio_write(GPIO_LED, on);
#endif
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

	if(!led_count && !led_event_pending) {
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

void	pin_motion_release ()
{
	gpio_set_output_en(PIN_MOTION, 1);
	sleep_us (1);
	gpio_set_output_en(PIN_MOTION, 0);
}

void proc_suspend (int en)
{
    
    if (mode_config == 2)
    {
    	/*u8 r = */irq_disable();
    	mode_config++;
    	analog_write (rega_sno+6, mode_config);			//reboot with ble mode on
        light_sw_reboot();
    }

	if(en){
		if (mode_motion_tick && clock_time_exceed (mode_motion_tick, 2000000))
		{
			mode_motion_tick = 0;
			pin_motion_release ();

			/*u8 r = */irq_disable();
			//u8 r = irq_disable();
			// BIT(0):	0 power   1 no power
			// BIT(1):	1  timer
			// BIT(2):	1 io
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
			analog_write (rega_sno + 5, led_mode);

			cpu_sleep_wakeup(1, PM_WAKEUP_PAD, 0) ;
			//cpu_sleep_wakeup(1, PM_WAKEUP_TIMER, clock_time() + 5000 * 32) ;
			active_time = clock_time ();
		}
		else
		{
		    u8 r = irq_disable();
		    led_onoff(0);
			cpu_sleep_wakeup(0, PM_WAKEUP_TIMER, clock_time() + 50000 * 32) ;
		    irq_restore (r);
		}
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

}


void light_hw_timer1_config(void){
 	//enable timer1 interrupt
	reg_irq_mask |= FLD_IRQ_TMR1_EN;
	reg_tmr1_tick = 0;
	reg_tmr1_capt = CLOCK_SYS_CLOCK_1US * IRQ_TIME1_INTERVAL * 1000;
	reg_tmr_ctrl |= FLD_TMR1_EN;
}

int light_slave_tx_command(u8 *p_cmd, int para)
{
	static u32 dbg_sc;
	dbg_sc++;
	memcpy(cmd_last_c, p_cmd, 11);
	cmd_dst_last = para;
	reg_usb_ep8_dat = p_cmd[0];
	reg_usb_ep8_dat = para;

	memset(cmd_op_para, 0, 11);

	cmd_op_para[0] = p_cmd[0];
    cmd_dst = para | 0x8000;
    
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

	cmd_op_para[0] |= 0xc0;
	cmd_op_para[1] = VENDOR_ID & 0xFF;
	cmd_op_para[2] = VENDOR_ID >> 8;

	//rf_link_user_command((u8 *) &slave_tx_cmd);

	mesh_push_user_command((int)(cmd_sno++), cmd_dst, cmd_op_para, 11);

    sw_flag = 1;
	mesh_send_user_command();

	return 1;
}

void tx_command_repeat ()
{
	light_slave_tx_command (cmd_last_c, cmd_dst_last);
	reg_usb_ep8_dat = cmd_last_c[0];
	reg_usb_ep8_dat = cmd_dst_last;
}

#if 1
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
        	if (op == LGT_CMD_SWITCH_CONFIG){
        	    cfg_led_event (config_led_event(4, 4, params[0], LED_MASK));
        	}else if (op == LGT_CMD_CONFIG_DEV_ADDR){
        		u16 val = (params[0] | (params[1] << 8));
    			extern u8 rf_link_add_dev_addr(u16 deviceaddress);
    			if(!dev_addr_with_mac_flag(params) || dev_addr_with_mac_match(params)){
    				if(rf_link_add_dev_addr(val)){
						mesh_pair_proc_get_mac_flag();
					}
    			}
			}else if (op == LGT_CMD_KICK_OUT){
        		if(is_mesh_cmd_need_delay(p, params, pp->val[op_cmd_len+params_len])){
        			return ;
        		}
        	    irq_disable();
        	    void kick_out(u8 par);
        	    kick_out(params[0]);
        	    light_sw_reboot();
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
	usb_log_init ();
	light_set_tick_per_us (CLOCK_SYS_CLOCK_HZ / 1000000);

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
	
	rf_set_power_level_index (RF_POWER_8dBm);
	
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
	
	//light_hw_timer1_config();
}

void light_set_master_data(void){
	// write_master_data means need to send some data to master,just padding data into send_to_master[]
	if(write_master_data){
		write_master_data = 0;
		memcpy(send_to_master, (u8*)(&slave_group), 2);
	}
}

void light_auth_check(void){
	extern u8 security_enable;
	if(security_enable && !pair_login_ok && slave_first_connected_tick && clock_time_exceed(slave_first_connected_tick, AUTH_TIME*1000*1000)){
		//rf_link_slave_disconnect(); // must login in 60s after connected, if need
		slave_first_connected_tick = 0;
	}
}

void light_user_func(void){
	light_set_master_data();
	
	light_auth_check();
}

u32 proc_button ()
{
    static u32 tick;
    if (!clock_time_exceed (tick, ACTIVE_INTERVAL))
    {
        return 0;
    }
    tick = clock_time();
    
    static u8 st = 0, st_key_release_cnt = 0;
    u8 s = !gpio_read (PIN_BUTTON);
    if(s){
        st_key_release_cnt = 0; // recount
        st = 1;
    }else{
        if(st){
            if(++st_key_release_cnt >= 3){      // eliminate dithering
                st = st_key_release_cnt = 0;
                return PIN_BUTTON;
            }
        }
    }

    return 0;
}

void proc_ui ()
{
	static u32	dbg_motion;
	if (!mode_config && !cmd_busy && !mode_motion_tick && gpio_read (PIN_MOTION))
	{
		dbg_motion++;
		pin_motion_release ();
		cfg_led_event (LED_EVENT_FLASH_1HZ_1T);
#if 1   //
		mode_motion_tick = clock_time () | 1;
		u8 cmd_op_para[11] = {LGT_CMD_LIGHT_ON | 0xc0, VENDOR_ID & 0xFF, VENDOR_ID>>8, 1};
		mesh_push_user_command(clock_time(), cmd_dst, cmd_op_para, 11);
#endif		
	}

	if (PIN_BUTTON == proc_button())		//configuration mode
	{
		if (mode_config >= 2)
		{
			mode_config = 0;
			cfg_led_event (LED_EVENT_FLASH_STOP);
			rf_link_slave_enable (0);
		}
		else if (!mode_config)
		{
			mode_config = 1;
			cfg_led_event (LED_EVENT_FLASH_4HZ_1T);		//entering configuration mode
		}
	}
	else if (mode_config == 1)
	{
		mode_config = 2;
	}

#if 1
    if(mode_config > 2){
        if(clock_time_exceed(config_start_time, 60*1000*1000))
        {
            mode_config = 0;
            cfg_led_event (LED_EVENT_FLASH_STOP);     // for DEBUG_SUSPEND mode
            rf_link_slave_enable (0);
        }

        if((0 == led_count) && (!led_event_pending)){
            cfg_led_event (LED_EVENT_FLASH_4HZ_1T);
        }
    }
#endif	
}

#if ADC_SET_CHN_ENABLE
void adc_io_init()
{
    // adc
    u32 pin[] = GPIO_ADC_PIN;
    foreach_arr(i,pin){
        gpio_set_input_en(pin[i], 1);
        gpio_set_output_en(pin[i], 0);
        gpio_set_func(pin[i], AS_GPIO);
        gpio_write(pin[i], 0);
        gpio_setup_up_down_resistor(pin[i], PM_PIN_PULLDOWN_100K);//PM_PIN_UP_DOWN_FLOAT
    }
    
    // adc power supply
    gpio_set_input_en(GPIO_ADC_POWER, 0);
    gpio_set_output_en(GPIO_ADC_POWER, 1);
    gpio_set_func(GPIO_ADC_POWER, AS_GPIO);
    gpio_write(GPIO_ADC_POWER, 1);
    gpio_setup_up_down_resistor(GPIO_ADC_POWER, PM_PIN_PULLDOWN_100K);
}

void adc_handle()
{
    static u32 adc_tmp_check_time;
    if(clock_time_exceed(adc_tmp_check_time, 40*1000)){
        adc_tmp_check_time = clock_time();

        u32 v_ref = adc_ref_get();
        adc_switch_chn(GPIO_ADC_DAY_NIGHT);
        static u32 T_adc_val_tmp;
        static u32 T_adc_mv;
        T_adc_val_tmp = adc_val_get();
        T_adc_mv = (T_adc_val_tmp * v_ref) >> 14 ;

        adc_switch_chn(GPIO_ADC_DELAY);
        static u32 T_adc_val_tmp_1;
        static u32 T_adc_mv_1;
        T_adc_val_tmp_1 = adc_val_get();
        T_adc_mv_1 = (T_adc_val_tmp_1 * v_ref) >> 14 ;
        
        adc_switch_chn(GPIO_ADC_LUX_DET);
        static u32 T_adc_val_tmp_2;
        static u32 T_adc_mv_2;
        T_adc_val_tmp_2 = adc_val_get();
        T_adc_mv_2 = (T_adc_val_tmp_2 * v_ref) >> 14 ;
    }
}
#endif
/////////////////// PIR sensor /////////////////////////////////////////////
void SendBit(u8 bData)
{
	gpio_write(PIN_SERDO, 0);
	sleep_us (2);
	gpio_write(PIN_SERDO, 1);
	sleep_us (2);
	gpio_write(PIN_SERDO, bData);
	sleep_us (180);
}

void E93196_Initialise(void)
{
	gpio_write (PIN_MOTION, 0);
	gpio_set_func(PIN_SERDO, AS_GPIO);
	gpio_set_output_en (PIN_SERDO, 1);

	u8 i,bb;
	//u8 bSensitivity=0x28;   //40
	//u8 bSensitivity=0x0a;   //10
	u8 bSensitivity=0x20;   //10

	bb = bSensitivity;

	// send start
	gpio_write(PIN_SERDO, 0);
	sleep_us (1850);

	for(i=0;i<8;i++)
	{
		if((bb&0x80)==0x80)
		{
			SendBit(1);
		}
		else
		{
			SendBit(0);
		}
		bb=bb<<1;
	}


//send Blind time
	SendBit(0);
	SendBit(0);
	SendBit(0);
	SendBit(1);
//send Pluse Counter
	SendBit(0);
	SendBit(1);
//send Window Time
	SendBit(0);
	SendBit(0);


//send Motion Detector Enable
	SendBit(1);
//send Interrupt Source
	SendBit(0);
//send ADC/Filter Voltage
	SendBit(0);
	SendBit(0);
//send Supply Regulator Enable
	SendBit(0);
//start self-test
	SendBit(0);
//send sample capactitor size
	SendBit(0);
//send User test-mode select
	SendBit(0);
	SendBit(0);

// send end
	gpio_write(PIN_SERDO, 0);

	sleep_us(1850);		//1000us
}

////////////////////////////////////////////////////////////////////////////
u32			A_t0, A_t1, A_t2, A_t3;
void main_loop(void)
{
	A_t0 = clock_time() - A_t0;
	//while (1);

	static u32  dbg_m_loop;
	dbg_m_loop++;
	
	if(is_receive_ota_window()){
		return ;
	}

    //flash_protect_debug();

	proc_ui ();

    sw_flag = 1;
    if (!mode_config)
    {
    	cmd_busy = mesh_send_user_command ();
    }

	light_user_func ();
	#if ADC_SET_CHN_ENABLE
	adc_handle();
	#endif
	
	rf_link_slave_proc ();

	proc_led ();
#if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(1);
#endif

	//proc_debug ();

#if(STACK_CHECK_ENABLE)
    stack_check();
#endif

	proc_suspend (!(DEBUG_SUSPEND || mode_config || cmd_busy));


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
}

#if PIN_MOTION_HIGH_VALID
#define PIN_MOTION_UP_DOWN           PM_PIN_PULLDOWN_100K
#else
#define PIN_MOTION_UP_DOWN           PM_PIN_PULLUP_10K
#endif

#if PIN_BUTTON_HIGH_VALID
#define PIN_BUTTON_UP_DOWN           PM_PIN_PULLDOWN_100K
#else
#define PIN_BUTTON_UP_DOWN           PM_PIN_PULLUP_10K
#endif

void wakeup_io_init()
{
    ////////// set up wakeup source: driver pin of keyboard  //////////
    gpio_setup_up_down_resistor(PIN_MOTION, PIN_MOTION_UP_DOWN);
    gpio_setup_up_down_resistor(PIN_BUTTON, PIN_BUTTON_UP_DOWN);

    // set wake up io for deep sleep
    cpu_set_gpio_wakeup (PIN_MOTION, PIN_MOTION_HIGH_VALID, 1); //active high
    cpu_set_gpio_wakeup (PIN_BUTTON, PIN_BUTTON_HIGH_VALID, 1); //active low

    #if 1
    // set wake up io for suspend
    gpio_set_wakeup (PIN_MOTION, PIN_MOTION_HIGH_VALID, 1);     // level : 1 (high); 0 (low)
    gpio_set_wakeup (PIN_BUTTON, PIN_BUTTON_HIGH_VALID, 1);     // level : 1 (high); 0 (low)
    gpio_core_wakeup_enable_all (1);
    #endif

    // init io pull up or down
    gpio_setup_up_down_resistor(PIN_SERDO, PM_PIN_PULLUP_10K);
    gpio_setup_up_down_resistor(GPIO_LED, PM_PIN_PULLUP_10K);
    
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// BIT(0):  0 power   1 no power
// BIT(1):  1  timer
// BIT(2):  1 io

void  user_init(void)
{
    #if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(0); //battery check must do before OTA relative operation
    #endif
	blc_readFlashSize_autoConfigCustomFlashSector();
	SW_Low_Power = 1;
    flash_get_id();

    erase_ota_data_handle();
    
#if ADC_SET_CHN_ENABLE
    adc_io_init();
    adc_set_chn_init();
#endif 
    
	extern void set_vendor_function(void);
	set_vendor_function();

	if(p_vendor_user_init){
		p_vendor_user_init();
		return;
	}

	u8 wakeup_flag = analog_read (rega_sno + 3);

	mode_config = analog_read (rega_sno + 6);			//mode_config
	analog_write (rega_sno + 6, 0);

	//if (mode_config == 3) while (1);

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
		led_mode = analog_read (rega_sno + 5);
	}
    
    wakeup_io_init();


	rf_link_slave_init (40000);
	if (mode_config)
	{
		 cfg_led_event (LED_EVENT_FLASH_4HZ_64S);
	}
	else
	{
		rf_link_slave_enable (0);
        cfg_led_event (LED_EVENT_FLASH_1HZ_1T);
	}

	extern void vendor_set_adv_data(void);
	vendor_set_adv_data();

#if(STACK_CHECK_ENABLE)
    stack_check_init();
#endif
    
    device_status_update();

	mesh_security_enable (1);

   	 if(!security_enable){
		//not_need_login = pair_login_ok = 1;// 1 means not need login when no-security mode.
   	 }

	extern u32 switch_rf_tx_once_time;
	switch_rf_tx_once_time = 600;       // us

    u8 areg_flag = analog_read(rega_deepsleep_flag);
    if (areg_flag & BIT(1))		//power on
    {
		E93196_Initialise ();
		analog_write (rega_deepsleep_flag, areg_flag & ~(BIT(1)));
    }
    else if(0 == wakeup_flag){
        // power on
	}
    else
    {
    	//while (1);
    }
}
#endif

