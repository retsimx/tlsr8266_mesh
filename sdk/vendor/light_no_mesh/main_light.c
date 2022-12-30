/********************************************************************************************************
 * @file     main_light.c 
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

#define	DEBUG_SUSPEND			0

#if UART_ENABLE
#define UART_DATA_LEN    44      // data max 252
typedef struct{
    u32 len;        // data max 252
    u8 data[UART_DATA_LEN];
}uart_data_t;
STATIC_ASSERT((sizeof(uart_data_t) % 16) == 0);

uart_data_t T_txdata_user;
uart_data_t T_txdata_buf;      // not for user

uart_data_t T_rxdata_user;
uart_data_t T_rxdata_buf;   // data max 252, user must copy rxdata to other Ram,but not use directly
unsigned char uart_rx_true;
#endif

FLASH_ADDRESS_EXTERN;

#define		DEUBG_UART_HEX_EN				0

#define		DEUBG_UART_TEXT_EN				0

#if		DEUBG_UART_HEX_EN
#define		USBUART(d)		reg_usb_ep8_dat = d
#else
#define		USBUART(d)
#endif

u16 		ota_pkt_cnt = 0;
u32 		lum_changed_time = 0;
u32 		active_time = 0;
int			led_dbg;
u32			buff_response[48][9];
u8 			table_map_idx = 0;
u8 			write_master_data = 0;
u32 		light_lum_addr = 0;
u8			led_val[6] = {PMW_MAX_TICK_BASE, PMW_MAX_TICK_BASE, PMW_MAX_TICK_BASE, PMW_MAX_TICK_BASE, PMW_MAX_TICK_BASE, PMW_MAX_TICK_BASE};
u8          led_lum = 100;
u8          led_lum_tmp = 0;
u8          music_time = 0;
u32         last_music_tick = 0;
u32			led_event_pending;
u16			cmd_delay_ms = 0;
u16			cmd_left_delay_ms = 0;
u32 		irq_timer1_cb_time = 0;
u32         slave_send_data = 0;
u8 			light_off = 1;
ll_packet_l2cap_data_t cmd_delay;

fp_rf_led_ota_ok 			p_vendor_rf_led_ota_ok 			= 0;
fp_rf_led_ota_error			p_vendor_rf_led_ota_error 		= 0;
fp_irq_timer1 				p_vendor_irq_timer1 			= 0;
fp_proc_led 				p_vendor_proc_led 				= 0;
fp_rf_link_data_callback	p_vendor_rf_link_data_callback	= 0;
fp_user_init 				p_vendor_user_init				= 0;

#define		reg_debug_cmd		REG_ADDR16(0x800c)

// send_to_master 16bytes data, will be reset after send to master
extern u8 	send_to_master[];

extern u8 	max_mesh_name_len;
extern u8* 	slave_p_mac;
extern u16 	slave_group;
extern u32	slave_first_connected_tick;
extern u8 	pair_login_ok;
extern u8   not_need_login;
extern u8 	slave_link_connected;
extern u8	rf_link_key[];
extern u8 	max_relay_num;
extern ll_adv_private_t adv_pri_data;
extern ll_adv_rsp_private_t adv_rsp_pri_data;
extern u8	adv_private_data_len;
extern u8 security_enable;

void led_onoff(u8 on);

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
#define	LED_EVENT_FLASH_4HZ_3T				config_led_event(2,2,3,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_3T				config_led_event(8,8,3,LED_MASK)

extern void light_set_tick_per_us(int tick);
extern void	rf_link_slave_pairing_enable(int en);
extern void	rf_link_slave_set_buffer (u32 *p, u8 n);
extern int rf_link_get_op_para(u8 *p, u8 *op, u8 *op_len, u8 *para, u8 *para_len, u8 mesh_flag);

void light_onoff_normal(u8 on){
    if(on){
        light_off = 0;
        led_onoff(1);
	}else{
        light_off = 1;
        led_onoff(0);      
	}
}

void device_status_update(){
    // packet
    u8 st_val_par[MESH_NODE_ST_PAR_LEN] = {0};
    memset(st_val_par, 0xFF, sizeof(st_val_par));
    // led_lum should not be 0, because app will take it to be light off
    st_val_par[0] = light_off ? 0x00 : (led_lum ? led_lum : 1);     //Note: bit7 of par[0] have been use internal for FLD_SYNCED
	st_val_par[1] = 0xFF;   // rsv
    // end
    
    ll_device_status_update(st_val_par, sizeof(st_val_par));
}

void light_onoff_hw(u8 on){
    light_onoff_normal(on);
}

void light_onoff(u8 on){
    light_onoff_hw(on);

	device_status_update();
}

/////////////////// rf interrupt handler ///////////////////////////
_attribute_ram_code_ void irq_handler(void)
{
	irq_light_slave_handler ();

#if UART_ENABLE
    static unsigned char enterRXIrq,enterTXIrq;
    unsigned char irqS = uart_IRQSourceGet();
    if(irqS & UARTRXIRQ_MASK){
        uart_rx_true = 1;
        enterRXIrq++;
    }
    
    if(irqS & UARTTXIRQ_MASK){
        uart_clr_tx_busy_flag();
        enterTXIrq++;
    }
#endif     
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
#if UART_ENABLE
    uart_ErrorCLR();
#endif
}

void	cfg_led_event (u32 e)
{
	led_event_pending = e;
}

void rf_link_light_event_callback (u8 status)
{
    extern void mesh_node_init ();
	if(status == LGT_CMD_SET_MESH_INFO)
	{
        mesh_node_init();
        device_status_update();
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
    gpio_write(GPIO_LED, on);
}

void proc_led(void)
{
	if(p_vendor_proc_led){
		p_vendor_proc_led();
		return;
	}
	static	int led_count = 0;
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
			int led_off = (led_is_on || !led_ton) && led_toff;
			int led_on = !led_is_on && led_ton;

			led_is_on = !led_is_on;
			if (led_is_on)
			{
				led_no++;
				led_dbg++;
				if (led_no - 1 == led_count)
				{
					led_count = led_no = 0;
					light_onoff_hw(!light_off); // should not report online status again
					return ;
				}
			}
			
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

void proc_suspend (int en, int active)
{
    if(SW_Low_Power && (!DEBUG_SUSPEND))
	{
		BLE_low_power_handle(0, 0);
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

void gpio_risc2_user_handle(void){
	return;
}

extern rf_packet_att_cmd_t	pkt_light_notify;

int light_notify(u8 *p, u8 len, u8* p_src){
    int err = -1;
    if(slave_link_connected && pair_login_ok){
        if(len > 10){   //max length of par is 10
            return -1;
        }
        
        pkt_light_notify.value[3] = p_src[0];
        pkt_light_notify.value[4] = p_src[1];
        
        u8 *p_par = &(pkt_light_notify.value[10]);
        memset(p_par, 0, 10);
        memcpy(p_par, p, len);
        u8 r = irq_disable();
        if (is_add_packet_buf_ready()){
            if(0 != rf_link_add_tx_packet ((u32)(&pkt_light_notify))){
                err = 0;
            }
        }
        irq_restore(r);
    }

    return err;
}

void light_sim_notify(void){    // just local light can use.
	static u32 cur_time = 0;
	static u8 cnt = 0;
	if(clock_time_exceed(cur_time, 5000*1000)){
		cur_time = clock_time();
	    #if 1
        u8 buf[10] = {0};
        buf[0] = cnt++;
        light_notify(buf, sizeof(buf), (u8 *)&device_address);
	    #else
		extern u8 SppDataServer2ClientData[];
		SppDataServer2ClientData[0] = cnt++;
		extern void sim_send_status(void);
		sim_send_status();
		#endif
	}
}

void light_hw_timer0_config(void){
#if IRQ_TIMER0_ENABLE
 	//enable timer0 interrupt
	reg_irq_mask |= FLD_IRQ_TMR0_EN;
	reg_tmr0_tick = 0;
	reg_tmr0_capt = CLOCK_MCU_RUN_CODE_1US * IRQ_TIME0_INTERVAL;
	reg_tmr_ctrl |= FLD_TMR0_EN;
#endif
}

void light_hw_timer1_config(void){
 	//enable timer1 interrupt
	reg_irq_mask |= FLD_IRQ_TMR1_EN;
	reg_tmr1_tick = 0;
	reg_tmr1_capt = CLOCK_MCU_RUN_CODE_1US * IRQ_TIME1_INTERVAL * 1000;
	
	reg_tmr_ctrl |= FLD_TMR1_EN;
}

enum{
	LUM_SAVE_FLAG = 0xA5,
};

typedef struct{
    u8 save_falg;
    u8 lum;
    u8 ledval[ARRAY_SIZE(led_val)];
    u8 map_idx;
}lum_save_t;

extern void light_lum_erase(void);
//retrieve LUM : brightness or RGB/CT value
void light_lum_retrieve(void){
    for(int i = 0; i < (FLASH_SECTOR_SIZE); i += sizeof(lum_save_t)){
		light_lum_addr = flash_adr_lum + i;
	
        lum_save_t *lum_save = (lum_save_t *)(light_lum_addr);
		if(LUM_SAVE_FLAG == lum_save->save_falg){
            led_lum = lum_save->lum;
            table_map_idx = lum_save->map_idx;
            memcpy(led_val, lum_save->ledval, sizeof(led_val));
		}else if(lum_save->save_falg != 0xFF){
		    //invalid
		    continue;
		}else{
		    break;
		}
	}

	//effect
	u8 val = analog_read(rega_light_off);
	if(val & FLD_LIGHT_OFF){
	    analog_write(rega_light_off, val & (~ FLD_LIGHT_OFF));
	    light_onoff(0);
	}else{
	    light_onoff(1);
	}
}

//save cur lum value, if disconnected for a while
u8 light_lum_store(void){
	if(light_lum_addr >= (flash_adr_lum + FLASH_SECTOR_SIZE - sizeof(lum_save_t))){
		light_lum_erase();
		return 1;
	}
	
    lum_save_t lum_save = {0};
	
    lum_save.save_falg = LUM_SAVE_FLAG;
    lum_save.lum = led_lum;
    lum_save.map_idx = table_map_idx;
    memcpy(lum_save.ledval, led_val, sizeof(lum_save.ledval));
	
	flash_write_page(light_lum_addr, sizeof(lum_save), (u8 *)(&lum_save));
	light_lum_addr += sizeof(lum_save_t);
	
	return 1;
}

//erase flash
void light_lum_erase(void){
	light_lum_addr = flash_adr_lum;
	flash_erase_sector(flash_adr_lum);
	light_lum_store();
}

#if 1
void rf_link_data_callback (u8 *p)
{
    // p start from l2capLen of rf_packet_att_cmd_t
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
        	if(op == LGT_CMD_LIGHT_ONOFF){
                if(params[0] == LIGHT_ON_PARAM){
            		light_onoff(1);
        		}else if(params[0] == LIGHT_OFF_PARAM){
        		    if(ON_OFF_FROM_OTA == params[3]){ // only PWM off, 
        		        led_onoff(0);
        		    }else{
            		    light_onoff(0);
            		}
        		}else if(params[0] == LIGHT_SYNC_REST_PARAM){
        		}
            }
            else if (op == LGT_CMD_LIGHT_CONFIG_GRP){
        		u16 val = (params[1] | (params[2] << 8));
        		if(params[0] == LIGHT_DEL_GRP_PARAM){
        			extern u8 rf_link_del_group(u16 group);
        			if(rf_link_del_group(val)){
        			    cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
        			}
        		}else if(params[0] == LIGHT_ADD_GRP_PARAM){
        			extern u8 rf_link_add_group(u16 group);
        			if(rf_link_add_group(val)){
        			    cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
        			}
        		}
        	}else if (op == LGT_CMD_CONFIG_DEV_ADDR){
        		u16 val = (params[0] | (params[1] << 8));
    			extern u8 rf_link_add_dev_addr(u16 deviceaddress);
    			if(!dev_addr_with_mac_flag(params) || dev_addr_with_mac_match(params)){
    				if(rf_link_add_dev_addr(val)){
						mesh_pair_proc_get_mac_flag();
					}
    			}
        	}
        	else if (op == LGT_CMD_KICK_OUT)
        	{
        		if(is_mesh_cmd_need_delay(p, params, pp->val[op_cmd_len+params_len])){
        			return ;
        		}
        	    irq_disable();
        	    void kick_out(u8 par);
        	    kick_out(params[0]);
        	    light_sw_reboot();
        	}
#if(ALARM_EN)
        	else if (op == LGT_CMD_SET_TIME)
        	{
                rtc_t rtc_old, rtc_new;
                memcpy(&rtc_old, &rtc, sizeof(rtc_t));
        	    memcpy(&rtc_new.year, params, 7);
        	    if(0 == rtc_set_time((rtc_t *)params)){
        	        //ok
        	        check_event_after_set_time(&rtc_new, &rtc_old);
                    cfg_led_event(LED_EVENT_FLASH_1HZ_3T);
        	    }else{
        	        //invalid params
                    cfg_led_event(LED_EVENT_FLASH_4HZ_3T);
        	    }
        	}
        	else if (op == LGT_CMD_ALARM)
        	{
        	    if(0 == alarm_ev_callback((u8*)params)){
                    cfg_led_event(LED_EVENT_FLASH_1HZ_3T);
        	    }else{
                    cfg_led_event(LED_EVENT_FLASH_4HZ_3T);
        	    }
        	}
#endif        	
#if(MESH_PAIR_ENABLE)
	        else if (op == LGT_CMD_MESH_PAIR)
	        {
	            mesh_pair_cb(params);
	        }
#endif
#if(SCENE_EN)
        	else if (op == LGT_CMD_SET_SCENE)
        	{
        	    if(params[0] == SCENE_ADD){
        	        // add scene: params valid & no repetition
        	        scene_t *pData = (scene_t*)(params+1);
        	        if(pData->id && pData->lum <= 100 
            	            && pData->rgb[0] <= 0xFF
            	            && pData->rgb[1] <= 0xFF
            	            && pData->rgb[2] <= 0xFF){
                        if(scene_add(pData)){
                            cfg_led_event(LED_EVENT_FLASH_1HZ_3T);
                        }
        	        }
        	            
        	    }else if(params[0] == SCENE_DEL){
        	        // del scene
        	        if(scene_del(params[1])){
                        cfg_led_event(LED_EVENT_FLASH_1HZ_3T);
        	        }
        	    }
        	}
        	else if (op == LGT_CMD_LOAD_SCENE)
        	{
        	    scene_load(params[0]);
            }
#endif        	
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
	
	#if(ALARM_EN || SCENE_EN)
	u8 need_bridge = (0 != memcmp(ppp->src, ppp->dst, 2));
	#endif

	u8 params[10] = {0};
	memcpy(params, ppp->val+3, sizeof(params));
	memset(ppp->val+3, 0, 10);
	    
	ppp->val[1] = VENDOR_ID & 0xFF;
	ppp->val[2] = VENDOR_ID >> 8;

	ppp->val[18] = max_relay_num;

    u8 idx = 0;
	if(ppp->val[15] == GET_STATUS){
	    ppp->val[0] = LGT_CMD_LIGHT_STATUS | 0xc0;
	    foreach(i, 6){//params[0]
            ppp->val[i+3] = (light_off)?0:led_val[i];
	    }
	}else if(ppp->val[15] == GET_GROUP1){
	    ppp->val[0] = LGT_CMD_LIGHT_GRP_RSP1 | 0xc0;
	    foreach(i, MAX_GROUP_NUM){
            ppp->val[i+3] = 0xFF;
	        if(group_address[i]){
    	        ppp->val[idx+3] = group_address[i];
    	        ++idx;
    	    }
	    }
	}else if(ppp->val[15] == GET_GROUP2){
	    ppp->val[0] = LGT_CMD_LIGHT_GRP_RSP2 | 0xc0;
	    foreach(i, MAX_GROUP_NUM){
            ppp->val[i+3] = 0xFF;
	        if(group_address[i/2]){
    	        ppp->val[idx+3] = (i%2)?(group_address[i/2]>>8):(group_address[i/2]);
    	        ++idx;
    	    }
	    }
	}else if(ppp->val[15] == GET_GROUP3){
	    ppp->val[0] = LGT_CMD_LIGHT_GRP_RSP3 | 0xc0;
	    foreach(i, MAX_GROUP_NUM){
            ppp->val[i+3] = 0xFF;
	        if(group_address[4+i/2]){
    	        ppp->val[idx+3] = (i%2)?(group_address[4+i/2]>>8):(group_address[4+i/2]);
    	        ++idx;
    	    }
	    }
	}else if(ppp->val[15] == GET_DEV_ADDR){
	    ppp->val[0] = LGT_CMD_DEV_ADDR_RSP | 0xc0;
		if(dev_addr_with_mac_flag(params)){
			return dev_addr_with_mac_rsp(params, ppp->val + 3);
	    }else{
		    memcpy(ppp->val + 3, &device_address, 2);
	    }
#if(ALARM_EN)
	}else if(ppp->val[15] == GET_ALARM){
        if(!is_bridge_task_busy()){
            ppp->val[0] = LGT_CMD_ALARM_RSP | 0xc0;
            u8 id = params[1];
            if(id != 0){   // parameter : alarm id
                if(id == 0xff){
                    alarm_get_all_id(ppp->val + 3);
                }else{
                    alarm_get_by_id(ppp->val + 3, id);
                }
            }else{
                alarm_poll_notify_init(need_bridge);    
                alarm_poll_notify(ppp->val + 3);
                if(need_bridge){
                    alarm_rsp_mesh();
                    return 0;
                }
            }
        }else{
            return 0;
        }
	}else if(ppp->val[15] == GET_TIME){
        ppp->val[0] = LGT_CMD_TIME_RSP | 0xc0;
        rtc_get_time(ppp->val + 3);
#endif	    
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
#if(SCENE_EN)
	}else if(ppp->val[15] == GET_SCENE){
        if(!is_bridge_task_busy()){
	        ppp->val[0] = LGT_CMD_SCENE_RSP | 0xc0;
            u8 id = params[1];
            if(id != 0){   // parameter : scene id
                if(id == 0xff){
                    scene_get_all_id(ppp->val + 3);
                }else{
                    scene_get_by_id(ppp->val + 3, id);
                }
            }else{
                scene_poll_notify_init(need_bridge);    
                scene_poll_notify(ppp->val + 3);
                if(need_bridge){
                    scene_rsp_mesh();
                    return 0;
                }
            }
        }else{
            return 0;
        }
#endif        
	}else{
	    return 0;
	}
	
	return 1;
}

void light_init_default(void){
	extern u8 advData[3];
	u8 len = sizeof(advData) + (sizeof(ll_adv_private_t) + 2);
	if(len >= 31){
		// error
		max_mesh_name_len = 0;
	}else{
		max_mesh_name_len = 31 - len - 2;
		max_mesh_name_len = max_mesh_name_len < 16 ? max_mesh_name_len : 16;
	}

    // get fw version @flash 0x02,0x03,0x04,0x05
    mesh_get_fw_version();

	//add the user_data after the adv_pri_data
	u8 user_const_data[6]={0x05,0x02,0x19,0x00,0x69,0x69};
	memcpy(user_data,user_const_data,sizeof(user_const_data));
    user_data_len =0 ; //disable add the userdata after the adv_pridata

	usb_log_init ();
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
	
	extern u8 pair_config_mesh_ltk[17];
	u8 msehLtk[] = {MESH_LTK};
	memset(pair_config_mesh_ltk, 0, sizeof(pair_config_mesh_ltk));
	memcpy(pair_config_mesh_ltk, msehLtk, 16); 

	u8 service_uuid[16] = TELINK_SPP_UUID_SERVICE;
	u8 data_s2c_uuid[16] = TELINK_SPP_DATA_SERVER2CLIENT;
	u8 data_c2s_uuid[16] = TELINK_SPP_DATA_CLIENT2SERVER;
	u8 data_ota_uuid[16] = TELINK_SPP_DATA_OTA;
	u8 data_pair_uuid[16] = TELINK_SPP_DATA_PAIR;
	extern void setSppUUID(u8 *p_service_uuid, u8 *p_data_s2c_uuid, u8 *p_data_c2s_uuid, u8 *p_data_ota_uuid, u8 *p_data_pair_uuid);
	setSppUUID(service_uuid, data_s2c_uuid, data_c2s_uuid, data_ota_uuid, data_pair_uuid);

	p_adv_pri_data = (u8*)(&adv_pri_data);
	adv_private_data_len = sizeof(ll_adv_private_t);
	p_adv_rsp_data = (u8*)(&adv_rsp_pri_data);

	rf_link_slave_pairing_enable (1);
	
	rf_set_power_level_index (RF_POWER_8dBm);
	
	rf_link_slave_set_buffer (buff_response[0], 48);

	//rf_link_set_debug_adv_channel (38);

	extern void rf_link_set_max_bridge (int num);
	rf_link_set_max_bridge (BRIDGE_MAX_CNT);
	vendor_id_init(VENDOR_ID);
	
	usb_dp_pullup_en (1);

	//extern void	rf_link_set_max_relay (u8 num);
	//rf_link_set_max_relay (3);
	
	//light_hw_timer0_config();             // don't use timer interrupt in suspend mode
	//light_hw_timer1_config();

	extern u32 online_status_timeout;
    online_status_timeout = ONLINE_STATUS_TIMEOUT;
    
#if(PA_ENABLE)
    pa_init(0, 0);
#endif

#if(SCENE_EN)
    scene_init();
#endif
#if(MESH_PAIR_ENABLE)
	mesh_pair_init();
#endif
}

void light_set_master_data(void){
	// write_master_data means need to send some data to master,just padding data into send_to_master[]
	if(write_master_data){
		write_master_data = 0;
		memcpy(send_to_master, (u8*)&slave_send_data, 4);
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

    extern int factory_reset_cnt_check ();
	factory_reset_cnt_check();
}

////////////////////////////////////////////////////////////////////////////
void main_loop(void)
{
	static u32  dbg_m_loop;
	dbg_m_loop++;
	
	if(is_receive_ota_window()){
		return ;
	}

	//flash_protect_debug();
	
#if(ALARM_EN)
	alarm_handle();
#endif

#if(SCENE_EN)
	scene_handle();
#endif	
    
	//light_sim_notify();

	light_user_func ();
	
	rf_link_slave_proc ();

	proc_led ();
#if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(1);
#endif
#if GATEWAY_EN
    proc_ui_gateway ();
#endif    

#if(STACK_CHECK_ENABLE)
    stack_check();
#endif

	//proc_debug ();

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

#if UART_ENABLE 
    if(uart_rx_true){
        uart_rx_true = 0;
        u32 rx_len = T_rxdata_buf.len + 4 > sizeof(T_rxdata_user) ? sizeof(T_rxdata_user) : T_rxdata_buf.len + 4;
        memcpy(&T_rxdata_user, &T_rxdata_buf, rx_len);
        uart_Send((u8 *)(&T_rxdata_user));
    }
#endif

#if ADC_ENABLE
    static u32 adc_check_time;
    if(clock_time_exceed(adc_check_time, 1000*1000)){
        adc_check_time = clock_time();
        static u16 T_adc_val;
        T_adc_val = adc_BatteryValueGet();
    }    
#endif
#if ADC_SET_CHN_ENABLE
	static u32 adc_tmp_check_time;
	if(clock_time_exceed(adc_tmp_check_time, 40*1000)){
        adc_tmp_check_time = clock_time();
        static u32 T_adc_val_tmp;
		static u32 T_adc_mv;
        T_adc_val_tmp = adc_val_get();
		T_adc_mv = (T_adc_val_tmp * adc_ref_get()) >> 14 ;
    }
#endif

	proc_suspend (!(DEBUG_SUSPEND), 0);
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void  user_init(void)
{
    #if (BATT_CHECK_ENABLE)
    app_battery_power_check_and_sleep_handle(0); //battery check must do before OTA relative operation
    #endif
	blc_readFlashSize_autoConfigCustomFlashSector();
    sw_flag = SW_Low_Power = 1;
    flash_get_id();

    erase_ota_data_handle();
    
#if ADC_SET_CHN_ENABLE
    adc_set_chn_init();
#endif 
#if ADC_ENABLE
	adc_Init();
	adc_BatteryCheckInit(1);    
#endif

	extern void set_vendor_function(void);
	set_vendor_function();
	
	light_init_default();

	if(p_vendor_user_init){
		p_vendor_user_init();
	}else{
	    #if (MCU_CORE_TYPE == MCU_CORE_8267)
		REG_ADDR8(0x5b1) = 0x0;     // set default  function Mux to PWM
		REG_ADDR8(0x5b4) |= 0x3;    // set default  function Mux to PWM for PE0/PE1
		#endif
		
		//retrieve lumen value
		light_lum_retrieve();
	}
	
	rf_link_slave_init (40000);

    extern int factory_reset_handle ();
    factory_reset_handle();

	extern void vendor_set_adv_data(void);
	vendor_set_adv_data();

#if UART_ENABLE
    //Initial IO
    uart_io_init(UART_GPIO_SEL);
    
    CLK32M_UART115200;
    uart_BuffInit((u8 *)(&T_rxdata_buf), sizeof(T_rxdata_buf), (u8 *)(&T_txdata_buf));
#endif	
	
#if(IBEACON_ENABLE)
    #if 0 // eddystone UID
    extern u8 mac_id[];
    memcpy(eddystone_uid + 23, mac_id, 6);
    beacon_len = 31;
    memcpy(iBeaconData, eddystone_uid, beacon_len);
    set_ibeacon_data(iBeaconData, beacon_len);
    #endif
    #if 0 // eddystone URL
    beacon_len = eddystone_url[0]+eddystone_url[3]+eddystone_url[7]+3;
    memcpy(iBeaconData, eddystone_url, beacon_len);
    set_ibeacon_data(iBeaconData, beacon_len);
    #endif
    #if 0 // eddystone TLM
    beacon_len = 25;
    memcpy(iBeaconData, eddystone_tlm, beacon_len);
    set_ibeacon_data(iBeaconData, beacon_len);
    #endif
    iBeaconInterval = IBEACON_INTERVAL;
    //beacon_with_mesh_adv = 1;
#endif

#if(STACK_CHECK_ENABLE)
    stack_check_init();
#endif

	device_status_update();
    mesh_security_enable (1);
	
	if(!security_enable){
	    //not_need_login = pair_login_ok = 1;// 1 means not need login when no-security mode.
	}else{
	    //extern u8 rands_fix_flag;
        //rands_fix_flag = 1;       // 1 means rands[8] = {0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7};
	}

#if GATEWAY_EN
    gateway_init();
    gateway_mode_onoff(1);
    gateway_security = 0;
    gateway_set_login_flag();
#endif

    #if 0// callback for node status changed
    extern void light_node_status_change_cb(u8 *p, u8 new_node);
    extern void	register_mesh_node_status_callback (void *p);
    register_mesh_node_status_callback(&light_node_status_change_cb);
    #endif

    #if 0// 'set_mesh_info_time' means after this time from power-on,set mesh info will be refused.
    extern u32 set_mesh_info_time;// unit: s
    set_mesh_info_time = 30;
    #endif
    
    #if 0
    extern u8 send_adv_flag; // default value is 1, 0 means don't send adv pkt.
    send_adv_flag = 0;
    #endif
}
#endif

