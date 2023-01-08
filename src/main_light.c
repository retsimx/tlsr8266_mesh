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
#include "proj/tl_common.h"
#include "proj/mcu/watchdog_i.h"
#include "proj_lib/rf_drv.h"
#include "proj_lib/pm.h"
#include "proj_lib/light_ll/light_ll.h"
#include "proj_lib/light_ll/light_frame.h"
#include "proj_lib/ble_ll/service.h"
#include "vendor/common/rtc.h"
#include "proj/drivers/flash.h"
#include "vendor/common/common.h"
#include "proj_lib/ble_ll/blueLight.h"

#define LED_INDICATE_VAL    (0xff)

#define LIGHT_ADJUST_STEP_EN    1
#define LIGHT_NOTIFY_MESH_EN    0
#define SW_GET_MESH_DATA		0		//exchange data with switch(use mesh)

FLASH_ADDRESS_EXTERN;

u16 		ota_pkt_cnt = 0;
u32 		lum_changed_time = 0;
u32 		active_time = 0;
//u8 			write_master_data = 0;
u32 		light_lum_addr = 0;
u16			led_val[3] = {MAX_LUM_BRIGHTNESS_VALUE, MAX_LUM_BRIGHTNESS_VALUE, MAX_LUM_BRIGHTNESS_VALUE};
u16         led_lum = 0xffff;
extern u32			led_event_pending;
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
#define 	PANEL_LED		    GPIO_PA5

extern u8* 	slave_p_mac;
extern u16 	slave_group;
extern u8 	pair_login_ok;
extern u8   not_need_login;
extern u8 	slave_link_connected;
extern u8	rf_link_key[];
extern u8 	max_relay_num;
extern ll_adv_private_t adv_pri_data;
extern ll_adv_rsp_private_t adv_rsp_pri_data;
extern u8	adv_private_data_len;

////////////////////////////////////////////////////////////////////////////
rf_custom_dat_t slave_tx_cmd = {
		0x0, 0x0,					//nid,ttc
		0xffff,						//group
		0xffffffff,					//destination id
		{0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	};
	
rf_custom1_dat_t slave_tx_cmd1 = {
	{0, 0, 0},
	{0x11, 0x11}
	};

//////////////////////////////////////////////////////
#define LED_MASK							0x07
#define	config_led_event(on,off,n,sel)		(on | (off<<8) | (n<<16) | (sel<<24))

#define	LED_EVENT_FLASH_4HZ_10S				config_led_event(2,2,40,LED_MASK)
#define	LED_EVENT_FLASH_STOP				config_led_event(1,1,1,LED_MASK)
#define	LED_EVENT_FLASH_2HZ_2S				config_led_event(4,4,4,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_1S				config_led_event(8,8,1,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_2S				config_led_event(8,8,2,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_3S				config_led_event(8,8,3,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_4S				config_led_event(8,8,4,LED_MASK)
#define	LED_EVENT_FLASH_4HZ					config_led_event(2,2,0,LED_MASK)
#define	LED_EVENT_FLASH_1HZ					config_led_event(8,8,0,LED_MASK)
#define	LED_EVENT_FLASH_4HZ_3T				config_led_event(2,2,3,LED_MASK)
#define	LED_EVENT_FLASH_1HZ_3T				config_led_event(8,8,3,LED_MASK)
#define	LED_EVENT_FLASH_0p25HZ_1T			config_led_event(4,60,1,LED_MASK)

extern void light_set_tick_per_us(int tick);
extern void	rf_link_slave_pairing_enable(int en);
extern void	rf_link_slave_set_buffer (u32 *p, u8 n);
extern int rf_link_get_op_para(u8 *p, u8 *op, u8 *op_len, u8 *para, u8 *para_len, u8 mesh_flag);

float calculate_lumen_map(u16 val) {
    float percentage = ((float) val / (float) MAX_LUM_BRIGHTNESS_VALUE) * 100.f;
    float x2 = percentage * percentage;
    float x3 = x2 * percentage;
    return (-0.00539160*x3)+(4.47709595*x2)+(153.72442036*percentage);
}

void	pwm_set_lum (int id, u32 y, int pol)
{
    u32 lum = ((u32)y * PMW_MAX_TICK) / MAX_LUM_BRIGHTNESS_VALUE;

	pwm_set_cmp (id, pol ? PMW_MAX_TICK - lum : lum);
}

u32 get_pwm_cmp(u16 val, u16 lum){
    float val_lumen_map = calculate_lumen_map(lum);
    
//#if LIGHT_ADJUST_STEP_EN
//    light_step_correct_mod(&val_lumen_map, lum);
//#endif

    float val_temp = val;
    return (val_temp * val_lumen_map) / MAX_LUM_BRIGHTNESS_VALUE;
}

void light_adjust_R(u16 val, u16 lum){
    pwm_set_lum (PWMID_R, get_pwm_cmp(val, lum), 0);
}

void light_adjust_G(u16 val, u16 lum){
    pwm_set_lum (PWMID_G, get_pwm_cmp(val, lum), 0);
}

void light_adjust_B(u16 val, u16 lum){
    pwm_set_lum (PWMID_B, get_pwm_cmp(val, lum), 1);
}

void light_adjust_RGB_hw(u16 val_R, u16 val_G, u16 val_B, u16 lum){
	light_adjust_R(val_R, lum);
	light_adjust_G(val_G, lum);
	light_adjust_B(val_B, lum);
}

extern void light_step_reset(u16 target);

void light_onoff_normal(u8 on){
    if(on){
        light_off = 0;
    	light_adjust_RGB_hw(led_val[0], led_val[1], led_val[2], led_lum);
	}else{
        light_off = 1;
    	light_adjust_RGB_hw(0, 0, 0, 0);
	}
}

void device_status_update(){
    // packet
    u8 st_val_par[MESH_NODE_ST_PAR_LEN] = {0};
    memset(st_val_par, 0xFF, sizeof(st_val_par));

    u16 value = light_off ? 0x00 : (led_lum ? led_lum : 1);

    // led_lum should not be 0, because app will take it to be light off
#if SUB_ADDR_EN
    st_val_par[0] = get_sub_addr_onoff();                           //Note: bit7 of par[0] have been use internal for FLD_SYNCED
#else
    st_val_par[0] = value & 0xff;     //Note: bit7 of par[0] have been use internal for FLD_SYNCED
#endif
    st_val_par[1] = (value >> 8) & 0xff;   // rsv
    // end

    ll_device_status_update(st_val_par, sizeof(st_val_par));
}

void device_status_update2(u16 cmd){
    // packet
    u8 st_val_par[MESH_NODE_ST_PAR_LEN] = {0};
    memset(st_val_par, 0xFF, sizeof(st_val_par));
    // led_lum should not be 0, because app will take it to be light off
    st_val_par[0] = cmd & 0xff;     //Note: bit7 of par[0] have been use internal for FLD_SYNCED
    st_val_par[1] = (cmd >> 8) & 0xff;   // rsv
    // end

    ll_device_status_update(st_val_par, sizeof(st_val_par));
}

void light_onoff_hw(u8 on){
    #if LIGHT_ADJUST_STEP_EN
    light_onoff_step(on);
    #else
    light_onoff_normal(on);
	#endif
}

void light_onoff(u8 on){
    light_onoff_hw(on);

	device_status_update();
}

/////////////////// rf interrupt handler ///////////////////////////
_attribute_ram_code_ void irq_handler(void)
{
	irq_light_slave_handler ();

#if (I2C_HW_MODOULE_EN && I2C_IRQ_EN)
    I2C_IrqSrc i2c_src = I2C_SlaveIrqGet();
    if(I2C_IRQ_NONE != i2c_src){
        I2C_SlaveIrqClr(i2c_src);
    }
#endif
}

void irq_timer0(void){
#if IRQ_TIMER0_ENABLE
	static u8 a_irq_timer0;
	a_irq_timer0++;
#endif
}

#if (SYNC_TIME_EN)
STATIC_ASSERT((SYNC_ALTER_TIME % (IRQ_TIME1_INTERVAL * CLOCK_SYS_CLOCK_1MS) == 0));
STATIC_ASSERT((SYNC_CMD_DELAY_TIME_MS % IRQ_TIME1_INTERVAL) == 0);
STATIC_ASSERT((IRQ_TIME1_INTERVAL >= 10) && (IRQ_TIME1_INTERVAL <= 100));
STATIC_ASSERT((SYNC_CMD_DELAY_TIME_MS == 400) && ((SYNC_CMD_DELAY_TIME_MS / IRQ_TIME1_INTERVAL) <= 255));
STATIC_ASSERT(SYNC_TIME * (CLOCK_SYS_CLOCK_1S/CLOCK_SYS_CLOCK_1MS) > (SYNC_ALTER_TIME/CLOCK_SYS_CLOCK_1MS));
STATIC_ASSERT((SYNC_ALTER_TIME == 100*CLOCK_SYS_CLOCK_1MS)
                   ||(SYNC_ALTER_TIME == 200*CLOCK_SYS_CLOCK_1MS)
                   ||(SYNC_ALTER_TIME == 400*CLOCK_SYS_CLOCK_1MS)
                   ||(SYNC_ALTER_TIME >= 500*CLOCK_SYS_CLOCK_1MS));
STATIC_ASSERT((2*CLOCK_SYS_CLOCK_1S)/SYNC_ALTER_TIME >= 2);

#if(SYNC_ALTER_TIME <= SYNC_CMD_DELAY_TIME_MS*CLOCK_SYS_CLOCK_1MS)
u8 sync_cmd_change_level = (((SYNC_CMD_DELAY_TIME_MS*CLOCK_SYS_CLOCK_1MS)/SYNC_ALTER_TIME)%2 == 1);
#else
u8 sync_cmd_change_level = 0;
#endif
u32 sync_alter_cycle_10ms = SYNC_ALTER_TIME / (IRQ_TIME1_INTERVAL * CLOCK_SYS_CLOCK_1MS);
u32 t1_cycle_tick = (IRQ_TIME1_INTERVAL * CLOCK_SYS_CLOCK_1MS);
u32 sync_near_time2alter = (2*CLOCK_SYS_CLOCK_1S)/SYNC_ALTER_TIME;
u32 sync_10ms_tick_last = 0;

ll_packet_l2cap_data_t sync_cmd_delay;
static u32 sync_alter_cnts;

void sync_alter_action_callback(u8 high)
{
    #if SYNC_LIGHT_ONOFF_EN
    #if 0   // pattern 1
    light_onoff_normal(high);
    #elif 1   // pattern 2
    if((sync_alter_cnts * sync_alter_cycle_10ms) % 350 < 250){
        light_onoff_normal(1);
    }else{
        light_onoff_normal(0);
    }
    sync_alter_cnts++;
    #endif
    #endif
}

void sync_cmd_action_callback(u8 sync_reset)
{
    if(sync_reset){
        sync_alter_cnts = 0;    // init
        // sync by app fuction
    }
}
#endif

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
    
    #if (SYNC_TIME_EN)
    // must after rf_link_data_callback()
    sync_time_irq_handle(SYNC_CMD_DELAY_TIME_MS / IRQ_TIME1_INTERVAL, SYNC_TIME2ALTERS);
    #endif
    
    #if LIGHT_ADJUST_STEP_EN
	light_onoff_step_timer();
	#endif
}

void	cfg_led_event (u32 e)
{
	led_event_pending = e;
}

//////////////////////////////////////////////////////

void	pwm_led_en (int id, int en)
{
	if (id == 0)
	{
		gpio_set_func (PWM_G, en);
	}
	else if (id == 1)
	{
		gpio_set_func (PWM_B, en);
	}
	else if (id == 2)
	{
		gpio_set_func (PWM_R, en);
	}
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

#if(LIGHT_MODE == LIGHT_MODE_CCT)
const u8 rgb_map[RGB_MAP_MAX][3] = {
	{204,   0,      0},
	{204,   31,     0},
	{204,   77,     0},
	{179,   117,    0},
	{153,   156,    0},
	{133,   179,    3},
	{117,   207,    5},
	{111,   255,    9},
	{99,    255,    15},
	{92,    255,    23},
	{84,    255,    31},
	{79,    255,    38},
	{69,    255,    43},
	{59,    255,    48},
	{56,    255,    56},
	{51,    255,    64},
	{43,    255,    69},
	{38,    255,    74},
	{31,    255,    79},
	{23,    255,    84},
	{18,    255,    89},
	{13,    255,    94},
	{8,     255,    99},
	{3,     255,    107},
	{0,     255,    115},
	{0,     240,    120},
	{0,     230,    128},
	{0,     217,    133},
	{0,     209,    140},
	{0,     196,    145},
	{0,     184,    153},
	{0,     176,    158},
	{0,     166,    163},
	{0,     156,    168},
	{0,     145,    173},
	{0,     138,    179},
	{0,     128,    184},
	{0,     117,    186},
	{0,     110,    194},
	{0,     99,     196},
	{0,     92,     204},
	{0,     84,     207},
	{0,     77,     212},
	{0,     69,     217},
	{0,     61,     219},
	{0,     54,     224},
	{0,     46,     230},
	{0,     38,     235},
	{0,     31,     237},
	{0,     23,     242},
	{0,     15,     245},
};
#else //if(LIGHT_MODE == LIGHT_MODE_RGB)
const u8 rgb_map[RGB_MAP_MAX][3] = {
	{	4	,	255 ,	128 },
	{	8	,	252 ,	132 },
	{	12	,	248 ,	136 },
	{	16	,	244 ,	140 },
	{	20	,	240 ,	144 },
	{	24	,	236 ,	148 },
	{	28	,	232 ,	152 },
	{	32	,	228 ,	156 },
	{	36	,	224 ,	160 },
	{	40	,	220 ,	164 },
	{	44	,	216 ,	168 },
	{	48	,	212 ,	172 },
	{	52	,	208 ,	176 },
	{	56	,	204 ,	180 },
	{	60	,	200 ,	184 },
	{	64	,	196 ,	188 },
	{	68	,	192 ,	192 },
	{	72	,	188 ,	196 },
	{	76	,	184 ,	200 },
	{	80	,	180 ,	204 },
	{	84	,	176 ,	208 },
	{	88	,	172 ,	212 },
	{	92	,	168 ,	216 },
	{	96	,	164 ,	220 },
	{	100 ,	160 ,	224 },
	{	104 ,	156 ,	228 },
	{	108 ,	152 ,	232 },
	{	112 ,	148 ,	236 },
	{	116 ,	144 ,	240 },
	{	120 ,	140 ,	244 },
	{	124 ,	136 ,	248 },
	{	128 ,	132 ,	252 },
	{	132 ,	128 ,	255 },
	{	136 ,	124 ,	4	},
	{	140 ,	120 ,	8	},
	{	144 ,	116 ,	12	},
	{	148 ,	112 ,	16	},
	{	152 ,	108 ,	20	},
	{	156 ,	104 ,	24	},
	{	160 ,	100 ,	28	},
	{	164 ,	96	,	32	},
	{	168 ,	92	,	36	},
	{	172 ,	88	,	40	},
	{	176 ,	84	,	44	},
	{	180 ,	80	,	48	},
	{	184 ,	76	,	52	},
	{	188 ,	72	,	56	},
	{	192 ,	68	,	60	},
	{	196 ,	64	,	64	},
	{	200 ,	60	,	68	},
	{	204 ,	56	,	72	},
	{	208 ,	52	,	76	},
	{	212 ,	48	,	80	},
	{	216 ,	44	,	84	},
	{	220 ,	40	,	88	},
	{	224 ,	36	,	92	},
	{	228 ,	32	,	96	},
	{	232 ,	28	,	100 },
	{	236 ,	24	,	104 },
	{	240 ,	20	,	108 },
	{	244 ,	16	,	112 },
	{	248 ,	12	,	116 },
	{	252 ,	8	,	120 },
	{	255 ,	4	,	124 },
};
#endif

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

#if (LIGHT_NOTIFY_MESH_EN)
void light_sim_notify_mesh(void){    // all light can use.
	static u32 last_time = 0;
	if(clock_time_exceed(last_time, 5000*1000)){
        if(!is_tx_cmd_busy()){   // it must be used, if want enough bridges(default is BRIDGE_MAX_CNT 8).
    		last_time = clock_time();
    		static u8 cnt = 0;
    		u8 op_para[16] = {0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    		op_para[0] = LGT_CMD_NOTIFY_MESH;
    		op_para[3] = (++cnt);
    		light_slave_tx_command(op_para, 0xFFFF);
		}
	}
}
#endif

void light_sim_light2light(void){
	static u32 last_time = 0;
	if(clock_time_exceed(last_time, 5000*1000)){
        if(!is_tx_cmd_busy()){   // it must be used, if want enough bridges(default is BRIDGE_MAX_CNT 8).
    		last_time = clock_time();
    		static u8 cnt = 0;
    		u8 op_para[16] = {0};
    		op_para[0] = LGT_CMD_LIGHT_ONOFF;
    		op_para[3] = (++cnt)%2;//params[0], on or off
    		light_slave_tx_command(op_para, 0xFFFF);//0xFFFF or 0x8001......
        }
	}
}

#if SW_GET_MESH_DATA
u8 SWTxCnt = 0;
u32 SWClock = 0;
void light_respond_to_sw(void){
	u8 op_para[16] = {0};
	op_para[0] = LGT_CMD_SW_RSP;
	op_para[3] = SWTxCnt;
	op_para[12] = SWTxCnt;
	light_slave_tx_command(op_para, 0xFFFF);//0xFFFF or 0x8001......
}
#endif

void light_hw_timer0_config(void){
#if IRQ_TIMER0_ENABLE
 	//enable timer0 interrupt
	reg_irq_mask |= FLD_IRQ_TMR0_EN;
	reg_tmr0_tick = 0;
	reg_tmr0_capt = CLOCK_SYS_CLOCK_1US * IRQ_TIME0_INTERVAL;
	reg_tmr_ctrl |= FLD_TMR0_EN;
#endif
}

void light_hw_timer1_config(void){
 	//enable timer1 interrupt
	reg_irq_mask |= FLD_IRQ_TMR1_EN;
	reg_tmr1_tick = 0;
	reg_tmr1_capt = CLOCK_SYS_CLOCK_1US * IRQ_TIME1_INTERVAL * 1000;
	
    #if (SYNC_TIME_EN)
	sync_10ms_tick_last = clock_time();     // init
	#endif
	reg_tmr_ctrl |= FLD_TMR1_EN;
}

// p_cmd : cmd[3]+para[10]
// para    : dst
int light_slave_tx_command_ll(u8 *p_cmd, int para, u8 sub_addr)
{
    static u8 dbg_tx = 0;
    dbg_tx++;

    if(is_bridge_task_busy()){
        return 0;	// user should retry tx command to handle this case.
    }
    
    u8 cmd_op_para[16] = {0};
    static int cmd_sno = 0;
    cmd_sno = clock_time() + device_address;
    memset(cmd_op_para, 0, 16);
    memcpy(cmd_op_para, p_cmd, 13);
    cmd_op_para[0] = p_cmd[0] | 0xc0;
    cmd_op_para[1] = VENDOR_ID & 0xFF;
    cmd_op_para[2] = VENDOR_ID >> 8;
    #if 0   // for test
    static u8 a_oppara[16];
    memcpy(a_oppara, p_cmd, 16);
    #endif

    u16 dst = (u16)para;
    #if SUB_ADDR_EN
    if(sub_addr){
        mesh_push_user_command_sub_addr(cmd_sno++, sub_addr, dst, cmd_op_para, 13);
    }else
    #endif
    {
        mesh_push_user_command(cmd_sno++, dst, cmd_op_para, 13);
    }

    return 1;
}

int light_slave_tx_command(u8 *p_cmd, int para)
{
    return light_slave_tx_command_ll(p_cmd, para, 0);
}

#if SUB_ADDR_EN
int light_slave_tx_command_sub_addr(u8 *p_cmd, int para, u8 sub_addr)
{
    return light_slave_tx_command_ll(p_cmd, para, sub_addr);
}
#endif

enum{
	LUM_SAVE_FLAG = 0xA5,
};

typedef struct{
    u8 save_falg;
    u16 lum;
    u16 ledval[3];
}lum_save_t;

extern void light_lum_erase(void);
//retrieve LUM : brightness or RGB/CT value
void light_lum_retrieve(void){
    for(int i = 0; i < (FLASH_SECTOR_SIZE); i += sizeof(lum_save_t)){
		light_lum_addr = flash_adr_lum + i;
	
        lum_save_t *lum_save = (lum_save_t *)(light_lum_addr);
		if(LUM_SAVE_FLAG == lum_save->save_falg){
            led_lum = lum_save->lum;
            memcpy(led_val, lum_save->ledval, sizeof(led_val));
		}else if(lum_save->save_falg != 0xFF){
		    //invalid
		    continue;
		}else{
		    break;
		}
	}

	//effect
    #if LIGHT_ADJUST_STEP_EN
	light_adjust_RGB_hw(0, 0, 0, 0);
	#endif

	mesh_ota_master_100_flag_check();
	
	u8 val = analog_read(rega_light_off);
	if(val & FLD_LIGHT_OFF){
	    analog_write(rega_light_off, val & (~ FLD_LIGHT_OFF));
	    light_onoff(0);
	}else{
	    light_onoff(1);
	}
}

/*@brief: This function is called in IRQ state, use IRQ stack.
*/
char lightOn = 0;
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

        	if(op == LGT_CMD_LIGHT_ONOFF){
        		if(cmd_left_delay_ms){
        			return;
        		}
        		cmd_delay_ms = params[1] | (params[2] << 8);
        		if(cmd_delay_ms && !irq_timer1_cb_time){
        			u16 cmd_delayed_ms = light_cmd_delayed_ms(pp->val[op_cmd_len+params_len]);
        			if(cmd_delay_ms > cmd_delayed_ms){
        				memcpy(&cmd_delay, p, sizeof(ll_packet_l2cap_data_t));
        				cmd_left_delay_ms = cmd_delay_ms - cmd_delayed_ms;
        				irq_timer1_cb_time = clock_time();
        				return;
        			}
        		}
        	}
        	
            mesh_ota_timeout_handle(op, params);

            if(op == LGT_CMD_LIGHT_ONOFF){
                if(params[0] == LIGHT_ON_PARAM){
					#if SUB_ADDR_EN
            		light_multy_onoff(pp->dst, 1);
            		#else
            		light_onoff(1);
            		#endif
        		}else if(params[0] == LIGHT_OFF_PARAM){
        		    if(ON_OFF_FROM_OTA == params[3]){ // only PWM off, 
                        light_step_reset(0);
        		    }else{
        		        #if SUB_ADDR_EN
        		        light_multy_onoff(pp->dst, 0);
        		        #else
            		    light_onoff(0);
            		    #endif
            		}
        		}else if(params[0] == LIGHT_SYNC_REST_PARAM){
                    #if (SYNC_TIME_EN)
                    sync_time_reset();
            		#endif
        		}
            }
#if (SYNC_TIME_EN)
            else if(op == LGT_CMD_TIME_SYNC){    // don't modify, internal use
                u16 cmd_delayed_ms = light_cmd_delayed_ms(pp->val[op_cmd_len+params_len]);
                if(sync_time_callback(params, cmd_delayed_ms, IRQ_TIME1_INTERVAL, p)){
                    return ;
                }
            }
#endif        	
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
            else if (op == LGT_CMD_SET_LIGHT)
        	{
                if (params[8] & 0x1) {
                    // Brightness
                    u16 value = ((u16) params[1] << 8) | (u8) params[0];
                    if(light_off){
                        led_lum = value;
                        return;
                    }
                    light_step_reset(value);
                }
                if (params[8] & 0x2) {
                    // Temperature
                    u16 value = ((u16) params[3] << 8) | (u8) params[2];

                    led_val[0] = 0;
                    led_val[1] = MAX_LUM_BRIGHTNESS_VALUE - value;
                    led_val[2] = value;

                    if(light_off){
                        return;
                    }

                    light_step_reset(led_lum);
                }

                lum_changed_time = clock_time();
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
            else if (op == LGT_CMD_NOTIFY_MESH)
            {
                light_notify(pp->val+3, 10, pp->src);
            }
            else if (op == LGT_CMD_MESH_OTA_DATA)
            {
                u16 idx = params[0] + (params[1] << 8);
                if(!is_master_ota_st()){  // no update firmware for itself
                    if(CMD_START_MESH_OTA == idx){
                        mesh_ota_master_start_firmware_from_own();
                        //cfg_led_event(LED_EVENT_FLASH_1HZ_4S);
                    }else if(CMD_STOP_MESH_OTA == idx){
                        if(is_mesh_ota_slave_running()){
                            // reboot to initial flash: should be delay to relay command.
                            mesh_ota_slave_reboot_delay();  // reboot after 320ms
                        }
                    }else{
                        if(mesh_ota_slave_save_data(params)){
                            static u16 mesh_ota_error_cnt;
                            mesh_ota_error_cnt++;
                        }
                    }
                }else{
                    if(CMD_STOP_MESH_OTA == idx){
                        mesh_ota_master_cancle(OTA_STATE_MASTER_OTA_REBOOT_ONLY, 0);
                        //cfg_led_event(LED_EVENT_FLASH_4HZ_3T);
                    }
                }
            }
#if(MESH_PAIR_ENABLE)
            else if (op == LGT_CMD_MESH_PAIR)
            {
                mesh_pair_cb(params);
            }
#endif
#if SW_GET_MESH_DATA
		else if(op == LGT_CMD_SW_DATA)
		{
			static u8 cnt;
			if((++cnt)%2)
				light_onoff(0);
			else
				light_onoff(1);
			SWClock = clock_time();
			SWTxCnt = 0;
		}
#endif
    }
}

/*@brief: This function is called in IRQ state, use IRQ stack.
**@param: p: p is pointer to response
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
	#if SUB_ADDR_EN
	device_addr_sub_t *p_adr_sub = (device_addr_sub_t *)ppp->src;
	u8 need_bridge = p_adr_sub->dev_id != get_addr_by_pointer(ppp->dst);
	#else
	u8 need_bridge = (0 != memcmp(ppp->src, ppp->dst, 2));
	#endif
#endif

	u8 params[10] = {0};
	memcpy(params, ppp->val+3, sizeof(params)); // be same with p_req->val+3
	memset(ppp->val+3, 0, 10);
	    
	ppp->val[1] = VENDOR_ID & 0xFF;
	ppp->val[2] = VENDOR_ID >> 8;

	ppp->val[18] = max_relay_num;

    u8 idx = 0;
	if(ppp->val[15] == GET_STATUS){
	    ppp->val[0] = LGT_CMD_LIGHT_STATUS | 0xc0;
	    foreach(i, 3){//params[0]
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
	}else if(ppp->val[15] == GET_USER_NOTIFY){
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
        
	    ppp->val[0] = LGT_CMD_USER_NOTIFY_RSP | 0xc0;
	    foreach(i, 8){//params[2]
            ppp->val[5+i] = i;
	    }
	    memcpy(ppp->val + 3, &device_address, 2);
	}else if(ppp->val[15] == GET_MESH_OTA){
	    ppp->val[0] = LGT_CMD_MESH_OTA_READ_RSP | 0xc0;
#if(MESH_PAIR_ENABLE)
		if(params[1] == PAR_READ_MESH_PAIR_CONFIRM){
			foreach(i, 8){
				ppp->val[5+i] = get_mesh_pair_checksum(i);
			}
			memcpy(ppp->val + 3, &device_address, 2);
			return 1;
		}
#endif
	    return mesh_ota_slave_set_response(ppp->val+3, params[1]);
	}else{
	    return 0;
	}
	
	return 1;
}

void wd_clear(void)
{
    SET_FLD(reg_tmr_ctrl, FLD_CLR_WD);
}
