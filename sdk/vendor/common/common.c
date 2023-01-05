/********************************************************************************************************
 * @file     common.c 
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
#include "../../proj_lib/light_ll/light_frame.h"
#include "../../proj_lib/ble_ll/blueLight.h"
#include "../../proj_lib/light_ll/light_ll.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/pm.h"
#include "common.h"
#include "../../proj/drivers/uart.h"
#include "../../proj_lib/ble_ll/att.h"
#include "../../proj_lib/rf_drv.h"
#include "dual_mode_adapt.h"

FLASH_ADDRESS_EXTERN;
/////////////// password encode sk initial  ///////////////////////////////////////////////////
u8	pair_config_pwd_encode_sk[17] = {MESH_PWD_ENCODE_SK};
u8  pair_config_pwd_encode_enable = 1;

u8	auth_code[4] = {0x01,0x02,0x03,0x04};
u8  auth_code_en = 0;

u8 tx_packet_bridge_random_en = 0;
u32 ota_firmware_size_k = FW_SIZE_MAX_K;  // used in library. 

/////////////// adv par define ///////////////////////////////////////////////////
u16 adv_interval2listen_interval = 4;           // unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
#if NOTIFY_MESH_COMMAND_TO_MASTER_EN
u16 online_status_interval2listen_interval = 4; // unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
#else
u16 online_status_interval2listen_interval = 8; // unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
#endif
u8	rf_slave_ota_busy_mesh_en = 0;

#if PASSIVE_EN
/////////////// for passive switch ///////////////////////////////////////////////
u8  separate_ADVpkt = 1;					//if 1 send one adv packet in each interrupt
u8  mesh_chn_amount = 2;				//amount of sys_chn_listen
/////////////// listen chanel define ///////////////////////////////////////////////////

#define SYS_CHN_LISTEN_MESH     {2, 12, 2, 12}	//8, 30, 52, 74
#else
/////////////// for passive switch ///////////////////////////////////////////////
u8  separate_ADVpkt = 0;					//if 1 send one adv packet in each interrupt
u8  mesh_chn_amount = 4;				//amount of sys_chn_listen
/////////////// listen chanel define ///////////////////////////////////////////////////

#define SYS_CHN_LISTEN_MESH     {2, 12, 23, 34}	//8, 30, 52, 74
#endif
u8 sys_chn_listen[4] = SYS_CHN_LISTEN_MESH;
const u8 SYS_CHN_LISTEN_ORG[4] = SYS_CHN_LISTEN_MESH;

/////////////// mesh node define ////////////////////////////////////////////////////
mesh_node_st_t mesh_node_st[MESH_NODE_MAX_NUM];
status_record_t slave_status_record[MESH_NODE_MAX_NUM];
u16 slave_status_record_size = sizeof(slave_status_record);

u32	mesh_node_mask[(MESH_NODE_MAX_NUM + 31) >> 5];
u16 mesh_node_max_num = MESH_NODE_MAX_NUM;
u8 mesh_node_st_val_len = MESH_NODE_ST_VAL_LEN;
u8 mesh_node_st_par_len = MESH_NODE_ST_PAR_LEN;
u8 mesh_node_st_len = sizeof(mesh_node_st_t);

void mesh_node_buf_init ()
{
	for (int i=0; i<mesh_node_max_num; i++)
	{
	    memset(&mesh_node_st[i], 0, sizeof(mesh_node_st_t));
	}
	device_status_update();
}

u8	SW_Low_Power = 0;
u8	SW_Low_Power_rsp_flag = 0;
u8	mesh_ota_only_calibrate_type1 = 0;

u16 device_address_mask = DEVICE_ADDR_MASK_DEFAULT;

#if((__TL_LIB_8258__ || (MCU_CORE_TYPE == MCU_CORE_8258)) || (__TL_LIB_8278__ || (MCU_CORE_TYPE == MCU_CORE_8278)))
u8 my_rf_power_index = MY_RF_POWER_INDEX;   // use in library
#endif

//STATIC_ASSERT((MESH_NODE_MAX_NUM <= 256) && ((MESH_NODE_MAX_NUM % 32) == 0));
STATIC_ASSERT((MESH_NODE_MAX_NUM >= 1) && (MESH_NODE_MAX_NUM <= 256));
STATIC_ASSERT((MESH_NODE_ST_VAL_LEN >= 4) && ((MESH_NODE_ST_VAL_LEN <= 10)));
STATIC_ASSERT(0 == (MESH_PAIR_ENABLE && SW_NO_PAIR_ENABLE));
STATIC_ASSERT(0 == (FW_SIZE_MAX_K % 4));    // 4k align
////////////////////mesh command cache define///////////////////////////////////////////
#if NOTIFY_MESH_COMMAND_TO_MASTER_EN
#define RC_PKT_BUF_MAX              4
#else
#define RC_PKT_BUF_MAX              2
#endif

rc_pkt_buf_t rc_pkt_buf[RC_PKT_BUF_MAX];
u8 mesh_cmd_cache_num = RC_PKT_BUF_MAX;
///////////////////////////////////////////////////////////////////////////////////
#define TYPE_FIX_STEP               1
#define TYPE_FIX_TIME               2

#define STEP_TYPE                   TYPE_FIX_TIME // TYPE_FIX_STEP // 

#if(STEP_TYPE == TYPE_FIX_STEP)
#define LIGHT_ADJUST_STEP           (2)   //unit: lum step 1--100
#define LIGHT_ADJUST_INTERVAL       (2)   // unit :10ms;     min:20ms
#else
#define LIGHT_ADJUST_TIME           (100)   //unit: 10ms
#define LIGHT_ADJUST_INTERVAL       (2)   // unit :10ms;     min:20ms
#endif

extern void light_adjust_RGB_hw(u16 val_R, u16 val_G, u16 val_B, u16 lum);
void light_onoff_normal(u8 on);

extern float calculate_lumen_map(u16 val);
extern u8 light_off;
extern u16 led_lum;
extern u16 led_val[3];
extern ll_adv_rsp_private_t adv_rsp_pri_data;

typedef struct{
    u32 time;
    s32 lum_temp;
    s32 lum_dst;
    u16 step;
    u16 step_mod;
    u16 remainder;
    u8 adjusting_flag;
}light_step_t;

static light_step_t light_step = {};

enum{
    LUM_UP = 0,
    LUM_DOWN,
};

void get_next_lum(u8 direction){    
    u32 temp = light_step.remainder + light_step.step_mod;
    light_step.remainder = (u16)temp;
    
    if(LUM_UP == direction){
        light_step.lum_temp += light_step.step;
        if(temp >= 0x10000){
            light_step.lum_temp += 1;
        }
        if(light_step.lum_temp >= light_step.lum_dst){
            light_step.lum_temp = light_step.lum_dst;
            light_step.remainder = 0;
        }
    }else{
        light_step.lum_temp -= light_step.step;
        if(temp >= 0x10000){
            light_step.lum_temp -= 1;
        }
        if(light_step.lum_temp <= light_step.lum_dst){
            light_step.lum_temp = light_step.lum_dst;
            light_step.remainder = 0;
        }
    }
}

void get_step(u8 direction){
    light_step.remainder = 0;       // reset
    #if(STEP_TYPE == TYPE_FIX_STEP)
    light_step.step = LIGHT_ADJUST_STEP;
    light_step.step_mod = 0;
    #else   // fix time
    if(LUM_UP == direction){
        light_step.step = (light_step.lum_dst - light_step.lum_temp)/(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL);
        light_step.step_mod = (((light_step.lum_dst - light_step.lum_temp)%(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL))*256)/(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL);
    }else{
        light_step.step = (light_step.lum_temp - light_step.lum_dst)/(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL);
        light_step.step_mod = (((light_step.lum_temp - light_step.lum_dst)%(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL))*256)/(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL);
    }
    #endif
}

void light_step_correct_mod(float *pwm_val, u16 lum){
    #if(STEP_TYPE == TYPE_FIX_TIME)
    int temp_pwm = light_step.remainder;
    
    if(light_step.adjusting_flag && (light_step.lum_dst != light_step.lum_temp)
   && (lum > 0)
   && (light_step.remainder)){
        if(light_step.lum_dst > light_step.lum_temp){
            temp_pwm = *pwm_val + (temp_pwm * (calculate_lumen_map(lum+1) - calculate_lumen_map(lum))) / 256;

            if(temp_pwm > U16_MAX){
                temp_pwm = U16_MAX;
            }
        }else{
            temp_pwm = *pwm_val - (temp_pwm * (calculate_lumen_map(lum) - calculate_lumen_map(lum-1))) / 256;
            if(temp_pwm < 0){
                temp_pwm = 0;
            }
        }

        *pwm_val = temp_pwm;
    }
    #endif
}

void light_onoff_step_init()
{
    //light_step.adjusting_flag = 0;
    memset((u8 *)(&light_step), 0, sizeof(light_step));
}

void light_step_reset(u16 target) {
    u8 r = irq_disable();
    if (light_step.adjusting_flag == 0 && target == led_lum) {
        light_adjust_RGB_hw(led_val[0], led_val[1], led_val[2], target);
        goto end;
    }

    if (light_step.adjusting_flag == 1) {
        if (target < light_step.lum_temp) {
            light_step.lum_dst = target;
            get_step(LUM_DOWN);
        }

        if (target > light_step.lum_temp) {
            light_step.lum_dst = target;
            get_step(LUM_UP);
        }
    } else {
        if (target < led_lum) {
            light_step.lum_temp = led_lum;
            light_step.lum_dst = target;
            get_step(LUM_DOWN);
        }

        if (target > led_lum) {
            light_step.lum_temp = led_lum;
            light_step.lum_dst = target;
            get_step(LUM_UP);
        }
    }

    light_step.adjusting_flag = 1;
    light_step.time = 0;
    led_lum = target;

    end:
    irq_restore(r);
}

void light_onoff_step(u8 on){
    if(light_step.adjusting_flag){
        //return ;
    }

    u8 set_flag= 1;
    
    if(on){
        if(light_off){
            if(0 == light_step.adjusting_flag){
                light_step.lum_temp = 0;
            }
            light_step.lum_dst = led_lum;
            get_step(LUM_UP);
    	}else{
    	    set_flag = 0;
    	    light_onoff_normal(1); // make sure on. unnecessary.
    	}
        light_off = 0;
	}else{
        if(light_off){
    	    set_flag = 0;
    	    light_onoff_normal(0); // make sure off. unnecessary.
    	}else{
            if(0 == light_step.adjusting_flag){
                light_step.lum_temp = led_lum;
            }
            light_step.lum_dst = 0;
            get_step(LUM_DOWN);
    	}
        light_off = 1;    
	}
	
    light_step.adjusting_flag = set_flag;
    light_step.time = 0;
}

void light_onoff_step_timer(){
    if(light_step.adjusting_flag){
        if(0 == light_step.time){
            if(light_step.lum_dst != light_step.lum_temp){
                if(light_step.lum_temp < light_step.lum_dst){
                    get_next_lum(LUM_UP);
                }else{
                    get_next_lum(LUM_DOWN);
                }
                light_adjust_RGB_hw(led_val[0], led_val[1], led_val[2], light_step.lum_temp);                
            }else{
                light_step.adjusting_flag = 0;
                memset((u8 *)(&light_step), 0, sizeof(light_step));
            }
        }
        
        light_step.time++;
        if(light_step.time >= LIGHT_ADJUST_INTERVAL){
            light_step.time = 0;
        }
    }
}

u8 is_lum_invalid(u8 lum){
    #define LED_LUM_MIN         5
    if(lum < LED_LUM_MIN){
        return LED_LUM_MIN;
    }else{
        return 0;
    }
}

extern u8 rf_slave_ota_busy;
void pa_init(u8 tx_pin_level, u8 rx_pin_level)
{
#if(PA_ENABLE)
    rf_set_power_level_index (PA_RF_POWER);
    #if(PA_HW)
    rf_rffe_set_pin(PA_TXEN_PIN, PA_RXEN_PIN);
    #else
    gpio_set_func(PA_TXEN_PIN, AS_GPIO);
    gpio_set_input_en(PA_TXEN_PIN, 0);
    gpio_set_output_en(PA_TXEN_PIN, 1);
    gpio_write(PA_TXEN_PIN, tx_pin_level);
    
    gpio_set_func(PA_RXEN_PIN, AS_GPIO);
    gpio_set_input_en(PA_RXEN_PIN, 0);
    gpio_set_output_en(PA_RXEN_PIN, 1);
    gpio_write(PA_RXEN_PIN, tx_pin_level);
    #endif
#endif    
}

#if ((CLOCK_SYS_CLOCK_HZ <= 16000000) && (MCU_CORE_TYPE < MCU_CORE_8258))
_attribute_ram_code_    // relate to scan response
#endif
void pa_txrx(u8 val)
{
#if(PA_ENABLE)
    #if(PA_HW)
    return;
    #else
    if(val == PA_OFF/* || rf_slave_ota_busy*/){
        gpio_write(PA_TXEN_PIN, 0);
        gpio_write(PA_RXEN_PIN, 0);
    }else if(val == PA_TX){
        gpio_write(PA_RXEN_PIN, 0);
        gpio_write(PA_TXEN_PIN, 1);
    }else if(val == PA_RX){
        gpio_write(PA_TXEN_PIN, 0);
        gpio_write(PA_RXEN_PIN, 1);
    }
    #endif
#endif
}

u8 iBeaconInterval = 0;
u8 beacon_with_mesh_adv = 0;// 0 means only send beacon adv pkt;  1 means send both of beacon pkt and mesh adv pkt
#if(IBEACON_ENABLE)
//apple ibeacon
ibeacon_adv_t ibeacon_customer_data = {
	0x02, 0x01, 0x06,		// not connect
	0x1A, 
	0xFF, 
	0x004C, 
	0x1502,
	{0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
	0x3327,
	0x1FBA,
	(7 - 57),			//7-8dbm not 0dbm, Measured power one meter distance, link loss -41dbm
};

u8 eddystone_uid[31] = {
                    0x02, 0x01, 0x06,       // not connect
                    0x03, 0x03, 0xAA, 0xFE, // uuid
                    0x17, 0x16, 0xAA, 0xFE, // UID type's len is 0x17
                        0x00,               // UID type
                        0x08,               // tx power
                        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, // NID
                        0x01, 0x02, 0x03, 0x04, 0x05, 0x06,                         // BID
                        0x00, 0x00          // RFU
                    };
u8 eddystone_url[31] = {
                    0x02, 0x01, 0x06,       // not connect
                    0x03, 0x03, 0xAA, 0xFE, // uuid
                    0x12, 0x16, 0xAA, 0xFE, // URL type's len is variable
                        0x10,               // URL type
                        0x08,               // tx power
                        0x00,               // URL Scheme 0x00-http://www.  0x01-https://www.  0x02-http://  0x03-https://
                        0x74, 0x65, 0x6c, 0x69, 0x6e, 0x6b, 0x2d, 0x73, 0x65, 0x6d, 0x69,// telink-semi
                        0x07,               // 0x07-.com  0x08-.org 0x09-.edu  0x0a-.net............
                    };

u8 eddystone_tlm[31] = {
                    0x02, 0x01, 0x06,       // not connect
                    0x03, 0x03, 0xAA, 0xFE, // uuid
                    0x11, 0x16, 0xAA, 0xFE, // TLM type's len is 0x11
                        0x20,               // TLM type
                        0x00,               // TLM version
                        0x00, 0x00,         // Battery voltage 1mV/bit
                        0x00, 0x80,         // Temperature
                        0x00, 0x00, 0x00, 0x00, // ADV_CNT
                        0x00, 0x00, 0x00, 0x00, // SEC_CNT unit:0.1s
                    };

u8 iBeaconData[31] = {0};
u8 beacon_len = 0;
extern rf_packet_adv_ind_module_t pkt_ibeacon;
extern u8* slave_p_mac;
void set_ibeacon_data(u8 *val, int n){
    pkt_ibeacon.dma_len = n + 8;
    pkt_ibeacon.type = FLG_BLE_ADV_NONCONN_IND;//Set ADV type to non-connectable FLG_BLE_ADV_NONCONN_IND
    pkt_ibeacon.rf_len = n + 6;
	memcpy (pkt_ibeacon.advA, slave_p_mac, 6);
    memcpy (pkt_ibeacon.data, val, n);
}
#endif

u8 flash_user_data[16] = {0};
u8 user_data_idx;					//always pointing to the next block address to be written
#define max_data_block 256			//256*16= 4096
u8 get_user_data_idx(void)
{
	int i;
	u8 data[16];
	u8 data1[16] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
					0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	user_data_idx = 0;
	for (i=0; i<max_data_block; i++)
	{
		flash_read_page((flash_adr_user_data + user_data_idx*16),16,(u8 *)(&data));
		if(!memcmp(data,data1,16))
			break;
		user_data_idx++;
	}
	if(max_data_block == i)
	{
		memcpy(flash_user_data,data,16);
		flash_erase_sector(flash_adr_user_data);
		user_data_idx = 0;
	}
	if(0 == user_data_idx)
	{
		flash_write_page(flash_adr_user_data,16,(u8 *)(&flash_user_data));	//reset
		user_data_idx ++;
	}
	return user_data_idx;
}

void get_flash_user_data(void)		//get data from flash
{
	get_user_data_idx();
	flash_read_page((flash_adr_user_data + (user_data_idx-1)*16),16,(u8 *)(&flash_user_data));
}



void store_user_data_2flash(void)
{
	get_user_data_idx();
	flash_write_page((flash_adr_user_data + user_data_idx*16),16,(u8 *)(&flash_user_data));
	user_data_idx ++;
}

int compare_user_data(void)
{
	u8 temp[16];
	flash_read_page((flash_adr_user_data + (user_data_idx-1)*16),16,(u8 *)(&temp));
	return memcmp(temp,flash_user_data,16);
}


void check_store_user_data(void)	//if flash_user_data changed store it
{
	get_user_data_idx();
	if(!compare_user_data())
		return;							//data not changed
	store_user_data_2flash();
}

#if ((__PROJECT_LIGHT_SWITCH__)         \
    ||(__PROJECT_LPN__)                 \
    ||(__PROJECT_LIGHT_8266__)          \
    ||(__PROJECT_8266_MESH_CONFIG__)    \
    ||(__PROJECT_LIGHT_8267__)          \
    ||(__PROJECT_LIGHT_8269__)          \
    ||(__PROJECT_LIGHT_8258__)          \
    ||(__PROJECT_LIGHT_8278__)          \
    || (__PROJECT_LIGHT_8267_UART__)	\
    || (__PROJECT_LIGHT_GATEWAY__)	\
    ||(__PROJECT_LIGHT_NO_MESH__)          \
    ||(__PROJECT_MASTER_LIGHT_8266__)          \
    ||(__PROJECT_MASTER_LIGHT_8267__)          \
    ||(__PROJECT_MOTIONSENSOR_8267__)   \
    ||(__PROJECT_BLE_MASTER__)          \
    ||(__PROJECT_MONITOR_8266__))

// for ota
extern fp_rf_led_ota_ok 			p_vendor_rf_led_ota_ok;
extern fp_rf_led_ota_error			p_vendor_rf_led_ota_error;
#if (__PROJECT_LIGHT_SWITCH__ || __PROJECT_LPN__ || __PROJECT_MOTIONSENSOR_8267__ || __PROJECT_LIGHT_NO_MESH__)
#define 	OTA_LED				GPIO_LED
#else
#define 	OTA_LED				PWM_R
#endif

#define OTA_ERROR_TEST_EN		0
#if OTA_ERROR_TEST_EN
static u8 ota_err_no = 0;
#define SET_OTA_GATT_ERROR_NUM(error_num)	do{ota_err_no = error_num;}while(0);
#else
#define SET_OTA_GATT_ERROR_NUM(error_num)	
#endif

void rf_led_ota_ok(void){
	if(p_vendor_rf_led_ota_ok){
		p_vendor_rf_led_ota_ok();
		return;
	}
	gpio_set_func(OTA_LED, AS_GPIO);
	gpio_set_output_en(OTA_LED, 1);
	static u8 led_onoff = 1;
	foreach(i, 6){
		gpio_write(OTA_LED, led_onoff);
		led_onoff = !led_onoff;
#if(MODULE_WATCHDOG_ENABLE)
        wd_clear();
#endif
		sleep_us(1000*1000);
	}
}
void rf_led_ota_error(void){
	if(p_vendor_rf_led_ota_error){
		p_vendor_rf_led_ota_error();
		return;
	}
	gpio_set_func(OTA_LED, AS_GPIO);
	gpio_set_output_en(OTA_LED, 1);
	static u8 led_onoff = 1;
	#if OTA_ERROR_TEST_EN
	foreach(i, 60*1000)
	#else
	foreach(i, 60)
	#endif
	{
		gpio_write(OTA_LED, led_onoff);
		led_onoff = !led_onoff;
#if(MODULE_WATCHDOG_ENABLE)
        wd_clear();
#endif
		sleep_us(100*1000);

		#if OTA_ERROR_TEST_EN
		static u8 volatile ota_test_key;
		if(ota_test_key){
			ota_test_key = 0;
			break;
		}
		#endif
	}
}

void mesh_ota_led_cb(u32 type)
{
    if(MESH_OTA_LED_OK == type){
        rf_led_ota_ok();
    }else if(MESH_OTA_LED_ERROR == type){
        rf_led_ota_error();
    }else if(MESH_OTA_LED_STOP == type){
        rf_led_ota_error();
    }
}
#endif

///////////////  light_slave_tx_command call back /////////////////////////////////
void rf_link_data_callback (u8 *);
void light_slave_tx_command_callback (u8 *p){
    #if !(__PROJECT_LIGHT_SWITCH__ || __PROJECT_LPN__)
    rf_link_data_callback(p);
    #endif
}

///////////////  set BLE interval and timeout parameter /////////////////////////////////
const u16 conn_para_data[UPDATE_CONN_PARA_CNT][3] = {{18, 18+16, 200}, {16, 16+16, 200}, {32, 32+16, 200}, {48, 48+16, 200}};
u8 conn_update_cnt = 0;
u8 conn_update_successed = 0;
u8 rf_update_conn_para(u8 * p)
{
    rf_pkt_l2cap_sig_connParaUpRsp_t* pp = (rf_pkt_l2cap_sig_connParaUpRsp_t*)p;
    u8 sig_conn_param_update_rsp[9] = { 0x0A, 0x06, 0x00, 0x05, 0x00, 0x13, 0x01, 0x02, 0x00 };
    if(!memcmp(sig_conn_param_update_rsp, &pp->rf_len, 9) && ((pp->type&0b11) == 2)){//l2cap data pkt, start pkt
        if(pp->result == 0x0000){
            conn_update_cnt = 0;
            conn_update_successed = 1;
        }else if(pp->result == 0x0001){
            if(conn_update_cnt >= UPDATE_CONN_PARA_CNT){
                conn_update_cnt = 0;
            }else{
                setup_ble_parameter_start(1, conn_para_data[conn_update_cnt][0], conn_para_data[conn_update_cnt][1], conn_para_data[conn_update_cnt][2]);
                conn_update_cnt++;
            }
        }
    }

    return 0;
}

/************
 *
 * int setup_ble_parameter_start(u16 delay, u16 interval_min, u16 interval_max, u16 timeout);
 *
 * delay   :  unit: one ble interval 
 * interval_min,interval_max:  if all 0,will keep the system parameter for android but not ios.   unit: 1.25ms; must longer than 20ms.
 * timeout:  if 0,will keep the system parameter.   unit: 10ms; must longer than 3second for steady connect.
 *
 * return 0 means setup parameters is valid.
 * return -1 means parameter of interval is invalid.
 * return -2 means parameter of timeout is invalid.
 *
 *
 * void rf_link_slave_connect_callback()
 * system will call this function when receive command of BLE connect request.
     IOS Note: 
     20 ms <= interval_min
     interval_min + 20 ms <= interval_max <= 2second
     timeout <= 6second
 */
void update_ble_parameter_cb(){
    if(conn_update_successed == 0){
        setup_ble_parameter_start(1, conn_para_data[0][0], conn_para_data[0][1], conn_para_data[0][2]);  // interval 32: means 40ms;   timeout 200: means 2000ms
        conn_update_cnt++;
    }
}

// extern void mesh_listen_chn_set2SIG();

void mesh_listen_chn_restore2def()
{
    memcpy(sys_chn_listen, SYS_CHN_LISTEN_ORG, sizeof(sys_chn_listen));
}

#if(WORK_SLEEP_EN)
u8 work_sleep_en = 1;
u32 start2adv = 10000;//us
u8 need_sleep = 1;
u8 need_sleep_pre = 0;
u8 hb_timeout_pre = 0;
extern u32 slave_listen_interval;
extern u32 tick_per_us;
extern u16 adv_interval2listen_interval;
u32 last_send_hb_time = 0;
u32 last_rcv_hb_time = 0;
void sleep_work_chn(u8 chn_mode){
	if(chn_mode == SLEEP_CHN){
		if(adv_interval2listen_interval != 2){
			adv_interval2listen_interval = 2;
			sys_chn_listen[0] = 2;
			sys_chn_listen[1] = 23;
			sys_chn_listen[2] = 2;
			sys_chn_listen[3] = 23;
		}
	}else if(adv_interval2listen_interval != 4){
		adv_interval2listen_interval = 4;
		mesh_listen_chn_restore2def();
	}
}

void send_duty_cmd(u8 cmd_type){
	if(cmd_type == DUTY_CMD_WORK){
		hb_timeout_pre = need_sleep_pre = need_sleep = 0;
		rf_link_set_max_bridge(CIRCLE_TIME*1000/(slave_listen_interval/tick_per_us)*4);
	}
	u8 op_para[16] = {0};
	op_para[0] = LGT_CMD_WORK_OR_SLEEP;
	op_para[3] = cmd_type;
	light_slave_tx_command(op_para, 0xFFFF);
	if(cmd_type == DUTY_CMD_WORK){
		rf_link_set_max_bridge(BRIDGE_MAX_CNT);
	}
}

void main_loop_sleep_proc(void){
	if(need_sleep_pre && mesh_user_cmd_idx == 0){
		hb_timeout_pre = need_sleep_pre = 0;
		need_sleep = 1;
	}
	
	if(need_sleep){
		sleep_work_chn(SLEEP_CHN);
		static u32 work_time = 0;
		if(0xAB != analog_read (0x3b)){
		    work_time = 1500*1000;
		}else{
            work_time = slave_listen_interval/tick_per_us*adv_interval2listen_interval;
		}
		static u32 last_work_time = 0;
		if(last_work_time == 0){
			last_work_time = clock_time() | 1;
		}
		if(clock_time_exceed(last_work_time, work_time)){
			last_work_time = 0;
			analog_write(0x3b, 0xAB);
			cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM, PM_WAKEUP_TIMER, clock_time() + (CIRCLE_TIME*1000-work_time)*tick_per_us);
			while(1);
		}
	}else{
		sleep_work_chn(WORK_CHN);
		extern u8 pair_login_ok;
		if(pair_login_ok && mesh_user_cmd_idx == 0){
			// send hb pkt
			if(last_send_hb_time == 0){
				last_send_hb_time = clock_time() | 1;
			}
			if(clock_time_exceed(last_send_hb_time, CIRCLE_TIME*1000)){
				last_send_hb_time = clock_time() | 1;
				extern void send_duty_cmd(u8 cmd_type);
				send_duty_cmd(DUTY_CMD_HB);
			}
		}else{
			if(last_rcv_hb_time == 0){
				last_rcv_hb_time = clock_time() | 1;
			}
			if(clock_time_exceed(last_rcv_hb_time, CIRCLE_TIME*HB_TIME_OUT/2*1000)){
				if(hb_timeout_pre == 0){
					hb_timeout_pre = 1;
					last_rcv_hb_time = clock_time() | 1;
				}else{
					hb_timeout_pre = 0;
					need_sleep = 1;
#if 0
					static u32 a_cur_time = 0;
					static u32 a_last_time = 0;
					a_cur_time = clock_time();
					a_last_time = last_rcv_hb_time;
					extern void light_onoff_hw(u8 on);
					light_onoff_hw(0);
					static u8 a_cnt = 0;
					while(1){
						sleep_us(100*1000);
						++a_cnt;
#if(MODULE_WATCHDOG_ENABLE)
						wd_clear();
#endif
					}
#endif
				}
			}
		}
	}
}
#else
u8 work_sleep_en = 0;
u32 start2adv = 0;//us
#endif

void rf_link_slave_connect_callback(){
#if DUAL_MODE_ADAPT_EN
    mesh_service_change_report();
#endif

#if(WORK_SLEEP_EN)
	send_duty_cmd(DUTY_CMD_WORK);
#endif
}

void cb_ble_slave_disconnect(){
#if(WORK_SLEEP_EN)
	send_duty_cmd(DUTY_CMD_SLEEP);//not use yet
#endif
}

#ifndef PWM_R
#define PWM_R   0
#endif
#ifndef PWM_G
#define PWM_G   0
#endif
#ifndef PWM_B
#define PWM_B   0
#endif
#ifndef PWM_W
#define PWM_W   0
#endif
#ifndef PWM_Y
#define PWM_Y   0
#endif

/*
disable for 8258 to reduce suspend current (1.2mA) and working current (0.4mA)
*/
void pwm_io_input_disable() 
{
	u32 pwm_pin[] = {PWM_R, PWM_G, PWM_B, PWM_W, PWM_Y};
	foreach_arr(i,pwm_pin){
	    if(pwm_pin[i]){
	        gpio_set_input_en(pwm_pin[i], 0);
	    }
	}
}

// duty cycle simple flow
#if 1
int	rf_link_time_allow_1 (u32 us)
{
    return 1;
    
	u32 t = reg_system_tick_irq - clock_time () - us * 16;
	return t < BIT(30);
}

#if 1   // better
#define DUTY_CYCLE_SLEEP_US         (40*1000)
#define DUTY_CYCLE_US               (200*1000)
#elif 0
#define DUTY_CYCLE_SLEEP_US         (10*1000)
#define DUTY_CYCLE_US               (50*1000)
#else
#define DUTY_CYCLE_SLEEP_US         (70*1000)
#define DUTY_CYCLE_US               (350*1000)
#endif
#define DUTY_RAND_MAX               (20*1000)

STATIC_ASSERT((DUTY_RAND_MAX) > 0 && (DUTY_RAND_MAX < DUTY_CYCLE_US));

extern int				slave_link_state;

void simple_sleep_duty_cycle()
{
    // connected
    if((FLG_SYS_LINK_BRIDGE == slave_link_state)||(FLG_SYS_LINK_LISTEN == slave_link_state)){
        static u32 tick_duty_conn = 0;
        static u16 rand_duty_us = 0;
        if(clock_time_exceed(tick_duty_conn,rand_duty_us + DUTY_CYCLE_US - DUTY_RAND_MAX/2) && rf_link_time_allow_1(DUTY_CYCLE_SLEEP_US+1000)){
            u8 r = irq_disable();
            if(rf_link_time_allow_1(DUTY_CYCLE_SLEEP_US+800)){    // check again
                tick_duty_conn = clock_time();
                rand_duty_us = rand()%DUTY_RAND_MAX;
                cpu_sleep_wakeup(0, PM_WAKEUP_TIMER, clock_time() + DUTY_CYCLE_SLEEP_US * CLOCK_SYS_CLOCK_1US);
                reg_rf_irq_status = FLD_RF_IRQ_RX;  // clear rx
            }
            irq_restore(r);
        }
    }
}
#endif

void cb_pair_failed(){
}

// system will call p_cb_ble_slave_disconnect() when BLE disconnect.
cb_func_void_t	p_cb_ble_slave_disconnect = 0;  // cb_ble_slave_disconnect  //
void register_disconnect_cb(void *p)
{
	p_cb_ble_slave_disconnect = p;
}

// system will call p_cb_pair_failed() when login failed or set mesh name/password/ltk failed.
cb_func_void_t	p_cb_pair_failed = 0;   // cb_pair_failed   //

/************
 * u8 get_setup_ble_parameter_result()
 *
 * return 0 means setup parameters fail.
 * return 1 means setup parameters success.
 */
 
u8 get_setup_ble_parameter_result(){
    extern u8 update_ble_par_success_flag;
    return update_ble_par_success_flag;
}

////////////////// gate way /////////////////////////////////////
int rf_link_slave_data (rf_packet_ll_data_t *p, u32 t);

extern u8 	slave_link_connected;
extern u8   pair_login_ok;
extern u8   not_need_login;

typedef struct{
    u8 sno[3];
    u8 src[2];
    u8 dst[2];
    u8 op;
    u16 vendor_id;
    u8 par[10];
}app_cmd_value_t;

typedef struct{
	u32 dma_len;
	u8	type;
	u8  rf_len;
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 handle;
	u8 handle1;
	app_cmd_value_t app_cmd_v;
	u8 rsv[10];
}rf_packet_ll_app_t;

#define MASTER_CMD_MESH_CMD                 0x00
#define MASTER_CMD_ADD_DEVICE               0x24

#define SLAVE_CMD_ACK                           0x25
#define MASTER_CMD_SET_GW_MESH_NAME             0x26
#define MASTER_CMD_SET_GW_MESH_PASSWORD         0x27
#define MASTER_CMD_SET_GW_MESH_LTK              0x28
#define MASTER_CMD_TAKE_EFFECT                  0x29
#define TACK_EFFECT_ONLY_SET_GATEWAY            0x00
#define TACK_EFFECT_ADD_CONFIGURED_DEVICE       0x01    /* Add node which has been configured */
#define TACK_EFFECT_MESH_PAIR                   0x02
#define MASTER_CMD_START_PROV_NODE              0x2a
#define MASTER_CMD_GET_NETWORK_INFO             0x2b
#define SLAVE_NETWORK_INFO_ACK                  0x2c
#define MASTER_CMD_SWITCH_TO_DEFAULT_MESH       0x2d

#define GATEWAY_EVENT_MESH                        0x81
#define GATEWAY_EVENT_NEW_DEVICE_FOUND            0x82
#define GATEWAY_EVENT_PROVISION_COMPLETE          0x83
#define GATEWAY_EVENT_PROVISION_BY_OTHERS         0x84

#if PROVISIONING_ENABLE
gateway_status_t gateway_status = GATEWAY_STATUS_NORMAL;
#endif

#if GATEWAY_EN

#if PROVISIONING_ENABLE
extern my_fifo_t hci_rx_fifo;
extern my_fifo_t hci_tx_fifo;

u8 scan_dev_time_out;

typedef enum
{
    MASTER_CMD_ERR_SUCCESS = 0,
    MASTER_CMD_ERR_INVALID_PARA,
    MASTER_CMD_ERR_INVALID_STATE,
    MASTER_CMD_ERR_UNKNOW_CMD,
}master_cmd_err_t;

extern u8	buff_command[];

u16 crc_total(u8 *p_name, u8 *p_pw, u8 *p_ltk)
{
    unsigned char buf[16 * 3], *p_dat;
    int len;
    int str_len;

    /* Init */
    p_dat = buf;
    len   = 0;

    /* Copy name to buf */
    str_len = 16;
    if(!p_name[15])
    {
        str_len = strlen((char*)p_name);
    }
    memcpy(p_dat, p_name, str_len);
    p_dat += str_len;
    len   += str_len;

    /* Copy password to buffer */
    str_len = 16;
    if(!p_pw[15])
    {
        str_len = strlen((char*)p_pw);
    }
    memcpy(p_dat, p_pw, str_len);
    p_dat += str_len;
    len   += str_len;
    
    /* Copy LTK to buffer */
    memcpy(p_dat, p_ltk, 16);
    
    len += 16;
    
    return crc16(buf, len);
}

int host_write_gateway_provision(int n)
{
	extern u8 pair_ltk_mesh[16];
	u8 buf[64];
	rf_packet_ll_control_t *p_packet;
	u8 cmd = buff_command[0];

	if(cmd <= 3)
	{
		return 0;
	}
	
	p_packet = (rf_packet_ll_control_t*)buf;
	master_cmd_err_t err = MASTER_CMD_ERR_SUCCESS;

    if(gateway_status == GATEWAY_STATUS_TEMP_DEFALT_MESH && cmd != MASTER_CMD_ADD_DEVICE)
    {
		err = MASTER_CMD_ERR_INVALID_STATE;
    }
    if(gateway_status >= GATEWAY_STATUS_SWITCH_TO_DEFAULT_MESH ||gateway_status == GATEWAY_STATUS_NODE_ROLE)
	{
		err = MASTER_CMD_ERR_INVALID_STATE;
	}

	if(err == MASTER_CMD_ERR_SUCCESS){
		switch(cmd)
		{
			case MASTER_CMD_ADD_DEVICE:
				/* Directly send change default mesh command */
				memset(buf, 0x00, 16);
				buf[0] = LGT_CMD_MESH_PAIR;
				buf[3] = MESH_PAIR_DEFAULT_MESH;
	            buf[4] = buff_command[1]; /* Default mesh keep time */

	            if((buff_command[1]) && (buff_command[1] != 0xff) && buff_command[1] <= buff_command[2])
				{
					err = MASTER_CMD_ERR_INVALID_PARA;
					break;
				}

	            if(!buff_command[1] || \
	                buff_command[1] == 0xff)
	            {
	                if(buff_command[2] != buff_command[1])
	                {
	                    err = MASTER_CMD_ERR_INVALID_PARA;
	                    break;
	                }
	            }
	            
				if(!light_slave_tx_command(buf, 0xffff))
				{
					err = MASTER_CMD_ERR_INVALID_STATE;
				}
				else
				{
					extern u32 default_mesh_time_ref;
					/* Gateway shall wait one packet send out */
					default_mesh_time_ref = clock_time() | 1;
					/* One pakcet sending time is 320ms. */
					extern u32 default_mesh_time;
					default_mesh_time	  = 500;
					scan_dev_time_out     = buff_command[2];
	                gateway_status        = GATEWAY_STATUS_SWITCH_TO_DEFAULT_MESH;
				}
				break;
				
			case MASTER_CMD_SET_GW_MESH_NAME:
				if(n - 1 > 16)
				{
					err = MASTER_CMD_ERR_INVALID_PARA;
				}
				else
				{
					memset(new_mesh_name, 0x00, 16);
					memcpy(new_mesh_name, buff_command + 1, n - 1);
				} 
				break;
				
			case MASTER_CMD_SET_GW_MESH_PASSWORD:
				if(n - 1 > 16)
				{
					err = MASTER_CMD_ERR_INVALID_PARA;
				}
				else
				{
					memset(new_mesh_pwd, 0x00, 16);
					memcpy(new_mesh_pwd, buff_command + 1, n - 1);
				} 
				break;
				
			case MASTER_CMD_SET_GW_MESH_LTK:
				if(n - 1 != 16)
				{
					err = MASTER_CMD_ERR_INVALID_PARA;
				}
				else
				{
					memset(new_mesh_ltk, 0x00, 16);
					memcpy(new_mesh_ltk, buff_command + 1, n - 1);
				} 
				break;
				
			case MASTER_CMD_TAKE_EFFECT:
				/* 1bytes Flag 0 for gateway itself ,1 for temporary 2Bytes crc16 MSB */
				/* This command use for set gateway itself */  

				/* Check if CRC is equal */
				if(n == 2 || ((((buff_command[2] << 8) &0xff00) | buff_command[3]) == \
					crc_total(new_mesh_name, new_mesh_pwd, new_mesh_ltk) && n == 4))
				{
				    if(buff_command[1] == TACK_EFFECT_MESH_PAIR)
                    {
                        if(mesh_pair_enable)
						{
							/* Change Access code */
                            memcpy(pair_nn, new_mesh_name, 16);
						    memcpy(pair_pass, new_mesh_pwd, 16);
                            memcpy(pair_ltk_mesh, new_mesh_ltk, 16);
                            
							pair_setting_flag = PAIR_SET_MESH_TX_START;
							gateway_status = GATEWAY_STATUS_CFG_CUR_NETWORK;
						}
						else
						{
							err = MASTER_CMD_ERR_INVALID_STATE;
						}
                    }
					else if(buff_command[1] == TACK_EFFECT_ADD_CONFIGURED_DEVICE)
					{
						/* Only provisioning, not change current network.
						   Shall retore pair_ltk , when provisioning finished */
						memcpy(pair_ltk_mesh, pair_ltk, 16);
						memcpy(pair_ltk, new_mesh_ltk, 16);
						/* No need save to flash */
						if(mesh_pair_enable)
						{
							/* Change Access code */
							extern u32 access_code(u8 *p_name, u8 *p_pw);
							pair_ac = access_code(new_mesh_name, new_mesh_pwd);
							pair_setting_flag = PAIR_SET_MESH_TX_START;
							gateway_status = GATEWAY_STATUS_ADD_CONFIGURED_DEVICE;
						}
						else
						{
							/* Do nothing. If buff_command[4] equal 1, mesh_pair_enable must be none-zero */
							pair_setting_flag = PAIR_SETTED;
						}
					}
					else if(buff_command[1] == TACK_EFFECT_ONLY_SET_GATEWAY)
					{
						if(gateway_status == GATEWAY_STATUS_CFG_UNPRO_DEV)
						{
							err = MASTER_CMD_ERR_INVALID_STATE;
						}
						else
						{
							memcpy(pair_nn, new_mesh_name, 16);
							memcpy(pair_pass, new_mesh_pwd, 16);
							memcpy(pair_ltk, new_mesh_ltk, 16);
							/* Set gateway itsef */
							pair_setting_flag = PAIR_SETTED;
							/* When APP and mcu-master configures the gateway at the same time,
							   mcu-master's priority is higher.  */
							effect_new_mesh = 0;
							extern u32 default_mesh_time_ref;
							default_mesh_time_ref = 0;
							save_effect_new_mesh();
						}
					}
                    else
                    {
                        err = MASTER_CMD_ERR_INVALID_PARA;
                    }
					
				}
				else
				{
					err = MASTER_CMD_ERR_INVALID_PARA;
				}
				
				break;
			case MASTER_CMD_GET_NETWORK_INFO:
				switch(buff_command[1])
				{
					case MASTER_CMD_SET_GW_MESH_NAME:
						p_packet->rf_len = 4 + (pair_nn[15] ? 16 : strlen((char*)pair_nn));
						p_packet->opcode = SLAVE_NETWORK_INFO_ACK;
						p_packet->dat[0] = buff_command[1];
						p_packet->dat[1] = err;
						memcpy(p_packet->dat + 2, pair_nn, pair_nn[15] ? 16 : strlen((char*)pair_nn));
						my_fifo_push_hci_tx(&p_packet->rf_len, p_packet->rf_len);
						break;
						
					case MASTER_CMD_SET_GW_MESH_PASSWORD:
						p_packet->rf_len = 4 + (pair_pass[15] ? 16 : strlen((char*)pair_pass));
						p_packet->opcode = SLAVE_NETWORK_INFO_ACK;
						p_packet->dat[0] = buff_command[1];
						p_packet->dat[1] = err;
						memcpy(p_packet->dat + 2, pair_pass, pair_pass[15] ? 16 : strlen((char*)pair_pass));
						my_fifo_push_hci_tx(&p_packet->rf_len, p_packet->rf_len);
						break;
						
					case MASTER_CMD_SET_GW_MESH_LTK:
						p_packet->rf_len = 4 + 16;
						p_packet->opcode = SLAVE_NETWORK_INFO_ACK;
						p_packet->dat[0] = buff_command[1];
						p_packet->dat[1] = err;
						memcpy(p_packet->dat + 2, pair_ltk, 16);
						my_fifo_push_hci_tx(&p_packet->rf_len, p_packet->rf_len);
						break;
	                default:
						p_packet->rf_len = 4;
						p_packet->opcode = SLAVE_NETWORK_INFO_ACK;
						p_packet->dat[0] = buff_command[1];
						p_packet->dat[1] = MASTER_CMD_ERR_UNKNOW_CMD;
						my_fifo_push_hci_tx(&p_packet->rf_len, p_packet->rf_len);
	                    break;
				}
				return 0;
			default:
				err = MASTER_CMD_ERR_UNKNOW_CMD;
				break;
		}
	}
	
	/* SPI ACK */
	p_packet->rf_len = 4;
	p_packet->opcode = SLAVE_CMD_ACK;
	p_packet->dat[0] = cmd;
	p_packet->dat[1] = err;
	my_fifo_push_hci_tx(&p_packet->rf_len, p_packet->rf_len);

	return 0;
}
#endif

u32 gateway_cmd_sno;

u8          mode_master = 0;

#if (HCI_ACCESS == HCI_USE_UART)
#define GATEWAY_REPORT_INTERVAL_MS      (10)
#else   // usb
#define GATEWAY_REPORT_INTERVAL_MS      (20)    // min: 18ms
#endif

void rf_link_slave_data_app(rf_packet_ll_app_t *pkt_app)
{
    u8 r = irq_disable();
    if(gateway_en && !gateway_security){
        not_need_login = pair_login_ok = 1;
    }
    rf_link_slave_data((rf_packet_ll_data_t *)(pkt_app), 0);
    irq_restore(r);
}
/*///////////////////////////////////////////////////
int	ble_gateway_ll_data (u8 *p, int n);

p[0] : rf_packet_att_cmd_t.type
n : length of input buffer, max is 29
///////////////////////////////////////////////////*/
int	ble_gateway_ll_data (u8 *p, int n)
{
	if (1) {
	    rf_packet_ll_app_t  pkt_app_data = {};
	    memset (&pkt_app_data, 0, sizeof(pkt_app_data));
		memcpy (&pkt_app_data.type, p, n);
		pkt_app_data.dma_len = n;			// recalculate to make sure length is ok
		pkt_app_data.rf_len = n - 2;        // recalculate to make sure length is ok
		if (pkt_app_data.type < 3)
		{
			pkt_app_data.l2capLen = n - 6;  // recalculate to make sure length is ok
		}

        rf_link_slave_data_app(&pkt_app_data);
		return 1;
	}
	return 0;
}

#if 1    // for test
int gatway_tx_command(u8 cmd, u16 dst_adr, u8 *par, u8 par_len, u8 op){ // should call in fp_gateway_rx_proc();
    if(par_len > (is_cmd_long_par(cmd) ? 15 : 10)){
        return -1;
    }

    // packet 
	rf_packet_ll_app_t  pkt_app_data = {};
    memset(&pkt_app_data, 0, sizeof(pkt_app_data));
    pkt_app_data.type = 0x02;
    pkt_app_data.rf_len = 17 + par_len;
    pkt_app_data.dma_len = pkt_app_data.rf_len + 2;
    pkt_app_data.l2capLen = pkt_app_data.rf_len - 4;
    pkt_app_data.chanId = 0x04;
    pkt_app_data.opcode = op;
    pkt_app_data.handle= 0x15;
    pkt_app_data.handle1 = 0x00;
    
    gateway_cmd_sno++;
    memcpy(pkt_app_data.app_cmd_v.sno, &gateway_cmd_sno, 3);
    //memcpy(pkt_app_data.app_cmd_v.src, &device_address, 2);
    memcpy(pkt_app_data.app_cmd_v.dst, &dst_adr, 2);
    pkt_app_data.app_cmd_v.op = (cmd & 0x3F) | 0xC0;
    pkt_app_data.app_cmd_v.vendor_id = VENDOR_ID;
    memcpy(pkt_app_data.app_cmd_v.par, par, par_len);
    
    // send command
    rf_link_slave_data_app(&pkt_app_data);
    
    return 0;
}

// update firmware for itself
int gatway_local_OTA(u8 *par, u8 par_len){ // should call in fp_gateway_rx_proc();
    if(par_len > 20){
        return -1;
    }

    // packet 
	rf_packet_ll_app_t  pkt_app_data = {};
    memset(&pkt_app_data, 0, sizeof(pkt_app_data));
    pkt_app_data.type = 0x02;
    pkt_app_data.rf_len = 7 + par_len;
    pkt_app_data.dma_len = pkt_app_data.rf_len + 2;
    pkt_app_data.l2capLen = pkt_app_data.rf_len - 4;
    pkt_app_data.chanId = 0x04;
    pkt_app_data.opcode = ATT_OP_WRITE_CMD;
    pkt_app_data.handle= 0x18;
    pkt_app_data.handle1 = 0x00;
    
    memcpy(&pkt_app_data.app_cmd_v, par, par_len);
    
    // send command
    rf_link_slave_data_app(&pkt_app_data);
    
    return 0;
}

void gatway_tx_command_test(){
    u8 par[10] = {0};   // must init to zero;  max size of par is 10
    u16 dst_adr = 0xFFFF;
    u8 cmd = LGT_CMD_LIGHT_ONOFF;
    
    static u32 tx_cmd_cnt;
    tx_cmd_cnt++;
    if(tx_cmd_cnt & 1){
        par[0] = LIGHT_ON_PARAM;
    }else{
        par[0] = LIGHT_OFF_PARAM;
    }
    
    gatway_tx_command(cmd, dst_adr, par, sizeof(par), ATT_OP_WRITE_REQ);
}

///////////////  sample function of MASTER UART handle received data  /////////////////////////////////
/************
 * void    gateway_uart_host_handle(void *pkt_uart);
 *
 * pkt_uart: pointer to UART data
 *
 * MASTER UART will call this function when receive data from gateway.
 */
void    gateway_uart_host_handle(void *pkt_uart)
{
    rf_packet_ll_app_t *pkt_rsp = CONTAINER_OF(pkt_uart, rf_packet_ll_app_t, l2capLen);
    if(0x04 == pkt_rsp->chanId){   // LLID : 2
        if(ATT_OP_WRITE_RSP == pkt_rsp->opcode){
            //
        }else if(ATT_OP_READ_RSP == pkt_rsp->opcode){
            //
        }else if(ATT_OP_HANDLE_VALUE_NOTI == pkt_rsp->opcode){  // notify pkt
            //
        }
    }
}
#endif

#define     EP_BO           5
#define		USB_ENDPOINT_BULK_IN			8
#define		USB_ENDPOINT_BULK_OUT			5
#define		USB_ENDPOINT_BULK_OUT_FLAG		(1 << (USB_ENDPOINT_BULK_OUT & 7))

static inline u32 usb_endpoint_busy(u32 ep) {
#if((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
	write_reg8 (0x80013d, 0);
	return read_reg8 (0x800120 + (ep&7)) & 1;
#else
	return (reg_usb_ep_ctrl(ep) & FLD_USB_EP_BUSY);
#endif
}

void gateway_hci_busy_clr()
{
#if (HCI_ACCESS == HCI_USE_UART)
    uart_tx_busy_clear();
#else   // usb
    reg_usb_ep_ctrl(USB_ENDPOINT_BULK_IN) = FLD_USB_EP_BUSY;
#endif
}

int gateway_hci_busy()
{
    int busy = 0;
#if (HCI_ACCESS == HCI_USE_UART)
    busy = uart_tx_busy_check();
#else   // usb
    busy = usb_endpoint_busy(USB_ENDPOINT_BULK_IN);
#endif
    static u8 hci_busy_start;
    static u32 hci_busy_tick;
    if(busy){
        if(0 == hci_busy_start){
            hci_busy_start = 1;
            hci_busy_tick = clock_time();
        }
        if(clock_time_exceed(hci_busy_tick, 1000*1000)){
            hci_busy_start = 0;
            gateway_hci_busy_clr();
            return 0;
        }
    }else{
        hci_busy_start = 0;
    }
    
    return busy;
}

// Note: check differrence IO of button for differrence PCB
void	proc_ui_gateway ()
{
    if(gateway_en && rf_slave_ota_busy){
        u8 r = irq_disable();   // must
        fp_gateway_rx_proc();   // handle ota data as soon as possible
        irq_restore(r);
    }
    
    if(gateway_en){             // report notify as soon as possible
        static u32 gateway_report_tick;
        if((0 == gateway_hci_busy()) 
        && clock_time_exceed(gateway_report_tick, GATEWAY_REPORT_INTERVAL_MS*1000)
        ){
        #if !PROVISIONING_ENABLE
            if(gateway_report_poll()){
                gateway_report_tick = clock_time();
            }
        #else
            /* Gateway report and command response 
               send data as the same way */
    		u8 r = irq_disable();   // must
    		if(my_fifo_wptr(&hci_tx_fifo)){	// make sure hci tx fifo is not full
	            gateway_report_poll();
	        }
	        irq_restore(r);
	        
            if(hci_tx_fifo_poll()){
                gateway_report_tick = clock_time();
            }
        #endif         
        }
    }

	static u32 tick;
	if (!clock_time_exceed (tick, 40000))
	{
		return;
	}
	tick = clock_time();
	
    if(!gateway_en){
        // when in gateway mode, proccess in irq_st_ble_rx(), so it just receive BLE command in irq_st_ble_rx().
        fp_gateway_rx_proc();
    }
    
    #if (!(__PROJECT_MASTER_LIGHT_8266__ || __PROJECT_MASTER_LIGHT_8267__))	// because button function use for switch master/slave mode in master light
	static u8 st = 0;
	u8 s = !gpio_read (SWITCH_MODE_BUTTON1) || !gpio_read(SWITCH_MODE_BUTTON2);
	if ((!st) & s)
	{
	    #if 0
	    gatway_tx_command_test();      // for test
	    #else
	    if(0 == gateway_mode_onoff(!gateway_en)){
            rf_link_light_event_callback(LGT_CMD_DEL_PAIR);
	    }
	    #endif
	}
	st = s;
	#endif
}

const rf_packet_ll_init_t	pkt_gateway_init = {
		sizeof (rf_packet_ll_init_t) - 4,		// dma_len
		FLG_BLE_LIGHT_CONNECT_REQ,				// type
		sizeof (rf_packet_ll_init_t) - 6,		// rf_len
		{0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5},	// scanA
		{0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5},	// advA
		{0xd6, 0xbe, 0x89, 0x8e},				// access code
		{0x8d, 0x26, 0xd8},						// crcinit[3]
		0x02,									// wsize
		0x0005,									// woffset
		0x0020,									// interval: 32 * 1.25 ms = 40 ms
		0x0000,									// latency
		0x0032,									// timeout: 50*10 ms = 500ms
		{0xff, 0xff, 0xff, 0xff, 0x1f},			// chm[5]
		0xac,									// hop: initial channel - 12
};


void	ble_master_data_callback (u8 *p)
{
	static	u32 bdbg_data;
	bdbg_data++;
	/*static */u8 buf[48];
	memset4(buf, 0, sizeof(buf));

	rf_packet_ll_app_t *pkt_rsp;
	if(mode_master){
	    pkt_rsp = (rf_packet_ll_app_t *)(p+4);
	}else{
	    pkt_rsp = (rf_packet_ll_app_t *)p;
	}

	u8 rf_len = pkt_rsp->rf_len;
#if PROVISIONING_ENABLE
	u8 push_len = rf_len + 2;
	buf[0] = push_len;              	// add len and event flag; not include length of UART
	buf[1] = GATEWAY_EVENT_MESH;		// event flag
	memcpy(buf + 2, &pkt_rsp->l2capLen, rf_len);
	my_fifo_push_hci_tx(buf, push_len);
#else
	buf[0] = rf_len;                  	// buf[0-3]: length of UART
	memcpy(buf+4, &pkt_rsp->l2capLen, rf_len);
	
#if (HCI_ACCESS == HCI_USE_UART)
#if UART_FIFO_ENABLE
    extern int my_fifo_push_hci_tx(unsigned char *para, unsigned short len);
    u8 tmp_x[64];

    tmp_x[0] = rf_len + 8;   // length of USB data
	tmp_x[1] = 0;
	u8 *pdata = buf+4;              // same witch (&pkt_rsp->l2capLen)
	for (int i=0; i < rf_len + 5; i++) // 5 = DC offset (2 BYTE) + CRC (3BYTE)
	{
		tmp_x[2 + i] = pdata[i];
	}
	if(mode_master){
	    tmp_x[2 + rf_len + 5] = p[0];     //rssi
	}else{
	    tmp_x[2 + rf_len + 5] = 0;        //rssi
	}
    
    my_fifo_push_hci_tx(tmp_x, rf_len + 8);
#else
	uart_Send((u8*)(buf));          // UART MASTER would call gateway_uart_host_handle() to handle this data
#endif
#else
	#if ((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
	write_reg8 (0x800110 + (USB_ENDPOINT_BULK_IN & 7), 0);//reset pointer to 0
	#endif
	reg_usb_ep8_dat = rf_len + 8;   // length of USB data
	reg_usb_ep8_dat = 0;
	u8 *pdata = buf+4;              // same witch (&pkt_rsp->l2capLen)
	for (int i=0; i < rf_len + 5; i++) // 5 = DC offset (2 BYTE) + CRC (3BYTE)
	{
		reg_usb_ep8_dat = pdata[i];
	}
	if(mode_master){
	    reg_usb_ep8_dat = p[0];     //rssi
	}else{
	    reg_usb_ep8_dat = 0;        //rssi
	}
	
	#if ((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
	write_reg8 (0x800120 + (USB_ENDPOINT_BULK_IN & 7), 1);		//ACK
	#else
	reg_usb_ep8_ctrl = BIT(7);
	#endif
#endif
#endif
}

/*
ble_event_callback() is only needed if it is  security mode between master and gateway.
*/
void ble_event_callback (u8 status, u8 *p, u8 rssi)
{
	static u32 bdbg_event;
	bdbg_event++;

#if (__PROJECT_MASTER_LIGHT_8266__ || __PROJECT_MASTER_LIGHT_8267__)
	if(FLG_SYS_LINK_LOST == status){
		extern int host_ota_start; 
		host_ota_start = 0;			// must, if not, it can not reconnect, when host_ota_start not zero. 
	}
	if(FLG_SYS_DEVICE_SCAN_TIMEOUT == status){// u32 master_scan_timeut: default is 10*1000 ms
		
	}
#endif

#if (__PROJECT_LIGHT_8267_UART__ || (HCI_ACCESS == HCI_USE_UART))
	#if UART_FIFO_ENABLE// for sample
    u8 buf[64];
    int n = 0;
	if (p)
	{
		n = *p++;
	}
	buf[0] = n + 8;
    buf[1] = status;
    
	for (int i=0; i < n + 5; i++)
	{
		buf[2 + i] = *p++;
	}
	buf[2 + n + 5] = rssi;

	extern int my_fifo_push_hci_tx(unsigned char *para, unsigned short len);
    my_fifo_push_hci_tx(buf, 8 + n);
    #endif
    #if 0
	/*static */u8 buf[48];
	memset4(buf, 0, sizeof(buf));
	rf_packet_adv_t *p_adv = CONTAINER_OF(p,rf_packet_adv_t,rf_len);
	buf[0] = p_adv->rf_len + 2; 					 // buf[0-3]: length of UART
	memcpy(buf+4, &p_adv->type, buf[0]);
	uart_Send((u8*)(buf));
	#endif
#else
	//////////////////////////////////////////////
	int n = 0;
	if (p)
	{
		n = *p++;
	}
	#if (MCU_CORE_TYPE == MCU_CORE_8258 || MCU_CORE_TYPE == MCU_CORE_8278)
	write_reg8 (0x800110 + (USB_ENDPOINT_BULK_IN & 7), 0);//reset pointer to 0
	#endif
	reg_usb_ep8_dat = n + 8;
	reg_usb_ep8_dat = status;
	for (int i=0; i<n+5; i++)
	{
		reg_usb_ep8_dat = *p++;
	}
	reg_usb_ep8_dat = rssi;
	#if (MCU_CORE_TYPE == MCU_CORE_8258 || MCU_CORE_TYPE == MCU_CORE_8278)
	write_reg8 (0x800120 + (USB_ENDPOINT_BULK_IN & 7), 1);		//ACK
	#else
	reg_usb_ep8_ctrl = BIT(7);
	#endif
#endif
	///////////////////////////////////////////////
}


//////////////////////////////////////////////////////////
//	USB interfuace BI/BO
//////////////////////////////////////////////////////////
u8	buff_command[64];
int host_write_gateway (int n)
{
	static u32 no_cmd;
	no_cmd++;

	u8 cmd = buff_command[0];

	if (cmd == 0 && n > 0)		// data
	{
	    if(gateway_en){
		    ble_gateway_ll_data (buff_command + 1, n - 1);
		}
	}
	else if (cmd == 2)	// start master
	{
	    extern rf_packet_adv_ind_module_t	pkt_adv;
	    // report adv packet to PC, because master need MAC of gate way in security mode
	    ble_event_callback (FLG_SYS_DEVICE_FOUND, (u8 *)(&(pkt_adv.rf_len)), 0);
	}
	else if (cmd == 3)	// stop master
	{
	}
	else
	{
		#if PROVISIONING_ENABLE
		host_write_gateway_provision(n);
		#endif
	}
    
	return 0;
}


int usb_bulk_out_get_data (u8 *p)
{
	//if (read_reg8 (0x800139) & USB_ENDPOINT_BULK_OUT_FLAG)
	{
		//clear interrupt flag
		write_reg8 (0x800139, USB_ENDPOINT_BULK_OUT_FLAG);

		// read data
		int n = read_reg8 (0x800110 + (USB_ENDPOINT_BULK_OUT & 7));
		write_reg8 (0x800110 + (USB_ENDPOINT_BULK_OUT & 7), 0);
		for (int i=0; i<n; i++)
		{
			p[i] = read_reg8 (0x800118 + (USB_ENDPOINT_BULK_OUT & 7));
		}
		write_reg8 (0x800120 + (USB_ENDPOINT_BULK_OUT & 7), 1);		//ACK

		return n;
	}
	//return 0;
}


void host_init ()
{
#if ((MCU_CORE_TYPE != MCU_CORE_8258) && (MCU_CORE_TYPE != MCU_CORE_8278))	// 
	/////////// ID initialization for host control software //////////
	u16 dongle_usb_id = 0x82bd;
	if(REG_ADDR16(0x7e) == 0x5327){
		dongle_usb_id = 0x82bc; // fix write error
	}
	REG_ADDR8(0x74) = 0x53;
	REG_ADDR16(0x7e) = dongle_usb_id;
	REG_ADDR8(0x74) = 0x00;

	/////////// enable USB device /////////////////////////////////////
	#if (MCU_CORE_TYPE == MCU_CORE_8267 || MCU_CORE_TYPE == MCU_CORE_8269)    // 
	gpio_setup_up_down_resistor(GPIO_DP, PM_PIN_PULLUP_10K);
	usb_dp_pullup_en (0);    // fix 8267 A0 error:1.5K pull up
    #else
	usb_dp_pullup_en (1);
	#endif
	reg_usb_ep_ctrl(EP_BO) = BIT(0);
#else
	REG_ADDR8(0x74) = 0x62;
	REG_ADDR16(0x7e) = 0x82bd;
	REG_ADDR8(0x74) = 0x00;

	usb_log_init ();
	usb_dp_pullup_en (1);  //open USB enum
	write_reg8 (0x800120 + (USB_ENDPOINT_BULK_OUT & 7), 1);		//ACK
#endif
}



void proc_host ()
{
	//////////// host interface change  //////////////////////////////////
#if PROVISIONING_ENABLE
    extern my_fifo_t hci_rx_fifo;
	u8 rc_cmd_flag = my_fifo_get(&hci_rx_fifo) ? 1 : 0;
#else
#if (HCI_ACCESS == HCI_USE_UART)
    #if UART_FIFO_ENABLE
    extern my_fifo_t hci_rx_fifo;
    u8 rc_cmd_flag = my_fifo_get(&hci_rx_fifo) ? 1 : 0;
    #else
    extern unsigned char uart_rx_true;
	u8 rc_cmd_flag = uart_rx_true;
    #endif
#else
	#if ((MCU_CORE_TYPE != MCU_CORE_8258) && (MCU_CORE_TYPE != MCU_CORE_8278))
	u8 rc_cmd_flag = (reg_usb_irq & BIT(EP_BO));
	#else
	u8 rc_cmd_flag = (read_reg8 (0x800139) & USB_ENDPOINT_BULK_OUT_FLAG);
	#endif
#endif
#endif

	if(rc_cmd_flag){
		memset4(buff_command, 0, sizeof(buff_command));
		#if (HCI_ACCESS == HCI_USE_UART)
		u32 gateway_proc(void);
		int n = gateway_proc();
		#else
		#if ((MCU_CORE_TYPE != MCU_CORE_8258) && (MCU_CORE_TYPE != MCU_CORE_8278))
		int n = reg_usb_ep_ptr (EP_BO);
		reg_usb_ep_ptr(EP_BO) = 0;
		for (int i=0; i<n; i++)
		{
			buff_command[i] = reg_usb_ep_dat(EP_BO);
		}
		#else
		int n = usb_bulk_out_get_data (buff_command);
		#endif
		#endif
		
	    if(!mode_master){
	        host_write_gateway (n);
	    }
		#if MODE_MASTER_SLAVE
	    else{
		    host_write_master (n);
		}
		#endif

		#if (HCI_ACCESS == HCI_USE_USB && ((MCU_CORE_TYPE != MCU_CORE_8258) && (MCU_CORE_TYPE != MCU_CORE_8278)))
		reg_usb_irq = BIT(EP_BO);
		reg_usb_ep_ctrl(EP_BO) = BIT(0);
		#endif
	}
	#if (HCI_ACCESS == HCI_USE_USB)
	else{
		static u32	tick_bulk_out;
		if (reg_usb_ep_ctrl(EP_BO) & FLD_USB_EP_BUSY)
		{
			tick_bulk_out = clock_time ();
		}
		else if (clock_time_exceed (tick_bulk_out, 1000000))
		{
			reg_usb_ep_ctrl(EP_BO) = BIT(0);
		}
	}
	#endif
}

void light_rx_from_mesh_cb(u8 *p)
{
    // don't modify any data of command
    app_cmd_value_t *p_cmd = (app_cmd_value_t *)p;
    if(p_cmd->dst[1] & 0x80){    // is group
        //if( == p_cmd->op){
            // TODO
        //}
    }
}

void gateway_init(){
    gateway_cmd_sno = ((clock_time() + device_address) & 0xff) << 16;
    fp_gateway_tx_proc = &ble_master_data_callback;
    fp_gateway_rx_proc = &proc_host;
    //register_cb_rx_from_mesh(light_rx_from_mesh_cb);
    
	host_init();
}

void gateway_set_login_flag(){
    if(gateway_en && (!gateway_security)){
        not_need_login = pair_login_ok = 1;
    }else{
        not_need_login = pair_login_ok = 0;
    }
}

int gateway_mode_onoff(u8 on){
    u8 r = irq_disable();
    int ret = 0;
    //mesh_send_online_status_flag = 1;
    if(on){
        if(0 == gateway_en){
            if(0 == slave_link_connected){  // if connected by other phone
                gateway_en = 1;
#if PROVISIONING_ENABLE
                gateway_status = GATEWAY_STATUS_NORMAL;
#endif
                int rf_link_slave_connect (rf_packet_ll_init_t *p, u32 t);
                rf_link_slave_connect((rf_packet_ll_init_t *)&pkt_gateway_init, clock_time());
                slave_link_connected = 1;
                gateway_set_login_flag();
                ble_event_callback (FLG_SYS_LINK_CONNECTED, 0, 0);
            }else{
                ble_conn_terminate = 1;
                ret = -1;  // wait for timeout
            }
        }
    }else{
		#if PROVISIONING_ENABLE
		if(GATEWAY_STATUS_NORMAL == gateway_status)
		#endif
		{
			#if PROVISIONING_ENABLE
			gateway_status = GATEWAY_STATUS_NODE_ROLE;
			hci_rx_fifo.rptr = hci_rx_fifo.wptr = 0;
			hci_tx_fifo.rptr = hci_tx_fifo.wptr = 0;
			#endif
	        if(gateway_en){
	            gateway_set_login_flag();
	        }
	        gateway_en = 0;     // will timeout and into adv mode
	        ble_event_callback (FLG_SYS_LINK_LOST, 0, 0);
        }
    }

    irq_restore(r);
    return ret;
}
#endif

/************* for master dongle with UART ***********/
#if UART_REPORT_ADD_ESCAPE_EN
#define ESCAPE_CHAR      0xE0
#define START_CHAR       0x5A
#define END_CHAR         0xA5
#define ESCAPE_ESCAPE    (ESCAPE_CHAR+1)
#define ESCAPE_START     (START_CHAR+1)
#define ESCAPE_END       (END_CHAR+1)

int uart_add_escape(u8 *data_in, u16 len_in, u8 *data_out, u16 len_out_max)
{
    int len_out = 0;
    data_out[len_out++] = START_CHAR;
    for(int i=0;i<len_in;i++) {
        if(len_out + 2 + 1 > len_out_max){  // 1: for END_CHAR
            break;
        }
        
        if (data_in[i] == START_CHAR) {
            data_out[len_out++] = ESCAPE_CHAR;
            data_out[len_out++] = ESCAPE_START;
        }else if (data_in[i] == END_CHAR) {
            data_out[len_out++] = ESCAPE_CHAR;
            data_out[len_out++] = ESCAPE_END;
        }else if (data_in[i] == ESCAPE_CHAR) {
            data_out[len_out++] = ESCAPE_CHAR;
            data_out[len_out++] = ESCAPE_ESCAPE;
        }else{
            data_out[len_out++] = data_in[i];
        }
    }
    data_out[len_out++] = END_CHAR;

    return len_out;
}
#endif

#define MESH_OTA_THIRD_FW_EN    (0)

u8 mesh_ota_third_fw_flag = 0;  // for the third MCU which connected to 826x through UART/USB.

void mesh_ota_third_complete_cb(int calibrate_flag)    // not irq callback
{
#if MESH_OTA_THIRD_FW_EN
    u8 r = irq_disable();

    if(READ_CALI_RSP_OK == calibrate_flag){
        // TODO
        // down load the frimware to the third MCU,
        
        mesh_ota_led_cb(MESH_OTA_LED_OK);
    }else{
        mesh_ota_led_cb(MESH_OTA_LED_ERROR);
    }
    
    light_sw_reboot();
    irq_restore();
#endif
}

void mesh_ota_set_start_par_user(mesh_ota_pkt_start_command_t *p)
{
    #if MESH_OTA_THIRD_FW_EN
    // redefine parameter by user if needed.
    // max size of start command par is 8
    //memset(p, 0, 8);
    #endif
}

int is_light_mode_match_check_fw(u8 *new_fw_dev_info)
{
    u32 light_mode_self = LIGHT_MODE;
    return (0 == memcmp(new_fw_dev_info, &light_mode_self, 2));
}

int mesh_ota_slave_need_ota(u8 *params)
{
    int ret = 1;
    mesh_ota_pkt_start_command_t *p =  (mesh_ota_pkt_start_command_t *)(params+2);

#if MESH_OTA_THIRD_FW_EN
    if(1){  // comfirm by user
        ret = 1;
        mesh_ota_third_fw_flag = 1;
    }
#else
    if(LIGHT_MODE == p->dev_info.dev_mode){
        void get_fw_version(u8 *ver);
        u8 ver_myself[4];
        get_fw_version(ver_myself);
        #if OTA_ERROR_TEST_EN	// for test
        if(!memcmp(p->version,ver_myself,sizeof(ver_myself))){
            ret = 0;
        }
        #else
        if(p->version[1] < ver_myself[1]){
            ret = 0;
        }else if(p->version[1] == ver_myself[1]){
            if(p->version[3] <= ver_myself[3]){
                ret = 0;
            }
        }
        #endif
    }else{
        ret = 0;
    }
#endif

#if DUAL_MODE_ADAPT_EN
    if(ret){
        dual_mode_disable();
    }
#endif
    
    return ret;
}

void mesh_ota_start_unprotect_flash()   // it will be called by library after receive CMD_OTA_START
{
    if(flash_protect_en){
        flash_unprotect_OTA_start();
    }
}


/* 
for start mesh ota, user should call 
mesh_ota_master_start_firmware() or mesh_ota_master_start_firmware_from_own();

Note: only gate way or some node in BLE connected can start mesh ota now.
*/

int is_valid_fw_len(u32 fw_len)
{
	return (fw_len <= (FW_SIZE_MAX_K * 1024));
}

u32 get_fw_len(u32 fw_adr)
{
	u32 fw_len = 0;
	flash_read_page(fw_adr+0x18, 4, (u8 *)&fw_len);	// use flash read should be better
	return fw_len;
}

void mesh_ota_master_start_firmware(mesh_ota_dev_info_t *p_dev_info, u32 new_fw_adr)
{
	u32 fw_len = get_fw_len(new_fw_adr);
	if(is_valid_fw_len(fw_len)){
		mesh_ota_master_start((u8 *)new_fw_adr, fw_len, p_dev_info);
	}
}

#if GATEWAY_EN
// max fw size: 124K,  because gateway store 3rd firmware from 0x21000(GATEWAY_OTA_OTHER_FW_ADR) -- 0x3FFFF
//STATIC_ASSERT(FW_SIZE_MAX_K <= (128 - 4));

void mesh_ota_master_start_firmware_by_gateway(u16 dev_mode)
{
	u32 new_fw_adr = GATEWAY_OTA_OTHER_FW_ADR;
	u32 fw_len = get_fw_len(new_fw_adr);
	if(is_valid_fw_len(fw_len)){
		mesh_ota_dev_info_t dev_info = {0};
		dev_info.dev_mode = dev_mode;
		mesh_ota_master_start((u8 *)new_fw_adr, fw_len, &dev_info);
	}
}
#endif

void mesh_ota_master_start_firmware_from_own()
{
#if (MCU_CORE_TYPE == MCU_CORE_8267 || MCU_CORE_TYPE == MCU_CORE_8269 || \
	MCU_CORE_TYPE == MCU_CORE_8258 || MCU_CORE_TYPE == MCU_CORE_8278)
    u32 adr_fw = ota_program_offset ? 0 : flash_adr_light_new_fw;  // 8267/8269 should use "ota_program_offset"
#else
    u32 adr_fw = 0;
#endif
	u32 fw_len = get_fw_len(adr_fw);
	if(is_valid_fw_len(fw_len)){
		mesh_ota_dev_info_t dev_info = {0};
		dev_info.dev_mode = LIGHT_MODE;
		mesh_ota_master_start((u8 *)adr_fw, fw_len, &dev_info);
	}
}

////////////////// check stack /////////////////////////////////////
#if(STACK_CHECK_ENABLE)
extern u32 _start_bss_, _end_bss_;
#define STACK_CHECK_FLAG        (0x1324adbc)    // use a random data

void stack_check_init(){
    _start_bss_ = _end_bss_ = STACK_CHECK_FLAG;
}

void stack_check(){
	static u32 stack_check_tick;
	if (!clock_time_exceed (stack_check_tick, 1000*1000))
	{
		return;
	}
	stack_check_tick = clock_time();
	
    if((_start_bss_ != STACK_CHECK_FLAG)    // irq_stack
    || (_end_bss_ != STACK_CHECK_FLAG)){    // system_stack
        irq_disable();
        while(1){
            #if(MODULE_WATCHDOG_ENABLE)
    		wd_clear();
            #endif
            static u8 stack_overflow;
            stack_overflow++;
        }
    }
}
#endif

////////////////// ALARM and SCENE /////////////////////////////////////
#include "rtc.h"
#include "scene.h"

int is_bridge_task_busy(){
    return ((ALARM_EN && (is_alarm_poll_notify_busy()))
          || (SCENE_EN && (is_scene_poll_notify_busy())));
}

int is_tx_cmd_busy(){   // it must be used, if want enough bridges(default is BRIDGE_MAX_CNT 8).
    return (0 != mesh_user_cmd_idx);
}

////////////////// OTA  /////////////////////////////////////
extern void ota_boot_check();
extern int rf_ota_save_data(u8 * data);
extern void rf_ota_set_flag();

u32  rf_slave_ota_finished_time = 0;
extern u8  rf_slave_ota_terminate_flag;
extern u32 cur_ota_flash_addr;
extern u32 tick_per_us;
extern u16 rf_slave_ota_timeout_def_s;
extern u16  rf_slave_ota_timeout_s;

const u8 pkt_terminate[8] = {0x04, 0x00, 0x00, 0x00, 0x03, 0x02, 0x02, 0x13};
u8 slave_ota_data_cache_idx=0;

// set to 12 for debug simple in tdebug tool
extern u32 buff_response[][12];	// array size should be greater than 16

void rf_ota_set_flag()
{
    u32 fw_flag_telink = START_UP_FLAG;
    u32 flag_new = 0;
	flash_read_page(flash_adr_ota_offset + 8, sizeof(flag_new), (u8 *)&flag_new);
	flag_new &= 0xffffff4b;
    if(flag_new != fw_flag_telink){
        return ; // invalid flag
    }
    
#if(__TL_LIB_8267__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8267) \
    || __TL_LIB_8269__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8269) \
    || __TL_LIB_8258__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8258)	\
    || __TL_LIB_8278__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8278))
	u32 flag = 0x4b;
	flash_write_page(flash_adr_ota_offset + 8, 1, (u8 *)&flag);		//Set FW flag

    if(flash_protect_en){
	    flash_protect_8267_OTA_clr_flag();
	}
	#if (!PINGPONG_OTA_DISABLE)
	flag = 0;
	flash_write_page((flash_adr_ota_offset ? 0 : flash_adr_light_new_fw) + 8, 4, (u8 *)&flag);	//Invalid flag
	#endif
	
    if(0x40000 == ota_program_offset){
    	#define ADR_20008		(0x20008)
    	u32 start_flag = 0;
    	flash_read_page(ADR_20008, 4, (u8 *)&start_flag);
    	if(0x544c4e4b == start_flag){
    		u32 zero = 0;
    		flash_write_page(ADR_20008, 1, (u8 *)&zero);	//make sure not valid flag in 0x20008
    	}
	}
#else
	u32 flag0 = 0;
	flash_read_page(flash_adr_ota_offset + 8, 1, (u8 *)&flag0);
	if(0x4b != flag0){
	    flag0 = 0x4b;
	    flash_write_page(flash_adr_ota_offset + 8, 1, (u8 *)&flag0);		//Set FW flag, make sure valid. because the firmware may be from 8267 by mesh ota
	}
	
    if(flash_protect_en){
        flash_protect_8266_OTA_clr_flag();
    }
	flash_erase_sector (FLASH_ADR_OTA_READY_FLAG);
	u32 flag = 0xa5;
	flash_write_page (FLASH_ADR_OTA_READY_FLAG, 4, (u8 *)&flag);

    if(flash_protect_en){
        flash_protect_OTA_copy(); // prepare for OTA_COPY
    }
#endif
}

void rf_slave_ota_finished_flag_set(u8 reset_flag)
{
	rf_slave_ota_finished_flag = reset_flag;
	rf_slave_ota_finished_time = clock_time();
}

int	rf_link_slave_data_ota(void *ph)
{
    if(rf_slave_ota_finished_flag){
        return 1;
    }

    if(!rf_slave_ota_busy){
        if(!pair_login_ok || is_master_sending_ota_st() || is_mesh_ota_slave_running()){
            return 1;
        }
        
        rf_slave_ota_busy = 1;
        extern u8 slave_read_status_busy;
        extern void rf_link_slave_read_status_stop ();
        if (slave_read_status_busy)
        {
            rf_link_slave_read_status_stop ();
        }
    }
	memcpy(buff_response[(slave_ota_data_cache_idx++)%16], ph,sizeof(rf_packet_att_data_t));
	return 1;
}

u8 get_ota_check_type(u8 *par)
{
	if(par[0] == 0x5D){
		return par[1];
	}
	return 0;
}

#if (!__PROJECT_BEACON_DETECTOR__)
int	rf_link_slave_data_ota_save()
{
	extern u16 ota_pkt_cnt;
	static u16 ota_rcv_last_idx = -1;
	u8 reset_flag=0;
	static u32 fw_check_val = 0;
	static u8 need_check_type = 0;//=1:crc val sum
	static u16 ota_pkt_total = 0;
    
	foreach(i,slave_ota_data_cache_idx){
		rf_packet_att_data_t *p = (rf_packet_att_data_t *)buff_response[i];
		int nDataLen = p->l2cap - 7;

		if(crc16(p->dat, 2+nDataLen) == (p->dat[nDataLen+2] | p->dat[nDataLen+3]<<8)){
		
#if(WORK_SLEEP_EN)
            last_rcv_hb_time = clock_time() | 1;
            hb_timeout_pre = 0;
#endif
			rf_slave_ota_timeout_s = rf_slave_ota_timeout_def_s;	// refresh timeout
			
			u16 cur_idx = (*(p->dat) | ((*(p->dat+1)) << 8));
			if(nDataLen == 0){
				if((cur_idx == ota_rcv_last_idx+1) && (cur_idx == ota_pkt_total)){
					// ota ok, save, reboot
					reset_flag = OTA_STATE_OK;
				}else{
					// ota err
					cur_ota_flash_addr = ota_pkt_cnt = ota_rcv_last_idx = 0;
					reset_flag = OTA_STATE_ERROR;
					SET_OTA_GATT_ERROR_NUM(1);
				}
			}else{
				if(cur_idx == 0){
					// start ota
					if(cur_ota_flash_addr){
					    // 0x10000 should be 0x00
						cur_ota_flash_addr = ota_pkt_cnt = ota_rcv_last_idx = 0;
	                    reset_flag = OTA_STATE_ERROR;
						SET_OTA_GATT_ERROR_NUM(2);
					}else{
	                    #if(__TL_LIB_8266__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8266))
	                    ota_boot_check();
	                    #endif
	                    
	                    if(flash_protect_en){
					        flash_unprotect_OTA_start();
					    }
					    
						#if (ZBIT_FLASH_WRITE_TIME_LONG_WORKAROUND_EN)
						check_and_set_1p95v_to_zbit_flash();
						#endif
					    
					    need_check_type = get_ota_check_type(&p->dat[8]);
					    if(need_check_type == 1){
					    	fw_check_val = (p->dat[nDataLen+2] | p->dat[nDataLen+3]<<8);
					    }
	    				reset_flag = rf_ota_save_data(p->dat+2);
                        #if DUAL_MODE_ADAPT_EN
                        dual_mode_disable();
                        #endif
					}
				}else if(cur_idx == ota_rcv_last_idx + 1){
					// ota fw check
					if(cur_idx == 1){
						ota_pkt_total = (((p->dat[10]) |( (p->dat[11] << 8) & 0xFF00) | ((p->dat[12] << 16) & 0xFF0000) | ((p->dat[13] << 24) & 0xFF000000)) + 15)/16;
						if((ota_pkt_total < 3
						||((APP_OTA_HCI_TYPE_GATT_ONLY == app_ota_hci_type)&&(0 == mesh_ota_master_get_no_check_dev_mode_flag())
						    &&(!is_light_mode_match_check_fw(p->dat+2+(LIGHT_MODE_POSITION % 16)))))){
							// invalid fw
							cur_ota_flash_addr = ota_pkt_cnt = ota_rcv_last_idx = 0;
							reset_flag = OTA_STATE_ERROR;
							SET_OTA_GATT_ERROR_NUM(3);
						}else if(need_check_type == 1){
							fw_check_val += (p->dat[nDataLen+2] | p->dat[nDataLen+3]<<8);
						}
					}else if(cur_idx < ota_pkt_total - 1 && need_check_type == 1){
						fw_check_val += (p->dat[nDataLen+2] | p->dat[nDataLen+3]<<8);
					}else if(cur_idx == ota_pkt_total - 1 && need_check_type == 1){
						if(fw_check_val != ((p->dat[2]) |( (p->dat[3] << 8) & 0xFF00) | ((p->dat[4] << 16) & 0xFF0000) | ((p->dat[5] << 24) & 0xFF000000)) ){
							cur_ota_flash_addr = ota_pkt_cnt = ota_rcv_last_idx = 0;
							reset_flag = OTA_STATE_ERROR;
							SET_OTA_GATT_ERROR_NUM(4);
						}
					}
					
					if(cur_ota_flash_addr + 16 > (FW_SIZE_MAX_K * 1024)){ // !is_valid_fw_len()
					    reset_flag = OTA_STATE_ERROR;
                        SET_OTA_GATT_ERROR_NUM(0x41);
				    }else{
					    reset_flag = rf_ota_save_data(p->dat+2);
					}
				}else{
					// error, ota failed
					#if OTA_ERROR_TEST_EN
					static u16 ota_err_idx_last;ota_err_idx_last = ota_rcv_last_idx;
					#endif
					cur_ota_flash_addr = ota_pkt_cnt = ota_rcv_last_idx = 0;
					reset_flag = OTA_STATE_ERROR;
					SET_OTA_GATT_ERROR_NUM(5);
				}

				ota_rcv_last_idx = cur_idx;
			}
		}else{
			// error, ota failed
			cur_ota_flash_addr = ota_pkt_cnt = ota_rcv_last_idx = 0;
		    reset_flag = OTA_STATE_ERROR;
			SET_OTA_GATT_ERROR_NUM(6);
		}
		if(reset_flag){
		    if(0 == rf_slave_ota_finished_flag){
		    	if((APP_OTA_HCI_TYPE_MESH == app_ota_hci_type)
		    	&& (OTA_STATE_OK == reset_flag)){
		    		mesh_ota_master_start_firmware_from_backup();
		    		rf_slave_ota_timeout_s = 0;	// stop gatt ota timeout check
		    		rf_slave_ota_busy = 0;		// must
			    }else{
			    	rf_slave_ota_finished_flag_set(reset_flag);
			    }
		    }

		    #if OTA_ERROR_TEST_EN
		    if(reset_flag != OTA_STATE_OK){
		    	rf_led_ota_error();
		    }
		    #endif
		    
			break;
		}
	}
	slave_ota_data_cache_idx = 0;
	return 1;
}
#endif

void rf_link_slave_ota_finish_led_and_reboot(int st)
{
    if(OTA_STATE_ERROR == st){
        erase_ota_data(flash_adr_ota_offset);
        rf_led_ota_error();
    }else if(OTA_STATE_OK == st){
        //rf_ota_save_data(0);
        rf_ota_set_flag (); 
        rf_led_ota_ok();
    }else if(OTA_STATE_MASTER_OTA_REBOOT_ONLY == st){
    	// just reboot
    }
    irq_disable ();
    light_sw_reboot();
}

void rf_link_slave_ota_finish_handle()		// poll when ota busy in bridge
{
	rf_link_slave_data_ota_save();
	
    if(rf_slave_ota_finished_flag){
        static u8 terminate_cnt;
        u8 reboot_flag = 0;
        if((0 == terminate_cnt) && (rf_slave_ota_terminate_flag)){
            if(is_add_packet_buf_ready()){
                terminate_cnt = 6;
                rf_link_add_tx_packet ((u32)(&pkt_terminate));
            }
        }
        
        if(terminate_cnt){
            terminate_cnt--;
            if(!terminate_cnt){
                reboot_flag = 1;
            }
        }
        
        if((!rf_slave_ota_terminate_flag)
         &&((u32)(clock_time() - rf_slave_ota_finished_time) > 2000*1000 * tick_per_us)){
            rf_slave_ota_terminate_flag = 1;    // for ios: no last read command
        }
        
        if(((u32)(clock_time() - rf_slave_ota_finished_time) > 4000*1000 * tick_per_us)){
            reboot_flag = 1;
        }
        
        if(reboot_flag){
            rf_link_slave_ota_finish_led_and_reboot(rf_slave_ota_finished_flag);
            // have been reboot
        }
    }
}

u32 get_ota_erase_sectors()
{
    return ERASE_SECTORS_FOR_OTA;
}

int is_ota_area_valid(u32 adr){
#if (DUAL_MODE_ADAPT_EN)
	if(DUAL_MODE_NOT_SUPPORT != dual_mode_state){
	    return 1;
	}
#endif

	u8 buf[4] = {0};
	foreach(i, ERASE_SECTORS_FOR_OTA){
    	flash_read_page(adr + i*0x1000, 4, buf);
    	u32 tmp = buf[0] | (buf[1]<<8) |(buf[2]<<16) | (buf[3]<<24);
    	if(tmp != ONES_32){
            return 0;
    	}
    }
	return 1;
}

void erase_ota_data(u32 adr){
#if (! __PROJECT_BLE_MASTER__)  // 	master dongle can't erase 0x40000,because new firmware size would be greater than 128K, and will use 0x40000
    flash_protect_OTA_data_erase();
    #if 1
    foreach(i, ERASE_SECTORS_FOR_OTA){
        flash_erase_sector(adr+(ERASE_SECTORS_FOR_OTA -1 - i)*0x1000);
    }
    #else
    //  Note: differrent size or type may use differrent command of block erase.
    STATIC_ASSERT((ERASE_SECTORS_FOR_OTA % 16) == 0);
    u32 block_cnt = ERASE_SECTORS_FOR_OTA/16;
    foreach(i, block_cnt){
        flash_erase_block(adr+(block_cnt -1 - i)*0x10000);
    }
    #endif
#endif
}

void erase_ota_data_handle(){
	// for app ota  
    #if (MCU_CORE_TYPE == MCU_CORE_8267 || MCU_CORE_TYPE == MCU_CORE_8269 || MCU_CORE_TYPE == MCU_CORE_8258 || MCU_CORE_TYPE == MCU_CORE_8278)
    u32 adr_ota_data = ota_program_offset;  // 8267 should use "ota_program_offset"
    #else
    u32 adr_ota_data = flash_adr_light_new_fw;
    #endif
	if(0 == is_ota_area_valid(adr_ota_data)){
		erase_ota_data(adr_ota_data);
	}

    #if ((MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269)||(MCU_CORE_TYPE == MCU_CORE_8258) ||(MCU_CORE_TYPE == MCU_CORE_8278))
    flash_protect_8267_normal();    // must after erase_sector for OTA
    #else
    flash_protect_8266_normal();
    #endif
}
void light_node_status_change_cb(u8 *p, u8 new_node){
    mesh_node_st_val_t *p_data = (mesh_node_st_val_t*)p;
    extern u8  sync_time_enable;
    if(sync_time_enable){
        p_data->par[0] &= ~FLD_SYNCED;   //Note: bit7 of par[0] have been use internal
    }
#if(WORK_SLEEP_EN)
    if(p_data->par[1] == 0){// need_sleep
        if(need_sleep){
            hb_timeout_pre = need_sleep_pre = need_sleep = 0;
        }
    }
#endif    
    static u8 dev_addr = 0;
    if(new_node){
        #if PROVISIONING_ENABLE
        if(gateway_status == GATEWAY_STATUS_SCAN_UNPROV_DEV)
        {
            u8 buf[4];
            buf[0] = 0x03;
            buf[1] = GATEWAY_EVENT_NEW_DEVICE_FOUND;
            buf[2] = p_data->dev_adr;
            my_fifo_push_hci_tx(buf, 3);
        }
        #endif
        static u8 dev_new_node = 0;
        dev_new_node++;
        dev_addr = p_data->dev_adr;
    }else{
        static u8 dev_old_node = 0;
        dev_old_node++;
        dev_addr = p_data->dev_adr;
        if(0 == p_data->sn){    // off line
            // rssi_online_status_pkt_cb(CONTAINER_OF(p_data,mesh_node_st_t,val), 0, 0);
        }
    }
}

void rssi_online_status_pkt_cb(mesh_node_st_t *p_node_st, u8 rssi, int online_again)    // call in IRQ state
{
    #if MESH_RSSI_RECORD_EN
    // just only can handle rssi, don't modify any other par inside p_node_st
    if((p_node_st->rssi != rssi) || online_again){     // online_again: new device or offline before
        #if 0 // test
        static u32 A_buf_cnt;
        static u8 A_buf[100][2];
        if(A_buf_cnt < ARRAY_SIZE(A_buf)){
            A_buf[A_buf_cnt][0] = p_node_st->val.dev_adr;
            A_buf[A_buf_cnt][1] = rssi;
            A_buf_cnt++;
        }
        #endif
        p_node_st->rssi = rssi; // rssi -= 110;
        // TODO: only report rssi to master here
    }
    #else
    // TODO: only report rssi to master here
    #endif
}

cb_rx_from_mesh_t p_cb_rx_from_mesh = 0;    // for user call back handle
void register_cb_rx_from_mesh (cb_rx_from_mesh_t p)
{
	p_cb_rx_from_mesh = p;
}

#if SUB_ADDR_EN
STATIC_ASSERT(TEST_LED_CNT <= ARRAY_SIZE(led_val));

void light_multy_onoff_sub_addr(u8 sub_addr, u8 on, bool update_online_st)
{
    if(sub_addr && sub_addr <= TEST_LED_CNT){
        int index = sub_addr - 1;
        led_val[index] = on ? MAX_LUM_BRIGHTNESS_VALUE : 0;
        if(0 == index){
            light_adjust_R(led_val[index], led_lum);
        }else if(1 == index){
            light_adjust_G(led_val[index], led_lum);
        }else if(2 == index){
            light_adjust_B(led_val[index], led_lum);
        }

        if(update_online_st){
            device_status_update();
        }
    }
}

void light_multy_onoff(u8 *dst_addr, u8 on)
{
    device_addr_sub_t *p_addr = (device_addr_sub_t *)dst_addr;
    if(p_addr->group_flag || (0 == p_addr->sub_addr)){ // group
        if((0xff == dst_addr[0] && 0xff == dst_addr[1])||(0 == p_addr->sub_addr)){  // all
            foreach(i,TEST_LED_CNT){
                light_multy_onoff_sub_addr(i+1, on, 0);
            }
            device_status_update();
        }else{ // group
            // TODO:
        }
    }else{
        if(p_addr->sub_addr && p_addr->sub_addr <= TEST_LED_CNT){
            light_multy_onoff_sub_addr(p_addr->sub_addr, on, 1);
        }
    }
}

u8 get_sub_addr_onoff()
{
    u8 val = 0;
    foreach(i,TEST_LED_CNT){
        if(led_val[i]){
            val |= BIT(i);
        }
    }
    return val;
}
#endif

/*callback function: set sub address for mesh_push_user_command_sub_addr()*/
void cb_set_sub_addr_tx_cmd(u8 *src, u16 sub_adr)
{
#if SUB_ADDR_EN
    device_addr_sub_t *p_src = (device_addr_sub_t *)src;
    p_src->sub_addr = sub_adr;
#endif
}

void user_init_common()
{
#if SUB_ADDR_EN
    set_device_addr_mask();
#endif
}

void set_firmware_type(u32 sdk_type)
{
    u32 mesh_type = 0;
    flash_read_page(FLASH_ADR_MESH_TYPE_FLAG, sizeof(mesh_type), (u8 *)&mesh_type);
	if(mesh_type != 0xffffffff){
		flash_erase_sector(FLASH_ADR_MESH_TYPE_FLAG);
	}
	mesh_type = sdk_type;
	flash_write_page (FLASH_ADR_MESH_TYPE_FLAG, 4, (u8 *)&mesh_type);
}

void set_firmware_type_TLK_mesh()
{
    set_firmware_type(TYPE_TLK_MESH);
}

void set_firmware_type_init()
{
    flash_erase_sector(FLASH_ADR_MESH_TYPE_FLAG);
}

/*proc_sig_mesh_to_telink_mesh: called by rf_link_slave_init() in library*/
u8 proc_sig_mesh_to_telink_mesh(void)
{
    user_init_common();

#if (MCU_CORE_TYPE == MCU_CORE_8267 || MCU_CORE_TYPE == MCU_CORE_8269 || MCU_CORE_TYPE == MCU_CORE_8258 || MCU_CORE_TYPE == MCU_CORE_8278)
	u32 mesh_type = *(u32 *) FLASH_ADR_MESH_TYPE_FLAG;

	#if DUAL_MODE_ADAPT_EN
    //LOG_MSG_INFO(TL_LOG_NODE_SDK,0, 0,"sdk type 0x73000:0x%x",mesh_type);
	if(TYPE_DUAL_MODE_STANDBY == mesh_type){
		return 0;
	}if(0xffffffff == mesh_type){
        set_firmware_type(TYPE_DUAL_MODE_STANDBY);
        //LOG_MSG_INFO(TL_LOG_NODE_SDK,0, 0,"sdk type: Factory status", 0);
		return 0;
	}else if(TYPE_DUAL_MODE_RECOVER == mesh_type){
        factory_reset();
        set_firmware_type(TYPE_DUAL_MODE_STANDBY);
        //LOG_MSG_INFO(TL_LOG_NODE_SDK,0, 0,"sdk type: Recover from SIG MESH", 0);
        return 0;
	}
	#endif

	if(TYPE_TLK_MESH == mesh_type){
		return 1;
	}

	u8 ret = 0;
	if(mesh_type == TYPE_SIG_MESH || (mesh_type == TYPE_TLK_BLE_SDK) || (mesh_type == TYPE_TLK_ZIGBEE)){
	    #if ((0 == FLASH_1M_ENABLE) && (CFG_SECTOR_ADR_MAC_CODE == CFG_ADR_MAC_512K_FLASH))
	    if(CFG_ADR_CALIBRATION_512K_FLASH == FLASH_ADR_DC){ // DC and TP is in the same address when 1M flash.
    		u8 flash_data = 0;
    		flash_read_page(FLASH_ADR_DC, 1, &flash_data);
    		if(flash_data == 0xff){
    			flash_read_page(0x77000, 1, &flash_data);
    			flash_write_page(FLASH_ADR_DC, 1, &flash_data);
    		}
    		#if ((MCU_CORE_TYPE == MCU_CORE_8266)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
    		flash_read_page(FLASH_ADR_TP_LOW, 1, &flash_data);
    		if(flash_data == 0xff){
    			flash_read_page(0x77040, 1, &flash_data);
    			flash_write_page(FLASH_ADR_TP_LOW, 1, &flash_data);
    		}
    		
    		flash_read_page(FLASH_ADR_TP_HIGH, 1, &flash_data);
    		if(flash_data == 0xff){
    			flash_read_page(0x77041, 1, &flash_data);
    			flash_write_page(FLASH_ADR_TP_HIGH, 1, &flash_data);
    		}
    		#endif
    		// no RC32K_CAP_INFO
		}
	    #endif
	
		factory_reset();
		ret = 1;
	}

    set_firmware_type_TLK_mesh();
    
	return ret;
#endif
	return 0;
}


#if ADV_UUID
u8 adv_uuid_flag = 1;
#else
u8 adv_uuid_flag = 0;
#endif
u8 adv_uuid[4] = {0x03, 0x02, 0xAB, 0xCD};

// recover status before software reboot
void light_sw_reboot_callback(void){
#if ((__PROJECT_LIGHT_8266__)           \
    ||(__PROJECT_LIGHT_8267__)          \
    ||(__PROJECT_LIGHT_8269__)          \
    ||(__PROJECT_LIGHT_8258__)          \
    ||(__PROJECT_LIGHT_8278__)          \
    ||(__PROJECT_LIGHT_NO_MESH__))
    if(rf_slave_ota_busy || is_mesh_ota_slave_running()){	// rf_slave_ota_busy means mesh ota master busy also.
        analog_write (rega_light_off, light_off ? FLD_LIGHT_OFF : 0);
    }
#endif

#if MESH_OTA_MASTER_FLAG_EN
    if(is_master_ota_st_record_100()){
		u8 val = analog_read(rega_light_off);
    	val |= FLD_MESH_OTA_MASTER_100;
        analog_write (rega_light_off, val);
    }
#endif
}

#if PM_DEEPSLEEP_RETENTION_ENABLE
void light_sw_reboot_with_retention()
{
    /*u8 r = */irq_disable();
    // light_sw_reboot_callback();
    cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM, PM_WAKEUP_TIMER, clock_time() + 3*CLOCK_SYS_CLOCK_1MS); // retention reboot, tick refer to (EMPTYRUN_TIME_US + 200)
    while (1);  // wait for reboot.
}
#endif

void mesh_ota_master_100_flag_check()
{
	u8 val = analog_read(rega_light_off);
	if(val & FLD_MESH_OTA_MASTER_100){
		mesh_ota_master_100_flag = 1;
		analog_write (rega_light_off, val & (~ FLD_MESH_OTA_MASTER_100));
	}
}

/////////////////// mesh_node_filter  //////////////////////
#if 1
cb_mesh_node_filter_t	cb_mesh_node_filter = 0;
#else   // for test relay function  
int is_neighbor_light(u8 device_adr, u8 *p)
{
    rf_packet_att_cmd_t *pp = (rf_packet_att_cmd_t *)p;
    pp = pp;
	foreach(i,6){
		u8 dev_filter = *(u8 *)(0x60000+i);
		if(dev_filter == device_adr){
			// only receive some neighbor devices
			return 1;
		}
	}
    return 0;
}

/*////////////////// mesh_node_filter  //////////////////////
int (*cb_mesh_node_filter_t) (u8 device_adr);
fuction      : use to test relay function  

device_adr : means this packet is sent from this device_adr
p              : point to this packet

return 0 : means skip this packet
return 1 : means accept this packet
*/

cb_mesh_node_filter_t	cb_mesh_node_filter = is_neighbor_light;
#endif

/////////////////// passive device  //////////////////////
u8 passive_en = PASSIVE_EN;
int get_command_type(u8 *p_att_value)
{
#if PASSIVE_EN
    rf_packet_att_value_t *p = (rf_packet_att_value_t*)(p_att_value);
    u8 op = p->val[0] & 0x3F;
    u8 *par = p->val+3;
    if(LGT_CMD_LIGHT_ONOFF == op){
        if(ON_OFF_FROM_PASSIVE_DEV == par[3]){
            return CMD_TYPE_PASSIVE_DEV;
        }else if(ON_OFF_FROM_PASSIVE_DEV_ALT == par[3]){
            return CMD_TYPE_PASSIVE_DEV_ALT;
        }
    }
#endif

    return CMD_TYPE_NORMAL;
}

// Note: par[8], par[9] of passive command have been used internal for sno2.
void set_command_type2alt(u8 *p_att_value)
{
#if PASSIVE_EN
    rf_packet_att_value_t *p = (rf_packet_att_value_t*)(p_att_value);
    u8 op = p->val[0] & 0x3F;
    u8 *par = p->val+3;
    if(LGT_CMD_LIGHT_ONOFF == op){
        if(ON_OFF_FROM_PASSIVE_DEV == par[3]){
            par[3] = ON_OFF_FROM_PASSIVE_DEV_ALT;
        }
    }
#endif
}

int mesh_cmd_notify(u8 op, u8 *p, u8 len, u16 dev_adr)
{
    int err = -1;
    if(slave_link_connected && pair_login_ok){
        if(len > 10){   //max length of par is 10
            return -1;
        }
        
		rf_packet_att_cmd_t pkt_notify = {
				0x1d,						// dma_len
				0x02,						// type
				0x1b,						// rf_len
				0x17,						// u16
				0x04,						// chanid
				0x1b,						// notify
				0x12, 0x00, 				// status handler
				{0, 0, 0,					// seqno
				 0, 0,						// src
				 0, 0,						// dst
				 0, VENDOR_ID & 0xff, VENDOR_ID >> 8,
				}
		};

        pkt_notify.value[3] = dev_adr & 0xFF;
        pkt_notify.value[4] = dev_adr >> 8;
		pkt_notify.value[7] = op | 0xc0;
        
        u8 *p_par = &(pkt_notify.value[10]);
        memcpy(p_par, p, len);
        u8 r = irq_disable();
        if (is_add_packet_buf_ready()){
            if(0 != rf_link_add_tx_packet ((u32)(&pkt_notify))){
                err = 0;
            }
        }
        irq_restore(r);
    }

    return err;
}

void set_mesh_provision_info(bool save_flag, u8 *name, u8 *pw, u8 *ltk)
{
    extern u8 max_mesh_name_len;
    if(name){
        memcpy (pair_nn, name, max_mesh_name_len);
    }
    
    if(pw){
        memcpy (pair_pass, pw, 16);
    }
    
    if(ltk){
        memcpy (pair_ltk, ltk, 16);
    }
    
    if(save_flag){
        pair_save_key();
    }else{
        pair_update_key ();
    }
}

#if(MESH_PAIR_ENABLE)
u8 mesh_pair_enable = 0;
u32 mesh_pair_cmd_interval = 0;
u32 mesh_pair_start_time = 0;
u32 mesh_pair_timeout = 0;
u8 new_mesh_name[16] = {0};
u8 new_mesh_pwd[16] = {0};
u8 new_mesh_ltk[16] = {0};
u32 effect_new_mesh_delay_time = 0;
u8 effect_new_mesh = 0;
u32 mesh_pair_start_notify_time = 0;
u8 mesh_pair_retry_max = 3;
u8 mesh_pair_retry_cnt = 0;
u8 mesh_pair_notify_rsp_mask[32] = {0};
u8 mesh_pair_checksum[8] = {0};
u32 default_mesh_time = 0;
u32 default_mesh_time_ref = 0;
u32 default_mesh_effect_delay_ref = 0;  /* When receive change to default mesh command, shall delay at least 500ms */
extern u8 pair_ltk_mesh[16];
u8 get_mesh_pair_checksum(u8 idx){
    u8 i = idx % 8;
    return (new_mesh_name[i] ^ new_mesh_name[i+8]) ^ (new_mesh_pwd[i] ^ new_mesh_pwd[i+8]) ^ (new_mesh_ltk[i] ^ new_mesh_ltk[i+8]);
}

u8 mesh_pair_notify_refresh(rf_packet_att_cmd_t *p){
    if(!memcmp(mesh_pair_checksum, p->value + 12, 8)){
        // mesh pair : success one device, clear the mask flag
        mesh_pair_notify_rsp_mask[(p->value[10]) / 8] &= ~(BIT(p->value[10] % 8));
    }
    return 1;// if return 2, then the notify rsp will not report to master.
}

int mesh_pair_complete_notify()
{
	u8 par[1] = {CMD_NOTIFY_MESH_PAIR_END};
	return mesh_cmd_notify(LGT_CMD_MESH_CMD_NOTIFY, par, sizeof(par), device_address);
}

void save_effect_new_mesh(void)
{
#if GATEWAY_EN
#if PROVISIONING_ENABLE
    if(gateway_status == GATEWAY_STATUS_CFG_UNPRO_DEV ||\
       gateway_status == GATEWAY_STATUS_ADD_CONFIGURED_DEVICE ||\
       gateway_status == GATEWAY_STATUS_TEMP_DEFALT_MESH)
    {
        /* Set by gateway itself */
        pair_load_key();
        default_mesh_time_ref = 0;
        gateway_status = GATEWAY_STATUS_NORMAL;
        /* Add provision complete event */
        u8 buf[2];
        buf[0] = 0x02;
        buf[1] = GATEWAY_EVENT_PROVISION_COMPLETE;
        my_fifo_push_hci_tx(buf, 2);
        /* Refresh */
        extern void mesh_node_init ();
        mesh_node_init();
        device_status_update();
        goto L_RETURN;
    }
    else if(gateway_status == GATEWAY_STATUS_NORMAL)
    {
        /* Set by others */
        u8 buf[2];
        buf[0] = 0x02;
        buf[1] = GATEWAY_EVENT_PROVISION_BY_OTHERS;
        my_fifo_push_hci_tx(buf, 2);
    }
    else if(gateway_status == GATEWAY_STATUS_CFG_CUR_NETWORK)
    {
        gateway_status = GATEWAY_STATUS_NORMAL;
    }
#endif
#else
    if(default_mesh_time_ref || get_mac_en)
    {
		mesh_pair_complete_notify();
		sleep_us(1000);
        /* Switch to normal mesh */
        pair_load_key();
        default_mesh_time_ref = 0;
        
        extern void mesh_node_init ();
        mesh_node_init();
        device_status_update();
        goto L_RETURN;
    }
#endif
    if(effect_new_mesh == 0){
        memcpy4(pair_nn, new_mesh_name, 16);
        memcpy4(pair_pass, new_mesh_pwd, 16);
        memcpy4(pair_ltk, new_mesh_ltk, 16);
    }
    else
    {
        memcpy4(pair_ltk, pair_ltk_mesh, 16);
    }

	mesh_pair_complete_notify();
    
    #if 1	// make sure not receive legacy mesh data from now on
    u8 r = irq_disable();
    pair_save_key();
    rf_set_ble_access_code ((u8 *)&pair_ac);// use new access code at once.
    rf_link_light_event_callback(LGT_CMD_SET_MESH_INFO);	// clear online status :mesh_node_init()
	sleep_us (1000);
    reg_rf_irq_status = FLD_RF_IRQ_RX;		// clear current rx in buffer
    irq_restore(r);
    #endif
L_RETURN:
    memset4(new_mesh_name, 0, 16);
    memset4(new_mesh_pwd, 0, 16);
    memset4(new_mesh_ltk, 0, 16);
    mesh_pair_start_notify_time = mesh_pair_retry_cnt = mesh_pair_start_time = 0;
    memset4(mesh_pair_notify_rsp_mask, 0, 32);
    pair_setting_flag = PAIR_SETTED;
}


void switch_to_default_mesh(u8 delay_s)
{
    default_mesh_time_ref = clock_time() | 1;
    default_mesh_time = delay_s * 1000;
    
    extern u32 access_code(u8 *p_name, u8 *p_pw);
    extern u8	pair_config_mesh_name[17];
	extern u8	pair_config_mesh_pwd[17];
	extern u8	pair_config_mesh_ltk[17];
    #if GATEWAY_EN
	    memcpy(pair_ltk_mesh, pair_ltk, 16);
    #endif
    /* Only change AC and LTK */
    pair_ac = access_code(pair_config_mesh_name, pair_config_mesh_pwd);
    memcpy(pair_ltk, pair_config_mesh_ltk, 16);
}

void mesh_pair_cb(u8 *params)
{
    dual_mode_set_adv_provisoning_flag();
    
    #if !GATEWAY_EN
    if(default_mesh_time_ref){
        // return;
        default_mesh_time_ref = clock_time() | 1;
    }
    #endif
    if(params[0] == MESH_PAIR_NAME1){
        mesh_pair_start_time = clock_time() | 1;
        memcpy(new_mesh_name, params + 1, 8);
    }else if(params[0] == MESH_PAIR_NAME2){
        memcpy(new_mesh_name + 8, params + 1, 8);
    }else if(params[0] == MESH_PAIR_PWD1){
        memcpy(new_mesh_pwd, params + 1, 8);
    }else if(params[0] == MESH_PAIR_PWD2){
        memcpy(new_mesh_pwd + 8, params + 1, 8);
    }else if(params[0] == MESH_PAIR_LTK1){
        memcpy(new_mesh_ltk, params + 1, 8);
    }else if(params[0] == MESH_PAIR_LTK2){
        memcpy(new_mesh_ltk + 8, params + 1, 8);
    }else if(params[0] == MESH_PAIR_EFFECT_DELAY){
        effect_new_mesh_delay_time = clock_time() | 1;
        #if !GATEWAY_EN
        if(default_mesh_time_ref){
            /* Keep default_mesh_time_ref non-zero */
            default_mesh_time = mesh_pair_cmd_interval * 2;
        }
        #endif
    }else if(params[0] == MESH_PAIR_EFFECT){
        effect_new_mesh = 1;
    }
    else if(params[0] == MESH_PAIR_DEFAULT_MESH){
        #if !GATEWAY_EN
        default_mesh_effect_delay_ref = clock_time() | 1;
        default_mesh_time = params[1] * 1000;
        #endif
    }
}

u8 get_online_node_cnt(void)
{
    u8 cnt = 0;
	foreach(i, mesh_node_max){
	    if(mesh_node_st[i].tick){
	        cnt++;
	        if(i > 0){
	            mesh_pair_notify_rsp_mask[mesh_node_st[i].val.dev_adr / 8] |= BIT(mesh_node_st[i].val.dev_adr % 8);
	        }
	    }
	}
#if(PROVISIONING_ENABLE)
    if(gateway_status == GATEWAY_STATUS_CFG_UNPRO_DEV ||\
       gateway_status == GATEWAY_STATUS_ADD_CONFIGURED_DEVICE)
    {
        /* If provisioning device to network, shall at least return two device */
        if(cnt < 2)
        {
            return 2;
        }
    }
#endif
	return cnt;
}

void mesh_pair_proc(void)
{
    static u32 mesh_pair_time = 0;
    static u8 mesh_pair_state = MESH_PAIR_NAME1;
    u16 dst_addr = 0xFFFF;
    u8 op_para[16] = {0};

    
    #if GATEWAY_EN && PROVISIONING_ENABLE
    if(default_mesh_time_ref && clock_time_exceed_lib(default_mesh_time_ref, default_mesh_time * 1000))
    {
        /* Time out then start setting */
        switch(gateway_status)
        {
            case GATEWAY_STATUS_SWITCH_TO_DEFAULT_MESH:
                switch_to_default_mesh(scan_dev_time_out);
                if(scan_dev_time_out == 0xff || scan_dev_time_out == 0x00)
                {
                    gateway_status = GATEWAY_STATUS_TEMP_DEFALT_MESH;
                }
                else
                {
                    gateway_status = GATEWAY_STATUS_SCAN_UNPROV_DEV;
                }
                break;
                
            case GATEWAY_STATUS_SCAN_UNPROV_DEV:
                pair_setting_flag = PAIR_SET_MESH_TX_START;
                gateway_status = GATEWAY_STATUS_CFG_UNPRO_DEV;
                default_mesh_time_ref = 0;
                break;
                
            case GATEWAY_STATUS_TEMP_DEFALT_MESH:
                /* temporary default mesh time out shall change to normal */
                if(!scan_dev_time_out)
                {
                    save_effect_new_mesh();
                }
                else
                {
                    default_mesh_time_ref = clock_time() | 1;
                }
                break;
            default:
                break;
        }
    }
    #else
    if(default_mesh_effect_delay_ref && clock_time_exceed_lib(default_mesh_effect_delay_ref, MESH_PAIR_CMD_INTERVAL * 1000))
    {
        default_mesh_effect_delay_ref = 0;
       
        if(default_mesh_time == 0x00)
        {
            pair_load_key();
            default_mesh_time_ref = 0;
        }
        else
        {
            switch_to_default_mesh(default_mesh_time / 1000);
            default_mesh_time_ref = clock_time() | 1;
        }
    }
    else if(default_mesh_time_ref && clock_time_exceed_lib(default_mesh_time_ref, default_mesh_time * 1000))
    {
        /* Switch to normal mesh */
        if(default_mesh_time == 255000)
        {
            default_mesh_time_ref = clock_time() | 1;
        }
        else
        {
            pair_load_key();
            default_mesh_time_ref = 0;
        }
    }
    #endif
    if(mesh_pair_start_time && clock_time_exceed_lib(mesh_pair_start_time, mesh_pair_timeout*1000*1000)){
        //mesh pair time out 
        pair_load_key();
        memset4(new_mesh_name, 0, 16);
        memset4(new_mesh_pwd, 0, 16);
        memset4(new_mesh_ltk, 0, 16);
        mesh_pair_state = MESH_PAIR_NAME1;
        mesh_pair_start_notify_time = mesh_pair_retry_cnt = mesh_pair_start_time = 0;
        memset4(mesh_pair_notify_rsp_mask, 0, 32);
        pair_setting_flag = PAIR_SETTED;
        rf_link_light_event_callback(LGT_CMD_MESH_PAIR_TIMEOUT);
        return;
    }
    
    if(pair_setting_flag == PAIR_SET_MESH_TX_START && (mesh_pair_state == MESH_PAIR_NAME1) && get_online_node_cnt() == 1){
        op_para[0] = LGT_CMD_MESH_PAIR;
        op_para[3] = MESH_PAIR_EFFECT;
        dst_addr = 0x0000;// there is noly one device in mesh,just effect itself.
        mesh_pair_state = MESH_PAIR_NAME1;
        mesh_pair_start_notify_time = mesh_pair_retry_cnt = mesh_pair_start_time = 0;
        memset4(mesh_pair_notify_rsp_mask, 0, 32);
        pair_setting_flag = PAIR_SETTED;
    }else if(pair_setting_flag >= PAIR_SET_MESH_TX_START && clock_time_exceed(mesh_pair_time, mesh_pair_cmd_interval*1000)){
        mesh_pair_time = clock_time();
        if(pair_setting_flag == PAIR_SET_MESH_TX_START){
            op_para[0] = LGT_CMD_MESH_PAIR;
            op_para[3] = mesh_pair_state;
            if(mesh_pair_state == MESH_PAIR_NAME1){
                // send mesh name [0-7]
        		memcpy(op_para + 4, pair_nn, 8);
        		mesh_pair_state = MESH_PAIR_NAME2;
            }else if(mesh_pair_state == MESH_PAIR_NAME2){
                // send mesh name [8-15]
        		memcpy(op_para + 4, pair_nn + 8, 8);
        		mesh_pair_state = MESH_PAIR_PWD1;
            }else if(mesh_pair_state == MESH_PAIR_PWD1){
                // send mesh pwd [0-7]
        		memcpy(op_para + 4, pair_pass, 8);
        		mesh_pair_state = MESH_PAIR_PWD2;
            }else if(mesh_pair_state == MESH_PAIR_PWD2){
                // send mesh pwd [8-15]
        		memcpy(op_para + 4, pair_pass + 8, 8);
        		mesh_pair_state = MESH_PAIR_LTK1;
            }else if(mesh_pair_state == MESH_PAIR_LTK1){
                // send mesh ltk [0-7]
        		memcpy(op_para + 4, pair_ltk_mesh, 8);
        		mesh_pair_state = MESH_PAIR_LTK2;
            }else if(mesh_pair_state == MESH_PAIR_LTK2){
                // send mesh ltk [8-15]
        		memcpy(op_para + 4, pair_ltk_mesh + 8, 8);
        		mesh_pair_state = MESH_PAIR_NAME1;
        		pair_setting_flag = PAIR_SET_MESH_TX_DONE;
            }else{
                mesh_pair_state = MESH_PAIR_NAME1;
                mesh_pair_start_notify_time = mesh_pair_retry_cnt = mesh_pair_start_time = 0;
                memset4(mesh_pair_notify_rsp_mask, 0, 32);
        		pair_setting_flag = PAIR_SETTED;
        		return;
            }
        }else if(pair_setting_flag == PAIR_SET_MESH_TX_DONE){
            // get mesh nodes' confirm value
            //rf_link_slave_read_status_start();
            op_para[0] = LGT_CMD_MESH_OTA_READ;
            op_para[3] = 0x10;// bridge cnt
            op_para[4] = PAR_READ_MESH_PAIR_CONFIRM;
            pair_setting_flag = PAIR_SET_MESH_RX_DONE;
            mesh_pair_start_notify_time = clock_time() | 0;
            foreach(i, 8){
                mesh_pair_checksum[i] = get_mesh_pair_checksum(i);
            }
        }else if(pair_setting_flag == PAIR_SET_MESH_RX_DONE){
            u8 zero_buff[32] = {0};
            u8 effect_flag = 0;
            effect_flag = !memcmp(mesh_pair_notify_rsp_mask, zero_buff, 32);
            if(!effect_flag && clock_time_exceed_lib(mesh_pair_start_time, MESH_PAIR_NOTIFY_TIMEOUT*1000)){
                if(mesh_pair_retry_cnt++ < mesh_pair_retry_max){
                    mesh_pair_start_time = clock_time() | 1;
                    pair_setting_flag = PAIR_SET_MESH_TX_START;
                    mesh_pair_state = MESH_PAIR_NAME1;
                }else{
                    // retry timeout, effect or cancel?? effect now
                    effect_flag = 1;
                }
            }
            if(effect_flag){
                //send cmd to switch to new mesh
                op_para[0] = LGT_CMD_MESH_PAIR;
                op_para[3] = MESH_PAIR_EFFECT_DELAY;
                mesh_pair_state = MESH_PAIR_NAME1;
                mesh_pair_start_notify_time = mesh_pair_retry_cnt = mesh_pair_start_time = 0;
                memset4(mesh_pair_notify_rsp_mask, 0, 32);
                pair_setting_flag = PAIR_SETTED;
            }
        }
    }else{
        return;
    }
    
    light_slave_tx_command(op_para, dst_addr);
}
#else
u8 mesh_pair_enable = 0;
void mesh_pair_proc(void)
{
    return;
}

u8 mesh_pair_notify_refresh(rf_packet_att_cmd_t *p){
    return 1;
}
#endif
#if 1	// no use internal
/*
int user_cmd2app_cmd()
cmd: mesh command (0--3F)
*/
int user_cmd2app_cmd(u8 cmd, u16 dst_adr, u8 *par, u8 par_len){	// should be in connected state
    if(par_len > 10 || (!(slave_link_connected))){
        return -1;
    }

    // packet 
	rf_packet_ll_app_t  pkt_app_data = {};
    memset(&pkt_app_data, 0, sizeof(pkt_app_data));
    pkt_app_data.type = 0x02;
    pkt_app_data.rf_len = 17 + par_len;
    pkt_app_data.dma_len = pkt_app_data.rf_len + 2;
    pkt_app_data.l2capLen = pkt_app_data.rf_len - 4;
    pkt_app_data.chanId = 0x04;
    pkt_app_data.opcode = ATT_OP_WRITE_CMD;
    pkt_app_data.handle= 0x15;//handle & 0xff;
    pkt_app_data.handle1 = 0x00;//(handle >> 8) & 0xff;

    static u8 user_sno_cmd_init_flag = 1;
	static int user_sno_cmd;
	if(user_sno_cmd_init_flag){
		user_sno_cmd_init_flag = 0;
		user_sno_cmd = ((clock_time() + device_address) & 0xff) << 16;;		// should be initialized as random value.
	}
	
    user_sno_cmd++;
    if(0 == user_sno_cmd){
    	user_sno_cmd = 1;		// make sure not zero.
    }
    
    memcpy(pkt_app_data.app_cmd_v.sno, &user_sno_cmd, 3);
    //memcpy(pkt_app_data.app_cmd_v.src, &device_address, 2);
    memcpy(pkt_app_data.app_cmd_v.dst, &dst_adr, 2);
    pkt_app_data.app_cmd_v.op = (cmd & 0x3F) | 0xC0;
    pkt_app_data.app_cmd_v.vendor_id = VENDOR_ID;
    memcpy(pkt_app_data.app_cmd_v.par, par, par_len);
    
    // handle and send command
    u8 r = irq_disable();
    rf_link_slave_data_write_no_dec((rf_packet_ll_data_t *)(&pkt_app_data));
    irq_restore(r);
    
    return 0;
}
#endif

/////////////////// set and get mac by mesh  //////////////////////
u8 get_mac_en = 1;		// no use in library

int dev_addr_with_mac_flag(u8 *params)
{
	return (DEV_ADDR_PAR_WITH_MAC == params[2]);
}

int dev_addr_with_mac_match(u8 *params)
{
	if((params[0] == 0xff)&&(params[1] == 0xff)){	// get
		return get_mac_en;
	}else{											// set
		return (!memcmp(params+4, slave_p_mac, 6));
	}
}

int dev_addr_with_mac_rsp(u8 *params, u8 *par_rsp)
{
	if(dev_addr_with_mac_match(params)){
        dual_mode_set_adv_provisoning_flag();
	    
		memcpy(par_rsp, &device_address, 2);
		memcpy(par_rsp + 2, slave_p_mac, 6);
		memcpy(par_rsp + 8, &adv_rsp_pri_data.ProductUUID, 2);
		return 1;
	}
	return 0;
}

/////////////////// Adjust TP according to the temperature  //////////////////////
#if  ADC_TEMP_ENABLE
#define ADC_TEMP_SAMP_PERIOD		3000000		// unit: us
#define TEMP_ADC_TP_CRIT_VALUE		5

#define	TEMP_HIGH_VAL				(80)		// degree centigrade

u8			adc_adj_tp_flag = 1;
u8 			rf_tp_base_init;
u8 			rf_tp_delta;

u8 adc_adj_tp_map[][2]=
{
	{25,0x11},				// room temperature
//	{40,0x12},
//	{60,0x13},
	{TEMP_HIGH_VAL,0x14},
//	{100,0x15},
//	{120,0x16},
};

extern int rf_tp_base,rf_tp_gain;

void adc_adj_tp_init(void)
{
	if (*(u8 *) FLASH_ADR_TP_HIGH != 0xFF){
		rf_tp_delta = *(u8 *) FLASH_ADR_TP_LOW - *(u8 *) FLASH_ADR_TP_HIGH;
    }
	else
	{
		rf_tp_delta = rf_get_default_tp_delta();
	}

	rf_tp_base_init = rf_tp_base;
	
	adc_adj_tp_map[0][1] = rf_tp_base_init;	//tp base  when temperature = 25
	
	foreach_arr(i,adc_adj_tp_map){
		if(adc_adj_tp_map[i][0] == TEMP_HIGH_VAL){
			adc_adj_tp_map[i][1] = rf_tp_base_init + 3;
		}
	}
	
}

void ADC_TempSensorInit(enum ADCINPUTCH chn){
	WriteAnalogReg(0x05, ReadAnalogReg(0x05) & 0xdf);
	adc_ChannelInit(chn, SINGLEEND, RV_AVDD, RES14, S_3);
}


#define ADC_SIZE 10

unsigned short TEMSENSOR_N_buff[ADC_SIZE] = {0x00};
unsigned short TEMSENSOR_P_buff[ADC_SIZE] = {0x00};
unsigned int TEMSENSOR_N_value = 0;
unsigned int TEMSENSOR_P_value = 0;
unsigned int TEMSENSOR_voltage = 0;   		//0.00mv
int adc_temperature = 0;				//temperature value 0.0 degree
unsigned int dif_PN = 0;        			//P_ch - N_ch

int adc_get_temp(void)
{
	unsigned short i = 0;
	unsigned int temp = 0;
	static unsigned int sum = 0;

	//Init AD ch TEMSENSORN
	ADC_TempSensorInit(TEMSENSORN);
	for(i=0;i<ADC_SIZE+8;i++){
		if(i<8)
		{
			temp = adc_SampleValueGet();////get ADC value of pin B1
		}
		else
		{
			TEMSENSOR_N_buff[i-8] = adc_SampleValueGet();////get ADC value of pin B1
			sum += TEMSENSOR_N_buff[i-8];
		}
	}
	sum /= ADC_SIZE;
	if(sum < 128){
		sum = 128;
	}
	TEMSENSOR_N_value = sum;
	sum = 0;

	//Init AD ch TEMSENSORP
	ADC_TempSensorInit(TEMSENSORP);
	for(i=0;i<ADC_SIZE+8;i++){
		if(i<8)
		{
			temp = adc_SampleValueGet();////get ADC value of pin B1
		}
		else
		{
			TEMSENSOR_P_buff[i-8] = adc_SampleValueGet();////get ADC value of pin B1
			sum += TEMSENSOR_P_buff[i-8];
		}
	}
	sum /= ADC_SIZE;
	if(sum < 128){
		sum = 128;
	}
	TEMSENSOR_P_value = sum;
	sum = 0;

	dif_PN = TEMSENSOR_P_value - TEMSENSOR_N_value;
	TEMSENSOR_voltage = 306100*(dif_PN-128)/(16383-256);
	temp = (TEMSENSOR_voltage-14362)*1650/10006 - 400;

	return temp;
}

unsigned char adc_temp_proc(void){
	
	//static u32 temp_time_dly,temp_time_dly2;
	
	//temp_time_dly = clock_time();
	u8 r = irq_disable();

	adc_temperature = adc_get_temp();
	
	adc_chn_ref_init();			// recover channel and reference configure

	irq_restore(r);
	//temp_time_dly2 = (clock_time() - temp_time_dly)/CLOCK_SYS_CLOCK_1US;
	
	return 1;
}

void adc_adj_tp_proc(void)
{
	u8 tmp_tp_base = rf_tp_base;

	foreach_arr(i,adc_adj_tp_map){
		if(adc_temperature > (adc_adj_tp_map[i][0]*10+TEMP_ADC_TP_CRIT_VALUE*10)&& rf_tp_base < adc_adj_tp_map[i][1]){
			tmp_tp_base = adc_adj_tp_map[i][1];
		}
		else if(rf_tp_base == adc_adj_tp_map[i][1] && adc_temperature < (adc_adj_tp_map[i][0]*10-TEMP_ADC_TP_CRIT_VALUE*10)){
			if(i>0){
				tmp_tp_base = adc_adj_tp_map[i-1][1];
			}
		}
	}

	if(tmp_tp_base != rf_tp_base){
		rf_tp_base = tmp_tp_base;
		rf_tp_gain = rf_get_tp_gain(rf_tp_base, rf_tp_base-rf_tp_delta);
	}
}

void adc_adj_tp_handle(void)
{
	static u32 adj_tp_check_time;
	
	if((!rf_slave_ota_busy) && adc_adj_tp_flag && clock_time_exceed(adj_tp_check_time, ADC_TEMP_SAMP_PERIOD)){
		if(adc_temp_proc()){
			adj_tp_check_time = clock_time();
			adc_adj_tp_proc();
		}
	}
}

void adc_adj_tp_reset(void)
{
	if(rf_tp_base != rf_tp_base_init){
		rf_tp_base = rf_tp_base_init;
		rf_tp_gain = rf_get_tp_gain(rf_tp_base, rf_tp_base-rf_tp_delta);
	}
	if(adc_adj_tp_flag){
		adc_adj_tp_flag = 0;
	}
}

#endif

void mesh_pair_init(void){
#if(MESH_PAIR_ENABLE)
	mesh_pair_enable = 1;
	mesh_pair_cmd_interval = MESH_PAIR_CMD_INTERVAL;
	mesh_pair_timeout = MESH_PAIR_TIMEOUT;
#endif
}

void mesh_pair_proc_effect(void){
#if(MESH_PAIR_ENABLE)
    if(effect_new_mesh || (effect_new_mesh_delay_time && (clock_time_exceed_lib(effect_new_mesh_delay_time, mesh_pair_cmd_interval*1000)))){
        save_effect_new_mesh();
        effect_new_mesh = effect_new_mesh_delay_time = 0;
    }
#endif
}

void mesh_pair_proc_get_mac_flag(void){
#if(MESH_PAIR_ENABLE)
	get_mac_en = 0; 	// set success
	if(mesh_pair_enable){
		extern int adr_flash_cfg_idx;
		flash_write_page(flash_adr_pairing + adr_flash_cfg_idx + 1, 1, &get_mac_en);
	}
#endif
}

extern ll_packet_l2cap_data_t cmd_delay;

u16 light_cmd_delayed_ms(u8 data){
	u8 ttc_prec = (data >> 6);
	u8 ttc_val 	= (data & 0x3F);
	return (ttc_prec == 0)? ttc_val:\
			(ttc_prec == 1)? ttc_val << 2:\
			(ttc_prec == 2)? ttc_val << 4:\
			(ttc_prec == 3)? ttc_val << 8:0;
}

int is_mesh_cmd_need_delay(u8 *p_cmd, u8 *params, u8 ttc)
{
	u16 delay_tmp;
	delay_tmp = params[1] | (params[2] << 8);
	if(delay_tmp){
		if(cmd_left_delay_ms){
			return 1;
		}
		cmd_delay_ms = delay_tmp;
		if(cmd_delay_ms && !irq_timer1_cb_time){
			u16 cmd_delayed_ms = light_cmd_delayed_ms(ttc);
			if(cmd_delay_ms > cmd_delayed_ms){
				memcpy(&cmd_delay, p_cmd, sizeof(ll_packet_l2cap_data_t));
				cmd_left_delay_ms = cmd_delay_ms - cmd_delayed_ms;
				irq_timer1_cb_time = clock_time();
				return 1;
			}
		}
	}
	return 0;
}

#if 0
void disable_all_initiative_tx ()
{
	iBeaconInterval = 0;
	send_adv_flag = 0;
	mesh_send_online_status_flag = 0;
	pkt_need_relay = 0;
	extern u8 send_local_valid_time_per_30s_en;
	send_local_valid_time_per_30s_en = 0;
}
#endif

#if 1 // sensor function
#define SENSOR_ENABLE       0

u8 sensor_enable = SENSOR_ENABLE;
u16 sensor_last_adv_sleep_time_us = 1200;//us

#if SENSOR_ENABLE
u32 sensor_enter_deep_time = 500;//ms

void sensor_auto_proc(void){
	return;
	// demo code
	static u8 sensor_proc_cnt = 0;
    if(!is_tx_cmd_busy()){   // it must be used, if want enough bridges(default is BRIDGE_MAX_CNT 8).
		u8 op_para[16] = {0};
		op_para[0] = LGT_CMD_LIGHT_ONOFF;
		op_para[3] = (++sensor_proc_cnt)%2;//params[0], on or off
		light_slave_tx_command(op_para, 0xFFFF);//0xFFFF or 0x8001......

		while(mesh_send_user_command()){
			u8 r = irq_disable();
			cpu_sleep_wakeup (0, PM_WAKEUP_TIMER, clock_time()+10*tick_per_us) ;
			irq_restore (r);
		}
    }
}
#endif

void sensor_enter_deep_cb(void){
    #if SENSOR_ENABLE
	sensor_auto_proc();
	gpio_write(GPIO_PD7, 0);
	cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM, PM_WAKEUP_TIMER, clock_time() + (sensor_enter_deep_time*1000)*tick_per_us);
	while(1);
    #endif
}
#endif

#if 0 // i2c read demo code
#if (MCU_CORE_TYPE == MCU_CORE_8258 && I2C_HW_MODOULE_EN)
void i2c_read_hx300t()
{
    u8 r = irq_disable();
    i2c_write_byte((0x44<<1), 1, 0);
    sleep_us(35*1000);
    
    u8 val_buffer[4] = {0};
    i2c_read_series(0, 0, val_buffer, 4);
    static u16 tempreture, humidity = 0;
    humidity = ((val_buffer[0]<<8) | val_buffer[1]) & 0x3fff;
    tempreture = ((val_buffer[2]<<8) | val_buffer[3]) >> 2;
    humidity = humidity * 100 / 0x3fff;
    tempreture = tempreture * 165 / 0x3fff - 40;
    irq_restore(r);
}
#endif
#endif

// ----friend ship
// ---- common 
#if 1
void clear_rx_buffer()
{
    reg_rf_irq_status = FLD_RF_IRQ_RX;		// clear current rx in buffer
}

int is_default_mesh()
{
    u8 name_def[17] = {MESH_NAME};
    u8 ps_def[17] = {MESH_PWD};
    return ((!memcmp(name_def, pair_nn, 16))&&(!memcmp(ps_def, pair_pass, 16)));
}

u8 is_wakeup_by_gpio()
{
    return (FLAG_WAKEUP_GPIO == get_wakeup_src_flag());
}

static inline u32 get_sno_rand_base()
{
    return (clock_time() & 0x003f0000) + rand();
}

u32 get_current_chn_array_idx()
{
    u32 chn = read_reg8(0x80040d);
    foreach(i,ARRAY_SIZE(sys_chn_listen)){
        if(sys_chn_listen[i] == chn){
            return i;
        }
    }
    return 0;
}

u16 get_remain_tx_time_us(u32 enc_time_us)  // enc_time_us: encryption data time later
{
    u16 t_us = (TX_ONE_RF_CYCLE_US * (ARRAY_SIZE(sys_chn_listen) - get_current_chn_array_idx() - 1));   // calculate decryption time later
    u32 cost = enc_time_us + ((clock_time() - rcv_pkt_time) / CLOCK_SYS_CLOCK_1US);
    if(t_us > cost){
        return (t_us - cost);
    }
    return 0;
}

void message_cache_buf_init()
{
    memset(&rc_pkt_buf, 0, sizeof(rc_pkt_buf));    // init
    memset(slave_sno, 0xff, sizeof(slave_sno));
    slave_link_cmd = 0xff;
}

void mesh_enc_pkt(mesh_pkt_t *pkt)
{
    if(security_enable){
        pkt->type |= BIT(7);
        pair_enc_packet_mesh ((u8 *)pkt);
    }
}

void fn_mesh_enc_pkt(mesh_pkt_t *pkt)
{
    pkt->handle1 = FLD_HANDLE1_FN_FLAG;
    mesh_enc_pkt(pkt);
}

int mesh_pkt_assemble (mesh_pkt_t *pkt, int sno, u16 dst, u8 op, u8 *par, u8 len, u8 ttl)
{
    op = op | 0xc0;
    u8 long_par_flag = is_cmd_long_par(op) || ((op & 0x3f) >= 0x30);
    if(len > (long_par_flag ? 15 : 10)){
        return 0;
    }

    u8 r = irq_disable();   // must, because of pkt would be used in irq.
    memset((u8 *)pkt, 0x00, sizeof(mesh_pkt_t));
	pkt->dma_len = 39;//26 + len;
    pkt->type = FLG_BLE_LIGHT_DATA;
    pkt->rf_len = pkt->dma_len - 2;
    pkt->l2capLen = pkt->dma_len - 6;
    pkt->chanId = FLG_RF_MESH_CMD;
    pkt->src_tx = device_address;
    pkt->handle1 = 0;
    memcpy(pkt->sno, &sno, sizeof(pkt->sno));
	pkt->src_adr = device_address;
	pkt->dst_adr = dst;
	pkt->op = op;
	pkt->vendor_id = VENDOR_ID;
	memcpy(pkt->par, par, min(len, 10));
	if(len > 10){ // for cyber security check
		memcpy(pkt->internal_par1, par+10, len-10);
	}
	pkt->ttl = ttl;
    irq_restore(r);
    
	return 1;
}

#if FEATURE_FRIEND_EN
#if FN_DEBUG_PIN_EN
_attribute_ram_code_ void fn_debug_alter_debug_pin(int reset)
{
    static u8 debug_pin_level = 0;
    if(reset){
        debug_pin_level = 0;
    }else{
        debug_pin_level = !debug_pin_level;
    }
    
    fn_debug_set_debug_pin(debug_pin_level);
}
#endif

static int fn_cmd_sno;
fs_proc_fn_t fs_proc_fn[FN_CACHE_LPN_MAX];

#define FN_REQUEST_RANGE_MS     (LPN_REQUEST_REC_WIN_MS - 4)

STATIC_ASSERT(0 == sizeof(fs_proc_fn_t)%4); // because pkt must u32 aligned, use fifo later.
STATIC_ASSERT(FN_REQUEST_RANGE_MS > 3);
STATIC_ASSERT(10 == sizeof(fs_tx_cmd_par_t));

void friendship_tx_cmd_fn(u8 sub_cmd, fs_proc_fn_t *p_proc)
{
    if(0 == fn_cmd_sno){
        fn_cmd_sno = get_sno_rand_base();
    }
    
    fs_tx_cmd_par_t cmd = {0};
    cmd.sub_cmd = sub_cmd;

    u8 chn_idx = p_proc->chn_idx;
    u32 retransmit_cnt = FN2LPN_RETRANSMIT_CNT;
    if(CMD_CTL_OFFER == sub_cmd){
        //chn_idx = CHANEL_ALL;
        retransmit_cnt = 1; // don't send too much, because maybe there is many FN sending offer
    }else if(CMD_CTL_UPDATE == sub_cmd){
        cmd.updata_poll_type = p_proc->poll_type;
    }else if(CMD_CTL_GROUP_REPORT_ACK == sub_cmd){
    }else{
        return ;
    }

    u8 r = irq_disable();
    static mesh_pkt_t pkt = {0};    // use global now, comfirm later.
    mesh_pkt_assemble(&pkt, (fn_cmd_sno++), p_proc->lpn_adr, LGT_CMD_FRIENDSHIP, (u8 *)&cmd, sizeof(cmd), 0);
    fn_mesh_enc_pkt(&pkt);
    fn_debug_set_TX_pin(1);
    mesh_send_command((u8 *)&pkt, chn_idx, retransmit_cnt);
    fn_debug_set_TX_pin(0);
    irq_restore(r);
}

fs_proc_fn_t * get_proc_by_request(u16 lpn_adr)
{
    fs_proc_fn_t *p_proc_idle = 0;
    foreach(i,FN_CACHE_LPN_MAX){
        if(fs_proc_fn[i].lpn_adr == lpn_adr){
            return &fs_proc_fn[i];
        }else if((!p_proc_idle) && (0 == fs_proc_fn[i].lpn_adr)){
            p_proc_idle = &fs_proc_fn[i];
        }
    }

    return p_proc_idle;
}

void fn_poll_par_handle(fs_proc_fn_t *p_proc, fs_tx_cmd_par_t *p_fs)
{
    if(POLL_TYPE_ONLINE_ST == p_fs->par_type){
        mesh_node_st_val_t st = {0};
        st.dev_adr = p_proc->lpn_adr;
        st.sn = p_proc->online_st_sn++;
        memcpy(st.par, p_fs->online_st_par, sizeof(st.par));
        mesh_node_update_status((u8 *)&st, 1);
    }else if(POLL_TYPE_GROUP_OW == p_fs->par_type){
        memset(p_proc->group_list, 0, sizeof(p_proc->group_list));
        u32 len = min(sizeof(p_proc->group_list), sizeof(p_fs->group));
        memcpy(p_proc->group_list, p_fs->group, len);
    }
}

void set_proc_st_fn(fs_proc_fn_t *p, u8 st)
{
    p->st = st;
    p->tick_st = clock_time();
}

void clear_proc_st_fn(fs_proc_fn_t *p)
{
    memset(p, 0, sizeof(fs_proc_fn_t));
}

//_attribute_ram_code_
int fn_rx_friendship_cmd_proc(mesh_pkt_t *pkt) // only handle friend ship command.
{
    fn_debug_set_irq_pin(0);
    fs_tx_cmd_par_t *p_fs = (fs_tx_cmd_par_t *)(pkt->par);
    if(CMD_CTL_REQUEST == p_fs->sub_cmd){
        fs_proc_fn_t *p_proc = get_proc_by_request(pkt->src_adr);
        if(p_proc){
            clear_proc_st_fn(p_proc);   // init
            p_proc->PollTimeout = p_fs->PollTimeout;
            p_proc->lpn_adr = pkt->src_adr;
            p_proc->chn_idx = p_fs->chn_idx_req;
            set_proc_st_fn(p_proc, ST_FN_OFFER);
            
            u16 delay_us = get_remain_tx_time_us(ENCRYPTION_TIME_US) + ((FN_REQUEST_RANGE_MS * ((rand() & 0x3ff)))/1024)*1000;   // 4ms: margin and tx time
            sleep_us(delay_us);
            friendship_tx_cmd_fn(CMD_CTL_OFFER, p_proc);
        }
    }else{
        if(pkt->dst_adr != device_address){ // must unicast match
            return -1;
        }

        foreach(i,FN_CACHE_LPN_MAX){
            fs_proc_fn_t *p_proc = &fs_proc_fn[i];
            if(p_proc->lpn_adr == pkt->src_adr){
                if(CMD_CTL_POLL == p_fs->sub_cmd){
                    p_proc->poll_type = p_fs->par_type;
                    p_proc->chn_idx = p_fs->chn_idx;
                    
                    int cache_tx = 0;
                    if(ST_FN_OFFER == p_proc->st){
                        p_proc->st = ST_FN_LINK_OK;   // establish complete
                    }else if(ST_FN_LINK_OK == p_proc->st){
                        if(ST_CACHE_RX == p_proc->cache_flag){
                            p_proc->cache_flag = ST_CACHE_TX2LPN;
                            p_proc->FSN = p_fs->FSN;
                        }else if(ST_CACHE_TX2LPN == p_proc->cache_flag){
                            if(p_proc->FSN != p_fs->FSN){
                                p_proc->cache_flag = 0;
                            }
                        }
                        cache_tx = p_proc->cache_flag;
                    }

                    if(ST_FN_LINK_OK == p_proc->st){
                        fn_debug_set_TX_pin(1);
                        sleep_us(get_remain_tx_time_us(cache_tx ? 0 : ENCRYPTION_TIME_US));
                        fn_debug_set_TX_pin(0);
                        if(cache_tx){
                            fn_debug_set_TX_pin(1);
                            mesh_send_command((u8 *)&p_proc->pkt_cache, p_proc->chn_idx, FN2LPN_RETRANSMIT_CNT);
                            fn_debug_set_TX_pin(0);
                        }else{
                            friendship_tx_cmd_fn(CMD_CTL_UPDATE, p_proc);
                        }
                        
                        p_proc->tick_st = clock_time();
                    }
                    
                    fn_poll_par_handle(p_proc, p_fs);
                }
                
                break;
            }
        }
    }

    clear_rx_buffer();
    return 0;
}

int is_group_list_for_lpn(fs_proc_fn_t *p_proc, u16 adr_dst)
{
    if(adr_dst & 0x8000){
        foreach_arr(i,p_proc->group_list){
            if(p_proc->group_list[i] == adr_dst){
                return 1;
            }
        }
    }
    return 0;
}

void fn_rx_push_to_cache(u8 *p) // call in irq handle
{
#if NOTIFY_MESH_COMMAND_TO_MASTER_EN
    nctm_rx_gatt_message_cb(p);
#endif

    mesh_pkt_t *pkt = (mesh_pkt_t *)p;
    foreach(i,FN_CACHE_LPN_MAX){
        fs_proc_fn_t *p_proc = &fs_proc_fn[i];
        if((ST_FN_LINK_OK == p_proc->st)
        && (p_proc->lpn_adr == pkt->dst_adr || is_group_list_for_lpn(p_proc, pkt->dst_adr) /*|| (0xffff == pkt->dst_adr)*/)){
            memcpy(&p_proc->pkt_cache, p, sizeof(p_proc->pkt_cache));
            fn_mesh_enc_pkt(&p_proc->pkt_cache); // encryption first
            p_proc->cache_flag = ST_CACHE_RX;
        }
    }
}


STATIC_ASSERT((LPN_POLL_TIMEOUT_100MS) < ((0xFFFFFFFF / CLOCK_SYS_CLOCK_HZ)*10));
STATIC_ASSERT((FN_POLL_2_OFFER_TIMEOUT_MS) > (LPN_POLL_RETRY_MAX * (LPN_POLL_RETRY_DELAY_MS +  LPN_POLL_REC_WIN_MS)));

void friendship_proc_fn()
{
    foreach(i,FN_CACHE_LPN_MAX){
        fs_proc_fn_t *p_proc = &fs_proc_fn[i];
        if(p_proc->st){
            u32 timeout_ms = ((ST_FN_OFFER == p_proc->st) ? (FN_POLL_2_OFFER_TIMEOUT_MS) : (p_proc->PollTimeout * 100)) + 100;
            if(clock_time_exceed(p_proc->tick_st, timeout_ms * 1000)){
                clear_proc_st_fn(p_proc);
                if(ST_FN_LINK_OK == p_proc->st){
                    // friend_ship_disconnect_fn
                }
            }
        }
    }
}


#elif __PROJECT_LPN__

fs_proc_lpn_t fs_proc_lpn = {
    .st = 0, // ST_LPN_REQUEST,
};

static inline void irq_rx_disable()
{
    reg_irq_mask &= ~FLD_IRQ_ZB_RT_EN;
}

void set_proc_st_lpn(u8 st)
{
    fs_proc_lpn.st = st;
    fs_proc_lpn.tick_st = clock_time();
}

u8 get_proc_st_lpn()
{
    return fs_proc_lpn.st;
}

void lpn_proc_st_init()
{
    if(!is_default_mesh()){
        set_proc_st_lpn(ST_LPN_REQUEST);
    }
}

u32 lpn_get_rand_ms()
{
    return (rand() & 0x1f); // 0~31ms
}

void lpn_suspend_ms(int sleep_mode, u32 ms, int gpio_wakeup_en)
{
    u8 r = irq_disable();
    lpn_debug_set_RX_pin(0);
    lpn_debug_set_current_pin(0);
    #if DEBUG_SUSPEND
    foreach(i,ms){
        #if(MODULE_WATCHDOG_ENABLE)
        wd_clear();
        #endif
        sleep_us(1000);
    }
    #else
        #if (!PM_DEEPSLEEP_RETENTION_ENABLE)
    if(DEEPSLEEP_MODE_RET_SRAM == sleep_mode){
        sleep_mode = SUSPEND_MODE;
    }
        #endif
    int pm_wake_gpio = gpio_wakeup_en ? ((SUSPEND_MODE == sleep_mode) ? PM_WAKEUP_CORE : PM_WAKEUP_PAD) : 0;
        #if LPN_DEBUG_PIN_EN
    pm_wake_gpio = 0;	// can't set gpio wakeup, because use dongle instead of switch panel.
        #endif
    cpu_sleep_wakeup (sleep_mode, PM_WAKEUP_TIMER | pm_wake_gpio, clock_time () + ms * CLOCK_SYS_CLOCK_1MS) ;
    key_wakeup_flag = is_wakeup_by_gpio();
    #endif
    clear_rx_buffer();
    lpn_debug_set_current_pin(1);
    irq_restore (r);
}

void friendship_tx_cmd_lpn(u8 sub_cmd, u16 adr_dst)
{
    extern u8 cmd_sno;
    fs_tx_cmd_par_t cmd = {0};
    cmd.sub_cmd = sub_cmd;
    
    u32 retransmit_cnt = 0; // total count = n+1;  should be clear after tx completed.
    u8 chn_idx = st_listen_no & 3;
    if(CMD_CTL_REQUEST == sub_cmd){
        retransmit_cnt = 2;
        cmd.PollTimeout = LPN_POLL_TIMEOUT_100MS;
        cmd.chn_idx_req = chn_idx;
        cmd.PollInterval = (LPN_POLL_INTERVAL_MS + 99)/100;
    }else if(CMD_CTL_POLL == sub_cmd){
        fs_proc_lpn.poll_cnt++;
        fs_proc_lpn.rx_update_flag = 0;
        cmd.FSN = fs_proc_lpn.FSN;
        cmd.chn_idx = chn_idx;
        cmd.par_type = fs_proc_lpn.poll_type;
        if(POLL_TYPE_ONLINE_ST == fs_proc_lpn.poll_type){
            get_online_st_par(cmd.online_st_par);
        }else if(POLL_TYPE_GROUP_OW == fs_proc_lpn.poll_type){
            u32 cnt = 0;
            foreach(i,MAX_GROUP_NUM){
                if(group_address[i]){
                    cmd.group[cnt++] = group_address[i];
                    if(cnt >= ARRAY_SIZE(cmd.group)){
                        break;
                    }
                }
            }
        }
    }else if(CMD_CTL_GROUP_REPORT == sub_cmd){
    }else{
        return ;
    }
    
    if(0 == cmd_sno){
        cmd_sno = get_sno_rand_base();
    }
    
    u8 r = irq_disable();
    
    static mesh_pkt_t pkt = {0};    // use global now, comfirm later.
    mesh_pkt_assemble(&pkt, (cmd_sno++), adr_dst, LGT_CMD_FRIENDSHIP, (u8 *)&cmd, sizeof(cmd), 0);
    mesh_enc_pkt(&pkt);
    lpn_debug_set_TX_pin(1);
    mesh_send_command((u8 *)&pkt, CHANEL_ALL, retransmit_cnt);
    lpn_debug_set_TX_pin(0);
    
    lpn_debug_set_RX_pin(1);
    rf_set_rxmode_mesh_listen();
    st_listen_no++;     // 
    message_cache_buf_init();
    
    reg_irq_mask = FLD_IRQ_ZB_RT_EN;    // only enable RF irq
    clear_rx_buffer();
    extern u32 p_st_handler;
    p_st_handler = 0;   // not use system tick irq.
    
    irq_restore(r);
}

void lpn_send_light_rsp_for_notify_req_cmd()
{
    mesh_pkt_t *p_rsp = (mesh_pkt_t *)&pkt_light_status;
    p_rsp->src_tx = device_address;
    p_rsp->handle1 = 0;
    mesh_enc_pkt(p_rsp);
    foreach(i,BRIDGE_MAX_CNT){
        mesh_send_command((u8 *)&pkt_light_status, CHANEL_ALL, 0);
        u32 delay_ms = 30 + (rand()&15);
        lpn_suspend_ms(SUSPEND_MODE, delay_ms, 0);
        wd_clear();
    }
}

void lpn_ack2poll_proc(fs_tx_cmd_par_t *p_fs)
{
    if(ST_LPN_UPDATE == fs_proc_lpn.st){
        fs_proc_lpn.rx_update_flag = 1;
        fs_proc_lpn.poll_cnt = 0;
        fs_proc_lpn.FSN = !fs_proc_lpn.FSN;
		if(CMD_CTL_UPDATE == p_fs->sub_cmd){
	        if(fs_proc_lpn.poll_type && (fs_proc_lpn.poll_type == p_fs->updata_poll_type)){
	            fs_proc_lpn.poll_type = 0;  // means FN rx OK.
	        }
		}
        irq_rx_disable();
    }
}

int lpn_rx_friendship_cmd_proc(mesh_pkt_t *pkt)
{
    lpn_debug_set_irq_pin(0);
    
    u8 op = pkt->op & 0x3F;
    fs_tx_cmd_par_t *p_fs = (fs_tx_cmd_par_t *)(pkt->par);
    if(!((slave_link_state == FLG_SYS_LINK_CONNECTED)
          || (pkt->handle1 & FLD_HANDLE1_FN_FLAG))){   // cache messages must from FN node
        return -1;
    }

    if(op == LGT_CMD_FRIENDSHIP){
        if(pkt->dst_adr != device_address){             // must unicast match
            return -1;
        }
        
        if(CMD_CTL_OFFER == p_fs->sub_cmd){
            if(ST_LPN_OFFER == fs_proc_lpn.st){
                set_proc_st_lpn(ST_LPN_POLL);
                fs_proc_lpn.fn_adr = pkt->src_adr;
                fs_proc_lpn.rx_offer_flag = 1;
            }
        }else if(CMD_CTL_UPDATE == p_fs->sub_cmd){
            lpn_ack2poll_proc(p_fs);
        }
    }else{
        lpn_ack2poll_proc(p_fs);
    }
    
    return 0;
}

STATIC_ASSERT(LPN_REQUEST_INTERVAL_MS > LPN_POLL_REC_WIN_MS);

int friendship_proc_lpn()
{
    if(!fs_proc_lpn.st){
        return 0;
    }

    u8 r = irq_disable();
    if(ST_LPN_REQUEST == fs_proc_lpn.st){
        friendship_tx_cmd_lpn(CMD_CTL_REQUEST, 0xffff);
        set_proc_st_lpn(ST_LPN_OFFER);  // should be after tx cmd. because tick st.
    }else if(ST_LPN_OFFER == fs_proc_lpn.st){
        if(clock_time_exceed(fs_proc_lpn.tick_st, (LPN_REQUEST_REC_WIN_MS * 1000))){
            set_proc_st_lpn(ST_LPN_REQUEST);
            lpn_suspend_ms(DEEPSLEEP_MODE_RET_SRAM, LPN_REQUEST_INTERVAL_MS - LPN_REQUEST_REC_WIN_MS + lpn_get_rand_ms(), 1);
        }
    }else if(ST_LPN_POLL == fs_proc_lpn.st){
        if(fs_proc_lpn.rx_offer_flag){
            fs_proc_lpn.rx_offer_flag = 0;
            fs_proc_lpn.poll_type = POLL_TYPE_GROUP_OW;
            friend_ship_establish_ok_cb_lpn();
            lpn_suspend_ms(DEEPSLEEP_MODE_RET_SRAM, 100, 0);   // delay for fn tx ok, comfirm later
        }

        friendship_tx_cmd_lpn(CMD_CTL_POLL, fs_proc_lpn.fn_adr);
        set_proc_st_lpn(ST_LPN_UPDATE);  // should be after tx cmd. because tick st.
    }else if(ST_LPN_UPDATE == fs_proc_lpn.st){
        if(fs_proc_lpn.rx_update_flag){
            fs_proc_lpn.rx_update_flag = 0;
            if(SW_Low_Power_rsp_flag){
                SW_Low_Power_rsp_flag = 0;
                lpn_send_light_rsp_for_notify_req_cmd();
            }
            set_proc_st_lpn(ST_LPN_POLL);
            u32 t_ms = LPN_POLL_INTERVAL_MS + lpn_get_rand_ms();
            if(fs_proc_lpn.poll_report_online_st_now){
                fs_proc_lpn.poll_report_online_st_now = 0;
                t_ms = LPN_POLL_RETRY_DELAY_MS;
            }
            lpn_suspend_ms(DEEPSLEEP_MODE_RET_SRAM, t_ms, 1);
        }else if(clock_time_exceed(fs_proc_lpn.tick_st, (LPN_POLL_REC_WIN_MS * 1000))){
            lpn_debug_set_RX_pin(0);
            if(fs_proc_lpn.poll_cnt >= LPN_POLL_RETRY_MAX){
                friend_ship_disconnect_cb_lpn();
                memset(&fs_proc_lpn, 0, sizeof(fs_proc_lpn));
                fs_proc_lpn.st = ST_LPN_REQUEST;        // restart establish friend ship.
            }else{
                set_proc_st_lpn(ST_LPN_POLL);
                lpn_suspend_ms(DEEPSLEEP_MODE_RET_SRAM, LPN_POLL_RETRY_DELAY_MS + lpn_get_rand_ms(), 0);
            }
        }
    }
    irq_restore(r);

    return 1;
}

#if LPN_DEBUG_PIN_EN
_attribute_ram_code_ void lpn_debug_alter_debug_pin(int reset)
{
    static u8 debug_pin_level = 0;
    if(reset){
        debug_pin_level = 0;
    }else{
        debug_pin_level = !debug_pin_level;
    }
    
    lpn_debug_set_debug_pin(debug_pin_level);
}
#endif

#endif
#endif

#if !FEATURE_FRIEND_EN
void fn_rx_push_to_cache(u8 *p)
{
    #if NOTIFY_MESH_COMMAND_TO_MASTER_EN
    nctm_rx_gatt_message_cb(p);
    #endif
}
#endif

#if 1
void BLE_low_power_handle(u8 mode, u32 key_scan_interval)
{	
    if(mode == 1)
    {
        u8 r = irq_disable();
        cpu_sleep_wakeup (0, PM_WAKEUP_TIMER, clock_time() + key_scan_interval * tick_per_us) ;
        irq_restore (r);
    }
    if(Connect_suspend || ADV_Suspend)
    {
        u8 r = irq_disable();
        u32 wakeup_tick = reg_system_tick_irq;
#if (!((__TL_LIB_8258__ || (MCU_CORE_TYPE == MCU_CORE_8258)) || (__TL_LIB_8278__ || (MCU_CORE_TYPE == MCU_CORE_8278))))
        if(Connect_suspend){
            wakeup_tick -= 150 * tick_per_us;//because of cpu_sleep_wakeup() is not  _attribute_ram_code_
            key_scan_interval = 0;
        }
#endif
        u8 tick_irq_flag = 1;
        if(key_scan_interval){
            if(wakeup_tick - clock_time() > (key_scan_interval + 20*1000) * tick_per_us){
                wakeup_tick = clock_time() + key_scan_interval * tick_per_us;  // key scan interval
                tick_irq_flag = 0;
            }
        }

        if(is_tx_packet_empty_suspend()){
			#if(PM_DEEPSLEEP_RETENTION_ENABLE)
			u8 mode = ADV_Suspend?DEEPSLEEP_MODE_RET_SRAM:0;
			if(tick_irq_flag&&(mode == DEEPSLEEP_MODE_RET_SRAM)){
				Connect_suspend = ADV_Suspend = 0;
			}	
            cpu_sleep_wakeup (mode, PM_WAKEUP_TIMER, wakeup_tick);
			#else
			cpu_sleep_wakeup (0, PM_WAKEUP_TIMER, wakeup_tick);
			#endif
            if(ADV_Suspend && tick_irq_flag && (!Connect_suspend)){
                reg_system_tick_irq = clock_time() + 50 * tick_per_us;  // enter adv mode at once
            	reg_irq_src = FLD_IRQ_SYSTEM_TIMER;
			}
        }
        if(tick_irq_flag){
			Connect_suspend = ADV_Suspend = 0;  // init
			#if(MCU_CORE_TYPE != MCU_CORE_8278)
            reg_system_tick_mode |= FLD_SYSTEM_TICK_IRQ_EN;
			#else
			reg_system_irq_mask |= BIT(2);
			#endif
        }
        irq_restore (r);
    }
}

#endif

#if((__TL_LIB_8258__ || (MCU_CORE_TYPE == MCU_CORE_8258)) || (__TL_LIB_8278__ || (MCU_CORE_TYPE == MCU_CORE_8278)))
_attribute_ram_code_ void get_mul32x32_result(u32 a, u32 b, u64 *result)
{
	u64 res = mul32x32_64(a, b);
	memcpy(result, &res, sizeof(u64));
}
#endif

// adr:0=flag 16=name 32=pwd 48=ltk
// p:data
// n:length
void save_pair_info(int adr, u8 *p, int n)
{
	extern int adr_flash_cfg_idx;
	flash_write_page (flash_adr_pairing + adr_flash_cfg_idx + adr, n, p);
}

//*********************** fifo for LGT_CMD_NOTIFY_MESH (C2) ******************************************//
#if NOTIFY_MESH_FIFO_EN
MYFIFO_INIT(notify_mesh_fifo, (sizeof(notify_mesh_data_t)+2), NOTIFY_MESH_FIFO_CNT);    // 2: sizeof(my_fifo_buf_t.len)

int my_fifo_push_mesh_notify(rf_packet_att_value_t *pp)
{
    int err = -1;
    if(slave_link_connected && pair_login_ok){
        my_fifo_t *p_fifo = &notify_mesh_fifo;
        int exist = 0;
        //u8 cnt = my_fifo_data_cnt_get(p_fifo);
        //u16 src = pp->src[0] + (pp->src[1] << 8);
        foreach(i,p_fifo->num){
            my_fifo_buf_t *p_fifo_buf = (my_fifo_buf_t *)my_fifo_get_offset(p_fifo, i);
            notify_mesh_data_t *p_notify = (notify_mesh_data_t *)p_fifo_buf->data;
            #if NOTIFY_MESH_COMMAND_TO_MASTER_EN
            if((0 == memcmp(pp->sno, p_notify->sno, sizeof(notify_mesh_data_t))))
            #else
            if((0 == memcmp(pp->sno, p_notify->sno, OFFSETOF(notify_mesh_data_t,data)))
            &&(0 == memcmp(pp->val+3, p_notify->data, sizeof(p_notify->data))))
            #endif
            {
                return 0; // repeat
            }
        }

        if(!exist){
            err = my_fifo_push(p_fifo, pp->val+3, SIZEOF_MEMBER(notify_mesh_data_t,data), pp->sno, OFFSETOF(notify_mesh_data_t,data));
            if(err){
                // should not happen here.
                return -1;
            }
        }
    }
    return err;
}

void notify_mesh_fifo_proc ()
{
    my_fifo_t *p_fifo = &notify_mesh_fifo;
    if(p_fifo->rptr != p_fifo->wptr){   // quick handle, so don't use my_fifo_get();
        if(slave_link_connected && pair_login_ok){
        	my_fifo_buf_t *p = (my_fifo_buf_t *)my_fifo_get (p_fifo);
        	if (p){
                notify_mesh_data_t *p_buf = (notify_mesh_data_t *)p->data;
                #if NOTIFY_MESH_COMMAND_TO_MASTER_EN
                if(0 == notify_mesh_command2_master(p_buf))
                #else
                if(0 == light_notify(p_buf->data, 10, (u8 *)&p_buf->src_adr))
                #endif
                {
                    my_fifo_pop (p_fifo);
                }
        	}
        }else{
            p_fifo->rptr = p_fifo->wptr;    // my_fifo_reset(p_fifo);   // clear
        }
    }
}
#endif

//*********************** END ******************************************//

#if (0 == NOTIFY_MESH_COMMAND_TO_MASTER_EN)
void forced_single_cmd_in_ble_interval_handle(u8 *ph){}
void mesh_node_keep_alive_other (){}
void rx_mesh_adv_message_cb(u8 *p, int mac_match){} // don't change data that "p" pointer to.
#endif

//*********************** callback for OPPLE******************************************//
u8 vendor_rf_link_is_notify_req(u8 op){
/* telink original
	return (op == LGT_CMD_LIGHT_STATUS || op == LGT_CMD_LIGHT_GRP_RSP1
			 || op == LGT_CMD_LIGHT_GRP_RSP2 ||  op == LGT_CMD_LIGHT_GRP_RSP3
			 || op == LGT_CMD_ALARM_RSP || op == LGT_CMD_SCENE_RSP || op == LGT_CMD_TIME_RSP || op == LGT_CMD_DEV_ADDR_RSP || op == LGT_CMD_USER_NOTIFY_RSP
			 || op == LGT_CMD_MESH_OTA_READ_RSP);
*/
	return (op == LGT_CMD_LIGHT_READ_STATUS  || op == LGT_CMD_CONFIG_DEV_ADDR  || op == LGT_CMD_USER_NOTIFY_REQ
			 || (op == LGT_CMD_MESH_OTA_READ));

}

u8 vendor_rf_link_is_notify_rsp(u8 op){
/* telink original
 return (op == LGT_CMD_LIGHT_STATUS || op == LGT_CMD_LIGHT_GRP_RSP1
         || op == LGT_CMD_LIGHT_GRP_RSP2 ||  op == LGT_CMD_LIGHT_GRP_RSP3
         || op == LGT_CMD_ALARM_RSP || op == LGT_CMD_SCENE_RSP || op == LGT_CMD_TIME_RSP || op == LGT_CMD_DEV_ADDR_RSP || op == LGT_CMD_USER_NOTIFY_RSP
         || op == LGT_CMD_MESH_OTA_READ_RSP);	
 */
	return (op == LGT_CMD_LIGHT_STATUS || op == LGT_CMD_DEV_ADDR_RSP || op == LGT_CMD_USER_NOTIFY_RSP
		 || op == LGT_CMD_MESH_OTA_READ_RSP);
}

u8 vendor_mesh_op_is_conflict(u8 op, u16 vendor_id){
	return ((op == LGT_CMD_MESH_OTA_READ)&&(vendor_id != 0x01FF)); //oplle use c7 command
}

// must
void register_opple_conflict_interface_callback(){
	send_self_online_status_cycle  = 6;
	p_vendor_rf_link_is_notify_req = vendor_rf_link_is_notify_req;
	p_vendor_rf_link_is_notify_rsp = vendor_rf_link_is_notify_rsp;
	p_vendor_mesh_op_is_conflict   = vendor_mesh_op_is_conflict;
}

// online status report
void	register_mesh_node_report_status_callback (void *p)
{
	//p point to: (pkt_light_report.value + 10)
	p_vendor_mesh_node_status_report = (cb_mesh_node_report_status_t) p;
}

void register_slave_read_status_stop_callback (void *p)
{
	p_vendor_mesh_node_read_stop = (cb_mesh_node_read_stop) p;
}

void register_slave_read_status_start_callback (void *p)
{
	p_vendor_mesh_node_read_start = (cb_mesh_node_read_stop) p;
}

void register_mesh_node_rcv_rsp_callback (void *p)
{
	// p pointer to: rf_packet_att_value_t
	p_vendor_mesh_node_rcv_rsp = (cb_mesh_node_rcv_rsp) p;
}

void register_mesh_node_rcv_online_callback (void *p)
{
	//p point to mesh_node_st_t->val
	p_mesh_node_any_online_status = (cb_mesh_node_any_online_status) p;
}
//**************************end OPPLE callback********************************************/

