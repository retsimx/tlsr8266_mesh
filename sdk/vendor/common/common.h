/********************************************************************************************************
 * @file     common.h 
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

#pragma once

#include "../../proj/tl_common.h"
#include "../../proj_lib/ble_ll/blueLight.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "../../proj_lib/light_ll/light_ll.h"
#include "../../proj/drivers/i2c.h"
#include "../../vendor/common/crc.h"
#include "dual_mode_adapt.h"
#include "battery_check.h"

void light_onoff_step(u8 on);
void light_onoff_step_timer();
void light_step_correct_mod(float *pwm_val, u16 lum);
void light_onoff(u8 on);
void light_onoff_step_init();
u8 is_lum_invalid(u8 lum);

void pa_init(u8 tx_pin_level, u8 rx_pin_level);
void pa_txrx(u8 val);
void light_slave_tx_command_callback (u8 *p);
void rf_link_slave_connect_callback();
void update_ble_parameter_cb();
int light_slave_tx_command(u8 *p_cmd, int para);
int light_slave_tx_command_sub_addr(u8 *p_cmd, int para, u8 sub_addr);
int factory_reset();
void mesh_send_command (u8 *p, u32 chn_idx, u32 retransmit_cnt);
int pair_enc_packet_mesh (u8 *ps);

/////////////// adv par define ///////////////////////////////////////////////////
extern u16 adv_interval2listen_interval;
extern u16 online_status_interval2listen_interval;
extern u8  separate_ADVpkt;
extern u8  mesh_chn_amount;
/////////////// mesh node define ////////////////////////////////////////////////////

#if (__PROJECT_LIGHT_SWITCH__ || __PROJECT_LPN__)
#define				MESH_NODE_MAX_NUM		32      // switch use minimum value
#else
#define				MESH_NODE_MAX_NUM		64      // must multy of 32 and max 256
#endif

/*
Note: ONLINE_STATUS_TIMEOUT may be set longer after set MESH_NODE_MAX_NUM or MESH_NODE_ST_VAL_LEN larger, 
*/
#define				MESH_NODE_ST_VAL_LEN    4       // MIN: 4,   MAX: 10
#define				MESH_NODE_ST_PAR_LEN    (MESH_NODE_ST_VAL_LEN - 2)  //  //lumen-rsv, exclude dev_adr and sn.

typedef struct{
    u8 dev_adr;     // don't change include type
    u8 sn;          // don't change include type
    u8 par[MESH_NODE_ST_PAR_LEN];  //lumen-rsv,
}mesh_node_st_val_t;

typedef struct{
    u16 tick;       // don't change include type
    mesh_node_st_val_t val;
	#if MESH_RSSI_RECORD_EN
    u8 rssi;        // must after "val", and don't use in library
	#endif
	#if NOTIFY_MESH_COMMAND_TO_MASTER_EN
	u8 manual_flag  :1;    // insert node flag
	u8 alive_flag   :1;
	u8 rfu          :6;
	#endif
}mesh_node_st_t;

typedef struct{
    u8 adr[1];      // don't modify, use internal
    u8 alarm_id;    // don't modify, use internal
}status_record_t;

extern status_record_t slave_status_record[];
extern u16 slave_status_record_size;

extern u8 mesh_node_st_val_len;
extern u8 mesh_node_st_par_len;
extern u8 mesh_node_st_len;
extern u16 mesh_node_max_num;
extern u8 mesh_node_max;    // count of current mesh
extern u8 light_rcv_rssi;
extern u8 flash_protect_en;

#define START_UP_FLAG		(0x544c4e4b)
#define CHANEL_ALL          (0xFF)

typedef struct{
	u8 op;
	u8 sno[3];
	u8 notify_ok_flag;
	u8 sno2[2];   // for passive
}rc_pkt_buf_t;

extern u8 mesh_cmd_cache_num;

void mesh_node_buf_init ();
void device_status_update();

extern u8   SW_Low_Power;
extern u8   SW_Low_Power_rsp_flag;
extern u8	mesh_ota_only_calibrate_type1;

extern void get_flash_user_data();
extern void check_store_user_data();
extern u8 flash_user_data[16];
extern u8 user_data_idx;

////////////////// gate way /////////////////////////////////////
void gateway_init();
int gatway_tx_command(u8 cmd, u16 dst_adr, u8 *par, u8 par_len, u8 op);
int gateway_mode_onoff(u8 on);
void proc_ui_gateway ();
void gateway_set_login_flag();
void ble_event_callback (u8 status, u8 *p, u8 rssi);
void proc_host ();
int host_write_master (int n);
void host_init ();

extern u8   gateway_en;
extern u8   gateway_security;
extern u8   mode_master;

////////////////// check stack /////////////////////////////////////
void stack_check_init();
void stack_check();
////////////////// ALARM and SCENE /////////////////////////////////////
int is_bridge_task_busy();
extern u8 	mesh_user_cmd_idx;
int is_tx_cmd_busy();

////////////////// OTA  /////////////////////////////////////
u32 get_ota_erase_sectors();
void erase_ota_data(u32 adr);
void erase_ota_new_fw_area();
int is_ota_area_valid(u32 adr);
void erase_ota_data_handle();
void set_firmware_type_init();
void set_firmware_type_TLK_mesh();

extern u32  ota_program_offset;

extern u8 iBeaconInterval;
extern u8 iBeaconData[];
extern u8 eddystone_uid[];
extern u8 eddystone_url[];
extern u8 eddystone_tlm[];
extern u8 beacon_len;
extern u8 beacon_with_mesh_adv;// 0 means only send beacon adv pkt;  1 means send both of beacon pkt and mesh adv pkt
extern void set_ibeacon_data(u8 *val, int n);

/////////////// encode / decode password  //////////////
void encode_password(unsigned char *pd);
void decode_password(unsigned char *pd);
extern u8   pair_config_pwd_encode_enable;
extern u8	auth_code_en;

extern u8 adv_uuid_flag;
extern u8 adv_uuid[4];

// recover status before software reboot
enum{
	FLD_LIGHT_OFF				= BIT(0),
	FLD_MESH_OTA_MASTER_100		= BIT(1),
	LOW_BATT_FLG                = BIT(2),
	// LOW_BATT_LOOP_FLG           = BIT(3),    // 0 means check by user_init, 1 means by main loop
};

#if((__TL_LIB_8258__ || (MCU_CORE_TYPE == MCU_CORE_8258)) || (__TL_LIB_8278__ || (MCU_CORE_TYPE == MCU_CORE_8278)))
#define		rega_light_off              DEEP_ANA_REG0   // only light project use
#elif(__TL_LIB_8266__ || (MCU_CORE_TYPE == MCU_CORE_8266))
#define		rega_light_off              0x3a            // only light project use, 0x34-0x39 will be clear by ota_boot.bin
#else
#define		rega_light_off              0x34            // only light project use
#endif

void light_sw_reboot_callback(void);
void light_sw_reboot_with_retention();
void mesh_ota_master_100_flag_check();

/////////////////// mesh_node_filter  //////////////////////
typedef int (*cb_mesh_node_filter_t) (u8 device_adr, u8 *p);
extern cb_mesh_node_filter_t	cb_mesh_node_filter;

/////////////////// call back function  //////////////////////
typedef void (*cb_func_void_t) ();
extern cb_func_void_t	p_cb_ble_slave_disconnect;
extern cb_func_void_t	p_cb_pair_failed;

/////////////////// passive device  //////////////////////
int get_command_type(u8 *p_att_value);
void set_command_type2alt(u8 *p_att_value);
extern u8 passive_en;

enum {
	CMD_TYPE_NORMAL				= 0,
	CMD_TYPE_PASSIVE_DEV		= 1,
	CMD_TYPE_PASSIVE_DEV_ALT	= 2,
};

/////////////////// mesh ota master  //////////////////////
void mesh_ota_master_start_firmware_by_gateway(u16 dev_mode);
void mesh_ota_master_start_firmware_from_own();
void mesh_ota_led_cb(u32 type);
void mesh_ota_start_unprotect_flash();
u32 get_fw_len(u32 fw_adr);
u8 get_ota_check_type(u8 *par);
void rf_slave_ota_finished_flag_set(u8 reset_flag);

extern u8	rf_slave_ota_busy_mesh_en;
///////////////////////////////////////////////////
int user_cmd2app_cmd(u8 cmd, u16 dst_adr, u8 *par, u8 par_len);
void light_rx_from_mesh_cb(u8 *p);
typedef void (*cb_rx_from_mesh_t) (u8 *p);
void register_cb_rx_from_mesh (cb_rx_from_mesh_t p);
void mesh_listen_chn_restore2def();

extern u8 tx_packet_bridge_random_en;
extern cb_rx_from_mesh_t p_cb_rx_from_mesh;

/////////////////// set and get mac by mesh  //////////////////////
int dev_addr_with_mac_flag(u8 *params);
int dev_addr_with_mac_match(u8 *params);
int dev_addr_with_mac_rsp(u8 *params, u8 *par_rsp);
int is_default_mesh();
void pair_save_key();
void pair_update_key ();
void set_mesh_provision_info(bool save_flag, u8 *name, u8 *pw, u8 *ltk);

extern u8* 	slave_p_mac;
extern u8 get_mac_en;
extern u8 mesh_pair_enable;
extern u8 pair_nn[];
extern u8 pair_pass[];
extern u8 pair_ltk[];
extern u32 pair_ac;

extern void mesh_pair_proc(void);
u8 mesh_pair_notify_refresh(rf_packet_att_cmd_t *p);
enum{
    PAIR_SETTED = 0,
    PAIR_SETTING,
    PAIR_SET_MESH_TX_START,
    PAIR_SET_MESH_TX_DONE,// send notify req, get mesh nodes' ac
    PAIR_SET_MESH_RX_DONE,// received all mesh nodes' ac, send cmd to switch to new mesh
};
#if(MESH_PAIR_ENABLE)
//unit: ms
#define MESH_PAIR_CMD_INTERVAL          500
extern u32 mesh_pair_cmd_interval;
//unit: s
#define MESH_PAIR_TIMEOUT               10
//unit: ms
#define MESH_PAIR_NOTIFY_TIMEOUT        2500
extern u32 mesh_pair_timeout;
extern u32 mesh_pair_start_time;
extern u8 new_mesh_name[];
extern u8 new_mesh_pwd[];
extern u8 new_mesh_ltk[];
extern u32 effect_new_mesh_delay_time;
extern u8 effect_new_mesh;
extern u8 pair_setting_flag;
extern u8 get_mesh_pair_checksum(u8 idx);
extern void save_effect_new_mesh(void);
extern void mesh_pair_cb(u8 *params);
extern void pair_load_key ();
extern u8 mesh_pair_checksum[];

enum{
    MESH_PAIR_NAME1 = 0,
    MESH_PAIR_NAME2,
    MESH_PAIR_PWD1,
    MESH_PAIR_PWD2,
    MESH_PAIR_LTK1,
    MESH_PAIR_LTK2,
    MESH_PAIR_EFFECT_DELAY,
    MESH_PAIR_EFFECT,
    MESH_PAIR_DEFAULT_MESH,
};
enum{
    USER_ALL_GET_MAC = 1,
    USER_ALL_GET_PAIR_CONFIRM,
};

#endif

#if(IBEACON_ENABLE)
//apple ibeacon
typedef struct{
    u8 flag_len; //constant, 0x02
    u8 flag_type;//constant, 0x01, flags type
    u8 flag; //constant, 0x06, flags indicating LE General discoverable and not supporting BR/EDR
    u8 ibeacon_len;//constant, 0x1A
    u8 manu_spec; //constant, 0xFF
    u16 company_ID;//constant,0X004C, Apple Inc
    u16 beacon_type; //constant,0X0215
    u8 proximity_UUID[16];//Defined by customer
    u16 major;//Defined by Customer
    u16 minor;//Defined by Customer
    s8 measured_power;//Defined by Customer, TX Power-41
}ibeacon_adv_t;
extern ibeacon_adv_t ibeacon_customer_data;
#endif

#if PROVISIONING_ENABLE
typedef enum
{
    GATEWAY_STATUS_NORMAL = 0,             /* Normal gateway role */
    GATEWAY_STATUS_NODE_ROLE,              /* As node role, when pushed button */
    
    GATEWAY_STATUS_TEMP_DEFALT_MESH,       /* In default mesh temporary */
    GATEWAY_STATUS_SWITCH_TO_DEFAULT_MESH,
    GATEWAY_STATUS_SCAN_UNPROV_DEV,        /* Scanning unpair device status */
    GATEWAY_STATUS_CFG_UNPRO_DEV,          /* Only provision device */
    GATEWAY_STATUS_CFG_CUR_NETWORK,        /* Change current network's information */
    GATEWAY_STATUS_ADD_CONFIGURED_DEVICE,  /* Add configured device */
}gateway_status_t;
#endif

extern unsigned char adc_temp_proc(void);
extern void adc_adj_tp_handle(void);
extern void adc_adj_tp_init(void);
extern void rssi_online_status_pkt_cb(mesh_node_st_t *p_node_st, u8 rssi, int online_again);
void rf_link_slave_ota_finish_led_and_reboot(int st);
int my_fifo_push_hci_tx(unsigned char *para, unsigned short len);
int hci_tx_fifo_poll(void);
void simple_sleep_duty_cycle();
void pwm_io_input_disable();

extern void mesh_pair_init(void);
extern void mesh_pair_proc_effect(void);
extern void mesh_pair_proc_get_mac_flag(void);


extern u16		cmd_delay_ms;
extern u16		cmd_left_delay_ms;
extern u32 		irq_timer1_cb_time;
extern int		slave_link_state;
extern u8       security_enable;
extern u8       pkt_need_relay;
extern u8	    slave_sno[4];
extern u8	    slave_link_cmd;
extern u32 		rcv_pkt_time;
extern rf_packet_att_cmd_t	pkt_light_status;
extern u8       ble_conn_terminate;
extern const u8 pkt_terminate[];

u16 light_cmd_delayed_ms(u8 data);
int is_mesh_cmd_need_delay(u8 *p_cmd, u8 *params, u8 ttc);

#define UPDATE_CONN_PARA_CNT  4
extern u8 conn_update_cnt;
extern u8 conn_update_successed;
extern u8 rf_update_conn_para(u8 * p);

extern u8 my_rf_power_index;
extern u8 mesh_ota_third_fw_flag;
extern u16 device_address_mask; // use in library

static inline u16 get_u16_by_pointer(u8 *addr)
{
    return (addr[0] + (addr[1]<<8));
}

static inline u16 get_addr_by_pointer(u8 *addr)
{
    return get_u16_by_pointer(addr);
}

#if SUB_ADDR_EN   //-- just for test
#define TEST_LED_CNT            (3)

typedef	struct {
	u16 dev_id      :8;
	u16 sub_addr    :4; // not mask
	u16 rsv         :3;
	u16 group_flag  :1; // must at bit 15
}device_addr_sub_t;    // don't use in library

static inline void set_device_addr_mask()
{
    u16 addr = 0x7fff;
    device_addr_sub_t *p_dev = (device_addr_sub_t *)&addr;
    device_address_mask = p_dev->dev_id;    // get mask from "device_addr_sub_t"
}

static inline void set_sub_addr2rsp(device_addr_sub_t *p_src, u8 *p_dst, bool dst_unicast)
{
    u16 addr_dst_rx = p_dst[0] + (p_dst[1] << 8);
	if(dst_unicast && (addr_dst_rx != device_address)){
	    device_addr_sub_t *p_r = (device_addr_sub_t *)&addr_dst_rx;
	    p_src->sub_addr = p_r->sub_addr;
	}
}
#else
#define set_sub_addr2rsp(p_src, p_dst, dst_unicast)    
#endif

void light_multy_onoff(u8 *dst_addr, u8 on);
u8 get_sub_addr_onoff();
void light_adjust_R(u16 val, u16 lum);
void light_adjust_G(u16 val, u16 lum);
void light_adjust_B(u16 val, u16 lum);

void cb_set_sub_addr_tx_cmd(u8 *src, u16 sub_adr);
#if AUTO_ADAPT_MAC_ADDR_TO_FLASH_TYPE_EN
void blc_readFlashSize_autoConfigCustomFlashSector(void);
#else
#define blc_readFlashSize_autoConfigCustomFlashSector()     // null
#endif

#if NOTIFY_MESH_FIFO_EN
typedef	struct {
	u8  sno[3];
	u16 src_adr;
	#if NOTIFY_MESH_COMMAND_TO_MASTER_EN
	u16 dst_adr;
	u8  op;
	u16 vendor_id;
	#endif
	u8  data[10];
}notify_mesh_data_t;    // rf_packet_att_value_t
extern my_fifo_t notify_mesh_fifo;
int my_fifo_push_mesh_notify(rf_packet_att_value_t *pp);
void notify_mesh_fifo_proc ();
int light_notify(u8 *p, u8 len, u8* p_src);
#endif
int notify_mesh_command2_master();
void rf_link_data_callback_user_cmd (u8 *p, u8 op);
void rx_mesh_adv_message_cb(u8 *p, int mac_match);
void nctm_rx_gatt_message_cb(u8 *p);
void nctm_loop();
void nctm_user_init();
void mesh_node_keep_alive_other ();
void forced_single_cmd_in_ble_interval_handle(u8 *ph);

void mesh_ota_third_complete_cb(int calibrate_flag);
int uart_add_escape(u8 *data_in, u16 len_in, u8 *data_out, u16 len_out_max);
int	rf_link_slave_data_ota_save();
void rf_led_ota_error(void);
void rf_led_ota_ok(void);

// sensor 
void sensor_enter_deep_cb(void);
extern u8 sensor_enable;
extern u16 sensor_last_adv_sleep_time_us;

// ----- wakeup
enum{
    FLAG_WAKEUP_NONE    = 0,
    FLAG_WAKEUP_TIMER   = BIT(1),
    FLAG_WAKEUP_GPIO    = BIT(2),
};

/*
return: FLAG_WAKEUP_NONE, FLAG_WAKEUP_TIMER, FLAG_WAKEUP_GPIO
*/
int get_wakeup_src_flag();

// ------ friend ship
typedef struct{
    u8 sub_cmd      :4;
    u8 par_type     :4;
    union{
        u8 sub_par[9];
        struct{
            u32 PollTimeout     :24;    // unit: 100ms
            u32 chn_idx_req     :8;
            u16 PollInterval;           // unit: 100ms
        };  // lpn request
        struct{
            u8 FSN          :1;
            u8 rsv1         :3;
            u8 chn_idx      :2;
            u8 rsv2         :2;
            union{
                u16 group[4];
                u8 online_st_par[MESH_NODE_ST_PAR_LEN];
            };
        };  // lpn poll
        struct{
            u8 updata_poll_type;// encryption before rx POLL, should be beeter.comfirm later
        };  // update
    };
}fs_tx_cmd_par_t;

#define ENCRYPTION_TIME_US          (400)
#define TX_ONE_RF_US                (570)
#define TX_ONE_RF_CYCLE_US          (TX_ONE_RF_US + 20)

// FN
#define FN_CACHE_LPN_MAX        (2)
//#define FN_CACHE_MESSAGE_CNT    (2)
#define FN2LPN_RETRANSMIT_CNT   (3)     // total count = n+1;  should be clear after tx completed.

#define FN_POLL_2_OFFER_TIMEOUT_MS  (5000)

enum{
	FLD_HANDLE1_FN_FLAG =		BIT(1), // beacuse 0xf5 have been use before
};

enum{
    ST_FN_IDLE          = 0,
    //ST_FN_REQUEST,
    ST_FN_OFFER,
    //ST_FN_POLL,
    //ST_FN_UPDATE,
    ST_FN_LINK_OK,      // same with ST_FN_TIMEOUT_CHECK
};

enum{
    ST_CACHE_NO_DATA    = 0,
    ST_CACHE_TX2LPN,
    ST_CACHE_RX,
};

#if FEATURE_FRIEND_EN
typedef struct{
	u32 PollTimeout;        // unit: 100ms, 0x0A~0x34BBFF
	u32 tick_st;
    u16 lpn_adr;
    u16 group_list[MAX_GROUP_NUM];
    u8 FSN;
    u8 poll_type;
    u8 chn_idx;
    u8 online_st_sn;
    u8 st;
    u8 cache_flag;          // 1 meas there is cache data
    mesh_pkt_t pkt_cache;   // must u32 aligned, size is 44
}fs_proc_fn_t;
#endif

int fn_rx_friendship_cmd_proc(mesh_pkt_t *pkt);
void fn_rx_push_to_cache(u8 *p);
void friendship_proc_fn();

// LPN
#define LPN_REQUEST_INTERVAL_MS     (2000)
#define LPN_POLL_INTERVAL_MS        (2000)
#define LPN_POLL_TIMEOUT_100MS      (80*10)
#define LPN_POLL_RETRY_MAX          (20)
#define LPN_POLL_RETRY_DELAY_MS     (100)

#define LPN_REQUEST_REC_WIN_MS      (30)

#define RETRANSMIT_DELAY_US         (200)   // don't modify, use in library;    // wait for lpn irq rx handle.
#define LPN_POLL_REC_WIN_MS         ((4 * (TX_ONE_RF_CYCLE_US+RETRANSMIT_DELAY_US))/1000 + 1) // only receive channel; 4: UPDATE_CONN_PARA_CNT;

enum{
    ST_LPN_IDLE         = 0,
    ST_LPN_REQUEST,
    ST_LPN_OFFER,
    ST_LPN_POLL,
    ST_LPN_UPDATE,
};

enum{
    POLL_TYPE_ONLINE_ST      = 0,
    POLL_TYPE_GROUP_OW,         // 2 byte group, max 4
    POLL_TYPE_GROUP_ADD,        // 2 byte group, max 4
    POLL_TYPE_GROUP_DEL,        // 2 byte group, max 4
    POLL_TYPE_GROUP_OW_1BYTE,   // 1 byte group, max 8
};

typedef struct{
    u32 tick_st;
    u16 fn_adr;
    u8 rx_offer_flag;
    u8 rx_update_flag;
    u8 FSN;
    u8 poll_report_online_st_now;
    u8 poll_type;
    u8 poll_type_tx_flag;
    u8 poll_cnt;
    u8 st;
}fs_proc_lpn_t;

void friendship_tx_cmd_lpn(u8 sub_cmd, u16 adr_dst);
int lpn_rx_friendship_cmd_proc(mesh_pkt_t *pkt);
int friendship_proc_lpn();
void lpn_proc_st_init();
u8 get_proc_st_lpn();
void set_proc_st_lpn(u8 st);
int is_led_busy();
void friend_ship_establish_ok_cb_lpn();
void friend_ship_disconnect_cb_lpn();
void get_online_st_par(u8 *p_st_out);
u8 is_wakeup_by_gpio();
int is_valid_fw_len(u32 fw_len);

extern u32 st_listen_no;
extern fs_proc_lpn_t fs_proc_lpn;
extern u8 led_onoff_st;
extern u8 key_wakeup_flag;
extern u8 slave_link_connected;
extern u32 ota_firmware_size_k;

#if 1 // debug for LA
#if LPN_DEBUG_PIN_EN
static const u32 lpn_debug_pin[] = {
    GPIO_PB2, GPIO_PB3, GPIO_PC2, GPIO_PC3, GPIO_PC6, GPIO_PC7
};

static inline void lpn_debug_gpio_init()
{
    foreach_arr(i,lpn_debug_pin){
        gpio_set_func(lpn_debug_pin[i], AS_GPIO);
        gpio_set_output_en(lpn_debug_pin[i], 1);
        gpio_write(lpn_debug_pin[i], 0);
    }
}

#define lpn_debug_set_TX_pin(level)         do{gpio_write(lpn_debug_pin[0], level);}while(0)
#define lpn_debug_set_RX_pin(level)         do{gpio_write(lpn_debug_pin[1], level);}while(0)
#define lpn_debug_set_current_pin(level)    do{gpio_write(lpn_debug_pin[2], level);}while(0)
#define lpn_debug_set_irq_pin(level)        do{gpio_write(lpn_debug_pin[3], level);}while(0)
#define lpn_debug_set_test_pin(level)       do{gpio_write(lpn_debug_pin[4], level);}while(0)
#define lpn_debug_set_debug_pin(level)      do{gpio_write(lpn_debug_pin[5], level);}while(0)
#else
#define lpn_debug_set_current_pin(level)  
#define lpn_debug_set_TX_pin(level)   
#define lpn_debug_set_RX_pin(level)   
#define lpn_debug_set_irq_pin(level)    
#define lpn_debug_set_debug_pin(level)  

#define lpn_debug_gpio_init()   
#endif

#if FN_DEBUG_PIN_EN
static const u32 fn_debug_pin[] = {
#if((__TL_LIB_8258__ || (MCU_CORE_TYPE == MCU_CORE_8258)) || (__TL_LIB_8278__ || (MCU_CORE_TYPE == MCU_CORE_8278)))
    GPIO_PB2, GPIO_PB3, GPIO_PC2, GPIO_PC3, GPIO_PC6, GPIO_PC7
#else   // 8267 / 8269
    GPIO_PA0, GPIO_PA1, GPIO_PA4, GPIO_PA7, GPIO_PE1, GPIO_PA3
#endif
};

static inline void fn_debug_gpio_init()
{
    foreach_arr(i,fn_debug_pin){
        gpio_set_func(fn_debug_pin[i], AS_GPIO);
        gpio_set_output_en(fn_debug_pin[i], 1);
        gpio_write(fn_debug_pin[i], 0);
    }
}

#define fn_debug_set_TX_pin(level)          do{gpio_write(fn_debug_pin[0], level);}while(0)
#define fn_debug_set_RX_pin(level)          do{gpio_write(fn_debug_pin[1], level);}while(0)
#define fn_debug_set_current_pin(level)     do{gpio_write(fn_debug_pin[2], level);}while(0)
#define fn_debug_set_irq_pin(level)         do{gpio_write(fn_debug_pin[3], level);}while(0)
#define fn_debug_set_debug_pin(level)       do{gpio_write(fn_debug_pin[4], level);}while(0)
#else
#define fn_debug_set_TX_pin(level)   
#define fn_debug_set_RX_pin(level)   
#define fn_debug_set_current_pin(level)  
#define fn_debug_set_irq_pin(level)    
#define fn_debug_set_debug_pin(level)  

#define fn_debug_gpio_init()   
#endif
#endif

