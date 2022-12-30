/********************************************************************************************************
 * @file     dual_mode_adapt.h 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
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


/*SIG MESH*/
#define MESH_ADV_TYPE_PRO	    (0x29)
#define MESH_ADV_TYPE_MESSAGE   (0x2A)
#define MESH_ADV_TYPE_BEACON 	(0x2B)

typedef enum{
	UNPROVISION_BEACON = 0,
	SECURE_BEACON ,
}BEACON_ADV_TYPE;

typedef struct {
	u8 flag_len;
	u8 flag_type;
	u8 flag_content;
	u8 uuid_len;
	u8 uuid_type;
	u8 uuid_pb_uuid[2];
	u8 service_len;
	u8 service_type;
	u8 service_uuid[2];
	u8 service_data[16];
	u8 oob_info[2];
}PB_GATT_ADV_DAT;

typedef enum{
	TRANS_START = 	0,
	TRANS_ACK ,
	TRANS_CON ,
	BEARS_CTL ,
}TRANS_BEARER_TYPE;

typedef enum{
	LINK_OPEN =0,
	LINK_ACK,
	LINK_CLOSE,
	LINK_RFU,
}TRANS_LINK_OPERATION;

typedef struct {
	u8 length;
	u8 ad_type;
	u8 link_ID[4];
	u8 trans_num;
	u8 GPCF         :2;
	u8 BearCtl      :6;
	u8 DeviceUuid[16];
}pro_PB_ADV;

typedef struct {
	u8 len;
	u8 type;
	u8 beacon_type;
	u8 device_uuid[16];
	u16 oob_info;
	u8 uri_hash[4];
}beacon_data_pk;

//------------ mesh config (user can config)-------------
#define MESH_NORMAL_MODE		0
#define MESH_CLOUD_ENABLE		1
#define MESH_SPIRIT_ENABLE		2// use this mode should burn in the para in 0x78000,or use init para should enable the  con_sec_data
#define MESH_AES_ENABLE 		3
#define MESH_GN_ENABLE 		    4
#define MESH_MI_ENABLE          5

#define MESH_USER_DEFINE_MODE 	MESH_NORMAL_MODE


#if (MESH_USER_DEFINE_MODE == MESH_SPIRIT_ENABLE)
#define PROVISION_FLOW_SIMPLE_EN    1
#elif(MESH_USER_DEFINE_MODE == MESH_CLOUD_ENABLE)
#define PROVISION_FLOW_SIMPLE_EN    1
#elif(MESH_USER_DEFINE_MODE == MESH_GN_ENABLE)
#define PROVISION_FLOW_SIMPLE_EN    1
#elif(MESH_USER_DEFINE_MODE == MESH_MI_ENABLE)
#define PROVISION_FLOW_SIMPLE_EN    0
#elif(MESH_USER_DEFINE_MODE == MESH_NORMAL_MODE || MESH_USER_DEFINE_MODE == MESH_AES_ENABLE )
#define PROVISION_FLOW_SIMPLE_EN    0
#endif

/* end */

enum{
	RF_MODE_TLK_MESH		    = 0,
	RF_MODE_SIG_MESH,
};

enum{
    DUAL_MODE_SAVE_ENABLE       = 0x5A,    // dual mode state should be define both 73000 and 76080
    DUAL_MODE_SAVE_DISABLE      = 0x00,
    // all other is disable exclude 0xff
};

enum{
    DUAL_MODE_NOT_SUPPORT       = 0x00,
    DUAL_MODE_SUPPORT_ENABLE    = 0x01, // must 0xff
    DUAL_MODE_SUPPORT_DISABLE   = 0x02,
};

extern u8 dual_mode_state;

void dual_mode_sig_mesh_par_init();
u8 * get_sig_mesh_adv();
void tlk_mesh_access_code_backup(u32 ac);
int is_telink_mesh_found();
int	pb_gatt_Write (void *p);
int dual_mode_rx_sig_beacon_proc(u8 *p, u32 t);
void user_prov_multi_device_uuid(u8 *uuid);

void dual_mode_loop_proc();
void dual_mode_en_init();
void dual_mode_select(u32 sdk_rf_mode);
void dual_mode_disable();
void dual_mode_check_and_select_disconnect_cb();
void dual_mode_channel_ac_proc(int connect_st);
void dual_mode_channel_ac_set_with_check_TLK();
void dual_mode_channel_ac_set_with_check_SIG();
void mesh_service_change_report();
u8 get_gatt_adv_cnt();

// ATT
extern const u8 my_pb_gattUUID[2];
extern const u8 my_pb_gatt_out_UUID[2];
extern const u8 my_pb_gatt_out_prop;
extern const u8 my_pb_gattOutName[10];
extern u8 my_pb_gattOutData[2];
extern const u8 my_pb_gatt_in_UUID[2];
extern const u8 my_pb_gatt_in_prop;
extern const u8 my_pb_gattInName[9];
extern u8 my_pb_gattInData[2];

extern u8 provision_In_ccc[2];
extern u8 provision_Out_ccc[2]; 

extern const u16  mi_gerneric_service;
extern const u16 mi_service_change_uuid;
extern const u8 mi_service_change_prop;
extern u8 mi_service_change_buf[4];
extern const u8 mi_service_change_str[15];

extern u8 dual_mode_telink_adv_provisoning_flag;
void dual_mode_set_adv_provisoning_flag();

