/********************************************************************************************************
 * @file     dual_mode_adapt.c 
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
#include "../../proj/tl_common.h"
#if !WIN32
#include "../../proj/mcu/watchdog_i.h"
#endif 
#include "../../proj_lib/ble_ll/gatt.h"
#include "../../proj_lib/ble_ll/service.h"
#include "../../proj_lib/ble_ll/mesh_md5.h"

#include "common.h"

#if DUAL_MODE_ADAPT_EN
// ---------- parameter define
#define DUAL_MODE_UNPROV_USE_SAME_CHANNEL_EN    (0)   // use the same channel as SIG for unprovisoion telink node.

#if (0 == DUAL_MODE_UNPROV_USE_SAME_CHANNEL_EN)
#define DUAL_MODE_INTERVAL_TLK_MS   (3000)
#define DUAL_MODE_INTERVAL_SIG_MS   (3000)
#define DUAL_MODE_CNT_TLK           (DUAL_MODE_INTERVAL_TLK_MS / 45)
#define DUAL_MODE_CNT_SIG           (DUAL_MODE_INTERVAL_SIG_MS / 45)
#define DUAL_MODE_TLK_CYCLE         (DUAL_MODE_CNT_TLK + DUAL_MODE_CNT_SIG)
#endif

#define UNPROV_BEACON_INV_US    (160*1000)

// ---------- function
u8 dual_mode_state = DUAL_MODE_NOT_SUPPORT;
//u8 rf_mode_rx = RF_MODE_TLK_MESH;   // only for rx mesh channel, gatt always received.

u8 dual_mode_telink_adv_provisoning_flag = 0;
u32 dual_mode_select_sig_tick = 0;
u32 dual_mode_tlk_ac;

rf_packet_adv_ind_module_t	pkt_adv_sig = {0};
rf_packet_adv_ind_module_t	pkt_adv_unprov = {0};

void sig_mesh_adv_init(u8 *payload, u32 len)
{
    if(len > sizeof(pkt_adv_sig.data)){
        return ;
    }
    
    rf_packet_adv_ind_module_t	*p = &pkt_adv_sig;
    p->type = FLG_BLE_LIGHT_ADV;
	memcpy (p->advA, slave_p_mac, sizeof(p->advA));
	memcpy (p->data, payload, len);
    p->rf_len = len + 6;
    p->dma_len = len + 8;
}

void dual_mode_sig_mesh_par_init()
{
    u8 payload[sizeof(PB_GATT_ADV_DAT)] = {
        0x02,0x01,0x06,
        0x03,0x03,0x27,0x18,
        0x15,0x16,0x27,0x18,
    };
    
    // ---- GATT adv
    PB_GATT_ADV_DAT *p = (PB_GATT_ADV_DAT *)payload;
    u8 uuid[16] = {'0','1','2','3','4','5','6','7','8','9'};
    memcpy(uuid + 10, slave_p_mac, 6);
    memcpy(p->service_data, uuid, sizeof(p->service_data));
    //p->oob_info = 0;
    
    sig_mesh_adv_init(payload, sizeof(payload));

    
    // ---- unprovision beacon, uuid must same with SIG MESH, if not, gateway provisioner will wait for 20s.
    pkt_adv_unprov.type = FLG_BLE_ADV_NONCONN_IND;
    memcpy(pkt_adv_unprov.advA, slave_p_mac, 6);
    
    beacon_data_pk *p_bc = (beacon_data_pk *)pkt_adv_unprov.data;
    p_bc->len = 20;
    p_bc->type = MESH_ADV_TYPE_BEACON ;
    p_bc->beacon_type = UNPROVISION_BEACON;
    if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){    // save time when dual mode disable.
        user_prov_multi_device_uuid(p_bc->device_uuid);
    }
    p_bc->oob_info = 0;

    pkt_adv_unprov.rf_len = p_bc->len + 1 + 6;
    pkt_adv_unprov.dma_len = pkt_adv_unprov.rf_len + 2;
}

void user_prov_multi_device_uuid(u8 *uuid)
{
    #if(MESH_USER_DEFINE_MODE == MESH_CLOUD_ENABLE || MESH_USER_DEFINE_MODE == MESH_SPIRIT_ENABLE)
    set_dev_uuid_for_sha256();
    #elif (MESH_USER_DEFINE_MODE == MESH_AES_ENABLE)
    uuid_create_by_mac(tbl_mac,uuid);
    #elif (MESH_USER_DEFINE_MODE == MESH_GN_ENABLE)
    set_dev_uuid_for_simple_flow(uuid);
    #elif (MESH_USER_DEFINE_MODE == MESH_NORMAL_MODE)
        #if PROVISION_FLOW_SIMPLE_EN
	set_dev_uuid_for_simple_flow(uuid);
	    #else
	uuid_create_by_mac(slave_p_mac,uuid);
		#endif
    #endif
}

// ---------------------dual mode switch check
const u8 MESH_CHN_LISTEN_SIG[4] = {38, 39, 37, 39};
const u32 MESH_AC_LISTEN_SIG = 0x8e89bed6;

extern u8 sys_chn_listen[4];

void mesh_listen_chn_set2SIG()
{
    memcpy(sys_chn_listen, MESH_CHN_LISTEN_SIG, sizeof(sys_chn_listen));
}

void tlk_mesh_access_code_backup(u32 ac)
{
    dual_mode_tlk_ac = ac;
}

void rf_channel_ac_set_TLK()
{
    pair_ac = dual_mode_tlk_ac;
    mesh_listen_chn_restore2def();
}

void rf_channel_ac_set_SIG()
{
    pair_ac = MESH_AC_LISTEN_SIG;
    mesh_listen_chn_set2SIG();
}

int is_rf_mode_sig()
{
    return ((MESH_AC_LISTEN_SIG == pair_ac)
          && !memcmp(sys_chn_listen, MESH_CHN_LISTEN_SIG, sizeof(sys_chn_listen)));
}

void dual_mode_channel_ac_set_with_check_TLK()
{
    if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
        rf_channel_ac_set_TLK();
    }
}

void dual_mode_channel_ac_set_with_check_SIG()
{
    if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
        rf_channel_ac_set_SIG();
    }
}

int is_telink_mesh_found()
{
	return dual_mode_telink_adv_provisoning_flag;
}

inline int is_sig_mesh_found()
{
    // should not be true here, because it has been reboot when selected sig.
	return 0;
}

void dual_mode_en_init()		// call in mesh_init_all();
{
	u8 en = 0;
	flash_read_page(CFG_ADR_DUAL_MODE_EN, 1, (u8 *)&en);
    //LOG_MSG_INFO(TL_LOG_NODE_SDK,0, 0,"dual mode enable flag 0x76080:0x%x",en);
	if(0xff == en){
	    en = DUAL_MODE_SAVE_ENABLE;
	    flash_write_page(CFG_ADR_DUAL_MODE_EN, 1, (u8 *)&en);
	}
	
	if(DUAL_MODE_SAVE_ENABLE == en){
		u32 startup_flag1 = 0;
		u32 startup_flag2 = 0;
		flash_read_page(0x00008, 4, (u8 *)&startup_flag1);
		flash_read_page(0x40008, 4, (u8 *)&startup_flag2);
		if((START_UP_FLAG == startup_flag1) && (START_UP_FLAG == startup_flag2)){
            u32 mesh_type = 0;
            flash_read_page(FLASH_ADR_MESH_TYPE_FLAG, sizeof(mesh_type), (u8 *)&mesh_type);
		    if(TYPE_DUAL_MODE_STANDBY == mesh_type){
			    dual_mode_state = DUAL_MODE_SUPPORT_ENABLE;
                //LOG_MSG_INFO(TL_LOG_NODE_SDK,0, 0,"Dual mode support enable",0);
		    }else{
			    dual_mode_state = DUAL_MODE_SUPPORT_DISABLE;
                //LOG_MSG_INFO(TL_LOG_NODE_SDK,0, 0,"Dual mode support disable",0);
			}
		}else{
            en = 0;
		}
	}

	if(en && (DUAL_MODE_SAVE_ENABLE != en)){
	    en = 0;
	    flash_write_page(CFG_ADR_DUAL_MODE_EN, 1, (u8 *)&en);
	}

	if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
		rf_link_light_event_callback(LGT_CMD_DUAL_MODE_MESH);
	}
}

void dual_mode_disable()
{
	if(DUAL_MODE_NOT_SUPPORT != dual_mode_state){
		dual_mode_state = DUAL_MODE_NOT_SUPPORT;
		u8 zero = 0;
		flash_write_page(CFG_ADR_DUAL_MODE_EN, 1, (u8 *)&zero);
        // erase_ota_data_handle(); // may disconnect
        //LOG_MSG_INFO(TL_LOG_NODE_SDK,0, 0,"Dual mode not support",0);
	}
}

/* 
call when set TLK mesh name OK in library.
*/
void dual_mode_select(u32 sdk_rf_mode)
{
	if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
		dual_mode_state = DUAL_MODE_SUPPORT_DISABLE;
		u8 zero = 0;
		if(sdk_rf_mode == RF_MODE_TLK_MESH){
            set_firmware_type_TLK_mesh();
            rf_channel_ac_set_TLK();
            //LOG_MSG_INFO(TL_LOG_NODE_SDK,0, 0,"Dual mode support disable: select BLE",0);
		}else if(sdk_rf_mode == RF_MODE_SIG_MESH){
			u32 adr_boot_disable = ota_program_offset ? 0 : 0x40000;
            flash_write_page(adr_boot_disable + 8, 1, (u8 *)&zero);
            light_sw_reboot();
		}
	}
}

void dual_mode_check_and_select_disconnect_cb() // call when GATT disconnect event
{
    if(dual_mode_select_sig_tick){
        dual_mode_select(RF_MODE_SIG_MESH); // will reboot here.
    }
}

void unprov_beacon_send() // no call by library
{
    //if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
        //if(is_rf_mode_sig()){   // sig mode
            u8 r = irq_disable();
            u8 mesh_chn_amount_backup = mesh_chn_amount;
            mesh_chn_amount = 3;
            mesh_send_command((u8 *)&pkt_adv_unprov, CHANEL_ALL, 0);
            mesh_chn_amount = mesh_chn_amount_backup;
            irq_restore(r);
        //}
    //}
}

int dual_mode_rx_sig_beacon_proc(u8 *p, u32 t)
{
    if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
        if(is_rf_mode_sig()){
            rf_packet_adv_ind_module_t *p_adv = (rf_packet_adv_ind_module_t *)p;
            if ((p_adv->type & 0x0f)  == FLG_BLE_ADV_NONCONN_IND){
                pro_PB_ADV *pro = (pro_PB_ADV *)p_adv->data;
                if((MESH_ADV_TYPE_PRO == pro->ad_type)&&((sizeof(pro_PB_ADV)-1) == pro->length)){
                    if((BEARS_CTL == pro->GPCF)&&(LINK_OPEN == pro->BearCtl)){
                        dual_mode_select(RF_MODE_SIG_MESH); // will reboot here.
                    }
                }
            }
            return 1;
        }
    }

    return 0;
}

void dual_mode_channel_ac_proc(int connect_st) // call once by library every 40ms in irq state.
{
    if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
        u8 r = irq_disable();
        #if DUAL_MODE_UNPROV_USE_SAME_CHANNEL_EN
        rf_channel_ac_set_SIG();
        #else
        if(is_telink_mesh_found()){
            rf_channel_ac_set_TLK();
        }else if(is_sig_mesh_found()){
            rf_channel_ac_set_SIG();    // should not happen here
        }else{
            static u16 dual_mode_channel_toggle_cnt;
            #if 0
            if(connect_st){    // no toggle when GATT connected.
                rf_channel_ac_set_TLK();
            }else
            #endif
            {
                dual_mode_channel_toggle_cnt++;
                if(dual_mode_channel_toggle_cnt <= DUAL_MODE_CNT_TLK){
                    rf_channel_ac_set_TLK();
                }else if(is_wait_for_tx_response()){  // telink mesh will tx response at soon.
                    rf_channel_ac_set_TLK();
                    dual_mode_channel_toggle_cnt--; // 
                }else{
                    rf_channel_ac_set_SIG();
                }
                
                if(dual_mode_channel_toggle_cnt >= DUAL_MODE_TLK_CYCLE){
                    dual_mode_channel_toggle_cnt = 0;
                }
            }

            if((0 == slave_link_connected) && is_rf_mode_sig()){
                static u32 tick_unprov_beacon;
                if(clock_time_exceed(tick_unprov_beacon, UNPROV_BEACON_INV_US)){
                    tick_unprov_beacon = clock_time();
                    unprov_beacon_send();
                }
            }
        }
        #endif
        irq_restore(r);
    }
}

void dual_mode_loop_proc()
{
	if(DUAL_MODE_SUPPORT_ENABLE != dual_mode_state){
		return ;
	}

	// make sure select ok.
    if(dual_mode_select_sig_tick && clock_time_exceed(dual_mode_select_sig_tick, 1000*1000)){
        dual_mode_select(RF_MODE_SIG_MESH); // will reboot here.
    }

    // 
}

// pb-gatt 
/************************* sig-mesh service uuid **************************/
#define SIG_MESH_PROVISION_SERVICE 			{0x27,0x18}
#define SIG_MESH_PROVISION_DATA_IN 			{0xdb,0x2a} //write without rsp 
#define SIG_MESH_PROVSIION_DATA_OUT			{0xdc,0x2a}//notify
#define MESH_PROVISON_DATA	                {0xce,0x7f}

// service
const u8 my_pb_gattUUID[2]=SIG_MESH_PROVISION_SERVICE;

// pb_gatt_out
const u8 my_pb_gatt_out_UUID[2]= SIG_MESH_PROVSIION_DATA_OUT;
const u8 my_pb_gatt_out_prop = CHAR_PROP_NOTIFY;
const u8 my_pb_gattOutName[10]={'P','B','G','A','T','T','-','O','U','T'};
u8 	my_pb_gattOutData[2] =MESH_PROVISON_DATA;

// pb_gatt_in   // for APP wirite
const u8 my_pb_gatt_in_UUID[2]= SIG_MESH_PROVISION_DATA_IN;
const u8 my_pb_gatt_in_prop =  CHAR_PROP_WRITE_WITHOUT_RSP;
const u8 my_pb_gattInName[9]={'P','B','G','A','T','T','-','I','N'};
u8 	my_pb_gattInData[2] =MESH_PROVISON_DATA;

// ccc
u8  provision_In_ccc[2]={0x01,0x00};// set it can work enable 
u8  provision_Out_ccc[2]={0x00,0x00}; 


int	pb_gatt_Write (void *p)
{
	//if(provision_In_ccc[0]==0 && provision_In_ccc[1]==0){
		//return 0;
	//}
	if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
    	rf_packet_att_write_t *pw = (rf_packet_att_write_t *)p;
    	u8 invite_cmd[3] = {0x03, 0x00, 0x00};
    	if((6 == pw->l2capLen) && (!memcmp(invite_cmd, &pw->value, sizeof(invite_cmd)))){
    	    dual_mode_select_sig_tick = clock_time()|1;
            rf_link_add_tx_packet ((u32)(&pkt_terminate));
    	}
	}
	
	return 1;
}

// service change UUID
const u16  mi_gerneric_service  = SERVICE_UUID_GENERIC_ATTRIBUTE;
const u16 mi_service_change_uuid = 0x2a05;
const u8 mi_service_change_prop = CHAR_PROP_INDICATE;
u8 mi_service_change_buf[4];
const u8 mi_service_change_str[15]="service change";

#define ATT_HANDLE_SERVICE_CHANGE_SIG       (0x29)

void mesh_service_change_report()
{
    // force to do the service changes 
    u8 service_data[4]={0x01,0x00,0xff,0x00};

    // dual_mode_TLK_service_change
    if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
        rf_packet_att_data_t pkt_srv = {13, 2, 11, 7, 4, ATT_OP_HANDLE_VALUE_IND};
        pkt_srv.hl = ATT_HANDLE_SERVICE_CHANGE_SIG; // hanle of service change in telink mesh SDK 
        if(sizeof(pkt_srv.dat) >= sizeof(service_data)){
            memcpy(pkt_srv.dat, service_data, sizeof(service_data));
        }
        rf_link_add_tx_packet((u32)(&pkt_srv));    // no need to check indication comfirm
    }
}


#else
void dual_mode_en_init(){}
void dual_mode_disable(){}
void dual_mode_select(u32 sdk_rf_mode){}
void dual_mode_check_and_select_disconnect_cb(){}
int dual_mode_rx_sig_beacon_proc(u8 *p, u32 t){return 0;}
void tlk_mesh_access_code_backup(u32 ac){}
void dual_mode_channel_ac_proc(int connect_st){}
void dual_mode_channel_ac_set_with_check_TLK(){}
void dual_mode_channel_ac_set_with_check_SIG(){}
#endif

#define GATT_ADV_CHN_CNT        (3)
u8 get_gatt_adv_cnt()
{
#if DUAL_MODE_ADAPT_EN
    if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
        return (GATT_ADV_CHN_CNT * 2);  // include telink + sig mesh adv.
    }
#endif

    return GATT_ADV_CHN_CNT;
}

u8 * get_sig_mesh_adv() // only call when dual mode enable
{
#if DUAL_MODE_ADAPT_EN
    if(DUAL_MODE_SUPPORT_ENABLE == dual_mode_state){
        return (u8 *)&pkt_adv_sig;
    }
#endif

    return 0;
}

void dual_mode_set_adv_provisoning_flag()
{
#if DUAL_MODE_ADAPT_EN
    dual_mode_telink_adv_provisoning_flag = 1;
#endif
}