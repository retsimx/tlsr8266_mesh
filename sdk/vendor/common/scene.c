/********************************************************************************************************
 * @file     scene.c 
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
#include "../../proj_lib/ble_ll/blueLight.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "common.h"
#include "scene.h"

#if(SCENE_EN)

FLASH_ADDRESS_EXTERN;

STATIC_ASSERT(sizeof(scene_t) + 1 <= 10);

extern u8 led_val[];
extern u8 led_lum;
extern u8 light_off;
extern void light_adjust_RGB(u8 val_R, u8 val_G, u8 val_B, u8 lum);
extern void device_status_update ();
scene_t scene_data[MAX_SCENE_CNT];
static u16 flash_scene_next_pos = 0;
// load info from flash
void scene_init(void){
    memset(scene_data, 0, sizeof(scene_data));
    
    u8 scene_cnt = 0;
    foreach(i, FLASH_SECTOR_SIZE / sizeof(scene_t)){
        flash_scene_next_pos = sizeof(scene_t)*i;
        u8 val = *(u8 *)(flash_adr_scene+flash_scene_next_pos);
        if(val == 0xFF){
            break;
        }else if(val == 0){
            continue;
        }else{
            memcpy(&scene_data[scene_cnt%MAX_SCENE_CNT], (void*)(flash_adr_scene+flash_scene_next_pos), sizeof(scene_t));
            scene_cnt++;
        }
    }
}

void scene_flash_clean(void){
    if(flash_scene_next_pos >= FLASH_SECTOR_SIZE - sizeof(scene_t)){
        flash_erase_sector(flash_adr_scene);
        flash_write_page(flash_adr_scene, sizeof(scene_data), (u8 *)(&scene_data[0]));
        flash_scene_next_pos = sizeof(scene_data);
    }    
}

u8 scene_find(u8 id){
    foreach(i, MAX_SCENE_CNT){
        if(id == scene_data[i].id){
            return i;
        }
    }

    return MAX_SCENE_CNT;
}

u8 scene_add(scene_t * data){
    u8 overwrite = 1;    
    static int oldest_pos = -1;

    if(data->id == 0){
        return 0;
    }

    scene_flash_clean();
    
    if(flash_scene_next_pos == 0){
        memcpy(&scene_data[0], data, sizeof(scene_t));
        overwrite = 0;
    }else{
        foreach(i, MAX_SCENE_CNT){
            if(data->id == scene_data[i].id){
                // exist
                if(scene_data[i].lum == data->lum
                    && scene_data[i].rgb[0] == data->rgb[0]
                    && scene_data[i].rgb[1] == data->rgb[1]
                    && scene_data[i].rgb[2] == data->rgb[2]
#if __CT__
                    && scene_data[i].ct == data->ct
#endif
                ){
                    //repetition
                    return 0;
                }
                scene_del(scene_data[i].id);
                memcpy(&scene_data[i], data, sizeof(scene_t));
                overwrite = 0;
                break;
            }else if(scene_data[i].id == 0){
                memcpy(&scene_data[i], data, sizeof(scene_t));
                overwrite = 0;
                break;
            }
        }
    }
    if(overwrite){
        if(oldest_pos == -1){
            oldest_pos = 0;
        }
        scene_del(scene_data[oldest_pos].id);
        memcpy(&scene_data[oldest_pos], data, sizeof(scene_t));
        oldest_pos = (oldest_pos + 1) % MAX_SCENE_CNT;
    }
    
    // add one scene
    flash_write_page(flash_adr_scene + flash_scene_next_pos, sizeof(scene_t), (u8 *)data);
    flash_scene_next_pos += sizeof(scene_t);

    return 1;
}

u8 scene_del(u8 id){
    u8 ret = 0;
    u8 update_flash = 0;
    if(flash_scene_next_pos == 0){
        return 0;
    }
    foreach(i, MAX_SCENE_CNT){
        if(id == 0xFF){//delete all
            memset(&scene_data[0], 0, sizeof(scene_data));
            update_flash = 1;
            break;
        }else if(scene_data[i].id == id){
            memset(&scene_data[i], 0, sizeof(scene_t));
            update_flash = 2;
            break;
        }
    }
    s16 i = flash_scene_next_pos - sizeof(scene_t);
    u8 scene_cnt = 0;
    scene_t del_val = {0};
    do{
        u8 val = *(u8 *)(flash_adr_scene + i);
        if(val){
            if(update_flash == 1 || (update_flash == 2 && val == id)){
                flash_write_page(flash_adr_scene + i, sizeof(scene_t), (u8 *)&del_val);
                ret = 1;
                if(update_flash == 2 || ++scene_cnt >= MAX_SCENE_CNT){
                    break;
                }
            }
        }
        if(i <= 0){
            break;
        }
        i -= sizeof(scene_t);
    }while(1);
    
    
    return ret;
}

u8 scene_load(u8 id){
    u8 idx = scene_find(id);
    if(idx < MAX_SCENE_CNT){
        if(scene_data[idx].lum == 0 || (scene_data[idx].rgb[0] == 0 && scene_data[idx].rgb[1] == 0 && scene_data[idx].rgb[2] == 0)){
            light_off = 1;
            light_adjust_RGB(0, 0, 0, 0);
        }else{
            light_off = 0;
            
            led_val[0] = scene_data[idx].rgb[0];
            led_val[1] = scene_data[idx].rgb[1];
            led_val[2] = scene_data[idx].rgb[2];
            u8 use_lum_min = is_lum_invalid(scene_data[idx].lum);
            led_lum = use_lum_min ? use_lum_min : scene_data[idx].lum;
            light_adjust_RGB(led_val[0], led_val[1], led_val[2], led_lum);
        }
        #if __CT__
        //set_ct(scene_data[idx].ct);
        #endif
        //lum_changed_time = clock_time(); // don't save scene state
        device_status_update();
    }

    return 0;
}

u8 pkt_mesh_scene_rsp[48];
extern u16 device_address;

extern u8 	mesh_user_cmd_idx;
extern rf_packet_att_cmd_t  pkt_light_status;
extern int	rf_link_slave_add_status (rf_packet_att_cmd_t *p);

static u8 scene_poll_notify_flag;
static u8 scene_poll_total;
static u8 scene_poll_notify_index;
static u8 scene_poll_notify_cnt;
static u32 scene_notify_time;
static u32 scene_notify_tick;
static u8 scene_need_bridge_flag;

int is_scene_poll_notify_busy(){
    return scene_poll_notify_flag;
}

int pair_enc_packet_mesh (u8 *ps);
void scene_rsp_mesh_enc(){
    pkt_light_status.type |= BIT(7);
    
    u8 *pp;
    pp = pkt_mesh_scene_rsp;
    memcpy (pkt_mesh_scene_rsp, &pkt_light_status, 48);
    pkt_mesh_scene_rsp[10] = device_address;      //mac
    pkt_mesh_scene_rsp[11] = device_address>>8;
    pkt_mesh_scene_rsp[12] = 0;
    
	pair_enc_packet_mesh (pp);    
}

void scene_rsp_mesh(){
    scene_poll_notify_flag = 1;
    scene_rsp_mesh_enc();
    mesh_user_cmd_idx = SCENE_MESH_NOTIFY_CNT;
}

int scene_poll_notify(u8 *pkt_rsp){
    int ret = 0;
    if(scene_poll_total == scene_poll_notify_cnt){
        //stop notify
        scene_poll_notify_cnt = scene_poll_total = scene_poll_notify_index = scene_poll_notify_flag = 0;
        ret = -1;
    }else{
        for(; scene_poll_notify_index < (MAX_SCENE_CNT);){
            scene_t *p_scene = &scene_data[scene_poll_notify_index++];
            if(0 == p_scene->id){
                continue;
            }
            ++scene_poll_notify_cnt;
            
            memcpy(pkt_rsp, p_scene, sizeof(scene_t));
            *(pkt_rsp + sizeof(scene_t)) = scene_poll_total;
                
            break;
        }
    }
    
    return ret;
}

int scene_get_by_id(u8 *pkt_rsp, u8 id){
    u8 idx = scene_find(id);
    if(idx < MAX_SCENE_CNT){    // found
        memcpy(pkt_rsp, &scene_data[idx], sizeof(scene_t));
        *(pkt_rsp + sizeof(scene_t)) = 1;
        return 1;
    }
    memset(pkt_rsp, 0, 10);     // not found
    return 0;
}

int scene_get_all_id(u8 *pkt_rsp){
    u8 total = 0;
    foreach(i,MAX_SCENE_CNT){
        scene_t *p_scene = &scene_data[i];
        if(p_scene->id){
            if(total < 10){  // max 10 per packet
                pkt_rsp[total++] = p_scene->id;
            }
        }
    }
    
    if(0 == total){
        memset(pkt_rsp, 0, 10);
    }
    return 0;
}

void scene_poll_notify_init(u8 need_bridge){
    scene_need_bridge_flag = need_bridge;
    scene_poll_notify_flag = 1;
    scene_poll_notify_cnt = scene_poll_notify_index = 0;
    scene_poll_total = 0;
    
    scene_notify_time = clock_time();
    scene_notify_tick = 0;
    
    foreach(i, MAX_SCENE_CNT){
        if(0 == scene_data[i].id){
            continue;
        }
        scene_poll_total++;
    }
}

void scene_handle(){
    if(is_scene_poll_notify_busy()){
        if(!scene_need_bridge_flag){
            if(clock_time_exceed(scene_notify_time, 50*1000)){
                scene_notify_time = clock_time();
                scene_notify_tick++;
                if(scene_notify_tick > 2){
                    if(0 == scene_poll_notify(pkt_light_status.value + 10)){
                        rf_link_slave_add_status (&pkt_light_status);
                    }
                }
            }
        }else{
            if(0 == mesh_user_cmd_idx){
                if(0 == scene_poll_notify(pkt_light_status.value + 10)){
                    scene_rsp_mesh();
                }
            }
        }
    }
}

int is_new_scene(rf_packet_att_value_t *p_buf, rf_packet_att_value_t *p){
    return (p_buf->val[3] != p->val[3]);
}

#else

u8 pkt_mesh_scene_rsp[1];   // must for compile

int is_scene_poll_notify_busy(){
    return 0;
}

int is_new_scene(rf_packet_att_value_t *p_buf, rf_packet_att_value_t *p){
    return 1;
}

#endif
