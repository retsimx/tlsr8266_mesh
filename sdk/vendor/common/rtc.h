/********************************************************************************************************
 * @file     rtc.h 
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

#define MESH_NOTIFY_CNT         6 // 4

typedef struct{
    u16 year;
    u8 month;
    u8 day;
    u8 hour;
    u8 minute;
    u8 second;
    u8 week;
    u32 tick_last;  //if add element, must after tick_last
}rtc_t;

extern rtc_t rtc;

int rtc_set_time(const rtc_t * rtc_set);
int rtc_get_time(u8 *rtc_get);
void alarm_handle();
int alarm_ev_callback(const u8 *ev);
void alarm_event_check();
int is_alarm_poll_notify_busy();
void alarm_poll_notify_init(u8 need_bridge);
int alarm_poll_notify(u8 *pkt_rsp);
void alarm_rsp_mesh();
int is_need_sync_time();
int is_valid_rtc_val(const rtc_t *r);
void mesh_send_alarm_time ();
int is_need_sync_rtc(rtc_t *r);
void memcopy_rtc(void *out);
void memcopy_rtc_hhmmss(void *out);
int alarm_get_by_id(u8 *pkt_rsp, u8 id);
int alarm_get_all_id(u8 *pkt_rsp);
void check_event_after_set_time(const rtc_t *rtc_new, const rtc_t *rtc_old);

