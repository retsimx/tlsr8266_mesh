/********************************************************************************************************
 * @file     rtc.c 
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
/**************************************************************************************************
  Filename:       	rtc.c
  Author:			
  Created Date:	
  
  Description:    

**************************************************************************************************/

#include "../../proj/tl_common.h"
#include "../../proj_lib/ble_ll/blueLight.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "../../proj_lib/light_ll/light_ll.h"
#include "rtc.h"

#if(ALARM_EN)

FLASH_ADDRESS_EXTERN;

#ifndef	RTC_USE_32K_RC_ENABLE
#define RTC_USE_32K_RC_ENABLE	0		// enable should be better when pm enable
#endif

#define LOG_RTC_DEBUG(format,...)		//mini_printf(format,__VA_ARGS__)

#if RTC_USE_32K_RC_ENABLE
#define RTC_CALI_CIRCLE			30		// unit:minute
#define RTC_ADJUST_PER_MINUTE	(34*CLOCK_SYS_CLOCK_1MS)	// "+" to tune slow, "-" to tune fast. 

STATIC_ASSERT((RTC_CALI_CIRCLE <= 60) && (60 % RTC_CALI_CIRCLE == 0));

u32 cal_unit_32k;
u32 cal_unit_16m;
u32 tick_16m_begin;
u32 tick_32k_begin;
u8 rtc_delta_adjust;
u8 rtc_cali_flag = 1;

_attribute_ram_code_  void read_tick_32k_16m(u8 *tick_32k, u32 * tick_16m)
{
	u8 r=irq_disable();
	u8 pre = analog_read(0x40);
	while( pre == analog_read(0x40) );
	(*tick_16m) = clock_time();			
	tick_32k[0] = analog_read(0x40);
	tick_32k[1] = analog_read(0x41);
	tick_32k[2] = analog_read(0x42);
	tick_32k[3] = analog_read(0x43);
	irq_restore(r);
	return;
}

void rtc_cal_init()
{
	u32 tmp_32k_1, tmp_32k_2;
	u32 tmp_16m_1, tmp_16m_2;
	read_tick_32k_16m((u8 *)&tmp_32k_1, (u32 *)&tmp_16m_1);
	sleep_us(512*1000);
	read_tick_32k_16m((u8 *)&tmp_32k_2, (u32 *)&tmp_16m_2);

	cal_unit_32k = tmp_32k_2 - tmp_32k_1;
	cal_unit_16m = tmp_16m_2 - tmp_16m_1;
	
	LOG_RTC_DEBUG("rtc_cal_init cal_unit_32k:%d cal_unit_16m:%d div:%d\r\n", cal_unit_32k, cal_unit_16m, cal_unit_16m/cal_unit_32k);
}
#endif

rtc_t rtc = {
    .year = 1970,
    .month = 1,
    .day = 1,
    .hour = 0,
    .minute = 0,
    .second = 0,
    .week = 1,
};

const u8 days_by_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static inline u8 is_leap_year(const u16 year){
    return ((year%4==0 && year%100!=0) || year%400==0);
}

static inline u8 get_days_by_month(const u16 year, const u8 month){
    if((2==month) && (is_leap_year(year))){
        return 29;
    }else{
        return days_by_month[(month-1)%12];
    }
}
static inline u8 get_week(){
    u32 y = rtc.year;
    u32 m = rtc.month;
    u32 d = rtc.day;
    if((1 == m) || (2 == m)){
        y = y - 1;
        m = m + 12;
    }
    //sunday is 0
    u8 week = ((d+2*m+3*(m+1)/5+y+y/4-y/100+y/400)+1) % 7;
    return week;
}

void rtc_set_week(){
    rtc.week = get_week();
}

int time_par_check(const rtc_t * rtc_set){
    u8 days = get_days_by_month(rtc_set->year, rtc_set->month);
    
    if((!rtc_set->year)
      ||(!rtc_set->month || (rtc_set->month > 12))
      ||(!rtc_set->day || (rtc_set->day > days))
      ||(rtc_set->hour >= 24)
      ||(rtc_set->minute >= 60)
      ||(rtc_set->second >= 60)){
            return -1;
    }

    return 0;
}

int rtc_set_time(const rtc_t *rtc_set){
    //check;
    if(time_par_check(rtc_set)){
        return -1;
    }
    
    memcpy(&(rtc.year), rtc_set, 7);
    rtc_set_week();
	#if RTC_USE_32K_RC_ENABLE
	rtc.tick_last = tick_16m_begin;
	#else
    rtc.tick_last = clock_time();
	#endif
    return 0;
}

int rtc_get_time(u8 *rtc_get){
    memcpy(rtc_get, &(rtc.year), 7);
    return 0;
}

#define ALARM_CNT_MAX           16
#define ALARM_STORE_MAX_SIZE    FLASH_SECTOR_SIZE

enum{
	ALARM_EV_ADD = 0,
	ALARM_EV_DEL,
	ALARM_EV_CHANGE,
	ALARM_EV_ENABLE,
	ALARM_EV_DISABLE,
	ALARM_EV_MAX,
};

enum{
	ALARM_CMD_OFF = 0,
	ALARM_CMD_ON,
	ALARM_CMD_SCENE,
	ALARM_CMD_MAX,
};

enum{
	ALARM_TYPE_DAY = 0,
	ALARM_TYPE_WEEK,
	ALARM_TYPE_MAX,
};

enum{
	ALARM_INVALID = 0x00,
	ALARM_VALID = 0xA5,
};

enum{
	ALARM_VALID_FLAG_VALID = 0,
	ALARM_VALID_FLAG_INVALID,
	ALARM_VALID_FLAG_END,
	ALARM_VALID_FLAG_ADR_MAX,
};

enum{
	READ_CACHE_MODE = 0,
	READ_FLASH_MODE,
};

enum{
    ALARM_DEL_OK = 0,
	ALARM_ENABLE_DISABLE_OK,
	ALARM_ENABLE_OK,
	ALARM_DISABLE_OK,
};

typedef struct{ // max 10BYTES
    union {
        u8 event;
        u8 valid_flag;
    }par0;
    u8 index;
    struct {
        u8 cmd : 4;
        u8 type : 3;        
        u8 enable : 1;         
    }par1;
    u8 month;
    union {
        u8 day;
        u8 week;    // BIT(n)
    }par2;
    u8 hour;
    u8 minute;
    u8 second;
    u8 scene_id;
    //u8 alarm total for notiffy
}alarm_ev_t;
STATIC_ASSERT(sizeof(alarm_ev_t) + 1 <= 10);

alarm_ev_t alarm_list[ALARM_CNT_MAX];
static u16  alarm_next_pos = 0;

static inline int alarm_get_flag(u32 adr, u8 cache_mode){
    u8 flag_val = 0;
    if(READ_CACHE_MODE == cache_mode){
        flag_val = *(u8 *)adr;
    }else{
        flash_read_page(adr, 1, &flag_val);
    }
    if(adr > flash_adr_alarm + ALARM_STORE_MAX_SIZE - sizeof(alarm_ev_t)){
        return ALARM_VALID_FLAG_ADR_MAX;
    }else if(flag_val == 0xff){
        return ALARM_VALID_FLAG_END;
    }else if(flag_val == ALARM_VALID){
        return ALARM_VALID_FLAG_VALID;
    }else{
        return ALARM_VALID_FLAG_INVALID;
    }
}

int alarm_del(const alarm_ev_t *p_ev, alarm_ev_t *p_ev_rsp){
    int ret = ALARM_DEL_OK;
    u8 update_flash = 0;
    if(alarm_next_pos == 0){
        return 0;
    }

    foreach(i, ALARM_CNT_MAX){
        alarm_ev_t *p_alarm = &alarm_list[i];
        if(ALARM_VALID != p_alarm->par0.valid_flag){
            continue;
        }
        
        if(p_ev->index == 0xFF){//delete all
            memset(p_alarm, 0, sizeof(alarm_ev_t));
            update_flash = 1;
        }else if(p_ev->index == p_alarm->index){
            update_flash = 2;

            if((ALARM_EV_ENABLE == p_ev->par0.event) || (ALARM_EV_DISABLE == p_ev->par0.event)){
                if(p_ev_rsp){
                    memcpy(p_ev_rsp, p_alarm, sizeof(alarm_ev_t));
                    ret = ALARM_ENABLE_DISABLE_OK;
                }
            }
            memset(p_alarm, 0, sizeof(alarm_ev_t));
            
            break;
        }
    }
    u8 alarm_cnt = 0;
    alarm_ev_t p_alarm;
    alarm_ev_t del_val;
    memset(&del_val, 0, sizeof(alarm_ev_t));
    s16 i = alarm_next_pos - sizeof(alarm_ev_t);
    do{
        flash_read_page(flash_adr_alarm + i, sizeof(alarm_ev_t),(u8 *)&p_alarm);
        if(update_flash == 1 || ((update_flash == 2) && ((ALARM_VALID == p_alarm.par0.valid_flag) && (p_ev->index == p_alarm.index)))){
            flash_write_page(flash_adr_alarm + i, sizeof(alarm_ev_t), (u8 *)(&del_val));
            
            if(update_flash == 2 || ++alarm_cnt >= ALARM_CNT_MAX){
                break;
            }
        }
        if(i <= 0){
            break;
        }
        i -= sizeof(alarm_ev_t);
    }while(1);
    
    return ret;
}

void alarm_flash_clean(){
    if(alarm_next_pos >= FLASH_SECTOR_SIZE - sizeof(alarm_ev_t)){
        flash_erase_sector(flash_adr_alarm);
        flash_write_page(flash_adr_alarm, sizeof(alarm_list), (u8 *)(&alarm_list));
        alarm_next_pos = sizeof(alarm_list);
    }    
}

int alarm_add(const alarm_ev_t *p_ev){
    u8 overwrite = 1;    

    alarm_flash_clean();
    
    alarm_ev_t *p_alarm = NULL;
    foreach(i, ALARM_CNT_MAX){
        p_alarm = &alarm_list[i];
        if((p_alarm->par0.valid_flag == ALARM_VALID)
         &&(p_alarm->index == p_ev->index)){
            // exist
            return -1;
        }else if(p_alarm->par0.valid_flag != ALARM_VALID){
            memcpy(p_alarm, p_ev, sizeof(alarm_ev_t));
            if(p_alarm->par1.cmd == ALARM_CMD_OFF || p_alarm->par1.cmd == ALARM_CMD_ON){
                p_alarm->scene_id = 0;
            }
            p_alarm->par0.valid_flag = ALARM_VALID;
            overwrite = 0;
            break;
        }
    }
    
    if(overwrite){
        return -1;
    }
    
    // add one group
    if(p_alarm){
        flash_write_page(flash_adr_alarm + alarm_next_pos, sizeof(alarm_ev_t), (u8 *)p_alarm);
        alarm_next_pos += sizeof(alarm_ev_t);
    }

    return 0;
}

void alarm_retrieve(void){
	u8 alarm_cnt = 0;
	u32 alarm_next_adr = flash_adr_alarm;
	foreach(i, ALARM_STORE_MAX_SIZE / sizeof(alarm_ev_t)){
	    alarm_next_pos = i * sizeof(alarm_ev_t);
	    alarm_next_adr = flash_adr_alarm + i * sizeof(alarm_ev_t);
	    int flag = alarm_get_flag(alarm_next_adr, READ_CACHE_MODE);
        if((ALARM_VALID_FLAG_END == flag) || (ALARM_VALID_FLAG_ADR_MAX == flag)){
            break;
        }
        
        if(ALARM_VALID_FLAG_VALID != flag){
            continue;
        }

        // alarm address
        memcpy(&alarm_list[alarm_cnt%ALARM_CNT_MAX], (u8 *)alarm_next_adr, sizeof(alarm_ev_t));
        alarm_cnt++;
        alarm_next_pos += sizeof(alarm_ev_t);
	}
}

void alarm_init(){
    alarm_retrieve();
#if 0
	if (*(u32 *) flash_adr_alarm == 0xffffffff)
	{
        flash_write_page(flash_adr_alarm, sizeof(alarm_ev_t), &alarm0);
	}
#endif	
}

void alarm_callback(alarm_ev_t *p_alarm){
#if (__PROJECT_LIGHT_8266__ || __PROJECT_LIGHT_8267__ || __PROJECT_LIGHT_8269__ || __PROJECT_MASTER_LIGHT_8267__  || __PROJECT_LIGHT_8258__  \
  || __PROJECT_LIGHT_8278__ || __PROJECT_MASTER_LIGHT_8266__ || __PROJECT_LIGHT_NO_MESH__ || __PROJECT_LIGHT_8267_UART__ || __PROJECT_LIGHT_GATEWAY__)
    extern void light_onoff(u8 on);
    extern u8 scene_load(u8 id);
    switch(p_alarm->par1.cmd){
        case ALARM_CMD_OFF:
            light_onoff(0);
            break;
        case ALARM_CMD_ON:
            light_onoff(1);
#if WORK_SLEEP_EN // demo code
            sleep_us(200*1000);
            light_onoff(0);
            sleep_us(200*1000);
            light_onoff(1);
            sleep_us(200*1000);
            light_onoff(0);
            sleep_us(200*1000);
            light_onoff(1);
            sleep_us(200*1000);
#endif    
            break;
        case ALARM_CMD_SCENE:
            #if SCENE_EN
            scene_load(p_alarm->scene_id);
            #endif
            break;
        default:
            break;
    }
#endif    
}

int alarm_par_check(const alarm_ev_t* p_ev){
    if((0 == p_ev->index)||(0xFF == p_ev->index)){
        return -1;
    }
     
    if((p_ev->par1.cmd >= ALARM_CMD_MAX)
     ||(p_ev->par1.type >= ALARM_TYPE_MAX)){
        return -1;
     }

    if((p_ev->hour >= 24)
      ||(p_ev->minute >= 60)
      ||(p_ev->second >= 60)){
            return -1;
    }
    
    if(ALARM_TYPE_DAY == p_ev->par1.type){
        if((!(p_ev->month)) || (p_ev->month > 12)){
            return -1;
        }

        u8 days = days_by_month[(p_ev->month-1)%12];
        if(2 == p_ev->month){
            days = 29;
        }
        if((!(p_ev->par2.day)) || (p_ev->par2.day > days)){
            return -1;
        }
    }
    else if(ALARM_TYPE_WEEK == p_ev->par1.type){
        if((0 == p_ev->par2.week)
         ||((p_ev->par2.week >= 0x80) || (0 == p_ev->par2.week))){
            return -1;
        }
    } 

    return 0;
}

u8 get_next_shedule_idx()
{
	for(u8 j = 1; j <= ALARM_CNT_MAX; ++j){
		for(u8 i = 0; i < ALARM_CNT_MAX; ++i){
			if(alarm_list[i].index == j){
				break;
			}else if(i == ALARM_CNT_MAX - 1){
				return j;
			}
		}
	}
	return 0xFF;
}

int alarm_ev_callback(const u8 *ev){
    alarm_ev_t* p_ev = (alarm_ev_t*)ev;
    int ret = 0;
    switch(p_ev->par0.event){
        case ALARM_EV_ADD:
            if(0 == p_ev->index){
                p_ev->index = get_next_shedule_idx();
            }
        
            if(alarm_par_check(p_ev) == -1){
                return -1;
            }

            ret = alarm_add(p_ev);
            break;
            
        case ALARM_EV_DEL:
            alarm_del(p_ev, NULL);
            break;
            
        case ALARM_EV_CHANGE:
            if(alarm_par_check(p_ev) == -1){
                return -1;
            }

            alarm_del(p_ev, NULL);
            ret = alarm_add(p_ev);
            break;
               
        case ALARM_EV_ENABLE:
        case ALARM_EV_DISABLE: 
            {
                alarm_ev_t ev_rsp;
                if(ALARM_ENABLE_DISABLE_OK == alarm_del(p_ev, &ev_rsp)){
                    ev_rsp.par1.enable = ALARM_EV_ENABLE == p_ev->par0.event ? 1 : 0;
                    ret = alarm_add(&ev_rsp);
                }else{
                    ret = -1;
                }
            }
            break;
            
        default :
            break;
    }

    alarm_event_check();
    return ret;
}

void alarm_event_check(){
    rtc_set_week();

    if((alarm_next_pos == 0) || (is_need_sync_time())){
        return ;
    }

    foreach(i, ALARM_CNT_MAX){
        alarm_ev_t *p_alarm = &alarm_list[i];
        if(ALARM_VALID != p_alarm->par0.valid_flag){
            continue;
        }
        
        if(p_alarm->par1.enable){
            if((p_alarm->hour == rtc.hour)
            && (p_alarm->minute == rtc.minute)
            && (p_alarm->second == rtc.second)){//if needed, please run alarm_event_check by sencond
                u8 result = 0;
                switch(p_alarm->par1.type){
                    case ALARM_TYPE_DAY:
                        if((!p_alarm->month || (p_alarm->month == rtc.month))
                        && (!p_alarm->par2.day || (p_alarm->par2.day == rtc.day))){
                            result = 1;
                        }
                    break;
                    
                    case ALARM_TYPE_WEEK:
                        if((p_alarm->par2.week) & (u8)(1 << rtc.week)){
                            result = 1;
                        }
                    break;

                    default:
                        break;
                }
                
                if(result){
                    alarm_callback(p_alarm);
                }
            }
        }
    }
}

void rtc_increase_and_check_event(){
    rtc.second++;
    if(rtc.second >= 60){
        rtc.second %= 60;
        rtc.minute++;
		#if RTC_USE_32K_RC_ENABLE
		rtc_delta_adjust = 1;
		if(0 == (rtc.minute%RTC_CALI_CIRCLE)){
			rtc_cali_flag = 1;
		}
		#endif
        if(60 == rtc.minute){
            rtc.minute = 0;
            rtc.hour++;
            if(24 == rtc.hour){
                rtc.hour = 0;
                rtc.day++;
                u8 days = get_days_by_month(rtc.year, rtc.month);
                if((days+1) == rtc.day){
                    rtc.day = 1;
                    rtc.month++;
                    if(12+1 == rtc.month){
                        rtc.month = 1;
                        rtc.year++;
                    }
                }
                rtc_set_week();
            }
        }
    }
    
    alarm_event_check();
}

void rtc_run(){
	#if RTC_USE_32K_RC_ENABLE
	if(rtc_cali_flag){
		rtc_cali_flag = 0;
		rtc_cal_init();
	}
	#endif

    static u32 tick_rtc_run;
    if(clock_time_exceed(tick_rtc_run, 100*1000)){  // less irq_disable() should be better.
        tick_rtc_run = clock_time();		
        
        
        #if RTC_USE_32K_RC_ENABLE 
		u32 tick_32k, tick_16m;
		u8 r = irq_disable();   // avoid interrupt by set time command.
		read_tick_32k_16m((u8 *)&tick_32k, (u32 *)&tick_16m);
		u32 unit_cnt = (u32)(tick_32k - tick_32k_begin)/cal_unit_32k;
		tick_32k_begin += unit_cnt*cal_unit_32k;
		tick_16m_begin += unit_cnt*cal_unit_16m;		
		u32 t_delta = tick_16m_begin - rtc.tick_last;
		if(t_delta && rtc_delta_adjust){
			rtc_delta_adjust = 0;
			rtc.tick_last += RTC_ADJUST_PER_MINUTE;
		}
		#else
		u8 r = irq_disable();   // avoid interrupt by set time command.
		u32 t_last = rtc.tick_last; // rtc.tick_last may be set value in irq of set time command	
		u32 time = clock_time();    // get value must after t_last; 
        u32 t_delta = time - t_last;
		#endif
		
        if(t_delta > CLOCK_SYS_CLOCK_1S){
            u32 second_cnt = t_delta/CLOCK_SYS_CLOCK_1S;
            rtc.tick_last += CLOCK_SYS_CLOCK_1S*second_cnt;
            
            static u8 rtc_init_flag = 1;
            if(rtc_init_flag){
                rtc_init_flag = 0;
                rtc_set_week();
                alarm_init();
            }

            foreach(i,second_cnt){
                rtc_increase_and_check_event();
           	}
			#if RTC_USE_32K_RC_ENABLE			
			LOG_RTC_DEBUG("d:%d h:%d m:%d s:%d delta:%d\r\n", rtc.day, rtc.hour, rtc.minute, rtc.second, ((tick_32k-tick_32k_begin)/32+(tick_16m_begin-rtc.tick_last)/CLOCK_SYS_CLOCK_1MS)%1000);    
			#else
			LOG_RTC_DEBUG("d:%d h:%d m:%d s:%d delta:%d\r\n", rtc.day, rtc.hour, rtc.minute, rtc.second, (t_delta%CLOCK_SYS_CLOCK_1S)/CLOCK_SYS_CLOCK_1MS); 
			#endif
		}
        irq_restore(r);
    }
}

extern u8 security_enable;
extern u16 device_address;

extern u8 	mesh_user_cmd_idx;
extern rf_packet_att_cmd_t	pkt_light_status;
extern int	rf_link_slave_add_status (rf_packet_att_cmd_t *p);

static u8 alarm_poll_notify_flag;
static u8 alarm_poll_total;
static u8 alarm_poll_notify_index;
static u8 alarm_poll_notify_cnt;
static u32 alarm_notify_time;
static u32 alarm_notify_tick;
static u8 alarm_need_bridge_flag;
u8 pkt_mesh_alarm_rsp[48];

int is_alarm_poll_notify_busy(){
    return alarm_poll_notify_flag;
}

int pair_enc_packet_mesh (u8 *ps);
void alarm_rsp_mesh_enc(){
    pkt_light_status.type |= BIT(7);
    
    u8 *pp;
    pp = pkt_mesh_alarm_rsp;
    memcpy (pkt_mesh_alarm_rsp, &pkt_light_status, sizeof(pkt_light_status));
    pkt_mesh_alarm_rsp[10] = device_address;      //mac
    pkt_mesh_alarm_rsp[11] = device_address>>8;
    pkt_mesh_alarm_rsp[12] = 0;
    
	pair_enc_packet_mesh (pp);    
}

void alarm_rsp_mesh(){
    alarm_poll_notify_flag = 1;
    alarm_rsp_mesh_enc();
    mesh_user_cmd_idx = MESH_NOTIFY_CNT;
}


int alarm_poll_notify(u8 *pkt_rsp){
    int ret = 0;
    if(alarm_poll_total == alarm_poll_notify_cnt){
        //stop notify
        alarm_poll_notify_cnt = alarm_poll_total = alarm_poll_notify_index = alarm_poll_notify_flag = 0;
        ret = -1;
    }else{
        for(; alarm_poll_notify_index < (ALARM_CNT_MAX);){
            alarm_ev_t *p_alarm = &alarm_list[alarm_poll_notify_index++];
            if(ALARM_VALID != p_alarm->par0.valid_flag){
                continue;
            }
            ++alarm_poll_notify_cnt;
            memcpy(pkt_rsp, p_alarm, sizeof(alarm_ev_t));
            *(pkt_rsp + sizeof(alarm_ev_t)) = alarm_poll_total;
                
            break;
        }
    }
    
    return ret;
}

u8 alarm_find(u8 id){
    foreach(i, ALARM_CNT_MAX){
        alarm_ev_t *p_alarm = &alarm_list[i];
        if((ALARM_VALID == p_alarm->par0.valid_flag) && (id == p_alarm->index)){
            return i;
        }
    }

    return ALARM_CNT_MAX;
}

int alarm_get_by_id(u8 *pkt_rsp, u8 id){
    u8 idx = alarm_find(id);
    if(idx < ALARM_CNT_MAX){    // found
        memcpy(pkt_rsp, &alarm_list[idx], sizeof(alarm_ev_t));
        *(pkt_rsp + sizeof(alarm_ev_t)) = 1;
        return 1;
    }
    memset(pkt_rsp, 0, 10);     // not found
    return 0;
}

int alarm_get_all_id(u8 *pkt_rsp){
    u8 total = 0;
    foreach(i,ALARM_CNT_MAX){
        alarm_ev_t *p_alarm = &alarm_list[i];
        if(ALARM_VALID == p_alarm->par0.valid_flag){
            if(total < 10){  // max 10 per packet
                pkt_rsp[total++] = p_alarm->index;
            }
        }
    }
    
    if(0 == total){
        memset(pkt_rsp, 0, 10);
    }
    return 0;
}

void alarm_poll_notify_init(u8 need_bridge){
    alarm_need_bridge_flag = need_bridge;
    alarm_poll_notify_flag = 1;
    alarm_poll_notify_cnt = alarm_poll_notify_index = 0;
    alarm_poll_total = 0;
    
    alarm_notify_time = clock_time();
    alarm_notify_tick = 0;
    foreach(i, ALARM_CNT_MAX){
        if(ALARM_VALID != alarm_list[i].par0.valid_flag){
            continue;
        }
        alarm_poll_total++;
    }
}

void alarm_handle(){
    rtc_run();
    
    if(is_alarm_poll_notify_busy()){
        if(!alarm_need_bridge_flag){
            if(clock_time_exceed(alarm_notify_time, 50*1000)){
                alarm_notify_time = clock_time();
                alarm_notify_tick++;
                if(alarm_notify_tick > 2){
                    if(0 == alarm_poll_notify(pkt_light_status.value + 10)){
                        rf_link_slave_add_status (&pkt_light_status);
                    }
                }
            }
        }else{
            if(0 == mesh_user_cmd_idx){
                if(0 == alarm_poll_notify(pkt_light_status.value + 10)){
                    alarm_rsp_mesh();
                }
            }
        }
    }
}

int is_rtc_earlier(const rtc_t *rtc_new, const rtc_t *rtc_old){    // 
    if(is_valid_rtc_val(rtc_new) && is_valid_rtc_val(rtc_old)){
        if (rtc_new->year > rtc_old->year){
            return 1;
        }else if(rtc_new->year == rtc_old->year){
            if(rtc_new->month > rtc_old->month){
                return 1;
            }else if(rtc_new->month == rtc_old->month){
                if (rtc_new->day > rtc_old->day){
                    return 1;
                }else if(rtc_new->day == rtc_old->day){
                    if (rtc_new->hour > rtc_old->hour){
                        return 1;
                    }else if(rtc_new->hour == rtc_old->hour){
                        if(rtc_new->minute > rtc_old->minute){
                            return 1;
                        }else if(rtc_new->minute == rtc_old->minute){
                            if(rtc_new->second > rtc_old->second){
                                return 1;
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}

void check_event_after_set_time(const rtc_t *rtc_new, const rtc_t *rtc_old){
    u8 r = irq_disable();
    if(is_rtc_earlier(rtc_new, rtc_old)){   // should be earlier less than 1min ,
        memcpy(&rtc, rtc_old, sizeof(rtc_t));
        while(1){
            rtc_increase_and_check_event();
            if(rtc.second == rtc_new->second){
                memcpy(&rtc.year, &(rtc_new->year), 7);   // make sure recover rtc.
                break;
            }
        }
    }
    irq_restore(r);
}

int is_new_alarm(rf_packet_att_value_t *p_buf, rf_packet_att_value_t *p){
    return (p_buf->val[4] != p->val[4]);
}

int is_need_sync_time(){
    return (!is_valid_rtc_val(&rtc));
}

u8 send_local_valid_time_per_30s_en = 1;
void mesh_send_alarm_time (){
	if(send_local_valid_time_per_30s_en){
	    void mesh_send_alarm_time_ll ();
	    mesh_send_alarm_time_ll();
    }
}

int is_need_sync_rtc(rtc_t *r)
{
    return (!((!is_valid_rtc_val(r)) && is_valid_rtc_val(&rtc)));
}

void memcopy_rtc(void *out)
{
    memcpy(out, &rtc.year, 7);
}

void memcopy_rtc_hhmmss(void *out)
{
    memcpy(out, &rtc.hour, 3);
}

#else

u8 pkt_mesh_alarm_rsp[1];   // must for compile

int is_alarm_poll_notify_busy(){
    return 0;
}

int is_new_alarm(rf_packet_att_value_t *p_buf, rf_packet_att_value_t *p){
    return 1;
}

int rtc_set_time(const rtc_t *rtc_set){
    return -1;
}

void alarm_event_check(){
}

int is_need_sync_time(){
    return 0;
}

void mesh_send_alarm_time (){
}

int is_need_sync_rtc(rtc_t *r)
{
    return 0;
}

void memcopy_rtc(void *out)
{
}

void memcopy_rtc_hhmmss(void *out)
{
}

void check_event_after_set_time(const rtc_t *rtc_new, const rtc_t *rtc_old)
{
}
#endif

int is_valid_rtc_val(const rtc_t *r){
    return (r->year >= 2000);
}

