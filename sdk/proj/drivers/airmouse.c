/********************************************************************************************************
 * @file     airmouse.c 
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
#if 1

#include "../tl_common.h"
#include "../mcu/watchdog_i.h"
#include "airmouse.h"
//#include "../../proj/os/ev.h"
#include "i2c.h"

int airmouse_init(unsigned short orientation);
void airmouse_start_xy(void);
int airmouse_fusion(s16 *acc, s16 *gyro, s16 *cali_acc, s16 *cali_gyro, int interval, s8* x, s8* y, u8* more);

u8 mpu6050_init_flag = 0;
u8 mpu6050_data_ready = 0;
u8 airmouse_enable_flag;
u8 airmouse_button = 0;
static u32 airmouse_read_time;


static s16 cali_acc[3], cali_gyro[3];
static inline void mpu6050_read_cali_data(void){
	s16 * p_calib = (s16 *) AIRMOUSE_CALIBRATION_ADDR;
	cali_gyro[0] = p_calib[0];
	cali_gyro[1] = p_calib[1];
	cali_gyro[2] = p_calib[2];

	cali_acc[0] = p_calib[3];
	cali_acc[1] = p_calib[4];
	cali_acc[2] = p_calib[5];
}

int mpu6050_init_with_orient(void){
	// 每3 bits 为一组分别代表X,Y,Z 轴向. 例如低3bit代表x轴的轴向
	// 此值的意思是// 0: +x,   4:-x,  1:+y,  5:-y,	2:+z,  6:-z
	// 那么0b010001000 分别代表+Z+Y+X，则轴向没有改变。
	if(airmouse_init(0b110101000)){
		return -1;
	}
	return 0;
}

#if 0
static int mpu6050_init_timer(void *data){
	wd_stop();
	mpu6050_read_cali_data();
	mpu6050_init_with_orient();
	wd_start();
	airmouse_enable_flag = mpu6050_init_flag = 1;
	airmouse_read_time = clock_time();
	return -1;
}

static u32 mpu6050_init(void){
	extern int mpu_reset(void);
	mpu_reset();
	static ev_time_event_t mpu6050_timer = {mpu6050_init_timer};
	ev_on_timer(&mpu6050_timer, 120*1000);		//	Must wait after reset
	return 0;
}
#endif

extern int mpu_reset(void);


void airmouse_enable(int en){
	if(en){
		airmouse_start_xy();
		i2c_init();
		if(mpu6050_init_flag){
			airmouse_enable_flag = 1;
			airmouse_read_time = clock_time();
		}else{
#if 0
			mpu6050_init();
#else
			mpu_reset();
			sleep_us (120000);
			mpu6050_read_cali_data();
			mpu6050_init_with_orient();
			//wd_start();
			airmouse_enable_flag = mpu6050_init_flag = 1;
			airmouse_read_time = clock_time();
#endif
		}
	}else{
		airmouse_enable_flag = 0;
	}
}

int airmouse_getxy(amouse_data_t *mouse_data){
	if(!(airmouse_enable_flag && mpu6050_init_flag)){
		return 0;
	}
	u32 now = clock_time();
	int interval = (now - airmouse_read_time) / CLOCK_SYS_CLOCK_1US; 	//please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2
	airmouse_read_time = now;					//save for next loop, please note interval will be invalid in first sample but we don't use it

	mouse_data->x = mouse_data->y = 0;
	s16 acc[3], gyro[3];
	irq_disable();
	int motion = airmouse_fusion(acc, gyro, cali_acc, cali_gyro, interval, &mouse_data->x, &mouse_data->y, &mpu6050_data_ready);
	irq_enable();
	return motion;

}

#define MOUSE_PRESS_BTN_DELAY     (300*1000)
int airmouse_dither(int has_data, amouse_data_t *data){
    static u8 first_btn = 1;
    static u8 delay_start = 0;
    //  鼠标按键去抖
	static u32 delay_check_time = 0;
    if(airmouse_button){
        if(first_btn){
            first_btn = 0;
			delay_start = 1;
		    delay_check_time = clock_time();
        }
    }else{
        first_btn = 1;
    }
    if(delay_start){
        if(clock_time_exceed(delay_check_time, MOUSE_PRESS_BTN_DELAY)){
            delay_start = 0;
        }else{
        	data->x = data->y = 0;	// must.  set zero in case btn pressed
        	has_data = 0;
        }
    }
	return has_data;
}

int mouse_check_event(void* arg){
	amouse_data_t data;
	int has_data = airmouse_getxy(&data);
	if(0 == data.x && 0 == data.y){
		has_data = 0;
	}
	return 0;
}
#endif
