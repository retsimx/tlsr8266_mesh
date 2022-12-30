/********************************************************************************************************
 * @file     mpu6050.c 
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
#include "../tl_common.h"
#include "../drivers/i2c.h"

#ifndef	GYRO_MPU6050_ENBALE
#define	GYRO_MPU6050_ENBALE		1
#endif

#if(GYRO_MPU6050_ENBALE)

//#include "mouse_sensor.h"
//#include "../drivers/mouse.h"
#include "../../proj_lib/airmouse.h"
//#include "../os/sys.h"
#include "mpu6050.h"
//#include "mouse_sensor.h"
//#include "../drivers/somatic_sensor.h"
#include "../../proj_lib/airmouse.h"
#include "../../proj/common/types.h"
#include "../../vendor/common/mouse_type.h"

#define AIRMOUSE_FUSION_ENABLE			1

#ifndef		AIRMOUSE_CALIBRATION_ADDR
#define		AIRMOUSE_CALIBRATION_ADDR			0x8000
#endif

s16 cali_acc[3], cali_gyro[3];
static inline void mouse_read_cali_data(void){
	s16 * p_calib = (s16 *) AIRMOUSE_CALIBRATION_ADDR;
	cali_gyro[0] = p_calib[0];
	cali_gyro[1] = p_calib[1];
	cali_gyro[2] = p_calib[2];

	cali_acc[0] = p_calib[3];
	cali_acc[1] = p_calib[4];
	cali_acc[2] = p_calib[5];
}

#if(AIRMOUSE_FUSION_ENABLE)
int mouse_init_with_orient(void){
	// 每3 bits 为一组分别代表X,Y,Z 轴向. 例如低3bit代表x轴的轴向
	// 此值的意思是// 0: +x,   4:-x,  1:+y,  5:-y,	2:+z,  6:-z
	// 那么0b010001000 分别代表+Z+Y+X，则轴向没有改变。
	extern int airmouse_init(unsigned short orientation);
	if(airmouse_init(0b110101000)){
		return -1;
	}
	return 0;
}

u8 mouse_init_flag = 0;
u8 air_mouse_enable;
u8 air_mouse_last_enable;
u8  mouse_enable_flag = 0;
u32 mouse_enable_time;
static int mouse_init_timer(void *data){
//	wd_stop();
	mouse_read_cali_data();
	mouse_init_with_orient();
//	wd_start();
	air_mouse_last_enable = air_mouse_enable = mouse_init_flag = 1;
	return -1;
}

u32 mouse_sensor_init(void){
	extern int mpu_reset(void);
	mpu_reset();
	//	Must wait after reset
#if 0
	static ev_time_event_t airmouse_timer = {mouse_init_timer};
	ev_on_timer(&airmouse_timer, 120*1000);
#else
	sleep_us (120*1000);
	mouse_init_timer (0);
#endif
	return 0;
}

#else
u32 mouse_sensor_init(void){
	static const u8 tbl_sensor_init[] = {
			0x6b, 0x01, 	// internal 8M clk oscillator
			0x38, 0x11, 	// data rdy irq  en
			0x1b, 0x18,		// gyro full rage: 2000 c/ s
			0x1a, 0x02, 	// Fs = 1k,  bandwidth = 98hz
			0x19, 0x09, 	// sample rate =  FS / (0x19+1)
			0x1c, 0x08,		// set 4g mode
	#if(AIRMOUSE_FIFO_MODE)
			0x23, 0x78, 	// enable gsense/gyro fifo
			0x6a, 0x40, 	// enable fifo
	#endif
	};
	// wait until MPU power ready
	int ok = 0;
	for (int i = 0; i < 500; ++i) {
		if (i2c_read(MPU6050_I2C_ID, 0x75) == 0x68) {
			ok++;
			if (ok++ > 3) {
				break;
			}
		}
		sleep_us (1000);
	}
	int len = sizeof (tbl_sensor_init);
	for (int j=0; j<len; j+=2) {
		i2c_write(MPU6050_I2C_ID, tbl_sensor_init[j], tbl_sensor_init[j+1]);
	}
	mouse_read_cali_data();
	return 0;
}
#endif	

void mouse_sensor_powerup(void){
	i2c_write(MPU6050_I2C_ID,0x6B,0x01);
}
void mouse_sensor_powerdown(void){
	i2c_write(MPU6050_I2C_ID,0x6B,0x41);
}

void mouse_drv_enable_6_axis(int en){}

void mouse_sensor_enable(int en){
	if(!en){
		air_mouse_last_enable = air_mouse_enable = 0;
	}
	else if(!air_mouse_last_enable){
		mouse_enable_flag = 0;
		mouse_enable_time = clock_time();
		void airmouse_start_xy(void);
		airmouse_start_xy();
		if(!mouse_init_flag){
			i2c_init ();
			mouse_sensor_init();
		}
	}
	
}

u8 gyro_air_button;
void mouse_drv_getbtn(mouse_data_t *mouse_data){
	if(air_mouse_enable){
		mouse_data->btn = gyro_air_button;
	}
}

int mouse_sensor_getdata_no_fifo(s16 *ax, s16 *ay, s16 *az, s16 *gx, s16 *gy, s16 *gz){
	u8 buff[16]={0};
	
	i2c_burst_read (MPU6050_I2C_ID, 0x3b, (u8*)buff, 14);
	
	*ax = (buff[0] << 8) + buff[1];
	*ay = (buff[2] << 8) + buff[3];
	*az = (buff[4] << 8) + buff[5];
	
	*gx = (buff[8] << 8) + buff[9];
	*gy = (buff[10] << 8) + buff[11];
	*gz = (buff[12] << 8) + buff[13];
	return 1;
}

int mouse_sensor_getdata(s16 *ax, s16 *ay, s16 *az, s16 *gx, s16 *gy, s16 *gz){

	u8 buff[16]={0};
	
	i2c_burst_read (MPU6050_I2C_ID, 0x74, (u8*)buff, 12);
	
	*ax = (buff[0] << 8) | buff[1];
	*ay = (buff[2] << 8) | buff[3];
	*az = (buff[4] << 8) | buff[5];
	
	*gx = (buff[6] << 8) | buff[7];
	*gy = (buff[8] << 8) | buff[9];
	*gz = (buff[10] << 8) | buff[11];
	return 1;
}

u8 gyro_data_ready = 0;
u8 game_mode_enable;

#if(MODULE_SOMATIC_ENABLE)
int mouse_sensor_getxy(mouse_data_t *mouse_data, somatic_sensor_data_t *somatic_data){
	if(!air_mouse_enable
		&& !game_mode_enable
#else
int mouse_sensor_getxy(mouse_data_t *mouse_data){
	if(!air_mouse_enable
#endif		
		){
		return 0;
	}
	if(0 == mouse_enable_flag && !clock_time_exceed(mouse_enable_time, 50*1000)){
		return 0;
	}
	mouse_enable_flag = 1;

	static u32 last_time;
#if(MODULE_PM_ENABLE && PM_SUSPEND_ENABLE)
	int interval = MOUSE_SCAN_INTERVAL; 	//please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2
#else
	u32 now = clock_time();
	int interval = (now - last_time) / CLOCK_SYS_CLOCK_1US; 	//please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2
	last_time = now;					//save for next loop, please note interval will be invalid in first sample but we don't use it
#endif
	mouse_data->x = mouse_data->y = 0;
#if(MODULE_SOMATIC_ENABLE)
	STATIC_ASSERT_INT_DIV(sizeof(somatic_sensor_data_t), 4);
	memset4(somatic_data, 0, sizeof(somatic_sensor_data_t));
#endif
	static s16 acc[3], gyro[3];
#if(AIRMOUSE_FUSION_ENABLE)
	extern int airmouse_fusion(s16 *acc, s16 *gyro, s16 *cali_acc, s16 *cali_gyro, int interval, s8* x, s8* y, u8* more);
	int motion = airmouse_fusion(acc, gyro, cali_acc, cali_gyro, interval, &mouse_data->x, &mouse_data->y, &gyro_data_ready);
#if(MODULE_SOMATIC_ENABLE)
	if(game_mode_enable){
		somatic_data->version = 0x100;
		somatic_data->gsensor_x = acc[0];
		somatic_data->gsensor_y = acc[1];
		somatic_data->gsensor_z = acc[2];
		somatic_data->gyro_x = gyro[0];
		somatic_data->gyro_y = gyro[1];
		somatic_data->gyro_z = gyro[2];
		return 0;
	}
#endif

#else
	
#if(AIRMOUSE_FIFO_MODE)
	u8 buff[2];
	int am_fifo_cnt = 0;
	i2c_burst_read (MPU6050_I2C_ID, 0x72, (u8*)buff, 2);
	am_fifo_cnt = (buff[0] << 8) + buff[1];
	if(am_fifo_cnt <= 0){
		return 0;
	}
	mouse_sensor_getdata(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
#else
	gyro_data_ready = 0;
	mouse_sensor_getdata_no_fifo(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);
#endif

	acc[0] -= cali_acc[0];
	acc[1] -= cali_acc[1];
	if(cali_acc[2] > 0){
		acc[2] = acc[2] - cali_acc[2] + 0x2000;
	}else{
		acc[2] = acc[2] - cali_acc[2] - 0x2000;
	}
	gyro[0] -= cali_gyro[0];
	gyro[1] -= cali_gyro[1];
	gyro[2] -= cali_gyro[2];
	
#if(MODULE_SOMATIC_ENABLE)
	if(game_mode_enable){
		somatic_data->version = 0x100;
		somatic_data->gsensor_x = acc[0];
		somatic_data->gsensor_y = acc[1];
		somatic_data->gsensor_z = acc[2];
		somatic_data->gyro_x = gyro[0];
		somatic_data->gyro_y = gyro[1];
		somatic_data->gyro_z = gyro[2];
		return 0;
	}
#endif

	extern int airmouse_fast_xy(s16 acc_x, s16 acc_y, s16 acc_z, s16 gy_x, s16 gy_y, s16 gy_z, int interval, s8* x, s8* y);
	// 参数方向为airmouse_xy(left == +x,  front == +y,  down == +z , left,  front,  down,  time, out_x, out_y)
	int motion = airmouse_fast_xy(acc[0], acc[1], -acc[2], gyro[0], gyro[1], -gyro[2], interval, &mouse_data->x, &mouse_data->y);

#endif

	return motion;

}

#if (__PROJECT_CALIB__)
static const u8 tbl_sensor_no_fifo_init[] = {
		0x6b, 0x01,		// internal 8M clk oscillator
		0x38, 0x11,		// data rdy irq  en
		0x1b, 0x18,		// gyro full rage: 250 c/ s
		0x1a, 0x02,		// Fs = 1k,  bandwidth = 98hz
		0x19, 0x03,		// sample rate =  FS / (0x19+1)
		0x1c, 0x08,		// set 8g mode
};

u32 mouse_sensor_no_fifo_init(void){
	// wait until MPU power ready
	static int ok = 0;
	for (int i = 0; i < 500; ++i) {
		if (i2c_read(MPU6050_I2C_ID, 0x75) == 0x68) {
			ok++;
			if (ok++ > 3) {
				break;
			}
		}
		sleep_us (1000);
	}
	int len = sizeof (tbl_sensor_no_fifo_init);
	for (int j=0; j<len; j+=2) {
		i2c_write(MPU6050_I2C_ID, tbl_sensor_no_fifo_init[j], tbl_sensor_no_fifo_init[j+1]);
	}
	return 1;
}

#endif	
#endif	

