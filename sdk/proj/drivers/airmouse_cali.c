/********************************************************************************************************
 * @file     airmouse_cali.c 
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
#if (__PROJECT_AIRMOUSE_CALI__)

#include "../../proj/tl_common.h"
#include "../../proj/drivers/airmouse.h"
#include "../../proj/drivers/led.h"
#include "../../proj/mcu/watchdog_i.h"

int mouse_sensor_getdata_no_fifo(s16 *ax, s16 *ay, s16 *az, s16 *gx, s16 *gy, s16 *gz){
	u8 buff[16];
	
	i2c_burst_read (MPU6050_I2C_ID, 0x3b, (u8*)buff, 14);
	
	*ax = (buff[0] << 8) + buff[1];
	*ay = (buff[2] << 8) + buff[3];
	*az = (buff[4] << 8) + buff[5];
	
	*gx = (buff[8] << 8) + buff[9];
	*gy = (buff[10] << 8) + buff[11];
	*gz = (buff[12] << 8) + buff[13];
	return 1;
}

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
static void calib(int max_cnt, s16 *avg_data) {
	int cnt = 0;
	int ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
	while(1){
		sleep_us(10*1000);

		static s16 acc_x, acc_y, acc_z, gy_x, gy_y, gy_z;
		
		if(mouse_sensor_getdata_no_fifo(&acc_x, &acc_y, &acc_z, &gy_x, &gy_y, &gy_z)){
			ax += acc_x;
			ay += acc_y;
			az += acc_z;

			gx += gy_x;
			gy += gy_y;
			gz += gy_z;

			++cnt;
			if(cnt >= max_cnt){
				break;
			}
		}
	}
	avg_data[0] = gx / max_cnt;
	avg_data[1] = gy / max_cnt;
	avg_data[2] = gz / max_cnt;
	avg_data[3] = ax / max_cnt;
	avg_data[4] = ay / max_cnt;
	avg_data[5] = az / max_cnt;

}

FLASH_ADDRESS_DEFINE;
int main (void){
	FLASH_ADDRESS_CONFIG;
	cpu_wakeup_init();
	clock_init();
	gpio_init();
	gpio_write(GPIO_PD7, 0);
	gpio_set_output_en(GPIO_PD7, 1);
	i2c_init();
	wd_stop();

	sleep_us(50*1000);
	
    mouse_sensor_no_fifo_init();

	sleep_us(1000*1000);	// for L3G,  must  delay enough time
	
	s16 avg_data[6];
	calib(64, avg_data);

	flash_erase_sector(AIRMOUSE_CALIBRATION_ADDR);
	flash_write_page(AIRMOUSE_CALIBRATION_ADDR, 12, avg_data);
	while (1);
	return 0;

}
#endif

