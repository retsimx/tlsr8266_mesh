/********************************************************************************************************
 * @file     airmouse.h 
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
#ifndef __AIRMOUSE_H__
#define __AIRMOUSE_H__

#define			CLOCKMHZ		30
#define			DPS250			0.00875
#define			DPSPS			DPS250

typedef struct {
	signed char x;
	signed char y;
	signed short d;
	signed short gx;
	signed short gy;
	signed int	 gz;
	signed int	dz;
	signed int	md;
	int			ox;
	int			oy;
} gyro_mouse_data_t;


extern int		am_scale_x;
extern int		am_scale_y;
extern int		am_static_th;
extern int		am_moving_th;
extern int		am_moving_md_thh;
extern int		am_moving_md_thl;

extern gyro_mouse_data_t	gyro_mouse_dat;

int AirMouse (int x, int y, int z, int t, gyro_mouse_data_t * pg, int key);
int AirMouseG (int x, int y, int z, gyro_mouse_data_t * pg);
void airmouse_start_xy(void);

#endif

