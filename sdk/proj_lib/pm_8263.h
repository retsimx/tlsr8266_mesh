/********************************************************************************************************
 * @file     pm_8263.h 
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

#if(__TL_LIB_8263__ || MCU_CORE_TYPE == MCU_CORE_8263)


static inline void usb_dp_pullup_en (int en)
{
}

enum {
	 //ana_0x17
	 PM_WAKEUP_CORE  = BIT(0),
	 PM_WAKEUP_TIMER = BIT(1),
	 PM_WAKEUP_PAD   = BIT(2),

	 PM_32KRC_RESET   = BIT(3),

	 //ana_0x18
	 PM_AUTO_PWDN_32K   = BIT(0),
	 PM_AUTO_PWDN_XTAL  = BIT(2),
	 PM_AUTO_PWDN_ANA   = BIT(3),
	 PM_AUTO_PWDN_EN    = BIT(4),
	 PM_AUTO_PWDN_LLLDO  = BIT(5),
	 PM_AUTO_PWDN_DLDO   = BIT(6),
	 PM_AUTO_PWDN_ISO    = BIT(7),

	 PM_AUTO_PWDN_SUSPEND    = 0X5e,
	 PM_AUTO_PWDN_DEEPSLEEP    = 0Xfe,
};

enum{
	// WAKEUP_SRC_ANA 0 -- 2  not supported
	 WAKEUP_PC3_GRP0 = BIT(0),
	 WAKEUP_PC4_GRP0 = BIT(1),
	 WAKEUP_PC5_GRP0 = BIT(2),
	 WAKEUP_PD0_GRP0 = BIT(3),
	 WAKEUP_PD1_GRP1 = BIT(4),
	 WAKEUP_PD2_GRP1 = BIT(5),
	 WAKEUP_PD3_GRP1 = BIT(6),
	 WAKEUP_PD4_GRP1 = BIT(7),
	 WAKEUP_PD5_GRP2 = BIT(8),
	 WAKEUP_PD6_GRP2 = BIT(9),
	 WAKEUP_PD7_GRP2 = BIT(10),
	 WAKEUP_PA0_GRP2 = BIT(11),
	 WAKEUP_PA1_GRP3 = BIT(12),
	 WAKEUP_PA2_GRP3 = BIT(13),
	 WAKEUP_PA3_GRP3 = BIT(14),
	 WAKEUP_PA4_GRP3 = BIT(15),
	 WAKEUP_PA7_GRP4 = BIT(16),
	 WAKEUP_PC6_GRP4 = BIT(17),
	 WAKEUP_PC7_GRP4 = BIT(18),
	 WAKEUP_PE0_GRP4 = BIT(19),
	 WAKEUP_PE1_GRP5 = BIT(20),
	 WAKEUP_PE2_GRP5 = BIT(21),
	 WAKEUP_PA5_GRP5 = BIT(22),
	 WAKEUP_PA6_GRP5 = BIT(23),
};
/*wakeup-level*/
enum{
	WAKEUP_GRP0_POS_EDG = 0,
	WAKEUP_GRP1_POS_EDG = 0,
	WAKEUP_GRP2_POS_EDG = 0,
	WAKEUP_GRP3_POS_EDG = 0,
	WAKEUP_GRP4_POS_EDG = 0,
	WAKEUP_GRP5_POS_EDG = 0,

	WAKEUP_GRP0_NEG_EDG = BIT(0),
	WAKEUP_GRP1_NEG_EDG = BIT(1),
	WAKEUP_GRP2_NEG_EDG = BIT(2),
	WAKEUP_GRP3_NEG_EDG = BIT(3),
	WAKEUP_GRP4_NEG_EDG = BIT(4),
	WAKEUP_GRP5_NEG_EDG = BIT(5),

};



///////////////////////////////////////////////////////////////////////
////////////////////////////battery dectect////////////////////////////
////////////////////////////////////////////////////////////////////////


//ana03<4:1>
enum  COMP_CHANNALE {
	COMP_ANA0 = 0x00,  COMP_GP1  = COMP_ANA0,
	COMP_ANA1 = 0x02,  COMP_GP2  = COMP_ANA1,
	COMP_ANA2 = 0x04,  COMP_GP3  = COMP_ANA2,
	COMP_ANA3 = 0x06,  COMP_GP4  = COMP_ANA3,
	COMP_ANA4 = 0x08,  COMP_GP7  = COMP_ANA4,
	COMP_ANA5 = 0x0a,  COMP_GP8  = COMP_ANA5,
	COMP_ANA6 = 0x0c,  COMP_GP9  = COMP_ANA6,
	COMP_ANA7 = 0x0e,  COMP_GP10 = COMP_ANA7,
	COMP_ANA8 = 0x10,  COMP_GP0  = COMP_ANA8,
	COMP_ANA9 = 0x12,  COMP_GP5  = COMP_ANA9,
	COMP_ANA10 = 0x14, COMP_GP6  = COMP_ANA10,
	//COMP_ANA11 not connect
	//COMP_ANA12 not connect
	COMP_AVDD =  0x1a
};


enum{
	V0P98,
	V1P1,
	V1P18,
	V1P25,
	V1P3,
	V1P66,
};


//ana03<6:5>
#define SCALING_SELECT_QUARTER 		0x00//25%
#define SCALING_SELECT_HALF 		0x20//50%
#define SCALING_SELECT_3QUARTER 	0x40//75%
#define SCALING_SELECT_FULL 		0x60//100%


/*Set 32K RC calibration mode
 * START_CAL_32K_FROM_SUSPEND 0: original mode, start cal after wakeup from suspend
 * START_CAL_32K_OUTSIDE 1: the time to start cal decided by user 
 * START_CAL_32K_FROM_SUSPEND_AND_WAIT500US 2: start cal after wakeup from suspend, and then wait for 500us until cal finished
 * */
 #define START_CAL_32K_FROM_SUSPEND  0
 #define START_CAL_32K_OUTSIDE             1
 #define START_CAL_32K_FROM_SUSPEND_AND_WAIT500US  2
 void blt_set_32k_cal_mode (u8 cal_mode);
  u8 blt_get_32k_cal_mode(void);

/*test data
 * standard      test
 * 0.832v <--->0.815v
 * 0.884v <--->0.86v
 * 0.936v <--->0.91v
 * 0.988v <--->0.955v
 * 1.040v <--->1.015v
 * 1.092v <--->1.05v
 * 1.196v <--->1.16v
 * */

//ana02<6:4>
#define REF_VOLTAGE_SEL_FLOAT			0x00//float
#define REF_VOLTAGE_SEL_1160_MV			0x10//theoretical value 1196 mv , actual test value 1160 mv
#define REF_VOLTAGE_SEL_1050_MV			0x20//theoretical value 1092 mv , actual test value 1050 mv
#define REF_VOLTAGE_SEL_1015_MV			0x30//theoretical value 1040 mv , actual test value 1015 mv
#define REF_VOLTAGE_SEL_955_MV			0x40//theoretical value 988 mv , actual test value 955 mv
#define REF_VOLTAGE_SEL_910_MV			0x50//theoretical value 936 mv , actual test value 910 mv
#define REF_VOLTAGE_SEL_860_MV			0x60//theoretical value 884 mv , actual test value 860 mv
#define REF_VOLTAGE_SEL_815_MV			0x70//theoretical value 832 mv , actual test value 815 mv


#define		V1P0			0
#define		V1P1			1
#define		V1P25			2
#define		V1P4			3



#ifndef		VBAT_LOW_LEVLE
#define		VBAT_LOW_LEVLE		V1P0
#endif


#ifndef		VBAT_CHANNEL
#if BATTERY_DETECTION_WITH_LDO_SET
#define		VBAT_CHANNEL		COMP_AVDD
#else
#define		VBAT_CHANNEL		COMP_ANA8
#endif
#endif


#define		V1P0_REF			REF_VOLTAGE_SEL_1015_MV    //1.045-1.055
#define		V1P0_SCALE			SCALING_SELECT_FULL

#define		V1P1_REF			REF_VOLTAGE_SEL_1050_MV
#define		V1P1_SCALE			SCALING_SELECT_HALF

#define		V1P7_REF			REF_VOLTAGE_SEL_860_MV	 //1.72 batt_det for single or double
#define		V1P7_SCALE			SCALING_SELECT_HALF

#define		V1P8_REF			REF_VOLTAGE_SEL_910_MV   //1.805-1.815
#define		V1P8_SCALE			SCALING_SELECT_HALF

#define 	V2P0_REF			REF_VOLTAGE_SEL_1015_MV  //2.020-2.030
#define 	V2P0_SCALE			SCALING_SELECT_HALF

#define 	V2P1_REF			REF_VOLTAGE_SEL_1050_MV
#define 	V2P1_SCALE			SCALING_SELECT_HALF

#define 	V2P3_REF			REF_VOLTAGE_SEL_1160_MV
#define 	V2P3_SCALE			SCALING_SELECT_HALF

#define		VBAT_LOW_SCALE		(VBAT_LOW_LEVLE==V1P0 ? V1P0_SCALE : V1P1_SCALE )
#define		VBAT_LOW_REF		(VBAT_LOW_LEVLE==V1P0 ? V1P0_REF : V1P1_REF)

#define		VBAT_LOW_SCALE_S	VBAT_LOW_SCALE
#define		VBAT_LOW_REF_S		VBAT_LOW_REF

#define		VBAT_LOW_SCALE_MIDL	(V1P7_SCALE)
#define		VBAT_LOW_REF_MIDL	(V1P7_REF)

#define		VBAT_LOW_SCALE_D	(VBAT_LOW_LEVLE==V1P0 ? V2P0_SCALE : V2P1_SCALE)
#define		VBAT_LOW_REF_D		(VBAT_LOW_LEVLE==V1P0 ? V2P0_REF : V2P1_REF)

int battery_low_by_set ( u8 chn, u8 v_ref, u8 v_scale ) ;
void battery_by_comp_init();

static inline u32 battery_low_detect_auto ( u8 vbat_chn ){
    if ( vbat_chn > COMP_AVDD )     //no detect
        return 0;
    
    int bat_low_flg = 0;
    if ( battery_low_by_set (vbat_chn, VBAT_LOW_REF_MIDL, VBAT_LOW_SCALE_MIDL) ){
        bat_low_flg = battery_low_by_set (vbat_chn, VBAT_LOW_REF_S, VBAT_LOW_SCALE_S) ;
    }
    else{
        bat_low_flg = battery_low_by_set (vbat_chn, VBAT_LOW_REF_D, VBAT_LOW_SCALE_D) ;
    }
    return bat_low_flg;
}

#define SUSPEND_MODE	0
#define DEEPSLEEP_MODE	1

#define WAKEUP_LEVEL_L 	0
#define WAKEUP_LEVEL_H 	1

// usually, we don't use gpio wakeup in suspend mode.
// If you do need it,  pls turn on this micro, add set  wakeup pin before calling cpu_sleep_wakeup

// like:  
// reg_gpio_f_wakeup_en = SUSPEND_WAKEUP_SRC_PWM0;
// reg_gpio_f_pol = SUSPEND_WAKEUP_SRC_PWM0;
// cpu_sleep_wakeup(1, 50, 0, 0)
#define PM_SUSPEND_WAKEUP_BY_GPIO_ENABLE		0

void pm_init(void);

static inline int cpu_get_32k_tick (void) {
	return 1;
}

enum {
	RC_TRACKING_32K_ENABLE = BIT(3),
	RC_TRACKING_32M_ENABLE = BIT(1),
};


static inline void cpu_rc_tracking_en (int en) {
	analog_write (0x24, en ? BIT(3) : 0);
}

#define cpu_rc_tracking_disable do{ analog_write (0x24, 0); }while(0)

//deepsleep mode must use this function for resume 1.8V analog register
void cpu_wakeup_init(void);
void cpu_set_gpio_wakeup (int pin, int pol, int en);
int cpu_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_tick);


extern const u16 wakeup_src_pin[];

#endif
