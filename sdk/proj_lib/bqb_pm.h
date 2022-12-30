/********************************************************************************************************
 * @file     bqb_pm.h 
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
#ifndef PM_H
#define PM_H

#include "bqb_bsp.h"

typedef enum {
	WAKEUP_SRC_PAD        = BIT(4),  
	WAKEUP_SRC_DIG_GPIO   = BIT(5) | 0X0800,
    WAKEUP_SRC_DIG_USB    = BIT(5) | 0X0400,
    WAKEUP_SRC_DIG_QDEC   = BIT(5) | 0X1000,  //0x6e[4] of 8267 is qdec wakeup enbale bit
	WAKEUP_SRC_TIMER      = BIT(6), 
	WAKEUP_SRC_COMP       = BIT(7), 
}WakeupSrc_TypeDef;

enum {
	LOWPWR_SUSPEND	  =		0,
	LOWPWR_DEEPSLEEP  =     1,
};

typedef enum{
    GROUPA_PIN0  = 0X000 | BIT(0),
    GROUPA_PIN1  = 0X000 | BIT(1),
    GROUPA_PIN2  = 0X000 | BIT(2),
    GROUPA_PIN3  = 0X000 | BIT(3),
    GROUPA_PIN4  = 0X000 | BIT(4),
    GROUPA_PIN5  = 0X000 | BIT(5),
    GROUPA_PIN6  = 0X000 | BIT(6),
    GROUPA_PIN7  = 0X000 | BIT(7),
    GROUPA_ALL   = 0X000 | 0X00ff,  

    GROUPB_PIN0  = 0X100 | BIT(0),
    GROUPB_PIN1  = 0X100 | BIT(1),
    GROUPB_PIN2  = 0X100 | BIT(2),
    GROUPB_PIN3  = 0X100 | BIT(3),
    GROUPB_PIN4  = 0X100 | BIT(4),
    GROUPB_PIN5  = 0X100 | BIT(5),
    GROUPB_PIN6  = 0X100 | BIT(6),
    GROUPB_PIN7  = 0X100 | BIT(7),
    GROUPB_ALL   = 0X100 | 0x00ff,

    GROUPC_PIN0  = 0X200 | BIT(0),
    GROUPC_PIN1  = 0X200 | BIT(1),
    GROUPC_PIN2  = 0X200 | BIT(2),
    GROUPC_PIN3  = 0X200 | BIT(3),
    GROUPC_PIN4  = 0X200 | BIT(4),
    GROUPC_PIN5  = 0X200 | BIT(5),
    GROUPC_PIN6  = 0X200 | BIT(6),
    GROUPC_PIN7  = 0X200 | BIT(7),
    GROUPC_ALL   = 0X200 | 0x00ff,

    GROUPD_PIN0  = 0X300 | BIT(0),
    GROUPD_PIN1  = 0X300 | BIT(1),
    GROUPD_PIN2  = 0X300 | BIT(2),
    GROUPD_PIN3  = 0X300 | BIT(3),
    GROUPD_PIN4  = 0X300 | BIT(4),
    GROUPD_PIN5  = 0X300 | BIT(5),
    GROUPD_PIN6  = 0X300 | BIT(6),
    GROUPD_PIN7  = 0X300 | BIT(7),
    GROUPD_ALL   = 0X300 | 0x00ff,

    GROUPE_PIN0  = 0X400 | BIT(0),
    GROUPE_PIN1  = 0X400 | BIT(1),
    GROUPE_PIN2  = 0X400 | BIT(2),
    GROUPE_PIN3  = 0X400 | BIT(3),
    GROUPE_ALL   = 0X400 | 0x000f,
}Pin_TypeDef;

extern void PM_PadSet(Pin_TypeDef pin, int pol, int en);
extern void PM_GPIOSet(Pin_TypeDef pin, int pol, int en);
extern void PM_WakeupInit(void);
extern int  PM_LowPwrEnter(int DeepSleep, int WakeupSrc, unsigned long WakeupTick); 

#endif //PM_H
