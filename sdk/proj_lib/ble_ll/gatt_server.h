/********************************************************************************************************
 * @file     gatt_server.h 
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

#include "gatt.h"


/** @addtogroup  GATT_Server_Module GATT server
 *  @{
 */

/** @addtogroup  GATT_Server_Types GATT server types
 *  @{
 */

/**
 *  @brief  Definition for client characteristic configuration change callback function
 */

typedef u8* (*att_handler_t)(u8 * p);
typedef int (*att_readwrite_callback_t)(void* p);
typedef struct attribute
{
  u8  attNum;
  u8  uuidLen;
  u8  attrLen;
  u8  attrMaxLen;
  u8* uuid;
  u8* pAttrValue;
  att_readwrite_callback_t w;
  att_readwrite_callback_t r;
} attribute_t;


/** @addtogroup GATT_Attr_Num_Calc GATT attribute number calculation
 * @{
 */
#define GATT_CALC_ATTR_NUM( attrArray )       (sizeof(attrArray) / sizeof(attribute_t))
/** @} end of group GATT_Attr_Num_Calc */

