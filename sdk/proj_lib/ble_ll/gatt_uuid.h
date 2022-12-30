/********************************************************************************************************
 * @file     gatt_uuid.h 
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

/** @addtogroup  GATT_Common_Module GATT UUID
 *  @{
 */


/** @addtogroup GATT_UUID GATT 16 bit UUID definition
 * @{
 */
#define GATT_UUID_PRIMARY_SERVICE        0x2800     //!< Primary Service
#define GATT_UUID_SECONDARY_SERVICE      0x2801     //!< Secondary Service
#define GATT_UUID_INCLUDE                0x2802     //!< Include
#define GATT_UUID_CHARACTER              0x2803     //!< Characteristic
#define GATT_UUID_CHAR_EXT_PROPS         0x2900     //!< Characteristic Extended Properties
#define GATT_UUID_CHAR_USER_DESC         0x2901     //!< Characteristic User Description
#define GATT_UUID_CLIENT_CHAR_CFG        0x2902     //!< Client Characteristic Configuration
#define GATT_UUID_SERVER_CHAR_CFG        0x2903     //!< Server Characteristic Configuration
#define GATT_UUID_CHAR_PRESENT_FORMAT    0x2904     //!< Characteristic Present Format
#define GATT_UUID_CHAR_AGG_FORMAT        0x2905     //!< Characteristic Aggregate Format
#define GATT_UUID_VALID_RANGE            0x2906     //!< Valid Range
#define GATT_UUID_EXT_REPORT_REF         0x2907     //!< External Report Reference
#define GATT_UUID_REPORT_REF             0x2908     //!< Report Reference

#define GATT_UUID_DEVICE_NAME            0x2a00     //!< Report Reference

#define CHARACTERISTIC_UUID_MANU_NAME_STRING                    0x2A29
#define CHARACTERISTIC_UUID_MODEL_NUM_STRING                    0x2A24
#define CHARACTERISTIC_UUID_SERIAL_NUM_STRING                   0x2A25
#define CHARACTERISTIC_UUID_HW_REVISION_STRING                  0x2A27
#define CHARACTERISTIC_UUID_FW_REVISION_STRING                  0x2A26
#define CHARACTERISTIC_UUID_SW_REVISION_STRING                  0x2A28
#define CHARACTERISTIC_UUID_SYSTEM_ID                           0x2A23
#define CHARACTERISTIC_UUID_IEEE_11073_CERT_LIST                0x2A2A
#define CHARACTERISTIC_UUID_PNP_ID                              0x2A50

/** @} end of group GATT_UUID */


/** @addtogroup  GATT_UUID_Variables GATT UUID variables
 *  @{
 */
/**
 *  @brief  External variable for GATT UUID which can be used in service definition
 */
/** @} end of group GATT_UUID_Variables */

/** @} end of group GATT_Common_Module */
