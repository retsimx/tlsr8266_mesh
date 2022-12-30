/********************************************************************************************************
 * @file     gatt.h 
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
#include "att.h"
#include "gatt_uuid.h"

/** @addtogroup  GATT_Common_Module GATT common
 *  @{
 */
 

/** @addtogroup GATT_Characteristic_Property GATT characteristic properties
 * @{
 */
#define CHAR_PROP_BROADCAST              0x01 //!< permit broadcasts of the Characteristic Value
#define CHAR_PROP_READ                   0x02 //!< permit reads of the Characteristic Value
#define CHAR_PROP_WRITE_WITHOUT_RSP      0x04 //!< Permit writes of the Characteristic Value without response
#define CHAR_PROP_WRITE                  0x08 //!< Permit writes of the Characteristic Value with response
#define CHAR_PROP_NOTIFY                 0x10 //!< Permit notifications of a Characteristic Value without acknowledgement
#define CHAR_PROP_INDICATE               0x20 //!< Permit indications of a Characteristic Value with acknowledgement
#define CHAR_PROP_AUTHEN                 0x40 //!< permit signed writes to the Characteristic Value
#define CHAR_PROP_EXTENDED               0x80 //!< additional characteristic properties are defined
/** @} end of group GATT_Characteristic_Property */


/** @addtogroup GATT_CCCC_Bits Client CharacteristicConfiguration bits
 * @{
 */
#define CLIENT_CHAR_CFG_NOTI              0x0001 //!< permit broadcasts of the Characteristic Value
#define CLIENT_CHAR_CFG_IND               0x0002 //!< permit reads of the Characteristic Value
/** @} end of group GATT_CCCC_Bits */


/** @addtogroup GATT_Property_length GATT characteristic property length
 * @{
 */
#define CHAR_PROP_SIZE                   1
/** @} end of group GATT_Property_length */

/** @addtogroup GATT_Char_Cfg_Bit_length GATT characteristic configuration Bits length
 * @{
 */
#define CHAR_CFG_BITS_SIZE               2
/** @} end of group GATT_Char_Cfg_Bit_length */


/** @addtogroup  GATT_Functions GATT common APIs
 *  @{
 */

/**
 * @brief   Initialize GATT common module for client or server
 *
 * @param   None
 *
 * @return  None
 */
void gatt_init(void);

/** @} end of group GATT_Functions */


/** @} end of group GATT_Common_Module */

