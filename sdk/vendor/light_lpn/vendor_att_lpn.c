/********************************************************************************************************
 * @file     vendor_att_switch.c 
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
#include "../../proj/tl_common.h"

///////////////////////////  ATT  protocol  ///////////////////////////
#include "../../proj_lib/ble_ll/ble_common.h"
#include "../../proj_lib/ble_ll/att.h"
#include "../../proj_lib/ble_ll/gatt_uuid.h"
#include "../../proj_lib/ble_ll/service.h"
#include "../../proj_lib/ble_ll/gatt_server.h"
#include "../../proj_lib/ble_ll/gap_const.h"
#include "../../proj_lib/light_ll/light_frame.h"

/**********************************************************************
 * LOCAL VARIABLES 
 */
const u16 primaryServiceUUID_switch = GATT_UUID_PRIMARY_SERVICE;
static const u16 characterUUID_switch = GATT_UUID_CHARACTER;

const u16 gapServiceUUID_switch = 0x1800;
const u16 devNameUUID_switch = GATT_UUID_DEVICE_NAME;
const u16 appearanceUIID_switch = 0x2a01;
const u16 periConnParamUUID_switch = 0x2a04;

// Device Name Characteristic Properties
static const u8 devNameCharacter_switch = CHAR_PROP_READ;

// Appearance Characteristic Properties
static const u8 appearanceCharacter_switch = CHAR_PROP_READ;

// Peripheral Preferred Connection Parameters Characteristic Properties
static const u8 periConnParamChar_switch = CHAR_PROP_READ;

u16 appearance_switch = GAP_APPEARE_UNKNOWN;

typedef struct
{
  /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  u16 intervalMin;
  /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  u16 intervalMax;
  /** Number of LL latency connection events (0x0000 - 0x03e8) */
  u16 latency;
  /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
  u16 timeout;
} gap_periConnectParams_t;

gap_periConnectParams_t periConnParameters_switch = {20, 40, 0, 1000};

u8	ble_g_devName_switch[] = {DEVICE_NAME, 0};

//////////////////////// SPP /////////////////////////////////////////////////////
u8 TelinkSppServiceUUID_switch[16]				= TELINK_SPP_UUID_SERVICE;

u8 TelinkSppDataServer2ClientUUID_switch[16] 	= TELINK_SPP_DATA_SERVER2CLIENT;
u8 TelinkSppDataClient2ServiceUUID_switch[16]	= TELINK_SPP_DATA_CLIENT2SERVER;
u8 TelinkSppDataOtaUUID_switch[16]				= TELINK_SPP_DATA_OTA;
u8 TelinkSppDataPairUUID_switch[16]				= TELINK_SPP_DATA_PAIR;

// Spp data from Server to Client characteristic variables
static const u8 SppDataServer2ClientProp_switch		= CHAR_PROP_READ | CHAR_PROP_NOTIFY;

// Spp data from Client to Server characteristic variables
static const u8 SppDataClient2ServerProp_switch		= CHAR_PROP_READ | CHAR_PROP_WRITE;

// Spp data for OTA characteristic variables
static const u8 SppDataOtaProp_switch					= CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP;
u8 SppDataOtaData_switch[20] = {0xe0};

u8 SppDataServer2ClientData_switch[20] 			= {0xf0};

// Spp data for OTA characteristic variables
static const u8 SppDataPairProp_switch				= CHAR_PROP_READ | CHAR_PROP_WRITE;
u8 SppDataPairData_switch[20] = {0xe0};

extern u8 send_to_master[16];
#define		SppDataClient2ServerData			send_to_master

const u16 userdesc_UUID_switch					= GATT_UUID_CHAR_USER_DESC;

const u8  spp_Statusname_switch[] 		= {'S', 't', 'a', 't', 'u', 's'};
const u8  spp_Commandname_switch[] 		= {'C', 'o', 'm', 'm', 'a', 'n', 'd'};
const u8  spp_otaname_switch[] 			= {'O', 'T', 'A'};
const u8  spp_pairname_switch[] 		= {'P', 'a', 'i', 'r'};

extern	int pairRead(void* p);
extern	int pairWrite(void* p);

attribute_t gAttributes_vendor[] = {
	{20,0,0,0,0,0},	//

	// gatt
	{7,2,2,2,(u8*)(&primaryServiceUUID_switch), 	(u8*)(&gapServiceUUID_switch), 0},
	{0,2,1,1,(u8*)(&characterUUID_switch), 		(u8*)(&devNameCharacter_switch), 0},
	{0,2,sizeof (ble_g_devName_switch), sizeof (ble_g_devName_switch),(u8*)(&devNameUUID_switch), 			(u8*)(ble_g_devName_switch), 0},
	{0,2,1,1,(u8*)(&characterUUID_switch), 		(u8*)(&appearanceCharacter_switch), 0},
	{0,2,sizeof (appearance_switch), sizeof (appearance_switch),(u8*)(&appearanceUIID_switch), 	(u8*)(&appearance_switch), 0},
	{0,2,1,1,(u8*)(&characterUUID_switch), 		(u8*)(&periConnParamChar_switch), 0},
	{0,2,sizeof (periConnParameters_switch), sizeof (periConnParameters_switch),(u8*)(&periConnParamUUID_switch), 	(u8*)(&periConnParameters_switch), 0},

	// spp
	{13,2,16,16,(u8*)(&primaryServiceUUID_switch), 	(u8*)(TelinkSppServiceUUID_switch), 0},
	{0,16,1,1,(u8*)(&characterUUID_switch), 		(u8*)(&SppDataServer2ClientProp_switch), 0},				//prop
	{0,16,1,1,(u8*)(TelinkSppDataServer2ClientUUID_switch), 	(u8*)(SppDataServer2ClientData_switch), 0},	//value
	{0,2,sizeof (spp_Statusname_switch), sizeof (spp_Statusname_switch),(u8*)(&userdesc_UUID_switch), (u8*)(spp_Statusname_switch), 0},

	{0,16,1,1,(u8*)(&characterUUID_switch), 		(u8*)(&SppDataClient2ServerProp_switch), 0},				//prop
	{0,16,16,16,(u8*)(TelinkSppDataClient2ServiceUUID_switch), 	(u8*)(SppDataClient2ServerData), 0},//value
	{0,2,sizeof (spp_Commandname_switch), sizeof (spp_Commandname_switch),(u8*)(&userdesc_UUID_switch), (u8*)(spp_Commandname_switch), 0},

	{0,16,1,1,(u8*)(&characterUUID_switch), 		(u8*)(&SppDataOtaProp_switch), 0},				//prop
	{0,16,16,16,(u8*)(TelinkSppDataOtaUUID_switch),	(SppDataOtaData_switch), 0},//value
	{0,2,sizeof (spp_otaname_switch), sizeof (spp_otaname_switch),(u8*)(&userdesc_UUID_switch), (u8*)(spp_otaname_switch), 0},

	{0,16,1,1,(u8*)(&characterUUID_switch), 		(u8*)(&SppDataPairProp_switch), 0},				//prop
	{0,16,16,16,(u8*)(TelinkSppDataPairUUID_switch),	(SppDataPairData_switch), &pairWrite, &pairRead},//value
	{0,2,sizeof (spp_pairname_switch), sizeof (spp_pairname_switch),(u8*)(&userdesc_UUID_switch), (u8*)(spp_pairname_switch), 0},
};

