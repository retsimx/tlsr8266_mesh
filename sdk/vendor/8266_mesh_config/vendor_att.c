/********************************************************************************************************
 * @file     vendor_att.c 
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
const u16 primaryServiceUUID_light = GATT_UUID_PRIMARY_SERVICE;
static const u16 characterUUID_light = GATT_UUID_CHARACTER;

const u16 gapServiceUUID_light = 0x1800;
const u16 devNameUUID_light = GATT_UUID_DEVICE_NAME;
const u16 appearanceUIID_light = 0x2a01;
const u16 periConnParamUUID_light = 0x2a04;

// Device Name Characteristic Properties
static const u8 devNameCharacter_light = CHAR_PROP_READ;

// Appearance Characteristic Properties
static const u8 appearanceCharacter_light = CHAR_PROP_READ;

// Peripheral Preferred Connection Parameters Characteristic Properties
static const u8 periConnParamChar_light = CHAR_PROP_READ;

u16 appearance_light = GAP_APPEARE_UNKNOWN;

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

gap_periConnectParams_t periConnParameters_light = {20, 40, 0, 1000};

u8	ble_g_devName_light[] = {DEVICE_NAME, 0};

//////////////////////// SPP /////////////////////////////////////////////////////
u8 TelinkSppServiceUUID_light[16]				= TELINK_SPP_UUID_SERVICE;

u8 TelinkSppDataServer2ClientUUID_light[16]		= TELINK_SPP_DATA_SERVER2CLIENT;
u8 TelinkSppDataClient2ServiceUUID_light[16]	= TELINK_SPP_DATA_CLIENT2SERVER;
u8 TelinkSppDataOtaUUID_light[16]				= TELINK_SPP_DATA_OTA;
u8 TelinkSppDataPairUUID_light[16]				= TELINK_SPP_DATA_PAIR;

// Spp data from Server to Client characteristic variables
static const u8 SppDataServer2ClientProp_light		= CHAR_PROP_READ | CHAR_PROP_NOTIFY;

// Spp data from Client to Server characteristic variables
static const u8 SppDataClient2ServerProp_light		= CHAR_PROP_READ | CHAR_PROP_WRITE;

// Spp data for OTA characteristic variables
static const u8 SppDataOtaProp_light					= CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP;
u8 SppDataOtaData_light[20] = {0xe0};

u8 SppDataServer2ClientData_light[20] 			= {0xf0};

// Spp data for OTA characteristic variables
static const u8 SppDataPairProp_light					= CHAR_PROP_READ | CHAR_PROP_WRITE;
u8 SppDataPairData_light[20] = {0xe0};

extern u8 send_to_master[16];
#define		SppDataClient2ServerData			send_to_master

const u16 userdesc_UUID_light					= GATT_UUID_CHAR_USER_DESC;

const u8  spp_Statusname_light[] 		= {'S', 't', 'a', 't', 'u', 's'};
const u8  spp_Commandname_light[] 		= {'C', 'o', 'm', 'm', 'a', 'n', 'd'};
const u8  spp_otaname_light[] 			= {'O', 'T', 'A'};
const u8  spp_pairname_light[] 			= {'P', 'a', 'i', 'r'};

extern	int pairRead(void* p);
extern	int pairWrite(void* p);

#if 1
//////////////////////// device /////////////////////////////////////////////////////
const	u8 	DeviceServiceUUID[16]		= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x30,0x19};

const	u8	DeviceMacAddrUUID[16]       = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x31,0x19};
const	u8	DeviceNameUUID[16]          = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x32,0x19};
const	u8	DeviceVersionUUID[16]       = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x33,0x19};
const	u8	DeviceTypeUUID[16]          = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x34,0x19};

static	const u8 	DeviceMacAddrProp	        = CHAR_PROP_READ;
static	const u8 	DeviceNameProp	            = CHAR_PROP_READ;
static	const u8 	DeviceVersionProp	        = CHAR_PROP_READ;
static	const u8 	DeviceTypeProp	            = CHAR_PROP_READ;

const	u8  device_mac_addr_name[]      = {'M', 'a', 'c', ' ', 'a', 'd', 'd', 'r'};
const	u8  device_name[]               = {'N', 'a', 'm', 'e'};
const	u8  device_version_name[]       = {'V', 'e', 'r', 's', 'i', 'o', 'n'};
const	u8  device_type_name[]          = {'T', 'y', 'p', 'e'};

extern  u8 	mac_id[6];

#define DeviceMacAddrData       mac_id
#define DeviceNameData          ble_g_devName_light
u8 DeviceVersionData[2]         = {0};
u16 DeviceTypeData              = 0;

void software_version(){
    memcpy(DeviceVersionData, (void *)0x02, 2);
}

//////////////////////// config /////////////////////////////////////////////////////
const	u8 	ConfigServiceUUID[16]		= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x40,0x19};

const	u8	ConfigSceneIDUUID[16]       = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x41,0x19};
const	u8	ConfigRelayTimesUUID[16]    = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x42,0x19};
const	u8	ConfigOthersUUID[16]        = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x43,0x19};

static	const u8 	ConfigSceneIDProp	        = CHAR_PROP_READ;
static	const u8 	ConfigRelayTimesProp	    = CHAR_PROP_READ;
static	const u8 	ConfigOthersProp	        = CHAR_PROP_READ;

const	u8  config_scene_id_name[]      = {'S', 'c', 'e', 'n', 'e', ' ', 'i', 'd'};
const	u8  config_relay_times_name[]   = {'R', 'e', 'l', 'a', 'y', ' ', 't', 'i', 'm', 'e', 's'};
const	u8  config_others_name[]        = {'O', 't', 'h', 'e', 'r', 's'};

extern u8 max_relay_num;

u16 ConfigSceneIDData           = 0;
#define ConfigRelayTimesData    max_relay_num
u16 ConfigOthersData            = 0;

//////////////////////// control /////////////////////////////////////////////////////
const	u8 	ControlServiceUUID[16]		= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x50,0x19};

const	u8	ControlOnOffUUID[16]        = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x51,0x19};
const	u8	ControlDimLevelUUID[16]     = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x52,0x19};
const	u8	ControlCCTUUID[16]          = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x53,0x19};
const	u8	ControlRGBUUID[16]          = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x54,0x19};

static	const u8 	ControlOnOffProp	        = CHAR_PROP_READ;
static	const u8 	ControlDimLevelProp         = CHAR_PROP_READ;
static	const u8 	ControlCCTProp	            = CHAR_PROP_READ;
static	const u8 	ControlRGBProp	            = CHAR_PROP_READ;

const	u8  control_on_off_name[]       = {'O', 'n', ' ', 'O', 'f', 'f'};
const	u8  control_dim_level_name[]    = {'D', 'i', 'm', ' ', 'l', 'e', 'v', 'e', 'l'};
const	u8  control_cct_name[]          = {'C', 'C', 'T'};
const	u8  control_rgb_name[]          = {'R', 'G', 'B'};

extern  u8 	light_off;
extern  u8	led_val[6];

#define ControlOnOffData        light_off
u8 ControlDimLevelData          = 0;
u16 ControlCCTData              = 0;
#define ControlRGBData          led_val

//////////////////////// connect /////////////////////////////////////////////////////
const	u8 	ConnectServiceUUID[16]		= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x60,0x19};

const	u8	ConnectMeshNameUUID[16]     = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x61,0x19};

static	const u8 	ConnectMeshNameProp	        = CHAR_PROP_READ;

const	u8  connect_mesh_name[]         = {'M', 'e', 's', 'h', ' ', 'n', 'a', 'm', 'e'};

extern  u8	pair_config_mesh_name[];

#define ConnectMeshNameData     pair_config_mesh_name

#endif

attribute_t gAttributes_vendor[] = {
	{60,0,0,0,0,0},	//

	// gatt
	{7,2,2,2,(u8*)(&primaryServiceUUID_light), 	(u8*)(&gapServiceUUID_light), 0},
	{0,2,1,1,(u8*)(&characterUUID_light), 		(u8*)(&devNameCharacter_light), 0},
	{0,2,sizeof (ble_g_devName_light), sizeof (ble_g_devName_light),(u8*)(&devNameUUID_light), 			(u8*)(ble_g_devName_light), 0},
	{0,2,1,1,(u8*)(&characterUUID_light), 		(u8*)(&appearanceCharacter_light), 0},
	{0,2,sizeof (appearance_light), sizeof (appearance_light),(u8*)(&appearanceUIID_light), 	(u8*)(&appearance_light), 0},
	{0,2,1,1,(u8*)(&characterUUID_light), 		(u8*)(&periConnParamChar_light), 0},
	{0,2,sizeof (periConnParameters_light), sizeof (periConnParameters_light),(u8*)(&periConnParamUUID_light), 	(u8*)(&periConnParameters_light), 0},

	// spp
	{13,2,16,16,(u8*)(&primaryServiceUUID_light), 	(u8*)(TelinkSppServiceUUID_light), 0},
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&SppDataServer2ClientProp_light), 0},				//prop
	{0,16,1,1,(u8*)(TelinkSppDataServer2ClientUUID_light), 	(u8*)(SppDataServer2ClientData_light), 0},	//value
	{0,2,sizeof (spp_Statusname_light), sizeof (spp_Statusname_light),(u8*)(&userdesc_UUID_light), (u8*)(spp_Statusname_light), 0},

	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&SppDataClient2ServerProp_light), 0},				//prop
	{0,16,16,16,(u8*)(TelinkSppDataClient2ServiceUUID_light), 	(u8*)(SppDataClient2ServerData), 0},//value
	{0,2,sizeof (spp_Commandname_light), sizeof (spp_Commandname_light),(u8*)(&userdesc_UUID_light), (u8*)(spp_Commandname_light), 0},

	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&SppDataOtaProp_light), 0},				//prop
	{0,16,16,16,(u8*)(TelinkSppDataOtaUUID_light),	(SppDataOtaData_light), 0},//value
	{0,2,sizeof (spp_otaname_light), sizeof (spp_otaname_light),(u8*)(&userdesc_UUID_light), (u8*)(spp_otaname_light), 0},

	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&SppDataPairProp_light), 0},				//prop
	{0,16,16,16,(u8*)(TelinkSppDataPairUUID_light),	(SppDataPairData_light), &pairWrite, &pairRead},//value
	{0,2,sizeof (spp_pairname_light), sizeof (spp_pairname_light),(u8*)(&userdesc_UUID_light), (u8*)(spp_pairname_light), 0},
	
	// device
	{13,2,16,16,(u8*)(&primaryServiceUUID_light), 	(u8*)(DeviceServiceUUID), 0},
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&DeviceMacAddrProp), 0},				//prop
	{0,16,6,6,(u8*)(DeviceMacAddrUUID), 	(u8*)(DeviceMacAddrData), 0},	//value
	{0,2,sizeof (device_mac_addr_name), sizeof (device_mac_addr_name),(u8*)(&userdesc_UUID_light), (u8*)(device_mac_addr_name), 0},
	
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&DeviceNameProp), 0},				//prop
	{0,16,sizeof(DeviceNameData),sizeof(DeviceNameData),(u8*)(DeviceNameUUID), 	(u8*)(DeviceNameData), 0},	//value
	{0,2,sizeof (device_name), sizeof (device_name),(u8*)(&userdesc_UUID_light), (u8*)(device_name), 0},
	
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&DeviceVersionProp), 0},				//prop
	{0,16,2,2,(u8*)(DeviceVersionUUID), 	(u8*)(DeviceVersionData), 0},	//value
	{0,2,sizeof (device_version_name), sizeof (device_version_name),(u8*)(&userdesc_UUID_light), (u8*)(device_version_name), 0},
	
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&DeviceTypeProp), 0},				//prop
	{0,16,2,2,(u8*)(DeviceTypeUUID), 	(u8*)(&DeviceTypeData), 0},	//value
	{0,2,sizeof (device_type_name), sizeof (device_type_name),(u8*)(&userdesc_UUID_light), (u8*)(device_type_name), 0},
	
	// config
	{10,2,16,16,(u8*)(&primaryServiceUUID_light), 	(u8*)(ConfigServiceUUID), 0},
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&ConfigSceneIDProp), 0},				//prop
	{0,16,2,2,(u8*)(ConfigSceneIDUUID), 	(u8*)(&ConfigSceneIDData), 0},	//value
	{0,2,sizeof (config_scene_id_name), sizeof (config_scene_id_name),(u8*)(&userdesc_UUID_light), (u8*)(config_scene_id_name), 0},
	
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&ConfigRelayTimesProp), 0},				//prop
	{0,16,1,1,(u8*)(ConfigRelayTimesUUID), 	(u8*)(&ConfigRelayTimesData), 0},	//value
	{0,2,sizeof (config_relay_times_name), sizeof (config_relay_times_name),(u8*)(&userdesc_UUID_light), (u8*)(config_relay_times_name), 0},
	
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&ConfigOthersProp), 0},				//prop
	{0,16,2,2,(u8*)(ConfigOthersUUID), 	(u8*)(&ConfigOthersData), 0},	//value
	{0,2,sizeof (config_others_name), sizeof (config_others_name),(u8*)(&userdesc_UUID_light), (u8*)(config_others_name), 0},

	// control
	{13,2,16,16,(u8*)(&primaryServiceUUID_light), 	(u8*)(ControlServiceUUID), 0},
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&ControlOnOffProp), 0},				//prop
	{0,16,1,1,(u8*)(ControlOnOffUUID), 	(u8*)(&ControlOnOffData), 0},	//value
	{0,2,sizeof (control_on_off_name), sizeof (control_on_off_name),(u8*)(&userdesc_UUID_light), (u8*)(control_on_off_name), 0},
	
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&ControlDimLevelProp), 0},				//prop
	{0,16,1,1,(u8*)(ControlDimLevelUUID), 	(u8*)(&ControlDimLevelData), 0},	//value
	{0,2,sizeof (control_dim_level_name), sizeof (control_dim_level_name),(u8*)(&userdesc_UUID_light), (u8*)(control_dim_level_name), 0},
	
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&ControlCCTProp), 0},				//prop
	{0,16,2,2,(u8*)(ControlCCTUUID), 	(u8*)(&ControlCCTData), 0},	//value
	{0,2,sizeof (control_cct_name), sizeof (control_cct_name),(u8*)(&userdesc_UUID_light), (u8*)(control_cct_name), 0},
	
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&ControlRGBProp), 0},				//prop
	{0,16,4,4,(u8*)(ControlRGBUUID), 	(u8*)(ControlRGBData), 0},	//value
	{0,2,sizeof (control_rgb_name), sizeof (control_rgb_name),(u8*)(&userdesc_UUID_light), (u8*)(control_rgb_name), 0},

	// connect
	{4,2,16,16,(u8*)(&primaryServiceUUID_light), 	(u8*)(ConnectServiceUUID), 0},
	{0,16,1,1,(u8*)(&characterUUID_light), 		(u8*)(&ConnectMeshNameProp), 0},				//prop
	{0,16,16,16,(u8*)(ConnectMeshNameUUID), 	(u8*)(ConnectMeshNameData), 0},	//value
	{0,2,sizeof (connect_mesh_name), sizeof (connect_mesh_name),(u8*)(&userdesc_UUID_light), (u8*)(connect_mesh_name), 0},
};

