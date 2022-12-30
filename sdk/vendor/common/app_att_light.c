/********************************************************************************************************
 * @file     app_att_light.c 
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
#include "../../proj_lib/ble_ll/blueLight.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "dual_mode_adapt.h"

/**********************************************************************
 * LOCAL VARIABLES 
 */
const u16 primaryServiceUUID = GATT_UUID_PRIMARY_SERVICE;
static const u16 characterUUID = GATT_UUID_CHARACTER;

const u16 gapServiceUUID = 0x1800;
const u16 devNameUUID = GATT_UUID_DEVICE_NAME;
const u16 appearanceUIID = 0x2a01;
const u16 periConnParamUUID = 0x2a04;

const u16 devInfoUUID = SERVICE_UUID_DEVICE_INFORMATION;
const u16 clientCharacterCfgUUID = GATT_UUID_CLIENT_CHAR_CFG;

u16 fwRevision_charUUID     = CHARACTERISTIC_UUID_FW_REVISION_STRING;
u16 manuNameString_charUUID = CHARACTERISTIC_UUID_MANU_NAME_STRING;
u16 modelId_charUUID        = CHARACTERISTIC_UUID_MODEL_NUM_STRING;
u16 hwRevision_charUUID     = CHARACTERISTIC_UUID_HW_REVISION_STRING;

// Device Name Characteristic Properties
static const u8 devNameCharacter = CHAR_PROP_READ;

// Appearance Characteristic Properties
static const u8 appearanceCharacter = CHAR_PROP_READ;
u16 appearance = GAP_APPEARE_UNKNOWN;

// Peripheral Preferred Connection Parameters Characteristic Properties
static const u8 periConnParamChar = CHAR_PROP_READ;

static const u8 fwRevisionChar = CHAR_PROP_READ;
u8 fwRevision_value[16] = {0};

static const u8 manuNameStringChar = CHAR_PROP_READ;
const u8 manuNameString_value[] = MESH_NAME;

static const u8 modelIdChar = CHAR_PROP_READ;
const u8 modelId_value[] = "model id 123";

static const u8 hwRevisionChar = CHAR_PROP_READ;
u32 hwRevision_value = 0x22222222;

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

gap_periConnectParams_t periConnParameters = {20, 40, 0, 1000};

u8	ble_g_devName [] = {DEVICE_NAME, 0};

//////////////////////// SPP /////////////////////////////////////////////////////
u8 TelinkSppServiceUUID[16]				= TELINK_SPP_UUID_SERVICE;

u8 TelinkSppDataServer2ClientUUID[16]	= TELINK_SPP_DATA_SERVER2CLIENT;
u8 TelinkSppDataClient2ServiceUUID[16]  = TELINK_SPP_DATA_CLIENT2SERVER;
u8 TelinkSppDataOtaUUID[16]				= TELINK_SPP_DATA_OTA;
u8 TelinkSppDataPairUUID[16]            = TELINK_SPP_DATA_PAIR;

// Spp data from Server to Client characteristic variables
static const u8 SppDataServer2ClientProp	= CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_NOTIFY;

// Spp data from Client to Server characteristic variables
static const u8 SppDataClient2ServerProp	= CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RSP;

// Spp data for OTA characteristic variables
static const u8 SppDataOtaProp			= CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP;
u8 SppDataOtaData[20] = {0xe0};

u8 SppDataServer2ClientData[4] = {0x0};
u8 status_ccc[2]={0x01,0x00};

extern u8 		pair_login_ok;

void mesh_report_status_enable(u8 mask);
void mesh_report_status_enable_mask(u8 *val, u16 len);

int meshStatusWrite(void *pw)
{
    if(!pair_login_ok){
        return 1;
    }
    
	rf_packet_att_write_t *p = pw;
	*SppDataServer2ClientData = p->value;
	if(p->l2capLen > (3 + 1)){
	    mesh_report_status_enable_mask (&(p->value), p->l2capLen - 3);
	}else{
	    mesh_report_status_enable (p->value);
	}
	return 1;
}

// Spp data for OTA characteristic variables
static const u8 SppDataPairProp			= CHAR_PROP_READ | CHAR_PROP_WRITE;
u8 SppDataPairData[20] = {0xe0};

extern u8 send_to_master[16];
#define		SppDataClient2ServerData		send_to_master

const u16 userdesc_UUID		= GATT_UUID_CHAR_USER_DESC;

const u8  spp_Statusname[] = {'S', 't', 'a', 't', 'u', 's'};
const u8  spp_Commandname[] = {'C', 'o', 'm', 'm', 'a', 'n', 'd'};
const u8  spp_otaname[] = {'O', 'T', 'A'};
const u8  spp_pairname[] = {'P', 'a', 'i', 'r'};
const u8  spp_devicename[] = {'D', 'e', 'v', 'N', 'a', 'm', 'e'};

extern	int pairRead(void* p);
extern	int pairWrite(void* p);

#define	VENDOR_ADD_SERVICE				0
#if VENDOR_ADD_SERVICE
const	u8 	DemoServiceUUID[16]			= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x20,0x19};
const	u8	DemoDataClient2ServiceUUID[16]	= {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x21,0x19};
static	const u8 	DemoDataClient2ServerProp	= CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_NOTIFY;;
const	u8  spp_demo_name[] = {'d', 'e', 'm', 'o'};
u8 spp_demo_ccc[2]={0x01,0x00};

extern int pair_dec_packet (u8 *ps);

int demo_write_callback (void *ph)
{
	static u8 demo_data_w[20] = {0};    // MAX 20
    rf_packet_att_write_t *p = (rf_packet_att_write_t *)ph;

	if (!pair_login_ok || !pair_dec_packet (ph))
	{
		return 0;
	}

	memset((void *)demo_data_w, 0, sizeof(demo_data_w));
	memcpy((void *)demo_data_w, (void *)(&(p->value)), p->l2capLen - 3);
	
	return 1;
}

u8 demo_data_r[16] = {0xd0};
#endif

attribute_t gAttributes_def[] = {
	{28 + (DUAL_MODE_ADAPT_EN ? (9+4) : 0) + (VENDOR_ADD_SERVICE ? 5 : 0),0,0,0,0,0},	//

	// gatt
	{6,2,2,2,(u8*)(&primaryServiceUUID), 	(u8*)(&gapServiceUUID), 0},
	
	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&devNameCharacter), 0},
	{0,2,sizeof (ble_g_devName), sizeof (ble_g_devName),(u8*)(&devNameUUID), 			(u8*)(ble_g_devName), 0},
    {0,2,sizeof (spp_devicename), sizeof (spp_devicename),(u8*)(&userdesc_UUID), (u8*)(spp_devicename), 0},

	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&appearanceCharacter), 0},
	{0,2,sizeof (appearance), sizeof (appearance),(u8*)(&appearanceUIID), 	(u8*)(&appearance), 0},

    //device info
    {9,2,2,2,(u8*)(&primaryServiceUUID),  (u8*)(&devInfoUUID), 0},
    
    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&fwRevisionChar), 0},
    {0,2,12, 12, (u8*)(&fwRevision_charUUID), fwRevision_value, 0},

    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&manuNameStringChar), 0},
    {0,2,sizeof (manuNameString_value), sizeof (manuNameString_value), (u8*)(&manuNameString_charUUID),    (u8*)manuNameString_value, 0},
        
    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&modelIdChar), 0},
    {0,2,sizeof (modelId_value), sizeof (modelId_value), (u8*)(&modelId_charUUID),    (u8*)(modelId_value), 0},

    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&hwRevisionChar), 0},
    {0,2,sizeof (hwRevision_value), sizeof (hwRevision_value), (u8*)(&hwRevision_charUUID),    (u8*)(&hwRevision_value), 0},
    
	// spp //0x10
    {13,2,16,16,(u8*)(&primaryServiceUUID),     (u8*)(TelinkSppServiceUUID), 0},
    
    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&SppDataServer2ClientProp), 0},               //prop
    {0,16,1,1,(u8*)(TelinkSppDataServer2ClientUUID),    (u8*)(SppDataServer2ClientData), &meshStatusWrite},    //value
    #if 0 // for compatibility with Tool, gateway.
    {0,2,sizeof (spp_Statusname), sizeof (spp_Statusname),(u8*)(&userdesc_UUID), (u8*)(spp_Statusname), 0},
    #else
    {0,2,sizeof(status_ccc),sizeof(status_ccc),(u8*)(&clientCharacterCfgUUID),     (u8*)(status_ccc), 0,0}, /*value, CCC is must for some third-party APP if there is notify or indication*/
    #endif

    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&SppDataClient2ServerProp), 0},               //prop
    {0,16,16,16,(u8*)(TelinkSppDataClient2ServiceUUID),     (u8*)(SppDataClient2ServerData), 0},//value
    {0,2,sizeof (spp_Commandname), sizeof (spp_Commandname),(u8*)(&userdesc_UUID), (u8*)(spp_Commandname), 0},

    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&SppDataOtaProp), 0},             //prop
    {0,16,16,16,(u8*)(TelinkSppDataOtaUUID),    (SppDataOtaData), 0},//value
    {0,2,sizeof (spp_otaname), sizeof (spp_otaname),(u8*)(&userdesc_UUID), (u8*)(spp_otaname), 0},

    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&SppDataPairProp), 0},                //prop
    {0,16,16,16,(u8*)(TelinkSppDataPairUUID),   (SppDataPairData), &pairWrite, &pairRead},//value
    {0,2,sizeof (spp_pairname), sizeof (spp_pairname),(u8*)(&userdesc_UUID), (u8*)(spp_pairname), 0},

#if DUAL_MODE_ADAPT_EN  // don't add servcie before, because handle of my_pb_gattUUID must be sync with SIG mesh
	// PB-GATT // 0x1d
	{9,2,2,2,(u8*)(&primaryServiceUUID),	(u8*)(&my_pb_gattUUID), 0},
    {0,2,1,1,(u8*)(&characterUUID),      (u8*)(&my_pb_gatt_out_prop), 0}, /*prop*/
    {0,2,sizeof(my_pb_gattOutData),sizeof(my_pb_gattOutData),(u8*)(&my_pb_gatt_out_UUID),    (my_pb_gattOutData), 0, 0}, /*value*/
    {0,2,sizeof (my_pb_gattOutName),sizeof (my_pb_gattOutName),(u8*)(&userdesc_UUID), (u8*)(my_pb_gattOutName), 0},
    {0,2,sizeof(provision_Out_ccc),sizeof(provision_Out_ccc),(u8*)(&clientCharacterCfgUUID),    (u8*)(provision_Out_ccc), 0,0}, /*value*/   \

    {0,2,1,1,(u8*)(&characterUUID),		(u8*)(&my_pb_gatt_in_prop), 0}, 			//prop
    {0,2,sizeof(my_pb_gattInData),sizeof(my_pb_gattInData),(u8*)(&my_pb_gatt_in_UUID),	(my_pb_gattInData), &pb_gatt_Write, 0}, 		//value
    {0,2,sizeof (my_pb_gattInName),sizeof (my_pb_gattInName),(u8*)(&userdesc_UUID), (u8*)(my_pb_gattInName), 0},
    {0,2,sizeof(provision_In_ccc),sizeof(provision_In_ccc),(u8*)(&clientCharacterCfgUUID),     (u8*)(provision_In_ccc), 0,0}, /*value*/

    // 0x26--0x29
	{4,2,2,2,(u8*)(&primaryServiceUUID),	(u8*)(&mi_gerneric_service), 0},
	{0,2,1,1,(u8*)(&characterUUID), 	(u8*)(&mi_service_change_prop), 0}, 			//prop
	{0,2,sizeof(mi_service_change_buf),sizeof(mi_service_change_buf),(u8*)(&mi_service_change_uuid), (mi_service_change_buf), 0, 0}, 		//value
	{0,2,sizeof(mi_service_change_str),sizeof(mi_service_change_str),(u8*)(&userdesc_UUID), (u8*)(mi_service_change_str), 0},	
#endif
#if VENDOR_ADD_SERVICE
    //add service ------------- demo code
	{5,2,16,16,(u8*)(&primaryServiceUUID), 	(u8*)(DemoServiceUUID), 0},
	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&DemoDataClient2ServerProp), 0},				                                          //prop
	{0,16,16,16,(u8*)(DemoDataClient2ServiceUUID), (demo_data_r), &demo_write_callback, 0},//value
	{0,2,sizeof (spp_demo_name), sizeof (spp_demo_name),(u8*)(&userdesc_UUID), (u8*)(spp_demo_name), 0},
    {0,2,sizeof(spp_demo_ccc),sizeof(spp_demo_ccc),(u8*)(&clientCharacterCfgUUID),     (u8*)(spp_demo_ccc), 0,0}, /*value, CCC is must for some third-party APP if there is notify or indication*/
#endif
};

