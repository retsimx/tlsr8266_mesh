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

//////////////////////// SPP /////////////////////////////////////////////////////
//u8 TelinkSppServiceUUID[16]				= TELINK_SPP_UUID_SERVICE;
//
//u8 TelinkSppDataServer2ClientUUID[16]	= TELINK_SPP_DATA_SERVER2CLIENT;
u8 TelinkSppDataClient2ServiceUUID[16]  = TELINK_SPP_DATA_CLIENT2SERVER;
u8 TelinkSppDataOtaUUID[16]				= TELINK_SPP_DATA_OTA;
u8 TelinkSppDataPairUUID[16]            = TELINK_SPP_DATA_PAIR;

//attribute_t gAttributes_def[] = {
//	{4,0,0,0,0,0},	//
//
//	// gatt
//	{6,2,2,2,(u8*)(&primaryServiceUUID), 	(u8*)(&gapServiceUUID), 0},
//
//	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&devNameCharacter), 0},
//	{0,2,sizeof (ble_g_devName), sizeof (ble_g_devName),(u8*)(&devNameUUID), 			(u8*)(ble_g_devName), 0},
//    {0,2,sizeof (spp_devicename), sizeof (spp_devicename),(u8*)(&userdesc_UUID), (u8*)(spp_devicename), 0},
//
////	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&appearanceCharacter), 0},
////	{0,2,sizeof (appearance), sizeof (appearance),(u8*)(&appearanceUIID), 	(u8*)(&appearance), 0},
////
////    //device info
////    {9,2,2,2,(u8*)(&primaryServiceUUID),  (u8*)(&devInfoUUID), 0},
////
////    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&fwRevisionChar), 0},
////    {0,2,12, 12, (u8*)(&fwRevision_charUUID), fwRevision_value, 0},
////
////    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&manuNameStringChar), 0},
////    {0,2,sizeof (manuNameString_value), sizeof (manuNameString_value), (u8*)(&manuNameString_charUUID),    (u8*)manuNameString_value, 0},
////
////    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&modelIdChar), 0},
////    {0,2,sizeof (modelId_value), sizeof (modelId_value), (u8*)(&modelId_charUUID),    (u8*)(modelId_value), 0},
////
////    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&hwRevisionChar), 0},
////    {0,2,sizeof (hwRevision_value), sizeof (hwRevision_value), (u8*)(&hwRevision_charUUID),    (u8*)(&hwRevision_value), 0},
////
////	// spp //0x10
////    {13,2,16,16,(u8*)(&primaryServiceUUID),     (u8*)(TelinkSppServiceUUID), 0},
////
////    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&SppDataServer2ClientProp), 0},               //prop
////    {0,16,1,1,(u8*)(TelinkSppDataServer2ClientUUID),    (u8*)(SppDataServer2ClientData), &meshStatusWrite},    //value
////    #if 0 // for compatibility with Tool, gateway.
////    {0,2,sizeof (spp_Statusname), sizeof (spp_Statusname),(u8*)(&userdesc_UUID), (u8*)(spp_Statusname), 0},
////    #else
////    {0,2,sizeof(status_ccc),sizeof(status_ccc),(u8*)(&clientCharacterCfgUUID),     (u8*)(status_ccc), 0,0}, /*value, CCC is must for some third-party APP if there is notify or indication*/
////    #endif
////
////    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&SppDataClient2ServerProp), 0},               //prop
////    {0,16,16,16,(u8*)(TelinkSppDataClient2ServiceUUID),     (u8*)(SppDataClient2ServerData), 0},//value
////    {0,2,sizeof (spp_Commandname), sizeof (spp_Commandname),(u8*)(&userdesc_UUID), (u8*)(spp_Commandname), 0},
////
////    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&SppDataOtaProp), 0},             //prop
////    {0,16,16,16,(u8*)(TelinkSppDataOtaUUID),    (SppDataOtaData), 0},//value
////    {0,2,sizeof (spp_otaname), sizeof (spp_otaname),(u8*)(&userdesc_UUID), (u8*)(spp_otaname), 0},
////
////    {0,2,1,1,(u8*)(&characterUUID),         (u8*)(&SppDataPairProp), 0},                //prop
////    {0,16,16,16,(u8*)(TelinkSppDataPairUUID),   (SppDataPairData), &pairWrite, &pairRead},//value
////    {0,2,sizeof (spp_pairname), sizeof (spp_pairname),(u8*)(&userdesc_UUID), (u8*)(spp_pairname), 0},
////
////#if DUAL_MODE_ADAPT_EN  // don't add servcie before, because handle of my_pb_gattUUID must be sync with SIG mesh
////	// PB-GATT // 0x1d
////	{9,2,2,2,(u8*)(&primaryServiceUUID),	(u8*)(&my_pb_gattUUID), 0},
////    {0,2,1,1,(u8*)(&characterUUID),      (u8*)(&my_pb_gatt_out_prop), 0}, /*prop*/
////    {0,2,sizeof(my_pb_gattOutData),sizeof(my_pb_gattOutData),(u8*)(&my_pb_gatt_out_UUID),    (my_pb_gattOutData), 0, 0}, /*value*/
////    {0,2,sizeof (my_pb_gattOutName),sizeof (my_pb_gattOutName),(u8*)(&userdesc_UUID), (u8*)(my_pb_gattOutName), 0},
////    {0,2,sizeof(provision_Out_ccc),sizeof(provision_Out_ccc),(u8*)(&clientCharacterCfgUUID),    (u8*)(provision_Out_ccc), 0,0}, /*value*/   \
////
////    {0,2,1,1,(u8*)(&characterUUID),		(u8*)(&my_pb_gatt_in_prop), 0}, 			//prop
////    {0,2,sizeof(my_pb_gattInData),sizeof(my_pb_gattInData),(u8*)(&my_pb_gatt_in_UUID),	(my_pb_gattInData), &pb_gatt_Write, 0}, 		//value
////    {0,2,sizeof (my_pb_gattInName),sizeof (my_pb_gattInName),(u8*)(&userdesc_UUID), (u8*)(my_pb_gattInName), 0},
////    {0,2,sizeof(provision_In_ccc),sizeof(provision_In_ccc),(u8*)(&clientCharacterCfgUUID),     (u8*)(provision_In_ccc), 0,0}, /*value*/
////
////    // 0x26--0x29
////	{4,2,2,2,(u8*)(&primaryServiceUUID),	(u8*)(&mi_gerneric_service), 0},
////	{0,2,1,1,(u8*)(&characterUUID), 	(u8*)(&mi_service_change_prop), 0}, 			//prop
////	{0,2,sizeof(mi_service_change_buf),sizeof(mi_service_change_buf),(u8*)(&mi_service_change_uuid), (mi_service_change_buf), 0, 0}, 		//value
////	{0,2,sizeof(mi_service_change_str),sizeof(mi_service_change_str),(u8*)(&userdesc_UUID), (u8*)(mi_service_change_str), 0},
////#endif
////#if VENDOR_ADD_SERVICE
////    //add service ------------- demo code
////	{5,2,16,16,(u8*)(&primaryServiceUUID), 	(u8*)(DemoServiceUUID), 0},
////	{0,2,1,1,(u8*)(&characterUUID), 		(u8*)(&DemoDataClient2ServerProp), 0},				                                          //prop
////	{0,16,16,16,(u8*)(DemoDataClient2ServiceUUID), (demo_data_r), &demo_write_callback, 0},//value
////	{0,2,sizeof (spp_demo_name), sizeof (spp_demo_name),(u8*)(&userdesc_UUID), (u8*)(spp_demo_name), 0},
////    {0,2,sizeof(spp_demo_ccc),sizeof(spp_demo_ccc),(u8*)(&clientCharacterCfgUUID),     (u8*)(spp_demo_ccc), 0,0}, /*value, CCC is must for some third-party APP if there is notify or indication*/
////#endif
//};
//
//STATIC_ASSERT(sizeof(gAttributes_def) == 20*5);