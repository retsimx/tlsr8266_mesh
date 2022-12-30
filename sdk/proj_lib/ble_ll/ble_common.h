/********************************************************************************************************
 * @file     ble_common.h 
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


/*********************************************************************
 * CONSTANTS
 */


/**
 *  @brief  Definition for BLE Address Constants
 */
#define BLE_ADDR_LEN                     6
#define BLE_ACCESS_ADDR_LEN              4

/* Device Address Type */
#define BLE_ADDR_PUBLIC                  0
#define BLE_ADDR_RANDOM                  1
#define BLE_ADDR_INVALID                 0xff

/**
 *  @brief  Definition for BLE Random Number Size
 */
#define BLE_RANDOM_NUM_SIZE              8

/**
 *  @brief  Definition for BLE Invalid connection handle value
 */
#define BLE_INVALID_CONNECTION_HANDLE    0xffff


/*********************************************************************
 * ENUMS
 */
typedef enum {
    BLE_SUCCESS = 0,
    
    // HCI Status, Per the Bluetooth Core Specification, V4.0.0, Vol. 2, Part D.
    HCI_ERR_UNKNOWN_HCI_CMD                                        = 0x01,
    HCI_ERR_UNKNOWN_CONN_ID                                        = 0x02,
    HCI_ERR_HW_FAILURE                                             = 0x03,
    HCI_ERR_PAGE_TIMEOUT                                           = 0x04,
    HCI_ERR_AUTH_FAILURE                                           = 0x05,
    HCI_ERR_PIN_KEY_MISSING                                        = 0x06,
    HCI_ERR_MEM_CAP_EXCEEDED                                       = 0x07,
    HCI_ERR_CONN_TIMEOUT                                           = 0x08,
    HCI_ERR_CONN_LIMIT_EXCEEDED                                    = 0x09,
    HCI_ERR_SYNCH_CONN_LIMIT_EXCEEDED                              = 0x0A,
    HCI_ERR_ACL_CONN_ALREADY_EXISTS                                = 0x0B,
    HCI_ERR_CMD_DISALLOWED                                         = 0x0C,
    HCI_ERR_CONN_REJ_LIMITED_RESOURCES                             = 0x0D,
    HCI_ERR_CONN_REJECTED_SECURITY_REASONS                         = 0x0E,
    HCI_ERR_CONN_REJECTED_UNACCEPTABLE_BDADDR                      = 0x0F,
    HCI_ERR_CONN_ACCEPT_TIMEOUT_EXCEEDED                           = 0x10,
    HCI_ERR_UNSUPPORTED_FEATURE_PARAM_VALUE                        = 0x11,
    HCI_ERR_INVALID_HCI_CMD_PARAMS                                 = 0x12,
    HCI_ERR_REMOTE_USER_TERM_CONN                                  = 0x13,
    HCI_ERR_REMOTE_DEVICE_TERM_CONN_LOW_RESOURCES                  = 0x14,
    HCI_ERR_REMOTE_DEVICE_TERM_CONN_POWER_OFF                      = 0x15,
    HCI_ERR_CONN_TERM_BY_LOCAL_HOST                                = 0x16,
    HCI_ERR_REPEATED_ATTEMPTS                                      = 0x17,
    HCI_ERR_PAIRING_NOT_ALLOWED                                    = 0x18,
    HCI_ERR_UNKNOWN_LMP_PDU                                        = 0x19,
    HCI_ERR_UNSUPPORTED_REMOTE_FEATURE                             = 0x1A,
    HCI_ERR_SCO_OFFSET_REJ                                         = 0x1B,
    HCI_ERR_SCO_INTERVAL_REJ                                       = 0x1C,
    HCI_ERR_SCO_AIR_MODE_REJ                                       = 0x1D,
    HCI_ERR_INVALID_LMP_PARAMS                                     = 0x1E,
    HCI_ERR_UNSPECIFIED_ERROR                                      = 0x1F,
    HCI_ERR_UNSUPPORTED_LMP_PARAM_VAL                              = 0x20,
    HCI_ERR_ROLE_CHANGE_NOT_ALLOWED                                = 0x21,
    HCI_ERR_LMP_LL_RESP_TIMEOUT                                    = 0x22,
    HCI_ERR_LMP_ERR_TRANSACTION_COLLISION                          = 0x23,
    HCI_ERR_LMP_PDU_NOT_ALLOWED                                    = 0x24,
    HCI_ERR_ENCRYPT_MODE_NOT_ACCEPTABLE                            = 0x25,
    HCI_ERR_LINK_KEY_CAN_NOT_BE_CHANGED                            = 0x26,
    HCI_ERR_REQ_QOS_NOT_SUPPORTED                                  = 0x27,
    HCI_ERR_INSTANT_PASSED                                         = 0x28,
    HCI_ERR_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED                    = 0x29,
    HCI_ERR_DIFFERENT_TRANSACTION_COLLISION                        = 0x2A,
    HCI_ERR_RESERVED1                                              = 0x2B,
    HCI_ERR_QOS_UNACCEPTABLE_PARAM                                 = 0x2C,
    HCI_ERR_QOS_REJ                                                = 0x2D,
    HCI_ERR_CHAN_ASSESSMENT_NOT_SUPPORTED                          = 0x2E,
    HCI_ERR_INSUFFICIENT_SECURITY                                  = 0x2F,
    HCI_ERR_PARAM_OUT_OF_MANDATORY_RANGE                           = 0x30,
    HCI_ERR_RESERVED2                                              = 0x31,
    HCI_ERR_ROLE_SWITCH_PENDING                                    = 0x32,
    HCI_ERR_RESERVED3                                              = 0x33,
    HCI_ERR_RESERVED_SLOT_VIOLATION                                = 0x34,
    HCI_ERR_ROLE_SWITCH_FAILED                                     = 0x35,
    HCI_ERR_EXTENDED_INQUIRY_RESP_TOO_LARGE                        = 0x36,
    HCI_ERR_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST                   = 0x37,
    HCI_ERR_HOST_BUSY_PAIRING                                      = 0x38,
    HCI_ERR_CONN_REJ_NO_SUITABLE_CHAN_FOUND                        = 0x39,
    HCI_ERR_CONTROLLER_BUSY                                        = 0x3A,
    HCI_ERR_UNACCEPTABLE_CONN_INTERVAL                             = 0x3B,
    HCI_ERR_DIRECTED_ADV_TIMEOUT                                   = 0x3C,
    HCI_ERR_CONN_TERM_MIC_FAILURE                                  = 0x3D,
    HCI_ERR_CONN_FAILED_TO_ESTABLISH                               = 0x3E,
    HCI_ERR_MAC_CONN_FAILED                                        = 0x3F,
    
    
    LL_ERR_START = 0x40,
    LL_ERR_WHITE_LIST_TABLE_FULL,                        //!< The white list table full
	LL_EER_FEATURE_NOT_SUPPORTED,
    
    
    L2CAP_ERR_START = 0x42,
    L2CAP_ERR_MUX_EXCCED,                                //!< The AUTOPEND pending all is turned on
    L2CAP_ERR_INVALID_PACKET_LEN,                        //!< The AUTOPEND pending all is turned off
    L2CAP_ERR_BEACON_LOSS,                               //!< The beacon was lost following a synchronization request
    L2CAP_ERR_CHANNEL_ACCESS_FAILURE,                    //!< The operation or data request failed because of activity on the channel
    L2CAP_ERR_DENIED,                                    //!< The l2cap was not able to enter low power mode
    L2CAP_ERR_INVALID_HANDLE,                            //!< The purge request contained an invalid handle
    L2CAP_ERR_INVALID_PARAMETER,                         //!< The API function parameter is out of range
    L2CAP_ERR_UNSUPPORTED,                               //!< The operation is not supported in the current configuration
    L2CAP_ERR_BAD_STATE,                                 //!< The operation could not be performed in the current state
    L2CAP_ERR_NO_RESOURCES,                              //!< The operation could not be completed because no memory resources were available
    L2CAP_ERR_TIME_OUT,                                  //!< The operation is time out
    L2CAP_ERR_NO_HANDLER,                                //!< No handle
    L2CAP_ERR_LEN_NOT_MATCH,                             //!< length not match
    
    
    ATT_ERR_START = 0x50,
    ATT_ERR_INVALID_HANDLE,                              //!< The attribute handle given was not valid on this server
    ATT_ERR_READ_NOT_PERMITTED,                          //!< The attribute cannot be read
    ATT_ERR_WRITE_NOT_PERMITTED,                         //!< The attribute cannot be written
    ATT_ERR_INVALID_PDU,                                 //!< The attribute PDU was invalid
    ATT_ERR_INSUFFICIENT_AUTH,                           //!< The attribute requires authentication before it can be read or written
    ATT_ERR_REQ_NOT_SUPPORTED,                           //!< Attribute server does not support the request received from the client
    ATT_ERR_INVALID_OFFSET,                              //!< Offset specified was past the end of the attribute
    ATT_ERR_INSUFFICIENT_AUTHOR,                         //!< The attribute requires authorization before it can be read or written
    ATT_ERR_PREPARE_QUEUE_FULL,                          //!< Too many prepare writes have been queued
    ATT_ERR_ATTR_NOT_FOUND,                              //!< No attribute found within the given attribute handle range
    ATT_ERR_ATTR_NOT_LONG,                               //!< The attribute cannot be read or written using the Read Blob Request
    ATT_ERR_INSUFFICIENT_KEY_SIZE,                       //!< The Encryption Key Size used for encrypting this link is insufficient
    ATT_ERR_INVALID_ATTR_VALUE_LEN,                      //!< The attribute value length is invalid for the operation
    ATT_ERR_UNLIKELY_ERR,                                //!< The attribute request that was requested has encountered an error that was unlikely, and therefore could not be completed as requested
    ATT_ERR_INSUFFICIENT_ENCRYPT,                        //!< The attribute requires encryption before it can be read or written
    ATT_ERR_UNSUPPORTED_GRP_TYPE,                        //!< The attribute type is not a supported grouping attribute as defined by a higher layer specification
    ATT_ERR_INSUFFICIENT_RESOURCES,                      //!< Insufficient Resources to complete the request
    ATT_ERR_ATTR_NUMBER_INVALID,                         //!< The attr number is 0 or too large to register
    ATT_ERR_ENQUEUE_FAILED,                              //!< register service failed when enqueue

    GAP_ERR_START = 0x64,
    GAP_ERR_INVALID_ROLE,
    GAP_ERR_MEMORY_ERROR,
    GAP_ERR_INVALID_STATE,
    GAP_ERR_INVALID_PARAMETER,
    GAP_ERR_LISTENER_FULL,
    GAP_ERR_ITEM_NOT_FOUND,

    SERVICE_ERR_START,
    SERVICE_ERR_INVALID_PARAMETER,
	SERVICE_ERR_NOTI_NOT_PERMITTED,
    
    SMP_EER_MUX_EXCCED = 0x80,                          //!< The AUTOPEND pending all is turned on 
    SMP_EER_INVALID_PACKET_LEN,                         //!< The AUTOPEND pending all is turned off 
    SMP_EER_INVALID_STATE,                              //!< received cmd in invalid state 
    SMP_EER_USER_CANCEL,                                //!< user channcel status  
    SMP_EER_SEC_FAILED,                                 //!< The l2cap was not able to enter low power mode. 
    SMP_EER_INVALID_HANDLE,                             //!< The purge request contained an invalid handle 
    SMP_EER_INVALID_PARAMETER,                          //!< The API function parameter is out of range 
    SMP_EER_UNSUPPORTED,                                //!< The operation is not supported in the current configuration 
    SMP_EER_BAD_STATE,                                  //!< The operation could not be performed in the current state 
    SMP_EER_NO_RESOURCES,                               //!< The operation could not be completed because no memory resources were available 
    SMP_EER_TIME_OUT,                                   //!< The operation is time out 
    SMP_EER_NO_HANDLER,                                 //!< The operation is time out 
    SMP_EER_LEN_NOT_MATCH,                              //!< The operation is time out 
    SMP_EER_NOT_FOUND,                                  //!< The operation is time out 
    SMP_EER_LINK_IS_ENCY,
    SMP_EER_PAIRING_IS_GOING_ON,
    SMP_EER_SIG_VERIFY_FAIL,                            //!< The operation is time out 
    SMP_EER_SIG_FAIL,                                   //!< The singature is failed
    SMP_EER_NO_SIGN_KEY,
    SMP_EER_ADDR_RESOLVE_FAIL,                          //!< The operation is time out 


    SPP_ERR_START = 0xA0,
    SPP_ERR_NO_HANDLER,
    

	BLE_COMMON_ERR_START = 0xE0,
    BLE_ERR_DUPLICATE_PACKET,
	BLE_ERR_INVALID_STATE,
	BLE_ERR_INVALID_PARAMETER,
	BLE_ERR_NO_RESOURCE,

} ble_sts_t;


/*********************************************************************
 * TYPES
 */

/**
 *  @brief  Definition for BLE Common Address Type
 */
typedef struct {
	u8 type;
	u8 address[BLE_ADDR_LEN];
} addr_t;

