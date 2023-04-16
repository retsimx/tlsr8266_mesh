/********************************************************************************************************
 * @file     att.h 
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

typedef unsigned char u8 ;
typedef signed char s8;

typedef unsigned short u16;
typedef signed short s16;

typedef int s32;
typedef unsigned int u32;

typedef long long s64;
typedef unsigned long long u64;

typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed int int32_t;
typedef unsigned int uint32_t;
typedef signed long long int64_t;
typedef unsigned long long uint64_t;


#define ATT_MTU_SIZE                        L2CAP_MTU_SIZE //!< Minimum ATT MTU size
#define ATT_MAX_ATTR_HANDLE                 0xFFFF
#define ATT_16BIT_UUID_LEN                  2
#define ATT_128BIT_UUID_LEN                 16
#define L2CAP_RESERVED_LEN                  14
#define OPCODE_SIZE                         1
#define L2CAP_PAYLOAD_OFFSET                (L2CAP_RESERVED_LEN + OPCODE_SIZE)
#define ATT_HANDLE_START                    0x0001
#define ATT_HANDLE_END                      0xFFFF

#define ATT_OP_ERROR_RSP                    0x01 //!< Error Response op code
#define ATT_OP_EXCHANGE_MTU_REQ             0x02 //!< Exchange MTU Request op code
#define ATT_OP_EXCHANGE_MTU_RSP             0x03 //!< Exchange MTU Response op code
#define ATT_OP_FIND_INFO_REQ                0x04 //!< Find Information Request op code
#define ATT_OP_FIND_INFO_RSP                0x05 //!< Find Information Response op code
#define ATT_OP_FIND_BY_TYPE_VALUE_REQ       0x06 //!< Find By Type Vaue Request op code
#define ATT_OP_FIND_BY_TYPE_VALUE_RSP       0x07 //!< Find By Type Vaue Response op code
#define ATT_OP_READ_BY_TYPE_REQ             0x08 //!< Read By Type Request op code
#define ATT_OP_READ_BY_TYPE_RSP             0x09 //!< Read By Type Response op code
#define ATT_OP_READ_REQ                     0x0a //!< Read Request op code
#define ATT_OP_READ_RSP                     0x0b //!< Read Response op code
#define ATT_OP_READ_BLOB_REQ                0x0c //!< Read Blob Request op code
#define ATT_OP_READ_BLOB_RSP                0x0d //!< Read Blob Response op code
#define ATT_OP_READ_MULTI_REQ               0x0e //!< Read Multiple Request op code
#define ATT_OP_READ_MULTI_RSP               0x0f //!< Read Multiple Response op code
#define ATT_OP_READ_BY_GROUP_TYPE_REQ       0x10 //!< Read By Group Type Request op code
#define ATT_OP_READ_BY_GROUP_TYPE_RSP       0x11 //!< Read By Group Type Response op code
#define ATT_OP_WRITE_REQ                    0x12 //!< Write Request op code
#define ATT_OP_WRITE_RSP                    0x13 //!< Write Response op code
#define ATT_OP_PREPARE_WRITE_REQ            0x16 //!< Prepare Write Request op code
#define ATT_OP_PREPARE_WRITE_RSP            0x17 //!< Prepare Write Response op code
#define ATT_OP_EXECUTE_WRITE_REQ            0x18 //!< Execute Write Request op code
#define ATT_OP_EXECUTE_WRITE_RSP            0x19 //!< Execute Write Response op code
#define ATT_OP_HANDLE_VALUE_NOTI            0x1b //!< Handle Value Notification op code
#define ATT_OP_HANDLE_VALUE_IND             0x1d //!< Handle Value Indication op code
#define ATT_OP_HANDLE_VALUE_CFM             0x1e //!< Handle Value Confirmation op code
#define ATT_OP_WRITE_CMD                    0x52 //!< ATT Write Command

/** @defgroup ATT_PERMISSIONS_BITMAPS GAP ATT Attribute Access Permissions Bit Fields
 * @{
 */

#define ATT_PERMISSIONS_READ                 0x01 //!< Attribute is Readable
#define ATT_PERMISSIONS_WRITE                0x02 //!< Attribute is Writable
#define ATT_PERMISSIONS_AUTHEN_READ          0x04 //!< Read requires Authentication
#define ATT_PERMISSIONS_AUTHEN_WRITE         0x08 //!< Write requires Authentication
#define ATT_PERMISSIONS_AUTHOR_READ          0x10 //!< Read requires Authorization
#define ATT_PERMISSIONS_AUTHOR_WRITE         0x20 //!< Write requires Authorization

/** @} End GAP_ATT_PERMISSIONS_BITMAPS */


typedef struct
{
  u8 len;                    //!< Length of UUID
  u8 uuid[16];               //!< UUID
} uuid_t;

/**
 * Error Response 
 */
typedef struct
{
    u8 reqOpcodeInErr; //!< The request that generated this error response
    u8 errCode;   //!< The reason why the request has generated an error response
    u16 attHandleInErr;   //!< The attribute handle that generated this error response
} errorRsp_t;

/**
 * Exchange MTU Request 
 */
typedef struct
{
    u16 clientRxMTU; //!< Attribute client receive MTU size 
} exchangeMTUReq_t;

/**
 * Exchange MTU Response 
 */
typedef struct
{
    u16 serverRxMTU; //!< Attribute server receive MTU size
} exchangeMTURsp_t;

/**
 * Find Information Request 
 */
typedef struct
{
    u16 startingHandle;       //!< First requested handle number
    u16 endingHandle;         //!< Last requested handle number
} findInformationReq_t;

/**
 * Handle(s) and 16-bit Bluetooth UUID(s)
 */
typedef struct
{
    u16 handle;                //!< Handle
    u8 uuid[ATT_16BIT_UUID_LEN]; //!< 16 bit UUID
} handleBtUUID_t;

/**
 * Handle(s) and 128-bit UUID(s)
 */
typedef struct
{
    u16 handle;             //!< Handle
    u8 uuid[ATT_128BIT_UUID_LEN]; //!< 128 bit UUID
} handleUUID_t;

/**
 * Find Information Response 
 */
typedef struct
{
    u8 format;       //!< Format of information
    u8 infoNum;      //!< information num
    u8 info[ATT_MTU_SIZE - 2];      //!< information
} findInformationRsp_t;

/**
 * Find By Type Value Request 
 */
typedef struct
{
  u16  startingHandle;          //!< First requested handle number 
  u16  endingHandle;            //!< Last requested handle number
  u16  uuid;                    //!< UUID to find
  u8   len;                     //!< Length of value
  u8   value[ATT_MTU_SIZE - 7];   //!< Attribute value to find
} findByTypeValueReq_t;

/**
 * Handles Infomation list element
 */
typedef struct
{
  u8 handle;         //!< Found attribute handle
  u8 groupEndHandle; //!< Group end handle
} handleInfo_t;

/**
 * Find By Type Value Response 
 */
typedef struct
{
  u8 handleInfoNum;                                       //!< Number of handles information below
  handleInfo_t handleInfo[1] ; //!< A list of 1 or more Handle Informations
} findByTypeValueRsp_t;

/**
 * Read By Type Request 
 */
typedef struct
{
  u16 startingHandle; //!< First requested handle number
  u16 endingHandle;   //!< Last requested handle number
  uuid_t attrType;    //!< 2 or 16 octet UUID
} readByTypeReq_t;

/**
 * Read By Type Response 
 */
typedef struct
{
  u8 numData;                  //!< Number of attribute data list item
  u8 len;                      //!< The size of each attribute handle-value pair
  u8 data[ATT_MTU_SIZE-2];     //!< Attribute Data List
} readByTypeRsp_t;

/**
 * Read Request 
 */
typedef struct
{
  u16 handle;               //!< The handle of the attribute to be read
} readReq_t;

/**
 * Read Response 
 */
typedef struct
{
  u8 len;                   //!< Length of value
  u8 attrValue[ATT_MTU_SIZE - 1];          //!< Value of the attribute with the handle given
} readRsp_t;

/**
 * Read Blob Req 
 */
typedef struct
{
  u16 handle; //!< The handle of the attribute to be read
  u16 offset; //!< The offset of the first octet to be read
} readBlobReq_t;

/**
 * Read Blob Response 
 */
typedef struct
{
  u8 len;      //!< Length of value
  u8 attrValue[ATT_MTU_SIZE - 1]; //!< Part of the value of the attribute with the handle given
} readBlobRsp_t;

/**
 * Read Multiple Request
 */
typedef struct
{
  u8  numHandles; //!< Number of attribute handles
  u16 handle[1];  //!< A set of two or more attribute handles
} readMultipleReq_t;

/**
 * Read Multiple Response 
 */
typedef struct
{
  u8 len;       //!< Length of values
  u8 values[ATT_MTU_SIZE - 1]; //!< A set of two or more values
} readMultiRsp_t;

/**
 * Read By Group Type Request 
 */
typedef struct
{
  u16 startingHandle; //!< First requested handle number (must be first field)
  u16 endingHandle;   //!< Last requested handle number
  uuid_t attrType;     //!< 2 or 16 octet UUID
} readByGroupTypeReq_t;

/**
 * Read By Group Type Response 
 */
typedef struct
{
  u8 grpNum;                  //!< The number of attributes in this group
  u8 len;                      //!< Length of each attribute handle
  u8 data[ATT_MTU_SIZE - 2];                 //!< Attribute Data
} readByGroupTypeRsp_t;

/**
 * Write Request 
 */
typedef struct
{
  u16 handle;                            //!< The handle of the attribute to be written (must be first field)
  u8 len;                                //!< Length of value
  u8 value[ATT_MTU_SIZE - 3];            //!< The value to be written to the attribute
} writeReq_t;

/**
 * Write Command 
 */
typedef struct
{
  u16 handle;                         //!< The handle of the attribute to be written (must be first field)
  u8 len;                             //!< Length of value
  u8 value[ATT_MTU_SIZE - 3];         //!< The value to be written to the attribute
  u8 sig;                             //!< the sig flag
} writeCmd_t;

/**
 * Prepare Write Request 
 */
typedef struct
{
  u16 handle;                 //!< Handle of the attribute to be written (must be first field)
  u16 offset;                 //!< Offset of the first octet to be written
  u8 len;                     //!< Length of value
  u8 value[ATT_MTU_SIZE - 5]; //!< Part of the value of the attribute to be written
} prepareWriteReq_t;

/**
 * Prepare Write Response 
 */
typedef struct
{
  u16 handle;                       //!< The handle of the attribute to be written
  u16 offset;                       //!< The offset of the first octet to be written
  u8 len;                           //!< Length of value
  u8 value[ATT_MTU_SIZE - 3];              //!< The value of the attribute to be written
} prepareWriteRsp_t;

/**
 * Execute Write Request 
 */
typedef struct
{
  u8 flags;   //!< 0x00 - cancel all prepared writes 0x01 - immediately write all pending prepared values
} executeWriteReq_t;

/**
 * Handle Value Notification 
 */
typedef struct
{
  u16 handle;               //!< The handle of the attribute
  u8 len;                   //!< Length of value
  u8 value[ATT_MTU_SIZE - 3];              //!< The current value of the attribute 
} handleValueNoti_t;

/**
 * Handle Value Indication 
 */
typedef struct
{
  u16 handle;               //!< The handle of the attribute
  u8 len;                   //!< Length of value
  u8 value[ATT_MTU_SIZE - 3];              //!< The current value of the attribute 
} handleValueInd_t;

typedef union attOpCode{
    struct{
        u8 method:6;
        u8 cmdFlag:1;
        u8 authSigFlag:1;
    }bitField;
    u8 byte;
}attOpCode_t;


typedef struct attProtocolReqPdu{
    u16 connHandle;
    u16 method;
    union reqPdu {
        exchangeMTUReq_t exchangeMTUReq;                 //!< Exchange MTU Req
        findInformationReq_t findInfoReq;                //!< Find Information Req
        findByTypeValueReq_t findByTypeValueReq;         //!< Find By Type Vaue Req
        readByTypeReq_t readByTypeReq;                   //!< Read By Type Req
        readReq_t readReq;                               //!< Read Req
        readBlobReq_t readBlobReq;                       //!< Read Blob Req
        readMultipleReq_t readMultiReq;                  //!< Read Multiple Req
        readByGroupTypeReq_t readByGrpTypeReq;             //!< Read By Group Type Req
        writeReq_t writeReq;                             //!< Write Req
        prepareWriteReq_t prepareWriteReq;               //!< Prepare Write Req
        executeWriteReq_t executeWriteReq;               //!< Execute Write Req
    }msg;
}attProtocolReqPdu_t;


typedef struct attProtocolRspPdu{
    u16 connHandle;
    u16 method;
    union rspPdu {
        errorRsp_t errorRsp;                             //!< Error Rsp
        exchangeMTURsp_t exchangeMTURsp;                 //!< Exchange MTU Rsp
        findInformationRsp_t findInfoRsp;                //!< Find Information Rsp
        findByTypeValueRsp_t findByTypeValueRsp;         //!< Find By Type Vaue Rsp
        readByTypeRsp_t readByTypeRsp;                   //!< Read By Type Rsp
        readRsp_t readRsp;                               //!< Read Rsp
        readBlobRsp_t readBlobRsp;                       //!< Read Blob Rsp
        readMultiRsp_t readMultiRsp;                     //!< Read Multiple Rsp
        readByGroupTypeRsp_t readByGrpTypeRsp;           //!< Read By Group Type Rsp
        prepareWriteRsp_t prepareWriteRsp;               //!< Prepare Write Rsp
        handleValueNoti_t handleValueNoti;               //!< Handle Value Noti
        handleValueInd_t handleValueInd;                 //!< Handle Value Ind
    }msg;
}attProtocolRspPdu_t;
