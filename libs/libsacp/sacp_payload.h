/**
 * @file sacp_payload.h
 * @author Liu Chuansen (179712066@qq.com)
 * @brief 简单属性控制协议
 * @version 0.1
 * @date 2022-10-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __SACP_PAYLOAD_H__
#define __SACP_PAYLOAD_H__


#include "portable.h"

/**
 * @brief 解决VSCODE 无法解析__packed，导致无法补全代码的问题
 * 
 */
#ifdef VSCODE_EDIT
#undef __PACKED
#define __PACKED
#endif 

/**
 * @brief 协议起始标记
 * 
 */
#define SACP_SOF  0xAA  


/**
 * @brief SACP最大帧大小
 * 
 */
#ifndef CONFIG_SACP_MAX_FRAME_SIZE 
#define CONFIG_SACP_MAX_FRAME_SIZE  135
#endif 


/**
 * @brief SACP每帧最多的属性数
 * 
 */
#ifndef CONFIG_SACP_MAX_ATTRIBUTES_PER_FRAME 
#define CONFIG_SACP_MAX_ATTRIBUTES_PER_FRAME  20
#endif 



/**
 * @brief 操作码
 * 
 */
enum SACP_FRAME_OPCODE
{
    SACP_OP_READ = 0,
    SACP_OP_WRITE = 1,
    SACP_OP_READ_ACK = 2,
    SACP_OP_WRITE_ACK = 3,
    SACP_OP_REPORT = 4,
};

#define SACP_OP_CODE_NAMES "READ\0WRITE\0READ_ACK\0WRITE_ACK\0REPORT\0"
#define SACP_OP_CODE_NAME(v) stringAt(SACP_OP_CODE_NAMES, v, SACP_OP_READ)

#define isValidSacpOpCode(_op) (((_op) >= SACP_OP_READ) && ((_op) <= SACP_OP_REPORT))


enum SACP_FRAME_PRIORITY
{
    SACP_PRIORITY_START = 0,
    SACP_PRIORITY_END = 1,
    SACP_PRIORITY_NUM,

    // 定义最高和最低优先级
    SACP_PRIORITY_HIGHEST = SACP_PRIORITY_END,
    SACP_PRIORITY_LOWEST = SACP_PRIORITY_START,
    SACP_PRIORITY_MASK = SACP_PRIORITY_NUM - 1,
};

/// 目前限制只使用两个优先级
#define isValidSacpPriority(_pri) (((_pri) >= SACP_PRIORITY_START) && ((_pri) <= SACP_PRIORITY_END))

/**
 * @brief SACP支持的数据类型
 * 
 */
enum SACP_TYPE
{
    SACP_TYPE_BOOL = 0,
    SACP_TYPE_UINT8 = 1,
    SACP_TYPE_INT8 = 2,
    SACP_TYPE_UINT16 = 3,
    SACP_TYPE_INT16 = 4,
    SACP_TYPE_UINT32 = 5,
    SACP_TYPE_INT32 = 6,
    SACP_TYPE_FLOAT = 7,   
    SACP_TYPE_DOUBLE = 8, 
    SACP_TYPE_UINT64 = 9,
    SACP_TYPE_INT64 = 10,
    SACP_TYPE_OCTET = 11,

    SACP_TYPE_STATUS = 15,
};

#define SACP_TYPE_NAMES \
"bool\0" \
"uint8_t\0" \
"int8_t\0" \
"uint16_t\0" \
"int16_t\0" \
"uint32_t\0" \
"int32_t\0" \
"float\0" \
"double\0" \
"uint64_t\0" \
"int64_t\0" \
"octet\0" \
"n/a\0" \
"n/a\0" \
"n/a\0" \
"status\0" 

#define SACP_TYPE_NAME(v) stringAt(SACP_TYPE_NAMES, v, SACP_TYPE_BOOL)



/**
 * @brief SACP属性状态码
 * 
 */
enum SACP_STATUS
{
    SACP_STATUS_OK = 0,
    SACP_STATUS_NO_SUCH_ATTRIBUTE,
    SACP_STATUS_TYPE_MISMATCH,
    SACP_STATUS_NO_WRITABLE,

    SACP_STATUS_USER_BASE = 128
};



/**
 * @brief 协议头部定义
 * 
 */
struct SacpHeader
{
    /// 起始标志
    uint8_t sof;
    /// 帧序列号
    uint8_t sequence;
    /// 操作字 [2:0] OP; [7:6] 优先级0:3, 其他，保留
    uint8_t control;
    /// 数据大有
    uint8_t dataSize;
    /// 头部校验和
    uint8_t headerSum;
    /// 数据
    uint8_t data[0];
}__PACKED;


typedef struct SacpHeader sacpHeader_t;

#define SACP_FRAME_CONTROL(_pri, _op) ((((_pri) & 0x03) << 6) | ((_op) & 0x07))

/// payload中定义仍支持四个， 实际支持能力，以::SACP_PRIORITY_NUM为准
#define SACP_FRAME_PRIORITY(_hdrOp)  (((_hdrOp) >> 6) & SACP_PRIORITY_MASK)
#define SACP_FRAME_OPCODE(_hdrOp)  ((_hdrOp) & 0x07)


/// 生成属性帧ID
#define SACP_FRAME_ATTRIBUTE_ID(_id, _type) (((_id) & 0xfff) | (((_type) & 0xf) << 12))
#define SACP_ATTRIBUTE_ID(_id)      ((_id) & 0xfff)
#define SACP_ATTRIBUTE_TYPE(_id)    (((_id) >> 12) & 0xf)


/**
 * @brief 数据格式0, 只有ID
 * 
 */
struct SacpDataItem0
{
    uint16_t id;
}__PACKED;

typedef struct SacpDataItem0 sacpDataItem0_t;

/**
 * @brief 数据格式1， 有ID和Value
 * 
 */
struct SacpDataItem1
{
    uint16_t id;
    uint8_t value[0];
}__PACKED;

typedef struct SacpDataItem1 sacpDataItem1_t;



#endif // __SACP_PAYLOAD_H__
