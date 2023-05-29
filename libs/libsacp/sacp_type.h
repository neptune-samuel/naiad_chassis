
#ifndef __SACP_TYPE_H__
#define __SACP_TYPE_H__

/**
 * @file sacp_type.h
 * @author Liu Chuansen (179712066@qq.com)
 * @brief SACP管理数据类型
 * @version 0.1
 * @date 2022-10-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifdef __cplusplus
 extern "C" {
#endif 


#include "sacp_payload.h"

/**
 * @brief 属性值
 * 
 */
typedef struct 
{
    /// 属性ID: 0-4095
    uint16_t id;
    /// 类型 ::SACP_TYPE 
    uint8_t type;
    /// 长度，目前仅用在OCTET类型上，其他固定大小
    uint8_t len;
    /// 属性值 
    union 
    {
        uint8_t v8;
        uint16_t v16;
        uint32_t v32;
        uint64_t v64;
        float f32;
        double f64;
        /// 有可能 需要指向一个可写区域
        const uint8_t *octet;
    }value;
}sacpAttribute_t;

/**
 * @brief 设置属性
 * 
 * @param attr 
 * @param id 
 * @param type 
 * @param value 
 */
static inline void sacpAttributeSetV8(sacpAttribute_t *attr, uint16_t id, uint8_t type, uint8_t value)
{
    attr->type = type;
    attr->id = id;
    attr->value.v8 = value;
}

/**
 * @brief 设置属性
 * 
 * @param attr 
 * @param id 
 * @param type 
 * @param value 
 */
static inline void sacpAttributeSetV16(sacpAttribute_t *attr, uint16_t id, uint8_t type, uint16_t value)
{
    attr->id = id;
    attr->type = type;
    attr->value.v16 = value;
}

/**
 * @brief 设置属性
 * 
 * @param attr 
 * @param id 
 * @param type 
 * @param value 
 */
static inline void sacpAttributeSetV32(sacpAttribute_t *attr, uint16_t id, uint8_t type, uint32_t value)
{
    attr->id = id;
    attr->type = type;
    attr->value.v32 = value;
}

/**
 * @brief 设置属性
 * 
 * @param attr 
 * @param id 
 * @param type 
 * @param value 
 */
static inline void sacpAttributeSetV64(sacpAttribute_t *attr, uint16_t id, uint8_t type, uint64_t value)
{
    attr->id = id;
    attr->type = type;
    attr->value.v64 = value;
}

#define sacpAttributeSetBool(_attr, _id, _value)    sacpAttributeSetV8(_attr, _id, SACP_TYPE_BOOL, _value)
#define sacpAttributeSetStatus(_attr, _id, _value)  sacpAttributeSetV8(_attr, _id, SACP_TYPE_STATUS, _value)

#define sacpAttributeSetUint8(_attr, _id, _value)  sacpAttributeSetV8(_attr, _id, SACP_TYPE_UINT8, _value)
#define sacpAttributeSetInt8(_attr, _id, _value)   sacpAttributeSetV8(_attr, _id, SACP_TYPE_INT8, (uint8_t)(_value))

#define sacpAttributeSetUint16(_attr, _id, _value)  sacpAttributeSetV16(_attr, _id, SACP_TYPE_UINT16, _value)
#define sacpAttributeSetInt16(_attr, _id, _value)   sacpAttributeSetV16(_attr, _id, SACP_TYPE_INT16, (uint16_t)(_value))

#define sacpAttributeSetUint32(_attr, _id, _value)  sacpAttributeSetV32(_attr, _id, SACP_TYPE_UINT32, _value)
#define sacpAttributeSetInt32(_attr, _id, _value)   sacpAttributeSetV32(_attr, _id, SACP_TYPE_INT32, (uint32_t)(_value))

#define sacpAttributeSetUint64(_attr, _id, _value)  sacpAttributeSetV64(_attr, _id, SACP_TYPE_UINT64, _value)
#define sacpAttributeSetInt64(_attr, _id, _value)   sacpAttributeSetV64(_attr, _id, SACP_TYPE_INT64, (uint64_t)(_value))

/**
 * @brief 设置属性
 * 
 * @param attr 
 * @param id 
 * @param type 
 * @param value 
 */
static inline void sacpAttributeSetFloat(sacpAttribute_t *attr, uint16_t id, float value)
{
    attr->id = id;
    attr->type = SACP_TYPE_FLOAT;
    attr->value.f32 = value;
}

/**
 * @brief 设置属性
 * 
 * @param attr 
 * @param id 
 * @param type 
 * @param value 
 */
static inline void sacpAttributeSetDouble(sacpAttribute_t *attr, uint16_t id, double value)
{
    attr->id = id;
    attr->type = SACP_TYPE_DOUBLE;
    attr->value.f64 = value;
}

/**
 * @brief 设置属性
 * 
 * @param attr 
 * @param id 
 * @param type 
 * @param value 
 */
static inline void sacpAttributeSetOctet(sacpAttribute_t *attr, uint16_t id, const uint8_t *data, int len)
{
    attr->id = id;
    attr->type = SACP_TYPE_OCTET;
    attr->value.octet = data;
    attr->len = len;
}

/**
 * @brief 将属性值转换为字串
 * 
 * @param attr 
 * @param buffer 
 * @param len 必须至少32字节, 否则返回空
 * @return const char* 
 */
const char *sacpAttributeValueString(const sacpAttribute_t *attr, char *buffer, int len);

/**
 * @brief 帧解析结构
 * 
 */
typedef struct 
{
    /// 序列号
    uint8_t sequence;
    /// 帧优先级
    uint8_t priority;
    /// 操作字
    uint8_t opCode;
    /// 属性数量 
    uint8_t attributesNum;
    /// 属性值
    sacpAttribute_t attributes[0];

    // 后面还会放置OCTET的值, 申请空间时会预留
}sacpFrameInfo_t;

#ifdef __cplusplus
}
#endif

#endif // __SACP_TYPE_H__
