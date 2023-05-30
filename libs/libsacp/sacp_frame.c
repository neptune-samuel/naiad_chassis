
/**
 * @file sacp_frame.c
 * @author Liu Chuansen (179712066@qq.com)
 * @brief SACP帧处理函数 
 * @version 0.1
 * @date 2022-10-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "sacp_type.h"

#ifndef __no_framework__

#define TRACE_ENABLE
#define TRACE_LEVEL T_INFO 
#define TRACE_TAG "sacp"
#include <common/trace.h>

#endif // __no_framework__

/**
 * @brief 将属性值转换为字串
 * 
 * @param attr 
 * @param buffer 
 * @param len 必须至少32字节, 否则返回空
 * @return const char* 
 */
const char *sacpAttributeValueString(const sacpAttribute_t *attr, char *buffer, int len)
{
    if (buffer == NULL || len < 32)
    {
        return "";
    }

    buffer[0] = '\0';

    switch(attr->type)
    {
        case SACP_TYPE_INT8:
            sprintf(buffer, "%d", (int8_t)attr->value.v8);
            break;
        case SACP_TYPE_BOOL:
        case SACP_TYPE_UINT8:
        case SACP_TYPE_STATUS:
            sprintf(buffer, "%u", attr->value.v8);
            break;
        case SACP_TYPE_INT16:
            sprintf(buffer, "%d", (int16_t)attr->value.v16);
            break;
        case SACP_TYPE_UINT16:
            sprintf(buffer, "%u", attr->value.v16);
            break;
        case SACP_TYPE_INT32:
            #ifdef __build_arch64__
            sprintf(buffer, "%d", (int32_t)attr->value.v32);
            #else 
            sprintf(buffer, "%ld", (int32_t)attr->value.v32);
            #endif 
            break;
        case SACP_TYPE_UINT32:
            #ifdef __build_arch64__
            sprintf(buffer, "%u", attr->value.v32);
            #else 
            sprintf(buffer, "%lu", attr->value.v32);
            #endif 
            break;
        case SACP_TYPE_INT64:
            #ifdef __build_arch64__
            sprintf(buffer, "%ld", (int64_t)attr->value.v64);
            #else 
            sprintf(buffer, "%lld", (int64_t)attr->value.v64);
            #endif 
            break;
        case SACP_TYPE_UINT64:
            #ifdef __build_arch64__
            sprintf(buffer, "%lu", (int64_t)attr->value.v64);
            #else 
            sprintf(buffer, "%llu", attr->value.v64);
            #endif 
            break;
        case SACP_TYPE_FLOAT:
            sprintf(buffer, "%f", attr->value.f32);
            break;
        case SACP_TYPE_DOUBLE:
            sprintf(buffer, "%f", attr->value.f64);
            break;
        case SACP_TYPE_OCTET:
            {
                // 转换成可十六进制字串
                #define tohex(_h) (((_h) > 9) ? (((_h) - 10) + 'A') : ((_h) + '0'))

                sprintf(buffer, "(%d)", attr->len);

                int n = strlen(buffer);
                for (int i = 0; (i < attr->len) && (n < len - 1); i ++)
                {
                    buffer[n ++] = tohex((attr->value.octet[i] >> 4) & 0x0f);
                    buffer[n ++] = tohex(attr->value.octet[i] & 0x0f);
                }
                buffer[n] = '\0';
            }
            break;
    }

    return buffer;
}

/**
 * @brief 计算头部SUM的方法
 * 
 * @param hdr 
 * @return uint8_t 
 */
static uint8_t makeCheckSum8(const uint8_t *p, uint16_t length)
{
    uint8_t sum = 0;

    for (uint16_t i = 0; i < length; i++)
    {
        sum += *p++;
    }

    sum = ~sum;
    sum += 1;

    return sum;
}


/**
 * @brief 获取整型变量的长度，如果是字串，返回为0
 * 
 * @param type 
 * @return int 
 */
static inline int getIntegerAttributeSize(uint8_t type)
{
    switch (type)
    {
        case SACP_TYPE_BOOL:
        case SACP_TYPE_UINT8:
        case SACP_TYPE_INT8:
        case SACP_TYPE_STATUS:
            return 1;
        case SACP_TYPE_UINT16:
        case SACP_TYPE_INT16:
            return 2;
        case SACP_TYPE_UINT32:
        case SACP_TYPE_INT32:
            return 4;
        case SACP_TYPE_UINT64:
        case SACP_TYPE_INT64:
            return 8;
    }

    return 0;
}

/**
 * @brief 获取浮点属性的长度，如果是字串或整形，返回为0
 * 
 * @param type 
 * @return int 
 */
static inline int getFloatAttributeSize(uint8_t type)
{
    switch (type)
    {
        case SACP_TYPE_FLOAT:
            return 4;
        case SACP_TYPE_DOUBLE:
            return 8;
    }

    return 0;
}


/**
 * @brief 填充一个整形属性数据 
 * 
 * @param data 
 * @param dataSize 
 * @param attr 
 * @return int 
 */
static int fillIntegerAttribute(uint8_t *data, uint8_t dataSize, const sacpAttribute_t *attr, int intSize)
{
    sacpDataItem1_t *item = (sacpDataItem1_t *)data;

    if (intSize <= 0)
    {
        intSize = getIntegerAttributeSize(attr->type);
    }

    if (intSize < 1)
    {
        elog("not an integer attribute(%d:%d)!", attr->id, attr->type);
        return 0;
    }

    // 看看长度是否足够
    if (sizeof(*item) + intSize > dataSize)
    {
        elog("no enough data for attribute(%d:%d)", attr->id, attr->type);
        return 0;
    }

    // 先写一个ID
    item->id = htoles(SACP_FRAME_ATTRIBUTE_ID(attr->id, attr->type));

    switch(intSize)
    {
        case 1:
        {
            *((uint8_t *)item->value) = attr->value.v8;
        }
        break;
        case 2:
        {
            *((uint16_t *)item->value) = htoles(attr->value.v16);
        }
        break;
        case 4:
        {
            *((uint32_t *)item->value) = htolel(attr->value.v32);
        }
        break;
        case 8:
        {
            uint64_t v64 = htolell(attr->value.v64);
            osMemcpy(item->value, &v64, sizeof(v64));
        }
        break;    
        default:
        {
            elog("unsupport int size :%d", intSize);
            return 0;
        }            
    }

    return (sizeof(*item) + intSize);
}


/**
 * @brief 填充一个浮点属性数据 
 * 
 * @param data 
 * @param dataSize 
 * @param attr 
 * @return int 
 */
static int fillFloatAttribute(uint8_t *data, uint8_t dataSize, const sacpAttribute_t *attr)
{
    sacpDataItem1_t *item = (sacpDataItem1_t *)data;
    int floatSize = getFloatAttributeSize(attr->type);

    // 看看长度是否足够
    if (sizeof(*item) + floatSize > dataSize)
    {
        elog("no enough data for attribute(%d:%d)", attr->id, attr->type);
        return 0;
    }

    // 先写一个ID
    item->id = htoles(SACP_FRAME_ATTRIBUTE_ID(attr->id, attr->type));


    switch(attr->type)
    {
        case SACP_TYPE_FLOAT:
        {
            //*((float *)item->value) = htolel(attr->value.f32);
            float v32 = htolel(attr->value.f32);
            osMemcpy(item->value, &v32, sizeof(v32));            
        }
        break;
        case SACP_TYPE_DOUBLE:
        {
            double v64 = htolell(attr->value.f64);
            osMemcpy(item->value, &v64, sizeof(v64));
        }
        break;    
        default:
        {
            elog("not a float attribute(%d:%d)!", attr->id, attr->type);
            return 0;
        }            
    }

    return (sizeof(*item) + floatSize);
}


/**
 * @brief 填充一个OCTET属性数据 
 * 
 * @param data 
 * @param dataSize 
 * @param attr 
 * @return int 
 */
static int fillOctetAttribute(uint8_t *data, uint8_t dataSize, const sacpAttribute_t *attr)
{
    sacpDataItem1_t *item = (sacpDataItem1_t *)data;

    // 看看长度是否足够, OCTET的值，有一字节是存长度， 支持空字串
    if (sizeof(*item) + attr->len + 1 > dataSize)
    {
        elog("no enough data for attribute(%d:%d)", attr->id, attr->type);
        return 0;
    }

    // 先写一个ID
    item->id = htoles(SACP_FRAME_ATTRIBUTE_ID(attr->id, attr->type));
    item->value[0] = attr->len;

    if (attr->len > 0)
    {
        osMemcpy(&item->value[1],  attr->value.octet, attr->len);
    }

    return (sizeof(*item) + attr->len + 1);
}


/**
 * @brief 填充帧PAYLOAD数据
 * 
 * @param opCode 
 * @param attrs 
 * @param attrsNum 
 * @param data 
 * @param dataSize 
 * @return int 
 */
static int sacpFrameFillData(uint8_t opCode,  const sacpAttribute_t *attrs, uint8_t attrsNum, uint8_t *data, uint16_t dataSize)
{    
    /*
    只有读请求，只给出ID值，其他请求均是ID+VALUE    
    */
    if (opCode == SACP_OP_READ)
    {
        sacpDataItem0_t *item = (sacpDataItem0_t *)data;
        int i;

        for (i = 0; (i < attrsNum) && (dataSize >= sizeof(*item)); i ++, item ++)
        {
            item->id = htoles(SACP_FRAME_ATTRIBUTE_ID(attrs[i].id, attrs[i].type));
        }

        // 没有足够的空间保存所有的数据
        if (i < attrsNum)
        {
            elog("fill read attributes(num=%d) failed", attrsNum);
            return 0;
        }

        return (attrsNum * sizeof(*item));
    }
    else 
    {
        uint8_t *item = data;
        uint8_t itemLen = 0;
        uint16_t dataLeft = dataSize;

        for (int i = 0; i < attrsNum; i ++)
        {
            // 处理整形数
            int intSize = getIntegerAttributeSize(attrs[i].type);

            if (intSize > 0)
            {
                itemLen = fillIntegerAttribute(item, dataLeft, &attrs[i], intSize);
                if (itemLen == 0)
                {
                    elog("fill integer attribute(%d:%d) failed, data size:%d", attrs[i].id, attrs[i].type, dataLeft);
                    return 0;
                }
            }
            else 
            {
                int floatSize = getFloatAttributeSize(attrs[i].type);

                if (floatSize > 0)
                {
                    itemLen = fillFloatAttribute(item, dataLeft, &attrs[i]);
                    if (itemLen == 0)
                    {
                        elog("fill float attribute(%d:%d) failed, data size:%d", attrs[i].id, attrs[i].type, dataLeft);
                        return 0;
                    }                    
                }
                else if (attrs[i].type == SACP_TYPE_OCTET)
                {
                    itemLen = fillOctetAttribute(item, dataLeft, &attrs[i]);
                    if (itemLen == 0)
                    {
                        elog("fill octet attribute(%d:%d) failed, data size:%d", attrs[i].id, attrs[i].type, dataLeft);
                        return 0;
                    }                     
                }
                else 
                {
                    elog("unsupport attribute(%d:%d)", attrs[i].id, attrs[i].type);
                    return 0; 
                }
            }

            item += itemLen;
            dataLeft -= itemLen;
        }

        return (dataSize - dataLeft);
    }
}



/**
 * @brief 生成SACP的帧
 * 
 * @param sequence 帧序列号
 * @param opCode 操作码
 * @param attrs 属性
 * @param attrsNum 属性数量
 * @param frame 帧数据空间，需确保frame内容为0
 * @param frameSize frame可用长度
 * @return int 
 *  0 - 生成帧失败 
 *  >0 - 生成帧的长度
 */
int sacpMakeFrame(uint8_t sequence, uint8_t control, const sacpAttribute_t *attrs, uint8_t attrsNum, uint8_t *frame, uint16_t frameSize)
{
    sacpHeader_t *hdr;
    int dataLeft, dataLen;

    if (frameSize <= sizeof(*hdr))
    {
        elog("frame size(%d) too small", frameSize);
        return 0;
    }

    hdr = (sacpHeader_t *)frame;
    dataLeft = frameSize - sizeof(*hdr) - 2; // 最后需要一个CRC

    dataLen = sacpFrameFillData(SACP_FRAME_OPCODE(control),  attrs, attrsNum, hdr->data, dataLeft);
    if (dataLen == 0)
    {
        elog("unable to fill all frame datas");
        return 0;
    }

    // 生成头部
    hdr->sof = SACP_SOF;
    hdr->sequence = sequence;
    hdr->control = control;
    hdr->dataSize = dataLen;
    hdr->headerSum = makeCheckSum8((uint8_t *)hdr, sizeof(*hdr) - 1);
    
    // 计算CRC
    uint16_t *pCrc = (uint16_t *)&frame[sizeof(*hdr) + dataLen];
    uint16_t crc = crc16Modbus(0xffff, (const uint8_t *)hdr, sizeof(*hdr) + dataLen);
    *pCrc = htoles(crc);

    return sizeof(*hdr) + dataLen + 2; // 2 is crc16
}


/**
 * @brief verify the header
 * 
 * @param hdr 
 * @return bool 
 */
bool sacpCheckHeader(const sacpHeader_t *hdr)
{
    // TODO:
    if (hdr->sof != SACP_SOF)
    {
        return false;
    }

    uint8_t sum = makeCheckSum8((const uint8_t *)hdr, sizeof(*hdr) - 1);

    if (sum != hdr->headerSum)
    {
        return false;
    }

    return true;
}


/**
 * @brief 获取当前属性的值的长度
 * 
 * @param item 当前属性
 * @param left 输入当前帧剩余的长度，防止访问溢出
 * @return int 
 */
static int getAttributeValueSize(const sacpDataItem1_t *item, int left)
{
    uint16_t id = letohs(item->id);
    uint8_t type = SACP_ATTRIBUTE_TYPE(id);
    int valueSize = 0;

    switch(type)
    {
        case SACP_TYPE_BOOL:
        case SACP_TYPE_UINT8:
        case SACP_TYPE_INT8:
        case SACP_TYPE_STATUS:        
        valueSize = (left < 1) ? 0 : 1;
        break;

        case SACP_TYPE_UINT16:
        case SACP_TYPE_INT16:                        
        valueSize = (left < 2) ? 0 : 2;
        break;

        case SACP_TYPE_UINT32:
        case SACP_TYPE_INT32:   
        case SACP_TYPE_FLOAT:
        valueSize = (left < 4) ? 0 : 4;
        break;

        case SACP_TYPE_UINT64:
        case SACP_TYPE_INT64:  
        case SACP_TYPE_DOUBLE:              
        valueSize = (left < 8) ? 0 : 8;
        break;

        case SACP_TYPE_OCTET:
        if (left < 1)
        {
            valueSize = 0;
        }
        else 
        {
            valueSize = 1 + item->value[0];
            if (left < valueSize)
            {
                valueSize = 0;
            }
        }        
        break;

        default:
        valueSize = 0;
        break;
    }  

    return valueSize;  
}

/**
 * @brief 解析帧数据
 * 
 * @param frame 
 * @param frameSize 
 * @return sacpFrameInfo_t* 
 */
sacpFrameInfo_t *sacpFrameParse(const uint8_t *frame, uint16_t frameSize)
{
    const sacpHeader_t *hdr = (const sacpHeader_t *)frame;
    uint8_t opCode = SACP_FRAME_OPCODE(hdr->control);    
    int dataSize = hdr->dataSize;
    int attributesNum = 0;    
    int octetDataSize = 0;
    sacpFrameInfo_t *frameInfo = NULL;   

    // check frame size
    if (frameSize < (sizeof(*hdr) + hdr->dataSize + 2))
    {
        return NULL;
    }

    // 获取属性数量 
    if (opCode == SACP_OP_READ)
    {
        // sacpDataItem0_t
        attributesNum = dataSize / 2;
    }
    else 
    {
        int left = dataSize;
        int offset = 0;
        int valueSize = 0;
        const sacpDataItem1_t *item;

        // 遍历每个属性，求出每个属性的长度
        do {
            item = (const sacpDataItem1_t *)&hdr->data[offset];

            valueSize = getAttributeValueSize(item, left);

            // 如果为0，数据解析错误
            if (valueSize == 0)
            {
                elog("frame parse failed at offset %d", offset);
                return NULL;
            }

            offset += sizeof(*item) + valueSize;
            attributesNum ++;

            if (SACP_ATTRIBUTE_TYPE(item->id) == SACP_TYPE_OCTET)
            {
                // valueSize 包含１字节的长度信息，用作间隔（结束符）
                octetDataSize += valueSize;
            }

            left = dataSize - offset;
        }while (left > 0);
    }

    {
        int frameInfoSize = sizeof(*frameInfo) + (attributesNum * sizeof(sacpAttribute_t)) + octetDataSize;

        frameInfo = osMalloc(frameInfoSize);
        if (frameInfo == NULL)
        {
            elog("%s: malloc(%d) failed", __FUNCTION__, frameInfoSize);
            return NULL;
        }

        osMemset(frameInfo, 0, frameInfoSize);        
    }
    
    frameInfo->sequence = hdr->sequence;
    frameInfo->priority = SACP_FRAME_PRIORITY(hdr->control);
    frameInfo->opCode = opCode;
    frameInfo->attributesNum = attributesNum;

    // 如果是读操作，只有ID没有值 
    if (opCode == SACP_OP_READ)
    {
        const uint16_t *ids = (const uint16_t *)hdr->data;

        for (int i = 0; i < attributesNum; i ++)
        {
            uint16_t id = letohs(ids[i]);

            frameInfo->attributes[i].id = SACP_ATTRIBUTE_ID(id);
            frameInfo->attributes[i].type = SACP_ATTRIBUTE_TYPE(id);
        }
    }
    else 
    {
        // 将报文中的值，读到attributes
        // 如果是OCTET，将值复制到frameInfo的后面OCTET部分
        uint8_t *octetBuffer = (octetDataSize > 0) ? (uint8_t *)&frameInfo->attributes[attributesNum] : NULL;
        int offset = 0;
        int valueSize = 0;

        for (int i = 0; i < attributesNum; i ++)
        {
            const sacpDataItem1_t *item = (const sacpDataItem1_t *)&hdr->data[offset];
            uint16_t id = letohs(item->id);
            frameInfo->attributes[i].id = SACP_ATTRIBUTE_ID(id);
            frameInfo->attributes[i].type = SACP_ATTRIBUTE_TYPE(id);            

            switch(frameInfo->attributes[i].type)
            {
                case SACP_TYPE_BOOL:
                case SACP_TYPE_UINT8:
                case SACP_TYPE_INT8:
                case SACP_TYPE_STATUS:
                    frameInfo->attributes[i].value.v8 = item->value[0];
                    valueSize = 1;
                break;
                case SACP_TYPE_UINT16:
                case SACP_TYPE_INT16:                
                    frameInfo->attributes[i].value.v16 = letohs(*((const uint16_t *)item->value));
                    valueSize = 2;
                break;
                case SACP_TYPE_UINT32:
                case SACP_TYPE_INT32:   
                case SACP_TYPE_FLOAT:             
                    frameInfo->attributes[i].value.v32 = letohl(*((const uint32_t *)item->value));
                    valueSize = 4;
                break;

                case SACP_TYPE_UINT64:
                case SACP_TYPE_INT64:  
                case SACP_TYPE_DOUBLE:
                {
                    uint64_t v64;
                    osMemcpy(&v64, item->value, sizeof(v64));                
                    frameInfo->attributes[i].value.v64 = letohll(v64);
                    valueSize = 8;
                }
                break;
                case SACP_TYPE_OCTET:
                    valueSize = 1 + item->value[0];
                    frameInfo->attributes[i].len = item->value[0];
                    
                    if (frameInfo->attributes[i].len > 0)
                    {
                        // 将帧数据复制到 octet存储位置
                        osMemcpy(octetBuffer, &item->value[1], valueSize - 1);
                        frameInfo->attributes[i].value.octet = octetBuffer;
                        octetBuffer += valueSize;
                    }
                    else 
                    {
                        // 等于它自己
                        frameInfo->attributes[i].value.octet = &frameInfo->attributes[i].value.v8;
                    }
                break;
            }

            offset += sizeof(*item) + valueSize;
        }
    }

    return frameInfo;
}

/**
 * @brief 显示帧信息
 * 
 * @param prompt 
 * @param frame 
 */
void sacpFrameInfoDump(const char *prompt, const sacpFrameInfo_t *frame)
{
    if (prompt)
    {
        rawPrint("%s ", prompt);
    }

    char buffer[40];
    
    rawPrint("OP:%s PRI:%d SEQ:%d, ATTRS:%d\r\n", SACP_OP_CODE_NAME(frame->opCode), frame->priority, frame->sequence, frame->attributesNum);

    for (int i = 0; i < frame->attributesNum; i ++)
    {
        rawPrint(" + [%4d](%-5s):%s\r\n", frame->attributes[i].id, SACP_TYPE_NAME(frame->attributes[i].type), 
            sacpAttributeValueString(&frame->attributes[i], buffer, sizeof(buffer)));
    }
}
