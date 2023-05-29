

#ifndef __SACP_FRAME_H__
#define __SACP_FRAME_H__


/**
 * @file sacp_frame.h
 * @author Liu Chuansen (179712066@qq.com)
 * @brief SACP帧报文处理头文件
 * @version 0.1
 * @date 2022-10-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifdef __cplusplus
 extern "C" {
#endif 


#include "sacp_type.h"

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
int sacpMakeFrame(uint8_t sequence, uint8_t control, const sacpAttribute_t *attrs, uint8_t attrsNum, uint8_t *frame, uint16_t frameSize);


/**
 * @brief verify the header
 * 
 * @param hdr 
 * @return bool 
 */
bool sacpCheckHeader(const sacpHeader_t *hdr);


/**
 * @brief 得到帧的CRC
 * 
 * @param frame 
 * @param frameSize 
 * @return uint16_t 
 */
static inline uint16_t sacpFrameCrc(const uint8_t *frame, uint16_t frameSize)
{
    const uint16_t *p = (const uint16_t *)&frame[frameSize - 2];
    return letohs(*p); 
}


/**
 * @brief 计算帧的CRC
 * 
 * @param frame 
 * @param frameSize 
 * @return uint16_t 
 */
static inline uint16_t sacpCalculateFrameCrc(const uint8_t *frame, uint16_t frameSize)
{
    return crc16Modbus(0xffff, frame, frameSize - 2);    
}

/**
 * @brief 解析帧数据
 * 
 * @param frame 
 * @param frameSize 
 * @return sacpFrameInfo_t* 
 *  返回为空，表示解析失败
 *  需要使用sacpFrameInfoFree()释放内存
 */
sacpFrameInfo_t *sacpFrameParse(const uint8_t *frame, uint16_t frameSize);


/**
 * @brief 释放FRAME INFO
 * 
 * @param frameInfo 
 */
static inline void sacpFrameInfoFree(sacpFrameInfo_t *frameInfo)
{
    if (frameInfo)
    {
        osFree(frameInfo);
    }
}

/**
 * @brief 显示帧信息
 * 
 * @param prompt 
 * @param frame 
 */
void sacpFrameInfoDump(const char *prompt, const sacpFrameInfo_t *frame);

#ifdef __cplusplus
}
#endif

#endif // __SACP_FRAME_H__
