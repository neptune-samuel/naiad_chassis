

/**
 * @file frame.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 封装SACP属性
 * @version 0.1
 * @date 2023-05-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>
#include <vector>
#include <iostream>
#include <cstring>

#include <common/logger.h>

#include <libsacp/sacp_frame.h>
#include <sacp/attribute.h>
#include <sacp/frame.h>


namespace sacp {


const std::size_t Frame::MaxAttributeNum = CONFIG_SACP_MAX_ATTRIBUTES_PER_FRAME;
const std::size_t Frame::MaxFrameSize = CONFIG_SACP_MAX_FRAME_SIZE;

/// @brief 添加一个属性
/// @param attr 
bool Frame::add_attribute(Attribute const & attr)
{
    // 暂不限制 
    // if (attributes_.size() > MaxAttributeNum)
    // {
    // }
    attributes_.push_back(attr);

    return true;
}    

/// @brief 返回帧的一些信息
/// @return 
std::string Frame::brief()
{
    char buffer[64];
    sprintf(buffer, "OP:%s PRI:%d SEQ:%d, ATTRS:%ld", SACP_OP_CODE_NAME((uint8_t)op_code_), (uint8_t)priority_, sequence_, attributes_.size());
    return std::string(buffer);
}

/// @brief 返回帧的详细信息
/// @return std::string
std::string Frame::info()
{
    std::string info = brief();
    int seq = 0;

    info += " {";
    for (auto & attr : attributes_)
    {        
        if (seq == 4)
        {
            info += "\r\n";
        }
        info += attr.brief();        

        seq ++;
    }

    info += "}";

    return info;
}


/// @brief 是否为空帧
/// @return 
bool Frame::is_empty()
{
    return (attributes_.size() == 0);
}

/// @brief  转换为可输出的数据帧
/// @param data 帧缓存 
/// @param size 大小
/// @return 返回实际的长度
///  0 - 生成失败  
std::size_t Frame::make_raw_frame(uint8_t *data, std::size_t size)
{ 
    sacpAttribute_t attrs[MaxAttributeNum] = { };
    std::size_t num = 0;

    if (attributes_.size() > MaxAttributeNum)
    {
        slog::warning("make_frame: got too much({}) attributes", attributes_.size());                
    }

    for (auto & a : attributes_)
    {
        if (num < MaxAttributeNum)
        {
            to_sacp_attribute(a, &attrs[num ++]);
        }        
    }

    return sacpMakeFrame(sequence_, SACP_FRAME_CONTROL(((uint8_t)priority_), ((uint8_t)op_code_)), attrs, num, data, size);
}


} // sacp 

