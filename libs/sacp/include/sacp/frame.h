
#ifndef __CXX_SACP_FRAME_H__
#define __CXX_SACP_FRAME_H__

/**
 * @file frame.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief SACP属性封装
 * @version 0.1
 * @date 2023-05-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>
#include <vector>
#include <iostream>
#include <cstring>
#include <chrono>

#include <common/sys_time.h>
#include <sacp/attribute.h>

namespace sacp {

class Frame
{

public:
    /// 定义属性类型
    enum class OpCode : uint8_t
    {
        Read = 0,
        Write = 1,
        ReadAck = 2,
        WriteAck = 3,
        Report = 4,
    };

    /// @brief 定义报文优先级
    enum class Priority : uint8_t
    {
        Priority0 = 0,
        Priority1 = 1,
        PriorityLowest = Priority0,
        PriorityHighest = Priority1,
    };

    Frame() { }

    Frame(
        std::string const & from, 
        Priority priority, 
        uint8_t sequence, 
        OpCode op_code, 
        std::vector<Attribute> const & attributes) 
        : from_(from),
        sequence_(sequence),
        priority_(priority),        
        op_code_(op_code),
        origin_(naiad::system::uptime()), 
        attributes_(attributes) { }

    Frame(
        std::string const & from, 
        Priority priority, 
        uint8_t sequence, 
        OpCode op_code) 
        : Frame(from, priority, sequence, op_code, {}) { }


    /// @brief 添加一个属性
    /// @param attr 
    /// @return 是否成功
    bool add_attribute(Attribute const & attr);    

    /// @brief 返回帧的一些信息
    /// @return std::string
    std::string brief();

    /// @brief 返回帧的详细信息
    /// @return std::string
    std::string info();    

    /// @brief 是否为空帧
    /// @return 
    bool is_empty();

    /// @brief  转换为可输出的数据帧
    /// @param data 帧缓存 
    /// @param size 大小
    /// @return 返回实际的长度
    ///  0 - 生成失败  
    std::size_t make_raw_frame(uint8_t *data, std::size_t size);

    /// @brief 返回报文类型
    /// @return 
    OpCode type() const 
    {
        return op_code_;
    }

    uint8_t sequence() const 
    {
        return sequence_;
    }

    Priority priority() const 
    {
        return priority_;
    }

    std::vector<Attribute> & attributes()
    {
        return attributes_;
    }

    naiad::system::SysTick const & origin() const 
    {
        return origin_;
    }

    /// @brief 最大属性数量
    static const std::size_t MaxAttributeNum;
    /// @brief 最大帧长度
    static const std::size_t MaxFrameSize;
private:
    // 来自哪里
    std::string from_;
    /// 序列号
    uint8_t sequence_;    
    // 优先级
    Priority priority_;
    OpCode op_code_;

    /// 插入的时间点
    naiad::system::SysTick origin_;

    /// 所有有属性
    std::vector<Attribute> attributes_;
};



} // end sacp


#endif // __CXX_SACP_FRAME_H__


