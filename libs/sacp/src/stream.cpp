

/**
 * @file attribute.cpp
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
#include <sacp/stream.h>


namespace sacp {


Stream::Stream(std::string const &name, int frame_fifo_num, int timeout_ms)
{
    name_ = name;
    max_frame_num_ = frame_fifo_num;
    timeout_ms_ = timeout_ms;
    parsed_size_ = 0;

    buffer_size_ = CONFIG_SACP_MAX_FRAME_SIZE;
    // 创建一个缓存 
    buffer_ = std::make_unique<uint8_t []>(buffer_size_);    
}


/// @brief 入队列 
/// @param data 
void Stream::stream_push(uint8_t data)
{
    std::lock_guard<std::mutex> lock(stream_mutex_);    
    stream_.push(data);
}

/// @brief 放回前面
/// @param data 
void Stream::stream_push_front(uint8_t data)
{
    std::lock_guard<std::mutex> lock(stream_mutex_);      
    stream_head_.push(data);
}


uint8_t Stream::stream_peek()
{
    uint8_t ret = 0;

    std::lock_guard<std::mutex> lock(stream_mutex_);  

    if (stream_head_.size() > 0)
    {
        return stream_head_.front();
    }

    if (stream_.size() > 0)
    {
        return stream_.front();
    }
    
    return ret;
}

/// @brief stream中读取一个字节
/// @return 
uint8_t Stream::stream_pop()
{
    uint8_t ret = 0;

    std::lock_guard<std::mutex> lock(stream_mutex_);  

    if (stream_head_.size() > 0)
    {
        ret = stream_head_.front();
        stream_head_.pop();
        return ret;
    }

    if (stream_.size() > 0)
    {
        ret = stream_.front();
        stream_.pop();
        return ret;
    }
    
    return ret;
}

int Stream::stream_size()
{
    std::lock_guard<std::mutex> lock(stream_mutex_);  
    return stream_head_.size() + stream_.size();
}


/**
 * @brief 返回缓存的帧数量
 * 
 * @return int 
 */
int Stream::frames_num()
{
    // 先处理报文
    parse_frame();

    std::lock_guard<std::mutex> lock(frames_mutex_);
    return frames_.size();    
}

/**
 * @brief 输入数据，返回队列中的帧数量
 * 
 * @param data 
 * @param size 
 * @return int 
 */
int Stream::input(uint8_t const *data, int size)
{
    // 检查上次接收数据是否超时
    // 只有大于0，才启用超时清除机制
    if (timeout_ms_ > 0)
    {
        auto now = std::chrono::steady_clock::now();

        if ((stream_size() > 0) && (now > rx_expired_))
        {
            slog::warning("{}: receive timeout, drop {} bytes", name_, stream_size());

            std::lock_guard<std::mutex> lock(stream_mutex_);

            decltype(stream_)().swap(stream_);
            decltype(stream_)().swap(stream_head_);
            // 清空接收的数据

            // 复位解析长度
            parsed_size_ = 0;
        }
        // 更新接收超时
        rx_expired_ = now + std::chrono::milliseconds(timeout_ms_);
    }

    // 将数据压入流中
    for (int i = 0; i < size; i ++)
    {
        stream_push(data[i]);
    }

    //slog::trace("-> queue {} bytes, stream size = {}", size, stream_size());

    // 解析帧数据
    parse_frame();

    return frames_num();
} 


/**
 * @brief 从FIFO中解析帧数据
 * 
 */
void Stream::parse_frame()
{
    // 没有任何数据，直接返回
    if (!stream_size())
    {
        return;
    }

    // 思路：
    //  - 找到头部  失败，
    //  - 有头部，确定数据大小，是否完整 
    // 完整，调用帧解析函数，转换

    if (parsed_size_ < sizeof(sacpHeader_t))
    {
        // 将数据读入到缓存 
        parsed_size_ = 0;

        // 找到SOF
        while ((stream_size() > 0) && (stream_peek() != SACP_SOF))
        {
            stream_pop();
        }

        if (stream_size() == 0)
        {
            slog::trace("{}: sacp-stream: sof not found", name_);
            return ;
        }

        // 找到SOF，确定头部长度是否足够
        if (stream_size() < sizeof(sacpHeader_t))
        {
            //slog::trace("{}: sacp-stream: wait for full header", name_);
            return;
        }

        //buffer_[parsed_size_ ++] = SACP_SOF;
        while (parsed_size_ < sizeof(sacpHeader_t))
        {
            buffer_[parsed_size_ ++ ] = stream_pop();
        }

        slog::trace("-> get header {} bytes, stream left {} bytes", parsed_size_, stream_size());

        // check header if ok
        if (!sacpCheckHeader((sacpHeader_t *)(buffer_.get())))
        {            
            slog::trace_data(buffer_.get(), parsed_size_, "-> header check failed:");

            // 出错了，需要重新找头部
            for (int i = 1; i < sizeof(sacpHeader_t) ; i ++)
            {
                stream_push_front(buffer_[i]);
            }

            parsed_size_ = 0;
            return ;
        }
    }

    // 接收完头部数据，确定报文长长
    sacpHeader_t *hdr = (sacpHeader_t *)(buffer_.get());
    uint16_t frame_size = sizeof(*hdr) + hdr->dataSize + 2;

    if (frame_size > buffer_size_)
    {
        slog::error("{}: rx frame size({}) is bigger than buffer size", name_, frame_size);

        parsed_size_ = 0;
        return ;
    }

    // 如果未全部接收完成，等下次再接收
    if (stream_size() < (frame_size - sizeof(*hdr)))
    {
        //slog::trace("-> wait for full data");
        return ;
    }

    // 先获取所有数据
    for (int i = 0; i < frame_size - sizeof(*hdr); ++ i)
    {
        buffer_[parsed_size_] = stream_pop();
        ++ parsed_size_;
    }

    // 校验帧是否有问题
    uint16_t calcCrc = sacpCalculateFrameCrc(buffer_.get(), frame_size);
    uint16_t rxCrc = sacpFrameCrc(buffer_.get(), frame_size);
    if (calcCrc != rxCrc)
    {
        slog::warning("{}: rx frame crc failed, rx:{:04X}, calc:{:04X}", name_, rxCrc, calcCrc);
        // 显示错包
        slog::warning_data(buffer_.get(), frame_size, "{}: crc-failed({}):", name_, frame_size);

        // 这里处理粘包问题，CRC出错的可能是丢包了，这样我需要把头部以下数据放回去，可以保证下一个帧正常接收 
        // 放回几个字节即可，比如2字节
        for (int i = parsed_size_ - 4; i < parsed_size_; ++ i)
        {
            stream_push_front(buffer_[i]);
        }

        parsed_size_ = 0;
        return ;
    }

    // 复位解析长度
    parsed_size_ = 0;

    // 到这里，意味着整帧接收完成 
    {
        sacpFrameInfo_t *fi = sacpFrameParse(buffer_.get(), frame_size);
        if (fi == nullptr)
        {
            slog::error("{}: frame parse failed", name_);
            return ;
        }

        auto ptr = std::make_unique<Frame>(name_, (Frame::Priority)fi->priority, fi->sequence, (Frame::OpCode)fi->opCode);

        for (int i = 0; i < fi->attributesNum; i ++)
        {
            Attribute attr;
            from_sacp_attribute(attr, &fi->attributes[i]);
            ptr->add_attribute(attr);
        }

        std::lock_guard<std::mutex> lock(frames_mutex_);

        // 压入数据帧
        slog::trace("{}: receive a frame : {}", name_, ptr->brief());
        frames_.push(std::move(ptr));

        // 检查队列是否超过最大数，如果丢弃        
        if (max_frame_num_ > 0)
        {
            int count  = 0;
            while (frames_.size() > max_frame_num_)
            {
                count ++;
                frames_.pop();
            }

            if (count > 0)
            {
                slog::warning("{}: drop {} frames due to fifo full", name_, count);
            }
        }

        free(fi);
    }

}

/**
 * @brief 从FIFO中接收一个数据帧
 * 
 * @return Frame 
 */
std::unique_ptr<Frame> Stream::receive()
{
    std::lock_guard<std::mutex> lock(frames_mutex_);

    if (frames_.empty())
    {
        parse_frame();

        if (frames_.empty())
        {
            return nullptr;
        }
    }

    auto f = std::move(frames_.front());
    frames_.pop();
    
    return f;
}


void Stream::reset()
{

    {
        std::lock_guard<std::mutex> lock(stream_mutex_);

        // 清空所有stream 
        decltype(stream_)().swap(stream_);
        decltype(stream_head_)().swap(stream_head_);
    }

    {
        std::lock_guard<std::mutex> lock(frames_mutex_);

        decltype(frames_)().swap(frames_);
    }
}




} // end sacp

