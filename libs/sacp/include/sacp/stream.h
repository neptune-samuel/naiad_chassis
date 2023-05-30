

#ifndef __CXX_SACP_STREAM_H__
#define __CXX_SACP_STREAM_H__

/**
 * @file stream.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 数据结构与流式数据之间转换
 * @version 0.1
 * @date 2023-05-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>
#include <queue>
#include <mutex>
#include <chrono>
#include <memory>

#include <sacp/attribute.h>
#include <sacp/frame.h>

namespace sacp {

class Stream
{
public:
    Stream(std::string const &name, int frame_fifo_num, int timeout_ms = 10); 
    explicit Stream(std::string const & name) :  Stream(name, 0, 0) { }
    
    // 不允许复制
    Stream(Stream const &) = delete;
    Stream & operator=(Stream const &) = delete;

    /// @brief 清空所有队列
    void reset();

    /**
     * @brief 输入数据，返回队列中的帧数量
     * 
     * @param data 
     * @param size 
     * @return int 
     */
    std::size_t input(uint8_t const *data, int size);


    /**
     * @brief 返回缓存的帧数量
     * 
     * @return int 
     */
    std::size_t frames_num();

    /**
     * @brief 从FIFO中接收一个数据帧
     * 
     * @return Frame 
     */
    std::unique_ptr<Frame> receive();
private:
    /// 名称
    std::string name_;
    /// 数据流
    std::queue<uint8_t> stream_;
    /// 在接收完数据，如果检测到有丢包，需要将数据放回队列中，std::queue无法实现
    /// 所以，增加一个队列，如果该队列不为空，先从这里取数据
    std::queue<uint8_t> stream_head_;
    /// 帧FIFO
    std::queue<std::unique_ptr<Frame>> frames_;
    /// 最多缓存的帧数据 
    std::size_t max_frame_num_;
    std::mutex stream_mutex_;
    std::mutex frames_mutex_;

    /// 接收超时时间
    int timeout_ms_;
    std::chrono::steady_clock::time_point rx_expired_;

    // 接收缓存 
    std::unique_ptr<uint8_t[]> buffer_;
    unsigned int buffer_size_ = 0;
    unsigned int parsed_size_ = 0;

    /// 解析一次帧
    void parse_frame();

    /// @brief 入队列 
    /// @param data 
    void stream_push(uint8_t data);

    /// @brief 放回前面
    /// @param data 
    void stream_push_front(uint8_t data);

    uint8_t stream_peek();

    /// @brief stream中读取一个字节
    /// @return 
    uint8_t stream_pop();

    /// @brief 返回流的长度
    /// @return 
    std::size_t stream_size();
};


} // end sacp


#endif // __CXX_SACP_STREAM_H__

