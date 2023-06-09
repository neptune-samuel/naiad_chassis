

#ifndef __NAIAD_SACP_CLIENT_H__
#define __NAIAD_SACP_CLIENT_H__

/**
 * @file sacp_client.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 封装一个SACP客户端的类
 * @version 0.1
 * @date 2023-05-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <future>
#include <atomic>

#include <common/sys_time.h>
#include <common/tcp_server.h>
#include <common/serial_port.h>

#include <sacp/attribute.h>
#include <sacp/stream.h>
#include <sacp/frame.h>

#include "vofa_debuger.h"


namespace sacp
{


/*
  SACP Client
    从串口接收数据  -> SacpStream -> SacpFrame 
                                        -> Report -> Report Queue
                                        -> Read/Write Ack -> if request_pending send ack , else drop 
                                        -> Other, drop, give a warning

    是否有request 
          -> 待发送？-> 发送请求，并设定超时 
          -> 待响应? -> 是否超时？ 超时，通知请求源超时  

    提供方法: 同步方法      
       read_attributes(Read, Attributes)
       write_attributes(Write, Attributes)
       send_request(buf, size)
    
       SacpFrame = receive_report()

    同步请求实现 
      请求数据结构：
         Frame
         Promise
         future
      发送时，将请求放入串口帧FIFO
      使用 feture.get();等待结果          

    // 异步请求实现：
    //    发送时，将请求放入串口帧FIFO
    //    返回帧ID

    // 可通过帧ID获取结果   


    数据可视化功能:
    使用VOFAService实现属性数据可视化功能，可以任意设定属性组合

    add_vofa_service(port, period, datas)


*/

/// 指定该函数为主线程使用
#define _MAIN_THREAD_
/// 指定为发送线程的函数
#define _TX_THREAD_  
/// 指定为导出函数
#define _EXPORT_API_  

class SacpClient
{

public:

    typedef std::function<void(std::vector<sacp::Attribute> const &)> ReportHandle;

    enum class OperationStatus : int 
    {
        Ok = 0,
        Going, // 正在处理中
        NoAttributes,
        FewAttributesMissed,
        TooManyAttributes,
        OpCodeNotSupported,
        QueueFull,
        Timeout,
        MakeFrameFailed,
        FrameSizeTooLarge,
        TransactionNotFound,
        NoSuchObject,
        InvalidParameter,
        InternalError,
    };

    static char const* OperationStatusName(OperationStatus status) 
    {
        switch(status)
        {
            case OperationStatus::Ok:
                return "Ok";
            case OperationStatus::Going:
                return "Going";
            case OperationStatus::NoAttributes:
                return "No Attributes";    
            case OperationStatus::FewAttributesMissed:
                return "FewAttributesMissed";
            case OperationStatus::TooManyAttributes:
                return "Too Many Attributes";
            case OperationStatus::OpCodeNotSupported:
                return "OpCode Not Supported";  
            case OperationStatus::QueueFull:
                return "Queue Full";    
            case OperationStatus::Timeout:
                return "Timeout";
            case OperationStatus::MakeFrameFailed:
                return "Make Frame Failed";                                 
            case OperationStatus::FrameSizeTooLarge:
                return "Frame Size Too Large"; 
            case OperationStatus::TransactionNotFound:
                return "Transaction Not Found";      
            case OperationStatus::NoSuchObject:
                return "No Such Object"; 
            case OperationStatus::InvalidParameter:
                return "Invalid Parameter";    
            case OperationStatus::InternalError:
                return "Internal Error";                                                
        }

        return "N/A";
    }

    // 读写操作函数返回的结果 
    struct OperationResult    
    {
        /// 带消息的设定错误
        OperationResult(OperationStatus set_status) : status(set_status) { }
        /// 设置正确的返回值
        OperationResult(std::vector<sacp::Attribute> const & set_attributes) : status(OperationStatus::Ok), attributes(set_attributes) { }

        // 操作状态 - true ok - failed
        OperationStatus status;
        std::vector<sacp::Attribute> attributes;

        std::string to_string() const
        {
            if (status != OperationStatus::Ok)
            {
                return std::string(OperationStatusName(status));
            }

            std::string str = "Ok:";
            for (auto &attr : attributes)
            {
                str += attr.brief();
            }

            return str;
        }
    };

    /// @brief 最大传输数量 
    static const int MaxTransactionNum;

    SacpClient(
        /// 串口
        std::string const & serial_device, 
        /// 串口速度
        std::string const & serial_options,         
        /// 调试TCP端口
        int debug_tcp_port,
        ReportHandle report_handle = nullptr)         
        : serial_(serial_device),
        debug_tcp_("tcp-debug", "0.0.0.0", debug_tcp_port),
        serial_options_(serial_options),
        main_loop_(uv::Loop::Type::New),
        serial_stream_("serial"),
        debug_tcp_stream_("tcp-debug"),
        report_handle_(report_handle)
    {
        name_ = "sacp-" + serial_.name();
    }

    ~SacpClient() 
    {
        stop();
    }

    /// @brief 启动客户端
    /// @return 
    bool start();

    /// @brief 停止客户端
    void stop();

    /// @brief 客户端是否正在运行
    /// @return 
    bool is_running()
    {
        return started_;
    }

    /// @brief 显示一些信息
    void dump();

    /// @brief 读属性操作
    /// @param from 标记这个请求源    
    /// @param priority 优先级
    /// @param attributes 需要读的属性
    /// @return  std::unique_ptr<OperationResult> 操作结果
    std::unique_ptr<OperationResult> read_attributes(
        std::string const & from, 
        sacp::Frame::Priority priority,
        std::vector<sacp::Attribute> const & attributes);

    /// @brief 写属性操作
    /// @param from 标记这个请求源
    /// @param priority 优先级
    /// @param attributes 需要写的属性值 
    /// @return  std::unique_ptr<OperationResult> 操作结果
    std::unique_ptr<OperationResult> write_attributes(
        std::string const & from, 
        sacp::Frame::Priority priority,        
        std::vector<sacp::Attribute> const & attributes);

    /// @brief 异步读属性
    /// @param attributes 属性值 
    /// @return uint32_t 返回请求ID, 0 - 表示请示失败
    uint32_t read_attributes_async(
        std::string const & from, 
        sacp::Frame::Priority priority,
        std::vector<sacp::Attribute> attributes);

    /// @brief 异步写属性
    /// @param attributes 属性值 
    /// @return uint32_t 返回请求ID， 0 - 表示请求失败
    uint32_t write_attributes_async(
        std::string const & from, 
        sacp::Frame::Priority priority,        
        std::vector<sacp::Attribute> attributes);

    /// @brief 读取异步操作的结果
    /// @param id 请求ID
    /// @param block 是否需要阻塞
    /// @return std::unique_ptr<OperationResult> 操作结果
    std::unique_ptr<OperationResult> get_result(uint32_t id, bool block);

    /// @brief 创建一个VOFA数据监控服务
    /// @param port 
    /// @param datas 
    /// @param period 
    /// @return 
    bool create_vofa_monitor_service(int port, std::vector<uint32_t> const & datas, int period);

    /// @brief 删除VOFA数据监控服务
    /// @param port 
    void destroy_vofa_monitor_service(int port);

private:

    // 定义一次传输任务
    struct Transaction
    {    
        enum class State : int 
        {
            Pending = 0,
            WaitingAck,            
            Success,
            Failed,
        };

        static char const* StateName(State state) 
        {
            switch(state)
            {
                case State::Pending:
                    return "Pending";
                case State::WaitingAck:
                    return "WaitingAck";    
                case State::Success:
                    return "Success";
                case State::Failed:
                    return "Failed";                
            }

            return "N/A";
        }

        Transaction(
            uint32_t req_id, 
            std::unique_ptr<sacp::Frame> &frame) 
            : state(State::Pending),
            status(OperationStatus::Ok),
            request_id(req_id),
            req_frame(std::move(frame)),
            ack_frame(nullptr),            
            queue_time(naiad::system::uptime()), 
            tx_time(0), rx_time(0), end_time(0)
        {

        }

        Transaction(
            std::string const &from,
            uint32_t req_id, 
            sacp::Frame::OpCode op_code, 
            sacp::Frame::Priority priority,
            std::vector<sacp::Attribute> const & attributes) 
            : state(State::Pending),
            status(OperationStatus::Ok),
            request_id(req_id),  
            req_frame(std::make_unique<sacp::Frame>(from, priority, (uint8_t)req_id, op_code, attributes)),
            ack_frame(nullptr),
            queue_time(naiad::system::uptime()), 
            tx_time(0), rx_time(0), end_time(0)            
        {

        }

        // 传输状态
        State state;
        // 操作状态
        OperationStatus status;
        /// 请求ID
        uint32_t request_id;
        /// 请求数据
        std::unique_ptr<sacp::Frame> req_frame;
        /// 响应数据
        std::unique_ptr<sacp::Frame> ack_frame;
        /// 建立时间
        naiad::system::SysTick queue_time;
        /// 发送时间
        naiad::system::SysTick tx_time;
        /// 接收时间
        naiad::system::SysTick rx_time;
        /// 结束时间, 请示完成后，保存时间指定时长删除它
        naiad::system::SysTick end_time;
        /// 是否已过期？
        bool outdated = false;

        /// promise & future
        std::promise<OperationResult> promise;
    };

    std::string name_;
    /// 主串口
    naiad::driver::SerialPort serial_;
    /// 调试使用的TCP服务
    naiad::network::TcpServer debug_tcp_;
    /// 串口选项 
    std::string serial_options_;
    /// 主循环
    uv::Loop main_loop_;

    // 主要的线程    
    std::thread main_thread_;
    /// 状态
    bool main_exit_ = false;
    bool started_ = false;

    // sacp streams
    sacp::Stream serial_stream_;
    sacp::Stream debug_tcp_stream_;

    // 请求自增ID
    uint32_t request_id_source_;
    
    std::mutex transaction_mutex_;
    std::condition_variable transaction_sync_;    
    std::queue<std::unique_ptr<Transaction>> pending_transactions_;
    std::vector<std::unique_ptr<Transaction>> completed_transactions_;

    ReportHandle report_handle_;
    /// @brief  VOFA的数据可视化
    sacp::VofaDebuger vofa_debuger_;

    /// @brief 返回请求ID
    uint32_t get_request_id()
    {
        ++ request_id_source_;
        // 不能等于0
        if (request_id_source_ == 0)
        {
            request_id_source_ = 1;
        }
        return request_id_source_;
    }

    // 主任务
    void main_task();

    // // 主任务
    // void main_task1();

    /// @brief 发起一上请求
    /// @param from 标记这个请求源    
    /// @param priority 优先级
    /// @param op_code 操作码
    /// @param attributes 需要读的属性
    /// @param request_id 返回的请求ID
    /// @return  发送一个读请求
    _MAIN_THREAD_
    OperationStatus submit_request(
        std::string const & from, 
        sacp::Frame::Priority priority,
        sacp::Frame::OpCode op_code,
        std::vector<sacp::Attribute> const & attributes, 
        uint32_t &request_id);

    /// @brief 传输事务处理接收的帧
    _MAIN_THREAD_
    void transaction_receive(std::unique_ptr<sacp::Frame> & ptr);

    /// @brief 处理所有传输事务
    // _TX_THREAD_
    // void transaction_process();

    /// @brief 事务子任务
    _TX_THREAD_
    void transaction_task();

};

} // end sacp


#endif // __NAIAD_SACP_CLIENT_H__

