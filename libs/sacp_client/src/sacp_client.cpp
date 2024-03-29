

/**
 * @file sacp_client.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 一个SACP客户端实现，支持使用TCP调试
 * @version 0.1
 * @date 2023-05-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>
#include <thread>
#include <queue>
#include <chrono>
#include <algorithm>

#include <common/logger.h>
#include <common/serial_port.h>
#include <common/sys_time.h>
#include <common/uv_helper.h>

#include <sacp_client/sacp_client.h>


namespace sacp 
{

const int SacpClient::MaxTransactionNum = 64;

/// 统计信息常量属性
/// 在mcu_framework/component/sacp/attribute/common_attributes.h中定义
const uint16_t SacpClient::ATTR_SACP_RX_BYTES = 70;  // u64
const uint16_t SacpClient::ATTR_SACP_TX_BYTES = 71;  // u64
const uint16_t SacpClient::ATTR_SACP_RX_RATE = 72;   // u32
const uint16_t SacpClient::ATTR_SACP_TX_RATE = 73;   // u32
const uint16_t SacpClient::ATTR_SACP_RX_FRAMES = 74; // u32
const uint16_t SacpClient::ATTR_SACP_TX_FRAMES = 75; // u32
const uint16_t SacpClient::ATTR_SACP_CRC_ERRORS = 76; // u32
const uint16_t SacpClient::ATTR_SACP_TX_QUEUED = 77; // u32
const uint16_t SacpClient::ATTR_SACP_TX_FAILED = 78; // u32


/// 启动
bool SacpClient::start()
{
    if (started_)
    {
        slog::warning("{}: service is running, please check", name_);
        return false;
    }

    slog::info("{}: create working thread...", name_);

    main_thread_ = std::thread(&SacpClient::main_task, this);
    main_started_ = true;
    return true;  
}


/// 停止
void SacpClient::stop()
{
    slog::trace("{}: -> stop()", name_);

    if (started_)
    {
        slog::trace("{}: -> tell sub-thread exit", name_);        
        // 先停止事务子线程, 必须放在括号里，不然锁不会退出来    
        {
            std::lock_guard<std::mutex> lock(transaction_mutex_);
            main_exit_ = true;
            transaction_sync_.notify_one();
        }            

        slog::trace("{}: -> tell main loop exit", name_);
        // 通知主loop停止
        main_loop_.async_stop();
    }

    // 有可能主线程中直接退出了，但还是回收一次资源 
    if (main_started_){
        slog::trace("{}: -> wait main loop exit", name_);
        // 等待线程回收
        main_thread_.join();
        main_started_ = false;
    }

    started_ = false;
}

/// @brief 显示统计信息
void SacpClient::dump()
{
    // 显示串口的信息
    naiad::driver::SerialStatistics stats = serial_.get_statistics();

    slog::info("- serial statistics -");
    slog::info("-  fifo peak/size : {}/{}", stats.fifo_peak_size, stats.fifo_size);
    slog::info("-  rx / drop      : {}/{}", stats.rx_bytes, stats.rx_drop_bytes);
    slog::info("-  tx             : {}", stats.tx_bytes);

}

/// @brief 异步读属性
/// @param attributes 属性值 
/// @return uint32_t 返回请求ID, 0 - 表示请示失败
uint32_t SacpClient::read_attributes_async(
    std::string const & from, 
    sacp::Frame::Priority priority,
    std::vector<sacp::Attribute> attributes)
{
    uint32_t request_id = 0;
    auto ret = submit_request(from, priority, sacp::Frame::OpCode::Read, attributes, request_id);
    if (ret != OperationStatus::Ok)
    {
        return 0;
    }

    return request_id;
}

/// @brief 异步写属性
/// @param attributes 属性值 
/// @return uint32_t 返回请求ID， 0 - 表示请求失败
uint32_t SacpClient::write_attributes_async(
    std::string const & from, 
    sacp::Frame::Priority priority,        
    std::vector<sacp::Attribute> attributes)
{
    uint32_t request_id = 0;
    auto ret = submit_request(from, priority, sacp::Frame::OpCode::Write, attributes, request_id);
    if (ret != OperationStatus::Ok)
    {
        return 0;
    }

    return request_id;
}

/// @brief 读取异步操作的结果
/// @param id 请求ID
/// @param block 是否需要阻塞
/// @return std::unique_ptr<OperationResult> 操作结果
std::unique_ptr<SacpClient::OperationResult> SacpClient::get_result(uint32_t id, bool block)
{
    // 结果处理函数
    auto get_result = [](std::unique_ptr<Transaction> &trans, bool blocking){
        if (blocking)
        {
            // 获取值 
            auto f = trans->promise.get_future();
            return std::make_unique<OperationResult>(std::move(f.get()));
        }
        else 
        {
            auto f = trans->promise.get_future();
            if (f.valid())
            {
                return std::make_unique<OperationResult>(std::move(f.get()));
            }
            else 
            {
                return std::make_unique<OperationResult>(OperationStatus::Going);
            }
        }
    };


    std::unique_lock<std::mutex> lock(transaction_mutex_);

    // 在当前帧中查找
    if (pending_transactions_.size() > 0)
    {
        auto  &trans = pending_transactions_.front();

        slog::trace("find in pending request, id:{}", trans->request_id);

        if (trans->request_id == id)
        {
            lock.unlock();
            return get_result(trans, block);
        }
    }

    for (auto & trans : completed_transactions_)
    {
        slog::trace("find in completed request, id:{}", trans->request_id);        
        if (trans->request_id == id)
        {
            lock.unlock();
            return get_result(trans, block);
        }
    }

    slog::trace("both not found");

    // 返回没有找到
    return std::make_unique<OperationResult>(OperationStatus::TransactionNotFound);
}


/// @brief 读属性操作
/// @param from 标记这个请求源    
/// @param priority 优先级
/// @param attributes 需要读的属性
/// @return  std::unique_ptr<OperationResult> 操作结果
std::unique_ptr<SacpClient::OperationResult> SacpClient::read_attributes(
    std::string const & from, 
    sacp::Frame::Priority priority,
    std::vector<sacp::Attribute> const & attributes)
{
    // 发送请求
    uint32_t request_id = 0;
    auto ret = submit_request(from, priority, sacp::Frame::OpCode::Read, attributes, request_id);
    if (ret != OperationStatus::Ok)
    {
        return std::make_unique<OperationResult>(ret);
    }

    slog::trace("get request id:{}", request_id);

    return get_result(request_id, true);
}

/// @brief 写属性操作
/// @param from 标记这个请求源
/// @param priority 优先级
/// @param attributes 需要写的属性值 
/// @return  std::unique_ptr<OperationResult> 操作结果
std::unique_ptr<SacpClient::OperationResult> SacpClient::write_attributes(
    std::string const & from, 
    sacp::Frame::Priority priority,        
    std::vector<sacp::Attribute> const & attributes)
{
    // 发送请求
    uint32_t request_id = 0;
    auto ret = submit_request(from, priority, sacp::Frame::OpCode::Write, attributes, request_id);
    if (ret != OperationStatus::Ok)
    {
        return std::make_unique<OperationResult>(ret);
    }

    // 等待结果 
    return get_result(request_id, true);
}


_MAIN_THREAD_
/// @brief 发起一上请求
/// @param from 标记这个请求源    
/// @param priority 优先级
/// @param op_code 操作码
/// @param attributes 需要读的属性
/// @param request_id 返回的请求ID
/// @return  发送一个读请求
SacpClient::OperationStatus SacpClient::submit_request(
    std::string const & from, 
    sacp::Frame::Priority priority,
    sacp::Frame::OpCode op_code,
    std::vector<sacp::Attribute> const & attributes, 
    uint32_t &request_id)
{
    // 没有属性？
    if (!attributes.size())
    {
        return OperationStatus::NoAttributes;
    }

    // 大多了？
    if (attributes.size() > sacp::Frame::MaxAttributeNum)
    {
        return OperationStatus::TooManyAttributes;
    }  

    if ((op_code != sacp::Frame::OpCode::Read) && (op_code != sacp::Frame::OpCode::Write))
    {
        return OperationStatus::OpCodeNotSupported;
    }

    {
        std::unique_lock<std::mutex> lock(transaction_mutex_);

        // 查看队列是否满了
        if (pending_transactions_.size() >= SacpClient::MaxTransactionNum)
        {
            slog::warning("Too many requests({}) pending!!!", pending_transactions_.size());
            return OperationStatus::QueueFull;
        }

        auto trans = std::make_unique<Transaction>(from, get_request_id(), op_code, priority, attributes);

        // 返回请求ID
        request_id = trans->request_id;
        slog::debug("Request({}) queued, from '{}': {}", trans->request_id, from, trans->req_frame->brief());

        pending_transactions_.emplace(std::move(trans));

        statistics_.tx_queue_frames ++;

        // 告诉发送子线程有任务来了
        transaction_sync_.notify_one();
    }

    return OperationStatus::Ok;
}


_MAIN_THREAD_
/// @brief 当主线程接收到帧时，提交给传输模块，查看是否有等待的请求
/// @param frame 需要处理的数据帧
void SacpClient::transaction_receive(std::unique_ptr<sacp::Frame> & frame)
{
    // 如果传输事务的待发送的请求有 Waiting的事务，才处理
    // reqid必须一样，
    // opcode 必须匹配
 
    // 读出头帧
    std::unique_lock<std::mutex> lock(transaction_mutex_);
    // 没有帧需要处理
    if (pending_transactions_.empty())
    {
        return ;
    }

    // 获取当前传输请求
    auto & head = pending_transactions_.front();

    // 比较序列号是否一致
    if (frame->sequence() != head->req_frame->sequence())
    {
        slog::warning("Request({}) coming frame sequence unmatch, got:{}, expect:{}", 
            head->request_id, frame->sequence(), head->req_frame->sequence());
        return ;
    }

    if (((frame->type() == sacp::Frame::OpCode::ReadAck) 
        && (head->req_frame->type() == sacp::Frame::OpCode::Read))
        || ((frame->type() == sacp::Frame::OpCode::WriteAck) 
        && (head->req_frame->type() == sacp::Frame::OpCode::Write)))
    {
        slog::trace("Reqeust({}) got response: {}", head->request_id, frame->brief());
        // got response 
        head->rx_time = frame->origin();
        head->ack_frame = std::move(frame);

        // 通知发送线程处理
        transaction_sync_.notify_one();
        return ;
    }

    // give an warning 
    slog::warning("received unexpected frame({}), drop it", frame->brief());
}


/// 主线程

/**
 * @brief 处理读写事务的子线程
 * 
 * @note 主要任务
 *  从外部线程中接收到发送请求，将发送请求转发为SACP请求原始格式，然后发送到串口
 *  然后等待响应或超时
 *  
 *  完成事务放到完成的FIFO中
 */
void SacpClient::transaction_task()
{
    while (!main_exit_)
    {
        std::unique_lock<std::mutex> lock(transaction_mutex_);

        /*
        std::condition_variable 的 wait_for 函数，该函数会阻塞当前线程，直到满足以下条件之一：

        1. 线程被唤醒，并且 pending_transactions_ 不为空。
        2. 等待时间超过指定的时间。
        */

        // 最多等10ms, 且队列不为空
        // transaction_sync_.wait_for(lock, std::chrono::milliseconds(10), [this]{
        //     return !pending_transactions_.empty();
        // });

        // 不能使用无超时版本，如果一个请求出错，会导致没有任务事件产生        
        transaction_sync_.wait_for(lock, std::chrono::milliseconds(10));

        //slog::trace("sub-thread get notify");

        if (main_exit_)
        {
            break;
        }

        // 如果队列为空，重新锁定
        if (pending_transactions_.empty())
        {
            continue;
        }

        // 检查线程是否需要退出

        // 每次只处理头结点
        // 根据头结点的状态，执行不同的过程
        // 如果是挂起状态，将它的数据发送到串口
        // 如果是等待状态，确认是否超时
        auto & head = pending_transactions_.front();

        // 旧的状态
        Transaction::State prev_state = head->state;

        switch(head->state)
        {
            case Transaction::State::Pending:
            {
                slog::debug("UART-TX: {}", head->req_frame->info());

                uint8_t frame[sacp::Frame::MaxFrameSize << 1] = { };
                std::size_t frame_size = head->req_frame->make_raw_frame(frame, sizeof(frame));
                /// 处理错误
                if ((frame_size > sacp::Frame::MaxFrameSize) || (frame_size == 0)){
                    head->state = Transaction::State::Failed;
                    head->status = (!frame_size) ? OperationStatus::MakeFrameFailed : OperationStatus::FrameSizeTooLarge;
                } else {
                    head->tx_time = naiad::system::uptime();
                    // 发送到串口
                    std::size_t ret = serial_.write(frame, frame_size);
                    if (ret != frame_size)
                    {
                        slog::warning("serial({}) write return unexpected size, got:{}, expect:{}", serial_.name(), ret, frame_size);                
                    }
                    // 切换到下一个状态
                    head->state = Transaction::State::WaitingAck;

                    statistics_.tx_frames ++;
                }
            }
            break;
            case Transaction::State::WaitingAck:
            {
                // 看看是否有响应报文进来
                if (head->ack_frame != nullptr)
                {
                    head->state = Transaction::State::Success;
                    head->status = OperationStatus::Ok;

                } else if (head->tx_time.is_after(50)){  // 检测超时时间是否到了
                    head->state = Transaction::State::Failed;
                    head->status = OperationStatus::Timeout;
                }
            }
            break;
            default:
            break;
        }

        // 是否状态改变了
        if (prev_state != head->state){
            slog::trace("Request({}) state changed: {} -> {}, operation status: {}", head->request_id, 
                Transaction::StateName(prev_state), Transaction::StateName(head->state), OperationStatusName(head->status));
        }

        // 帧结束了，将它转到结束队列中
        if ((head->state == Transaction::State::Success) || (head->state == Transaction::State::Failed)){
            // 标记完成时间
            head->end_time = naiad::system::uptime();

            if (head->state == Transaction::State::Success){
                slog::debug("Request({}) completed, takes {} ms, transfer time: {} ms", head->request_id, 
                    naiad::system::time_diff(head->end_time, head->queue_time),
                    naiad::system::time_diff(head->rx_time, head->tx_time));

                // 操作成功，设置结果
                head->promise.set_value(OperationResult(head->ack_frame->attributes()));
            } else {
                slog::debug("Request({}) completed, state:{} status: {}", head->request_id, 
                    Transaction::StateName(head->state), OperationStatusName(head->status));  

                // 操作成功，设置结果
                head->promise.set_value(OperationResult(head->status));

                // 发送失败了
                statistics_.tx_failed_frames ++;
            }        

            /// 转移到完成列表中  
            completed_transactions_.push_back(std::move(head));

            // 移除结点
            pending_transactions_.pop();
        }

        //清理已完成的传输，删除超过2秒的事务
        completed_transactions_.erase(std::remove_if(completed_transactions_.begin(), completed_transactions_.end(), 
            [](const std::unique_ptr<Transaction>& transaction) {
                // 判断元素是否满足条件
                if (transaction->end_time.is_after(2000))
                {
                    slog::debug("Request({}) outdated, remove it", transaction->request_id);                    
                    return true;
                }
                return false;
            }), completed_transactions_.end());
    }
}


/// another version of main task, use Loop
/// 
void SacpClient::main_task()
{
    // 清空缓存 
    serial_stream_.reset();
    debug_tcp_stream_.reset();

    // 启动串口
    if (!serial_.open(serial_options_.c_str()))
    {
        slog::error("{}: open serial({}) failed!!", name_, serial_.name());
        return ;
    }

    slog::info("{}: open serial port success", name_);
    
    //if (!serial_.async_read_start())
    bool ret = serial_.async_read_start(main_loop_, [this](uv::AsyncSignal::SignalId id){
        // 当串口数据准备好时
        uint8_t buf[256];

        slog::trace("get serial notify, id={}", (int)id);

        int size = serial_.async_read(buf, sizeof(buf));    
        if (size > 0){
            // 如果调试TCP端口可用，将数据发送给TCP端口
            // if (debug_tcp_.connections_num() > 0){            
            //     debug_tcp_.send(debug_tcp_.AllClients, buf, size);
            // }
            // 放进流解析器中
            serial_stream_.input(buf, size);
        }

        // 是否有解析到数据帧
        while (serial_stream_.frames_num() > 0)
        {
            auto frame = serial_stream_.receive();
            if (!frame->is_empty())
            {
                slog::debug("UART-RX: {}", frame->info());

                // 发给TCP调试端口
                if (debug_tcp_.connections_num() > 0){  
                    uint8_t buffer[sacp::Frame::MaxFrameSize << 1] = { };
                    std::size_t frame_size = frame->make_raw_frame(buffer, sizeof(buffer));
                    if (frame_size > 0){
                        debug_tcp_.send(debug_tcp_.AllClients, buffer, frame_size);
                    }
                }  

                // 如果是读写响应，提交给传输事务处理
                if (frame->type() == sacp::Frame::OpCode::Report){
                    // 调用回调函数处理上报属性
                    if (report_handle_){
                        report_handle_(frame->attributes());
                    }

                    // 将数据放在调试服务中
                    vofa_debuger_.input(frame->attributes());

                } else if ((frame->type() == sacp::Frame::OpCode::ReadAck) || (frame->type() == sacp::Frame::OpCode::WriteAck)){
                    transaction_receive(frame);
                } else if (frame->type() == sacp::Frame::OpCode::Read || frame->type() == sacp::Frame::OpCode::Write) {
                    slog::warning("receive Read or Write frame in sacp-client, frame type: {}", static_cast<uint8_t>(frame->type()));
                } else {
                    slog::warning("receive unknown frame type: {}", static_cast<uint8_t>(frame->type()));
                }              
            }
        }
    });

    if (!ret)
    {
        slog::error("{}: start serial read-thread failed", name_);
        serial_.close();
        return ;
    }

    // 启动TCP调试端口
    if (!debug_tcp_.start())
    {
        slog::error("{}: start tcp server({}) failed!!", name_, debug_tcp_.name());
        serial_.close();
        return ;
    }

    // 标记启动成功
    started_ = true;

    // TCP事件处理函数
    auto tcp_handle = [this](uv::AsyncSignal::SignalId id) {

        slog::trace("get tcp notify, id=({})", (int)id);

        // 处理接收报文
        if (id == debug_tcp_.SignalReceiveFrame){
            // 从TCP服务中接收字节流
            while (debug_tcp_.received_frames_num() > 0)
            {
                auto frame = debug_tcp_.receive();
                if (!frame.is_empty())
                {
                    // 放入流解析器中
                    debug_tcp_stream_.input(frame.data_pointer(), frame.size());
                }
            }
            // 处理解析好的数据帧
            while (debug_tcp_stream_.frames_num() > 0)
            {
                auto frame = debug_tcp_stream_.receive();
                if (!frame->is_empty())
                {
                    slog::debug("TCP-RX: {}", frame->info());
                    // 只处理读写请求
                    if (frame->type() == sacp::Frame::OpCode::Read || frame->type() == sacp::Frame::OpCode::Write) {
                        // 发送请求
                        std::unique_lock<std::mutex> lock(transaction_mutex_);
                        // 查看队列是否满了
                        if (pending_transactions_.size() >= SacpClient::MaxTransactionNum){
                            slog::warning("Too many requests({}) pending!!!", pending_transactions_.size());
                            continue;
                        }
                        // 放入队列中
                        pending_transactions_.emplace(std::make_unique<Transaction>(get_request_id(), frame));

                        statistics_.tx_queue_frames ++;

                        // 告诉发送子线程有任务来了
                        transaction_sync_.notify_one();
                    }
                }
            }
        }
    };

    // 绑定到TCP的信号处理
    debug_tcp_.signal_bind(debug_tcp_.SignalReceiveFrame, main_loop_, tcp_handle);

    main_exit_ = false;

    // 开一个子线程，处理请求事务，如读请求，写请求
    auto sub_thread = std::thread(&SacpClient::transaction_task, this);

    // 使用一个定时器，做统计分析
    uv::Timer stats_timer;
    stats_timer.bind(main_loop_, [this](){
        // 假设时间是准的
        naiad::driver::SerialStatistics serial_stats = serial_.get_statistics();
        Stream::Statistics stream_stats = serial_stream_.get_statistics();

        // 更新速率值
        statistics_.rx_rate = serial_stats.rx_bytes - statistics_.rx_bytes;
        statistics_.tx_rate = serial_stats.tx_bytes - statistics_.tx_bytes;
        statistics_.rx_bytes = serial_stats.rx_bytes;
        statistics_.tx_bytes = serial_stats.tx_bytes;

        statistics_.rx_frames = stream_stats.rx_frames;
        statistics_.crc_error_frames = stream_stats.crc_errors;


        // 如果TCP连接有效，创建数据帧并发送
        if (debug_tcp_.connections_num() > 0){
            //封装属性
            std::vector<Attribute> attrs;
            attrs.emplace_back(100, (uint8_t)255);
            attrs.emplace_back(ATTR_SACP_RX_BYTES, statistics_.rx_bytes);
            attrs.emplace_back(ATTR_SACP_TX_BYTES, statistics_.tx_bytes);
            attrs.emplace_back(ATTR_SACP_RX_RATE, statistics_.rx_rate);
            attrs.emplace_back(ATTR_SACP_TX_RATE, statistics_.tx_rate);
            attrs.emplace_back(ATTR_SACP_RX_FRAMES, statistics_.rx_frames);
            attrs.emplace_back(ATTR_SACP_TX_FRAMES, statistics_.tx_frames);
            attrs.emplace_back(ATTR_SACP_CRC_ERRORS, statistics_.crc_error_frames);
            attrs.emplace_back(ATTR_SACP_TX_FAILED, statistics_.tx_failed_frames);
            attrs.emplace_back(ATTR_SACP_TX_QUEUED, statistics_.tx_queue_frames);

            auto frame = std::make_unique<Frame>("sacp", Frame::Priority::PriorityLowest, 0, Frame::OpCode::Report, attrs);                        
            uint8_t buffer[sacp::Frame::MaxFrameSize << 1] = { };
            std::size_t frame_size = frame->make_raw_frame(buffer, sizeof(buffer));
            if (frame_size > 0){
                debug_tcp_.send(debug_tcp_.AllClients, buffer, frame_size);
            }
        }  

        // 定时打印统计信息
        if (statistics_print_tick_.is_after(60*1000)){
            // 更新时间
            statistics_print_tick_ = naiad::system::uptime();

            double rx_ok = (statistics_.rx_frames * 100.0) / (statistics_.rx_frames + statistics_.crc_error_frames);

            // 打印统计信息
            slog::info("Statistics: [rx-byte]: {}/{} [tx-byte]: {}/{}", statistics_.rx_bytes, statistics_.rx_rate, statistics_.tx_bytes, statistics_.tx_rate);
            slog::info("Statistics: [rx-frame]: {}/{} {:.3f} %%", statistics_.rx_frames, statistics_.crc_error_frames, rx_ok);
            slog::info("Statistics: [tx-frame]: {}/{} queued:{}", statistics_.tx_frames, statistics_.tx_failed_frames, statistics_.tx_queue_frames);
        }
    });

    stats_timer.start(1000);

    // 处理外部report报文
    uv::Timer report_timer;
    report_timer.bind(main_loop_, [this](){
        // 看看有没有需要发送的数据
        // 没有TCP连接，不需要处理数据
        if (debug_tcp_.connections_num() < 1){
            return;
        }

        while (!external_report_frames_.empty())
        {
            std::unique_ptr<Frame> frame = nullptr;
            {
                std::lock_guard<std::mutex> lock(external_report_frames_mutex_);
                frame = std::move(external_report_frames_.front());
                external_report_frames_.pop();
            }

            uint8_t buffer[sacp::Frame::MaxFrameSize << 1] = { };
            std::size_t frame_size = frame->make_raw_frame(buffer, sizeof(buffer));
            if (frame_size > 0){
                // 发到TCP的所有客户端
                debug_tcp_.send(debug_tcp_.AllClients, buffer, frame_size);
            }
        }
    });  

    report_timer.start(50);  

    // 进入事件等待
    main_loop_.spin();

    // 等待子进程结束 
    sub_thread.join();

    // 停止上报的定时器
    report_timer.stop();

    // 停止统计信息定时器
    stats_timer.stop();

    // 关闭TCP端口
    debug_tcp_.stop();
    // 关闭串口
    serial_.close();

    started_ = false;
}


/// @brief 创建一个VOFA数据监控服务
/// @param port 
/// @param datas 
/// @param period 
/// @return 
bool SacpClient::create_vofa_monitor_service(int port, std::vector<uint32_t> const & datas, int period = 0)
{
    return vofa_debuger_.create(port, datas, period);
}


/// @brief 删除VOFA数据监控服务
/// @param port 
void SacpClient::destroy_vofa_monitor_service(int port)
{
    return vofa_debuger_.destroy(port);
}

/// @brief 接收一个外部的report数据帧
/// @param frame
void SacpClient::external_report_frame_push(std::unique_ptr<sacp::Frame> & frame)
{
    // 没有TCP连接，不需要接收数据
    if (debug_tcp_.connections_num() < 1){
        return;
    } 

    // 必须是REPORT才可以
    if (frame->type() != Frame::OpCode::Report)
    {
        slog::warning("input frame is not a report frame");
        return;
    }

    // 入栈
    std::lock_guard<std::mutex> lock(external_report_frames_mutex_);
    // 是否限制 一下数量 
    if (external_report_frames_.size() >= 64)
    {
        slog::warning("external report frame queue full!!");
        external_report_frames_.pop();
    }

    external_report_frames_.emplace(std::move(frame));
}


} // sacp 

