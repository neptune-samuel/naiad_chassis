
#include <chrono>
#include <string>
#include <map>
#include <mutex>
#include <cmath>

#include "common/logger.h"
#include "common/tcp_server.h"
#include "common/uv_helper.h"


class VofaService
{
public:
    /**
     * @brief IPV4地址
     * 
     * @param ipv4_address 
     * @param ip_port
     * @param data_set 数据集
     * @param period_ms 周期，0 - 使用上报触发，> 0， 周期触发
     */
    VofaService(std::string const & ipv4_address, int ip_port, std::vector<uint32_t> data_set, int period_ms = 0)
    {
        name_ = "vofa-" + std::to_string(ip_port);

        report_period_ = period_ms;
        // 实例化一个TCP服务
        tcp_server_ = std::make_unique<naiad::network::TcpServer>(name_, ipv4_address, ip_port, 2);

        // 初始化数据
        for (auto & id : data_set)
        {
            data_cache_[id] = 0.0f;
        }

        slog::info("{}: init with {} datas", name_, data_cache_.size());
    }

    /// 启动该服务
    void start()
    {
        // 看看周期是否大于0, 如果大于，工作在定期模式
        if (report_period_ > 0)
        {
            // 绑定到LOOP
            timer_.bind(tcp_server_->get_loop());

            timer_.start(report_period_, [&]{
                    send_datas();
                });      
        }

        tcp_server_->start();
    }

    void input(std::map<uint32_t, float> const &datas)
    {
        std::size_t count = 0;

        // 更新数据
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            for (auto & kv : datas){
                if (data_cache_.count(kv.first) > 0){
                    count ++;
                    data_cache_[kv.first] = kv.second;
                }
            }
        }

        // 数据插入驱动时，只要有一个数据就上报一组数据
        if ((report_period_ <= 0) && count > 0)
        {
            send_datas();
        }
    }

    void stop()
    {
        timer_.stop();
        tcp_server_->stop();
    }

private:
    /// 服务名称
    std::string name_;    
    /// 周期
    int report_period_;
    // 指向一个TCP服务
    std::unique_ptr<naiad::network::TcpServer> tcp_server_;

    /// 数据集锁
    std::mutex data_mutex_;

    // 数据缓存
    std::map<uint32_t, float> data_cache_;

    // 创建一个定时器
    uv::Timer timer_;


    void send_datas()
    {
        if (tcp_server_->is_running() && tcp_server_->connections_num() > 0)
        {

            // VOFA使用浮点数据发送
            // 数据格式为
            // [F0][F1][F2][F3]...[END]
            // 其中 [END] 为 0x00, 0x00, 0x80, 0x7f
            float buf[32]; // 最多32个通道，4*8 = 32个字节
            std::size_t num = 0;

            {
                // 转换为字节流
                std::lock_guard<std::mutex> lock(data_mutex_); 
                for (auto & v: data_cache_)
                {
                    if (num > (sizeof(buf)/sizeof(buf[0]) - 1))
                    {
                        break;
                    }

                    buf[num ++] = v.second;
                }

                uint8_t *end = (uint8_t *)&buf[num];
                end[0] = 0x00;
                end[1] = 0x00;
                end[2] = 0x80;
                end[3] = 0x7f;

                num ++;
            }

            slog::debug_data(buf, num * sizeof(float), "{}: send {} bytes", name_, num * sizeof(float));

            // 发送报文
            tcp_server_->send(tcp_server_->AllClients, (uint8_t *)&buf[0], num * sizeof(float));
        }
    }

};




int main(int argc, const char *argv[])
{
    // 先初始化日志
    slog::make_stdout_logger("test_vofa", slog::LogLevel::Trace);

    uv::Loop loop(uv::Loop::Type::Default);

    auto signal_handle = [&]([[maybe_unused]]int signum){
            loop.stop();
        };

    loop.signal(SIGINT, signal_handle);
    loop.signal(SIGTERM, signal_handle);
    loop.signal(SIGKILL, signal_handle);

    VofaService vofa("0.0.0.0", 9700, {1, 2, 3, 4}, 0);

    vofa.start();

    uv::Timer timer;

    timer.bind(loop);

    const int k_points = 100;
    int k1 = 0;
    int k2 = 10;
    int k3 = 50;
    int k4 = 80;

    const float k_step = (2 * M_PI) / k_points;

    std::map<uint32_t, float> datas;

    timer.start(10, [&]{

            datas[1] = sin(k1 * k_step);
            datas[2] = sin(k2 * k_step);
            datas[3] = sin(k3 * k_step);
            datas[4] = sin(k4 * k_step);
            
            vofa.input(datas);


            k1 ++;
            k2 ++;
            k3 ++;
            k4 ++;

            if (k1 >= 100){
                k1 = 0;
            }
            if (k2 >= 100){
                k2 = 0;
            }
            if (k3 >= 100){
                k3 = 0;
            } 
            if (k4 >= 100){
                k4 = 0;
            }                                     
        });



    loop.spin();
    vofa.stop();

  
    slog::error("test vofa exited");

    return 0;
}




