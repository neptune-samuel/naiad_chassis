
#include <chrono>
#include <string>
#include <map>

#include "common/logger.h"
#include "common/tcp_server.h"
#include "common/uv_helper.h"


namespace naiad
{
namespace network 
{

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

        slog::info("{} : init with {} datas", name_, data_cache_.size());
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
        for (auto & kv : datas){
            if (data_cache_.count(kv.first) > 0){
                count ++;
                data_cache_[kv.first] = kv.second;
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
    // 数据缓存
    std::map<uint32_t, float> data_cache_;

    // 创建一个定时器
    uv::Timer timer_;


    void send_datas()
    {
        if (tcp_server_->is_running() && tcp_server_->connections_num() > 0)
        {
            // 发送数据
            // 转换为字节流

        }
    }

};





}
}
