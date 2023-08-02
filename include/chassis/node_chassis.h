
#ifndef __NAIAD_NODE_CHASSIS_H__
#define __NAIAD_NODE_CHASSIS_H__

/**
 * @file node_chassis.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 定义一个机架类结点
 * @version 0.1
 * @date 2023-06-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "common/logger.h"
#include "sacp_client/sacp_client.h"
#include "chassis/chassis_type.h"


namespace naiad 
{

namespace chassis
{

/*
列出运行参数
$ ros2 param list
/naiad_chassis:
  debug_tcp_port
  serial_options
  serial_port
  use_sim_time

在线修改参数 
  ros2 param set /naiad_chassis serial_options 460800

启动时指定参数
 ./bin/naiad_chassis --ros-args -p serial_port:=/dev/ttyTHS0
*/

class NodeChassis:public rclcpp::Node 
{
public:

    // 参数常量
    static const std::string SerialPort; 
    static const std::string SerialOptions;
    static const std::string StdoutLogLevel;
    static const std::string DebugTcpPort;
    static const std::string DataExportAdd;
    static const std::string DataExportRemove;
    static const std::string DebugFogData;


    // 上报回调函数
    typedef std::function<void(uint8_t group, std::vector<sacp::Attribute> const & attributes)> SacpReportHandle;

    // 构造函数
    NodeChassis(std::string const &name);


    /// 返回控制器信息
    MsgControllerInfo const &get_controller_info() const
    {
        return controller_info_;
    }

    /// @brief 返回SACP客户端
    /// @return 
    std::shared_ptr<sacp::SacpClient> get_sacp_client() 
    {
        return sacp_client_;
    } 


    bool start_sacp_client(SacpReportHandle report_handle)
    {
        report_handle_ = report_handle;
        return sacp_client_->start();
    }

    void stop_sacp_client()
    {
        sacp_client_->stop();
    }

private:
    /// 日志接口
    std::shared_ptr<slog::Logger> log_;    
    /// SACP客户端
    std::shared_ptr<sacp::SacpClient> sacp_client_;  
    
    /// SACP上报处理函数
    SacpReportHandle report_handle_ = nullptr;

    /// 定时器
    rclcpp::TimerBase::SharedPtr timer_;    

    /// 主控信息发布
    rclcpp::Publisher<MsgControllerInfo>::SharedPtr info_publisher_;    
    /// 主控状态发布
    rclcpp::Publisher<MsgControllerState>::SharedPtr state_publisher_;    
    /// 获取主控信息服务
    rclcpp::Service<SrvControllerGetInfo>::SharedPtr info_service_;
    /// 获取主控信息服务
    // rclcpp::Service<SrvControllerSetTime>::SharedPtr service_set_time_;
    /// 深度传感器发布
    rclcpp::Publisher<MsgDepthData>::SharedPtr depth_data_publisher_;    

    // 异步调用结果, 不需要使用，但必须全局有效保留，不然的话异步会变成同步执行
    // std::future<void> async_task_; 
    // /// 异步任务状态
    // bool async_task_running_ = false;   

    // 控制器信息
    MsgControllerInfo controller_info_;

    // 本地缓存的FOG信息
    MsgFogData fog_data_;
    MsgFogState fog_state_;
    // 订阅FOG数据
    rclcpp::Subscription<MsgFogData>::SharedPtr fog_data_subscriber_;        
    rclcpp::Subscription<MsgFogState>::SharedPtr fog_state_subscriber_;       

    bool rtc_time_synced = false;
    bool controller_info_synced = false;

    // 定时处理函数
    void timer_handle(void);

    // 上报报文处理
    void sacp_report_handle(std::vector<sacp::Attribute> const & attributes);

};


} // chassis
} // naiad


#endif // __NAIAD_NODE_CHASSIS_H__

