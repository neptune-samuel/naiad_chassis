
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

#include "naiad_interfaces/msg/main_controller_info.hpp"
#include "naiad_interfaces/msg/main_controller_state.hpp"
#include "naiad_interfaces/srv/main_controller_set_time.hpp"
#include "naiad_interfaces/srv/main_controller_get_info.hpp"

namespace naiad 
{

namespace chassis
{

using MsgContrllerInfo = naiad_interfaces::msg::MainControllerInfo;
using MsgContrllerState = naiad_interfaces::msg::MainControllerState;

using SrvControllerSetTime = naiad_interfaces::srv::MainControllerSetTime;
using SrvControllerGetInfo = naiad_interfaces::srv::MainControllerGetInfo;

using SrvControllerSetTimeRequest = naiad_interfaces::srv::MainControllerSetTime_Request;
using SrvControllerSetTimeResponse = naiad_interfaces::srv::MainControllerSetTime_Response;
using SrvControllerGetInfoRequest = naiad_interfaces::srv::MainControllerGetInfo_Request;
using SrvControllerGetInfoResponse = naiad_interfaces::srv::MainControllerGetInfo_Response;


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

    /**
     * @brief 定义一个类内的参数类型
     * 
     */
    struct Parameters
    {
        std::string serial_port;
        std::string serial_options;
        int debug_tcp_port;
    };

    // 构造函数
    NodeChassis(std::string const &name);

    // 获取参数
    Parameters const & get_boot_parameters() const
    {
        return parameters_;
    }

    /**
     * @brief 返回SACP客户端
     * 
     * @return std::shared_ptr<sacp::SacpClient> 
     */
    std::shared_ptr<sacp::SacpClient> get_sacp_client()
    {
        return sacp_client_;
    }

    /**
     * @brief 发布控制器的状态
     * 
     * @param state 
     */
    void report_controller_state(MsgContrllerState & state);

    /**
     * @brief 获取控制器的信息
     * 
     * @param info 
     * @return true 
     * @return false 
     */
    bool get_controller_info(MsgContrllerInfo &info);


    /**
     * @brief 同步主控的RTC时间
     * 
     * @return true 
     * @return false 
     */
    bool sync_controller_rtc_time();

private:
    /// 参数
    Parameters parameters_;
    /// SACP客户端
    std::shared_ptr<sacp::SacpClient> sacp_client_;     
    /// 定时器
    //rclcpp::TimerBase::SharedPtr timer_;    
    /// 日志接口
    std::shared_ptr<slog::Logger> log_;
    /// 主控状态发布
    rclcpp::Publisher<MsgContrllerState>::SharedPtr state_publisher_;    
    /// 获取主控信息服务
    rclcpp::Service<SrvControllerGetInfo>::SharedPtr service_get_info_;
    /// 获取主控信息服务
    // rclcpp::Service<SrvControllerSetTime>::SharedPtr service_set_time_;
};


} // chassis
} // naiad


#endif // __NAIAD_NODE_CHASSIS_H__

