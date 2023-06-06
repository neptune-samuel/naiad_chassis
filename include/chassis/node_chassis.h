
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


    NodeChassis(std::string const &name) : rclcpp::Node(name) 
    { 
        //timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&NodeChassis::test_timer, this));
        log_ = slog::make_stdout_logger(this->get_name(), slog::LogLevel::Debug);

        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("serial_options", "460800");
        this->declare_parameter("debug_tcp_port", 9600);

        this->get_parameter("serial_port", parameters_.serial_port);
        this->get_parameter("serial_options", parameters_.serial_options);
        this->get_parameter("debug_tcp_port", parameters_.debug_tcp_port);

        log_->info("parameter serial_port={}", parameters_.serial_port);
        log_->info("parameter serial_options={}", parameters_.serial_options);
        log_->info("parameter debug_tcp_port={}", parameters_.debug_tcp_port);        
    }

    // 获取参数
    void get_boot_parameters(Parameters &parameters) const
    {
        parameters = parameters_;
    }

private:
    /// 参数
    Parameters parameters_;
    /// 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    /// 日志接口
    std::shared_ptr<slog::Logger> log_;

    // /// 测试定时器
    // void test_timer()    
    // {    
    //     // int tmp_debug_tcp_port = parameter_debug_tcp_port_;
    //     // this->get_parameter("debug_tcp_port", tmp_debug_tcp_port);

    //     // if (tmp_debug_tcp_port != parameter_debug_tcp_port_)
    //     // {
    //     //     slog::info("parameter changed: debug_tcp_port {}->{}", parameter_debug_tcp_port_, tmp_debug_tcp_port);
    //     //     parameter_debug_tcp_port_ = tmp_debug_tcp_port;
    //     // }

    //     slog::info("test timer handle");
    // }
};


} // chassis
} // naiad


#endif // __NAIAD_NODE_CHASSIS_H__

