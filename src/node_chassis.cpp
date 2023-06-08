
#include <string>
#include <chrono>

#include "common/sys_time.h"
#include "common/logger.h"

#include "rclcpp/rclcpp.hpp"
#include "chassis/node_chassis.h"
#include "robot_n1/robot_n1.h"


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

NodeChassis::NodeChassis(std::string const &name) : rclcpp::Node(name) 
{ 
    //timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&NodeChassis::test_timer, this));

    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("serial_options", "460800");
    this->declare_parameter("debug_tcp_port", 9600);

    this->get_parameter("serial_port", parameters_.serial_port);
    this->get_parameter("serial_options", parameters_.serial_options);
    this->get_parameter("debug_tcp_port", parameters_.debug_tcp_port);

    log_->info("parameter serial_port={}", parameters_.serial_port);
    log_->info("parameter serial_options={}", parameters_.serial_options);
    log_->info("parameter debug_tcp_port={}", parameters_.debug_tcp_port);  


    // 启动一个日志
    log_ = slog::make_stdout_logger(this->get_name(), slog::LogLevel::Debug);

    // 用这些参数创建一个SACP客户端
    sacp_client_ = std::make_shared<sacp::SacpClient>(parameters_.serial_port, 
        parameters_.serial_options, parameters_.debug_tcp_port, 
        std::bind(&NodeChassis::sacp_report_handle, this, std::placeholders::_1));



}


// 同步主控的RTC时间
bool NodeChassis::sync_controller_rtc_time()
{
    uint32_t time = naiad::system::now() / 1000;
    auto result = robot::n1::set_controller_rtc_time();

}


void NodeChassis::report_controller_state(MsgContrllerState & state)
{
    state.header.stamp = this->get_clock()->now();
    state.header.frame_id = "controller";
    // 发布一个信息
    state_publisher_->publish(state); 
}



bool NodeChassis::get_controller_info(MsgContrllerInfo &info)
{
    

}


void NodeChassis::sacp_report_handle(std::vector<sacp::Attribute> const & attributes)
{

}



} // chassis
} // naiad
