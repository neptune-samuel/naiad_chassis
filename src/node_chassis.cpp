

/**
 * @file node_chassis.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 机架的结点实现
 * @version 0.1
 * @date 2023-06-08
 * 
 * @copyright Copyright (c) 2023
 * 
 * 
 * @note 它干什么？
 *  定期上报主控发布上来的状态
 *  首次连接时，自动同步RTC的时间
 *  首次连接时，自动获取系统信息，并发布
 *  
 *  提供一个服务： 
 *   获取主控信息
 * 
 *  管理主控状态 TODO
 *    如果多久没有收到信息，发布主控异常事件
 */

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

/// 参数名称 - 常量
const std::string NodeChassis::SerialPort = "serial_port"; 
const std::string NodeChassis::SerialOptions = "serial_options";
const std::string NodeChassis::StdoutLogLevel = "stdout_log_level";
const std::string NodeChassis::DebugTcpPort = "debug_tcp_port";


NodeChassis::NodeChassis(std::string const &name): rclcpp::Node(name) 
{ 
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&NodeChassis::timer_handle, this));

    this->declare_parameter(SerialPort, "/dev/ttyTHS2");
    this->declare_parameter(SerialOptions, "460800");
    this->declare_parameter(StdoutLogLevel, "");
    this->declare_parameter(DebugTcpPort, 9600);

    // 启动一个日志
    // 根据参数类型，启动一个日志
    std::string const & stdout_loglevel = get_parameter(StdoutLogLevel).as_string();
    if (!stdout_loglevel.empty()){
        // 是否为合法的日志等级
        auto level = slog::log_level_from_name(stdout_loglevel);
        if (level == slog::LogLevel::None){
            std::cout << "Unknown stdout log level, use default(info)" << std::endl;
            level = slog::LogLevel::Info;
        }
        log_ = slog::make_stdout_logger(this->get_name(), level);
    } else {
        log_ = slog::make_ros_logger(this->get_name());
    }

    log_->info("parameter {}={}", SerialPort, get_parameter(SerialPort).as_string());
    log_->info("parameter {}={}", SerialOptions,  get_parameter(SerialOptions).as_string());
    log_->info("parameter {}={}", StdoutLogLevel,  get_parameter(StdoutLogLevel).as_string()); 
    log_->info("parameter {}={}", DebugTcpPort,  get_parameter(DebugTcpPort).as_int());

    // 用这些参数创建一个SACP客户端
    sacp_client_ = std::make_shared<sacp::SacpClient>(get_parameter(SerialPort).as_string(), 
        get_parameter(SerialOptions).as_string(), get_parameter(DebugTcpPort).as_int(), 
        std::bind(&NodeChassis::sacp_report_handle, this, std::placeholders::_1));

    info_publisher_ = this->create_publisher<MsgControllerInfo>("controller/info", 10);
    state_publisher_ = this->create_publisher<MsgControllerState>("controller/state", 10);
    depth_data_publisher_ = this->create_publisher<MsgDepthData>("sensor_depth/data", 10);

    // 主控的信息服务    
    info_service_ = this->create_service<SrvControllerGetInfo>(
        "controller/get_info", [this]([[maybe_unused]]const std::shared_ptr<SrvControllerGetInfoRequest> req, 
            std::shared_ptr<SrvControllerGetInfoResponse> resp){

        auto result = robot::n1::get_controller_info(sacp_client_, resp->controller);
        if (result->status == sacp::SacpClient::OperationStatus::Ok){
            resp->status = true;
            resp->status_info = "success";
        } else {
            resp->status = false;
            resp->status_info = sacp::SacpClient::OperationStatusName(result->status);
        }
    });
}


/**
 * @brief 定时器处理函数
 * 
 * @note TODO： 必须是主控已连接的情况才做这个事情 
 */
void NodeChassis::timer_handle(void)
{
    // 检查是否时间是否同步完成 
    if (!rtc_time_synced)
    {
        uint32_t rtc_time = static_cast<uint32_t>(naiad::system::now() / 1000);
        auto result = robot::n1::set_controller_rtc_time(sacp_client_, rtc_time);
        if (result->status == sacp::SacpClient::OperationStatus::Ok)
        {
            slog::info("Sync RTC time({}) to controller success", rtc_time);

            rtc_time_synced = true;
        }
    }

    if (!controller_info_synced)
    {
        auto result = robot::n1::get_controller_info(sacp_client_, controller_info_);
        if (result->status == sacp::SacpClient::OperationStatus::Ok)
        {
            //发布出去
            controller_info_.header.stamp = this->get_clock()->now();
            controller_info_.header.frame_id = "controller";
            info_publisher_->publish(controller_info_);

            slog::info("Controller info: model:{} sn:{}, software version:{}, hardware version:{}", 
                controller_info_.model, controller_info_.serial_number, 
                controller_info_.software_version, controller_info_.hardware_version);

            controller_info_synced = true;
        }
    }
}


void NodeChassis::sacp_report_handle(std::vector<sacp::Attribute> const & attributes)
{
    uint8_t group = 255;
    auto attr = sacp::get_attribute(attributes, ATTR_GROUP_ID);

    if (attr.id() == 0)
    {
        slog::warning("receive unknown report, no group id");
        return;
    }

    // 组ID
    group = attr.get_uint8();
    slog::debug("report group-{}, with {} attributes", group, attributes.size());

    switch(group)
    {
        // case REPORT_GROUP_SYSTEM_INFO:
        // TODO:
        // break;
        case REPORT_GROUP_SYSTEM_STATE:
        {
            MsgControllerState state;
            bool result = robot::n1::parse_controller_status(attributes, state);
            if (result)
            {
                slog::trace("parse controller satate success");

                state.header.stamp = this->get_clock()->now();
                state.header.frame_id = "controller";
                state_publisher_->publish(state);
            }

            // 直接返回
            return ;
        }  
        break;

        case REPORT_GROUP_DEPTH_SENSOR:
        {
            MsgDepthData data;
            bool result = robot::n1::parse_depth_sensor_data(attributes, data);
            if (result)
            {
                //slog::trace("parse depth senosr success");

                data.header.stamp = this->get_clock()->now();
                data.header.frame_id = "depth_sensor";
                depth_data_publisher_->publish(data);
            }

            // 直接返回
            return ;
        }  
        break;
        
    }

    /// 如果有指定其他上报处理函数
    if (report_handle_)
    {
        report_handle_(group, attributes);
    }
}


} // chassis
} // naiad
