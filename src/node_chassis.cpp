

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


NodeChassis::NodeChassis(std::string const &name): rclcpp::Node(name) 
{ 
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&NodeChassis::timer_handle, this));

    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("serial_options", "460800");
    this->declare_parameter("debug_tcp_port", 9600);

    this->get_parameter("serial_port", parameters_.serial_port);
    this->get_parameter("serial_options", parameters_.serial_options);
    this->get_parameter("debug_tcp_port", parameters_.debug_tcp_port);

    // 启动一个日志
    log_ = slog::make_stdout_logger(this->get_name(), slog::LogLevel::Debug);

    // 用这些参数创建一个SACP客户端
    sacp_client_ = std::make_shared<sacp::SacpClient>(parameters_.serial_port, 
        parameters_.serial_options, parameters_.debug_tcp_port, 
        std::bind(&NodeChassis::sacp_report_handle, this, std::placeholders::_1));

    info_publisher_ = this->create_publisher<MsgControllerInfo>("controller/info", 10);
    state_publisher_ = this->create_publisher<MsgControllerState>("controller/state", 10);

    // 打印一些信息
    log_->info("parameter serial_port={}", parameters_.serial_port);
    log_->info("parameter serial_options={}", parameters_.serial_options);
    log_->info("parameter debug_tcp_port={}", parameters_.debug_tcp_port); 

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

            // 打印版本
            auto version_string = [](uint32_t version) -> std::string 
            {
                char buf[32] = {};
                sprintf(buf, "V%d.%d.%d", (version >> 24) & 0xff, (version >> 16) & 0xff, (version) & 0xff);
                return std::string(buf);
            };

            slog::info("Controller info: model:{} sn:{}, software version:{}, hardware version:{}", 
                controller_info_.model, controller_info_.serial_number, 
                version_string(controller_info_.software_version),
                version_string(controller_info_.hardware_version));

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
    }

    /// 如果有指定其他上报处理函数
    if (report_handle_)
    {
        report_handle_(group, attributes);
    }
}


} // chassis
} // naiad
