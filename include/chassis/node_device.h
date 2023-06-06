
#ifndef __NAIAD_NODE_DEVICE_H__
#define __NAIAD_NODE_DEVICE_H__

/**
 * @file node_device.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 这是一个设备结点类模板
 * @version 0.1
 * @date 2023-06-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>

#include "common/logger.h"
#include "rclcpp/rclcpp.hpp"
#include "naiad_interfaces/msg/device_info.hpp"

namespace naiad 
{

namespace chassis
{

/// 类型重定义
using MsgDeviceId = naiad_interfaces::msg::DeviceId;
using MsgDeviceBreif = naiad_interfaces::msg::DeviceBrief;
using MsgAdminStatus = naiad_interfaces::msg::AdminStatus;
using MsgDeviceInfo = naiad_interfaces::msg::DeviceInfo;

template<typename DeviceStateType>
class NodeDevice: public rclcpp::Node 
{
public:

    enum class DeviceState
    {
        Offline = 0,
        Online = 1,
        OnlineWithAlarm = 2,
    };


    explicit NodeDevice(std::string const & type): rclcpp::Node("naiad_" + type)
    {
        type_ = type;
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        //log_ = slog::make_ros_logger(type_);        

        info_publisher_ = this->create_publisher<MsgDeviceInfo>(type + "/info", 10);
        state_publisher_ = this->create_publisher<DeviceStateType>(type + "/state", 10);
    }

    ~NodeDevice()
    {

    }

    // 返回设备类型
    std::string const & get_device_type()
    {
        return type_;
    }


    void set_device_brief(uint8_t address, MsgDeviceBreif const &brief)
    {
        // 先找一下设备，如果没有，就创建一个新的
        auto it = std::find_if(devices_.begin(), devices_.end(), [&](MsgDeviceInfo const & dev){
            return (dev.device_id.address == address);
        });

        // 如果不存在，就创建一个新的
        if (it == devices_.end())
        {
            MsgDeviceInfo dev;
            dev.device_brief = brief;       
            dev.device_id.type = type_;
            dev.device_id.address = address; 
            devices_.emplace_back(dev); 

            slog::info("New device({}-{}) added", type_, address);
        }
        else 
        {                    
            it->device_brief = brief;
            slog::debug("device-{} brief info updated", address);            
        }
    }

    void report_admin_status(uint8_t address, MsgAdminStatus const &status)
    {
        // 更新管理状态，必须是已存在设备
        auto it = std::find_if(devices_.begin(), devices_.end(), [&](MsgDeviceInfo const & dev){
            return (dev.device_id.address == address);
        });

        // 如果不存在，就创建一个新的
        if (it != devices_.end())
        {
            it->header.stamp = clock_->now();
            it->header.frame_id = type_;

            // 记录ROS层日志告警
            if (status.link_status != it->admin_status.link_status)
            {
                DeviceState state = static_cast<DeviceState>(status.link_status);

                switch(state)
                {
                    case DeviceState::Offline:
                        slog::error("{}-{} link down", type_, address);
                    break;

                    case DeviceState::Online:
                        slog::info("{}-{} link up", type_, address);
                    break;
                    case DeviceState::OnlineWithAlarm:
                        slog::warning("{}-{} link up with alaram", type_, address);
                    break;
                }                
            }

            it->admin_status = status;

            // 发布一个信息
            info_publisher_->publish(*it); 
            slog::debug("{}-{} info published", type_, address);
        }
        else
        {
            slog::warning("receive admin-status before device created(address={})", address);
        }
    }

    void report_device_state(uint8_t address, DeviceStateType &state)
    {
        // 更新设备状态，必须是已存在设备
        auto it = std::find_if(devices_.begin(), devices_.end(), [&](MsgDeviceInfo const & dev){
            return (dev.device_id.address == address);
        });

        // 如果不存在，就创建一个新的
        if (it != devices_.end())
        {
            state.header.stamp = clock_->now();
            state.header.frame_id = type_;

            state.device_id.type = type_;
            state.device_id.address = address;

            // 发布一个信息
            state_publisher_->publish(state); 
            slog::debug("{}-{} satate published", type_, address);
        }
        else
        {
            slog::warning("receive device-state before device created(address={})", address);
        }
    }        

protected:
    std::string type_;
    // 得到一个时钟
    rclcpp::Clock::SharedPtr clock_;
    // 设备实例列表    
    std::vector<MsgDeviceInfo> devices_;

    // logger
    std::shared_ptr<slog::Logger> log_;

    // 创建一个基本信息发布
    rclcpp::Publisher<MsgDeviceInfo>::SharedPtr info_publisher_;
    std::shared_ptr<rclcpp::Publisher<DeviceStateType>> state_publisher_;
};

}
}

#endif // __NAIAD_NODE_DEVICE_H__
