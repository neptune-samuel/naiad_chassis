
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
#include <vector>
#include <future>

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

    /// 一个回调函数，更新设备信息
    typedef std::function<bool(uint8_t address, MsgDeviceInfo & info)> FunctionGetDeviceInfo;

    NodeDevice(std::string const & type, FunctionGetDeviceInfo get_device_info = nullptr): 
        rclcpp::Node("naiad_" + type), 
        type_(type), function_get_device_info_(get_device_info)
    {        
        //clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
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


    void set_device_info_update_function(FunctionGetDeviceInfo function)
    {
        function_get_device_info_ = function;
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

            {
                std::lock_guard<std::mutex> lock(devices_mutex_);
                devices_.emplace_back(dev); 
            }

            slog::info("New device({}-{}) added", type_, address);
        }
        else 
        {                    
            it->device_brief = brief;
            slog::trace("device-{} brief info updated", address);            
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
            it->header.stamp = this->get_clock()->now();
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
                        slog::info("{}-{} link up, model:{} name:{}", type_, address, it->device_brief.model, it->device_brief.name);
                    break;
                    case DeviceState::OnlineWithAlarm:
                        slog::warning("{}-{} link up with alaram", type_, address);
                    break;
                }                
            }

            it->admin_status = status;

            // 发布一个信息
            info_publisher_->publish(*it); 
            slog::trace("{}-{} info published", type_, address);
        }
        else
        {
            slog::warning("receive admin-status before created(address={})", address);
            update_device_info_async(address);
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
            state.header.stamp = this->get_clock()->now();
            state.header.frame_id = type_;

            state.device_id.type = type_;
            state.device_id.address = address;

            // 发布一个信息
            state_publisher_->publish(state); 
            slog::trace("{}-{} satate published", type_, address);
        }
        else
        {
            slog::warning("receive device-state before created(address={})", address);
            update_device_info_async(address);
        }
    }

protected:
    std::string type_;

    // 创建一个慢速定时器
    //rclcpp::TimerBase::SharedPtr timer_;

    // 互斥信号量
    std::mutex devices_mutex_; 

    // 设备实例列表    
    std::vector<MsgDeviceInfo> devices_;

    // logger
    std::shared_ptr<slog::Logger> log_;

    // 创建一个基本信息发布
    rclcpp::Publisher<MsgDeviceInfo>::SharedPtr info_publisher_;
    std::shared_ptr<rclcpp::Publisher<DeviceStateType>> state_publisher_;

    // 更新设备信息的函数
    FunctionGetDeviceInfo function_get_device_info_;

    // 异步调用结果, 不需要使用，但必须全局有效保留，不然的话异步会变成同步执行
    std::future<void> async_result_;    

    //需更新的设备信息的地址
    std::vector<uint8_t> care_device_address_;

    // 异步处理是否在运行
    bool async_running_ = false;

    /// @brief 启动设备信息同步
    /// @param address 
    void update_device_info_async(uint8_t address)
    {
        // 如果没有设置获取设备信息的函数，直接返回
        if (!function_get_device_info_){
            return ;
        }

        for (auto addr : care_device_address_){
            // 如果已在里面了，直接退出
            if (addr == address){
                return;
            }
        }

        // 将需要更新的地址加入进来
        care_device_address_.push_back(address);

        // 如果异步正在执行，不再启动
        if (!async_running_){
            async_running_ = true;

            async_result_ = std::async(std::launch::async, [&]{                
                int num = care_device_address_.size();
                //slog::debug("start fetching {} info, num={}", this->type_, num);
                int count = 0;

                for (int i = 0; i < num; i ++){
                    MsgDeviceInfo info;
                    uint8_t address = care_device_address_[i];
                    if (function_get_device_info_(address, info)){
                        set_device_brief(address, info.device_brief);
                        report_admin_status(address, info.admin_status);
                        count ++;
                    }
                }

                slog::debug("{} {} info updated", count, this->type_);

                // 清空队列 
                decltype(care_device_address_) tmp;
                tmp.swap(care_device_address_);
                
                async_running_ = false;
            });
        }
    }

};

}
}

#endif // __NAIAD_NODE_DEVICE_H__
