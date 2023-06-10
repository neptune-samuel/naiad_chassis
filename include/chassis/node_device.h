
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
#include "chassis/sacp_client.h"

#include "rclcpp/rclcpp.hpp"
#include "chassis/chassis_type.h"

namespace naiad 
{
namespace chassis
{


/// @brief  一个模板设备类
/// @tparam DeviceStateType 
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

    NodeDevice(std::string const & type): 
        rclcpp::Node("naiad_" + type), 
        type_(type)
    {        

        info_publisher_ = this->create_publisher<MsgDeviceInfo>(type + "/info", 10);
        state_publisher_ = this->create_publisher<DeviceStateType>(type + "/state", 10);
    }

    ~NodeDevice()
    {

    }

    /// @brief 处理上报文
    /// @param attributes 
    virtual void report_handle([[maybe_unused]]uint8_t group, [[maybe_unused]]std::vector<sacp::Attribute> const & attributes)
    {
        // TODO
    } 

    /// @brief  主动获取设备信息
    /// @param address 
    /// @param info 
    /// @return 
    virtual bool get_device_info([[maybe_unused]]uint8_t address, [[maybe_unused]]MsgDeviceInfo & info)
    {
        return false;
    }
       
protected:

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
                //std::lock_guard<std::mutex> lock(devices_mutex_);
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

    // 上报设备的状态
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

    /// @brief 启动设备信息同步
    /// @param address 
    void update_device_info_async(uint8_t address)
    {
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
                    if (get_device_info(address, info)){
                        set_device_brief(address, info.device_brief);
                        report_admin_status(address, info.admin_status);
                        count ++;
                    }
                }

                slog::debug("{} {} info updated", count, this->type_);

                // 清空队列 
                decltype(care_device_address_)().swap(care_device_address_);
                
                async_running_ = false;
            });
        }
    }
protected:
    // 类型名称
    std::string type_;

    // logger
    //std::shared_ptr<slog::Logger> log_;

    // 创建一个慢速定时器
    //rclcpp::TimerBase::SharedPtr timer_;

    // 互斥信号量
    //std::mutex devices_mutex_; 

    // 设备实例列表    
    std::vector<MsgDeviceInfo> devices_;

    // 创建一个基本信息发布
    rclcpp::Publisher<MsgDeviceInfo>::SharedPtr info_publisher_;
    std::shared_ptr<rclcpp::Publisher<DeviceStateType>> state_publisher_;

    // 异步调用结果, 不需要使用，但必须全局有效保留，不然的话异步会变成同步执行
    std::future<void> async_result_;    

    //需更新的设备信息的地址
    std::vector<uint8_t> care_device_address_;

    // 异步处理是否在运行
    bool async_running_ = false;

};

}
}

#endif // __NAIAD_NODE_DEVICE_H__
