



#ifndef __NAIAD_NODE_MOTION_H__
#define __NAIAD_NODE_MOTION_H__

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

#include "chassis/node_device.h"
#include "robot_n1/robot_n1.h"

#include "chassis/chassis_type.h"

namespace naiad 
{
namespace chassis
{

class NodeMotion: public NodeDevice<MsgMotorState> 
{
public:
    NodeMotion(std::string const &type, std::shared_ptr<sacp::SacpClient> sacp_client) : 
        NodeDevice<MsgMotorState>(type), sacp_client_(sacp_client) 
    {
        // 初始化运动控制订阅器
        motion_control_subscriber_ = this->create_subscription<MsgMotionControl>(
            "motion/control", 10, [this](const MsgMotionControl::SharedPtr msg){
            // 处理运动控制请求
            auto result = robot::n1::set_motion_target(sacp_client_, msg->velocity, msg->angle);
            if (result->status != sacp::SacpClient::OperationStatus::Ok){
                slog::warning("set motion target failed: {}", sacp::SacpClient::OperationStatusName(result->status));
            }
        });

        //创建两个运动控制订阅器
        motion_data_publisher_ = this->create_publisher<MsgMotionData>("motion/data", 10);
        motion_odometer_publisher_ = this->create_publisher<MsgMotionOdometer>("motion/odometer", 10);
    }


    /// @brief 处理上报文
    /// @param attributes 
    void report_handle(uint8_t group, std::vector<sacp::Attribute> const & attributes) override
    {
        // 处理自己的上报报文
        switch(group) 
        {
            case REPORT_GROUP_MOTION_STATE1:
            {                
                MsgMotionData data;
                bool result = robot::n1::parse_motion_data(attributes, data);
                if (result)
                {
                    data.header.frame_id = "motion";
                    data.header.stamp = this->get_clock()->now();
                    motion_data_publisher_->publish(data);
                }
            }
            break;

            case REPORT_GROUP_MOTION_STATE2:
            {
                MsgMotionOdometer data;
                bool result = robot::n1::parse_motion_odometer(attributes, data);
                if (result)
                {
                    data.header.frame_id = "motion";
                    data.header.stamp = this->get_clock()->now();
                    motion_odometer_publisher_->publish(data);
                }
            }
            break;

            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_INFO1):  
            {
                // robot::n1::DeviceIndex index ;
                // MsgDeviceBreif brief;
                // bool result = robot::n1::parse_motor_device_brief(attributes, index, brief);
                // if (result)
                // {
                //     slog::trace("parse motor({}) info success", index);
                //     set_device_brief((uint8_t)index, brief);
                // }            
            }
            break;

            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_INFO2):  
            {
                // robot::n1::DeviceIndex index ;
                // MsgDeviceBreif brief;
                // bool result = robot::n1::parse_motor_device_brief(attributes, index, brief);
                // if (result)
                // {
                //     slog::trace("parse motor({}) info success", index);
                //     set_device_brief((uint8_t)index, brief);
                // }            
            }
            break;

            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_ADMIN_STATUS):
            {
                robot::n1::DeviceIndex index ;
                MsgAdminStatus status;
                bool result = robot::n1::parse_motor_admin_status(attributes, index, status);
                if (result)
                {
                    slog::trace("parse motor({}) admin status success", (uint8_t)index);
                    report_admin_status((uint8_t)index, status);
                }
            }        
            break;

            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_STATE):
            {
                robot::n1::DeviceIndex index ;
                MsgMotorState state;
                bool result = robot::n1::parse_motor_state(attributes, index, state);
                if (result)
                {
                    slog::trace("parse motor({}) state success", (uint8_t)index);
                    report_device_state((uint8_t)index, state);
                }
            }        
            break;
        }
    } 

    /// @brief  主动获取设备信息
    /// @param index 
    /// @param info 
    /// @return 
    bool get_device_info(uint8_t index, MsgDeviceInfo & info) override
    {        
        auto result = robot::n1::read_motor_info(sacp_client_, static_cast<robot::n1::DeviceIndex>(index), info);
        return (result->status == sacp::SacpClient::OperationStatus::Ok);        
    }    

private:

    /// SACP客户端
    std::shared_ptr<sacp::SacpClient> sacp_client_;
    // 订阅运动控制请求
    rclcpp::Subscription<MsgMotionControl>::SharedPtr motion_control_subscriber_;     
    // 运动控制反馈发布器
    rclcpp::Publisher<MsgMotionData>::SharedPtr motion_data_publisher_;
    rclcpp::Publisher<MsgMotionOdometer>::SharedPtr motion_odometer_publisher_;

};


}
}

#endif // __NAIAD_NODE_MOTION_H__


