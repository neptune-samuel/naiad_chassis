



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
                slog::info("set motion target failed: {}", sacp::SacpClient::OperationStatusName(result->status));
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
            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_INFO1):  
            {
                // uint8_t address = 0;
                // MsgDeviceBreif brief;
                // bool result = robot::n1::parse_motor_device_brief(attributes, address, brief);
                // if (result)
                // {
                //     slog::trace("parse motor({}) info success", address);
                //     set_device_brief(address, brief);
                // }            
            }
            break;

            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_INFO2):  
            {
                // uint8_t address = 0;
                // MsgDeviceBreif brief;
                // bool result = robot::n1::parse_motor_device_brief(attributes, address, brief);
                // if (result)
                // {
                //     slog::trace("parse motor({}) info success", address);
                //     set_device_brief(address, brief);
                // }            
            }
            break;

            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_ADMIN_STATUS):
            {
                uint8_t address = 0;
                MsgAdminStatus status;
                bool result = robot::n1::parse_motor_admin_status(attributes, address, status);
                if (result)
                {
                    slog::trace("parse motor({}) admin status success", address);
                    report_admin_status(address, status);
                }
            }        
            break;

            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_STATE):
            {
                uint8_t address = 0;
                MsgMotorState state;
                bool result = robot::n1::parse_motor_state(attributes, address, state);
                if (result)
                {
                    slog::trace("parse motor({}) state success", address);
                    report_device_state(address, state);
                }
            }        
            break;
        }
    } 

    /// @brief  主动获取设备信息
    /// @param address 
    /// @param info 
    /// @return 
    bool get_device_info(uint8_t address, MsgDeviceInfo & info) override
    {        
        auto result = robot::n1::read_motor_info(sacp_client_, address, info);
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


