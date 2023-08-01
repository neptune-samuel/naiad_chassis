



#ifndef __NAIAD_NODE_LEDLIGHT_H__
#define __NAIAD_NODE_LEDLIGHT_H__

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

class NodeLedLight: public NodeDevice<MsgLedLightState> 
{
public:
    NodeLedLight(std::string const &type, std::shared_ptr<sacp::SacpClient> sacp_client) : 
        NodeDevice<MsgLedLightState>(type, (uint8_t)robot::n1::DeviceIndex::LedLightA, (uint8_t)robot::n1::DeviceIndex::LedLightD), sacp_client_(sacp_client) 
    {
        // 创建一个服务
        service_set_brightness_ = this->create_service<SrvLedLightSetBrightness>(
            this->type_ + "/set_brightness", [this](const std::shared_ptr<SrvLedLightSetBrightnessRequest> req, 
                std::shared_ptr<SrvLedLightSetBrightnessResponse> resp){

            slog::info("set ledlight brightness, index={} brightness={}", req->index, req->brightness);
            
            auto result = robot::n1::set_ledlight_brightness(sacp_client_, (robot::n1::DeviceIndex)req->index, req->brightness);
            if (result->status == sacp::SacpClient::OperationStatus::Ok){
                resp->status = true;
                resp->status_info = "success";
            } else {
                resp->status = false;
                resp->status_info = sacp::SacpClient::OperationStatusName(result->status);
            }
        });        
    }


    /// @brief 处理上报文
    /// @param attributes 
    void report_handle(uint8_t group, std::vector<sacp::Attribute> const & attributes) override
    {
        // 处理自己的上报报文
        switch(group) 
        {
            case REPORT_ID(_REPORT_LEDLIGHT_BASE, _DEVICE_INFO):  
            {
                robot::n1::DeviceIndex index = robot::n1::DeviceIndex::LedLightA;
                MsgDeviceBreif brief;
                bool result = robot::n1::parse_ledlight_device_brief(attributes, index, brief);
                if (result)
                {
                    slog::trace("parse ledlight({}) brief info success", (uint8_t)index);
                    set_device_brief((uint8_t)index, brief);
                }            
            }
            break;

            case REPORT_ID(_REPORT_LEDLIGHT_BASE, _ADMIN_STATUS):
            {
                robot::n1::DeviceIndex index = robot::n1::DeviceIndex::LedLightA;
                MsgAdminStatus status;
                bool result = robot::n1::parse_ledlight_admin_status(attributes, index, status);
                if (result)
                {
                    slog::trace("parse ledlight({}) admin status success", (uint8_t)index);
                    report_admin_status((uint8_t)index, status);
                }
            }        
            break;

            case REPORT_ID(_REPORT_LEDLIGHT_BASE, _RUNNING_STATE):
            {
                robot::n1::DeviceIndex index = robot::n1::DeviceIndex::LedLightA;
                MsgLedLightState state;
                bool result = robot::n1::parse_ledlight_device_state(attributes, index, state);
                if (result)
                {
                    slog::trace("parse ledlight({}) device state success", (uint8_t)index);
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
        auto result = robot::n1::read_ledlight_info(sacp_client_, static_cast<robot::n1::DeviceIndex>(index), info);
        return (result->status == sacp::SacpClient::OperationStatus::Ok);        
    }    

private:

    /// SACP客户端
    std::shared_ptr<sacp::SacpClient> sacp_client_;
    // 外围设备服务
    rclcpp::Service<SrvLedLightSetBrightness>::SharedPtr service_set_brightness_;     

};


}
}

#endif // __NAIAD_NODE_LEDLIGHT_H__


