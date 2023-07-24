



#ifndef __NAIAD_NODE_PUSHBOX_H__
#define __NAIAD_NODE_PUSHBOX_H__

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

class NodePushBox: public NodeDevice<MsgPushBoxState> 
{
public:
    NodePushBox(std::string const &type, std::shared_ptr<sacp::SacpClient> sacp_client) : 
        NodeDevice<MsgPushBoxState>(type), sacp_client_(sacp_client) 
    {
        // 创建一个服务
        service_control_ = this->create_service<SrvPushBoxControl>(
            this->type_ + "/push_action", [this](const std::shared_ptr<SrvPushBoxControlRequest> req, 
                std::shared_ptr<SrvPushBoxControlResponse> resp){
            slog::info("pushbox control, index={} action={}", req->index, req->control);            
            auto result = robot::n1::set_pushbox_control(sacp_client_, (robot::n1::DeviceIndex)req->index, req->control);
            if (result->status == sacp::SacpClient::OperationStatus::Ok){
                resp->status = true;
                resp->status_info = "success";
            } else {
                resp->status = false;
                resp->status_info = sacp::SacpClient::OperationStatusName(result->status);
            }
        });

        service_get_offline_config_ = this->create_service<SrvPushBoxGetOfflineConfig>(
            this->type_ + "/set_offline_config", [this](const std::shared_ptr<SrvPushBoxGetOfflineConfigRequest> req, 
                std::shared_ptr<SrvPushBoxGetOfflineConfigResponse> resp){
            slog::info("get pushbox offline config, index={}", req->index);  

            auto result = robot::n1::get_pushbox_offline_config(sacp_client_, (robot::n1::DeviceIndex)req->index, *resp);
            if (result->status == sacp::SacpClient::OperationStatus::Ok){
                resp->status = true;
                resp->status_info = "success";
            } else {
                resp->status = false;
                resp->status_info = sacp::SacpClient::OperationStatusName(result->status);
            }
        });

        service_set_offline_config_ = this->create_service<SrvPushBoxSetOfflineConfig>(
            this->type_ + "/get_offline_config", [this](const std::shared_ptr<SrvPushBoxSetOfflineConfigRequest> req, 
                std::shared_ptr<SrvPushBoxSetOfflineConfigResponse> resp){
            slog::info("set pushbox offline config, index={} enable={} minute={}", req->index, req->enable, req->minute);            
            auto result = robot::n1::set_pushbox_offline_config(sacp_client_, (robot::n1::DeviceIndex)req->index, *req);
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
            case REPORT_ID(_REPORT_PUSHBOX_BASE, _DEVICE_INFO):  
            {
                robot::n1::DeviceIndex index = robot::n1::DeviceIndex::PushBox;
                MsgDeviceBreif brief;
                bool result = robot::n1::parse_pushbox_device_brief(attributes, index, brief);
                if (result)
                {
                    slog::trace("parse pushbox({}) brief info success", (uint8_t)index);
                    set_device_brief((uint8_t)index, brief);
                }            
            }
            break;

            case REPORT_ID(_REPORT_PUSHBOX_BASE, _ADMIN_STATUS):
            {
                robot::n1::DeviceIndex index = robot::n1::DeviceIndex::PushBox;
                MsgAdminStatus status;
                bool result = robot::n1::parse_pushbox_admin_status(attributes, index, status);
                if (result)
                {
                    slog::trace("parse pushbox({}) admin status success", (uint8_t)index);
                    report_admin_status((uint8_t)index, status);
                }
            }        
            break;

            case REPORT_ID(_REPORT_PUSHBOX_BASE, _RUNNING_STATE):
            {
                robot::n1::DeviceIndex index = robot::n1::DeviceIndex::PushBox;
                MsgPushBoxState state;
                bool result = robot::n1::parse_pushbox_device_state(attributes, index, state);
                if (result)
                {
                    slog::trace("parse pushbox({}) device state success", (uint8_t)index);
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
        auto result = robot::n1::read_pushbox_info(sacp_client_, static_cast<robot::n1::DeviceIndex>(index), info);
        return (result->status == sacp::SacpClient::OperationStatus::Ok);        
    }    

private:

    /// SACP客户端
    std::shared_ptr<sacp::SacpClient> sacp_client_;
    // 外围设备服务
    rclcpp::Service<SrvPushBoxControl>::SharedPtr service_control_;
    rclcpp::Service<SrvPushBoxSetOfflineConfig>::SharedPtr service_set_offline_config_;
    rclcpp::Service<SrvPushBoxGetOfflineConfig>::SharedPtr service_get_offline_config_;

};


}
}

#endif // __NAIAD_NODE_PUSHBOX_H__


