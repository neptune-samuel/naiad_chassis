
#include <chrono>
#include <memory>

#include "common/logger.h"
#include "rclcpp/rclcpp.hpp"

#include "chassis/node_manager.h"
#include "robot_n1/robot_n1.h"


namespace naiad {

namespace chassis {


/*

这里实现一个简单的设备管理
如果收到了指定设备结点的上报信息，尝试去读取设备信息

*/


NodeManager::NodeManager(std::string const &name) 
{
    // 创建一系列结点
    node_chassis_ = std::make_shared<NodeChassis>(name);
    node_fogbox_ = std::make_shared<NodeFogBox>("fogbox");
    node_powerbox_ = std::make_shared<NodePowerBox>("powerbox");
    node_pushbox_ = std::make_shared<NodePushBox>("pushbox");
    node_pumpbox_ = std::make_shared<NodePumpBox>("pumpbox");
    node_ledlight_ = std::make_shared<NodeLedLight>("ledlight");
    node_lifter_ = std::make_shared<NodeLifter>("lifter");    

    // 获取启动参数
    NodeChassis::Parameters parameters = node_chassis_->get_boot_parameters();
    
    // 用这些参数创建一个SACP客户端
    sacp_client_ = std::make_shared<sacp::SacpClient>(parameters.serial_port, 
        parameters.serial_options, parameters.debug_tcp_port, 
        std::bind(&NodeManager::sacp_report_handle, this, std::placeholders::_1));
}


/// @brief  绑定到指定的执行器中
/// @param executor 
void NodeManager::bind(rclcpp::executors::SingleThreadedExecutor & executor)
{
    executor.add_node(node_chassis_);
    executor.add_node(node_fogbox_);
    executor.add_node(node_powerbox_);
    executor.add_node(node_pushbox_);
    executor.add_node(node_pumpbox_);
    executor.add_node(node_ledlight_);
    executor.add_node(node_lifter_);    
}

/// 启动所有
bool NodeManager::start()
{
    bool started = sacp_client_->start();

    if (!started)
    {
        slog::error("sacp client started failed!!!");
        return false;
    }

    slog::info("sacp client started success");

    // 更新设备信息的回调函数
    node_lifter_->set_device_info_update_function([this](uint8_t address, MsgDeviceInfo & info) -> bool {
            // 调用SACP读函数
            auto result = robot::n1::read_lifter_info(sacp_client_, address, info);
            return (result->status == sacp::SacpClient::OperationStatus::Ok);
        });
    node_powerbox_->set_device_info_update_function([this](uint8_t address, MsgDeviceInfo & info) -> bool {
            // 调用SACP读函数
            auto result = robot::n1::read_powerbox_info(sacp_client_, address, info);
            return (result->status == sacp::SacpClient::OperationStatus::Ok);
        });
    node_pumpbox_->set_device_info_update_function([this](uint8_t address, MsgDeviceInfo & info) -> bool {
            // 调用SACP读函数
            auto result = robot::n1::read_pumpbox_info(sacp_client_, address, info);
            return (result->status == sacp::SacpClient::OperationStatus::Ok);
        });
    node_fogbox_->set_device_info_update_function([this](uint8_t address, MsgDeviceInfo & info) -> bool {
            // 调用SACP读函数
            auto result = robot::n1::read_fogbox_info(sacp_client_, address, info);
            return (result->status == sacp::SacpClient::OperationStatus::Ok);
        });
    node_pushbox_->set_device_info_update_function([this](uint8_t address, MsgDeviceInfo & info) -> bool {
            // 调用SACP读函数
            auto result = robot::n1::read_pushbox_info(sacp_client_, address, info);
            return (result->status == sacp::SacpClient::OperationStatus::Ok);
        });                                
    node_ledlight_->set_device_info_update_function([this](uint8_t address, MsgDeviceInfo & info) -> bool {
            // 调用SACP读函数
            auto result = robot::n1::read_ledlight_info(sacp_client_, address, info);
            return (result->status == sacp::SacpClient::OperationStatus::Ok);
        });

    // 初始化结点服务
    if (!service_set_lifter_position_){
        service_set_lifter_position_ = node_lifter_->create_service<SrvLifterSetPosition>(
            node_lifter_->get_device_type() + "/set_position", [this](const std::shared_ptr<SrvLifterSetPositionRequest> req, 
                std::shared_ptr<SrvLifterSetPositionResponse> resp){

            slog::info("set lifter position, address={} position={}", req->address, req->position);
            
            auto result = robot::n1::set_lifter_position(sacp_client_, req->address, req->position);
            if (result->status == sacp::SacpClient::OperationStatus::Ok){
                resp->status = true;
                resp->status_info = "success";
            } else {
                resp->status = false;
                resp->status_info = sacp::SacpClient::OperationStatusName(result->status);
            }
        });
    }

    if (!service_set_ledlight_brightness_)
    {
        service_set_ledlight_brightness_ = node_ledlight_->create_service<SrvLedLightSetBrightness>(
            node_ledlight_->get_device_type() + "/set_brightness", [this](const std::shared_ptr<SrvLedLightSetBrightnessRequest> req, 
                std::shared_ptr<SrvLedLightSetBrightnessResponse> resp){
            slog::info("set ledlight brightness, address={} brightness={}", req->address, req->brightness);
            auto result = robot::n1::set_ledlight_brightness(sacp_client_, req->address, req->brightness);
            if (result->status == sacp::SacpClient::OperationStatus::Ok){
                resp->status = true;
                resp->status_info = "success";
            } else {
                resp->status = false;
                resp->status_info = sacp::SacpClient::OperationStatusName(result->status);
            }
        });
    }

    if (!service_pushbox_control_)
    {
        service_pushbox_control_ = node_pushbox_->create_service<SrvPushBoxControl>(
            node_pushbox_->get_device_type() + "/set_brightness", [this](const std::shared_ptr<SrvPushBoxControlRequest> req, 
                std::shared_ptr<SrvPushBoxControlResponse> resp){
            slog::info("pushbox control, address={} action={}", req->address, req->control);            
            auto result = robot::n1::set_pushbox_control(sacp_client_, req->address, req->control);
            if (result->status == sacp::SacpClient::OperationStatus::Ok){
                resp->status = true;
                resp->status_info = "success";
            } else {
                resp->status = false;
                resp->status_info = sacp::SacpClient::OperationStatusName(result->status);
            }
        });
    }

    if (!service_pushbox_get_offline_config_)
    {
        service_pushbox_get_offline_config_ = node_pushbox_->create_service<SrvPushBoxGetOfflineConfig>(
            node_pushbox_->get_device_type() + "/set_offline_config", [this](const std::shared_ptr<SrvPushBoxGetOfflineConfigRequest> req, 
                std::shared_ptr<SrvPushBoxGetOfflineConfigResponse> resp){
            slog::info("get pushbox offline config, address={}", req->address);  

            auto result = robot::n1::get_pushbox_offline_config(sacp_client_, req->address, *resp);
            if (result->status == sacp::SacpClient::OperationStatus::Ok){
                resp->status = true;
                resp->status_info = "success";
            } else {
                resp->status = false;
                resp->status_info = sacp::SacpClient::OperationStatusName(result->status);
            }
        });
    }

    if (!service_pushbox_set_offline_config_)
    {
        service_pushbox_set_offline_config_ = node_pushbox_->create_service<SrvPushBoxSetOfflineConfig>(
            node_pushbox_->get_device_type() + "/get_offline_config", [this](const std::shared_ptr<SrvPushBoxSetOfflineConfigRequest> req, 
                std::shared_ptr<SrvPushBoxSetOfflineConfigResponse> resp){
            slog::info("set pushbox offline config, address={} enable={} minute={}", req->address, req->enable, req->minute);            
            auto result = robot::n1::set_pushbox_offline_config(sacp_client_, req->address, *req);
            if (result->status == sacp::SacpClient::OperationStatus::Ok){
                resp->status = true;
                resp->status_info = "success";
            } else {
                resp->status = false;
                resp->status_info = sacp::SacpClient::OperationStatusName(result->status);
            }
        });
    }    

    return started;
}

/// 停止所有
void NodeManager::stop()
{
    sacp_client_->stop();
}

/// @brief 处理SACP上报的回调函数
/// @param attributes 
void NodeManager::sacp_report_handle(std::vector<sacp::Attribute> const & attributes)
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
        // // TODO:
        break;
        case REPORT_GROUP_SYSTEM_STATE:
        {
            MsgContrllerState state;
            bool result = robot::n1::parse_controller_status(attributes, state);
            if (result)
            {
                slog::trace("parse controller info success");                 
                //node_chassis_->report_controller_state(state);         
            }
        }  
        break;

        case REPORT_ID(_REPORT_POWERBOX_BASE, _DEVICE_INFO):  
        {
            uint8_t address = 0;
            MsgDeviceBreif brief;
            bool result = robot::n1::parse_powerbox_device_brief(attributes, address, brief);
            if (result)
            {
                slog::trace("parse powerbox({}) brief info success", address);
                node_lifter_->set_device_brief(address, brief);
            }            
        }
        break;

        case REPORT_ID(_REPORT_POWERBOX_BASE, _ADMIN_STATUS):
        {
            uint8_t address = 0;
            MsgAdminStatus status;
            bool result = robot::n1::parse_powerbox_admin_status(attributes, address, status);
            if (result)
            {
                slog::trace("parse powerbox({}) admin status success", address);
                node_powerbox_->report_admin_status(address, status);
            }
        }        
        break;

        case REPORT_ID(_REPORT_POWERBOX_BASE, _RUNNING_STATE):
        {
            uint8_t address = 0;
            MsgPowerBoxState state;
            bool result = robot::n1::parse_powerbox_device_state(attributes, address, state);
            if (result)
            {
                slog::trace("parse powerbox({}) device state success", address);
                node_powerbox_->report_device_state(address, state);
            }
        }        
        break;

        case REPORT_ID(_REPORT_PUMPBOX_BASE, _DEVICE_INFO):  
        {
            uint8_t address = 0;
            MsgDeviceBreif brief;
            bool result = robot::n1::parse_pumpbox_device_brief(attributes, address, brief);
            if (result)
            {
                slog::trace("parse pumpbox({}) brief info success", address);
                node_pumpbox_->set_device_brief(address, brief);
            }            
        }
        break;

        case REPORT_ID(_REPORT_PUMPBOX_BASE, _ADMIN_STATUS):
        {
            uint8_t address = 0;
            MsgAdminStatus status;
            bool result = robot::n1::parse_pumpbox_admin_status(attributes, address, status);
            if (result)
            {
                slog::trace("parse pumpbox({}) admin status success", address);
                node_pumpbox_->report_admin_status(address, status);
            }
        }        
        break;

        case REPORT_ID(_REPORT_PUMPBOX_BASE, _RUNNING_STATE):
        {
            uint8_t address = 0;
            MsgPumpBoxState state;
            bool result = robot::n1::parse_pumpbox_device_state(attributes, address, state);
            if (result)
            {
                slog::trace("parse pumpbox({}) device state success", address);
                node_pumpbox_->report_device_state(address, state);
            }
        }        
        break;

        case REPORT_ID(_REPORT_FOGBOX_BASE, _DEVICE_INFO):  
        {
            uint8_t address = 0;
            MsgDeviceBreif brief;
            bool result = robot::n1::parse_fogbox_device_brief(attributes, address, brief);
            if (result)
            {
                slog::trace("parse fogbox({}) brief info success", address);
                node_fogbox_->set_device_brief(address, brief);
            }            
        }
        break;

        case REPORT_ID(_REPORT_FOGBOX_BASE, _ADMIN_STATUS):
        {
            uint8_t address = 0;
            MsgAdminStatus status;
            bool result = robot::n1::parse_fogbox_admin_status(attributes, address, status);
            if (result)
            {
                slog::trace("parse fogbox({}) admin status success", address);
                node_fogbox_->report_admin_status(address, status);
            }
        }        
        break;

        case REPORT_ID(_REPORT_FOGBOX_BASE, _RUNNING_STATE):
        {
            uint8_t address = 0;
            MsgFogBoxState state;
            bool result = robot::n1::parse_fogbox_device_state(attributes, address, state);
            if (result)
            {
                slog::trace("parse fogbox({}) device state success", address);
                node_fogbox_->report_device_state(address, state);
            }
        }        
        break;
		
        case REPORT_ID(_REPORT_PUSHBOX_BASE, _DEVICE_INFO):  
        {
            uint8_t address = 0;
            MsgDeviceBreif brief;
            bool result = robot::n1::parse_pushbox_device_brief(attributes, address, brief);
            if (result)
            {
                slog::trace("parse pushbox({}) brief info success", address);
                node_pushbox_->set_device_brief(address, brief);
            }            
        }
        break;

        case REPORT_ID(_REPORT_PUSHBOX_BASE, _ADMIN_STATUS):
        {
            uint8_t address = 0;
            MsgAdminStatus status;
            bool result = robot::n1::parse_pushbox_admin_status(attributes, address, status);
            if (result)
            {
                slog::trace("parse pushbox({}) admin status success", address);
                node_pushbox_->report_admin_status(address, status);
            }
        }        
        break;

        case REPORT_ID(_REPORT_PUSHBOX_BASE, _RUNNING_STATE):
        {
            uint8_t address = 0;
            MsgPushBoxState state;
            bool result = robot::n1::parse_pushbox_device_state(attributes, address, state);
            if (result)
            {
                slog::trace("parse pushbox({}) device state success", address);
                node_pushbox_->report_device_state(address, state);
            }
        }        
        break;

        case REPORT_ID(_REPORT_LEDLIGHT_BASE, _DEVICE_INFO):  
        {
            uint8_t address = 0;
            MsgDeviceBreif brief;
            bool result = robot::n1::parse_ledlight_device_brief(attributes, address, brief);
            if (result)
            {
                slog::trace("parse ledlight({}) brief info success", address);
                node_ledlight_->set_device_brief(address, brief);
            }            
        }
        break;

        case REPORT_ID(_REPORT_LEDLIGHT_BASE, _ADMIN_STATUS):
        {
            uint8_t address = 0;
            MsgAdminStatus status;
            bool result = robot::n1::parse_ledlight_admin_status(attributes, address, status);
            if (result)
            {
                slog::trace("parse ledlight({}) admin status success", address);
                node_ledlight_->report_admin_status(address, status);
            }
        }        
        break;

        case REPORT_ID(_REPORT_LEDLIGHT_BASE, _RUNNING_STATE):
        {
            uint8_t address = 0;
            MsgLedLightState state;
            bool result = robot::n1::parse_ledlight_device_state(attributes, address, state);
            if (result)
            {
                slog::trace("parse ledlight({}) device state success", address);
                node_ledlight_->report_device_state(address, state);
            }
        }        
        break;
		        

        case REPORT_ID(_REPORT_LIFTER_BASE, _DEVICE_INFO):  
        {
            uint8_t address = 0;
            MsgDeviceBreif brief;
            bool result = robot::n1::parse_lifter_device_brief(attributes, address, brief);
            if (result)
            {
                slog::trace("parse lifter({}) brief info success", address);
                node_lifter_->set_device_brief(address, brief);
            }            
        }
        break;

        case REPORT_ID(_REPORT_LIFTER_BASE, _ADMIN_STATUS):
        {
            uint8_t address = 0;
            MsgAdminStatus status;
            bool result = robot::n1::parse_lifter_admin_status(attributes, address, status);
            if (result)
            {
                slog::trace("parse lifter({}) admin status success", address);
                node_lifter_->report_admin_status(address, status);
            }
        }        
        break;

        case REPORT_ID(_REPORT_LIFTER_BASE, _RUNNING_STATE):
        {
            uint8_t address = 0;
            MsgLifterState state;
            bool result = robot::n1::parse_lifter_device_state(attributes, address, state);
            if (result)
            {
                slog::trace("parse lifter({}) device state success", address);
                node_lifter_->report_device_state(address, state);
            }
        }        
        break;

    }
}

}
}
