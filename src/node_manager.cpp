
#include <chrono>
#include <memory>

#include "common/logger.h"
#include "rclcpp/rclcpp.hpp"

#include "chassis/node_manager.h"
#include "chassis/robot_n1_attributes.h"


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
    NodeChassis::Parameters parameters;
    node_chassis_->get_boot_parameters(parameters);
    
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

    // 初始化结点服务
    if (!service_set_lifter_position_)
    {
        service_set_lifter_position_ = node_lifter_->create_service<naiad_interfaces::srv::LifterSetPosition>(
            node_lifter_->get_device_type() + "/set_position", [this](const std::shared_ptr<SrvLifterSetPositionRequest> req, 
                std::shared_ptr<SrvLifterSetPositionResponse> resp){

            slog::info("set lifter position, address={} position={}", req->address, req->position);

            auto result = set_lifter_position(req->address, req->position);
            if (result->status == sacp::SacpClient::OperationStatus::Ok)
            {
                resp->status = true;
                resp->detail = "success";
            }
            else 
            {
                resp->status = false;
                resp->detail = sacp::SacpClient::OperationStatusName(result->status);
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
    slog::debug("report: {} attributes", attributes.size());


}


    /// @brief 设置升降器位置
/// @param address 
/// @param position 
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult>  NodeManager::set_lifter_position(uint8_t address, uint8_t position)
{

    if (!sacp_client_->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    // TODO: 从profile获取升降器的数量 
    // if (address < 1 || address > 4)
    // {
    //     return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);
    // }

    if (position > 100)
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InvalidParameter);
    }

    // 同步请求
    switch(address)
    {
        case 1:
        return sacp_client_->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
                ATTR_LIFTER_A_SET_POSITION(position)
            });        
        case 2:
        return sacp_client_->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
                ATTR_LIFTER_B_SET_POSITION(position)
            });
        case 3:
        return sacp_client_->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
                ATTR_LIFTER_C_SET_POSITION(position)
            });
        case 4:
        return sacp_client_->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
                ATTR_LIFTER_D_SET_POSITION(position)
            });        
    }
    
    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);    
}

// /// @brief 服务回调函数 - 设置升降器位置
// /// @param req 
// /// @param resp 
// void on_lifter_set_position(const std::shared_ptr<SrvLifterSetPositionRequest> req, 
//             std::shared_ptr<SrvLifterSetPositionResponse> resp)
// {


// }

// /// @brief 服务回调函数 - 设置LED的灯亮度
// /// @param req 
// /// @param resp 
// void on_ledlight_set_brightness(const std::shared_ptr<SrvLedLightSetBrightnessRequest> req, 
//             std::shared_ptr<SrvLedLightSetBrightnessResponse> resp)
// {

// }         

// /// @brief 服务回调函数 - 顶出控制 
// /// @param req 
// /// @param resp 
// void on_pushbox_control(const std::shared_ptr<SrvPushBoxControlRequest> req, 
//             std::shared_ptr<SrvPushBoxControlResponse> resp)
// {


// }   

// /// @brief 服务回调函数 - 离线顶出控制配置下发
// /// @param req 
// /// @param resp 
// void on_pushbox_set_offline_config(const std::shared_ptr<SrvPushBoxSetOfflineConfigRequest> req, 
//             std::shared_ptr<SrvPushBoxSetOfflineConfigResponse> resp)
// {


// }

// /// @brief 服务回调函数 - 获取离线顶出获取配置
// /// @param req 
// /// @param resp 
// void on_pushbox_get_offline_config(const std::shared_ptr<SrvPushBoxGetOfflineConfigRequest> req, 
//             std::shared_ptr<SrvPushBoxGetOfflineConfigResponse> resp)        
// {

// }   


}
}
