
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

    auto sacp_client = node_chassis_->get_sacp_client();

    node_fogbox_ = std::make_shared<NodeFogBox>("fogbox", sacp_client);
    node_powerbox_ = std::make_shared<NodePowerBox>("powerbox", sacp_client);
    node_pushbox_ = std::make_shared<NodePushBox>("pushbox", sacp_client);
    node_pumpbox_ = std::make_shared<NodePumpBox>("pumpbox", sacp_client);
    node_ledlight_ = std::make_shared<NodeLedLight>("ledlight", sacp_client);
    node_lifter_ = std::make_shared<NodeLifter>("lifter", sacp_client);   
    node_motion_ = std::make_shared<NodeMotion>("motor", sacp_client);    

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
    executor.add_node(node_motion_);    
}

/// 启动SACP客户端
bool NodeManager::start()
{
    return node_chassis_->start_sacp_client([&](uint8_t group, std::vector<sacp::Attribute> const & attributes){
        // 根据上报的数据类型，分发给不同的节点
        switch(group)
        {
            case REPORT_GROUP_MOTION_STATE1:
            case REPORT_GROUP_MOTION_STATE2:
            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_INFO1):  
            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_INFO2):  
            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_ADMIN_STATUS):
            case REPORT_ID(_REPORT_MOTOR_BASE, _MOTOR_STATE):
                node_motion_->report_handle(group, attributes);      
            break;

            case REPORT_ID(_REPORT_POWERBOX_BASE, _DEVICE_INFO):  
            case REPORT_ID(_REPORT_POWERBOX_BASE, _ADMIN_STATUS):
            case REPORT_ID(_REPORT_POWERBOX_BASE, _RUNNING_STATE):        
                node_powerbox_->report_handle(group, attributes);      
            break;

            case REPORT_ID(_REPORT_PUMPBOX_BASE, _DEVICE_INFO):  
            case REPORT_ID(_REPORT_PUMPBOX_BASE, _ADMIN_STATUS):
            case REPORT_ID(_REPORT_PUMPBOX_BASE, _RUNNING_STATE):
                node_pumpbox_->report_handle(group, attributes);      
            break;            
            case REPORT_ID(_REPORT_FOGBOX_BASE, _DEVICE_INFO):  
            case REPORT_ID(_REPORT_FOGBOX_BASE, _ADMIN_STATUS):
            case REPORT_ID(_REPORT_FOGBOX_BASE, _RUNNING_STATE):
                node_fogbox_->report_handle(group, attributes);      
            break;        
            case REPORT_ID(_REPORT_PUSHBOX_BASE, _DEVICE_INFO):  
            case REPORT_ID(_REPORT_PUSHBOX_BASE, _ADMIN_STATUS):
            case REPORT_ID(_REPORT_PUSHBOX_BASE, _RUNNING_STATE):
                node_pushbox_->report_handle(group, attributes);      
            break;
            case REPORT_ID(_REPORT_LEDLIGHT_BASE, _DEVICE_INFO):  
            case REPORT_ID(_REPORT_LEDLIGHT_BASE, _ADMIN_STATUS):
            case REPORT_ID(_REPORT_LEDLIGHT_BASE, _RUNNING_STATE):
                node_ledlight_->report_handle(group, attributes);      
            break;            
            case REPORT_ID(_REPORT_LIFTER_BASE, _DEVICE_INFO):  
            case REPORT_ID(_REPORT_LIFTER_BASE, _ADMIN_STATUS):
            case REPORT_ID(_REPORT_LIFTER_BASE, _RUNNING_STATE):
                node_lifter_->report_handle(group, attributes);      
            break;
        }
    });
}

/// 停止所有
void NodeManager::stop()
{
    // 停止 SACP客户端
    node_chassis_->stop_sacp_client();
}

}
}
