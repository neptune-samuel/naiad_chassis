
#ifndef __NAIAD_NODE_MANAGER_H__
#define __NAIAD_NODE_MANAGER_H__

/**
 * @file node_manager.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 管理所有结点，进行业务连接
 * @version 0.1
 * @date 2023-06-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "chassis/sacp_client.h"
#include "chassis/node_chassis.h"
#include "chassis/node_peripheral.h"

namespace naiad 
{
namespace chassis
{

class NodeManager
{
public:
    NodeManager(std::string const &name);
    ~NodeManager() { }

    /// @brief  绑定到指定的执行器中
    /// @param executor 
    void bind(rclcpp::executors::SingleThreadedExecutor & executor);

    /// 启动所有
    bool start();

    /// 停止所有
    void stop();

private:
    std::shared_ptr<NodeChassis> node_chassis_;
    std::shared_ptr<NodeFogBox> node_fogbox_;
    std::shared_ptr<NodePowerBox> node_powerbox_;
    std::shared_ptr<NodePumpBox> node_pumpbox_;
    std::shared_ptr<NodePushBox> node_pushbox_;
    std::shared_ptr<NodeLedLight> node_ledlight_;
    std::shared_ptr<NodeLifter> node_lifter_;

    std::shared_ptr<sacp::SacpClient> sacp_client_; 

    // 外围设备服务
    rclcpp::Service<naiad_interfaces::srv::LifterSetPosition>::SharedPtr service_set_lifter_position_; 
    rclcpp::Service<naiad_interfaces::srv::LedlightSetBrightness>::SharedPtr service_set_ledlight_brightness_;
    rclcpp::Service<naiad_interfaces::srv::PushboxControl>::SharedPtr service_pushbox_control_;
    rclcpp::Service<naiad_interfaces::srv::PushboxSetOfflineConfig>::SharedPtr service_pushbox_set_offline_config_;
    rclcpp::Service<naiad_interfaces::srv::PushboxGetOfflineConfig>::SharedPtr service_pushbox_get_offline_config_;

    /// @brief 处理SACP上报的回调函数
    /// @param attributes 
    void sacp_report_handle(std::vector<sacp::Attribute> const & attributes);   

    /// @brief 设置升降器位置
    /// @param address 
    /// @param position 
    /// @return 
    std::unique_ptr<sacp::SacpClient::OperationResult>  set_lifter_position(uint8_t address, uint8_t position);


};


    // /// @brief 服务回调函数 - 设置升降器位置
    // /// @param req 
    // /// @param resp 
    // void on_lifter_set_position(const std::shared_ptr<SrvLifterSetPositionRequest> req, 
    //             std::shared_ptr<SrvLifterSetPositionResponse> resp);    

    // /// @brief 服务回调函数 - 设置LED的灯亮度
    // /// @param req 
    // /// @param resp 
    // void on_ledlight_set_brightness(const std::shared_ptr<SrvLedLightSetBrightnessRequest> req, 
    //             std::shared_ptr<SrvLedLightSetBrightnessResponse> resp);    

    // /// @brief 服务回调函数 - 顶出控制 
    // /// @param req 
    // /// @param resp 
    // void on_pushbox_control(const std::shared_ptr<SrvPushBoxControlRequest> req, 
    //             std::shared_ptr<SrvPushBoxControlResponse> resp);    
    // /// @brief 服务回调函数 - 离线顶出控制配置下发
    // /// @param req 
    // /// @param resp 
    // void on_pushbox_set_offline_config(const std::shared_ptr<SrvPushBoxSetOfflineConfigRequest> req, 
    //             std::shared_ptr<SrvPushBoxSetOfflineConfigResponse> resp);    

    // /// @brief 服务回调函数 - 获取离线顶出获取配置
    // /// @param req 
    // /// @param resp 
    // void on_pushbox_get_offline_config(const std::shared_ptr<SrvPushBoxGetOfflineConfigRequest> req, 
    //             std::shared_ptr<SrvPushBoxGetOfflineConfigResponse> resp);    


}
}

#endif // __NAIAD_NODE_MANAGER_H__
