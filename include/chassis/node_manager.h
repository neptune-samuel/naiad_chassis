
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
    rclcpp::Service<SrvLifterSetPosition>::SharedPtr service_set_lifter_position_; 
    rclcpp::Service<SrvLedLightSetBrightness>::SharedPtr service_set_ledlight_brightness_;
    rclcpp::Service<SrvPushBoxControl>::SharedPtr service_pushbox_control_;
    rclcpp::Service<SrvPushBoxSetOfflineConfig>::SharedPtr service_pushbox_set_offline_config_;
    rclcpp::Service<SrvPushBoxGetOfflineConfig>::SharedPtr service_pushbox_get_offline_config_;

    /// @brief 处理SACP上报的回调函数
    /// @param attributes 
    void sacp_report_handle(std::vector<sacp::Attribute> const & attributes);   

};

}
}

#endif // __NAIAD_NODE_MANAGER_H__
