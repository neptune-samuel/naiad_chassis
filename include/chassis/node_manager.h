
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

#include "sacp_client/sacp_client.h"
#include "chassis/node_chassis.h"
#include "chassis/node_lifter.h"
#include "chassis/node_powerbox.h"
#include "chassis/node_pushbox.h"
#include "chassis/node_pumpbox.h"
#include "chassis/node_fogbox.h"
#include "chassis/node_ledlight.h"

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

};

}
}

#endif // __NAIAD_NODE_MANAGER_H__
