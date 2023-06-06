


#ifndef __NAIAD_NODE_PERIPHERAL_H__
#define __NAIAD_NODE_PERIPHERAL_H__

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

#include "naiad_interfaces/msg/device_info.hpp"

#include "naiad_interfaces/msg/fogbox_state.hpp"
#include "naiad_interfaces/msg/powerbox_state.hpp"
#include "naiad_interfaces/msg/pumpbox_state.hpp"
#include "naiad_interfaces/msg/pushbox_state.hpp"
#include "naiad_interfaces/msg/ledlight_state.hpp"
#include "naiad_interfaces/msg/lifter_state.hpp"

#include "naiad_interfaces/srv/ledlight_set_brightness.hpp"
#include "naiad_interfaces/srv/pushbox_control.hpp"
#include "naiad_interfaces/srv/pushbox_set_offline_config.hpp"
#include "naiad_interfaces/srv/pushbox_get_offline_config.hpp"
#include "naiad_interfaces/srv/lifter_set_position.hpp"

namespace naiad 
{
namespace chassis
{

/// 类型重定义
using MsgFogBoxState = naiad_interfaces::msg::FogboxState;
using MsgPowerBoxState = naiad_interfaces::msg::PowerboxState;
using MsgPumpBoxState = naiad_interfaces::msg::PumpboxState;
using MsgPushBoxState = naiad_interfaces::msg::PushboxState;
using MsgLedLightState = naiad_interfaces::msg::LedlightState;
using MsgLifterState = naiad_interfaces::msg::LifterState;

using SrvPushBoxControlRequest = naiad_interfaces::srv::PushboxControl_Request;
using SrvPushBoxControlResponse = naiad_interfaces::srv::PushboxControl_Response;
using SrvPushBoxGetOfflineConfigRequest = naiad_interfaces::srv::PushboxGetOfflineConfig_Request;
using SrvPushBoxGetOfflineConfigResponse = naiad_interfaces::srv::PushboxGetOfflineConfig_Response;
using SrvPushBoxSetOfflineConfigRequest = naiad_interfaces::srv::PushboxSetOfflineConfig_Request;
using SrvPushBoxSetOfflineConfigResponse = naiad_interfaces::srv::PushboxSetOfflineConfig_Response;

using SrvLedLightSetBrightnessRequest = naiad_interfaces::srv::LedlightSetBrightness_Request;
using SrvLedLightSetBrightnessResponse = naiad_interfaces::srv::LedlightSetBrightness_Response;

using SrvLifterSetPositionRequest = naiad_interfaces::srv::LifterSetPosition_Request;
using SrvLifterSetPositionResponse = naiad_interfaces::srv::LifterSetPosition_Response;


using NodeFogBox = NodeDevice<MsgFogBoxState>;
using NodePowerBox = NodeDevice<MsgPowerBoxState>;
using NodePumpBox = NodeDevice<MsgPumpBoxState>;
using NodePushBox = NodeDevice<MsgPushBoxState>;
using NodeLedLight = NodeDevice<MsgLedLightState>;
using NodeLifter = NodeDevice<MsgLifterState>;


// class NodeLifter : public NodeDevice<MsgLifterState>
// {
// public:
//     typedef std::function<void(const SrvLifterSetPositionRequest & req, SrvLifterSetPositionResponse & resp)> 


//     NodeLifter(const std::string &type) : NodeDevice<MsgLifterState>(type) { }



// protected:
//     rclcpp::Service<naiad_interfaces::srv::LifterSetPosition>::SharedPtr set_position_service_; 


// };




}
}

#endif // __NAIAD_NODE_PERIPHERAL_H__

