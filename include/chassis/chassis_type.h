
#include "common/logger.h"
#include "rclcpp/rclcpp.hpp"

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

#include "naiad_interfaces/msg/main_controller_info.hpp"
#include "naiad_interfaces/msg/main_controller_state.hpp"
#include "naiad_interfaces/srv/main_controller_set_time.hpp"
#include "naiad_interfaces/srv/main_controller_get_info.hpp"

namespace naiad
{
namespace chassis
{

/// 外设通用
using MsgDeviceId = naiad_interfaces::msg::DeviceId;
using MsgDeviceBreif = naiad_interfaces::msg::DeviceBrief;
using MsgAdminStatus = naiad_interfaces::msg::AdminStatus;
using MsgDeviceInfo = naiad_interfaces::msg::DeviceInfo;

/// 外设状态
using MsgFogBoxState = naiad_interfaces::msg::FogboxState;
using MsgPowerBoxState = naiad_interfaces::msg::PowerboxState;
using MsgPumpBoxState = naiad_interfaces::msg::PumpboxState;
using MsgPushBoxState = naiad_interfaces::msg::PushboxState;
using MsgLedLightState = naiad_interfaces::msg::LedlightState;
using MsgLifterState = naiad_interfaces::msg::LifterState;

/// 外设服务
using SrvPushBoxControl = naiad_interfaces::srv::PushboxControl;
using SrvPushBoxGetOfflineConfig = naiad_interfaces::srv::PushboxGetOfflineConfig;
using SrvPushBoxSetOfflineConfig = naiad_interfaces::srv::PushboxSetOfflineConfig;
using SrvLedLightSetBrightness = naiad_interfaces::srv::LedlightSetBrightness;
using SrvLifterSetPosition = naiad_interfaces::srv::LifterSetPosition;

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

// 主控消息定义
using MsgControllerInfo = naiad_interfaces::msg::MainControllerInfo;
using MsgControllerState = naiad_interfaces::msg::MainControllerState;
using SrvControllerSetTime = naiad_interfaces::srv::MainControllerSetTime;
using SrvControllerGetInfo = naiad_interfaces::srv::MainControllerGetInfo;
using SrvControllerSetTimeRequest = naiad_interfaces::srv::MainControllerSetTime_Request;
using SrvControllerSetTimeResponse = naiad_interfaces::srv::MainControllerSetTime_Response;
using SrvControllerGetInfoRequest = naiad_interfaces::srv::MainControllerGetInfo_Request;
using SrvControllerGetInfoResponse = naiad_interfaces::srv::MainControllerGetInfo_Response;


}
}
