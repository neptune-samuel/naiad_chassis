
#ifndef __CHASSIS_TYPE_H__
#define __CHASSIS_TYPE_H__

#include "common/logger.h"
#include "rclcpp/rclcpp.hpp"

#include "naiad_interfaces/msg/device_info.hpp"

#include "naiad_interfaces/msg/fogbox_state.hpp"
#include "naiad_interfaces/msg/powerbox_state.hpp"
#include "naiad_interfaces/msg/pumpbox_state.hpp"
#include "naiad_interfaces/msg/pushbox_state.hpp"
#include "naiad_interfaces/msg/ledlight_state.hpp"
#include "naiad_interfaces/msg/lifter_state.hpp"

#include "naiad_interfaces/srv/device_get_info.hpp"
#include "naiad_interfaces/srv/ledlight_set_brightness.hpp"
#include "naiad_interfaces/srv/pushbox_control.hpp"
#include "naiad_interfaces/srv/pushbox_set_offline_config.hpp"
#include "naiad_interfaces/srv/pushbox_get_offline_config.hpp"
#include "naiad_interfaces/srv/lifter_set_position.hpp"

#include "naiad_interfaces/msg/main_controller_info.hpp"
#include "naiad_interfaces/msg/main_controller_state.hpp"
#include "naiad_interfaces/srv/main_controller_set_time.hpp"
#include "naiad_interfaces/srv/main_controller_get_info.hpp"

#include "naiad_interfaces/msg/depth_data.hpp"

#include "naiad_interfaces/msg/motor_state.hpp"
#include "naiad_interfaces/msg/motion_control.hpp"
#include "naiad_interfaces/msg/motion_data.hpp"
#include "naiad_interfaces/msg/motion_odometer.hpp"

#include "naiad_interfaces/msg/fog_data.hpp"
#include "naiad_interfaces/msg/fog_state.hpp"

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
using SrvDeviceGetInfo = naiad_interfaces::srv::DeviceGetInfo;
using SrvPushBoxControl = naiad_interfaces::srv::PushboxControl;
using SrvPushBoxGetOfflineConfig = naiad_interfaces::srv::PushboxGetOfflineConfig;
using SrvPushBoxSetOfflineConfig = naiad_interfaces::srv::PushboxSetOfflineConfig;
using SrvLedLightSetBrightness = naiad_interfaces::srv::LedlightSetBrightness;
using SrvLifterSetPosition = naiad_interfaces::srv::LifterSetPosition;

using SrvDeviceGetInfoRequest = naiad_interfaces::srv::DeviceGetInfo_Request;
using SrvDeviceGetInfoResponse = naiad_interfaces::srv::DeviceGetInfo_Response;
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

// 深度传感器
using MsgDepthData = naiad_interfaces::msg::DepthData;

// 运动控制
using MsgMotorState = naiad_interfaces::msg::MotorState;
using MsgMotionControl = naiad_interfaces::msg::MotionControl;
using MsgMotionData = naiad_interfaces::msg::MotionData;
using MsgMotionOdometer = naiad_interfaces::msg::MotionOdometer;

using MsgFogData = naiad_interfaces::msg::FogData;
using MsgFogState = naiad_interfaces::msg::FogState;

// 16位版本转换为字串
static inline std::string version16_string(uint16_t version)
{
    char buf[32] = {};
    sprintf(buf, "V%d.%d", (version >> 8) & 0xff, (version) & 0xff);
    return std::string(buf);
}

// 32位版本ID转换为字串
static inline std::string version_string(uint32_t version)
{
    char buf[32] = {};
    sprintf(buf, "V%d.%d.%d", (version >> 24) & 0xff, (version >> 16) & 0xff, (version) & 0xff);
    return std::string(buf);
}


/// @brief 将UNIX时间转换为字串
/// @param time 
/// @return 
static inline std::string time_string(uint32_t time) 
{
    char buf[100] = { 0 };    
    std::time_t t = (std::time_t)time;
    std::tm tm = *std::localtime(&t);
    std::sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buf);
}


}
}

#endif // __CHASSIS_TYPE_H__
