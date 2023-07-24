


#ifndef __ROBOT_N1_DEVICE_INDEX_H__
#define __ROBOT_N1_DEVICE_INDEX_H__

/**
 * @file device_index.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 设备索引 
 * @version 0.1
 * @date 2023-06-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "attribute_helper.h"
#include "sacp_client/sacp_client.h"
#include "chassis/chassis_type.h"
#include "local_attributes.h"




namespace robot{
namespace n1
{

enum class DeviceIndex : uint8_t
{
    SingleInstance = 0,
    PowerBox = 0,
    FogBox = 0,
    PumpBox = 0,
    PushBox = 0,
    LedLightA = 0,
    LedLightB = 1,
    LedLightC = 2,
    LedLightD = 3,
    LifterA = 0,
    LifterB = 1,
    LifterC = 2,
    LifterD = 3,
    MotorRight = 0,
    MotorLeft = 1,
    MotorFront = 2,
    MotorSteering = 3,
};


static inline bool valid_ledlight_index(DeviceIndex index)
{
    return (index >= DeviceIndex::LedLightA) && (index <= DeviceIndex::LedLightD);
}

static inline bool valid_lifter_index(DeviceIndex index)
{
    return (index >= DeviceIndex::LifterA) && (index <= DeviceIndex::LifterD);
}

static inline bool valid_motor_index(DeviceIndex index)
{
    return (index >= DeviceIndex::MotorRight) && (index <= DeviceIndex::MotorSteering);
}

static inline size_t ledlight_attribute_offset(DeviceIndex index)
{
    return ((uint8_t)index) * (ATTR_LEDLIGHT_B_SET_BRIGHTNESS_ID - ATTR_LEDLIGHT_A_SET_BRIGHTNESS_ID);
}

static inline size_t lifter_attribute_offset(DeviceIndex index)
{
    return ((uint8_t)index) * (ATTR_LIFTER_A_SET_POSITION_ID - ATTR_LIFTER_B_SET_POSITION_ID);
}

static inline size_t motor_attribute_offset(DeviceIndex index)
{
    return ((uint8_t)index) * (ATTR_MOTOR_LEFT_ADDRESS_ID - ATTR_MOTOR_RIGHT_ADDRESS_ID);
}


}
}

#endif // __ROBOT_N1_DEVICE_INDEX_H__
