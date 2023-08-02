

/**
 * @file fog_report.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 处理FOG数据转发
 * @version 0.1
 * @date 2023-08-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "local_attributes.h"
#include "local_groups.h"
#include "attribute_helper.h"
#include "sacp_client/sacp_client.h"
#include "chassis/chassis_type.h"


namespace robot{
namespace n1
{

/**
 * @brief 生成一个FOG数据的Report帧
 * 
 * @param fog 
 * @return std::unique_ptr<sacp::Frame> 
 */
std::unique_ptr<sacp::Frame> make_fog_data_report(naiad::chassis::MsgFogData & fog)
{
    std::vector<sacp::Attribute> attrs = {
        sacp::Attribute(ATTR_GROUP_ID, (uint8_t)REPORT_GROUP_FOG_DATA),        
        ATTR_FOG_GYROSCOPE_X(fog.gyroscope[0]),
        ATTR_FOG_GYROSCOPE_Y(fog.gyroscope[1]),
        ATTR_FOG_GYROSCOPE_Z(fog.gyroscope[2]),
        ATTR_FOG_ACCELEROMETER_X(fog.accelerometer[0]),
        ATTR_FOG_ACCELEROMETER_X(fog.accelerometer[1]),
        ATTR_FOG_ACCELEROMETER_X(fog.accelerometer[2]),
        ATTR_FOG_EULAR_ROLL(fog.roll),
        ATTR_FOG_EULAR_YAW(fog.yaw),
        ATTR_FOG_EULAR_PITCH(fog.pitch),
        ATTR_FOG_QUATERNION_A(fog.quaternion[0]),
        ATTR_FOG_QUATERNION_B(fog.quaternion[1]),
        ATTR_FOG_QUATERNION_C(fog.quaternion[2]),
        ATTR_FOG_QUATERNION_D(fog.quaternion[3])
    };

    // 读取指定属性
    return std::make_unique<sacp::Frame>("ros", sacp::Frame::Priority::PriorityLowest, 0, sacp::Frame::OpCode::Report, attrs); 
}


/**
 * @brief 生成一个FOG状态的Report帧
 * 
 * @param fog 
 * @return std::unique_ptr<sacp::Frame> 
 */
std::unique_ptr<sacp::Frame> make_fog_state_report(naiad::chassis::MsgFogState & fog)
{
    std::vector<sacp::Attribute> attrs = {
        sacp::Attribute(ATTR_GROUP_ID, (uint8_t)REPORT_GROUP_FOG_STATE),
        ATTR_FOG_ACTIVED(fog.actived),
        ATTR_FOG_VERSION(fog.version),
        ATTR_FOG_SERIAL_NUMBER(fog.serial_number),
        ATTR_FOG_WORK_MODE(fog.work_mode),
        ATTR_FOG_SELF_CHECKING(fog.self_checking),
        ATTR_FOG_TMPERATURE(fog.temperature),
        ATTR_FOG_MANAUAL_CALIBRATION(fog.manaual_calibration)
    };

    // 读取指定属性
    return std::make_unique<sacp::Frame>("ros", sacp::Frame::Priority::PriorityLowest, 0, sacp::Frame::OpCode::Report, attrs); 
}

}
}
