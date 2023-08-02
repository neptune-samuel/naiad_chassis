
/**
 * @file motion_request.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 运动控制相关操作
 * @version 0.1
 * @date 2023-07-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "local_attributes.h"
#include "attribute_helper.h"
#include "sacp_client/sacp_client.h"
#include "chassis/chassis_type.h"
#include "device_index.h"


namespace robot{
namespace n1
{



/**
 * @brief 解析电机的设备简介
 * 
 * @param attrs 收到的属性
 * @param index 返回的索引
 * @param info 返回的信息
 * @return true 
 * @return false 
 */
bool parse_motor_info1(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgDeviceBreif &info)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_MOTOR_RIGHT_ADDRESS(0),         
        ATTR_MOTOR_RIGHT_MODEL(""), 		     
        ATTR_MOTOR_RIGHT_SN(""),		 
        ATTR_MOTOR_RIGHT_NAME("")
    };

    size_t offset = 0;
    size_t failed = 0;

    if (!parse_attributes_range(attrs, ATTR_MOTOR_RIGHT_MODEL_ID, ATTR_MOTOR_LEFT_MODEL_ID, offset, index))
    {
        return false;
    }    

    uint8_t address = get_attribute(attrs, failed, pattern[0], offset).get_uint8();
    info.address_info = std::to_string(address);   
    info.model = get_attribute(attrs, failed, pattern[1], offset).get_string();
    info.serial_number = get_attribute(attrs, failed, pattern[2], offset).get_string();  
    info.name = get_attribute(attrs, failed, pattern[3], offset).get_string(); 
    
    return (failed == 0);
}


/**
 * @brief 解析电机的设备简介
 * 
 * @param attrs 收到的属性
 * @param index 返回的索引
 * @param info 返回的信息
 * @return true 
 * @return false 
 */
bool parse_motor_info2(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgDeviceBreif &info)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_MOTOR_RIGHT_ADDRESS(0),
        ATTR_MOTOR_RIGHT_HW_VERSION(""),
        ATTR_MOTOR_RIGHT_SW_VERSION(""),
    };

    size_t offset = 0;
    size_t failed = 0;

    if (!parse_attributes_range(attrs, ATTR_MOTOR_RIGHT_ADDRESS_ID, ATTR_MOTOR_LEFT_ADDRESS_ID, offset, index))
    {
        return false;
    }   

    uint8_t address = get_attribute(attrs, failed, pattern[0], offset).get_uint8();
    info.address_info = std::to_string(address);   
    info.hardware_version = get_attribute(attrs, failed, pattern[1], offset).get_string();      
    info.software_version = get_attribute(attrs, failed, pattern[2], offset).get_string();
    
    return (failed == 0);
}

/**
 * @brief 解析电机的管理状态
 * 
 * @param attrs 收到的属性
 * @param index 返回的索引
 * @param status 返回的信息
 * @return true 
 * @return false 
 */
bool parse_motor_admin_status(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgAdminStatus &status)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_MOTOR_RIGHT_LINK_STATUS(0), 	
        ATTR_MOTOR_RIGHT_CONNECTED_TIME(0), 	
        ATTR_MOTOR_RIGHT_DISCONNECTED_TIME(0)
    };

    size_t offset = 0;
    size_t failed = 0;

    if (!parse_attributes_range(attrs, ATTR_MOTOR_RIGHT_LINK_STATUS_ID, ATTR_MOTOR_LEFT_LINK_STATUS_ID, offset, index))
    {
        return false;
    }   

    status.link_status = get_attribute(attrs, failed, pattern[0], offset).get_uint8(); 
    status.connected_time = get_attribute(attrs, failed, pattern[1], offset).get_uint32(); 
    status.disconnected_time = get_attribute(attrs, failed, pattern[2], offset).get_uint32(); 

    return (failed == 0);
}

/**
 * @brief 解析电机的设备状态信息
 * 
 * @param attrs 
 * @param index 
 * @param state 
 * @return true 
 * @return false 
 */
bool parse_motor_state(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgMotorState &state)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_MOTOR_RIGHT_NODE_STATE(0),
        ATTR_MOTOR_RIGHT_CURRENT(0),
        ATTR_MOTOR_RIGHT_RAW_SPEED(0),
        ATTR_MOTOR_RIGHT_RAW_POSITION(0),
        ATTR_MOTOR_RIGHT_RAW_TARGET_SPEED(0),
        ATTR_MOTOR_RIGHT_RAW_TARGET_POSITION(0),
        ATTR_MOTOR_RIGHT_EXTRA_ENCODER_POSITION(0),
        ATTR_MOTOR_RIGHT_EXTRA_ENCODER_STATE(0),
        ATTR_MOTOR_RIGHT_STATUS_CODE(0),
        ATTR_MOTOR_RIGHT_EMERGENCY_ERROR_CODE(0),
        ATTR_MOTOR_RIGHT_EMERGENCY_STATUS_CODE(0),
        ATTR_MOTOR_RIGHT_ERROR_STATUS(0),                        
    };

    size_t offset = 0;
    size_t failed = 0;

    if (!parse_attributes_range(attrs, ATTR_MOTOR_RIGHT_CURRENT_ID, ATTR_MOTOR_LEFT_CURRENT_ID, offset, index))
    {
        return false;
    }   

    state.node_state = get_attribute(attrs, failed, pattern[0], offset).get_uint8(); 
    state.current = get_attribute(attrs, failed, pattern[1], offset).get_float(); 
    state.speed = get_attribute(attrs, failed, pattern[2], offset).get_float(); 
    state.position = get_attribute(attrs, failed, pattern[3], offset).get_int32(); 
    state.target_speed = get_attribute(attrs, failed, pattern[4], offset).get_float();     
    state.target_position = get_attribute(attrs, failed, pattern[5], offset).get_int32(); 
    state.extra_encoder_position = get_attribute(attrs, failed, pattern[6], offset).get_int32(); 
    state.extra_encoder_state = get_attribute(attrs, failed, pattern[7], offset).get_uint8();   
    state.status_code = get_attribute(attrs, failed, pattern[8], offset).get_uint32();   
    state.emergency_error_code = get_attribute(attrs, failed, pattern[9], offset).get_uint16(); 
    state.emergency_error_status = get_attribute(attrs, failed, pattern[10], offset).get_uint8(); 
    state.error_status = get_attribute(attrs, failed, pattern[11], offset).get_uint32(); 

    return (failed == 0);
}


/// @brief 读取电机的信息
/// @param client 
/// @param address 
/// @param info 需要返回的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> read_motor_info(
    std::shared_ptr<sacp::SacpClient> client, 
    DeviceIndex index, naiad::chassis::MsgDeviceInfo & info)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    if (!valid_motor_index(index))
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);
    }

    std::vector<sacp::Attribute> attrs1 = 
    {
        ATTR_MOTOR_RIGHT_ADDRESS(0),         
        ATTR_MOTOR_RIGHT_MODEL(""), 		     
        ATTR_MOTOR_RIGHT_SN(""),		 
        ATTR_MOTOR_RIGHT_NAME("")
    };

    std::vector<sacp::Attribute> attrs2 = 
    {
        ATTR_MOTOR_RIGHT_ADDRESS(0),
        ATTR_MOTOR_RIGHT_HW_VERSION(""),
        ATTR_MOTOR_RIGHT_SW_VERSION(""),
        ATTR_MOTOR_RIGHT_LINK_STATUS(0), 	
        ATTR_MOTOR_RIGHT_CONNECTED_TIME(0), 	
        ATTR_MOTOR_RIGHT_DISCONNECTED_TIME(0)   
    };


    // 批量修改属性ID
    sacp::increase_attributes_id(attrs1, motor_attribute_offset(index));
    sacp::increase_attributes_id(attrs2, motor_attribute_offset(index));

    // 读取指定属性
    auto result1 = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs1); 
    // 如果操作失败，直接返回
    if (result1->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result1;
    }

    // 读取指定属性
    auto result2 = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs2); 
    // 如果操作失败，直接返回
    if (result2->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result2;
    }

    bool ret1 = parse_motor_info1(result1->attributes, index, info.device_brief);
    bool ret2 = parse_motor_info2(result2->attributes, index, info.device_brief);
    bool ret3 = parse_motor_admin_status(result2->attributes, index, info.admin_status);

    if (ret1 && ret2 && ret3)
    {
        return result1;
    }

    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::FewAttributesMissed); 
}


/**
 * @brief 解析控制反馈数据1
 * 
 * @param attrs 
 * @param data
 * @return true 
 * @return false 
 */
bool parse_motion_data(sacp::AttributeArray const &attrs, naiad::chassis::MsgMotionData &data)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_MOTION_TARGET_VELOCITY(0),
        ATTR_MOTION_TARGET_STEERING_ANGLE(0),
        ATTR_MOTION_REAL_VELOCITY(0),
        ATTR_MOTION_REAL_STEERING_ANGLE(0),
        ATTR_MOTION_RIGHT_WHEEL_TARGET_VELOCITY(0), // 4
        ATTR_MOTION_LEFT_WHEEL_TARGET_VELOCITY(0),  // 5
        ATTR_MOTION_FRONT_WHEEL_TARGET_VELOCITY(0), // 6
        ATTR_MOTION_RIGHT_WHEEL_REAL_VELOCITY(0),  // 7
        ATTR_MOTION_LEFT_WHEEL_REAL_VELOCITY(0),  // 8
        ATTR_MOTION_FRONT_WHEEL_REAL_VELOCITY(0), // 9
        ATTR_MOTION_RIGHT_MOTOR_REAL_VELOCITY(0), // 10
        ATTR_MOTION_LEFT_MOTOR_REAL_VELOCITY(0),  // 11
        ATTR_MOTION_FRONT_MOTOR_REAL_VELOCITY(0), // 12
        ATTR_MOTION_RIGHT_MOTOR_REAL_CURRENT(0),  // 13
        ATTR_MOTION_LEFT_MOTOR_REAL_CURRENT(0),
        ATTR_MOTION_FRONT_MOTOR_REAL_CURRENT(0), 
        ATTR_MOTION_STEERING_MOTOR_REAL_CURRENT(0),
    };

    size_t failed = 0;

    data.real_velocity = get_attribute(attrs, failed, pattern[2], 0).get_float();
    data.real_angle = get_attribute(attrs, failed, pattern[3], 0).get_float(); 
    data.wheel_velocity[0] = get_attribute(attrs, failed, pattern[7], 0).get_float(); 
    data.wheel_velocity[1] = get_attribute(attrs, failed, pattern[8], 0).get_float(); 
    data.wheel_velocity[2] = get_attribute(attrs, failed, pattern[9], 0).get_float(); 

    data.motor_velocity[0] = get_attribute(attrs, failed, pattern[10], 0).get_float(); 
    data.motor_velocity[1] = get_attribute(attrs, failed, pattern[11], 0).get_float(); 
    data.motor_velocity[2] = get_attribute(attrs, failed, pattern[12], 0).get_float(); 

    data.motor_current[0] = get_attribute(attrs, failed, pattern[13], 0).get_float(); 
    data.motor_current[1] = get_attribute(attrs, failed, pattern[14], 0).get_float(); 
    data.motor_current[2] = get_attribute(attrs, failed, pattern[15], 0).get_float(); 
    data.motor_current[3] = get_attribute(attrs, failed, pattern[16], 0).get_float(); 
    data.period_us = 20000;

    return (failed == 0);

}


/**
 * @brief 解析控制反馈数据2
 * 
 * @param attrs 
 * @param data
 * @return true 
 * @return false 
 */
bool parse_motion_odometer(sacp::AttributeArray const &attrs, naiad::chassis::MsgMotionOdometer &data)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_MOTION_RIGHT_WHEEL_DELTA_DISTANCE(0), 
        ATTR_MOTION_LEFT_WHEEL_DELTA_DISTANCE(0),
        ATTR_MOTION_FRONT_WHEEL_DELTA_DISTANCE(0), 
        ATTR_MOTION_RIGHT_WHEEL_ODOMETER(0), 
        ATTR_MOTION_LEFT_WHEEL_ODOMETER(0),
        ATTR_MOTION_FRONT_WHEEL_ODOMETER(0), 
        ATTR_MOTION_RIGHT_MOTOR_DELTA_DISTANCE(0), 
        ATTR_MOTION_LEFT_MOTOR_DELTA_DISTANCE(0),
        ATTR_MOTION_FRONT_MOTOR_DELTA_DISTANCE(0), 
        ATTR_MOTION_RIGHT_MOTOR_ODOMETER(0), 
        ATTR_MOTION_LEFT_MOTOR_ODOMETER(0),
        ATTR_MOTION_FRONT_MOTOR_ODOMETER(0), 
    };

    size_t failed = 0;

    data.wheel_delta_distance[0] = get_attribute(attrs, failed, pattern[0], 0).get_float(); 
    data.wheel_delta_distance[1] = get_attribute(attrs, failed, pattern[1], 0).get_float(); 
    data.wheel_delta_distance[2] = get_attribute(attrs, failed, pattern[2], 0).get_float(); 

    data.wheel_odometer[0] = get_attribute(attrs, failed, pattern[3], 0).get_float(); 
    data.wheel_odometer[1] = get_attribute(attrs, failed, pattern[4], 0).get_float(); 
    data.wheel_odometer[2] = get_attribute(attrs, failed, pattern[5], 0).get_float(); 

    data.motor_delta_distance[0] = get_attribute(attrs, failed, pattern[6], 0).get_float(); 
    data.motor_delta_distance[1] = get_attribute(attrs, failed, pattern[7], 0).get_float(); 
    data.motor_delta_distance[2] = get_attribute(attrs, failed, pattern[8], 0).get_float(); 

    data.motor_odometer[0] = get_attribute(attrs, failed, pattern[9], 0).get_float(); 
    data.motor_odometer[1] = get_attribute(attrs, failed, pattern[10], 0).get_float(); 
    data.motor_odometer[2] = get_attribute(attrs, failed, pattern[11], 0).get_float(); 

    return (failed == 0);

}


/// @brief 设置运动控制目标速度和角度
/// @param client SACP客户端
/// @param velocity 速度
/// @param angel 角度
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_motion_target(
    std::shared_ptr<sacp::SacpClient> client, float velocity, float angel)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    std::vector<sacp::Attribute> attrs = {
        ATTR_MOTION_SET_VELOCITY(velocity), 	
        ATTR_MOTION_SET_STEERING_ANGLE(angel)
    };

    // 读取指定属性
    return client->write_attributes("ros", sacp::Frame::Priority::PriorityHighest, attrs); 
    
}

}
}
