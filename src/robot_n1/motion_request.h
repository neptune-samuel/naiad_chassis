

#ifndef __ROBOT_N1_MOTION_REQUEST_H__
#define __ROBOT_N1_MOTION_REQUEST_H__

/**
 * @file motion_request.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 运动控制请求
 * @version 0.1
 * @date 2023-07-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "attribute_helper.h"
#include "sacp_client/sacp_client.h"
#include "chassis/chassis_type.h"



namespace robot{
namespace n1
{

/**
 * @brief 解析电机的设备简介
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_motor_info(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::MsgDeviceBreif &info);

/**
 * @brief 解析电机的管理状态
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_motor_admin_status(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::MsgAdminStatus &status);

/**
 * @brief 解析电机的设备状态信息
 * 
 * @param attrs 
 * @param address 
 * @param device 
 * @return true 
 * @return false 
 */
bool parse_motor_state(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::MsgMotorState &state);

/**
 * @brief 解析控制反馈数据1
 * 
 * @param attrs 
 * @param data
 * @return true 
 * @return false 
 */
bool parse_motion_data(sacp::AttributeArray const &attrs, naiad::chassis::MsgMotionData &data);

/**
 * @brief 解析控制反馈数据2
 * 
 * @param attrs 
 * @param data
 * @return true 
 * @return false 
 */
bool parse_motion_odometer(sacp::AttributeArray const &attrs, naiad::chassis::MsgMotionOdometer &data);


/// @brief 设置运动控制目标速度和角度
/// @param client SACP客户端
/// @param velocity 速度
/// @param angel 角度
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_motion_target(
    std::shared_ptr<sacp::SacpClient> client, float velocity, float angel);
    
}
}

#endif // __ROBOT_N1_MOTION_REQUEST_H__
