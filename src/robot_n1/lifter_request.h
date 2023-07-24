

#ifndef __ROBOT_N1_LIFTER_REQUEST_H__
#define __ROBOT_N1_LIFTER_REQUEST_H__

/**
 * @file lifter_request.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 封装外设的数据对接函数和请求函数
 * @version 0.1
 * @date 2023-06-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "attribute_helper.h"
#include "sacp_client/sacp_client.h"
#include "chassis/chassis_type.h"
#include "device_index.h"


namespace robot{
namespace n1
{

/**
 * @brief 解析升降器的设备简介
 * 
 * @param attrs 收到的属性
 * @param index 返回的索引
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_lifter_device_brief(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgDeviceBreif &device);

/**
 * @brief 解析升降器的管理状态
 * 
 * @param attrs 收到的属性
 * @param index 返回的索引
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_lifter_admin_status(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgAdminStatus &device);

/**
 * @brief 解析升降器的设备状态信息
 * 
 * @param attrs 
 * @param index 
 * @param device 
 * @return true 
 * @return false 
 */
bool parse_lifter_device_state(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgLifterState &device);

/// @brief 设置升降器位置
/// @param index 
/// @param position 
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_lifter_position(
    std::shared_ptr<sacp::SacpClient> client, DeviceIndex index, uint8_t position);

/// @brief 读升降器的信息
/// @param client 
/// @param index 
/// @param info 需要返回的升降器的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> read_lifter_info(
    std::shared_ptr<sacp::SacpClient> client, 
    DeviceIndex index, naiad::chassis::MsgDeviceInfo & info);
    
}
}

#endif // __ROBOT_N1_LIFTER_REQUEST_H__
