

#ifndef __ROBOT_N1_PUSHBOX_REQUEST_H__
#define __ROBOT_N1_PUSHBOX_REQUEST_H__

/**
 * @file pushbox_request.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 封装外设的数据对接函数和请求函数
 * @version 0.1
 * @date 2023-06-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "attribute_helper.h"
#include "chassis/sacp_client.h"
#include "chassis/chassis_type.h"



namespace robot{
namespace n1
{


/**
 * @brief 解析顶出盒的设备简介
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_pushbox_device_brief(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::MsgDeviceBreif &device);

/**
 * @brief 解析顶出盒的管理状态
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_pushbox_admin_status(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::MsgAdminStatus &device);

/**
 * @brief 解析顶出盒的设备状态信息
 * 
 * @param attrs 
 * @param address 
 * @param device 
 * @return true 
 * @return false 
 */
bool parse_pushbox_device_state(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::MsgPushBoxState &device);

/**
 * @brief 解析顶出盒的配置状态
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_pushbox_offline_config(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::SrvPushBoxGetOfflineConfigResponse &config);

/// @brief 配置顶出控制 
/// @param address 
/// @param position 
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_pushbox_control(
    std::shared_ptr<sacp::SacpClient> client, [[maybe_unused]]uint8_t address, uint8_t control);

/// @brief 读顶出盒的信息
/// @param client 
/// @param address 
/// @param info 需要返回的顶出盒的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> read_pushbox_info(
    std::shared_ptr<sacp::SacpClient> client, 
     [[maybe_unused]]uint8_t address, naiad::chassis::MsgDeviceInfo & info);

/// @brief 获取顶出盒配置
/// @param client 
/// @param address 
/// @param info 需要返回的顶出盒的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> get_pushbox_offline_config(
    std::shared_ptr<sacp::SacpClient> client, 
     [[maybe_unused]]uint8_t address, naiad::chassis::SrvPushBoxGetOfflineConfigResponse & config);

/// @brief 修改顶出盒配置
/// @param client 
/// @param address 
/// @param info 需要返回的顶出盒的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_pushbox_offline_config(
    std::shared_ptr<sacp::SacpClient> client, 
     [[maybe_unused]]uint8_t address, naiad::chassis::SrvPushBoxSetOfflineConfigRequest const & config);

}
}

#endif // __ROBOT_N1_PUSHBOX_REQUEST_H__
