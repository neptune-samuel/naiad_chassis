


#ifndef __ROBOT_N1_CONTROLLER_REQUEST_H__
#define __ROBOT_N1_CONTROLLER_REQUEST_H__

/**
 * @file controller_request.cpp
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
#include "chassis/node_chassis.h"


namespace robot{
namespace n1
{

/**
 * @brief 解析控制器的信息
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_controller_info(sacp::AttributeArray const &attrs, naiad::chassis::MsgControllerInfo &info);

/**
 * @brief 解析控制器的状态
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_controller_status(sacp::AttributeArray const &attrs, naiad::chassis::MsgControllerState &state);

/// @brief 同步MCU的RTC的时间
/// @param client 
/// @param time
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_controller_rtc_time(
    std::shared_ptr<sacp::SacpClient> client, uint32_t time);

/// @brief 获取主控的信息
/// @param client 
/// @param info
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> get_controller_info(
    std::shared_ptr<sacp::SacpClient> client, naiad::chassis::MsgControllerInfo & info);

    
}
}

#endif // __ROBOT_N1_CONTROLLER_REQUEST_H__
