#ifndef __ROBOT_N1_DEPTH_SENSOR_REQUEST_H__
#define __ROBOT_N1_DEPTH_SENSOR_REQUEST_H__

/**
 * @file depth_sensor_request.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 深度传感器数据解析
 * @version 0.1
 * @date 2023-07-21
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
 * @brief 解析深度传感器上报的数据
 * 
 * @param attrs 属性组
 * @param data 解析后的数据 
 * @return true 
 * @return false 
 */
bool parse_depth_sensor_data(sacp::AttributeArray const &attrs, naiad::chassis::MsgDepthData &data);

}
}

#endif // __ROBOT_N1_CONTROLLER_REQUEST_H__
