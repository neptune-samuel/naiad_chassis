/**
 * @file depth_sensor_request.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 解析深度传感器的数据 
 * @version 0.1
 * @date 2023-07-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "local_attributes.h"
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
bool parse_depth_sensor_data(sacp::AttributeArray const &attrs, naiad::chassis::MsgDepthData &data)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_DEPTH_SENSOR_STATE(0), 		     
        ATTR_DEPTH_SENSOR_PRESSURE(0), 		         
        ATTR_DEPTH_SENSOR_TEMPERATURE(0), 		     
    };

    size_t failed = 0;

    data.id = 0; //不使用
    data.state = get_attribute(attrs, failed, pattern[0], 0).get_uint8();    
    data.pressure = get_attribute(attrs, failed, pattern[1], 0).get_float();
    data.temperature = get_attribute(attrs, failed, pattern[2], 0).get_float();

    return (failed == 0);

}



}
}



