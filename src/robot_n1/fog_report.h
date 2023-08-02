
#ifndef __ROBOT_N1_FOG_REPORT_H__
#define __ROBOT_N1_FOG_REPORT_H__
/**
 * @file fog_report.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 处理FOG数据转发
 * @version 0.1
 * @date 2023-08-02
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
 * @brief 生成一个FOG数据的Report帧
 * 
 * @param fog 
 * @return std::unique_ptr<sacp::Frame> 
 */
std::unique_ptr<sacp::Frame> make_fog_data_report(naiad::chassis::MsgFogData & fog);

/**
 * @brief 生成一个FOG状态的Report帧
 * 
 * @param fog 
 * @return std::unique_ptr<sacp::Frame> 
 */
std::unique_ptr<sacp::Frame> make_fog_state_report(naiad::chassis::MsgFogState & fog);

}
}

#endif // __ROBOT_N1_FOG_REPORT_H__
