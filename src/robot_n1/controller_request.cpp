

/**
 * @file fogbox_request.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 封装外设的数据对接函数和请求函数
 * @version 0.1
 * @date 2023-06-07
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
 * @brief 解析控制器的信息
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_controller_info(sacp::AttributeArray const &attrs, naiad::chassis::MsgControllerInfo &info)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_APPLICATION_ID(0), 		     
        ATTR_APPLICATION_VERSION(0), 		         
        ATTR_HARDWARE_ID(0), 		     
        ATTR_HARDWARE_VERSION(0), 		         
        ATTR_SOFTWARE_VERSION(0), 		 
        ATTR_SOFTWARE_RELEASED_TIME(0),		 
        ATTR_PRODUCT_MODEL(""), 		 
        ATTR_PRODUCT_SN("")
    };

    size_t failed = 0;

    info.hardware_id = get_attribute(attrs, failed, pattern[2], 0).get_uint32();
    info.hardware_version = naiad::chassis::version_string(get_attribute(attrs, failed, pattern[3], 0).get_uint32()); 
    info.software_version = naiad::chassis::version_string(get_attribute(attrs, failed, pattern[4], 0).get_uint32()); 
    info.software_released_time = naiad::chassis::time_string(get_attribute(attrs, failed, pattern[5], 0).get_uint32()); 
    info.model = get_attribute(attrs, failed, pattern[6], 0).get_string(); 
    info.serial_number = get_attribute(attrs, failed, pattern[7], 0).get_string(); 

    return (failed == 0);
}

/**
 * @brief 解析控制器的状态
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_controller_status(sacp::AttributeArray const &attrs, naiad::chassis::MsgControllerState &state)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_UP_TIME(0), 	
        ATTR_RTC_TIME(0), 	
        ATTR_CPU_USAGED(0),
        ATTR_MEM_TOTAL(0),
        ATTR_MEM_FREE(0),
        ATTR_BOARD_STATE(0),
        ATTR_BOARD_TEMPERATURE(0),
        ATTR_BOARD_HUMIDITY(0),
        ATTR_MCU_TEMPERATURE(0)
    };

    size_t failed = 0;

    state.up_time = get_attribute(attrs, failed, pattern[0], 0).get_uint32();
    state.rtc_time = get_attribute(attrs, failed, pattern[1], 0).get_uint32(); 
    state.cpu_usaged = get_attribute(attrs, failed, pattern[2], 0).get_float(); 
    state.ram_size = get_attribute(attrs, failed, pattern[3], 0).get_uint32(); 
    state.ram_free = get_attribute(attrs, failed, pattern[4], 0).get_uint32(); 
    state.board_state = get_attribute(attrs, failed, pattern[5], 0).get_uint16(); 
    state.board_temperature = get_attribute(attrs, failed, pattern[6], 0).get_float(); 
    state.board_humidity = get_attribute(attrs, failed, pattern[7], 0).get_float();
    state.mcu_temperature = get_attribute(attrs, failed, pattern[8], 0).get_float();

    return (failed == 0);
}


/// @brief 同步MCU的RTC的时间
/// @param client 
/// @param time
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_controller_rtc_time(
    std::shared_ptr<sacp::SacpClient> client, uint32_t time)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    std::vector<sacp::Attribute> attrs = {
                ATTR_RTC_TIME(time)
            };

    // 写请求
    return client->write_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs);     
}

/// @brief 获取主控的信息
/// @param client 
/// @param info
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> get_controller_info(
    std::shared_ptr<sacp::SacpClient> client, naiad::chassis::MsgControllerInfo & info)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    std::vector<sacp::Attribute> attrs = {
                ATTR_APPLICATION_ID(0), 		     
                ATTR_APPLICATION_VERSION(0), 		         
                ATTR_HARDWARE_ID(0), 		     
                ATTR_HARDWARE_VERSION(0), 		         
                ATTR_SOFTWARE_VERSION(0), 		 
                ATTR_SOFTWARE_RELEASED_TIME(0),		 
                ATTR_PRODUCT_MODEL(""), 		 
                ATTR_PRODUCT_SN("")
            };

    // 读取指定属性
    auto result = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
    // 如果操作失败，直接返回
    if (result->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result;
    }

    bool ret = parse_controller_info(result->attributes, info);
    if (ret)
    {
        return result;
    }

    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::FewAttributesMissed);     
}


}
}

