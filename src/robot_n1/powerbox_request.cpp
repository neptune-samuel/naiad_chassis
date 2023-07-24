
/**
 * @file powerbox_request.cpp
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
#include "chassis/chassis_type.h"
#include "device_index.h"


namespace robot{
namespace n1
{


/**
 * @brief 解析电源盒的设备简介
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_powerbox_device_brief(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgDeviceBreif &device)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_POWERBOX_MODEL(""), 		     
        ATTR_POWERBOX_SN(""), 		         
        ATTR_POWERBOX_HW_VERSION(0), 		 
        ATTR_POWERBOX_SW_VERSION(0),		 
        ATTR_POWERBOX_ADDRESS(0), 		 
        ATTR_POWERBOX_NAME("")
    };

    size_t offset = 0;
    size_t failed = 0;

    device.model = get_attribute(attrs, failed, pattern[0], offset).get_string();
    device.serial_number = get_attribute(attrs, failed, pattern[1], offset).get_string(); 
    device.hardware_version = naiad::chassis::version16_string(get_attribute(attrs, failed, pattern[2], offset).get_uint16()); 
    device.software_version = naiad::chassis::version16_string(get_attribute(attrs, failed, pattern[3], offset).get_uint16()); 
    uint8_t address = get_attribute(attrs, failed, pattern[4], offset).get_uint8();
    device.address_info = std::to_string(address);      
    device.name = get_attribute(attrs, failed, pattern[5], offset).get_string(); 

    index = DeviceIndex::PowerBox;

    return (failed == 0);
}

/**
 * @brief 解析电源盒的管理状态
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_powerbox_admin_status(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgAdminStatus &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_POWERBOX_LINK_STATUS(0), 	
        ATTR_POWERBOX_CONNECTED_TIME(0), 	
        ATTR_POWERBOX_DISCONNECTED_TIME(0)
    };

    size_t offset = 0;
    size_t failed = 0;

    device.link_status = get_attribute(attrs, failed, pattern[0], offset).get_uint8(); 
    device.connected_time = get_attribute(attrs, failed, pattern[1], offset).get_uint32(); 
    device.disconnected_time = get_attribute(attrs, failed, pattern[2], offset).get_uint32(); 

    index = DeviceIndex::PowerBox;

    return (failed == 0);
}

/**
 * @brief 解析电源盒的设备状态信息
 * 
 * @param attrs 
 * @param address 
 * @param device 
 * @return true 
 * @return false 
 */
bool parse_powerbox_device_state(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgPowerBoxState &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_POWERBOX_STATE(0),
        ATTR_POWERBOX_TEMPERATURE(0),
        ATTR_POWERBOX_HUMIDITY(0),
        ATTR_POWERBOX_TOTAL_VOLTAGE(0),
        ATTR_POWERBOX_TOTAL_CURRENT(0),
        ATTR_POWERBOX_MAIN_IN_VOLTAGE(0),
        ATTR_POWERBOX_MAIN_IN_CURRENT(0),
        ATTR_POWERBOX_MAIN_OUT_VOLTAGE(0),
        ATTR_POWERBOX_MAIN_OUT_CURRENT(0),
        ATTR_POWERBOX_MAIN_TEMPERATURE(0),
        ATTR_POWERBOX_MAIN_STATUS_CODE(0),
        ATTR_POWERBOX_AUX_IN_VOLTAGE(0),
        ATTR_POWERBOX_AUX_IN_CURRENT(0),
        ATTR_POWERBOX_AUX_OUT_VOLTAGE(0),
        ATTR_POWERBOX_AUX_OUT_CURRENT(0),
        ATTR_POWERBOX_AUX_TEMPERATURE(0),
        ATTR_POWERBOX_AUX_STATUS_CODE(0)
    };

    size_t offset = 0;
    size_t failed = 0;

    device.state = get_attribute(attrs, failed, pattern[0], offset).get_uint16(); 
    device.temperature = get_attribute(attrs, failed, pattern[1], offset).get_float(); 
    device.humidity = get_attribute(attrs, failed, pattern[2], offset).get_float();     
    device.total_voltage = get_attribute(attrs, failed, pattern[3], offset).get_float(); 
    device.total_current = get_attribute(attrs, failed, pattern[4], offset).get_float(); 
    device.main_in_voltage = get_attribute(attrs, failed, pattern[5], offset).get_float(); 
    device.main_in_current = get_attribute(attrs, failed, pattern[6], offset).get_float(); 
    device.main_out_voltage = get_attribute(attrs, failed, pattern[7], offset).get_float(); 
    device.main_out_current = get_attribute(attrs, failed, pattern[8], offset).get_float(); 
    device.main_temperature = get_attribute(attrs, failed, pattern[9], offset).get_float(); 
    device.main_status_code = get_attribute(attrs, failed, pattern[10], offset).get_float(); 
    device.aux_in_voltage = get_attribute(attrs, failed, pattern[11], offset).get_float(); 
    device.aux_in_current = get_attribute(attrs, failed, pattern[12], offset).get_float(); 
    device.aux_out_voltage = get_attribute(attrs, failed, pattern[13], offset).get_float(); 
    device.aux_out_current = get_attribute(attrs, failed, pattern[14], offset).get_float(); 
    device.aux_temperature = get_attribute(attrs, failed, pattern[15], offset).get_float(); 
    device.aux_status_code = get_attribute(attrs, failed, pattern[16], offset).get_float(); 

    index = DeviceIndex::PowerBox;

    return (failed == 0);
}



/// @brief 读电源盒的信息
/// @param client 
/// @param address 
/// @param info 需要返回的电源盒的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> read_powerbox_info(
    std::shared_ptr<sacp::SacpClient> client, 
    DeviceIndex index, naiad::chassis::MsgDeviceInfo & info)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    std::vector<sacp::Attribute> attrs = {
        ATTR_POWERBOX_MODEL(""), 		     
        ATTR_POWERBOX_SN(""), 		         
        ATTR_POWERBOX_HW_VERSION(0), 		 
        ATTR_POWERBOX_SW_VERSION(0),		 
        ATTR_POWERBOX_ADDRESS(0), 	 
        ATTR_POWERBOX_NAME(""), 		     
        ATTR_POWERBOX_LINK_STATUS(0), 	
        ATTR_POWERBOX_CONNECTED_TIME(0), 	
        ATTR_POWERBOX_DISCONNECTED_TIME(0)
    };

    // 批量修改属性ID
    // sacp::increase_attributes_id(attrs, (address - 1) * (ATTR_LIFTER_B_ADDRESS_ID - ATTR_POWERBOX_ADDRESS_ID));

    // 读取指定属性
    auto result = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
    // 如果操作失败，直接返回
    if (result->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result;
    }

    bool ret1 = parse_powerbox_device_brief(result->attributes, index, info.device_brief);
    bool ret2 = parse_powerbox_admin_status(result->attributes, index, info.admin_status);
    
    if (ret1 && ret2)
    {
        return result;
    }

    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::FewAttributesMissed); 
}

    
}
}
