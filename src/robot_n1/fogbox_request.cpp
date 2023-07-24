
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
#include "chassis/chassis_type.h"
#include "device_index.h"

namespace robot{
namespace n1
{


/**
 * @brief 解析FOG盒的设备简介
 * 
 * @param attrs 收到的属性
 * @param index 返回设备索引
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_fogbox_device_brief(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgDeviceBreif &device)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_FOGBOX_MODEL(""), 		     
        ATTR_FOGBOX_SN(""), 		         
        ATTR_FOGBOX_HW_VERSION(0), 		 
        ATTR_FOGBOX_SW_VERSION(0),		 
        ATTR_FOGBOX_ADDRESS(0), 		 
        ATTR_FOGBOX_NAME("")
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
    
    index = DeviceIndex::FogBox;

    return (failed == 0);
}

/**
 * @brief 解析FOG盒的管理状态
 * 
 * @param attrs 收到的属性
 * @param index 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_fogbox_admin_status(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgAdminStatus &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_FOGBOX_LINK_STATUS(0), 	
        ATTR_FOGBOX_CONNECTED_TIME(0), 	
        ATTR_FOGBOX_DISCONNECTED_TIME(0)
    };

    size_t offset = 0;
    size_t failed = 0;

    device.link_status = get_attribute(attrs, failed, pattern[0], offset).get_uint8(); 
    device.connected_time = get_attribute(attrs, failed, pattern[1], offset).get_uint32(); 
    device.disconnected_time = get_attribute(attrs, failed, pattern[2], offset).get_uint32(); 

    index = DeviceIndex::FogBox;

    return (failed == 0);
}

/**
 * @brief 解析FOG盒的设备状态信息
 * 
 * @param attrs 
 * @param index 
 * @param device 
 * @return true 
 * @return false 
 */
bool parse_fogbox_device_state(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgFogBoxState &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_FOGBOX_STATE(0),
        ATTR_FOGBOX_TEMPERATURE(0),
        ATTR_FOGBOX_HUMIDITY(0),
    };

    size_t offset = 0;
    size_t failed = 0;

    device.state = get_attribute(attrs, failed, pattern[0], offset).get_uint16(); 
    device.temperature = get_attribute(attrs, failed, pattern[1], offset).get_float(); 
    device.humidity = get_attribute(attrs, failed, pattern[2], offset).get_float();     

    index = DeviceIndex::FogBox;

    return (failed == 0);
}



/// @brief 读FOG盒的信息
/// @param client 
/// @param index 
/// @param info 需要返回的FOG盒的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> read_fogbox_info(
    std::shared_ptr<sacp::SacpClient> client, 
    DeviceIndex index, naiad::chassis::MsgDeviceInfo & info)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    std::vector<sacp::Attribute> attrs = {
        ATTR_FOGBOX_MODEL(""), 		     
        ATTR_FOGBOX_SN(""), 		         
        ATTR_FOGBOX_HW_VERSION(0), 		 
        ATTR_FOGBOX_SW_VERSION(0),		 
        ATTR_FOGBOX_ADDRESS(0), 		 
        ATTR_FOGBOX_NAME(""), 		     
        ATTR_FOGBOX_LINK_STATUS(0), 	
        ATTR_FOGBOX_CONNECTED_TIME(0), 	
        ATTR_FOGBOX_DISCONNECTED_TIME(0)
    };

    // 批量修改属性ID
    // sacp::increase_attributes_id(attrs, (index - 1) * (ATTR_LIFTER_B_ADDRESS_ID - ATTR_FOGBOX_ADDRESS_ID));

    // 读取指定属性
    auto result = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
    // 如果操作失败，直接返回
    if (result->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result;
    }

    bool ret1 = parse_fogbox_device_brief(result->attributes, index, info.device_brief);
    bool ret2 = parse_fogbox_admin_status(result->attributes, index, info.admin_status);
    
    if (ret1 && ret2)
    {
        return result;
    }

    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::FewAttributesMissed); 
}

    
}
}
