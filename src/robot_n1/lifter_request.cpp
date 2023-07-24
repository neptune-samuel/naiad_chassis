
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

#include "local_attributes.h"
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
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_lifter_device_brief(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgDeviceBreif &device)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_LIFTER_A_MODEL(""), 		     
        ATTR_LIFTER_A_SN(""), 		         
        ATTR_LIFTER_A_HW_VERSION(0), 		 
        ATTR_LIFTER_A_SW_VERSION(0),		 
        ATTR_LIFTER_A_ADDRESS(0), 		 
        ATTR_LIFTER_A_NAME("")
    };

    size_t offset = 0;
    size_t failed = 0;

    if (!parse_attributes_range(attrs, ATTR_LIFTER_A_MODEL_ID, ATTR_LIFTER_B_MODEL_ID, offset, index))
    {
        return false;
    }    

    device.model = get_attribute(attrs, failed, pattern[0], offset).get_string();
    device.serial_number = get_attribute(attrs, failed, pattern[1], offset).get_string(); 
    device.hardware_version = naiad::chassis::version16_string(get_attribute(attrs, failed, pattern[2], offset).get_uint16()); 
    device.software_version = naiad::chassis::version16_string(get_attribute(attrs, failed, pattern[3], offset).get_uint16()); 
    uint8_t address = get_attribute(attrs, failed, pattern[4], offset).get_uint8();
    device.address_info = std::to_string(address);     
    device.name = get_attribute(attrs, failed, pattern[5], offset).get_string(); 
    
    return (failed == 0);
}

/**
 * @brief 解析升降器的管理状态
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_lifter_admin_status(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgAdminStatus &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_LIFTER_A_LINK_STATUS(0), 	
        ATTR_LIFTER_A_CONNECTED_TIME(0), 	
        ATTR_LIFTER_A_DISCONNECTED_TIME(0)
    };

    size_t offset = 0;
    size_t failed = 0;

    if (!parse_attributes_range(attrs, ATTR_LIFTER_A_LINK_STATUS_ID, ATTR_LIFTER_B_LINK_STATUS_ID, offset, index))
    {
        return false;
    }   

    device.link_status = get_attribute(attrs, failed, pattern[0], offset).get_uint8(); 
    device.connected_time = get_attribute(attrs, failed, pattern[1], offset).get_uint32(); 
    device.disconnected_time = get_attribute(attrs, failed, pattern[2], offset).get_uint32(); 

    return (failed == 0);
}

/**
 * @brief 解析升降器的设备状态信息
 * 
 * @param attrs 
 * @param address 
 * @param device 
 * @return true 
 * @return false 
 */
bool parse_lifter_device_state(sacp::AttributeArray const &attrs, 
   DeviceIndex & index, naiad::chassis::MsgLifterState &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_LIFTER_A_POSITION(0),
        ATTR_LIFTER_A_RAW_POSITION(0),
        ATTR_LIFTER_A_TEMPERATURE(0),
        ATTR_LIFTER_A_LOAD(0),
        ATTR_LIFTER_A_VOLTAGE(0),
        ATTR_LIFTER_A_CURRRENT(0),
        ATTR_LIFTER_A_RUNNING(0),
        ATTR_LIFTER_A_ALARM_STATUS(0)
    };

    size_t offset = 0;
    size_t failed = 0;

    if (!parse_attributes_range(attrs, ATTR_LIFTER_A_POSITION_ID, ATTR_LIFTER_B_POSITION_ID, offset, index))
    {
        return false;
    }   

    device.position = get_attribute(attrs, failed, pattern[0], offset).get_uint8(); 
    device.raw_position = get_attribute(attrs, failed, pattern[1], offset).get_uint16(); 
    device.temperature = get_attribute(attrs, failed, pattern[2], offset).get_float(); 
    device.load = get_attribute(attrs, failed, pattern[3], offset).get_uint8(); 
    device.voltage = get_attribute(attrs, failed, pattern[4], offset).get_float(); 
    device.current = get_attribute(attrs, failed, pattern[5], offset).get_float(); 
    device.running = get_attribute(attrs, failed, pattern[6], offset).get_bool(); 
    device.alarm_status = get_attribute(attrs, failed, pattern[7], offset).get_uint16(); 

    return (failed == 0);
}




/// @brief 设置升降器位置
/// @param address 
/// @param position 
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_lifter_position(
    std::shared_ptr<sacp::SacpClient> client, DeviceIndex index, uint8_t position)
{

    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    if (position > 100)
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InvalidParameter);
    }

    if (!valid_lifter_index(index))
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);
    }

    std::vector<sacp::Attribute> attrs = {
                ATTR_LIFTER_A_SET_POSITION(position)
            };
    // 批量修改属性ID
    sacp::increase_attributes_id(attrs, lifter_attribute_offset(index));

    // 写请求
    return client->write_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
}

/// @brief 读升降器的信息
/// @param client 
/// @param address 
/// @param info 需要返回的升降器的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> read_lifter_info(
    std::shared_ptr<sacp::SacpClient> client, 
    DeviceIndex index, naiad::chassis::MsgDeviceInfo & info)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    if (!valid_lifter_index(index))
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);
    }

    std::vector<sacp::Attribute> attrs = {
        ATTR_LIFTER_A_MODEL(""), 		     
        ATTR_LIFTER_A_SN(""), 		         
        ATTR_LIFTER_A_HW_VERSION(0), 		 
        ATTR_LIFTER_A_SW_VERSION(0),		 
        ATTR_LIFTER_A_ADDRESS(0), 		 
        ATTR_LIFTER_A_NAME(""), 		     
        ATTR_LIFTER_A_LINK_STATUS(0), 	
        ATTR_LIFTER_A_CONNECTED_TIME(0), 	
        ATTR_LIFTER_A_DISCONNECTED_TIME(0)
    };

    // 批量修改属性ID
    sacp::increase_attributes_id(attrs, lifter_attribute_offset(index));

    // 读取指定属性
    auto result = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
    // 如果操作失败，直接返回
    if (result->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result;
    }

    bool ret1 = parse_lifter_device_brief(result->attributes, index, info.device_brief);
    bool ret2 = parse_lifter_admin_status(result->attributes, index, info.admin_status);
    
    if (ret1 && ret2)
    {
        return result;
    }

    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::FewAttributesMissed); 
}

    
}
}
