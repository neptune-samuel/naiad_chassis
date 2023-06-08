
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

#include "local_attributes.h"
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
   uint8_t & address, naiad::chassis::MsgDeviceBreif &device)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_PUSHBOX_MODEL(""), 		     
        ATTR_PUSHBOX_SN(""), 		         
        ATTR_PUSHBOX_HW_VERSION(0), 		 
        ATTR_PUSHBOX_SW_VERSION(0),		 
        ATTR_PUSHBOX_ADDRESS(0), 		 
        ATTR_PUSHBOX_NAME("")
    };

    size_t offset = 0;
    uint8_t index = 0;

    // if (!parse_attributes_range(attrs, ATTR_PUSHBOX_MODEL_ID, ATTR_LIFTER_B_MODEL_ID, offset, index))
    // {
    //     return false;
    // }    

    size_t failed = 0;

    device.model = get_attribute(attrs, failed, pattern[0], offset).get_string();
    device.serial_number = get_attribute(attrs, failed, pattern[1], offset).get_string(); 
    device.hardware_version = get_attribute(attrs, failed, pattern[2], offset).get_uint16(); 
    device.software_version = get_attribute(attrs, failed, pattern[3], offset).get_uint16(); 
    //address = get_attribute(attrs, failed, pattern[4], offset).get_uint8();    
    device.name = get_attribute(attrs, failed, pattern[5], offset).get_string(); 
    
    // if (address != (index + 1))
    // {
    //     slog::warning("receive report with unexpected device address, got:{}, expect:{}", address, (index + 1));        
    // }

    address = index + 1;

    return (failed == 0);
}

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
   uint8_t & address, naiad::chassis::MsgAdminStatus &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_PUSHBOX_LINK_STATUS(0), 	
        ATTR_PUSHBOX_CONNECTED_TIME(0), 	
        ATTR_PUSHBOX_DISCONNECTED_TIME(0)
    };

    size_t offset = 0;
    uint8_t index = 0;

    // if (!parse_attributes_range(attrs, ATTR_PUSHBOX_LINK_STATUS_ID, ATTR_LIFTER_B_LINK_STATUS_ID, offset, index))
    // {
    //     return false;
    // }   

    size_t failed = 0;

    device.link_status = get_attribute(attrs, failed, pattern[0], offset).get_uint8(); 
    device.connected_time = get_attribute(attrs, failed, pattern[1], offset).get_uint32(); 
    device.disconnected_time = get_attribute(attrs, failed, pattern[2], offset).get_uint32(); 

    address = index + 1;

    return (failed == 0);
}

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
   uint8_t & address, naiad::chassis::MsgPushBoxState &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_PUSHBOX_STATE(0),
        ATTR_PUSHBOX_PUSH_STATE(0),
        ATTR_PUSHBOX_TEMPERATURE(0),
        ATTR_PUSHBOX_HUMIDITY(0),
        ATTR_PUSHBOX_VOLTAGE(0),
        ATTR_PUSHBOX_CURRENT(0),
        ATTR_PUSHBOX_BATTERY_VOLTAGE(0), 
        ATTR_PUSHBOX_BATTERY_CURRENT(0),
        ATTR_PUSHBOX_BATTERY_LEVEL(0),
        ATTR_PUSHBOX_BATTERY_TEMPERATURE(0),     
        ATTR_PUSHBOX_BATTERY_INCHARGING(0),
        ATTR_PUSHBOX_BATTERY_VOLTAGE_STATUS(0),
        ATTR_PUSHBOX_BATTERY_CURRENT_STATUS(0),     
    };

    size_t offset = 0;
    uint8_t index = 0;

    // if (!parse_attributes_range(attrs, ATTR_PUSHBOX_POSITION_ID, ATTR_LIFTER_B_POSITION_ID, offset, index))
    // {
    //     return false;
    // }   

    size_t failed = 0;

    device.state = get_attribute(attrs, failed, pattern[0], offset).get_uint16(); 
    device.push_state = get_attribute(attrs, failed, pattern[1], offset).get_uint8();     
    device.temperature = get_attribute(attrs, failed, pattern[2], offset).get_float(); 
    device.humidity = get_attribute(attrs, failed, pattern[3], offset).get_float();     
    device.voltage = get_attribute(attrs, failed, pattern[4], offset).get_float();     
    device.current = get_attribute(attrs, failed, pattern[5], offset).get_float();     
    device.battery_voltage = get_attribute(attrs, failed, pattern[6], offset).get_float();     
    device.battery_current = get_attribute(attrs, failed, pattern[7], offset).get_float();     
    device.battery_level = get_attribute(attrs, failed, pattern[8], offset).get_uint8();     
    device.battery_temperature = get_attribute(attrs, failed, pattern[9], offset).get_float();     
    device.battery_is_charging = get_attribute(attrs, failed, pattern[10], offset).get_bool();  
    device.battery_voltage_status = get_attribute(attrs, failed, pattern[11], offset).get_uint16();  
    device.battery_current_status = get_attribute(attrs, failed, pattern[12], offset).get_uint16();  


    address = index + 1;

    return (failed == 0);
}


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
   uint8_t & address, naiad::chassis::SrvPushBoxGetOfflineConfigResponse &config)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_PUSHBOX_OFFLINE_PUSH_ENABLE(0), 	
        ATTR_PUSHBOX_OFFLINE_PUSH_DELAY_TIME(0)
    };

    size_t offset = 0;
    uint8_t index = 0;

    size_t failed = 0;

    config.enable = get_attribute(attrs, failed, pattern[0], offset).get_bool(); 
    config.minute = get_attribute(attrs, failed, pattern[1], offset).get_uint16(); 

    address = index + 1;

    return (failed == 0);
}


/// @brief 配置顶出控制 
/// @param address 
/// @param position 
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_pushbox_control(
    std::shared_ptr<sacp::SacpClient> client, [[maybe_unused]]uint8_t address, uint8_t control)
{

    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    // 0 - 2 is valid
    // if (position > 100)
    // {
    //     return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InvalidParameter);
    // }
    std::vector<sacp::Attribute> attrs = {
                ATTR_PUSHBOX_CONTROL(control)
            };

    // 写请求
    return client->write_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
}

/// @brief 读顶出盒的信息
/// @param client 
/// @param address 
/// @param info 需要返回的顶出盒的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> read_pushbox_info(
    std::shared_ptr<sacp::SacpClient> client, 
     [[maybe_unused]]uint8_t address, naiad::chassis::MsgDeviceInfo & info)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    // if ((address < 1) || (address > 4))
    // {
    //     return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);
    // }

    std::vector<sacp::Attribute> attrs = {
        ATTR_PUSHBOX_MODEL(""), 		     
        ATTR_PUSHBOX_SN(""), 		         
        ATTR_PUSHBOX_HW_VERSION(0), 		 
        ATTR_PUSHBOX_SW_VERSION(0),		 
        /*ATTR_PUSHBOX_ADDRESS(0), */ 		 
        ATTR_PUSHBOX_NAME(""), 		     
        ATTR_PUSHBOX_LINK_STATUS(0), 	
        ATTR_PUSHBOX_CONNECTED_TIME(0), 	
        ATTR_PUSHBOX_DISCONNECTED_TIME(0)
    };

    // 批量修改属性ID
    // sacp::increase_attributes_id(attrs, (address - 1) * (ATTR_LIFTER_B_ADDRESS_ID - ATTR_PUSHBOX_ADDRESS_ID));

    // 读取指定属性
    auto result = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
    // 如果操作失败，直接返回
    if (result->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result;
    }

    bool ret1 = parse_pushbox_device_brief(result->attributes, address, info.device_brief);
    bool ret2 = parse_pushbox_admin_status(result->attributes, address, info.admin_status);
    
    if (ret1 && ret2)
    {
        return result;
    }

    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::FewAttributesMissed); 
}

/// @brief 获取顶出盒配置
/// @param client 
/// @param address 
/// @param info 需要返回的顶出盒的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> get_pushbox_offline_config(
    std::shared_ptr<sacp::SacpClient> client, 
     [[maybe_unused]]uint8_t address, naiad::chassis::SrvPushBoxGetOfflineConfigResponse & config)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    // if ((address < 1) || (address > 4))
    // {
    //     return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);
    // }

    std::vector<sacp::Attribute> attrs = {
        ATTR_PUSHBOX_OFFLINE_PUSH_ENABLE(0), 	
        ATTR_PUSHBOX_OFFLINE_PUSH_DELAY_TIME(0)
    };

    // 批量修改属性ID
    // sacp::increase_attributes_id(attrs, (address - 1) * (ATTR_LIFTER_B_ADDRESS_ID - ATTR_PUSHBOX_ADDRESS_ID));

    // 读取指定属性
    auto result = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
    // 如果操作失败，直接返回
    if (result->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result;
    }

    bool ret = parse_pushbox_offline_config(result->attributes, address, config);
    
    if (ret)
    {
        return result;
    }

    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::FewAttributesMissed); 
}


/// @brief 修改顶出盒配置
/// @param client 
/// @param address 
/// @param info 需要返回的顶出盒的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_pushbox_offline_config(
    std::shared_ptr<sacp::SacpClient> client, 
     [[maybe_unused]]uint8_t address, naiad::chassis::SrvPushBoxSetOfflineConfigRequest const & config)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    // if ((address < 1) || (address > 4))
    // {
    //     return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);
    // }

    std::vector<sacp::Attribute> attrs = {
        ATTR_PUSHBOX_OFFLINE_PUSH_ENABLE(config.enable), 	
        ATTR_PUSHBOX_OFFLINE_PUSH_DELAY_TIME(config.minute)
    };

    // 批量修改属性ID
    // sacp::increase_attributes_id(attrs, (address - 1) * (ATTR_LIFTER_B_ADDRESS_ID - ATTR_PUSHBOX_ADDRESS_ID));

    // 读取指定属性
    return client->write_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
}

}
}
