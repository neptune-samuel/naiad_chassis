
/**
 * @file pumpbox_request.cpp
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
#include "chassis/node_peripheral.h"


namespace robot{
namespace n1
{


/**
 * @brief 解析泵压力盒的设备简介
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_pumpbox_device_brief(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::MsgDeviceBreif &device)
{
    const sacp::AttributeArray pattern = 
    {
        ATTR_PUMPBOX_MODEL(""), 		     
        ATTR_PUMPBOX_SN(""), 		         
        ATTR_PUMPBOX_HW_VERSION(0), 		 
        ATTR_PUMPBOX_SW_VERSION(0),		 
        ATTR_PUMPBOX_ADDRESS(0), 		 
        ATTR_PUMPBOX_NAME("")
    };

    size_t offset = 0;
    uint8_t index = 0;

    // if (!parse_attributes_range(attrs, ATTR_PUMPBOX_MODEL_ID, ATTR_LIFTER_B_MODEL_ID, offset, index))
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
 * @brief 解析泵压力盒的管理状态
 * 
 * @param attrs 收到的属性
 * @param address 返回的地址
 * @param device 返回的信息
 * @return true 
 * @return false 
 */
bool parse_pumpbox_admin_status(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::MsgAdminStatus &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_PUMPBOX_LINK_STATUS(0), 	
        ATTR_PUMPBOX_CONNECTED_TIME(0), 	
        ATTR_PUMPBOX_DISCONNECTED_TIME(0)
    };

    size_t offset = 0;
    uint8_t index = 0;

    // if (!parse_attributes_range(attrs, ATTR_PUMPBOX_LINK_STATUS_ID, ATTR_LIFTER_B_LINK_STATUS_ID, offset, index))
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
 * @brief 解析泵压力盒的设备状态信息
 * 
 * @param attrs 
 * @param address 
 * @param device 
 * @return true 
 * @return false 
 */
bool parse_pumpbox_device_state(sacp::AttributeArray const &attrs, 
   uint8_t & address, naiad::chassis::MsgPumpBoxState &device)
{

    const sacp::AttributeArray pattern = 
    {
        ATTR_PUMPBOX_STATE(0),
        ATTR_PUMPBOX_TEMPERATURE(0),
        ATTR_PUMPBOX_HUMIDITY(0),
        ATTR_PUMPBOX_SENSOR_NUM(0),
        ATTR_PUMPBOX_SENSOR1_STATUS(0),
        ATTR_PUMPBOX_SENSOR1_VALUE(0),
        ATTR_PUMPBOX_SENSOR2_STATUS(0),
        ATTR_PUMPBOX_SENSOR2_VALUE(0),
        ATTR_PUMPBOX_SENSOR3_STATUS(0),
        ATTR_PUMPBOX_SENSOR3_VALUE(0),
        ATTR_PUMPBOX_SENSOR4_STATUS(0),
        ATTR_PUMPBOX_SENSOR4_VALUE(0),
        ATTR_PUMPBOX_SENSOR5_STATUS(0),
        ATTR_PUMPBOX_SENSOR5_VALUE(0),
    };

    size_t offset = 0;
    uint8_t index = 0;

    // if (!parse_attributes_range(attrs, ATTR_PUMPBOX_POSITION_ID, ATTR_LIFTER_B_POSITION_ID, offset, index))
    // {
    //     return false;
    // }   

    size_t failed = 0;

    device.state = get_attribute(attrs, failed, pattern[0], offset).get_uint16(); 
    device.temperature = get_attribute(attrs, failed, pattern[1], offset).get_float(); 
    device.humidity = get_attribute(attrs, failed, pattern[2], offset).get_float();    
    uint8_t num = get_attribute(attrs,  failed, pattern[3], offset).get_uint8(); 

    if (num == 0)
    {
        slog::warning("no pressure datas");
    }
    else 
    {
        if (num > 5)
        {
            num = 5;
        }

        for (int i = 0; i < num; i ++)
        {
            device.sensor_states[i] = get_attribute(attrs,  failed, pattern[4 + (i * 2)], offset).get_uint8(); 
            device.pressures[i] = get_attribute(attrs,  failed, pattern[5 + (i * 2)], offset).get_float(); 
        }
    }


    address = index + 1;

    return (failed == 0);
}



/// @brief 读泵压力盒的信息
/// @param client 
/// @param address 
/// @param info 需要返回的泵压力盒的信息
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> read_pumpbox_info(
    std::shared_ptr<sacp::SacpClient> client, 
    uint8_t address, naiad::chassis::MsgDeviceInfo & info)
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
        ATTR_PUMPBOX_MODEL(""), 		     
        ATTR_PUMPBOX_SN(""), 		         
        ATTR_PUMPBOX_HW_VERSION(0), 		 
        ATTR_PUMPBOX_SW_VERSION(0),		 
        /*ATTR_PUMPBOX_ADDRESS(0), */ 		 
        ATTR_PUMPBOX_NAME(""), 		     
        ATTR_PUMPBOX_LINK_STATUS(0), 	
        ATTR_PUMPBOX_CONNECTED_TIME(0), 	
        ATTR_PUMPBOX_DISCONNECTED_TIME(0)
    };

    // 批量修改属性ID
    // sacp::increase_attributes_id(attrs, (address - 1) * (ATTR_LIFTER_B_ADDRESS_ID - ATTR_PUMPBOX_ADDRESS_ID));

    // 读取指定属性
    auto result = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
    // 如果操作失败，直接返回
    if (result->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result;
    }

    bool ret1 = parse_pumpbox_device_brief(result->attributes, address, info.device_brief);
    bool ret2 = parse_pumpbox_admin_status(result->attributes, address, info.admin_status);
    
    if (ret1 && ret2)
    {
        return result;
    }

    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::FewAttributesMissed); 
}

    
}
}

