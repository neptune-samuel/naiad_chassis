
#ifndef __ROBOT_N1_REQUESTS_H__
#define __ROBOT_N1_REQUESTS_H__

#include "robot_n1_attributes.h"
#include "sacp_client.h"
#include "node_device.h"

namespace naiad {
namespace chassis {


bool parse_lifter_device_brief(sacp::AttributeArray const &attrs, 
   uint8_t addresss, naiad::chassis::MsgDeviceBreif &device)
{

    static const sacp::AttributeArray pattern = 
    {
        ATTR_LIFTER_A_MODEL(""), 		     
        ATTR_LIFTER_A_SN(""), 		         
        ATTR_LIFTER_A_HW_VERSION(0), 		 
        ATTR_LIFTER_A_SW_VERSION(0),		 
        ATTR_LIFTER_A_ADDRESS(0), 		 
        ATTR_LIFTER_A_NAME("")
    };

    size_t offset = (address - 1) * ATTR_LIFTER_A_ARRAY_SIZE;

    int failed = 0;

    dev.device_brief.model = sacp::get_attribute(attrs, failed, pattern[0], offset).get_string(); 
    dev.device_brief.model = sacp::get_attribute(attrs, failed, pattern[1], offset).get_uint16(); 
    dev.device_brief.model = sacp::get_attribute(attrs, failed, pattern[2], offset); 
    dev.device_brief.model = sacp::get_attribute(attrs, failed, pattern[3], offset); 
    dev.device_brief.model = sacp::get_attribute(attrs, failed, pattern[4], offset); 
    dev.device_brief.model = sacp::get_attribute(attrs, failed, pattern[5], offset); 

    return (failed == 0);
}


bool parse_lifter_admin_status(sacp::AttributeArray const &attrs, 
   uint8_t addresss, naiad::chassis::MsgAdminStatus &device)
{

    static const sacp::AttributeArray pattern = 
    {
        ATTR_LIFTER_A_LINK_STATUS(0), 	
        ATTR_LIFTER_A_CONNECTED_TIME(0), 	
        ATTR_LIFTER_A_DISCONNECTED_TIME(0)
    };

    size_t offset = (address - 1) * ATTR_LIFTER_A_ARRAY_SIZE;

    int failed = 0;

    dev.device_brief.model = sacp::get_attribute(attrs, failed, pattern[0], offset).get_string(); 
    dev.device_brief.model = sacp::get_attribute(attrs, failed, pattern[1], offset).get_uint16(); 
    dev.device_brief.model = sacp::get_attribute(attrs, failed, pattern[2], offset); 

    return (failed == 0);
}







/// @brief 设置升降器位置
/// @param address 
/// @param position 
/// @return 
std::unique_ptr<sacp::SacpClient::OperationResult> set_lifter_position(
    std::shared_ptr<sacp::SacpClient> client, uint8_t address, uint8_t position)
{

    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    if (position > 100)
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InvalidParameter);
    }

    if ((address < 1) || (address > 4))
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);
    }

    std::vector<sacp::Attribute> attrs = {
                ATTR_LIFTER_A_SET_POSITION(position)
            };
    // 批量修改属性ID
    sacp::increase_attributes_id(attrs, (address - 1) * ATTR_LIFTER_A_ARRAY_SIZE);

    // 写请求
    return client->write_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
}


std::unique_ptr<sacp::SacpClient::OperationResult> read_lifter_info(
    std::shared_ptr<sacp::SacpClient> client, 
    uint8_t address, naiad::chassis::MsgDeviceInfo & info)
{
    if (!client->is_running())
    {
        return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
    }

    if ((address < 1) || (address > 4))
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
    sacp::increase_attributes_id(attrs, (address - 1) * ATTR_LIFTER_A_ARRAY_SIZE);

    // 读取指定属性
    auto result = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
    // 如果操作失败，直接返回
    if (result->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return std::move(result);
    }


    bool ret1 = parse_lifter_device_brief(result->attributes, address, info->device_brief);
    bool ret2 = parse_lifter_admin_status(result->attributes, address, info->admin_status);
    
    if (ret1 && ret2)
    {
        return std::move(result);
    }

    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::FewAttributesMissed); 
}



}

}


#endif //__ROBOT_N1_REQUESTS_H__
