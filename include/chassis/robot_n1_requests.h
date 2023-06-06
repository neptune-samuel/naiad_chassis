
#ifndef __ROBOT_N1_REQUESTS_H__
#define __ROBOT_N1_REQUESTS_H__

#include "robot_n1_attributes.h"
#include "sacp_client.h"
#include "node_device.h"

namespace naiad {
namespace chassis {


// std::unique_ptr<SacpClient::OperationResult> (
//     std::string const & from, 
//     sacp::Frame::Priority priority,
//     std::vector<sacp::Attribute> const & attributes)
// {
//     // 发送请求
//     uint32_t request_id = 0;
//     auto ret = submit_request(from, priority, sacp::Frame::OpCode::Read, attributes, request_id);
//     if (ret != OperationStatus::Ok)
//     {
//         return std::make_unique<OperationResult>(ret);
//     }

//     slog::trace("get request id:{}", request_id);

//     return get_result(request_id, true);
// }





/// @brief 设置升降器位置
/// @param address 
/// @param position 
/// @return 
// std::unique_ptr<sacp::SacpClient::OperationResult> set_lifter_position(
//     std::shared_ptr<sacp::SacpClient> client, uint8_t address, uint8_t position)
// {

//     if (!client->is_running())
//     {
//         return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InternalError);    
//     }

//     if (position > 100)
//     {
//         return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::InvalidParameter);
//     }

//     // 同步请求
//     switch(address)
//     {
//         case 1:
//         return client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
//                 ATTR_LIFTER_A_SET_POSITION(position)
//             });        
//         case 2:
//         return client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
//                 ATTR_LIFTER_B_SET_POSITION(position)
//             });
//         case 3:
//         return client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
//                 ATTR_LIFTER_C_SET_POSITION(position)
//             });
//         case 4:
//         return client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
//                 ATTR_LIFTER_D_SET_POSITION(position)
//             });        
//     }
    
//     return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);    
// }


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

    // 同步请求
    switch(address)
    {
        case 1:
        return client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
                ATTR_LIFTER_A_SET_POSITION(position)
            });        
        case 2:
        return client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
                ATTR_LIFTER_B_SET_POSITION(position)
            });
        case 3:
        return client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
                ATTR_LIFTER_C_SET_POSITION(position)
            });
        case 4:
        return client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
                ATTR_LIFTER_D_SET_POSITION(position)
            });        
    }
    
    return std::make_unique<sacp::SacpClient::OperationResult>(sacp::SacpClient::OperationStatus::NoSuchObject);    
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

    // 
    size_t offset = (address - 1) * ATTR_LIFTER_A_MODEL_ARRAY_SIZE;

    // 批量修改属性ID
    sacp::increase_attributes_id(attrs, (address - 1) * ATTR_LIFTER_A_MODEL_ARRAY_SIZE);

    // 读取指定属性
    auto result = client->read_attributes("ros", sacp::Frame::Priority::PriorityLowest, attrs); 
    // 如果操作失败，直接返回
    if (result->status != sacp::SacpClient::OperationStatus::Ok)
    {
        return result;
    }

    do 
    {

        // 获取这个属性，需要判断类型是否一致，然后返回值 

        attr = get_attribute(attrs, id, type);
        if (!failed)
        {

        }
        


        sacp::Attribute attr = sacp::get_attribute(result->attributes, ATTR_LIFTER_A_MODEL_ID + offset);
        if (attr.id() == 0)
        {
            slog::warning("unabled to find attribute[{}]", offset);
            break;
        }
        else 
        {
            info.device_brief.model = ;
        }

        if ()

        info.admin_status.link_status = sacp::get_attribute(result->attributes, id).get_uint8();





    }while(0);


}



}

}


#endif //__ROBOT_N1_REQUESTS_H__
