



#ifndef __NAIAD_NODE_PUMPBOX_H__
#define __NAIAD_NODE_PUMPBOX_H__

/**
 * @file node_device.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 这是一个设备结点类模板
 * @version 0.1
 * @date 2023-06-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>

#include "common/logger.h"
#include "rclcpp/rclcpp.hpp"

#include "chassis/node_device.h"
#include "robot_n1/robot_n1.h"
#include "chassis/chassis_type.h"

namespace naiad 
{
namespace chassis
{

class NodePumpBox: public NodeDevice<MsgPumpBoxState> 
{
public:
    NodePumpBox(std::string const &type, std::shared_ptr<sacp::SacpClient> sacp_client) : 
        NodeDevice<MsgPumpBoxState>(type), sacp_client_(sacp_client) 
    {
      
    }


    /// @brief 处理上报文
    /// @param attributes 
    void report_handle(uint8_t group, std::vector<sacp::Attribute> const & attributes) override
    {
        // 处理自己的上报报文
        switch(group) 
        {
            case REPORT_ID(_REPORT_PUMPBOX_BASE, _DEVICE_INFO):  
            {
                uint8_t address = 0;
                MsgDeviceBreif brief;
                bool result = robot::n1::parse_pumpbox_device_brief(attributes, address, brief);
                if (result)
                {
                    slog::trace("parse pumpbox({}) brief info success", address);
                    set_device_brief(address, brief);
                }            
            }
            break;

            case REPORT_ID(_REPORT_PUMPBOX_BASE, _ADMIN_STATUS):
            {
                uint8_t address = 0;
                MsgAdminStatus status;
                bool result = robot::n1::parse_pumpbox_admin_status(attributes, address, status);
                if (result)
                {
                    slog::trace("parse pumpbox({}) admin status success", address);
                    report_admin_status(address, status);
                }
            }        
            break;

            case REPORT_ID(_REPORT_PUMPBOX_BASE, _RUNNING_STATE):
            {
                uint8_t address = 0;
                MsgPumpBoxState state;
                bool result = robot::n1::parse_pumpbox_device_state(attributes, address, state);
                if (result)
                {
                    slog::trace("parse pumpbox({}) device state success", address);
                    report_device_state(address, state);
                }
            }        
            break;
        }
    } 

    /// @brief  主动获取设备信息
    /// @param address 
    /// @param info 
    /// @return 
    bool get_device_info(uint8_t address, MsgDeviceInfo & info) override
    {        
        auto result = robot::n1::read_pumpbox_info(sacp_client_, address, info);
        return (result->status == sacp::SacpClient::OperationStatus::Ok);        
    }    

private:

    /// SACP客户端
    std::shared_ptr<sacp::SacpClient> sacp_client_;

};


}
}

#endif // __NAIAD_NODE_PUMPBOX_H__


