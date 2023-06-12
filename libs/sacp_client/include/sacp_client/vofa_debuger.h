
#ifndef __SACP_VOFA_DEBUGER_H__
#define __SACP_VOFA_DEBUGER_H__

/**
 * @file vofa_debuger.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 实现一个VOFA的调试器，数据可视化
 * @version 0.1
 * @date 2023-06-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>

#include "common/logger.h"
#include "common/vofa_service.h"
#include "common/uv_helper.h"
#include "sacp/attribute.h"


namespace sacp
{


class VofaDebuger 
{
public:
    VofaDebuger() { }
    ~VofaDebuger() { }
    
    /**
     * @brief 开启一个调试服务
     * 
     * @param port 
     * @param ids 
     * @param period 
     * @return bool 返回是否成功
     */
    bool create(int port, std::vector<uint32_t> const & datas, int period = 0)
    {
        // 检查是否有效: 端口可用，数据集有效
        // 是否已存在，如果已存在，就直接返回false
        if (find_service(port) != nullptr)
        {
            slog::warning("start vofa service(port:{}) failed, service exists", port);
            return false;
        }

        if (datas.empty())
        {
            slog::warning("start vofa service(port:{}) failed, no data provided", port);
            return false;
        }

        std::shared_ptr<naiad::network::VofaService> vofa = std::make_shared<naiad::network::VofaService>("0.0.0.0", port, datas, period);

        if (!vofa->start())
        {
            slog::warning("start vofa service(port:{}) failed, tcp started failed", port);
            return false;
        }

        // 启动成功, 加入管理队列中
        vofa_services_.push_back(vofa);

        slog::info("start vofa service(port:{}) success", port);
        return true;
    }

    /**
     * @brief 删除指定端口的服务
     * 
     * @param port 
     */
    void destroy(int port)
    {
        auto it = std::find_if(vofa_services_.begin(), vofa_services_.end(), [&](naiad::network::VofaService::SharedPtr const & service){
            return service->get_port() == port;
        });

        if (it != vofa_services_.end())
        {
            // 找到了一个
            slog::info("remove vofa service(port:{})", port);
            vofa_services_.erase(it);
        }
    }

    /**
     * @brief 暂停指定端口的服务
     * 
     * @param port 
     */
    void pause(int port)
    {
        auto service = find_service(port);

        if (service != nullptr)
        {
            // 停止服务
            service->stop();
        }        
    }

    /**
     * @brief 恢复指定端口的服务
     * 
     * @param port 
     */
    void resume(int port)
    {
        auto service = find_service(port);

        if ((service != nullptr) && !service->is_running())
        {
            // 停止服务
            service->start();
        }
    }

    /**
     * @brief 输入可视化的数据
     * 
     * @param attributes 
     */
    void input(sacp::AttributeArray const & attributes)
    {
        // 如果没有开启任何服务，不做任何事情 
        if (vofa_services_.empty())
        {
            return;
        }

        std::map<uint32_t, float> datas = DataFromAttributes(attributes);

        if (!datas.empty())
        {
            for (auto & service : vofa_services_)
            {
                service->input(datas);
            }
        }
    }

    /**
     * @brief 这个函数用来解析一个输入字串，格式为  PORT:PERIOD:DATAS, 
     *    其中DATAS为使用逗号分割的数据ID值， 
     *    如: "9700:0:1,2,3,4,5,6"
     * 
     * @param input 输入字串
     * @param port 
     * @param period 
     * @param data_set 
     * @return true 
     * @return false 
     */
    static bool ParseConfig(const std::string & input, int &port, int & period, std::vector<uint32_t> & data_set)
    {
        std::stringstream ss(input);
        std::string token;

        if (!std::getline(ss, token, ':')){
            return false;
        }

        port = std::stoi(token);

        if (!std::getline(ss, token, ':')){
            return false;
        }

        period = std::stoi(token);

        if (!std::getline(ss, token, ':')){
            return false;
        }

        std::stringstream ss2(token);

        while (std::getline(ss2, token, ','))
        {
            data_set.push_back(std::stoi(token));
        }

        return (data_set.size() > 0) ? true : false;
    }

    /**
     * @brief 这个函数将SACP属性转换为浮点数据集,不处理字符串类型
     * 
     * @param attributes 
     * @return std::map<uint32_t, float> 
     */
    static std::map<uint32_t, float> DataFromAttributes(sacp::AttributeArray const & attributes)
    {
        // 只处理整数类型的数值
        std::map<uint32_t, float> datas;

        for (auto & attr : attributes)
        {
            float value = 0;
            if (attribute_to_float(attr, value))
            {
                datas[attr.id()] = value;
            }
        }
        
        return datas;
    }

private:
    // 保存VOFA向量组
    std::vector<naiad::network::VofaService::SharedPtr> vofa_services_; 



    /**
     * @brief 基于端口查找一个服务
     * 
     * @param port 
     * @return naiad::network::VofaService::SharedPtr 
     */
    naiad::network::VofaService::SharedPtr find_service(int port)
    {
        auto it = std::find_if(vofa_services_.begin(), vofa_services_.end(), [&](naiad::network::VofaService::SharedPtr const & service){
            return service->get_port() == port;
        });

        if (it != vofa_services_.end())
        {
            return *it;
        }

        return nullptr;
    }

    /**
     * @brief 将属性值转换为浮点数
     * 
     * @param attr 
     * @param value 
     * @return true 
     * @return false 
     */
    static bool attribute_to_float(sacp::Attribute const & attr, float & value)
    {
        if (attr.type_id() == typeid(uint8_t)) { value = static_cast<float>(attr.get_uint8()); return true; }
        if (attr.type_id() == typeid(uint16_t)) { value = static_cast<float>(attr.get_uint16()); return true; }
        if (attr.type_id() == typeid(uint32_t)) { value = static_cast<float>(attr.get_uint32()); return true; }
        if (attr.type_id() == typeid(int8_t)) { value = static_cast<float>(attr.get_int8()); return true; }
        if (attr.type_id() == typeid(int16_t)) { value = static_cast<float>(attr.get_int16()); return true; }
        if (attr.type_id() == typeid(int32_t)) { value = static_cast<float>(attr.get_int32()); return true; }
        if (attr.type_id() == typeid(int64_t)) { value = static_cast<float>(attr.get_int64()); return true; }
        if (attr.type_id() == typeid(uint64_t)) { value = static_cast<float>(attr.get_uint64()); return true; }
        if (attr.type_id() == typeid(double)) { value = static_cast<float>(attr.get_double()); return true; }
        if (attr.type_id() == typeid(float)) { value = attr.get_float(); return true; }    

        if (attr.type_id() == typeid(bool))
        {
            value = attr.get_bool() ? 1.0f : 0.0f;
            return true;
        }

        return false;
    }

};



}

#endif // __SACP_VOFA_DEBUGER_H__
