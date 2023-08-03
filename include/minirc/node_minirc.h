


#ifndef __NAIAD_NODE_MINIRC_H__
#define __NAIAD_NODE_MINIRC_H__

/**
 * @file node_minirc.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "common/logger.h"
#include "common/tcp_server.h"

#include "chassis/chassis_type.h"


namespace naiad 
{

namespace minirc
{

/*
列出运行参数
$ ros2 param list
/naiad_fog:
  data_period
  dump_file
  serial_options
  serial_port
  state_publish_period
  stdout_log_level
  use_sim_time
  vofa_debuger

在线修改参数 
  ros2 param set /naiad_chassis serial_options 460800

启动时指定参数
 ./bin/naiad_chassis --ros-args -p serial_port:=/dev/ttyTHS0
*/

class NodeMiniRc:public rclcpp::Node 
{
public:
    
    static const std::string TcpPort; 
    static const std::string StdoutLogLevel;
    static const std::string MotionControlPeriod;

    enum class MiniRcEvent
    {
        None = 0,
        SingleClick,
        Pressed,
        Released,
    };

    enum class MiniRcKey
    {
        KeyNone = 0,
        KeyHello,
        KeyPower,
        KeyHome,
        KeyApp,
        KeyUp,
        KeyRight,
        KeyConfirm,
        KeyDown,
        KeyLeft,
        KeyBack,
        KeyPlus,
        KeyMinus,
        KeyMute,
        KeyFan,
        KeyConvert,
        KeyPath,
        KeyAi,
        KeyExtA,
        KeyExtB,
        KeyExtC,
        KeyExtD,
        KeyExtE,        
    };

    static const std::map<std::string, MiniRcKey> KeyMap;

    // 构造函数
    NodeMiniRc(std::string const &name);

private:
    /// 日志接口
    std::shared_ptr<slog::Logger> log_;    
    /// TCP服务
    std::shared_ptr<naiad::network::TcpServer> tcp_;
    /// 运动控制定时器
    rclcpp::TimerBase::SharedPtr motion_publish_timer_;  

    bool motion_actived = false;
    naiad::chassis::MsgMotionControl motion_control;

    /// 运动控制
    rclcpp::Publisher<naiad::chassis::MsgMotionControl>::SharedPtr motion_publisher_;   

    /// LED灯
    // rclcpp::Client<naiad::chassis::SrvLedLightSetBrightness>::SharedPtr set_led_brightness_client_;   
    // rclcpp::Client<naiad::chassis::SrvLifterSetPosition>::SharedPtr set_lifter_position_client_;
    //rclcpp::Client<naiad::chassis::SrvPushBoxControl>::SharedPtr pushbox_action_client_;    

    /**
     * @brief 命令事件处理
     * 
     * @param key 
     * @param event 
     * @param value 
     */
    void receive_command_handle(MiniRcKey key, MiniRcEvent event, int value);

    /// @brief 定时发布状态处理函数
    /// @param  
    void motion_publish_timer_handle(void);

    /// 返回Key
    MiniRcKey getKey(const std::string &cmd)
    {
        auto it = KeyMap.find(cmd);
        if (it != KeyMap.end()){
          return it->second;
        }
        return MiniRcKey::KeyNone;        
    }

};


} // minirc
} // naiad


#endif // __NAIAD_NODE_MINIRC_H__


