
/**
 * @file main.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 小程序遥控器
 * @version 0.1
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <chrono>
#include <memory>

#include "common/logger.h"
#include "rclcpp/rclcpp.hpp"
#include "minirc/node_minirc.h"

#define APP_NAME "naiad_minirc"
#define APP_VERSION  "v0.1.0"


/// @brief 结点主函数
/// @param argc 
/// @param argv 
/// @return 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<naiad::minirc::NodeMiniRc>(APP_NAME);
    slog::info("program started, version: {}, build time: {} {}", APP_VERSION,  __DATE__, __TIME__);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
