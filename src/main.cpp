

/**
 * @file test_tcp_server.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 演示如何使用TCP服务
 * @version 0.1
 * @date 2023-05-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <iostream>
#include <thread>
#include <chrono>

#include <common/logger.h>
#include <common/sys_time.h>
#include <common/uv_helper.h>
#include <common/tcp_server.h>
#include <common/serial_port.h>

#include <chassis/local_option.h>
#include <chassis/sacp_client.h>

#define APP_NAME  "nos_chassis"

#define APP_VERSION  APP_NAME " v1.0"

/**
 * @brief 主函数
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, const char *argv[])
{
    // 解析参数
    auto opt = nos::chassis::LocalOption(APP_VERSION, argc, argv);
    // 显示参数
    //opt.dump();

    if (!opt.check())
    {
        return 1;
    }

    // 先初始化日志
    slog::make_logger(APP_NAME, opt.get_log_level());
    slog::info("{} started, build time: {} {}", APP_VERSION,  __DATE__, __TIME__);

    //spdlog::enable_backtrace(32);
    uv::Loop loop;
    // 创建一个sacp客户端实例
    nos::chassis::SacpClient sacp(opt.get_string(opt.SerialPort), std::to_string(opt.get_int(opt.Baudrate)), opt.get_int(opt.DebugTcpPort));

    // 注册信号处理函数
    auto signal_handle = [&sacp](uv::Loop &loop, [[maybe_unused]]int signum){            
            sacp.stop();
            loop.stop();
        };

    loop.signal(SIGINT, signal_handle);
    loop.signal(SIGTERM, signal_handle);
    loop.signal(SIGKILL, signal_handle);
    loop.signal(SIGABRT, signal_handle);

    // 启动SACP服务
    sacp.start();

    // // 创建一个定时器，做一些事情 
    uv::Timer timer;
    timer.bind(loop);

    timer.start(100, [&]([[maybe_unused]]uv::Timer &self){

        // 同步请求
        auto result = sacp.read_attributes("ros", sacp::Frame::Priority::PriorityLowest, {
                sacp::Attribute(600, ""),
                sacp::Attribute(601, ""),
                sacp::Attribute(602, (uint16_t)0),
                sacp::Attribute(603, (uint16_t)0),
                sacp::Attribute(604, (uint8_t)0),
                sacp::Attribute(605, ""),
                sacp::Attribute(606, (uint8_t)0),
                sacp::Attribute(607, (uint32_t)0),
                sacp::Attribute(608, (uint32_t)0),
                sacp::Attribute(609, (uint8_t)0),
                sacp::Attribute(610, (uint16_t)0),
            });

        slog::info("read return '{}'", result->to_string());
    });

    uv::Timer timer_test;
    timer_test.bind(loop);

    timer_test.start(5000, [&]([[maybe_unused]]uv::Timer &self){

        static uint8_t lifter_position = 0;
        // 同步请求
        if (lifter_position == 0)
        {
            lifter_position = 100;
        }
        else 
        {
            lifter_position = 0;
        }

        auto result = sacp.write_attributes("ros", sacp::Frame::Priority::PriorityHighest, {
                sacp::Attribute(619, (uint8_t)lifter_position)
            });

        slog::info("write return '{}'", result->to_string());
    });



    loop.spin();
    sacp.stop();

    slog::warning(APP_NAME "  exited");

    return 0;
}




