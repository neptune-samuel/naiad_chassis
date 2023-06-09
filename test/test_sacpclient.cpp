

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

#include <sacp_client/sacp_client.h>

#include <cxxopts.hpp>
#include <iostream>
#include <memory>

#define APP_NAME  "sacp_client"
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
  cxxopts::Options options(APP_NAME, "SACP Test Client");

  options.add_options()
    ("h,help",    "Print help message")
    ("v,version", "Show version info")
    ("s,serial",  "Serial device name", cxxopts::value<std::string>()->default_value("/dev/ttyUSB0"))
    ("r,rate",    "Serial baudrate", cxxopts::value<int>()->default_value("460800"))
    ("t,tcp-port",      "Port of TCP server for debug", cxxopts::value<int>()->default_value("9600"))
    ("l,log-level",     "Logger level:'t' 'd' 'i' 'w' 'e'", cxxopts::value<std::string>()->default_value("info"))
    ("V,vofa",          "VOFA service for visualizatioin(PORT:PERIOD:ATTR1,ATTR2,...ATTRn)", cxxopts::value<std::vector<std::string>>())
    ;

  auto result = options.parse(argc, argv);


    std::cout << "help: " << result.count("help") << std::endl;
    std::cout << "v: " << result.count("v") << std::endl;
    std::cout << "s: " << result.count("s") << std::endl;
    std::cout << "r: " << result.count("r") << std::endl;
    std::cout << "t: " << result.count("t") << std::endl;
    std::cout << "l: " << result.count("l") << std::endl;
    std::cout << "V: " << result.count("V") << std::endl;

//   if (result.count("help")) {
//     std::cout << options.help() << std::endl;
//     return 0;
//   }

//   if (result.count("v"))
//   {
//     std::cout << "todo: print version" << std::endl;
//     return 0;
//   }

//   if (result.count("log-level"))
//   {
//     std::cout << "todo: set log level" << std::endl;
//     return 0;    
//   }


  return 0;
}

    // // 解析参数
    // auto opt = LocalOption(APP_VERSION, argc, argv);
    // // 显示参数
    // //opt.dump();

    // if (!opt.check())
    // {
    //     return 1;
    // }

    // // 先初始化日志
    // slog::make_stdout_logger(APP_NAME, opt.get_log_level());
    // slog::info("{} started, build time: {} {}", APP_VERSION,  __DATE__, __TIME__);

    // uv::Loop loop;
    
    // // 注册信号处理函数
    // auto signal_handle = [&]([[maybe_unused]]int signum){            
    //         loop.stop();
    //     };

    // loop.signal(SIGINT, signal_handle);
    // loop.signal(SIGTERM, signal_handle);
    // loop.signal(SIGKILL, signal_handle);
    // loop.signal(SIGABRT, signal_handle);

    // auto sacp_client = std::make_shared<sacp::SacpClient>(opt.get_string(opt.SerialPort), std::to_string(opt.get_int(opt.Baudrate)), opt.get_int(opt.DebugTcpPort));
    
    // // 启动SACP调试
    // {
    //     int port = 0, period = 0;
    //     std::vector<uint32_t> datas;

    //     if (sacp::VofaDebuger::ParseConfig(opt.get_string(opt.VofaConfigs), port, period, datas))
    //     {
    //         sacp_client->create_vofa_monitor_service(port, datas, period);
    //     }

    // }
    
    // // 启动SACP服务
    // sacp_client->start();


    // //定时控制升降器 
    // uv::Timer timer;

    // timer.bind(loop);

    // timer.start(5000, [&]{
        
    //     static bool lifter_state = false;
    //     uint8_t value = lifter_state ? 0 : 100;
    //     lifter_state = !lifter_state;

    //     sacp_client->write_attributes("test", sacp::Frame::Priority::PriorityLowest,
    //         {
    //             sacp::Attribute(619 + 25, value)
    //         }
    //     );
    // });




    // loop.spin();
    // sacp_client->stop();

    // slog::warning(APP_NAME "  exited");

