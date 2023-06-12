

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

#include <common/cxxargs.h>
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

    // 波特率检查，仅判断是否为数字吧
    auto check_number = [](std::string const & arg, [[maybe_unused]]std::string & msg)->bool {
            return cxxargs::helper::is_number(arg);
        };

    //检查日志
    auto check_loglevel = [](std::string const & arg, [[maybe_unused]]std::string & msg)->bool {
        return (slog::log_level_from_name(arg) != slog::LogLevel::None);
    };

    // 检查VOFA的配置
    auto check_vofa_option =  [](std::string const & arg, [[maybe_unused]]std::string & msg)->bool {
        int port = 0, period = 0;
        std::vector<uint32_t> datas;
        bool ret = sacp::VofaDebuger::ParseConfig(arg, port, period, datas);
        if (ret == false){
            msg = "Incorred format, e.g:9700:0:100,101,102";
        }
        return ret;
    };

    // 定义参数ID，方便获取
    enum class ArgId: int {Help, LogLevel, Serial, Baudrate, TcpPort, VofaDebug};

    // 选项
    auto args =
    cxxargs::Parser<ArgId>(APP_NAME)
        .option(ArgId::Help,        "-h,--help",                "Print this message")
        .option(ArgId::LogLevel,    "-l,--log-level <level>",   "Set log level, accept: trace,debug,info,warning,error", "info", check_loglevel)
        .option(ArgId::Serial,      "-s,--serial <device>",     "Set serial device", "/dev/ttyUSB0")
        .option(ArgId::Baudrate,    "-r,--rate <rate>",         "Set serial baudrate", "460800", check_number)
        .option(ArgId::TcpPort,    "-t,--tcp-port <port>",      "Set debug TCP port", "9600", check_number)
        .option(ArgId::VofaDebug,   "--vofa <config>...",       "Enable VOFA+ data source, format: PORT:PERIOD:ATTR1,ATTR2...", check_vofa_option)
        .set_help(ArgId::Help);

    // 解析命令行参数
    args.parse(argc, argv);

    // 先初始化日志
    slog::make_stdout_logger(APP_NAME, slog::log_level_from_name(args.get(ArgId::LogLevel).as_string()));
    slog::info("{} started, build time: {} {}", APP_VERSION,  __DATE__, __TIME__);

    uv::Loop loop;
    
    // 注册信号处理函数
    auto signal_handle = [&]([[maybe_unused]]int signum){            
            loop.stop();
        };

    loop.signal(SIGINT, signal_handle);
    loop.signal(SIGTERM, signal_handle);
    loop.signal(SIGKILL, signal_handle);
    loop.signal(SIGABRT, signal_handle);

    // 创建一个SACP客户端
    auto sacp_client = std::make_shared<sacp::SacpClient>(args.get(ArgId::Serial).as_string(),
        args.get(ArgId::Baudrate).as_string(), static_cast<int>(args.get(ArgId::TcpPort).as_number()));

    // 设定VOFA调试参数
    {
        int port = 0, period = 0;
        std::vector<uint32_t> datas;

        auto &configs = args.get(ArgId::VofaDebug).as_string_list();

        for (auto & cfg: configs){
            if (sacp::VofaDebuger::ParseConfig(cfg, port, period, datas)){
                sacp_client->create_vofa_monitor_service(port, datas, period);
            }
        }
    }
    
    // // 启动SACP服务
    sacp_client->start();

    //定时控制升降器 
    uv::Timer timer;
    timer.bind(loop);
    timer.start(5000, [&]{
        static bool lifter_state = false;
        uint8_t value = lifter_state ? 0 : 100;
        lifter_state = !lifter_state;
        sacp_client->write_attributes("test", sacp::Frame::Priority::PriorityLowest, {sacp::Attribute(619, value)});
    });

    
    loop.spin();
    sacp_client->stop();

    slog::warning(APP_NAME "  exited");

    return 0;
}

