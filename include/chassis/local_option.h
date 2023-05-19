
#ifndef LOCAL_OPTION_H_
#define LOCAL_OPTION_H_

/**
 * @file local_option.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 本地的选项校验和处理
 * @version 0.1
 * @date 2023-05-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>
#include <docopt/docopt.h>

#include <common/main_option.h>
#include <spdlog/logger.h>

namespace nos 
{

namespace chassis
{

/**
 * @brief 本地选项处理
 * 
 */
class LocalOption: public nos::MainOption
{
public:    
    /**
     * @brief Construct a new Local Option object
     * 
     * @param version 
     * @param argc 
     * @param argv 
     */
    LocalOption(const std::string &version, int argc, const char *argv[]) : MainOption(get_usage(), version, argc, argv) { }

    ~LocalOption() = default;

    /**
     * @brief 返回参数是否有误
     * 
     * @return true 
     * @return false 
     */
    bool check() override 
    {
        // 本地检查选项的准确性
        // 检查 日志选项是否合法
        if (get_log_level() == spdlog::level::off)
        {
            std::cout << "***No such log level: " << args_[option_log_level].asString() << std::endl;
            return false;
        }

        // 检查串口设备是否存在
        auto port = args_[option_serial_port].asString();

        struct stat buf;
        if (::stat(port.c_str(), &buf) != 0)
        {
            std::cout << "***Serial device does not exist: " << port << std::endl;
            return false;
        }
        
        return true;
    } 

    /**
     * @brief 如果返回 spdlog::level::off, 表示有误
     * 
     * @return spdlog::level::level_enum 
     */
    spdlog::level::level_enum get_log_level()
    {

        /*
            const std::string levels[] = {"trace", "debug", "info", "warning", "error"};
            const int num_levels = sizeof(levels) / sizeof(levels[0]);

            bool is_level(const std::string& str) {
                return std::find(levels, levels + num_levels, str) != levels + num_levels;
            }        
        */
        const std::string levels[] = {"trace", "debug", "info", "warning", "error"};
        const std::string short_levels [] = {"t", "d", "i", "w", "e"};
        const spdlog::level::level_enum log_levels[] = {
                spdlog::level::level_enum::trace,
                spdlog::level::level_enum::debug,
                spdlog::level::level_enum::info,
                spdlog::level::level_enum::warn,
                spdlog::level::level_enum::err  
                }; 

        int level_num = sizeof(levels) / sizeof(levels[0]);

        auto opt = args_[option_log_level].asString();

        for (int i = 0; i < level_num; ++ i)
        {
            if (short_levels[i] == opt)
            {
                return log_levels[i];
            }
        }

        for (int i = 0; i < level_num; ++ i)
        {
            if (levels[i] == opt)
            {
                return log_levels[i];
            }
        }

        return spdlog::level::off;
    }

    /// @brief  一些常量
    const std::string option_serial_port = "-s";
    const std::string option_serial_rate = "-r";
    const std::string option_log_level = "-L";
    const std::string option_debug_tcp_port = "-d";

private:
    /**
     * @brief 返回一个常量
     * 
     * @return const char* 
     */
    const char * get_usage()
    {
        static const char usage[] = 
        R"(  
        Usage: 
            nos_chassis [options]

        Options:
            -h, --help     Show this help.
            -v, --version  Show version info.
            -s SERIAL      Set serial device. [default:/dev/ttyUSB0]
            -r RATE        Set baudrate of serial port. [default:115200]
            -d PORT        Set debug tcp listen port. [default:9600]
            -L LEVEL       Set the logging level.
                            Available options: trace, debug, info, warning, error
                            [default: info]
        )";

        return usage;
    }

};

} // end chassis 

} // nos 

#endif // LOCAL_OPTION_H_
