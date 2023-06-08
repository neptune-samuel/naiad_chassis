
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

#include <sys/stat.h>

#include <string>
#include <docopt/docopt.h>

#include <common/main_option.h>
#include <common/logger.h>


/**
 * @brief 本地选项处理
 * 
 */
class LocalOption: public naiad::MainOption
{
public:    

    /// @brief  一些常量
    static const std::string SerialPort;
    static const std::string Baudrate;
    static const std::string LogLevel;
    static const std::string DebugTcpPort;

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
        if (get_log_level() == slog::LogLevel::None)
        {
            std::cout << "***Invalid or empty log-level: " << get_string(LogLevel) << std::endl;
            return false;
        }

        // 检查是否有默认值
        if (!test_option(DebugTcpPort))
        {
            std::cout << "***usage error: no default value for '" << DebugTcpPort << "'" << std::endl;            
            return false;
        }

        if (!test_option(SerialPort))
        {
            std::cout << "***usage error: no default value for '" << SerialPort << "'" << std::endl;            
            return false;
        }

        if (!test_option(Baudrate))
        {
            std::cout << "***usage error: no default value for '" << Baudrate << "'" << std::endl;            
            return false;
        }

        // 检查串口设备是否存在    
        auto port = get_string(SerialPort);

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
     * @return slog::LogLevel
     */
    slog::LogLevel get_log_level()
    {
        const std::string levels[] = {"trace", "debug", "info", "warning", "error"};
        const std::string short_levels [] = {"t", "d", "i", "w", "e"};
        const slog::LogLevel log_levels[] = {
                    slog::LogLevel::Trace,
                    slog::LogLevel::Debug,
                    slog::LogLevel::Info,
                    slog::LogLevel::Warning,
                    slog::LogLevel::Error,
                }; 

        int level_num = sizeof(levels) / sizeof(levels[0]);

        if (!args_[LogLevel])
        {
            std::cout << "***usage error: no default value for '-L'" << std::endl;
            return slog::LogLevel::None;
        }

        std::string opt = args_[LogLevel].asString();

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

        return slog::LogLevel::None;
    }

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
    -s SERIAL      Set serial device. [default: /dev/ttyUSB0]
    -r RATE        Set baudrate of serial port. [default: 460800]
    -d PORT        Set debug tcp listen port. [default: 9600]
    -L LEVEL       Set the logging level.
                    Available options: trace, debug, info, warning, error
                    [default: info]
)";

        return usage;
    }

};

const std::string LocalOption::SerialPort = "-s";
const std::string LocalOption::Baudrate = "-r";
const std::string LocalOption::LogLevel = "-L";
const std::string LocalOption::DebugTcpPort = "-d";


#endif // LOCAL_OPTION_H_
