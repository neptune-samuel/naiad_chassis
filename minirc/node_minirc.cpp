
#include <string>
#include <chrono>

#include "common/sys_time.h"
#include "common/logger.h"

#include "rclcpp/rclcpp.hpp"
#include "minirc/node_minirc.h"



namespace naiad 
{
namespace minirc
{

/// @brief 字串分割
/// @param str 
/// @param delimiter 
/// @return 
std::vector<std::string> string_split(const std::string& str, char delimiter) 
{
    std::vector<std::string> tokens;
    size_t start = 0, end = 0;
    while ((end = str.find(delimiter, start)) != std::string::npos) {
        tokens.push_back(str.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(str.substr(start));
    return tokens;
}

/// 参数名称 - 常量
const std::string NodeMiniRc::TcpPort = "tcp_port"; 
const std::string NodeMiniRc::MotionControlPeriod = "motion_control_period";
const std::string NodeMiniRc::StdoutLogLevel = "stdout_log_level";

/// @brief  键值映射 
const std::map<std::string, NodeMiniRc::MiniRcKey> NodeMiniRc::KeyMap = {
    {"hello", NodeMiniRc::MiniRcKey::KeyHello},
    {"power", NodeMiniRc::MiniRcKey::KeyPower},
    {"home", NodeMiniRc::MiniRcKey::KeyHome},
    {"app", NodeMiniRc::MiniRcKey::KeyApp},
    {"up", NodeMiniRc::MiniRcKey::KeyUp},
    {"right", NodeMiniRc::MiniRcKey::KeyRight},
    {"confirm", NodeMiniRc::MiniRcKey::KeyConfirm},
    {"down", NodeMiniRc::MiniRcKey::KeyDown},
    {"left", NodeMiniRc::MiniRcKey::KeyLeft},
    {"back", NodeMiniRc::MiniRcKey::KeyBack},
    {"plus", NodeMiniRc::MiniRcKey::KeyPlus},
    {"minus", NodeMiniRc::MiniRcKey::KeyMinus},
    {"mute", NodeMiniRc::MiniRcKey::KeyMute},
    {"fan", NodeMiniRc::MiniRcKey::KeyFan},
    {"convert", NodeMiniRc::MiniRcKey::KeyConvert},
    {"path", NodeMiniRc::MiniRcKey::KeyPath},
    {"ai", NodeMiniRc::MiniRcKey::KeyAi},
    {"ext_a", NodeMiniRc::MiniRcKey::KeyExtA},
    {"ext_b", NodeMiniRc::MiniRcKey::KeyExtB},
    {"ext_c", NodeMiniRc::MiniRcKey::KeyExtC},
    {"ext_d", NodeMiniRc::MiniRcKey::KeyExtD},
    {"ext_e", NodeMiniRc::MiniRcKey::KeyExtE},
};

NodeMiniRc::NodeMiniRc(std::string const &name): rclcpp::Node(name)
{ 
    motion_actived = false;
    motion_control.angle = 0.0f;
    motion_control.velocity = 0.0f;

    this->declare_parameter(MotionControlPeriod, 20);
    this->declare_parameter(TcpPort, 9800);
    this->declare_parameter(StdoutLogLevel, "");

    // 根据参数类型，启动一个日志
    std::string const & stdout_loglevel = get_parameter(StdoutLogLevel).as_string();
    if (!stdout_loglevel.empty()){
        // 是否为合法的日志等级
        auto level = slog::log_level_from_name(stdout_loglevel);
        if (level == slog::LogLevel::None){
            std::cout << "Unknown stdout log level, use default(info)" << std::endl;
            level = slog::LogLevel::Info;
        }
        log_ = slog::make_stdout_logger(this->get_name(), level);
    } else {
        log_ = slog::make_ros_logger(this->get_name());
    }

    // 打印一些参数信息
    log_->info("parameter {}={}", TcpPort, get_parameter(TcpPort).as_int());
    log_->info("parameter {}={}", StdoutLogLevel,  get_parameter(StdoutLogLevel).as_string()); 
    log_->info("parameter {}={}", MotionControlPeriod,  get_parameter(MotionControlPeriod).as_int());

    /// 创建一个FOG设备
    tcp_ = std::make_shared<naiad::network::TcpServer>("minirc", "0.0.0.0", get_parameter(TcpPort).as_int());

    /// 初始化发布者 
    motion_publisher_ = this->create_publisher<naiad::chassis::MsgMotionControl>("motion/control", 10);
    
    int period = get_parameter(MotionControlPeriod).as_int();
    if (period < 10){
        std::cout << "Motion publish period is too small, set to 10 ms" << std::endl;
        period = 10;
    }
    
    // 创建一个定时器，发布FOG状态数据
    motion_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(period), \
        std::bind(&NodeMiniRc::motion_publish_timer_handle, this));

    // 工作在回调函数的方式
    tcp_->start([this]([[maybe_unused]]naiad::network::Host const &host, void const * const data, std::size_t size){
        char buffer[32];
        if (size >= sizeof(buffer)){
            size = sizeof(buffer) - 1;
        }
        ::memcpy(buffer, data, size);
        buffer[size] = '\0'; 
        if (buffer[size - 1] == 0x0A){
            buffer[size - 1] = '\0';
        }

        log_->debug("input: {}", buffer);

        // 解析命令字串 
        /*
            remote #confirm
            remote #confirm
            remote >confirm 1     
            remote <right 
        */       
        std::string cmd(buffer);
        std::vector<std::string> args = string_split(cmd, ' ');
        if ((args.size() < 2) || (args[0] != "remote"))
        {
            return;
        }

        MiniRcEvent event = MiniRcEvent::None;
        int value = 0;
        char ch = args[1].at(0);

        if (ch == '#')
        {
            event = MiniRcEvent::SingleClick;
        }
        else if (ch == '>')
        {
            event = MiniRcEvent::Pressed;
        }
        else if (ch == '<')
        {
            event = MiniRcEvent::Released;
        }

        // 删除前面一个字节
        args[1].erase(0, 1);
   
        if (args.size() >= 3)
        {
            value = strtoul(args[2].c_str(), NULL, 0);
        }

        MiniRcKey key = getKey(args[1]);

        if ((key != MiniRcKey::KeyNone) && (event != MiniRcEvent::None))
        {
            receive_command_handle(key, event, value);
        }
    });     
}

/**
 * @brief 命令事件处理
 * 
 * @param key 
 * @param event 
 * @param value 
 */
void NodeMiniRc::receive_command_handle(MiniRcKey key, MiniRcEvent event, int value)
{
    slog::info("command: key {}, event {} value {}", (int)key, (int)event, value);

    // 控制前面左右
    if (event == MiniRcEvent::Pressed)
    {   
        switch(key)
        {
            case MiniRcKey::KeyUp: 
                motion_control.velocity = 0.2f;
                motion_control.angle = 0.0f;
                motion_actived = true;
            break;
            case MiniRcKey::KeyDown: 
                motion_control.velocity = -0.2f;
                motion_control.angle = 0.0f;
                motion_actived = true;
            break;
            case MiniRcKey::KeyRight:
                motion_control.velocity = 0.2f;
                motion_control.angle = 15.0f;
                motion_actived = true;
            break;
            case MiniRcKey::KeyLeft:
                motion_control.velocity = 0.2f;
                motion_control.angle = -15.0f;
                motion_actived = true;
            break; 
            default:
            break;                                   
        }
    }
    else if (event == MiniRcEvent::Released)
    {
        // 停止
        motion_control.velocity = 0.0f;
        motion_control.angle = 0.0f;
    }
}



/// @brief 定时发布状态处理函数
/// @param  
void NodeMiniRc::motion_publish_timer_handle(void)
{
    if (motion_actived)
    {
        motion_publisher_->publish(motion_control);
        if (motion_control.velocity == 0.0f)
        {
            motion_actived = false;
        }
    }
}


} // minirc
} // naiad 

