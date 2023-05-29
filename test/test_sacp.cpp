


#include <iostream>
#include <vector>

#include <common/logger.h>

#include <sacp/attribute.h>
#include <sacp/frame.h>
#include <sacp/stream.h>

namespace nlog = slog;


static void test_attribute()
{
    sacp::Attribute a1(123, true);
    sacp::Attribute a2(4000, (uint8_t)32);
    sacp::Attribute a3(3210, "hello");
    sacp::Attribute a4(4000, 24.987f);

    std::cout << a1.to_string() << std::endl;
    std::cout << a2.to_string() << std::endl;
    std::cout << a3.to_string() << std::endl;
    std::cout << a4.to_string() << std::endl;

    std::cout << a1.get_bool() << std::endl;

    uint8_t buf[6] = { 0xee, 0x44, 0x55, 0xef, 0xa4, 0xff};

    sacp::Attribute a5(230, buf, 6);
    
    std::cout << a5.to_string() << std::endl;

}


static void test_stream()
{
    std::vector<sacp::Attribute> attrs;
    attrs.emplace_back(100, (bool)true);
    attrs.emplace_back(101, (int8_t)123);
    attrs.emplace_back(102, (uint32_t)0x123456);
    attrs.emplace_back(103, "hello");

    sacp::Frame f1("test", sacp::Frame::Priority::Priority0, 1, sacp::Frame::OpCode::Read, attrs);
    sacp::Frame f2("test", sacp::Frame::Priority::Priority0, 1, sacp::Frame::OpCode::Write, attrs);
    sacp::Frame f3("test", sacp::Frame::Priority::Priority0, 1, sacp::Frame::OpCode::Report, attrs);

    
    uint8_t buf1[256];
    int size1;

    uint8_t buf2[256];
    int size2;

    uint8_t buf3[256];
    int size3;

    
    slog::info("f1: {}", f1.brief());
    size1 = f1.make_raw_frame(buf1, sizeof(buf1));
    slog::info_data(buf1, size1, "raw:");

    slog::info("f2: {}", f2.brief());
    size2 = f2.make_raw_frame(buf2, sizeof(buf2));
    slog::info_data(buf2, size2, "raw:");

    f3.add_attribute(sacp::Attribute(34, (float)3.1415f));
    slog::info("f3: {}", f3.brief());
    size3 = f3.make_raw_frame(buf3, sizeof(buf3));
    slog::info_data(buf3, size3, "raw:");


    sacp::Stream stream("serial");

    int ret = stream.input(buf1, size1);
    slog::info("stream.input() return {}", ret);

    ret = stream.input(buf2, size2);
    slog::info("stream.input() return {}", ret);

    uint8_t buf[2] = { 0xaa, 0xaa};

    stream.input(buf, 2);


    ret = stream.input(buf3, size3);
    slog::info("stream.input() return {}", ret);

    // 模拟丢了2字节
    ret = stream.input(buf3, size3 - 2);
    slog::info("stream.input() return {}", ret);

    ret = stream.input(buf2, size2);
    slog::info("stream.input() return {}", ret);

    ret = stream.input(buf1, size1);
    slog::info("stream.input() return {}", ret);

    while (stream.frames_num() > 0)
    {
        auto f = stream.receive();
        if (!f->is_empty())
        {
            slog::info("rx frame: {}", f->info());
        }        
    }

}


int main(int argc, const char** argv)
{

    slog::make_logger("test_sacp", slog::LogLevel::Debug);

    //slog::info("hello world, ret=%d", 1234);

    //test_attribute();

    test_stream();

    return 0;
}


/*
以下是一个简单的例程，演示如何封装spdlog为一个新的库，并使用该库进行日志记录：

1. 创建一个新的C++工程，例如"MyLogger"，并将spdlog的头文件和源文件添加到该工程中。

2. 在MyLogger命名空间中创建一个类"MyLogger"：

```cpp
#include <spdlog/spdlog.h>

namespace MyLogger {
    class MyLogger {
    public:
        MyLogger();
        static MyLogger& getLogger();
        void info(const std::string& message);
        void warn(const std::string& message);
        void error(const std::string& message);
    private:
        std::shared_ptr<spdlog::logger> m_logger;
    };
}
```

3. 在MyLogger类的构造函数中初始化默认的日志记录器：

```cpp
MyLogger::MyLogger() {
    m_logger = spdlog::stdout_color_mt("MyLogger");
}
```

4. 在MyLogger类中添加一个静态成员函数getLogger()，用于返回默认的日志记录器：

```cpp
MyLogger::MyLogger& MyLogger::MyLogger::getLogger() {
    static MyLogger logger;
    return logger;
}
```

5. 在MyLogger类中添加其他成员函数，例如info、warn、error，用于记录不同级别的日志：

```cpp
void MyLogger::MyLogger::info(const std::string& message) {
    m_logger->info(message);
}

void MyLogger::MyLogger::warn(const std::string& message) {
    m_logger->warn(message);
}

void MyLogger::MyLogger::error(const std::string& message) {
    m_logger->error(message);
}
```

6. 在MyLogger类中添加其他成员函数，例如setLevel、setPattern、setSink，用于设置日志级别、日志格式、日志输出：

```cpp
void MyLogger::MyLogger::setLevel(spdlog::level::level_enum level) {
    m_logger->set_level(level);
}

void MyLogger::MyLogger::setPattern(const std::string& pattern) {
    m_logger->set_pattern(pattern);
}

void MyLogger::MyLogger::setSink(const std::string& filename) {
    m_logger->set_sink(std::make_shared<spdlog::sinks::simple_file_sink_mt>(filename));
}
```

7. 编译MyLogger类成一个静态库或动态库，并将其导出。

8. 在其他项目中使用该库时，只需要包含MyLogger的头文件，并链接MyLogger库即可。

9. 在使用MyLogger时，可以直接调用MyLogger::getLogger()获取默认的日志记录器，并调用其他成员函数进行日志记录：

```cpp
#include "MyLogger/MyLogger.h"

int main() {
    MyLogger::MyLogger::getLogger().info("Hello, world!");
    MyLogger::MyLogger::getLogger().setLevel(spdlog::level::warn);
    MyLogger::MyLogger::getLogger().warn("This is a warning message.");
    MyLogger::MyLogger::getLogger().setPattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [thread %t] %v");
    MyLogger::MyLogger::getLogger().setSink("log.txt");
    MyLogger::MyLogger::getLogger().error("This is an error message.");
    return 0;
}
```
			
*/