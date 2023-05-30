


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

