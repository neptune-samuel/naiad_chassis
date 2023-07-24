
#include "common/logger.h"

#include "sacp/attribute.h"
#include "device_index.h"

namespace robot {
namespace n1 {


/**
 * @brief 返回一个只读的属性常量, 如果有错误，将错误计数加1
 * 
 * @param attrs 待读取的属性列表
 * @param failed_count 错误计数
 * @param pattern 属性影子
 * @param id_offset ID偏移量
 * @param print_log 是否打印日志
 * @return Attribute const& 
 */
sacp::Attribute const & get_attribute(
    sacp::AttributeArray const & attrs, 
    size_t & failed_count,
    sacp::Attribute const &pattern, 
    size_t id_offset)
{
    uint16_t id = pattern.id() + id_offset;
    auto it = std::find_if(attrs.begin(), attrs.end(), [&](sacp::Attribute const & attr){
        return (attr.id() == id);
    });

    if (it == attrs.end()){
        failed_count ++;
        slog::warning("attribute[{}] missed, please check", id);
        return sacp::Attribute::ZeroAttribute;
    }

    //是否类型相同
    if (!pattern.type_match(*it)) {
        failed_count ++;
        slog::warning("attribute[{}] type unmatched, got {}, expect {}");
        return sacp::Attribute::ZeroAttribute;
    }

    return *it;
}

/**
 * @brief 查找是否有任一属性，有的话，返回属性ID
 * 
 * @param attrs 
 * @param ids 
 * @return uint16_t 
 */
uint16_t has_attribute(sacp::AttributeArray const & attrs, sacp::AttributeIdPattern const & ids)
{
    for (auto v : ids)
    {
        auto it = std::find_if(attrs.begin(), attrs.end(), [&](sacp::Attribute const & attr){
                    return (attr.id() == v); 
                });       
        if (it != attrs.end())
        {
            return v;
        }
    }

    return 0;
}


/**
 * @brief 解析属性的地址范围，返回属性的偏移量及设备索引
 * 
 * @param first 
 * @param second 
 * @param offset 属性偏移量
 * @param index  设备索引，从0开始  
 * @return true 
 * @return false 
 */
bool parse_attributes_range(sacp::AttributeArray const &attrs, 
    uint16_t first_id, uint16_t second_id, size_t & offset, DeviceIndex & index)
{

    // 表示不需要查找，仅单实例
    if (second_id <= first_id)
    {
        offset = 0;
        index = DeviceIndex::SingleInstance;
        return true;
    }

    // 支持多实现，目前支持8个
    sacp::AttributeIdPattern range_check = {};
    // 动态生成8个ID，查找有哪些
    for (int i = 0; i < 8; i ++)
    {
        range_check.push_back(first_id + i * (second_id - first_id));
    }

    // 先看看属性在哪个区间
    uint16_t id = has_attribute(attrs, range_check);

    // 没有接收到的
    if (id < first_id)
    {
        slog::warning("unabled to find correct attributes");
        return false;
    }


    offset = id - first_id;
    // 将索引转换为索引类型
    index = static_cast<DeviceIndex>(offset / (second_id - first_id));

    return true;
}


}

}
