
#ifndef __ROBOT_N1_ATTRIBUTE_HELPER_H__
#define __ROBOT_N1_ATTRIBUTE_HELPER_H__

#include "common/logger.h"
#include "sacp/attribute.h"

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
sacp::Attribute const & get_attribute(sacp::AttributeArray const & attrs, 
    size_t & failed_count, sacp::Attribute const &pattern, size_t id_offset);

/**
 * @brief 查找是否有任一属性，有的话，返回属性ID
 * 
 * @param attrs 
 * @param ids 
 * @return uint16_t 
 */
uint16_t has_attribute(sacp::AttributeArray const & attrs, sacp::AttributeIdPattern const & ids);


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
    uint16_t first_id, uint16_t second_id, size_t & offset, uint8_t & index);

}

}

#endif // __ROBOT_N1_ATTRIBUTE_HELPER_H__
