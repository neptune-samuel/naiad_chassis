
/**
 * @file attribute.cpp
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief 封装SACP属性
 * @version 0.1
 * @date 2023-05-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>
#include <vector>
#include <iostream>
#include <cstring>

#include <libsacp/sacp_type.h>
#include <sacp/attribute.h>

namespace sacp {


enum class Attribute::Type : uint8_t 
{
    Bool   = SACP_TYPE_BOOL,  
    Uint8  = SACP_TYPE_UINT8, 
    Int8   = SACP_TYPE_INT8,  
    Uint16 = SACP_TYPE_UINT16,
    Int16  = SACP_TYPE_INT16, 
    Uint32 = SACP_TYPE_UINT32,
    Int32  = SACP_TYPE_INT32, 
    Float  = SACP_TYPE_FLOAT, 
    Double = SACP_TYPE_DOUBLE,
    Uint64 = SACP_TYPE_UINT64,
    Int64  = SACP_TYPE_INT64, 
    Octet  = SACP_TYPE_OCTET, 
    Status = SACP_TYPE_STATUS,    
};

/// 静态常量，OCTET最大长度
const int Attribute::MaxOctetSize = 128;

const Attribute Attribute::ZeroAttribute(0);

/**
 * @brief 返回类型名称
 * 
 * @param type 
 * @return char const* 
 */
char const *Attribute::TypeName(Type type) 
{
    return SACP_TYPE_NAME(static_cast<int>(type));
}

void Attribute::set_status(uint8_t value) { type_ = Type::Status; value_.uint8_value = value; }
void Attribute::set(bool value) { type_ = Type::Bool; value_.bool_value = value; }
void Attribute::set(uint8_t value) { type_ = Type::Uint8; value_.uint8_value = value; }
void Attribute::set(int8_t value) { type_ = Type::Int8; value_.int8_value = value; }
void Attribute::set(uint16_t value) { type_ = Type::Uint16; value_.uint16_value = value; }
void Attribute::set(int16_t value) { type_ = Type::Status; value_.int16_value = value; }
void Attribute::set(uint32_t value) { type_ = Type::Uint32; value_.uint32_value = value; }
void Attribute::set(int32_t value) { type_ = Type::Int32; value_.int32_value = value; }
void Attribute::set(float value) { type_ = Type::Float; value_.float_value = value; }
void Attribute::set(double value) { type_ = Type::Double; value_.double_value = value; }
void Attribute::set(uint64_t value) { type_ = Type::Uint64; value_.uint64_value = value; }
void Attribute::set(int64_t value) { type_ = Type::Int64; value_.int64_value = value; }
void Attribute::set(uint8_t const *octet, int size) { type_ = Type::Octet; octet_value_.set(octet, size);}
void Attribute::set(const char *str) { type_ = Type::Octet; octet_value_.set((uint8_t const *)str, strlen(str));}
void Attribute::set(std::string const &str) { type_ = Type::Octet; set(str.c_str()); }

std::string Attribute::value_string() const 
{
    switch(type_)
    {
        case Type::Bool: return std::to_string(value_.bool_value);
        case Type::Uint8: return std::to_string(value_.uint8_value);
        case Type::Int8: return std::to_string(value_.int8_value);
        case Type::Uint16: return std::to_string(value_.uint16_value);
        case Type::Int16: return std::to_string(value_.int16_value);
        case Type::Uint32: return std::to_string(value_.uint32_value);
        case Type::Int32: return std::to_string(value_.int32_value);
        case Type::Float: return std::to_string(value_.float_value);
        case Type::Double: return std::to_string(value_.double_value);
        case Type::Uint64: return std::to_string(value_.uint64_value);
        case Type::Int64: return std::to_string(value_.int64_value);
        case Type::Status: return std::to_string(value_.uint8_value);
        case Type::Octet: return octet_value_.to_string();
        default:
            break;
    }
    return std::string("");
}

/// @brief 返回一个子串 ATTR[0023](bool) = true
/// @return 
std::string Attribute::to_string() const 
{   
    char buf[64];
    sprintf(buf, "ATTR[%04d](%s) = ", id_, type_name());
    return std::string(buf) + value_string();
}

/// @brief 返回一个子串 ATTR[0023](bool) = true
/// @return 
std::string Attribute::brief() const
{   
    char buf[64];
    sprintf(buf, "%d(%s) ", id_, value_string().c_str());
    return std::string(buf);
}

int Attribute::size() const
{
    switch(type_)
    {
        case Type::Bool: return sizeof(value_.bool_value);
        case Type::Uint8: return sizeof(value_.uint8_value);
        case Type::Int8: return sizeof(value_.int8_value);
        case Type::Uint16: return sizeof(value_.uint16_value);
        case Type::Int16: return sizeof(value_.int16_value);
        case Type::Uint32: return sizeof(value_.uint32_value);
        case Type::Int32: return sizeof(value_.int32_value);
        case Type::Float: return sizeof(value_.float_value);
        case Type::Double: return sizeof(value_.double_value);
        case Type::Uint64: return sizeof(value_.uint64_value);
        case Type::Int64: return sizeof(value_.int64_value);
        case Type::Status: return sizeof(value_.uint8_value);
        case Type::Octet: return octet_value_.len();
        default:
            break;
    }
    return 0;
}


bool Attribute::type_match(Attribute const & attr) const
{
    return (attr.type_ == type_);
}

bool to_sacp_attribute(Attribute const & attr, void *ptr)
{
    if (ptr == nullptr)
    {
        return false;
    }

    sacpAttribute_t *cattr = (sacpAttribute_t *)ptr;


    cattr->id = attr.id_;
    cattr->type = static_cast<uint8_t>(attr.type_);
    cattr->len = attr.size();

    switch(cattr->type)
    {
        case SACP_TYPE_BOOL:
        cattr->value.v8 = (uint8_t)attr.value_.bool_value;
        break;
        case SACP_TYPE_UINT8:
        cattr->value.v8 = (uint8_t)attr.value_.uint8_value;
        break;
        case SACP_TYPE_INT8:
        cattr->value.v8 = (uint8_t)attr.value_.int8_value;
        break;
        case SACP_TYPE_UINT16:
        cattr->value.v16 = (uint16_t)attr.value_.uint16_value;
        break;
        case SACP_TYPE_INT16:
        cattr->value.v16 = (uint16_t)attr.value_.int16_value;
        break;
        case SACP_TYPE_UINT32:
        cattr->value.v32 = (uint32_t)attr.value_.uint32_value;
        break;
        case SACP_TYPE_INT32:
        cattr->value.v32 = (uint32_t)attr.value_.int32_value;
        break;
        case SACP_TYPE_FLOAT:
        cattr->value.f32 = attr.value_.float_value;
        break;
        case SACP_TYPE_DOUBLE:
        cattr->value.f64 = attr.value_.double_value;
        break;            
        case SACP_TYPE_UINT64:
        cattr->value.v64 = (uint64_t)attr.value_.uint64_value;
        break;
        case SACP_TYPE_INT64:
        cattr->value.v64 = (uint64_t)attr.value_.int64_value;
        break;    
        case SACP_TYPE_OCTET:
        cattr->value.octet = attr.octet_value_.data();
        break;
        case SACP_TYPE_STATUS:
        cattr->value.v8 = attr.value_.uint8_value;                  
        break;
    }        
    return true;
}


bool from_sacp_attribute(Attribute &attr, void *ptr)
{
    if (ptr == nullptr)
    {
        return false;
    }

    sacpAttribute_t *cattr = (sacpAttribute_t *)ptr;

    attr.id_ = cattr->id;
    attr.type_ = (Attribute::Type)cattr->type;

    switch(cattr->type)
    {
        case SACP_TYPE_BOOL:
        attr.value_.bool_value = (bool)cattr->value.v8;
        break;
        case SACP_TYPE_UINT8:
        attr.value_.uint8_value = (uint8_t)cattr->value.v8;
        break;
        case SACP_TYPE_INT8:
        attr.value_.int8_value = (int8_t)cattr->value.v8;
        break;
        case SACP_TYPE_UINT16:
        attr.value_.uint16_value = (uint16_t)cattr->value.v16;
        break;
        case SACP_TYPE_INT16:
        attr.value_.int16_value = (int16_t)cattr->value.v16;
        break;
        case SACP_TYPE_UINT32:
        attr.value_.uint32_value = (uint32_t)cattr->value.v32;
        break;
        case SACP_TYPE_INT32:
        attr.value_.int32_value = (int32_t)cattr->value.v32;
        break;
        case SACP_TYPE_FLOAT:
        attr.value_.float_value = cattr->value.f32;
        break;
        case SACP_TYPE_DOUBLE:
        attr.value_.double_value = cattr->value.f64;
        break;            
        case SACP_TYPE_UINT64:
        attr.value_.uint64_value = (uint64_t)cattr->value.v64;
        break;
        case SACP_TYPE_INT64:
        attr.value_.int64_value = (int64_t)cattr->value.v64;
        break;     
        case SACP_TYPE_OCTET:
        attr.octet_value_.set(cattr->value.octet, cattr->len);
        break;
        case SACP_TYPE_STATUS:
        attr.value_.uint8_value = (uint8_t)cattr->value.v8;                   
        break;
    }

    return true;
}


/**
 * @brief 批量增加属性的ID
 * 
 * @param attrs 
 * @param offset 
 */
void increase_attributes_id(std::vector<Attribute> & attrs, size_t offset)
{
    for (auto & v : attrs)
    {
        v.id_ += offset;
    }
}

/**
 * @brief 批量减少属性的ID
 * 
 * @param attrs 
 * @param offset 
 */
void decrease_attributes_id(std::vector<Attribute> & attrs, size_t offset)
{
    for (auto & v : attrs)
    {
        v.id_ -= offset;
    }
}



} // end sacp

