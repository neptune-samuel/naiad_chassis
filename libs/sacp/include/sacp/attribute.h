
#ifndef __CXX_SACP_ATTRIBUTE_H__
#define __CXX_SACP_ATTRIBUTE_H__

/**
 * @file attribute.h
 * @author Liu Chuansen (samule@neptune-robotics.com)
 * @brief SACP属性封装
 * @version 0.1
 * @date 2023-05-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>
#include <vector>
#include <iostream>
#include <cstring>

namespace sacp {

class Attribute
{
public:
    /// 定义属性类型
    enum class Type : uint8_t;

    Attribute() : id_(0) { }    
    explicit Attribute(uint16_t id) : id_(id) , octet_value_(0) { }

    Attribute(uint16_t id, bool value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, uint8_t value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, int8_t value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, uint16_t value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, int16_t value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, uint32_t value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, int32_t value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, float value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, double value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, int64_t value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, uint64_t value) : Attribute(id) { set(value); }
    Attribute(uint16_t id, uint8_t const *octet, int size) : id_(id), octet_value_(MaxOctetSize) { set(octet, size); }    
    Attribute(uint16_t id, const char *str) : id_(id), octet_value_(MaxOctetSize) { set(str); }    
    Attribute(uint16_t id, std::string const & str) : id_(id), octet_value_(MaxOctetSize) { set(str); }  
    
    void set(bool value);
    void set(uint8_t value);
    void set(int8_t value);
    void set(uint16_t value);
    void set(int16_t value);
    void set(uint32_t value);
    void set(int32_t value);
    void set(float value);
    void set(double value);
    void set(uint64_t value);
    void set(int64_t value);
    void set(uint8_t const *octet, int size);
    void set(const char *str);
    void set(std::string const &str);
    void set_status(uint8_t value);

    /**
     * @brief 获取当前类型名称
     * 
     * @return char const* 
     */
    char const *type_name() const 
    {
        return Attribute::TypeName(type_);
    }

    std::string value_string() const ;

    /// @brief 返回一个子串 ATTR[0023](bool) = true
    /// @return 
    std::string to_string() const ;

    std::string brief() const ;

    int size() const;

    uint16_t id() const 
    {
        return id_;
    }

    bool get_bool() const { return value_.bool_value; }
    uint8_t get_uint8() const { return value_.uint8_value; }
    int8_t get_int8() const { return value_.int8_value; }
    uint16_t get_uint16() const { return value_.uint16_value; }
    int16_t get_int16() const { return value_.int16_value; }
    uint32_t get_uint32() const { return value_.uint32_value; }
    int32_t get_int32() const { return value_.int32_value; }
    float get_float() const { return value_.float_value; }
    double get_double() const { return value_.double_value; }
    uint64_t get_uint64() const { return value_.uint64_value; }
    int64_t get_int64() const { return value_.int64_value; }
    uint8_t get_status() const { return value_.uint8_value; }
    const uint8_t *get_octet() const { return octet_value_.data(); }


    /**
     * @brief 静态函数，获取类型名称
     * 
     * @param type 
     * @return char const* 
     */
    static char const *TypeName(Type type);
    static const int MaxOctetSize;

private:
    /// 基本类型的联合体
    union BasicValue
    {
        bool bool_value;
        uint8_t uint8_value;
        int8_t int8_value;
        uint16_t uint16_value;
        int16_t int16_value;
        uint32_t uint32_value;
        int32_t int32_value;
        uint64_t uint64_value;        
        int64_t int64_value;
        float float_value;
        double double_value;
    };

    /// OCTET类型
    struct OctetValue 
    {
        /// @brief 默认构造函数，所有为空
        OctetValue(): capacity_(0), len_(0), data_(nullptr) { }

        /// @brief 创建一个指定大小的OCTET值 
        /// @param capacity 
        explicit OctetValue(int capacity) : capacity_(capacity), len_(0)
        {
            if (capacity_ > 0)
            {
                data_ = new uint8_t [capacity_];
                memset(data_, 0, capacity_);
            }
            else 
            {
                data_ = nullptr;
            }            
        }

        ~OctetValue() { 
            if (data_ != nullptr)
            {
                delete [] data_;
            }
        }

        // 复制构造函数
        OctetValue(OctetValue const & other)
        {
            capacity_ = other.capacity_;
            len_ = other.len_;

            if (capacity_ > 0)
            {
                data_ = new uint8_t [capacity_];
                memcpy(data_, other.data_, capacity_);
            }
            else 
            {
                data_ = nullptr;
            }
        }

        OctetValue & operator=(OctetValue const &) = delete;

        int capacity() const 
        {
            return data_ ? capacity_ : 0;
        }

        int len() const 
        {
            return data_ ? len_ : 0;
        }

        const uint8_t * data() const 
        {
            return data_;
        }

        /**
         * @brief 设置值 
         * 
         * @param data 
         * @param len 
         * @return bool 
         */
        void set(uint8_t const *data, int len)
        {
            // 如果容量不够，创建新的
            if (data_ == nullptr || capacity_ < len)
            {
                if (data_)
                {
                    delete [] data_;
                }

                capacity_ = (len < Attribute::MaxOctetSize) ? Attribute::MaxOctetSize : len;
                data_ = new uint8_t[capacity_];
            }

            if (data && len)
            {
                memcpy(data_, data, len);
                len_ = len;
            }
        }

        uint8_t & operator [](int index) 
        {
            if (index < 0 || index >= capacity_)
            {
                static uint8_t dummy = 0;
                return dummy;
            }
            
            return data_[index];
        }

        std::string to_hex() const
        {
            char buffer[512];
            
            #define tohex(_h) (((_h) > 9) ? (((_h) - 10) + 'A') : ((_h) + '0'))

            sprintf(buffer, "(%d)", len_);

            int n = strlen(buffer);
            for (int i = 0; (i < len_) && (n < (int)(sizeof(buffer) - 1)); i ++)
            {
                buffer[n ++] = tohex((data_[i] >> 4) & 0x0f);
                buffer[n ++] = tohex(data_[i] & 0x0f);
            }
            buffer[n] = '\0';            

            return std::string(buffer);
        }

        std::string to_string()const 
        {
            if (len_ == 0)
            {
                return std::string("");
            }

            if (!isprint(data_[0]))
            {
                return to_hex();
            }

            std::string ret;
            for (int i = 0; i < len_; ++ i)
            {
                if (isprint(data_[i]))
                {
                    ret.push_back(data_[i]);
                }
                else 
                {
                    ret.push_back('.');
                }                
            }
            
            return ret;
        }

    private:
        // 当前大小
        int capacity_;
        // 容量
        int len_;        
        uint8_t *data_;
    };

    /// 属性ID
    uint16_t id_;
    /// 属性类型
    Type type_;

    BasicValue value_;
    OctetValue octet_value_;

    // 转换为C类型的属性
    friend bool to_sacp_attribute(Attribute const & attr, void *ptr);
    friend bool from_sacp_attribute(Attribute &attr, void *ptr);
};

/**
 * @brief 将Attribute转换成C结构体
 * 
 * @param attr 
 * @param ptr sacpAttribute_t *
 * @return true 
 * @return false 
 */
bool to_sacp_attribute(Attribute const & attr, void *ptr);

/**
 * @brief 使用C结构体设置属性
 * 
 * @param attr 
 * @param ptr sacpAttribute_t *
 * @return true 
 * @return false 
 */
bool from_sacp_attribute(Attribute &attr, void *ptr);


} // end sacp


#endif // __CXX_SACP_ATTRIBUTE_H__


