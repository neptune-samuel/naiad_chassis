

    void set(sacpAttribute_t const & attr)
    {
        id_ = attr.id;
        type_ = static_cast<Type>(attr.type);

        switch(attr.type)
        {
            case SACP_TYPE_BOOL:
            value_.bool_value = (bool)attr.value.v8;
            break;
            case SACP_TYPE_UINT8:
            value_.uint8_value = (uint8_t)attr.value.v8;
            break;
            case SACP_TYPE_INT8:
            value_.int8_value = (int8_t)attr.value.v8;
            break;
            case SACP_TYPE_UINT16:
            value_.uint16_value = (uint16_t)attr.value.v16;
            break;
            case SACP_TYPE_INT16:
            value_.int16_value = (int16_t)attr.value.v16;
            break;
            case SACP_TYPE_UINT32:
            value_.uint32_value = (uint32_t)attr.value.v32;
            break;
            case SACP_TYPE_INT32:
            value_.int32_value = (int32_t)attr.value.v32;
            break;
            case SACP_TYPE_FLOAT:
            value_.float_value = attr.value.f32;
            break;
            case SACP_TYPE_DOUBLE:
            value_.double_value = attr.value.f64;
            break;            
			case SACP_TYPE_UINT64:
			value_.uint64_value = (uint64_t)attr.value.v64;
            break;
            case SACP_TYPE_INT64:
            value_.int64_value = (int64_t)attr.value.v64;
            break;     
            case SACP_TYPE_OCTET:
            octet_value_.set(attr.value.octet, attr.len);
            break;
            case SACP_TYPE_STATUS:
            value_.uint8_value = (uint8_t)attr.value.v8;                   
            break;
        }
    }

    /**
     * @brief 获取当前类型名称
     * 
     * @return char const* 
     */
    char const *type_name()
    {
        return Attribute::TypeName(type_);
    }

    std::string value_string()
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
    std::string to_string()
    {   
        char buf[64];
        sprintf(buf, "ATTR[%04d](%s) = ", id_, type_name());
        return std::string(buf) + value_string();
    }

    int size() const
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

    uint16_t id() const 
    {
        return id_;
    }



    /// 转换成C的结构体
    void to_sacp_attribute(sacpAttribute_t & attr)
    {
        attr.id = id_;
        attr.type = static_cast<uint8_t>(type_);
        attr.len = size();

        switch(attr.type)
        {
            case SACP_TYPE_BOOL:
            attr.value.v8 = (uint8_t)value_.bool_value;
            break;
            case SACP_TYPE_UINT8:
            attr.value.v8 = (uint8_t)value_.uint8_value;
            break;
            case SACP_TYPE_INT8:
            attr.value.v8 = (uint8_t)value_.int8_value;
            break;
            case SACP_TYPE_UINT16:
            attr.value.v16 = (uint16_t)value_.uint16_value;
            break;
            case SACP_TYPE_INT16:
            attr.value.v16 = (uint16_t)value_.int16_value;
            break;
            case SACP_TYPE_UINT32:
            attr.value.v32 = (uint32_t)value_.uint32_value;
            break;
            case SACP_TYPE_INT32:
            attr.value.v32 = (uint32_t)value_.int32_value;
            break;
            case SACP_TYPE_FLOAT:
            attr.value.f32 = value_.float_value;
            break;
            case SACP_TYPE_DOUBLE:
            attr.value.f64 = value_.double_value;
            break;            
            case SACP_TYPE_UINT64:
            attr.value.v64 = (uint64_t)value_.uint64_value;
            break;
            case SACP_TYPE_INT64:
            attr.value.v64 = (uint64_t)value_.int64_value;
            break;    
            case SACP_TYPE_OCTET:
            attr.value.octet = octet_value_.data();
            break;
            case SACP_TYPE_STATUS:
            attr.value.v8 = value_.uint8_value;                  
            break;
        }        
    }

    /**
     * @brief 静态函数，获取类型名称
     * 
     * @param type 
     * @return char const* 
     */
    static char const *TypeName(Type type)
    {
        return SACP_TYPE_NAME(static_cast<int>(type));
    }

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

    // 内部使用, 不允许复制
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

        OctetValue(const OctetValue &) = delete;
        OctetValue & operator=(const OctetValue &) = delete;

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
            char buffer[Attribute::MaxOctetSize * 3];
            
            #define tohex(_h) (((_h) > 9) ? (((_h) - 10) + 'A') : ((_h) + '0'))

            sprintf(buffer, "(%d)", len_);

            int n = strlen(buffer);
            for (int i = 0; (i < len_) && (n < sizeof(buffer) - 1); i ++)
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

    uint16_t id_;
    Type type_;

    BasicValue value_;
    OctetValue octet_value_;
};

