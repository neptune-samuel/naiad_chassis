
#include "portable.h"

#ifdef __no_framework__

/**
 * @brief Tables for CRC16/Modbus
 *
 */
const static uint16_t s_crctalbeabs[] = {
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
};

/**
 * @brief CRC16/Modbus, AC UART-Protocol use CRC16/Modbus
 *
 * @param crc init code
 * @param  data
 * @param len len
 * @return uint16_t
 */
uint16_t crc16Modbus(uint16_t crc, const uint8_t *data, uint32_t len)
{
    uint16_t i;
    uint8_t ch;

    for (i = 0; i < len; i++)
    {
        ch = *data++;
        crc = s_crctalbeabs[(ch ^ crc) & 15] ^ (crc >> 4);
        crc = s_crctalbeabs[((ch >> 4) ^ crc) & 15] ^ (crc >> 4);
    }

    return crc;
}


/**
 * @brief 字符串解析函数，返回字符串中的第index个子串
 * 
 * @param stringArray  输入字串常量，格式如 "aaaa\0bbbb\0"
 * @param index  需要返回的字串索引
 * @param offset 字串偏移量 实现索引为 index - offset
 * @return const char* 
 *  返回子串的地址
 */
const char *stringAt(const char *stringArray, int index, int offset)
{
    int i;
    const char *p;
    
    /* the first one */
    if (index <= offset)
    {
        return stringArray;
    }

    p = stringArray;
    i = 0;
    
    while(i >= 0)
    {       
        if (*p == '\0')
        {
            /* end condition ? */
            if (*(p  + 1) == '\0')
            {
                return p;
            }

            i ++;

            if (i == (index - offset))
            {
                return (p + 1);
            }
        }
        p ++;
    }

    return "";
}

/**
 * @brief 将十六进制数据编码为HEX字串
 * 
 * @param buffer 
 * @param bufferSize 
 * @param data 
 * @param dataSize 
 * @return const char* 
 */
const char *hexStringEncode(char *buffer, int bufferSize, unsigned char *data, int dataSize)
{
    int slen = 0;
    if ((buffer == NULL) || (bufferSize < (dataSize * 2 + 1)))
    {
        return "";
    }

    for (int i = 0; i < dataSize; i ++)
    {
        slen += sprintf(buffer + slen, "%02X", data[i]);
    }
    
    return buffer;
}

#endif // __no_framework__
