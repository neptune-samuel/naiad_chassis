
/**
 * @file sacp_payload.h
 * @author Liu Chuansen (179712066@qq.com)
 * @brief 简单属性控制协议
 * @version 0.1
 * @date 2022-10-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __SACP_PORTABLE_H__
#define __SACP_PORTABLE_H__

#ifdef __no_framework__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>


#ifndef __PACKED
#define __PACKED           __attribute__((packed)) 
#endif


#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_a)  (sizeof(_a) / sizeof(_a[0]))
#endif


#define htoles(_s) (_s)
#define htolel(_l) (_l)
#define htolell(_ll) (_ll)

#define letohs(_s) (_s)
#define letohl(_l) (_l)
#define letohll(_ll) (_ll)

#define osMalloc(_s)  malloc(_s)
#define osFree(_v)    free(_v)

#define osMemcpy(_dst, _src, _size)     memcpy(_dst, _src, _size)
#define osMemmove(_dst, _src, _size)    memmove(_dst, _src, _size)
#define osMemcmp(_dst, _src, _size)     memcmp(_dst, _src, _size)
#define osMemset(_dst, _val, _size)     memset(_dst, _val, _size)

#define osStrcpy(_dst, _src)            strcpy(_dst, _src)
#define osStrncpy(_dst, _src, _size)    strncpy(_dst, _src, _size)
#define osStrcmp(_dst, _src)            strcmp(_dst, _src)
#define osStrncmp(_dst, _src, _size)    strncmp(_dst, _src, _size)


extern uint16_t crc16Modbus(uint16_t crc, const uint8_t *data, uint32_t len);
extern const char *stringAt(const char *stringArray, int index, int offset);
extern const char *hexStringEncode(char *buffer, int bufferSize, unsigned char *data, int dataSize);

/// rawPrint(...) 
#define rawPrint(...)  printf(__VA_ARGS__)

#define elog(...)      do {printf(__VA_ARGS__);printf("\r\n");}while(0)
#define wlog(...)      do {printf(__VA_ARGS__);printf("\r\n");}while(0)
#define ilog(...)      do {printf(__VA_ARGS__);printf("\r\n");}while(0)
#define dlog(...)      do {printf(__VA_ARGS__);printf("\r\n");}while(0)

#else 

#include <common/generic.h>
#include <common/byte_order.h>
#include <misc/string_helper.h>
#include <misc/simple_crc.h>

#endif //not __no_framework__

#endif // __SACP_PORTABLE_H__


