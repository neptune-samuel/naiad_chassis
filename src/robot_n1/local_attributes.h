
#ifndef __ROBOT_N1_ATTRIBUTES_H__
#define __ROBOT_N1_ATTRIBUTES_H__

#include <sacp/attribute.h>


// -- ROBOT_N1属性定义 


// 这个文件由工程的(robot_controller_mcu)的属性管理工具自动生成


/* 属性组 - system */

#define ATTR_APPLICATION_ID_ID 		1
#define ATTR_APPLICATION_ID(_val) 		sacp::Attribute(1, (uint16_t)_val) 		/* 1    ATTR_APPLICATION_ID                      uint16_t RD */
#define ATTR_APPLICATION_VERSION_ID 		2
#define ATTR_APPLICATION_VERSION(_val) 		sacp::Attribute(2, (uint16_t)_val) 		/* 2    ATTR_APPLICATION_VERSION                 uint16_t RD */
#define ATTR_HARDWARE_ID_ID 		3
#define ATTR_HARDWARE_ID(_val) 		sacp::Attribute(3, (uint32_t)_val) 		/* 3    ATTR_HARDWARE_ID                         uint32_t RD */
#define ATTR_HARDWARE_VERSION_ID 		4
#define ATTR_HARDWARE_VERSION(_val) 		sacp::Attribute(4, (uint32_t)_val) 		/* 4    ATTR_HARDWARE_VERSION                    uint32_t RD */
#define ATTR_SOFTWARE_VERSION_ID 		5
#define ATTR_SOFTWARE_VERSION(_val) 		sacp::Attribute(5, (uint32_t)_val) 		/* 5    ATTR_SOFTWARE_VERSION                    uint32_t RD */
#define ATTR_SOFTWARE_RELEASED_TIME_ID 		6
#define ATTR_SOFTWARE_RELEASED_TIME(_val) 		sacp::Attribute(6, (uint32_t)_val) 		/* 6    ATTR_SOFTWARE_RELEASED_TIME              uint32_t RD */
#define ATTR_UP_TIME_ID 		9
#define ATTR_UP_TIME(_val) 		sacp::Attribute(9, (uint32_t)_val) 		/* 9    ATTR_UP_TIME                             uint32_t RD */
#define ATTR_RTC_TIME_ID 		10
#define ATTR_RTC_TIME(_val) 		sacp::Attribute(10, (uint32_t)_val) 		/* 10   ATTR_RTC_TIME                            uint32_t RW */
#define ATTR_CPU_USAGED_ID 		12
#define ATTR_CPU_USAGED(_val) 		sacp::Attribute(12, (float)_val) 		/* 12   ATTR_CPU_USAGED                          float  RD */
#define ATTR_MEM_TOTAL_ID 		13
#define ATTR_MEM_TOTAL(_val) 		sacp::Attribute(13, (uint32_t)_val) 		/* 13   ATTR_MEM_TOTAL                           uint32_t RD */
#define ATTR_MEM_FREE_ID 		14
#define ATTR_MEM_FREE(_val) 		sacp::Attribute(14, (uint32_t)_val) 		/* 14   ATTR_MEM_FREE                            uint32_t RD */
#define ATTR_PRODUCT_MODEL_ID 		15
#define ATTR_PRODUCT_MODEL(_val) 		sacp::Attribute(15, _val) 		/* 15   ATTR_PRODUCT_MODEL                       octet  RD */
#define ATTR_PRODUCT_SN_ID 		16
#define ATTR_PRODUCT_SN(_val) 		sacp::Attribute(16, _val) 		/* 16   ATTR_PRODUCT_SN                          octet  RD */
#define ATTR_SYSTEM_CONTROL_ID 		20
#define ATTR_SYSTEM_CONTROL(_val) 		sacp::Attribute(20, (uint8_t)_val) 		/* 20   ATTR_SYSTEM_CONTROL                      uint8_t RW */
#define ATTR_SACP_RX_BYTES_ID 		70
#define ATTR_SACP_RX_BYTES(_val) 		sacp::Attribute(70, (uint64_t)_val) 		/* 70   ATTR_SACP_RX_BYTES                       uint64_t RD */
#define ATTR_SACP_TX_BYTES_ID 		71
#define ATTR_SACP_TX_BYTES(_val) 		sacp::Attribute(71, (uint64_t)_val) 		/* 71   ATTR_SACP_TX_BYTES                       uint64_t RD */
#define ATTR_SACP_RX_RATE_ID 		72
#define ATTR_SACP_RX_RATE(_val) 		sacp::Attribute(72, (uint32_t)_val) 		/* 72   ATTR_SACP_RX_RATE                        uint32_t RD */
#define ATTR_SACP_TX_RATE_ID 		73
#define ATTR_SACP_TX_RATE(_val) 		sacp::Attribute(73, (uint32_t)_val) 		/* 73   ATTR_SACP_TX_RATE                        uint32_t RD */
#define ATTR_SACP_RX_FRAMES_ID 		74
#define ATTR_SACP_RX_FRAMES(_val) 		sacp::Attribute(74, (uint32_t)_val) 		/* 74   ATTR_SACP_RX_FRAMES                      uint32_t RD */
#define ATTR_SACP_TX_FRAMES_ID 		75
#define ATTR_SACP_TX_FRAMES(_val) 		sacp::Attribute(75, (uint32_t)_val) 		/* 75   ATTR_SACP_TX_FRAMES                      uint32_t RD */
#define ATTR_SACP_CRC_ERRORS_ID 		76
#define ATTR_SACP_CRC_ERRORS(_val) 		sacp::Attribute(76, (uint32_t)_val) 		/* 76   ATTR_SACP_CRC_ERRORS                     uint32_t RD */
#define ATTR_SACP_TX_QUEUED_ID 		77
#define ATTR_SACP_TX_QUEUED(_val) 		sacp::Attribute(77, (uint32_t)_val) 		/* 77   ATTR_SACP_TX_QUEUED                      uint32_t RD */
#define ATTR_SACP_TX_FAILED_ID 		78
#define ATTR_SACP_TX_FAILED(_val) 		sacp::Attribute(78, (uint32_t)_val) 		/* 78   ATTR_SACP_TX_FAILED                      uint32_t RD */
#define ATTR_BOARD_STATE_ID 		120
#define ATTR_BOARD_STATE(_val) 		sacp::Attribute(120, (uint16_t)_val) 		/* 120  ATTR_BOARD_STATE                         uint16_t RD */
#define ATTR_BOARD_TEMPERATURE_ID 		121
#define ATTR_BOARD_TEMPERATURE(_val) 		sacp::Attribute(121, (float)_val) 		/* 121  ATTR_BOARD_TEMPERATURE                   float  RD */
#define ATTR_BOARD_HUMIDITY_ID 		122
#define ATTR_BOARD_HUMIDITY(_val) 		sacp::Attribute(122, (float)_val) 		/* 122  ATTR_BOARD_HUMIDITY                      float  RD */
#define ATTR_MCU_TEMPERATURE_ID 		123
#define ATTR_MCU_TEMPERATURE(_val) 		sacp::Attribute(123, (float)_val) 		/* 123  ATTR_MCU_TEMPERATURE                     float  RD */
#define ATTR_DEPTH_SENSOR_STATE_ID 		124
#define ATTR_DEPTH_SENSOR_STATE(_val) 		sacp::Attribute(124, (uint8_t)_val) 		/* 124  ATTR_DEPTH_SENSOR_STATE                  uint8_t RD */
#define ATTR_DEPTH_SENSOR_PRESSURE_ID 		125
#define ATTR_DEPTH_SENSOR_PRESSURE(_val) 		sacp::Attribute(125, (float)_val) 		/* 125  ATTR_DEPTH_SENSOR_PRESSURE               float  RD */
#define ATTR_DEPTH_SENSOR_TEMPERATURE_ID 		126
#define ATTR_DEPTH_SENSOR_TEMPERATURE(_val) 		sacp::Attribute(126, (float)_val) 		/* 126  ATTR_DEPTH_SENSOR_TEMPERATURE            float  RD */

/* Total: 30 */


/* 属性组 - fogbox */

#define ATTR_FOGBOX_MODEL_ID 		350
#define ATTR_FOGBOX_MODEL(_val) 		sacp::Attribute(350, _val) 		/* 350  ATTR_FOGBOX_MODEL                        octet  RD */
#define ATTR_FOGBOX_SN_ID 		351
#define ATTR_FOGBOX_SN(_val) 		sacp::Attribute(351, _val) 		/* 351  ATTR_FOGBOX_SN                           octet  RD */
#define ATTR_FOGBOX_HW_VERSION_ID 		352
#define ATTR_FOGBOX_HW_VERSION(_val) 		sacp::Attribute(352, (uint16_t)_val) 		/* 352  ATTR_FOGBOX_HW_VERSION                   uint16_t RD */
#define ATTR_FOGBOX_SW_VERSION_ID 		353
#define ATTR_FOGBOX_SW_VERSION(_val) 		sacp::Attribute(353, (uint16_t)_val) 		/* 353  ATTR_FOGBOX_SW_VERSION                   uint16_t RD */
#define ATTR_FOGBOX_ADDRESS_ID 		354
#define ATTR_FOGBOX_ADDRESS(_val) 		sacp::Attribute(354, (uint8_t)_val) 		/* 354  ATTR_FOGBOX_ADDRESS                      uint8_t RD */
#define ATTR_FOGBOX_NAME_ID 		355
#define ATTR_FOGBOX_NAME(_val) 		sacp::Attribute(355, _val) 		/* 355  ATTR_FOGBOX_NAME                         octet  RD */
#define ATTR_FOGBOX_LINK_STATUS_ID 		356
#define ATTR_FOGBOX_LINK_STATUS(_val) 		sacp::Attribute(356, (uint8_t)_val) 		/* 356  ATTR_FOGBOX_LINK_STATUS                  uint8_t RD */
#define ATTR_FOGBOX_CONNECTED_TIME_ID 		357
#define ATTR_FOGBOX_CONNECTED_TIME(_val) 		sacp::Attribute(357, (uint32_t)_val) 		/* 357  ATTR_FOGBOX_CONNECTED_TIME               uint32_t RD */
#define ATTR_FOGBOX_DISCONNECTED_TIME_ID 		358
#define ATTR_FOGBOX_DISCONNECTED_TIME(_val) 		sacp::Attribute(358, (uint32_t)_val) 		/* 358  ATTR_FOGBOX_DISCONNECTED_TIME            uint32_t RD */
#define ATTR_FOGBOX_STATE_ID 		359
#define ATTR_FOGBOX_STATE(_val) 		sacp::Attribute(359, (uint16_t)_val) 		/* 359  ATTR_FOGBOX_STATE                        uint16_t RD */
#define ATTR_FOGBOX_TEMPERATURE_ID 		360
#define ATTR_FOGBOX_TEMPERATURE(_val) 		sacp::Attribute(360, (float)_val) 		/* 360  ATTR_FOGBOX_TEMPERATURE                  float  RD */
#define ATTR_FOGBOX_HUMIDITY_ID 		361
#define ATTR_FOGBOX_HUMIDITY(_val) 		sacp::Attribute(361, (float)_val) 		/* 361  ATTR_FOGBOX_HUMIDITY                     float  RD */

/* Total: 12 */


/* 属性组 - pumpbox */

#define ATTR_PUMPBOX_MODEL_ID 		250
#define ATTR_PUMPBOX_MODEL(_val) 		sacp::Attribute(250, _val) 		/* 250  ATTR_PUMPBOX_MODEL                       octet  RD */
#define ATTR_PUMPBOX_SN_ID 		251
#define ATTR_PUMPBOX_SN(_val) 		sacp::Attribute(251, _val) 		/* 251  ATTR_PUMPBOX_SN                          octet  RD */
#define ATTR_PUMPBOX_HW_VERSION_ID 		252
#define ATTR_PUMPBOX_HW_VERSION(_val) 		sacp::Attribute(252, (uint16_t)_val) 		/* 252  ATTR_PUMPBOX_HW_VERSION                  uint16_t RD */
#define ATTR_PUMPBOX_SW_VERSION_ID 		253
#define ATTR_PUMPBOX_SW_VERSION(_val) 		sacp::Attribute(253, (uint16_t)_val) 		/* 253  ATTR_PUMPBOX_SW_VERSION                  uint16_t RD */
#define ATTR_PUMPBOX_ADDRESS_ID 		254
#define ATTR_PUMPBOX_ADDRESS(_val) 		sacp::Attribute(254, (uint8_t)_val) 		/* 254  ATTR_PUMPBOX_ADDRESS                     uint8_t RD */
#define ATTR_PUMPBOX_NAME_ID 		255
#define ATTR_PUMPBOX_NAME(_val) 		sacp::Attribute(255, _val) 		/* 255  ATTR_PUMPBOX_NAME                        octet  RD */
#define ATTR_PUMPBOX_LINK_STATUS_ID 		256
#define ATTR_PUMPBOX_LINK_STATUS(_val) 		sacp::Attribute(256, (uint8_t)_val) 		/* 256  ATTR_PUMPBOX_LINK_STATUS                 uint8_t RD */
#define ATTR_PUMPBOX_CONNECTED_TIME_ID 		257
#define ATTR_PUMPBOX_CONNECTED_TIME(_val) 		sacp::Attribute(257, (uint32_t)_val) 		/* 257  ATTR_PUMPBOX_CONNECTED_TIME              uint32_t RD */
#define ATTR_PUMPBOX_DISCONNECTED_TIME_ID 		258
#define ATTR_PUMPBOX_DISCONNECTED_TIME(_val) 		sacp::Attribute(258, (uint32_t)_val) 		/* 258  ATTR_PUMPBOX_DISCONNECTED_TIME           uint32_t RD */
#define ATTR_PUMPBOX_STATE_ID 		259
#define ATTR_PUMPBOX_STATE(_val) 		sacp::Attribute(259, (uint16_t)_val) 		/* 259  ATTR_PUMPBOX_STATE                       uint16_t RD */
#define ATTR_PUMPBOX_TEMPERATURE_ID 		260
#define ATTR_PUMPBOX_TEMPERATURE(_val) 		sacp::Attribute(260, (float)_val) 		/* 260  ATTR_PUMPBOX_TEMPERATURE                 float  RD */
#define ATTR_PUMPBOX_HUMIDITY_ID 		261
#define ATTR_PUMPBOX_HUMIDITY(_val) 		sacp::Attribute(261, (float)_val) 		/* 261  ATTR_PUMPBOX_HUMIDITY                    float  RD */
#define ATTR_PUMPBOX_SENSOR_NUM_ID 		262
#define ATTR_PUMPBOX_SENSOR_NUM(_val) 		sacp::Attribute(262, (uint8_t)_val) 		/* 262  ATTR_PUMPBOX_SENSOR_NUM                  uint8_t RD */
#define ATTR_PUMPBOX_SENSOR1_STATUS_ID 		263
#define ATTR_PUMPBOX_SENSOR1_STATUS(_val) 		sacp::Attribute(263, (uint8_t)_val) 		/* 263  ATTR_PUMPBOX_SENSOR1_STATUS              uint8_t RD */
#define ATTR_PUMPBOX_SENSOR1_VALUE_ID 		264
#define ATTR_PUMPBOX_SENSOR1_VALUE(_val) 		sacp::Attribute(264, (float)_val) 		/* 264  ATTR_PUMPBOX_SENSOR1_VALUE               float  RD */
#define ATTR_PUMPBOX_SENSOR2_STATUS_ID 		265
#define ATTR_PUMPBOX_SENSOR2_STATUS(_val) 		sacp::Attribute(265, (uint8_t)_val) 		/* 265  ATTR_PUMPBOX_SENSOR2_STATUS              uint8_t RD */
#define ATTR_PUMPBOX_SENSOR2_VALUE_ID 		266
#define ATTR_PUMPBOX_SENSOR2_VALUE(_val) 		sacp::Attribute(266, (float)_val) 		/* 266  ATTR_PUMPBOX_SENSOR2_VALUE               float  RD */
#define ATTR_PUMPBOX_SENSOR3_STATUS_ID 		267
#define ATTR_PUMPBOX_SENSOR3_STATUS(_val) 		sacp::Attribute(267, (uint8_t)_val) 		/* 267  ATTR_PUMPBOX_SENSOR3_STATUS              uint8_t RD */
#define ATTR_PUMPBOX_SENSOR3_VALUE_ID 		268
#define ATTR_PUMPBOX_SENSOR3_VALUE(_val) 		sacp::Attribute(268, (float)_val) 		/* 268  ATTR_PUMPBOX_SENSOR3_VALUE               float  RD */
#define ATTR_PUMPBOX_SENSOR4_STATUS_ID 		269
#define ATTR_PUMPBOX_SENSOR4_STATUS(_val) 		sacp::Attribute(269, (uint8_t)_val) 		/* 269  ATTR_PUMPBOX_SENSOR4_STATUS              uint8_t RD */
#define ATTR_PUMPBOX_SENSOR4_VALUE_ID 		270
#define ATTR_PUMPBOX_SENSOR4_VALUE(_val) 		sacp::Attribute(270, (float)_val) 		/* 270  ATTR_PUMPBOX_SENSOR4_VALUE               float  RD */
#define ATTR_PUMPBOX_SENSOR5_STATUS_ID 		271
#define ATTR_PUMPBOX_SENSOR5_STATUS(_val) 		sacp::Attribute(271, (uint8_t)_val) 		/* 271  ATTR_PUMPBOX_SENSOR5_STATUS              uint8_t RD */
#define ATTR_PUMPBOX_SENSOR5_VALUE_ID 		272
#define ATTR_PUMPBOX_SENSOR5_VALUE(_val) 		sacp::Attribute(272, (float)_val) 		/* 272  ATTR_PUMPBOX_SENSOR5_VALUE               float  RD */

/* Total: 23 */


/* 属性组 - powerbox */

#define ATTR_POWERBOX_MODEL_ID 		200
#define ATTR_POWERBOX_MODEL(_val) 		sacp::Attribute(200, _val) 		/* 200  ATTR_POWERBOX_MODEL                      octet  RD */
#define ATTR_POWERBOX_SN_ID 		201
#define ATTR_POWERBOX_SN(_val) 		sacp::Attribute(201, _val) 		/* 201  ATTR_POWERBOX_SN                         octet  RD */
#define ATTR_POWERBOX_HW_VERSION_ID 		202
#define ATTR_POWERBOX_HW_VERSION(_val) 		sacp::Attribute(202, (uint16_t)_val) 		/* 202  ATTR_POWERBOX_HW_VERSION                 uint16_t RD */
#define ATTR_POWERBOX_SW_VERSION_ID 		203
#define ATTR_POWERBOX_SW_VERSION(_val) 		sacp::Attribute(203, (uint16_t)_val) 		/* 203  ATTR_POWERBOX_SW_VERSION                 uint16_t RD */
#define ATTR_POWERBOX_ADDRESS_ID 		204
#define ATTR_POWERBOX_ADDRESS(_val) 		sacp::Attribute(204, (uint8_t)_val) 		/* 204  ATTR_POWERBOX_ADDRESS                    uint8_t RD */
#define ATTR_POWERBOX_NAME_ID 		205
#define ATTR_POWERBOX_NAME(_val) 		sacp::Attribute(205, _val) 		/* 205  ATTR_POWERBOX_NAME                       octet  RD */
#define ATTR_POWERBOX_LINK_STATUS_ID 		206
#define ATTR_POWERBOX_LINK_STATUS(_val) 		sacp::Attribute(206, (uint8_t)_val) 		/* 206  ATTR_POWERBOX_LINK_STATUS                uint8_t RD */
#define ATTR_POWERBOX_CONNECTED_TIME_ID 		207
#define ATTR_POWERBOX_CONNECTED_TIME(_val) 		sacp::Attribute(207, (uint32_t)_val) 		/* 207  ATTR_POWERBOX_CONNECTED_TIME             uint32_t RD */
#define ATTR_POWERBOX_DISCONNECTED_TIME_ID 		208
#define ATTR_POWERBOX_DISCONNECTED_TIME(_val) 		sacp::Attribute(208, (uint32_t)_val) 		/* 208  ATTR_POWERBOX_DISCONNECTED_TIME          uint32_t RD */
#define ATTR_POWERBOX_STATE_ID 		209
#define ATTR_POWERBOX_STATE(_val) 		sacp::Attribute(209, (uint16_t)_val) 		/* 209  ATTR_POWERBOX_STATE                      uint16_t RD */
#define ATTR_POWERBOX_TEMPERATURE_ID 		210
#define ATTR_POWERBOX_TEMPERATURE(_val) 		sacp::Attribute(210, (float)_val) 		/* 210  ATTR_POWERBOX_TEMPERATURE                float  RD */
#define ATTR_POWERBOX_HUMIDITY_ID 		211
#define ATTR_POWERBOX_HUMIDITY(_val) 		sacp::Attribute(211, (float)_val) 		/* 211  ATTR_POWERBOX_HUMIDITY                   float  RD */
#define ATTR_POWERBOX_TOTAL_VOLTAGE_ID 		212
#define ATTR_POWERBOX_TOTAL_VOLTAGE(_val) 		sacp::Attribute(212, (float)_val) 		/* 212  ATTR_POWERBOX_TOTAL_VOLTAGE              float  RD */
#define ATTR_POWERBOX_TOTAL_CURRENT_ID 		213
#define ATTR_POWERBOX_TOTAL_CURRENT(_val) 		sacp::Attribute(213, (float)_val) 		/* 213  ATTR_POWERBOX_TOTAL_CURRENT              float  RD */
#define ATTR_POWERBOX_MAIN_IN_VOLTAGE_ID 		215
#define ATTR_POWERBOX_MAIN_IN_VOLTAGE(_val) 		sacp::Attribute(215, (float)_val) 		/* 215  ATTR_POWERBOX_MAIN_IN_VOLTAGE            float  RD */
#define ATTR_POWERBOX_MAIN_IN_CURRENT_ID 		216
#define ATTR_POWERBOX_MAIN_IN_CURRENT(_val) 		sacp::Attribute(216, (float)_val) 		/* 216  ATTR_POWERBOX_MAIN_IN_CURRENT            float  RD */
#define ATTR_POWERBOX_MAIN_OUT_VOLTAGE_ID 		218
#define ATTR_POWERBOX_MAIN_OUT_VOLTAGE(_val) 		sacp::Attribute(218, (float)_val) 		/* 218  ATTR_POWERBOX_MAIN_OUT_VOLTAGE           float  RD */
#define ATTR_POWERBOX_MAIN_OUT_CURRENT_ID 		219
#define ATTR_POWERBOX_MAIN_OUT_CURRENT(_val) 		sacp::Attribute(219, (float)_val) 		/* 219  ATTR_POWERBOX_MAIN_OUT_CURRENT           float  RD */
#define ATTR_POWERBOX_MAIN_TEMPERATURE_ID 		221
#define ATTR_POWERBOX_MAIN_TEMPERATURE(_val) 		sacp::Attribute(221, (float)_val) 		/* 221  ATTR_POWERBOX_MAIN_TEMPERATURE           float  RD */
#define ATTR_POWERBOX_MAIN_STATUS_CODE_ID 		222
#define ATTR_POWERBOX_MAIN_STATUS_CODE(_val) 		sacp::Attribute(222, (uint16_t)_val) 		/* 222  ATTR_POWERBOX_MAIN_STATUS_CODE           uint16_t RD */
#define ATTR_POWERBOX_AUX_IN_VOLTAGE_ID 		223
#define ATTR_POWERBOX_AUX_IN_VOLTAGE(_val) 		sacp::Attribute(223, (float)_val) 		/* 223  ATTR_POWERBOX_AUX_IN_VOLTAGE             float  RD */
#define ATTR_POWERBOX_AUX_IN_CURRENT_ID 		224
#define ATTR_POWERBOX_AUX_IN_CURRENT(_val) 		sacp::Attribute(224, (float)_val) 		/* 224  ATTR_POWERBOX_AUX_IN_CURRENT             float  RD */
#define ATTR_POWERBOX_AUX_OUT_VOLTAGE_ID 		226
#define ATTR_POWERBOX_AUX_OUT_VOLTAGE(_val) 		sacp::Attribute(226, (float)_val) 		/* 226  ATTR_POWERBOX_AUX_OUT_VOLTAGE            float  RD */
#define ATTR_POWERBOX_AUX_OUT_CURRENT_ID 		227
#define ATTR_POWERBOX_AUX_OUT_CURRENT(_val) 		sacp::Attribute(227, (float)_val) 		/* 227  ATTR_POWERBOX_AUX_OUT_CURRENT            float  RD */
#define ATTR_POWERBOX_AUX_TEMPERATURE_ID 		229
#define ATTR_POWERBOX_AUX_TEMPERATURE(_val) 		sacp::Attribute(229, (float)_val) 		/* 229  ATTR_POWERBOX_AUX_TEMPERATURE            float  RD */
#define ATTR_POWERBOX_AUX_STATUS_CODE_ID 		230
#define ATTR_POWERBOX_AUX_STATUS_CODE(_val) 		sacp::Attribute(230, (uint16_t)_val) 		/* 230  ATTR_POWERBOX_AUX_STATUS_CODE            uint16_t RD */

/* Total: 26 */


/* 属性组 - pushbox */

#define ATTR_PUSHBOX_MODEL_ID 		300
#define ATTR_PUSHBOX_MODEL(_val) 		sacp::Attribute(300, _val) 		/* 300  ATTR_PUSHBOX_MODEL                       octet  RD */
#define ATTR_PUSHBOX_SN_ID 		301
#define ATTR_PUSHBOX_SN(_val) 		sacp::Attribute(301, _val) 		/* 301  ATTR_PUSHBOX_SN                          octet  RD */
#define ATTR_PUSHBOX_HW_VERSION_ID 		302
#define ATTR_PUSHBOX_HW_VERSION(_val) 		sacp::Attribute(302, (uint16_t)_val) 		/* 302  ATTR_PUSHBOX_HW_VERSION                  uint16_t RD */
#define ATTR_PUSHBOX_SW_VERSION_ID 		303
#define ATTR_PUSHBOX_SW_VERSION(_val) 		sacp::Attribute(303, (uint16_t)_val) 		/* 303  ATTR_PUSHBOX_SW_VERSION                  uint16_t RD */
#define ATTR_PUSHBOX_ADDRESS_ID 		304
#define ATTR_PUSHBOX_ADDRESS(_val) 		sacp::Attribute(304, (uint8_t)_val) 		/* 304  ATTR_PUSHBOX_ADDRESS                     uint8_t RD */
#define ATTR_PUSHBOX_NAME_ID 		305
#define ATTR_PUSHBOX_NAME(_val) 		sacp::Attribute(305, _val) 		/* 305  ATTR_PUSHBOX_NAME                        octet  RD */
#define ATTR_PUSHBOX_LINK_STATUS_ID 		306
#define ATTR_PUSHBOX_LINK_STATUS(_val) 		sacp::Attribute(306, (uint8_t)_val) 		/* 306  ATTR_PUSHBOX_LINK_STATUS                 uint8_t RD */
#define ATTR_PUSHBOX_CONNECTED_TIME_ID 		307
#define ATTR_PUSHBOX_CONNECTED_TIME(_val) 		sacp::Attribute(307, (uint32_t)_val) 		/* 307  ATTR_PUSHBOX_CONNECTED_TIME              uint32_t RD */
#define ATTR_PUSHBOX_DISCONNECTED_TIME_ID 		308
#define ATTR_PUSHBOX_DISCONNECTED_TIME(_val) 		sacp::Attribute(308, (uint32_t)_val) 		/* 308  ATTR_PUSHBOX_DISCONNECTED_TIME           uint32_t RD */
#define ATTR_PUSHBOX_STATE_ID 		309
#define ATTR_PUSHBOX_STATE(_val) 		sacp::Attribute(309, (uint16_t)_val) 		/* 309  ATTR_PUSHBOX_STATE                       uint16_t RD */
#define ATTR_PUSHBOX_PUSH_STATE_ID 		310
#define ATTR_PUSHBOX_PUSH_STATE(_val) 		sacp::Attribute(310, (uint8_t)_val) 		/* 310  ATTR_PUSHBOX_PUSH_STATE                  uint8_t RD */
#define ATTR_PUSHBOX_TEMPERATURE_ID 		311
#define ATTR_PUSHBOX_TEMPERATURE(_val) 		sacp::Attribute(311, (float)_val) 		/* 311  ATTR_PUSHBOX_TEMPERATURE                 float  RD */
#define ATTR_PUSHBOX_HUMIDITY_ID 		312
#define ATTR_PUSHBOX_HUMIDITY(_val) 		sacp::Attribute(312, (float)_val) 		/* 312  ATTR_PUSHBOX_HUMIDITY                    float  RD */
#define ATTR_PUSHBOX_VOLTAGE_ID 		313
#define ATTR_PUSHBOX_VOLTAGE(_val) 		sacp::Attribute(313, (float)_val) 		/* 313  ATTR_PUSHBOX_VOLTAGE                     float  RD */
#define ATTR_PUSHBOX_CURRENT_ID 		314
#define ATTR_PUSHBOX_CURRENT(_val) 		sacp::Attribute(314, (float)_val) 		/* 314  ATTR_PUSHBOX_CURRENT                     float  RD */
#define ATTR_PUSHBOX_BATTERY_VOLTAGE_ID 		315
#define ATTR_PUSHBOX_BATTERY_VOLTAGE(_val) 		sacp::Attribute(315, (float)_val) 		/* 315  ATTR_PUSHBOX_BATTERY_VOLTAGE             float  RD */
#define ATTR_PUSHBOX_BATTERY_CURRENT_ID 		316
#define ATTR_PUSHBOX_BATTERY_CURRENT(_val) 		sacp::Attribute(316, (float)_val) 		/* 316  ATTR_PUSHBOX_BATTERY_CURRENT             float  RD */
#define ATTR_PUSHBOX_BATTERY_LEVEL_ID 		317
#define ATTR_PUSHBOX_BATTERY_LEVEL(_val) 		sacp::Attribute(317, (uint8_t)_val) 		/* 317  ATTR_PUSHBOX_BATTERY_LEVEL               uint8_t RD */
#define ATTR_PUSHBOX_BATTERY_TEMPERATURE_ID 		318
#define ATTR_PUSHBOX_BATTERY_TEMPERATURE(_val) 		sacp::Attribute(318, (float)_val) 		/* 318  ATTR_PUSHBOX_BATTERY_TEMPERATURE         float  RD */
#define ATTR_PUSHBOX_BATTERY_INCHARGING_ID 		319
#define ATTR_PUSHBOX_BATTERY_INCHARGING(_val) 		sacp::Attribute(319, (bool)_val) 		/* 319  ATTR_PUSHBOX_BATTERY_INCHARGING          bool   RD */
#define ATTR_PUSHBOX_BATTERY_VOLTAGE_STATUS_ID 		320
#define ATTR_PUSHBOX_BATTERY_VOLTAGE_STATUS(_val) 		sacp::Attribute(320, (uint16_t)_val) 		/* 320  ATTR_PUSHBOX_BATTERY_VOLTAGE_STATUS      uint16_t RD */
#define ATTR_PUSHBOX_BATTERY_CURRENT_STATUS_ID 		321
#define ATTR_PUSHBOX_BATTERY_CURRENT_STATUS(_val) 		sacp::Attribute(321, (uint16_t)_val) 		/* 321  ATTR_PUSHBOX_BATTERY_CURRENT_STATUS      uint16_t RD */
#define ATTR_PUSHBOX_OFFLINE_PUSH_ENABLE_ID 		322
#define ATTR_PUSHBOX_OFFLINE_PUSH_ENABLE(_val) 		sacp::Attribute(322, (bool)_val) 		/* 322  ATTR_PUSHBOX_OFFLINE_PUSH_ENABLE         bool   RW */
#define ATTR_PUSHBOX_OFFLINE_PUSH_DELAY_TIME_ID 		323
#define ATTR_PUSHBOX_OFFLINE_PUSH_DELAY_TIME(_val) 		sacp::Attribute(323, (uint16_t)_val) 		/* 323  ATTR_PUSHBOX_OFFLINE_PUSH_DELAY_TIME     uint16_t RW */
#define ATTR_PUSHBOX_CONTROL_ID 		324
#define ATTR_PUSHBOX_CONTROL(_val) 		sacp::Attribute(324, (uint8_t)_val) 		/* 324  ATTR_PUSHBOX_CONTROL                     uint8_t RW */

/* Total: 25 */


/* 属性组 - ledlight */

#define ATTR_LEDLIGHT_A_MODEL_ID 		400
#define ATTR_LEDLIGHT_A_MODEL(_val) 		sacp::Attribute(400, _val) 		/* 400  ATTR_LEDLIGHT_A_MODEL                    octet  RD */
#define ATTR_LEDLIGHT_A_SN_ID 		401
#define ATTR_LEDLIGHT_A_SN(_val) 		sacp::Attribute(401, _val) 		/* 401  ATTR_LEDLIGHT_A_SN                       octet  RD */
#define ATTR_LEDLIGHT_A_HW_VERSION_ID 		402
#define ATTR_LEDLIGHT_A_HW_VERSION(_val) 		sacp::Attribute(402, (uint16_t)_val) 		/* 402  ATTR_LEDLIGHT_A_HW_VERSION               uint16_t RD */
#define ATTR_LEDLIGHT_A_SW_VERSION_ID 		403
#define ATTR_LEDLIGHT_A_SW_VERSION(_val) 		sacp::Attribute(403, (uint16_t)_val) 		/* 403  ATTR_LEDLIGHT_A_SW_VERSION               uint16_t RD */
#define ATTR_LEDLIGHT_A_ADDRESS_ID 		404
#define ATTR_LEDLIGHT_A_ADDRESS(_val) 		sacp::Attribute(404, (uint8_t)_val) 		/* 404  ATTR_LEDLIGHT_A_ADDRESS                  uint8_t RD */
#define ATTR_LEDLIGHT_A_NAME_ID 		405
#define ATTR_LEDLIGHT_A_NAME(_val) 		sacp::Attribute(405, _val) 		/* 405  ATTR_LEDLIGHT_A_NAME                     octet  RD */
#define ATTR_LEDLIGHT_A_LINK_STATUS_ID 		406
#define ATTR_LEDLIGHT_A_LINK_STATUS(_val) 		sacp::Attribute(406, (uint8_t)_val) 		/* 406  ATTR_LEDLIGHT_A_LINK_STATUS              uint8_t RD */
#define ATTR_LEDLIGHT_A_CONNECTED_TIME_ID 		407
#define ATTR_LEDLIGHT_A_CONNECTED_TIME(_val) 		sacp::Attribute(407, (uint32_t)_val) 		/* 407  ATTR_LEDLIGHT_A_CONNECTED_TIME           uint32_t RD */
#define ATTR_LEDLIGHT_A_DISCONNECTED_TIME_ID 		408
#define ATTR_LEDLIGHT_A_DISCONNECTED_TIME(_val) 		sacp::Attribute(408, (uint32_t)_val) 		/* 408  ATTR_LEDLIGHT_A_DISCONNECTED_TIME        uint32_t RD */
#define ATTR_LEDLIGHT_A_STATE_ID 		409
#define ATTR_LEDLIGHT_A_STATE(_val) 		sacp::Attribute(409, (uint16_t)_val) 		/* 409  ATTR_LEDLIGHT_A_STATE                    uint16_t RD */
#define ATTR_LEDLIGHT_A_BRIGHTNESS_ID 		410
#define ATTR_LEDLIGHT_A_BRIGHTNESS(_val) 		sacp::Attribute(410, (uint8_t)_val) 		/* 410  ATTR_LEDLIGHT_A_BRIGHTNESS               uint8_t RD */
#define ATTR_LEDLIGHT_A_TEMPERATURE_ID 		411
#define ATTR_LEDLIGHT_A_TEMPERATURE(_val) 		sacp::Attribute(411, (float)_val) 		/* 411  ATTR_LEDLIGHT_A_TEMPERATURE              float  RD */
#define ATTR_LEDLIGHT_A_HUMIDITY_ID 		412
#define ATTR_LEDLIGHT_A_HUMIDITY(_val) 		sacp::Attribute(412, (float)_val) 		/* 412  ATTR_LEDLIGHT_A_HUMIDITY                 float  RD */
#define ATTR_LEDLIGHT_A_LED_TEMPERATURE_ID 		413
#define ATTR_LEDLIGHT_A_LED_TEMPERATURE(_val) 		sacp::Attribute(413, (float)_val) 		/* 413  ATTR_LEDLIGHT_A_LED_TEMPERATURE          float  RD */
#define ATTR_LEDLIGHT_A_INPUT_VOLTAGE_ID 		414
#define ATTR_LEDLIGHT_A_INPUT_VOLTAGE(_val) 		sacp::Attribute(414, (float)_val) 		/* 414  ATTR_LEDLIGHT_A_INPUT_VOLTAGE            float  RD */
#define ATTR_LEDLIGHT_A_VOLTAGE_ID 		415
#define ATTR_LEDLIGHT_A_VOLTAGE(_val) 		sacp::Attribute(415, (float)_val) 		/* 415  ATTR_LEDLIGHT_A_VOLTAGE                  float  RD */
#define ATTR_LEDLIGHT_A_CURRENT_ID 		416
#define ATTR_LEDLIGHT_A_CURRENT(_val) 		sacp::Attribute(416, (float)_val) 		/* 416  ATTR_LEDLIGHT_A_CURRENT                  float  RD */
#define ATTR_LEDLIGHT_A_POWER_ID 		417
#define ATTR_LEDLIGHT_A_POWER(_val) 		sacp::Attribute(417, (float)_val) 		/* 417  ATTR_LEDLIGHT_A_POWER                    float  RD */
#define ATTR_LEDLIGHT_A_SET_BRIGHTNESS_ID 		418
#define ATTR_LEDLIGHT_A_SET_BRIGHTNESS(_val) 		sacp::Attribute(418, (uint8_t)_val) 		/* 418  ATTR_LEDLIGHT_A_SET_BRIGHTNESS           uint8_t RW */
#define ATTR_LEDLIGHT_B_MODEL_ID 		425
#define ATTR_LEDLIGHT_B_MODEL(_val) 		sacp::Attribute(425, _val) 		/* 425  ATTR_LEDLIGHT_B_MODEL                    octet  RD */
#define ATTR_LEDLIGHT_B_SN_ID 		426
#define ATTR_LEDLIGHT_B_SN(_val) 		sacp::Attribute(426, _val) 		/* 426  ATTR_LEDLIGHT_B_SN                       octet  RD */
#define ATTR_LEDLIGHT_B_HW_VERSION_ID 		427
#define ATTR_LEDLIGHT_B_HW_VERSION(_val) 		sacp::Attribute(427, (uint16_t)_val) 		/* 427  ATTR_LEDLIGHT_B_HW_VERSION               uint16_t RD */
#define ATTR_LEDLIGHT_B_SW_VERSION_ID 		428
#define ATTR_LEDLIGHT_B_SW_VERSION(_val) 		sacp::Attribute(428, (uint16_t)_val) 		/* 428  ATTR_LEDLIGHT_B_SW_VERSION               uint16_t RD */
#define ATTR_LEDLIGHT_B_ADDRESS_ID 		429
#define ATTR_LEDLIGHT_B_ADDRESS(_val) 		sacp::Attribute(429, (uint8_t)_val) 		/* 429  ATTR_LEDLIGHT_B_ADDRESS                  uint8_t RD */
#define ATTR_LEDLIGHT_B_NAME_ID 		430
#define ATTR_LEDLIGHT_B_NAME(_val) 		sacp::Attribute(430, _val) 		/* 430  ATTR_LEDLIGHT_B_NAME                     octet  RD */
#define ATTR_LEDLIGHT_B_LINK_STATUS_ID 		431
#define ATTR_LEDLIGHT_B_LINK_STATUS(_val) 		sacp::Attribute(431, (uint8_t)_val) 		/* 431  ATTR_LEDLIGHT_B_LINK_STATUS              uint8_t RD */
#define ATTR_LEDLIGHT_B_CONNECTED_TIME_ID 		432
#define ATTR_LEDLIGHT_B_CONNECTED_TIME(_val) 		sacp::Attribute(432, (uint32_t)_val) 		/* 432  ATTR_LEDLIGHT_B_CONNECTED_TIME           uint32_t RD */
#define ATTR_LEDLIGHT_B_DISCONNECTED_TIME_ID 		433
#define ATTR_LEDLIGHT_B_DISCONNECTED_TIME(_val) 		sacp::Attribute(433, (uint32_t)_val) 		/* 433  ATTR_LEDLIGHT_B_DISCONNECTED_TIME        uint32_t RD */
#define ATTR_LEDLIGHT_B_STATE_ID 		434
#define ATTR_LEDLIGHT_B_STATE(_val) 		sacp::Attribute(434, (uint16_t)_val) 		/* 434  ATTR_LEDLIGHT_B_STATE                    uint16_t RD */
#define ATTR_LEDLIGHT_B_BRIGHTNESS_ID 		435
#define ATTR_LEDLIGHT_B_BRIGHTNESS(_val) 		sacp::Attribute(435, (uint8_t)_val) 		/* 435  ATTR_LEDLIGHT_B_BRIGHTNESS               uint8_t RD */
#define ATTR_LEDLIGHT_B_TEMPERATURE_ID 		436
#define ATTR_LEDLIGHT_B_TEMPERATURE(_val) 		sacp::Attribute(436, (float)_val) 		/* 436  ATTR_LEDLIGHT_B_TEMPERATURE              float  RD */
#define ATTR_LEDLIGHT_B_HUMIDITY_ID 		437
#define ATTR_LEDLIGHT_B_HUMIDITY(_val) 		sacp::Attribute(437, (float)_val) 		/* 437  ATTR_LEDLIGHT_B_HUMIDITY                 float  RD */
#define ATTR_LEDLIGHT_B_LED_TEMPERATURE_ID 		438
#define ATTR_LEDLIGHT_B_LED_TEMPERATURE(_val) 		sacp::Attribute(438, (float)_val) 		/* 438  ATTR_LEDLIGHT_B_LED_TEMPERATURE          float  RD */
#define ATTR_LEDLIGHT_B_INPUT_VOLTAGE_ID 		439
#define ATTR_LEDLIGHT_B_INPUT_VOLTAGE(_val) 		sacp::Attribute(439, (float)_val) 		/* 439  ATTR_LEDLIGHT_B_INPUT_VOLTAGE            float  RD */
#define ATTR_LEDLIGHT_B_VOLTAGE_ID 		440
#define ATTR_LEDLIGHT_B_VOLTAGE(_val) 		sacp::Attribute(440, (float)_val) 		/* 440  ATTR_LEDLIGHT_B_VOLTAGE                  float  RD */
#define ATTR_LEDLIGHT_B_CURRENT_ID 		441
#define ATTR_LEDLIGHT_B_CURRENT(_val) 		sacp::Attribute(441, (float)_val) 		/* 441  ATTR_LEDLIGHT_B_CURRENT                  float  RD */
#define ATTR_LEDLIGHT_B_POWER_ID 		442
#define ATTR_LEDLIGHT_B_POWER(_val) 		sacp::Attribute(442, (float)_val) 		/* 442  ATTR_LEDLIGHT_B_POWER                    float  RD */
#define ATTR_LEDLIGHT_B_SET_BRIGHTNESS_ID 		443
#define ATTR_LEDLIGHT_B_SET_BRIGHTNESS(_val) 		sacp::Attribute(443, (uint8_t)_val) 		/* 443  ATTR_LEDLIGHT_B_SET_BRIGHTNESS           uint8_t RW */
#define ATTR_LEDLIGHT_C_MODEL_ID 		450
#define ATTR_LEDLIGHT_C_MODEL(_val) 		sacp::Attribute(450, _val) 		/* 450  ATTR_LEDLIGHT_C_MODEL                    octet  RD */
#define ATTR_LEDLIGHT_C_SN_ID 		451
#define ATTR_LEDLIGHT_C_SN(_val) 		sacp::Attribute(451, _val) 		/* 451  ATTR_LEDLIGHT_C_SN                       octet  RD */
#define ATTR_LEDLIGHT_C_HW_VERSION_ID 		452
#define ATTR_LEDLIGHT_C_HW_VERSION(_val) 		sacp::Attribute(452, (uint16_t)_val) 		/* 452  ATTR_LEDLIGHT_C_HW_VERSION               uint16_t RD */
#define ATTR_LEDLIGHT_C_SW_VERSION_ID 		453
#define ATTR_LEDLIGHT_C_SW_VERSION(_val) 		sacp::Attribute(453, (uint16_t)_val) 		/* 453  ATTR_LEDLIGHT_C_SW_VERSION               uint16_t RD */
#define ATTR_LEDLIGHT_C_ADDRESS_ID 		454
#define ATTR_LEDLIGHT_C_ADDRESS(_val) 		sacp::Attribute(454, (uint8_t)_val) 		/* 454  ATTR_LEDLIGHT_C_ADDRESS                  uint8_t RD */
#define ATTR_LEDLIGHT_C_NAME_ID 		455
#define ATTR_LEDLIGHT_C_NAME(_val) 		sacp::Attribute(455, _val) 		/* 455  ATTR_LEDLIGHT_C_NAME                     octet  RD */
#define ATTR_LEDLIGHT_C_LINK_STATUS_ID 		456
#define ATTR_LEDLIGHT_C_LINK_STATUS(_val) 		sacp::Attribute(456, (uint8_t)_val) 		/* 456  ATTR_LEDLIGHT_C_LINK_STATUS              uint8_t RD */
#define ATTR_LEDLIGHT_C_CONNECTED_TIME_ID 		457
#define ATTR_LEDLIGHT_C_CONNECTED_TIME(_val) 		sacp::Attribute(457, (uint32_t)_val) 		/* 457  ATTR_LEDLIGHT_C_CONNECTED_TIME           uint32_t RD */
#define ATTR_LEDLIGHT_C_DISCONNECTED_TIME_ID 		458
#define ATTR_LEDLIGHT_C_DISCONNECTED_TIME(_val) 		sacp::Attribute(458, (uint32_t)_val) 		/* 458  ATTR_LEDLIGHT_C_DISCONNECTED_TIME        uint32_t RD */
#define ATTR_LEDLIGHT_C_STATE_ID 		459
#define ATTR_LEDLIGHT_C_STATE(_val) 		sacp::Attribute(459, (uint16_t)_val) 		/* 459  ATTR_LEDLIGHT_C_STATE                    uint16_t RD */
#define ATTR_LEDLIGHT_C_BRIGHTNESS_ID 		460
#define ATTR_LEDLIGHT_C_BRIGHTNESS(_val) 		sacp::Attribute(460, (uint8_t)_val) 		/* 460  ATTR_LEDLIGHT_C_BRIGHTNESS               uint8_t RD */
#define ATTR_LEDLIGHT_C_TEMPERATURE_ID 		461
#define ATTR_LEDLIGHT_C_TEMPERATURE(_val) 		sacp::Attribute(461, (float)_val) 		/* 461  ATTR_LEDLIGHT_C_TEMPERATURE              float  RD */
#define ATTR_LEDLIGHT_C_HUMIDITY_ID 		462
#define ATTR_LEDLIGHT_C_HUMIDITY(_val) 		sacp::Attribute(462, (float)_val) 		/* 462  ATTR_LEDLIGHT_C_HUMIDITY                 float  RD */
#define ATTR_LEDLIGHT_C_LED_TEMPERATURE_ID 		463
#define ATTR_LEDLIGHT_C_LED_TEMPERATURE(_val) 		sacp::Attribute(463, (float)_val) 		/* 463  ATTR_LEDLIGHT_C_LED_TEMPERATURE          float  RD */
#define ATTR_LEDLIGHT_C_INPUT_VOLTAGE_ID 		464
#define ATTR_LEDLIGHT_C_INPUT_VOLTAGE(_val) 		sacp::Attribute(464, (float)_val) 		/* 464  ATTR_LEDLIGHT_C_INPUT_VOLTAGE            float  RD */
#define ATTR_LEDLIGHT_C_VOLTAGE_ID 		465
#define ATTR_LEDLIGHT_C_VOLTAGE(_val) 		sacp::Attribute(465, (float)_val) 		/* 465  ATTR_LEDLIGHT_C_VOLTAGE                  float  RD */
#define ATTR_LEDLIGHT_C_CURRENT_ID 		466
#define ATTR_LEDLIGHT_C_CURRENT(_val) 		sacp::Attribute(466, (float)_val) 		/* 466  ATTR_LEDLIGHT_C_CURRENT                  float  RD */
#define ATTR_LEDLIGHT_C_POWER_ID 		467
#define ATTR_LEDLIGHT_C_POWER(_val) 		sacp::Attribute(467, (float)_val) 		/* 467  ATTR_LEDLIGHT_C_POWER                    float  RD */
#define ATTR_LEDLIGHT_C_SET_BRIGHTNESS_ID 		468
#define ATTR_LEDLIGHT_C_SET_BRIGHTNESS(_val) 		sacp::Attribute(468, (uint8_t)_val) 		/* 468  ATTR_LEDLIGHT_C_SET_BRIGHTNESS           uint8_t RW */
#define ATTR_LEDLIGHT_D_MODEL_ID 		475
#define ATTR_LEDLIGHT_D_MODEL(_val) 		sacp::Attribute(475, _val) 		/* 475  ATTR_LEDLIGHT_D_MODEL                    octet  RD */
#define ATTR_LEDLIGHT_D_SN_ID 		476
#define ATTR_LEDLIGHT_D_SN(_val) 		sacp::Attribute(476, _val) 		/* 476  ATTR_LEDLIGHT_D_SN                       octet  RD */
#define ATTR_LEDLIGHT_D_HW_VERSION_ID 		477
#define ATTR_LEDLIGHT_D_HW_VERSION(_val) 		sacp::Attribute(477, (uint16_t)_val) 		/* 477  ATTR_LEDLIGHT_D_HW_VERSION               uint16_t RD */
#define ATTR_LEDLIGHT_D_SW_VERSION_ID 		478
#define ATTR_LEDLIGHT_D_SW_VERSION(_val) 		sacp::Attribute(478, (uint16_t)_val) 		/* 478  ATTR_LEDLIGHT_D_SW_VERSION               uint16_t RD */
#define ATTR_LEDLIGHT_D_ADDRESS_ID 		479
#define ATTR_LEDLIGHT_D_ADDRESS(_val) 		sacp::Attribute(479, (uint8_t)_val) 		/* 479  ATTR_LEDLIGHT_D_ADDRESS                  uint8_t RD */
#define ATTR_LEDLIGHT_D_NAME_ID 		480
#define ATTR_LEDLIGHT_D_NAME(_val) 		sacp::Attribute(480, _val) 		/* 480  ATTR_LEDLIGHT_D_NAME                     octet  RD */
#define ATTR_LEDLIGHT_D_LINK_STATUS_ID 		481
#define ATTR_LEDLIGHT_D_LINK_STATUS(_val) 		sacp::Attribute(481, (uint8_t)_val) 		/* 481  ATTR_LEDLIGHT_D_LINK_STATUS              uint8_t RD */
#define ATTR_LEDLIGHT_D_CONNECTED_TIME_ID 		482
#define ATTR_LEDLIGHT_D_CONNECTED_TIME(_val) 		sacp::Attribute(482, (uint32_t)_val) 		/* 482  ATTR_LEDLIGHT_D_CONNECTED_TIME           uint32_t RD */
#define ATTR_LEDLIGHT_D_DISCONNECTED_TIME_ID 		483
#define ATTR_LEDLIGHT_D_DISCONNECTED_TIME(_val) 		sacp::Attribute(483, (uint32_t)_val) 		/* 483  ATTR_LEDLIGHT_D_DISCONNECTED_TIME        uint32_t RD */
#define ATTR_LEDLIGHT_D_STATE_ID 		484
#define ATTR_LEDLIGHT_D_STATE(_val) 		sacp::Attribute(484, (uint16_t)_val) 		/* 484  ATTR_LEDLIGHT_D_STATE                    uint16_t RD */
#define ATTR_LEDLIGHT_D_BRIGHTNESS_ID 		485
#define ATTR_LEDLIGHT_D_BRIGHTNESS(_val) 		sacp::Attribute(485, (uint8_t)_val) 		/* 485  ATTR_LEDLIGHT_D_BRIGHTNESS               uint8_t RD */
#define ATTR_LEDLIGHT_D_TEMPERATURE_ID 		486
#define ATTR_LEDLIGHT_D_TEMPERATURE(_val) 		sacp::Attribute(486, (float)_val) 		/* 486  ATTR_LEDLIGHT_D_TEMPERATURE              float  RD */
#define ATTR_LEDLIGHT_D_HUMIDITY_ID 		487
#define ATTR_LEDLIGHT_D_HUMIDITY(_val) 		sacp::Attribute(487, (float)_val) 		/* 487  ATTR_LEDLIGHT_D_HUMIDITY                 float  RD */
#define ATTR_LEDLIGHT_D_LED_TEMPERATURE_ID 		488
#define ATTR_LEDLIGHT_D_LED_TEMPERATURE(_val) 		sacp::Attribute(488, (float)_val) 		/* 488  ATTR_LEDLIGHT_D_LED_TEMPERATURE          float  RD */
#define ATTR_LEDLIGHT_D_INPUT_VOLTAGE_ID 		489
#define ATTR_LEDLIGHT_D_INPUT_VOLTAGE(_val) 		sacp::Attribute(489, (float)_val) 		/* 489  ATTR_LEDLIGHT_D_INPUT_VOLTAGE            float  RD */
#define ATTR_LEDLIGHT_D_VOLTAGE_ID 		490
#define ATTR_LEDLIGHT_D_VOLTAGE(_val) 		sacp::Attribute(490, (float)_val) 		/* 490  ATTR_LEDLIGHT_D_VOLTAGE                  float  RD */
#define ATTR_LEDLIGHT_D_CURRENT_ID 		491
#define ATTR_LEDLIGHT_D_CURRENT(_val) 		sacp::Attribute(491, (float)_val) 		/* 491  ATTR_LEDLIGHT_D_CURRENT                  float  RD */
#define ATTR_LEDLIGHT_D_POWER_ID 		492
#define ATTR_LEDLIGHT_D_POWER(_val) 		sacp::Attribute(492, (float)_val) 		/* 492  ATTR_LEDLIGHT_D_POWER                    float  RD */
#define ATTR_LEDLIGHT_D_SET_BRIGHTNESS_ID 		493
#define ATTR_LEDLIGHT_D_SET_BRIGHTNESS(_val) 		sacp::Attribute(493, (uint8_t)_val) 		/* 493  ATTR_LEDLIGHT_D_SET_BRIGHTNESS           uint8_t RW */

/* Total: 76 */


/* 属性组 - lifter */

#define ATTR_LIFTER_A_MODEL_ID 		600
#define ATTR_LIFTER_A_MODEL(_val) 		sacp::Attribute(600, _val) 		/* 600  ATTR_LIFTER_A_MODEL                      octet  RD */
#define ATTR_LIFTER_A_SN_ID 		601
#define ATTR_LIFTER_A_SN(_val) 		sacp::Attribute(601, _val) 		/* 601  ATTR_LIFTER_A_SN                         octet  RD */
#define ATTR_LIFTER_A_HW_VERSION_ID 		602
#define ATTR_LIFTER_A_HW_VERSION(_val) 		sacp::Attribute(602, (uint16_t)_val) 		/* 602  ATTR_LIFTER_A_HW_VERSION                 uint16_t RD */
#define ATTR_LIFTER_A_SW_VERSION_ID 		603
#define ATTR_LIFTER_A_SW_VERSION(_val) 		sacp::Attribute(603, (uint16_t)_val) 		/* 603  ATTR_LIFTER_A_SW_VERSION                 uint16_t RD */
#define ATTR_LIFTER_A_ADDRESS_ID 		604
#define ATTR_LIFTER_A_ADDRESS(_val) 		sacp::Attribute(604, (uint8_t)_val) 		/* 604  ATTR_LIFTER_A_ADDRESS                    uint8_t RD */
#define ATTR_LIFTER_A_NAME_ID 		605
#define ATTR_LIFTER_A_NAME(_val) 		sacp::Attribute(605, _val) 		/* 605  ATTR_LIFTER_A_NAME                       octet  RD */
#define ATTR_LIFTER_A_LINK_STATUS_ID 		606
#define ATTR_LIFTER_A_LINK_STATUS(_val) 		sacp::Attribute(606, (uint8_t)_val) 		/* 606  ATTR_LIFTER_A_LINK_STATUS                uint8_t RD */
#define ATTR_LIFTER_A_CONNECTED_TIME_ID 		607
#define ATTR_LIFTER_A_CONNECTED_TIME(_val) 		sacp::Attribute(607, (uint32_t)_val) 		/* 607  ATTR_LIFTER_A_CONNECTED_TIME             uint32_t RD */
#define ATTR_LIFTER_A_DISCONNECTED_TIME_ID 		608
#define ATTR_LIFTER_A_DISCONNECTED_TIME(_val) 		sacp::Attribute(608, (uint32_t)_val) 		/* 608  ATTR_LIFTER_A_DISCONNECTED_TIME          uint32_t RD */
#define ATTR_LIFTER_A_POSITION_ID 		609
#define ATTR_LIFTER_A_POSITION(_val) 		sacp::Attribute(609, (uint8_t)_val) 		/* 609  ATTR_LIFTER_A_POSITION                   uint8_t RD */
#define ATTR_LIFTER_A_RAW_POSITION_ID 		610
#define ATTR_LIFTER_A_RAW_POSITION(_val) 		sacp::Attribute(610, (uint16_t)_val) 		/* 610  ATTR_LIFTER_A_RAW_POSITION               uint16_t RD */
#define ATTR_LIFTER_A_TEMPERATURE_ID 		611
#define ATTR_LIFTER_A_TEMPERATURE(_val) 		sacp::Attribute(611, (float)_val) 		/* 611  ATTR_LIFTER_A_TEMPERATURE                float  RD */
#define ATTR_LIFTER_A_LOAD_ID 		614
#define ATTR_LIFTER_A_LOAD(_val) 		sacp::Attribute(614, (uint8_t)_val) 		/* 614  ATTR_LIFTER_A_LOAD                       uint8_t RD */
#define ATTR_LIFTER_A_VOLTAGE_ID 		615
#define ATTR_LIFTER_A_VOLTAGE(_val) 		sacp::Attribute(615, (float)_val) 		/* 615  ATTR_LIFTER_A_VOLTAGE                    float  RD */
#define ATTR_LIFTER_A_CURRRENT_ID 		616
#define ATTR_LIFTER_A_CURRRENT(_val) 		sacp::Attribute(616, (float)_val) 		/* 616  ATTR_LIFTER_A_CURRRENT                   float  RD */
#define ATTR_LIFTER_A_RUNNING_ID 		617
#define ATTR_LIFTER_A_RUNNING(_val) 		sacp::Attribute(617, (bool)_val) 		/* 617  ATTR_LIFTER_A_RUNNING                    bool   RD */
#define ATTR_LIFTER_A_ALARM_STATUS_ID 		618
#define ATTR_LIFTER_A_ALARM_STATUS(_val) 		sacp::Attribute(618, (uint16_t)_val) 		/* 618  ATTR_LIFTER_A_ALARM_STATUS               uint16_t RD */
#define ATTR_LIFTER_A_SET_POSITION_ID 		619
#define ATTR_LIFTER_A_SET_POSITION(_val) 		sacp::Attribute(619, (uint8_t)_val) 		/* 619  ATTR_LIFTER_A_SET_POSITION               uint8_t RW */
#define ATTR_LIFTER_B_MODEL_ID 		625
#define ATTR_LIFTER_B_MODEL(_val) 		sacp::Attribute(625, _val) 		/* 625  ATTR_LIFTER_B_MODEL                      octet  RD */
#define ATTR_LIFTER_B_SN_ID 		626
#define ATTR_LIFTER_B_SN(_val) 		sacp::Attribute(626, _val) 		/* 626  ATTR_LIFTER_B_SN                         octet  RD */
#define ATTR_LIFTER_B_HW_VERSION_ID 		627
#define ATTR_LIFTER_B_HW_VERSION(_val) 		sacp::Attribute(627, (uint16_t)_val) 		/* 627  ATTR_LIFTER_B_HW_VERSION                 uint16_t RD */
#define ATTR_LIFTER_B_SW_VERSION_ID 		628
#define ATTR_LIFTER_B_SW_VERSION(_val) 		sacp::Attribute(628, (uint16_t)_val) 		/* 628  ATTR_LIFTER_B_SW_VERSION                 uint16_t RD */
#define ATTR_LIFTER_B_ADDRESS_ID 		629
#define ATTR_LIFTER_B_ADDRESS(_val) 		sacp::Attribute(629, (uint8_t)_val) 		/* 629  ATTR_LIFTER_B_ADDRESS                    uint8_t RD */
#define ATTR_LIFTER_B_NAME_ID 		630
#define ATTR_LIFTER_B_NAME(_val) 		sacp::Attribute(630, _val) 		/* 630  ATTR_LIFTER_B_NAME                       octet  RD */
#define ATTR_LIFTER_B_LINK_STATUS_ID 		631
#define ATTR_LIFTER_B_LINK_STATUS(_val) 		sacp::Attribute(631, (uint8_t)_val) 		/* 631  ATTR_LIFTER_B_LINK_STATUS                uint8_t RD */
#define ATTR_LIFTER_B_CONNECTED_TIME_ID 		632
#define ATTR_LIFTER_B_CONNECTED_TIME(_val) 		sacp::Attribute(632, (uint32_t)_val) 		/* 632  ATTR_LIFTER_B_CONNECTED_TIME             uint32_t RD */
#define ATTR_LIFTER_B_DISCONNECTED_TIME_ID 		633
#define ATTR_LIFTER_B_DISCONNECTED_TIME(_val) 		sacp::Attribute(633, (uint32_t)_val) 		/* 633  ATTR_LIFTER_B_DISCONNECTED_TIME          uint32_t RD */
#define ATTR_LIFTER_B_POSITION_ID 		634
#define ATTR_LIFTER_B_POSITION(_val) 		sacp::Attribute(634, (uint8_t)_val) 		/* 634  ATTR_LIFTER_B_POSITION                   uint8_t RD */
#define ATTR_LIFTER_B_RAW_POSITION_ID 		635
#define ATTR_LIFTER_B_RAW_POSITION(_val) 		sacp::Attribute(635, (uint16_t)_val) 		/* 635  ATTR_LIFTER_B_RAW_POSITION               uint16_t RD */
#define ATTR_LIFTER_B_TEMPERATURE_ID 		636
#define ATTR_LIFTER_B_TEMPERATURE(_val) 		sacp::Attribute(636, (float)_val) 		/* 636  ATTR_LIFTER_B_TEMPERATURE                float  RD */
#define ATTR_LIFTER_B_LOAD_ID 		639
#define ATTR_LIFTER_B_LOAD(_val) 		sacp::Attribute(639, (uint8_t)_val) 		/* 639  ATTR_LIFTER_B_LOAD                       uint8_t RD */
#define ATTR_LIFTER_B_VOLTAGE_ID 		640
#define ATTR_LIFTER_B_VOLTAGE(_val) 		sacp::Attribute(640, (float)_val) 		/* 640  ATTR_LIFTER_B_VOLTAGE                    float  RD */
#define ATTR_LIFTER_B_CURRRENT_ID 		641
#define ATTR_LIFTER_B_CURRRENT(_val) 		sacp::Attribute(641, (float)_val) 		/* 641  ATTR_LIFTER_B_CURRRENT                   float  RD */
#define ATTR_LIFTER_B_RUNNING_ID 		642
#define ATTR_LIFTER_B_RUNNING(_val) 		sacp::Attribute(642, (bool)_val) 		/* 642  ATTR_LIFTER_B_RUNNING                    bool   RD */
#define ATTR_LIFTER_B_ALARM_STATUS_ID 		643
#define ATTR_LIFTER_B_ALARM_STATUS(_val) 		sacp::Attribute(643, (uint16_t)_val) 		/* 643  ATTR_LIFTER_B_ALARM_STATUS               uint16_t RD */
#define ATTR_LIFTER_B_SET_POSITION_ID 		644
#define ATTR_LIFTER_B_SET_POSITION(_val) 		sacp::Attribute(644, (uint8_t)_val) 		/* 644  ATTR_LIFTER_B_SET_POSITION               uint8_t RW */
#define ATTR_LIFTER_C_MODEL_ID 		650
#define ATTR_LIFTER_C_MODEL(_val) 		sacp::Attribute(650, _val) 		/* 650  ATTR_LIFTER_C_MODEL                      octet  RD */
#define ATTR_LIFTER_C_SN_ID 		651
#define ATTR_LIFTER_C_SN(_val) 		sacp::Attribute(651, _val) 		/* 651  ATTR_LIFTER_C_SN                         octet  RD */
#define ATTR_LIFTER_C_HW_VERSION_ID 		652
#define ATTR_LIFTER_C_HW_VERSION(_val) 		sacp::Attribute(652, (uint16_t)_val) 		/* 652  ATTR_LIFTER_C_HW_VERSION                 uint16_t RD */
#define ATTR_LIFTER_C_SW_VERSION_ID 		653
#define ATTR_LIFTER_C_SW_VERSION(_val) 		sacp::Attribute(653, (uint16_t)_val) 		/* 653  ATTR_LIFTER_C_SW_VERSION                 uint16_t RD */
#define ATTR_LIFTER_C_ADDRESS_ID 		654
#define ATTR_LIFTER_C_ADDRESS(_val) 		sacp::Attribute(654, (uint8_t)_val) 		/* 654  ATTR_LIFTER_C_ADDRESS                    uint8_t RD */
#define ATTR_LIFTER_C_NAME_ID 		655
#define ATTR_LIFTER_C_NAME(_val) 		sacp::Attribute(655, _val) 		/* 655  ATTR_LIFTER_C_NAME                       octet  RD */
#define ATTR_LIFTER_C_LINK_STATUS_ID 		656
#define ATTR_LIFTER_C_LINK_STATUS(_val) 		sacp::Attribute(656, (uint8_t)_val) 		/* 656  ATTR_LIFTER_C_LINK_STATUS                uint8_t RD */
#define ATTR_LIFTER_C_CONNECTED_TIME_ID 		657
#define ATTR_LIFTER_C_CONNECTED_TIME(_val) 		sacp::Attribute(657, (uint32_t)_val) 		/* 657  ATTR_LIFTER_C_CONNECTED_TIME             uint32_t RD */
#define ATTR_LIFTER_C_DISCONNECTED_TIME_ID 		658
#define ATTR_LIFTER_C_DISCONNECTED_TIME(_val) 		sacp::Attribute(658, (uint32_t)_val) 		/* 658  ATTR_LIFTER_C_DISCONNECTED_TIME          uint32_t RD */
#define ATTR_LIFTER_C_POSITION_ID 		659
#define ATTR_LIFTER_C_POSITION(_val) 		sacp::Attribute(659, (uint8_t)_val) 		/* 659  ATTR_LIFTER_C_POSITION                   uint8_t RD */
#define ATTR_LIFTER_C_RAW_POSITION_ID 		660
#define ATTR_LIFTER_C_RAW_POSITION(_val) 		sacp::Attribute(660, (uint16_t)_val) 		/* 660  ATTR_LIFTER_C_RAW_POSITION               uint16_t RD */
#define ATTR_LIFTER_C_TEMPERATURE_ID 		661
#define ATTR_LIFTER_C_TEMPERATURE(_val) 		sacp::Attribute(661, (float)_val) 		/* 661  ATTR_LIFTER_C_TEMPERATURE                float  RD */
#define ATTR_LIFTER_C_LOAD_ID 		664
#define ATTR_LIFTER_C_LOAD(_val) 		sacp::Attribute(664, (uint8_t)_val) 		/* 664  ATTR_LIFTER_C_LOAD                       uint8_t RD */
#define ATTR_LIFTER_C_VOLTAGE_ID 		665
#define ATTR_LIFTER_C_VOLTAGE(_val) 		sacp::Attribute(665, (float)_val) 		/* 665  ATTR_LIFTER_C_VOLTAGE                    float  RD */
#define ATTR_LIFTER_C_CURRRENT_ID 		666
#define ATTR_LIFTER_C_CURRRENT(_val) 		sacp::Attribute(666, (float)_val) 		/* 666  ATTR_LIFTER_C_CURRRENT                   float  RD */
#define ATTR_LIFTER_C_RUNNING_ID 		667
#define ATTR_LIFTER_C_RUNNING(_val) 		sacp::Attribute(667, (bool)_val) 		/* 667  ATTR_LIFTER_C_RUNNING                    bool   RD */
#define ATTR_LIFTER_C_ALARM_STATUS_ID 		668
#define ATTR_LIFTER_C_ALARM_STATUS(_val) 		sacp::Attribute(668, (uint16_t)_val) 		/* 668  ATTR_LIFTER_C_ALARM_STATUS               uint16_t RD */
#define ATTR_LIFTER_C_SET_POSITION_ID 		669
#define ATTR_LIFTER_C_SET_POSITION(_val) 		sacp::Attribute(669, (uint8_t)_val) 		/* 669  ATTR_LIFTER_C_SET_POSITION               uint8_t RW */
#define ATTR_LIFTER_D_MODEL_ID 		675
#define ATTR_LIFTER_D_MODEL(_val) 		sacp::Attribute(675, _val) 		/* 675  ATTR_LIFTER_D_MODEL                      octet  RD */
#define ATTR_LIFTER_D_SN_ID 		676
#define ATTR_LIFTER_D_SN(_val) 		sacp::Attribute(676, _val) 		/* 676  ATTR_LIFTER_D_SN                         octet  RD */
#define ATTR_LIFTER_D_HW_VERSION_ID 		677
#define ATTR_LIFTER_D_HW_VERSION(_val) 		sacp::Attribute(677, (uint16_t)_val) 		/* 677  ATTR_LIFTER_D_HW_VERSION                 uint16_t RD */
#define ATTR_LIFTER_D_SW_VERSION_ID 		678
#define ATTR_LIFTER_D_SW_VERSION(_val) 		sacp::Attribute(678, (uint16_t)_val) 		/* 678  ATTR_LIFTER_D_SW_VERSION                 uint16_t RD */
#define ATTR_LIFTER_D_ADDRESS_ID 		679
#define ATTR_LIFTER_D_ADDRESS(_val) 		sacp::Attribute(679, (uint8_t)_val) 		/* 679  ATTR_LIFTER_D_ADDRESS                    uint8_t RD */
#define ATTR_LIFTER_D_NAME_ID 		680
#define ATTR_LIFTER_D_NAME(_val) 		sacp::Attribute(680, _val) 		/* 680  ATTR_LIFTER_D_NAME                       octet  RD */
#define ATTR_LIFTER_D_LINK_STATUS_ID 		681
#define ATTR_LIFTER_D_LINK_STATUS(_val) 		sacp::Attribute(681, (uint8_t)_val) 		/* 681  ATTR_LIFTER_D_LINK_STATUS                uint8_t RD */
#define ATTR_LIFTER_D_CONNECTED_TIME_ID 		682
#define ATTR_LIFTER_D_CONNECTED_TIME(_val) 		sacp::Attribute(682, (uint32_t)_val) 		/* 682  ATTR_LIFTER_D_CONNECTED_TIME             uint32_t RD */
#define ATTR_LIFTER_D_DISCONNECTED_TIME_ID 		683
#define ATTR_LIFTER_D_DISCONNECTED_TIME(_val) 		sacp::Attribute(683, (uint32_t)_val) 		/* 683  ATTR_LIFTER_D_DISCONNECTED_TIME          uint32_t RD */
#define ATTR_LIFTER_D_POSITION_ID 		684
#define ATTR_LIFTER_D_POSITION(_val) 		sacp::Attribute(684, (uint8_t)_val) 		/* 684  ATTR_LIFTER_D_POSITION                   uint8_t RD */
#define ATTR_LIFTER_D_RAW_POSITION_ID 		685
#define ATTR_LIFTER_D_RAW_POSITION(_val) 		sacp::Attribute(685, (uint16_t)_val) 		/* 685  ATTR_LIFTER_D_RAW_POSITION               uint16_t RD */
#define ATTR_LIFTER_D_TEMPERATURE_ID 		686
#define ATTR_LIFTER_D_TEMPERATURE(_val) 		sacp::Attribute(686, (float)_val) 		/* 686  ATTR_LIFTER_D_TEMPERATURE                float  RD */
#define ATTR_LIFTER_D_LOAD_ID 		689
#define ATTR_LIFTER_D_LOAD(_val) 		sacp::Attribute(689, (uint8_t)_val) 		/* 689  ATTR_LIFTER_D_LOAD                       uint8_t RD */
#define ATTR_LIFTER_D_VOLTAGE_ID 		690
#define ATTR_LIFTER_D_VOLTAGE(_val) 		sacp::Attribute(690, (float)_val) 		/* 690  ATTR_LIFTER_D_VOLTAGE                    float  RD */
#define ATTR_LIFTER_D_CURRRENT_ID 		691
#define ATTR_LIFTER_D_CURRRENT(_val) 		sacp::Attribute(691, (float)_val) 		/* 691  ATTR_LIFTER_D_CURRRENT                   float  RD */
#define ATTR_LIFTER_D_RUNNING_ID 		692
#define ATTR_LIFTER_D_RUNNING(_val) 		sacp::Attribute(692, (bool)_val) 		/* 692  ATTR_LIFTER_D_RUNNING                    bool   RD */
#define ATTR_LIFTER_D_ALARM_STATUS_ID 		693
#define ATTR_LIFTER_D_ALARM_STATUS(_val) 		sacp::Attribute(693, (uint16_t)_val) 		/* 693  ATTR_LIFTER_D_ALARM_STATUS               uint16_t RD */
#define ATTR_LIFTER_D_SET_POSITION_ID 		694
#define ATTR_LIFTER_D_SET_POSITION(_val) 		sacp::Attribute(694, (uint8_t)_val) 		/* 694  ATTR_LIFTER_D_SET_POSITION               uint8_t RW */

/* Total: 72 */


/* 属性组 - motor */

#define ATTR_MOTOR_RIGHT_MODEL_ID 		800
#define ATTR_MOTOR_RIGHT_MODEL(_val) 		sacp::Attribute(800, _val) 		/* 800  ATTR_MOTOR_RIGHT_MODEL                   octet  RD */
#define ATTR_MOTOR_RIGHT_SN_ID 		801
#define ATTR_MOTOR_RIGHT_SN(_val) 		sacp::Attribute(801, _val) 		/* 801  ATTR_MOTOR_RIGHT_SN                      octet  RD */
#define ATTR_MOTOR_RIGHT_HW_VERSION_ID 		802
#define ATTR_MOTOR_RIGHT_HW_VERSION(_val) 		sacp::Attribute(802, _val) 		/* 802  ATTR_MOTOR_RIGHT_HW_VERSION              octet  RD */
#define ATTR_MOTOR_RIGHT_SW_VERSION_ID 		803
#define ATTR_MOTOR_RIGHT_SW_VERSION(_val) 		sacp::Attribute(803, _val) 		/* 803  ATTR_MOTOR_RIGHT_SW_VERSION              octet  RD */
#define ATTR_MOTOR_RIGHT_ADDRESS_ID 		804
#define ATTR_MOTOR_RIGHT_ADDRESS(_val) 		sacp::Attribute(804, (uint8_t)_val) 		/* 804  ATTR_MOTOR_RIGHT_ADDRESS                 uint8_t RD */
#define ATTR_MOTOR_RIGHT_NAME_ID 		805
#define ATTR_MOTOR_RIGHT_NAME(_val) 		sacp::Attribute(805, _val) 		/* 805  ATTR_MOTOR_RIGHT_NAME                    octet  RD */
#define ATTR_MOTOR_RIGHT_LINK_STATUS_ID 		806
#define ATTR_MOTOR_RIGHT_LINK_STATUS(_val) 		sacp::Attribute(806, (uint8_t)_val) 		/* 806  ATTR_MOTOR_RIGHT_LINK_STATUS             uint8_t RD */
#define ATTR_MOTOR_RIGHT_NODE_STATE_ID 		807
#define ATTR_MOTOR_RIGHT_NODE_STATE(_val) 		sacp::Attribute(807, (uint8_t)_val) 		/* 807  ATTR_MOTOR_RIGHT_NODE_STATE              uint8_t RD */
#define ATTR_MOTOR_RIGHT_CONNECTED_TIME_ID 		808
#define ATTR_MOTOR_RIGHT_CONNECTED_TIME(_val) 		sacp::Attribute(808, (uint32_t)_val) 		/* 808  ATTR_MOTOR_RIGHT_CONNECTED_TIME          uint32_t RD */
#define ATTR_MOTOR_RIGHT_DISCONNECTED_TIME_ID 		809
#define ATTR_MOTOR_RIGHT_DISCONNECTED_TIME(_val) 		sacp::Attribute(809, (uint32_t)_val) 		/* 809  ATTR_MOTOR_RIGHT_DISCONNECTED_TIME       uint32_t RD */
#define ATTR_MOTOR_RIGHT_TEMPERATURE_ID 		810
#define ATTR_MOTOR_RIGHT_TEMPERATURE(_val) 		sacp::Attribute(810, (float)_val) 		/* 810  ATTR_MOTOR_RIGHT_TEMPERATURE             float  RD */
#define ATTR_MOTOR_RIGHT_VOLTAGE_ID 		811
#define ATTR_MOTOR_RIGHT_VOLTAGE(_val) 		sacp::Attribute(811, (float)_val) 		/* 811  ATTR_MOTOR_RIGHT_VOLTAGE                 float  RD */
#define ATTR_MOTOR_RIGHT_CURRENT_ID 		812
#define ATTR_MOTOR_RIGHT_CURRENT(_val) 		sacp::Attribute(812, (float)_val) 		/* 812  ATTR_MOTOR_RIGHT_CURRENT                 float  RD */
#define ATTR_MOTOR_RIGHT_RAW_SPEED_ID 		813
#define ATTR_MOTOR_RIGHT_RAW_SPEED(_val) 		sacp::Attribute(813, (float)_val) 		/* 813  ATTR_MOTOR_RIGHT_RAW_SPEED               float  RD */
#define ATTR_MOTOR_RIGHT_RAW_POSITION_ID 		814
#define ATTR_MOTOR_RIGHT_RAW_POSITION(_val) 		sacp::Attribute(814, (int32_t)_val) 		/* 814  ATTR_MOTOR_RIGHT_RAW_POSITION            int32_t RD */
#define ATTR_MOTOR_RIGHT_RAW_TARGET_SPEED_ID 		815
#define ATTR_MOTOR_RIGHT_RAW_TARGET_SPEED(_val) 		sacp::Attribute(815, (float)_val) 		/* 815  ATTR_MOTOR_RIGHT_RAW_TARGET_SPEED        float  RD */
#define ATTR_MOTOR_RIGHT_RAW_TARGET_POSITION_ID 		816
#define ATTR_MOTOR_RIGHT_RAW_TARGET_POSITION(_val) 		sacp::Attribute(816, (int32_t)_val) 		/* 816  ATTR_MOTOR_RIGHT_RAW_TARGET_POSITION     int32_t RD */
#define ATTR_MOTOR_RIGHT_EXTRA_ENCODER_POSITION_ID 		817
#define ATTR_MOTOR_RIGHT_EXTRA_ENCODER_POSITION(_val) 		sacp::Attribute(817, (int32_t)_val) 		/* 817  ATTR_MOTOR_RIGHT_EXTRA_ENCODER_POSITION  int32_t RD */
#define ATTR_MOTOR_RIGHT_EXTRA_ENCODER_STATE_ID 		818
#define ATTR_MOTOR_RIGHT_EXTRA_ENCODER_STATE(_val) 		sacp::Attribute(818, (uint8_t)_val) 		/* 818  ATTR_MOTOR_RIGHT_EXTRA_ENCODER_STATE     uint8_t RD */
#define ATTR_MOTOR_RIGHT_STATUS_CODE_ID 		819
#define ATTR_MOTOR_RIGHT_STATUS_CODE(_val) 		sacp::Attribute(819, (uint16_t)_val) 		/* 819  ATTR_MOTOR_RIGHT_STATUS_CODE             uint16_t RD */
#define ATTR_MOTOR_RIGHT_EMERGENCY_ERROR_CODE_ID 		820
#define ATTR_MOTOR_RIGHT_EMERGENCY_ERROR_CODE(_val) 		sacp::Attribute(820, (uint16_t)_val) 		/* 820  ATTR_MOTOR_RIGHT_EMERGENCY_ERROR_CODE    uint16_t RD */
#define ATTR_MOTOR_RIGHT_EMERGENCY_STATUS_CODE_ID 		821
#define ATTR_MOTOR_RIGHT_EMERGENCY_STATUS_CODE(_val) 		sacp::Attribute(821, (uint8_t)_val) 		/* 821  ATTR_MOTOR_RIGHT_EMERGENCY_STATUS_CODE   uint8_t RD */
#define ATTR_MOTOR_RIGHT_ERROR_STATUS_ID 		822
#define ATTR_MOTOR_RIGHT_ERROR_STATUS(_val) 		sacp::Attribute(822, (uint32_t)_val) 		/* 822  ATTR_MOTOR_RIGHT_ERROR_STATUS            uint32_t RD */
#define ATTR_MOTOR_RIGHT_DEBUG_TARGET_ID 		823
#define ATTR_MOTOR_RIGHT_DEBUG_TARGET(_val) 		sacp::Attribute(823, (float)_val) 		/* 823  ATTR_MOTOR_RIGHT_DEBUG_TARGET            float  RW */
#define ATTR_MOTOR_RIGHT_EXTRA_COMMAND_ID 		824
#define ATTR_MOTOR_RIGHT_EXTRA_COMMAND(_val) 		sacp::Attribute(824, (uint32_t)_val) 		/* 824  ATTR_MOTOR_RIGHT_EXTRA_COMMAND           uint32_t RW */
#define ATTR_MOTOR_LEFT_MODEL_ID 		850
#define ATTR_MOTOR_LEFT_MODEL(_val) 		sacp::Attribute(850, _val) 		/* 850  ATTR_MOTOR_LEFT_MODEL                    octet  RD */
#define ATTR_MOTOR_LEFT_SN_ID 		851
#define ATTR_MOTOR_LEFT_SN(_val) 		sacp::Attribute(851, _val) 		/* 851  ATTR_MOTOR_LEFT_SN                       octet  RD */
#define ATTR_MOTOR_LEFT_HW_VERSION_ID 		852
#define ATTR_MOTOR_LEFT_HW_VERSION(_val) 		sacp::Attribute(852, _val) 		/* 852  ATTR_MOTOR_LEFT_HW_VERSION               octet  RD */
#define ATTR_MOTOR_LEFT_SW_VERSION_ID 		853
#define ATTR_MOTOR_LEFT_SW_VERSION(_val) 		sacp::Attribute(853, _val) 		/* 853  ATTR_MOTOR_LEFT_SW_VERSION               octet  RD */
#define ATTR_MOTOR_LEFT_ADDRESS_ID 		854
#define ATTR_MOTOR_LEFT_ADDRESS(_val) 		sacp::Attribute(854, (uint8_t)_val) 		/* 854  ATTR_MOTOR_LEFT_ADDRESS                  uint8_t RD */
#define ATTR_MOTOR_LEFT_NAME_ID 		855
#define ATTR_MOTOR_LEFT_NAME(_val) 		sacp::Attribute(855, _val) 		/* 855  ATTR_MOTOR_LEFT_NAME                     octet  RD */
#define ATTR_MOTOR_LEFT_LINK_STATUS_ID 		856
#define ATTR_MOTOR_LEFT_LINK_STATUS(_val) 		sacp::Attribute(856, (uint8_t)_val) 		/* 856  ATTR_MOTOR_LEFT_LINK_STATUS              uint8_t RD */
#define ATTR_MOTOR_LEFT_NODE_STATE_ID 		857
#define ATTR_MOTOR_LEFT_NODE_STATE(_val) 		sacp::Attribute(857, (uint8_t)_val) 		/* 857  ATTR_MOTOR_LEFT_NODE_STATE               uint8_t RD */
#define ATTR_MOTOR_LEFT_CONNECTED_TIME_ID 		858
#define ATTR_MOTOR_LEFT_CONNECTED_TIME(_val) 		sacp::Attribute(858, (uint32_t)_val) 		/* 858  ATTR_MOTOR_LEFT_CONNECTED_TIME           uint32_t RD */
#define ATTR_MOTOR_LEFT_DISCONNECTED_TIME_ID 		859
#define ATTR_MOTOR_LEFT_DISCONNECTED_TIME(_val) 		sacp::Attribute(859, (uint32_t)_val) 		/* 859  ATTR_MOTOR_LEFT_DISCONNECTED_TIME        uint32_t RD */
#define ATTR_MOTOR_LEFT_TEMPERATURE_ID 		860
#define ATTR_MOTOR_LEFT_TEMPERATURE(_val) 		sacp::Attribute(860, (float)_val) 		/* 860  ATTR_MOTOR_LEFT_TEMPERATURE              float  RD */
#define ATTR_MOTOR_LEFT_VOLTAGE_ID 		861
#define ATTR_MOTOR_LEFT_VOLTAGE(_val) 		sacp::Attribute(861, (float)_val) 		/* 861  ATTR_MOTOR_LEFT_VOLTAGE                  float  RD */
#define ATTR_MOTOR_LEFT_CURRENT_ID 		862
#define ATTR_MOTOR_LEFT_CURRENT(_val) 		sacp::Attribute(862, (float)_val) 		/* 862  ATTR_MOTOR_LEFT_CURRENT                  float  RD */
#define ATTR_MOTOR_LEFT_RAW_SPEED_ID 		863
#define ATTR_MOTOR_LEFT_RAW_SPEED(_val) 		sacp::Attribute(863, (float)_val) 		/* 863  ATTR_MOTOR_LEFT_RAW_SPEED                float  RD */
#define ATTR_MOTOR_LEFT_RAW_POSITION_ID 		864
#define ATTR_MOTOR_LEFT_RAW_POSITION(_val) 		sacp::Attribute(864, (int32_t)_val) 		/* 864  ATTR_MOTOR_LEFT_RAW_POSITION             int32_t RD */
#define ATTR_MOTOR_LEFT_RAW_TARGET_SPEED_ID 		865
#define ATTR_MOTOR_LEFT_RAW_TARGET_SPEED(_val) 		sacp::Attribute(865, (float)_val) 		/* 865  ATTR_MOTOR_LEFT_RAW_TARGET_SPEED         float  RD */
#define ATTR_MOTOR_LEFT_RAW_TARGET_POSITION_ID 		866
#define ATTR_MOTOR_LEFT_RAW_TARGET_POSITION(_val) 		sacp::Attribute(866, (int32_t)_val) 		/* 866  ATTR_MOTOR_LEFT_RAW_TARGET_POSITION      int32_t RD */
#define ATTR_MOTOR_LEFT_EXTRA_ENCODER_POSITION_ID 		867
#define ATTR_MOTOR_LEFT_EXTRA_ENCODER_POSITION(_val) 		sacp::Attribute(867, (int32_t)_val) 		/* 867  ATTR_MOTOR_LEFT_EXTRA_ENCODER_POSITION   int32_t RD */
#define ATTR_MOTOR_LEFT_EXTRA_ENCODER_STATE_ID 		868
#define ATTR_MOTOR_LEFT_EXTRA_ENCODER_STATE(_val) 		sacp::Attribute(868, (uint8_t)_val) 		/* 868  ATTR_MOTOR_LEFT_EXTRA_ENCODER_STATE      uint8_t RD */
#define ATTR_MOTOR_LEFT_STATUS_CODE_ID 		869
#define ATTR_MOTOR_LEFT_STATUS_CODE(_val) 		sacp::Attribute(869, (uint16_t)_val) 		/* 869  ATTR_MOTOR_LEFT_STATUS_CODE              uint16_t RD */
#define ATTR_MOTOR_LEFT_EMERGENCY_ERROR_CODE_ID 		870
#define ATTR_MOTOR_LEFT_EMERGENCY_ERROR_CODE(_val) 		sacp::Attribute(870, (uint16_t)_val) 		/* 870  ATTR_MOTOR_LEFT_EMERGENCY_ERROR_CODE     uint16_t RD */
#define ATTR_MOTOR_LEFT_EMERGENCY_STATUS_CODE_ID 		871
#define ATTR_MOTOR_LEFT_EMERGENCY_STATUS_CODE(_val) 		sacp::Attribute(871, (uint8_t)_val) 		/* 871  ATTR_MOTOR_LEFT_EMERGENCY_STATUS_CODE    uint8_t RD */
#define ATTR_MOTOR_LEFT_ERROR_STATUS_ID 		872
#define ATTR_MOTOR_LEFT_ERROR_STATUS(_val) 		sacp::Attribute(872, (uint32_t)_val) 		/* 872  ATTR_MOTOR_LEFT_ERROR_STATUS             uint32_t RD */
#define ATTR_MOTOR_LEFT_DEBUG_TARGET_ID 		873
#define ATTR_MOTOR_LEFT_DEBUG_TARGET(_val) 		sacp::Attribute(873, (float)_val) 		/* 873  ATTR_MOTOR_LEFT_DEBUG_TARGET             float  RW */
#define ATTR_MOTOR_LEFT_EXTRA_COMMAND_ID 		874
#define ATTR_MOTOR_LEFT_EXTRA_COMMAND(_val) 		sacp::Attribute(874, (uint32_t)_val) 		/* 874  ATTR_MOTOR_LEFT_EXTRA_COMMAND            uint32_t RW */
#define ATTR_MOTOR_FRONT_MODEL_ID 		900
#define ATTR_MOTOR_FRONT_MODEL(_val) 		sacp::Attribute(900, _val) 		/* 900  ATTR_MOTOR_FRONT_MODEL                   octet  RD */
#define ATTR_MOTOR_FRONT_SN_ID 		901
#define ATTR_MOTOR_FRONT_SN(_val) 		sacp::Attribute(901, _val) 		/* 901  ATTR_MOTOR_FRONT_SN                      octet  RD */
#define ATTR_MOTOR_FRONT_HW_VERSION_ID 		902
#define ATTR_MOTOR_FRONT_HW_VERSION(_val) 		sacp::Attribute(902, _val) 		/* 902  ATTR_MOTOR_FRONT_HW_VERSION              octet  RD */
#define ATTR_MOTOR_FRONT_SW_VERSION_ID 		903
#define ATTR_MOTOR_FRONT_SW_VERSION(_val) 		sacp::Attribute(903, _val) 		/* 903  ATTR_MOTOR_FRONT_SW_VERSION              octet  RD */
#define ATTR_MOTOR_FRONT_ADDRESS_ID 		904
#define ATTR_MOTOR_FRONT_ADDRESS(_val) 		sacp::Attribute(904, (uint8_t)_val) 		/* 904  ATTR_MOTOR_FRONT_ADDRESS                 uint8_t RD */
#define ATTR_MOTOR_FRONT_NAME_ID 		905
#define ATTR_MOTOR_FRONT_NAME(_val) 		sacp::Attribute(905, _val) 		/* 905  ATTR_MOTOR_FRONT_NAME                    octet  RD */
#define ATTR_MOTOR_FRONT_LINK_STATUS_ID 		906
#define ATTR_MOTOR_FRONT_LINK_STATUS(_val) 		sacp::Attribute(906, (uint8_t)_val) 		/* 906  ATTR_MOTOR_FRONT_LINK_STATUS             uint8_t RD */
#define ATTR_MOTOR_FRONT_NODE_STATE_ID 		907
#define ATTR_MOTOR_FRONT_NODE_STATE(_val) 		sacp::Attribute(907, (uint8_t)_val) 		/* 907  ATTR_MOTOR_FRONT_NODE_STATE              uint8_t RD */
#define ATTR_MOTOR_FRONT_CONNECTED_TIME_ID 		908
#define ATTR_MOTOR_FRONT_CONNECTED_TIME(_val) 		sacp::Attribute(908, (uint32_t)_val) 		/* 908  ATTR_MOTOR_FRONT_CONNECTED_TIME          uint32_t RD */
#define ATTR_MOTOR_FRONT_DISCONNECTED_TIME_ID 		909
#define ATTR_MOTOR_FRONT_DISCONNECTED_TIME(_val) 		sacp::Attribute(909, (uint32_t)_val) 		/* 909  ATTR_MOTOR_FRONT_DISCONNECTED_TIME       uint32_t RD */
#define ATTR_MOTOR_FRONT_TEMPERATURE_ID 		910
#define ATTR_MOTOR_FRONT_TEMPERATURE(_val) 		sacp::Attribute(910, (float)_val) 		/* 910  ATTR_MOTOR_FRONT_TEMPERATURE             float  RD */
#define ATTR_MOTOR_FRONT_VOLTAGE_ID 		911
#define ATTR_MOTOR_FRONT_VOLTAGE(_val) 		sacp::Attribute(911, (float)_val) 		/* 911  ATTR_MOTOR_FRONT_VOLTAGE                 float  RD */
#define ATTR_MOTOR_FRONT_CURRENT_ID 		912
#define ATTR_MOTOR_FRONT_CURRENT(_val) 		sacp::Attribute(912, (float)_val) 		/* 912  ATTR_MOTOR_FRONT_CURRENT                 float  RD */
#define ATTR_MOTOR_FRONT_RAW_SPEED_ID 		913
#define ATTR_MOTOR_FRONT_RAW_SPEED(_val) 		sacp::Attribute(913, (float)_val) 		/* 913  ATTR_MOTOR_FRONT_RAW_SPEED               float  RD */
#define ATTR_MOTOR_FRONT_RAW_POSITION_ID 		914
#define ATTR_MOTOR_FRONT_RAW_POSITION(_val) 		sacp::Attribute(914, (int32_t)_val) 		/* 914  ATTR_MOTOR_FRONT_RAW_POSITION            int32_t RD */
#define ATTR_MOTOR_FRONT_RAW_TARGET_SPEED_ID 		915
#define ATTR_MOTOR_FRONT_RAW_TARGET_SPEED(_val) 		sacp::Attribute(915, (float)_val) 		/* 915  ATTR_MOTOR_FRONT_RAW_TARGET_SPEED        float  RD */
#define ATTR_MOTOR_FRONT_RAW_TARGET_POSITION_ID 		916
#define ATTR_MOTOR_FRONT_RAW_TARGET_POSITION(_val) 		sacp::Attribute(916, (int32_t)_val) 		/* 916  ATTR_MOTOR_FRONT_RAW_TARGET_POSITION     int32_t RD */
#define ATTR_MOTOR_FRONT_EXTRA_ENCODER_POSITION_ID 		917
#define ATTR_MOTOR_FRONT_EXTRA_ENCODER_POSITION(_val) 		sacp::Attribute(917, (int32_t)_val) 		/* 917  ATTR_MOTOR_FRONT_EXTRA_ENCODER_POSITION  int32_t RD */
#define ATTR_MOTOR_FRONT_EXTRA_ENCODER_STATE_ID 		918
#define ATTR_MOTOR_FRONT_EXTRA_ENCODER_STATE(_val) 		sacp::Attribute(918, (uint8_t)_val) 		/* 918  ATTR_MOTOR_FRONT_EXTRA_ENCODER_STATE     uint8_t RD */
#define ATTR_MOTOR_FRONT_STATUS_CODE_ID 		919
#define ATTR_MOTOR_FRONT_STATUS_CODE(_val) 		sacp::Attribute(919, (uint16_t)_val) 		/* 919  ATTR_MOTOR_FRONT_STATUS_CODE             uint16_t RD */
#define ATTR_MOTOR_FRONT_EMERGENCY_ERROR_CODE_ID 		920
#define ATTR_MOTOR_FRONT_EMERGENCY_ERROR_CODE(_val) 		sacp::Attribute(920, (uint16_t)_val) 		/* 920  ATTR_MOTOR_FRONT_EMERGENCY_ERROR_CODE    uint16_t RD */
#define ATTR_MOTOR_FRONT_EMERGENCY_STATUS_CODE_ID 		921
#define ATTR_MOTOR_FRONT_EMERGENCY_STATUS_CODE(_val) 		sacp::Attribute(921, (uint8_t)_val) 		/* 921  ATTR_MOTOR_FRONT_EMERGENCY_STATUS_CODE   uint8_t RD */
#define ATTR_MOTOR_FRONT_ERROR_STATUS_ID 		922
#define ATTR_MOTOR_FRONT_ERROR_STATUS(_val) 		sacp::Attribute(922, (uint32_t)_val) 		/* 922  ATTR_MOTOR_FRONT_ERROR_STATUS            uint32_t RD */
#define ATTR_MOTOR_FRONT_DEBUG_TARGET_ID 		923
#define ATTR_MOTOR_FRONT_DEBUG_TARGET(_val) 		sacp::Attribute(923, (float)_val) 		/* 923  ATTR_MOTOR_FRONT_DEBUG_TARGET            float  RW */
#define ATTR_MOTOR_FRONT_EXTRA_COMMAND_ID 		924
#define ATTR_MOTOR_FRONT_EXTRA_COMMAND(_val) 		sacp::Attribute(924, (uint32_t)_val) 		/* 924  ATTR_MOTOR_FRONT_EXTRA_COMMAND           uint32_t RW */
#define ATTR_MOTOR_STEERING_MODEL_ID 		950
#define ATTR_MOTOR_STEERING_MODEL(_val) 		sacp::Attribute(950, _val) 		/* 950  ATTR_MOTOR_STEERING_MODEL                octet  RD */
#define ATTR_MOTOR_STEERING_SN_ID 		951
#define ATTR_MOTOR_STEERING_SN(_val) 		sacp::Attribute(951, _val) 		/* 951  ATTR_MOTOR_STEERING_SN                   octet  RD */
#define ATTR_MOTOR_STEERING_HW_VERSION_ID 		952
#define ATTR_MOTOR_STEERING_HW_VERSION(_val) 		sacp::Attribute(952, _val) 		/* 952  ATTR_MOTOR_STEERING_HW_VERSION           octet  RD */
#define ATTR_MOTOR_STEERING_SW_VERSION_ID 		953
#define ATTR_MOTOR_STEERING_SW_VERSION(_val) 		sacp::Attribute(953, _val) 		/* 953  ATTR_MOTOR_STEERING_SW_VERSION           octet  RD */
#define ATTR_MOTOR_STEERING_ADDRESS_ID 		954
#define ATTR_MOTOR_STEERING_ADDRESS(_val) 		sacp::Attribute(954, (uint8_t)_val) 		/* 954  ATTR_MOTOR_STEERING_ADDRESS              uint8_t RD */
#define ATTR_MOTOR_STEERING_NAME_ID 		955
#define ATTR_MOTOR_STEERING_NAME(_val) 		sacp::Attribute(955, _val) 		/* 955  ATTR_MOTOR_STEERING_NAME                 octet  RD */
#define ATTR_MOTOR_STEERING_LINK_STATUS_ID 		956
#define ATTR_MOTOR_STEERING_LINK_STATUS(_val) 		sacp::Attribute(956, (uint8_t)_val) 		/* 956  ATTR_MOTOR_STEERING_LINK_STATUS          uint8_t RD */
#define ATTR_MOTOR_STEERING_NODE_STATE_ID 		957
#define ATTR_MOTOR_STEERING_NODE_STATE(_val) 		sacp::Attribute(957, (uint8_t)_val) 		/* 957  ATTR_MOTOR_STEERING_NODE_STATE           uint8_t RD */
#define ATTR_MOTOR_STEERING_CONNECTED_TIME_ID 		958
#define ATTR_MOTOR_STEERING_CONNECTED_TIME(_val) 		sacp::Attribute(958, (uint32_t)_val) 		/* 958  ATTR_MOTOR_STEERING_CONNECTED_TIME       uint32_t RD */
#define ATTR_MOTOR_STEERING_DISCONNECTED_TIME_ID 		959
#define ATTR_MOTOR_STEERING_DISCONNECTED_TIME(_val) 		sacp::Attribute(959, (uint32_t)_val) 		/* 959  ATTR_MOTOR_STEERING_DISCONNECTED_TIME    uint32_t RD */
#define ATTR_MOTOR_STEERING_TEMPERATURE_ID 		960
#define ATTR_MOTOR_STEERING_TEMPERATURE(_val) 		sacp::Attribute(960, (float)_val) 		/* 960  ATTR_MOTOR_STEERING_TEMPERATURE          float  RD */
#define ATTR_MOTOR_STEERING_VOLTAGE_ID 		961
#define ATTR_MOTOR_STEERING_VOLTAGE(_val) 		sacp::Attribute(961, (float)_val) 		/* 961  ATTR_MOTOR_STEERING_VOLTAGE              float  RD */
#define ATTR_MOTOR_STEERING_CURRENT_ID 		962
#define ATTR_MOTOR_STEERING_CURRENT(_val) 		sacp::Attribute(962, (float)_val) 		/* 962  ATTR_MOTOR_STEERING_CURRENT              float  RD */
#define ATTR_MOTOR_STEERING_RAW_SPEED_ID 		963
#define ATTR_MOTOR_STEERING_RAW_SPEED(_val) 		sacp::Attribute(963, (float)_val) 		/* 963  ATTR_MOTOR_STEERING_RAW_SPEED            float  RD */
#define ATTR_MOTOR_STEERING_RAW_POSITION_ID 		964
#define ATTR_MOTOR_STEERING_RAW_POSITION(_val) 		sacp::Attribute(964, (int32_t)_val) 		/* 964  ATTR_MOTOR_STEERING_RAW_POSITION         int32_t RD */
#define ATTR_MOTOR_STEERING_RAW_TARGET_SPEED_ID 		965
#define ATTR_MOTOR_STEERING_RAW_TARGET_SPEED(_val) 		sacp::Attribute(965, (float)_val) 		/* 965  ATTR_MOTOR_STEERING_RAW_TARGET_SPEED     float  RD */
#define ATTR_MOTOR_STEERING_RAW_TARGET_POSITION_ID 		966
#define ATTR_MOTOR_STEERING_RAW_TARGET_POSITION(_val) 		sacp::Attribute(966, (int32_t)_val) 		/* 966  ATTR_MOTOR_STEERING_RAW_TARGET_POSITION  int32_t RD */
#define ATTR_MOTOR_STEERING_EXTRA_ENCODER_POSITION_ID 		967
#define ATTR_MOTOR_STEERING_EXTRA_ENCODER_POSITION(_val) 		sacp::Attribute(967, (int32_t)_val) 		/* 967  ATTR_MOTOR_STEERING_EXTRA_ENCODER_POSITION int32_t RD */
#define ATTR_MOTOR_STEERING_EXTRA_ENCODER_STATE_ID 		968
#define ATTR_MOTOR_STEERING_EXTRA_ENCODER_STATE(_val) 		sacp::Attribute(968, (uint8_t)_val) 		/* 968  ATTR_MOTOR_STEERING_EXTRA_ENCODER_STATE  uint8_t RD */
#define ATTR_MOTOR_STEERING_STATUS_CODE_ID 		969
#define ATTR_MOTOR_STEERING_STATUS_CODE(_val) 		sacp::Attribute(969, (uint16_t)_val) 		/* 969  ATTR_MOTOR_STEERING_STATUS_CODE          uint16_t RD */
#define ATTR_MOTOR_STEERING_EMERGENCY_ERROR_CODE_ID 		970
#define ATTR_MOTOR_STEERING_EMERGENCY_ERROR_CODE(_val) 		sacp::Attribute(970, (uint16_t)_val) 		/* 970  ATTR_MOTOR_STEERING_EMERGENCY_ERROR_CODE uint16_t RD */
#define ATTR_MOTOR_STEERING_EMERGENCY_STATUS_CODE_ID 		971
#define ATTR_MOTOR_STEERING_EMERGENCY_STATUS_CODE(_val) 		sacp::Attribute(971, (uint8_t)_val) 		/* 971  ATTR_MOTOR_STEERING_EMERGENCY_STATUS_CODE uint8_t RD */
#define ATTR_MOTOR_STEERING_ERROR_STATUS_ID 		972
#define ATTR_MOTOR_STEERING_ERROR_STATUS(_val) 		sacp::Attribute(972, (uint32_t)_val) 		/* 972  ATTR_MOTOR_STEERING_ERROR_STATUS         uint32_t RD */
#define ATTR_MOTOR_STEERING_DEBUG_TARGET_ID 		973
#define ATTR_MOTOR_STEERING_DEBUG_TARGET(_val) 		sacp::Attribute(973, (float)_val) 		/* 973  ATTR_MOTOR_STEERING_DEBUG_TARGET         float  RW */
#define ATTR_MOTOR_STEERING_EXTRA_COMMAND_ID 		974
#define ATTR_MOTOR_STEERING_EXTRA_COMMAND(_val) 		sacp::Attribute(974, (uint32_t)_val) 		/* 974  ATTR_MOTOR_STEERING_EXTRA_COMMAND        uint32_t RW */

/* Total: 100 */


/* 属性组 - motion */

#define ATTR_MOTION_TARGET_VELOCITY_ID 		1000
#define ATTR_MOTION_TARGET_VELOCITY(_val) 		sacp::Attribute(1000, (float)_val) 		/* 1000 ATTR_MOTION_TARGET_VELOCITY              float  RD */
#define ATTR_MOTION_TARGET_STEERING_ANGLE_ID 		1001
#define ATTR_MOTION_TARGET_STEERING_ANGLE(_val) 		sacp::Attribute(1001, (float)_val) 		/* 1001 ATTR_MOTION_TARGET_STEERING_ANGLE        float  RD */
#define ATTR_MOTION_REAL_VELOCITY_ID 		1002
#define ATTR_MOTION_REAL_VELOCITY(_val) 		sacp::Attribute(1002, (float)_val) 		/* 1002 ATTR_MOTION_REAL_VELOCITY                float  RD */
#define ATTR_MOTION_REAL_STEERING_ANGLE_ID 		1003
#define ATTR_MOTION_REAL_STEERING_ANGLE(_val) 		sacp::Attribute(1003, (float)_val) 		/* 1003 ATTR_MOTION_REAL_STEERING_ANGLE          float  RD */
#define ATTR_MOTION_RIGHT_WHEEL_TARGET_VELOCITY_ID 		1004
#define ATTR_MOTION_RIGHT_WHEEL_TARGET_VELOCITY(_val) 		sacp::Attribute(1004, (float)_val) 		/* 1004 ATTR_MOTION_RIGHT_WHEEL_TARGET_VELOCITY  float  RD */
#define ATTR_MOTION_LEFT_WHEEL_TARGET_VELOCITY_ID 		1005
#define ATTR_MOTION_LEFT_WHEEL_TARGET_VELOCITY(_val) 		sacp::Attribute(1005, (float)_val) 		/* 1005 ATTR_MOTION_LEFT_WHEEL_TARGET_VELOCITY   float  RD */
#define ATTR_MOTION_FRONT_WHEEL_TARGET_VELOCITY_ID 		1006
#define ATTR_MOTION_FRONT_WHEEL_TARGET_VELOCITY(_val) 		sacp::Attribute(1006, (float)_val) 		/* 1006 ATTR_MOTION_FRONT_WHEEL_TARGET_VELOCITY  float  RD */
#define ATTR_MOTION_RIGHT_WHEEL_REAL_VELOCITY_ID 		1007
#define ATTR_MOTION_RIGHT_WHEEL_REAL_VELOCITY(_val) 		sacp::Attribute(1007, (float)_val) 		/* 1007 ATTR_MOTION_RIGHT_WHEEL_REAL_VELOCITY    float  RD */
#define ATTR_MOTION_LEFT_WHEEL_REAL_VELOCITY_ID 		1008
#define ATTR_MOTION_LEFT_WHEEL_REAL_VELOCITY(_val) 		sacp::Attribute(1008, (float)_val) 		/* 1008 ATTR_MOTION_LEFT_WHEEL_REAL_VELOCITY     float  RD */
#define ATTR_MOTION_FRONT_WHEEL_REAL_VELOCITY_ID 		1009
#define ATTR_MOTION_FRONT_WHEEL_REAL_VELOCITY(_val) 		sacp::Attribute(1009, (float)_val) 		/* 1009 ATTR_MOTION_FRONT_WHEEL_REAL_VELOCITY    float  RD */
#define ATTR_MOTION_RIGHT_MOTOR_REAL_VELOCITY_ID 		1010
#define ATTR_MOTION_RIGHT_MOTOR_REAL_VELOCITY(_val) 		sacp::Attribute(1010, (float)_val) 		/* 1010 ATTR_MOTION_RIGHT_MOTOR_REAL_VELOCITY    float  RD */
#define ATTR_MOTION_LEFT_MOTOR_REAL_VELOCITY_ID 		1011
#define ATTR_MOTION_LEFT_MOTOR_REAL_VELOCITY(_val) 		sacp::Attribute(1011, (float)_val) 		/* 1011 ATTR_MOTION_LEFT_MOTOR_REAL_VELOCITY     float  RD */
#define ATTR_MOTION_FRONT_MOTOR_REAL_VELOCITY_ID 		1012
#define ATTR_MOTION_FRONT_MOTOR_REAL_VELOCITY(_val) 		sacp::Attribute(1012, (float)_val) 		/* 1012 ATTR_MOTION_FRONT_MOTOR_REAL_VELOCITY    float  RD */
#define ATTR_MOTION_RIGHT_MOTOR_REAL_CURRENT_ID 		1013
#define ATTR_MOTION_RIGHT_MOTOR_REAL_CURRENT(_val) 		sacp::Attribute(1013, (float)_val) 		/* 1013 ATTR_MOTION_RIGHT_MOTOR_REAL_CURRENT     float  RD */
#define ATTR_MOTION_LEFT_MOTOR_REAL_CURRENT_ID 		1014
#define ATTR_MOTION_LEFT_MOTOR_REAL_CURRENT(_val) 		sacp::Attribute(1014, (float)_val) 		/* 1014 ATTR_MOTION_LEFT_MOTOR_REAL_CURRENT      float  RD */
#define ATTR_MOTION_FRONT_MOTOR_REAL_CURRENT_ID 		1015
#define ATTR_MOTION_FRONT_MOTOR_REAL_CURRENT(_val) 		sacp::Attribute(1015, (float)_val) 		/* 1015 ATTR_MOTION_FRONT_MOTOR_REAL_CURRENT     float  RD */
#define ATTR_MOTION_STEERING_MOTOR_REAL_CURRENT_ID 		1016
#define ATTR_MOTION_STEERING_MOTOR_REAL_CURRENT(_val) 		sacp::Attribute(1016, (float)_val) 		/* 1016 ATTR_MOTION_STEERING_MOTOR_REAL_CURRENT  float  RD */
#define ATTR_MOTION_RIGHT_WHEEL_DELTA_DISTANCE_ID 		1017
#define ATTR_MOTION_RIGHT_WHEEL_DELTA_DISTANCE(_val) 		sacp::Attribute(1017, (float)_val) 		/* 1017 ATTR_MOTION_RIGHT_WHEEL_DELTA_DISTANCE   float  RD */
#define ATTR_MOTION_LEFT_WHEEL_DELTA_DISTANCE_ID 		1018
#define ATTR_MOTION_LEFT_WHEEL_DELTA_DISTANCE(_val) 		sacp::Attribute(1018, (float)_val) 		/* 1018 ATTR_MOTION_LEFT_WHEEL_DELTA_DISTANCE    float  RD */
#define ATTR_MOTION_FRONT_WHEEL_DELTA_DISTANCE_ID 		1019
#define ATTR_MOTION_FRONT_WHEEL_DELTA_DISTANCE(_val) 		sacp::Attribute(1019, (float)_val) 		/* 1019 ATTR_MOTION_FRONT_WHEEL_DELTA_DISTANCE   float  RD */
#define ATTR_MOTION_RIGHT_WHEEL_ODOMETER_ID 		1020
#define ATTR_MOTION_RIGHT_WHEEL_ODOMETER(_val) 		sacp::Attribute(1020, (float)_val) 		/* 1020 ATTR_MOTION_RIGHT_WHEEL_ODOMETER         float  RD */
#define ATTR_MOTION_LEFT_WHEEL_ODOMETER_ID 		1021
#define ATTR_MOTION_LEFT_WHEEL_ODOMETER(_val) 		sacp::Attribute(1021, (float)_val) 		/* 1021 ATTR_MOTION_LEFT_WHEEL_ODOMETER          float  RD */
#define ATTR_MOTION_FRONT_WHEEL_ODOMETER_ID 		1022
#define ATTR_MOTION_FRONT_WHEEL_ODOMETER(_val) 		sacp::Attribute(1022, (float)_val) 		/* 1022 ATTR_MOTION_FRONT_WHEEL_ODOMETER         float  RD */
#define ATTR_MOTION_RIGHT_MOTOR_DELTA_DISTANCE_ID 		1023
#define ATTR_MOTION_RIGHT_MOTOR_DELTA_DISTANCE(_val) 		sacp::Attribute(1023, (float)_val) 		/* 1023 ATTR_MOTION_RIGHT_MOTOR_DELTA_DISTANCE   float  RD */
#define ATTR_MOTION_LEFT_MOTOR_DELTA_DISTANCE_ID 		1024
#define ATTR_MOTION_LEFT_MOTOR_DELTA_DISTANCE(_val) 		sacp::Attribute(1024, (float)_val) 		/* 1024 ATTR_MOTION_LEFT_MOTOR_DELTA_DISTANCE    float  RD */
#define ATTR_MOTION_FRONT_MOTOR_DELTA_DISTANCE_ID 		1025
#define ATTR_MOTION_FRONT_MOTOR_DELTA_DISTANCE(_val) 		sacp::Attribute(1025, (float)_val) 		/* 1025 ATTR_MOTION_FRONT_MOTOR_DELTA_DISTANCE   float  RD */
#define ATTR_MOTION_RIGHT_MOTOR_ODOMETER_ID 		1026
#define ATTR_MOTION_RIGHT_MOTOR_ODOMETER(_val) 		sacp::Attribute(1026, (float)_val) 		/* 1026 ATTR_MOTION_RIGHT_MOTOR_ODOMETER         float  RD */
#define ATTR_MOTION_LEFT_MOTOR_ODOMETER_ID 		1027
#define ATTR_MOTION_LEFT_MOTOR_ODOMETER(_val) 		sacp::Attribute(1027, (float)_val) 		/* 1027 ATTR_MOTION_LEFT_MOTOR_ODOMETER          float  RD */
#define ATTR_MOTION_FRONT_MOTOR_ODOMETER_ID 		1028
#define ATTR_MOTION_FRONT_MOTOR_ODOMETER(_val) 		sacp::Attribute(1028, (float)_val) 		/* 1028 ATTR_MOTION_FRONT_MOTOR_ODOMETER         float  RD */
#define ATTR_MOTION_CONTROL_PERIOD_ID 		1029
#define ATTR_MOTION_CONTROL_PERIOD(_val) 		sacp::Attribute(1029, (uint32_t)_val) 		/* 1029 ATTR_MOTION_CONTROL_PERIOD               uint32_t RD */
#define ATTR_MOTION_SET_VELOCITY_ID 		1050
#define ATTR_MOTION_SET_VELOCITY(_val) 		sacp::Attribute(1050, (float)_val) 		/* 1050 ATTR_MOTION_SET_VELOCITY                 float  RW */
#define ATTR_MOTION_SET_STEERING_ANGLE_ID 		1051
#define ATTR_MOTION_SET_STEERING_ANGLE(_val) 		sacp::Attribute(1051, (float)_val) 		/* 1051 ATTR_MOTION_SET_STEERING_ANGLE           float  RW */

/* Total: 32 */


/* 属性组 - fog */

#define ATTR_FOG_GYROSCOPE_X_ID 		1100
#define ATTR_FOG_GYROSCOPE_X(_val) 		sacp::Attribute(1100, (float)_val) 		/* 1100 ATTR_FOG_GYROSCOPE_X                     float  RD */
#define ATTR_FOG_GYROSCOPE_Y_ID 		1101
#define ATTR_FOG_GYROSCOPE_Y(_val) 		sacp::Attribute(1101, (float)_val) 		/* 1101 ATTR_FOG_GYROSCOPE_Y                     float  RD */
#define ATTR_FOG_GYROSCOPE_Z_ID 		1102
#define ATTR_FOG_GYROSCOPE_Z(_val) 		sacp::Attribute(1102, (float)_val) 		/* 1102 ATTR_FOG_GYROSCOPE_Z                     float  RD */
#define ATTR_FOG_ACCELEROMETER_X_ID 		1103
#define ATTR_FOG_ACCELEROMETER_X(_val) 		sacp::Attribute(1103, (float)_val) 		/* 1103 ATTR_FOG_ACCELEROMETER_X                 float  RD */
#define ATTR_FOG_ACCELEROMETER_Y_ID 		1104
#define ATTR_FOG_ACCELEROMETER_Y(_val) 		sacp::Attribute(1104, (float)_val) 		/* 1104 ATTR_FOG_ACCELEROMETER_Y                 float  RD */
#define ATTR_FOG_ACCELEROMETER_Z_ID 		1105
#define ATTR_FOG_ACCELEROMETER_Z(_val) 		sacp::Attribute(1105, (float)_val) 		/* 1105 ATTR_FOG_ACCELEROMETER_Z                 float  RD */
#define ATTR_FOG_EULAR_ROLL_ID 		1106
#define ATTR_FOG_EULAR_ROLL(_val) 		sacp::Attribute(1106, (float)_val) 		/* 1106 ATTR_FOG_EULAR_ROLL                      float  RD */
#define ATTR_FOG_EULAR_YAW_ID 		1107
#define ATTR_FOG_EULAR_YAW(_val) 		sacp::Attribute(1107, (float)_val) 		/* 1107 ATTR_FOG_EULAR_YAW                       float  RD */
#define ATTR_FOG_EULAR_PITCH_ID 		1108
#define ATTR_FOG_EULAR_PITCH(_val) 		sacp::Attribute(1108, (float)_val) 		/* 1108 ATTR_FOG_EULAR_PITCH                     float  RD */
#define ATTR_FOG_QUATERNION_A_ID 		1109
#define ATTR_FOG_QUATERNION_A(_val) 		sacp::Attribute(1109, (float)_val) 		/* 1109 ATTR_FOG_QUATERNION_A                    float  RD */
#define ATTR_FOG_QUATERNION_B_ID 		1110
#define ATTR_FOG_QUATERNION_B(_val) 		sacp::Attribute(1110, (float)_val) 		/* 1110 ATTR_FOG_QUATERNION_B                    float  RD */
#define ATTR_FOG_QUATERNION_C_ID 		1111
#define ATTR_FOG_QUATERNION_C(_val) 		sacp::Attribute(1111, (float)_val) 		/* 1111 ATTR_FOG_QUATERNION_C                    float  RD */
#define ATTR_FOG_QUATERNION_D_ID 		1112
#define ATTR_FOG_QUATERNION_D(_val) 		sacp::Attribute(1112, (float)_val) 		/* 1112 ATTR_FOG_QUATERNION_D                    float  RD */
#define ATTR_FOG_ACTIVED_ID 		1113
#define ATTR_FOG_ACTIVED(_val) 		sacp::Attribute(1113, (bool)_val) 		/* 1113 ATTR_FOG_ACTIVED                         bool   RD */
#define ATTR_FOG_VERSION_ID 		1114
#define ATTR_FOG_VERSION(_val) 		sacp::Attribute(1114, _val) 		/* 1114 ATTR_FOG_VERSION                         octet  RD */
#define ATTR_FOG_SERIAL_NUMBER_ID 		1115
#define ATTR_FOG_SERIAL_NUMBER(_val) 		sacp::Attribute(1115, _val) 		/* 1115 ATTR_FOG_SERIAL_NUMBER                   octet  RD */
#define ATTR_FOG_WORK_MODE_ID 		1116
#define ATTR_FOG_WORK_MODE(_val) 		sacp::Attribute(1116, (uint32_t)_val) 		/* 1116 ATTR_FOG_WORK_MODE                       uint32_t RD */
#define ATTR_FOG_SELF_CHECKING_ID 		1117
#define ATTR_FOG_SELF_CHECKING(_val) 		sacp::Attribute(1117, (uint32_t)_val) 		/* 1117 ATTR_FOG_SELF_CHECKING                   uint32_t RD */
#define ATTR_FOG_TMPERATURE_ID 		1118
#define ATTR_FOG_TMPERATURE(_val) 		sacp::Attribute(1118, (float)_val) 		/* 1118 ATTR_FOG_TMPERATURE                      float  RD */
#define ATTR_FOG_MANAUAL_CALIBRATION_ID 		1119
#define ATTR_FOG_MANAUAL_CALIBRATION(_val) 		sacp::Attribute(1119, (bool)_val) 		/* 1119 ATTR_FOG_MANAUAL_CALIBRATION             bool   RD */

/* Total: 20 */


// 总共 416 Attributes 

#endif // __ROBOT_N1_ATTRIBUTES_H__ 
