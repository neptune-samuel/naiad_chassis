
#ifndef __ROBOT_N1_GROUPS_H__
#define __ROBOT_N1_GROUPS_H__

/**
 * @brief 上报组标识 
 * 
 */
enum REPORT_GROUP  
{
    /// 系统信息
    REPORT_GROUP_SYSTEM_INFO = 0,
    /// 系统状态
    REPORT_GROUP_SYSTEM_STATE,

    /// 电源盒基址
    _REPORT_POWERBOX_BASE = 20,
    /// 泵压力盒基址
    _REPORT_PUMPBOX_BASE = 30,
    /// FOG盒基本信息
    _REPORT_FOGBOX_BASE = 40,
    /// 顶出盒基本信息
    _REPORT_PUSHBOX_BASE = 50,
    /// LED照明基址
    _REPORT_LEDLIGHT_BASE = 60,
    /// 升降器基址
    _REPORT_LIFTER_BASE = 100,

    // 添加更多的上报组在这里 140 开始
};


/**
 * @brief 设备通用上报组
 * 
 */
enum DEVICE_COMMON_REPORT_INDEX
{
    _DEVICE_INFO = 0,
    _ADMIN_STATUS = 1,
    _RUNNING_STATE = 2,
    _DEVICE_CONFIG = 3,
};

/// 通用组ID
#define REPORT_ID(_base, _index) ((_base) + (_index))

/// 组ID
#define ATTR_GROUP_ID     100

#endif // __ROBOT_N1_GROUPS_H__ 

