
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
    /// 传感器 深度传感器
    REPORT_GROUP_DEPTH_SENSOR,
    /// 运控状态
    REPORT_GROUP_MOTION_STATE1,
    /// 运控状态
    REPORT_GROUP_MOTION_STATE2,

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
    
    /// 电机基址
    _REPORT_MOTOR_BASE = 140,

    // 添加更多的上报组在这里 200 开始
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

/**
 * @brief 电机上报组
 * 
 */
enum MOTOR_REPORT_INDEX
{
    _MOTOR_INFO1 = 0,
    _MOTOR_INFO2 = 1,    
    _MOTOR_ADMIN_STATUS = 2,
    _MOTOR_STATE = 3,
};


/// 通用组ID
#define REPORT_ID(_base, _index) ((_base) + (_index))

/// 组ID
#define ATTR_GROUP_ID     100

#endif // __ROBOT_N1_GROUPS_H__ 

