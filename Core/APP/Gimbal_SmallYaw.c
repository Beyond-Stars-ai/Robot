#include "Gimbal_SmallYaw.h"
#include "can.h"
#include <math.h>

//=========================== PID定义 ===========================//

PID_PositionInitTypedef SmallYaw_PositionPID;
PID_PositionInitTypedef SmallYaw_SpeedPID;

//=========================== 内部变量 ===========================//

static float virtual_SmallYaw = 0.0f;       // 虚拟坐标（0=机械中值，向右为正）

//=========================== 工具函数 ===========================//

static float limit_value(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

//=========================== 初始化 ===========================//

void Gimbal_SmallYaw_Init(void)
{
    // 虚拟坐标归零（对应机械中值）
    virtual_SmallYaw = 0.0f;
    
    // 位置环PID - 目标值将在控制循环中动态计算
    PID_PositionStructureInit(&SmallYaw_PositionPID, (float)origin_SmallYaw_count);
    PID_PositionSetParameter(&SmallYaw_PositionPID, 0.5f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&SmallYaw_PositionPID, -4000.0f, 4000.0f);
    PID_PositionSetEkRange(&SmallYaw_PositionPID, -5.0f, 5.0f);
    
    // 速度环PID
    PID_PositionStructureInit(&SmallYaw_SpeedPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_SpeedPID, 20.0f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&SmallYaw_SpeedPID, -10000.0f, 10000.0f);
    PID_PositionSetEkRange(&SmallYaw_SpeedPID, -3.0f, 3.0f);
}

//=========================== 控制循环 ===========================//

void Gimbal_SmallYaw_Control(void)
{
    // 获取当前实际编码值（0-8191）
    float current_angle = (float)Can2_M6020_MotorStatus[SMALLYAW_MOTOR_IDX].Angle;
    float current_speed = (float)Can2_M6020_MotorStatus[SMALLYAW_MOTOR_IDX].Speed;
    
    // ============ 1. 遥控器更新虚拟坐标 ============
    // ch[2]中位1024，向前打杆（向右）值增大
    int16_t rc_value = global_rc_control.rc.ch[SMALLYAW_RC_CHANNEL];
    float delta = (rc_value - 1024) * SMALLYAW_RC_SENS;
    virtual_SmallYaw += delta;
    
    // 软限幅
    virtual_SmallYaw = limit_value(virtual_SmallYaw, 
        -SMALLYAW_VIRTUAL_LIMIT, SMALLYAW_VIRTUAL_LIMIT);
    
    // ============ 2. 虚拟坐标 → 实际目标编码 ============
    // SmallYaw: 向右转编码减小，所以 目标 = 中值 - 虚拟值
    float target_angle = (float)origin_SmallYaw_count - virtual_SmallYaw;
    
    // 归一化到0-8191范围
    while (target_angle < 0) target_angle += 8192.0f;
    while (target_angle >= 8192.0f) target_angle -= 8192.0f;
    
    // ============ 3. 更新状态变量（供调试）============
    now_SmallYaw_count = (int16_t)current_angle;
    error_SmallYaw_count = (int16_t)(target_angle - current_angle);
    
    // ============ 4. 位置环计算 ============
    PID_PositionSetNeedValue(&SmallYaw_PositionPID, target_angle);
    PID_PositionCalc_Encoder(&SmallYaw_PositionPID, current_angle);
    
    // ============ 5. 速度环计算 ============
    PID_PositionSetNeedValue(&SmallYaw_SpeedPID, SmallYaw_PositionPID.OUT);
    PID_PositionCalc(&SmallYaw_SpeedPID, current_speed);
    
    // ============ 6. 发送输出 ============
    // SmallYaw是ID 0x206，对应Voltage1的第2路
    Motor_6020_Voltage1(0, (int16_t)SmallYaw_SpeedPID.OUT, 0, 0, &hcan2);
}
