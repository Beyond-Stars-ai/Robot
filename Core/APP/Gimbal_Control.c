#include "Gimbal_Control.h"
#include "can.h"
#include <math.h>

//=========================== PID定义 ===========================//

PID_PositionInitTypedef Pitch_PositionPID;
PID_PositionInitTypedef Pitch_SpeedPID;
PID_PositionInitTypedef SmallYaw_PositionPID;
PID_PositionInitTypedef SmallYaw_SpeedPID;
PID_PositionInitTypedef BigYaw_PositionPID;
PID_PositionInitTypedef BigYaw_SpeedPID;

//=========================== 内部变量 ===========================//

static float virtual_SmallYaw = 0.0f;       // SmallYaw虚拟坐标（0=机械中值，向右为正）
static int16_t big_yaw_output = 0;          // BigYaw输出缓存

//=========================== 工具函数 ===========================//

static float limit_value(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

//=========================== 初始化 ===========================//

void Gimbal_Control_Init(void)
{
    //---------- Pitch初始化 ----------
    PID_PositionStructureInit(&Pitch_PositionPID, 4074.0f);
    PID_PositionSetParameter(&Pitch_PositionPID, 0.5f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&Pitch_PositionPID, -400.0f, 400.0f);
    PID_PositionSetEkRange(&Pitch_PositionPID, -3.0f, 3.0f);

    PID_PositionStructureInit(&Pitch_SpeedPID, 0.0f);
    PID_PositionSetParameter(&Pitch_SpeedPID, 50.0f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&Pitch_SpeedPID, -20000.0f, 20000.0f);
    PID_PositionSetEkRange(&Pitch_SpeedPID, -3.0f, 3.0f);

    //---------- SmallYaw初始化 ----------
    virtual_SmallYaw = 0.0f;

    PID_PositionStructureInit(&SmallYaw_PositionPID, (float)origin_SmallYaw_count);
    PID_PositionSetParameter(&SmallYaw_PositionPID, 0.5f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&SmallYaw_PositionPID, -4000.0f, 4000.0f);
    PID_PositionSetEkRange(&SmallYaw_PositionPID, -5.0f, 5.0f);

    PID_PositionStructureInit(&SmallYaw_SpeedPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_SpeedPID, 20.0f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&SmallYaw_SpeedPID, -10000.0f, 10000.0f);
    PID_PositionSetEkRange(&SmallYaw_SpeedPID, -3.0f, 3.0f);

    //---------- BigYaw初始化 ----------
    big_yaw_output = 0;

    PID_PositionStructureInit(&BigYaw_PositionPID, (float)origin_BigYaw_count);
    PID_PositionSetParameter(&BigYaw_PositionPID, 0.8f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&BigYaw_PositionPID, -6000.0f, 6000.0f);
    PID_PositionSetEkRange(&BigYaw_PositionPID, -5.0f, 5.0f);

    PID_PositionStructureInit(&BigYaw_SpeedPID, 0.0f);
    PID_PositionSetParameter(&BigYaw_SpeedPID, 30.0f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&BigYaw_SpeedPID, -20000.0f, 20000.0f);
    PID_PositionSetEkRange(&BigYaw_SpeedPID, -3.0f, 3.0f);
}

//=========================== Pitch控制 ===========================//

void Gimbal_Pitch_Control(void)
{
    // ============更新位置目标============
    Pitch_PositionPID.Need_Value -= PITCH_RC_SENS * global_rc_control.rc.ch[PITCH_RC_CHANNEL];

    // 限幅
    if (Pitch_PositionPID.Need_Value > PITCH_LIMIT_UP)
        Pitch_PositionPID.Need_Value = PITCH_LIMIT_UP;
    else if (Pitch_PositionPID.Need_Value < PITCH_LIMIT_DOWN)
        Pitch_PositionPID.Need_Value = PITCH_LIMIT_DOWN;

    // ============位置环计算============
    PID_PositionCalc(&Pitch_PositionPID, Can1_M6020_MotorStatus[1].Position);

    // ============速度环计算============
    PID_PositionSetNeedValue(&Pitch_SpeedPID, Pitch_PositionPID.OUT);
    PID_PositionCalc(&Pitch_SpeedPID, Can1_M6020_MotorStatus[1].Speed);

    // ============发送输出============
    Motor_6020_Voltage1(0, (int16_t)Pitch_SpeedPID.OUT, 0, 0, &hcan1);
}

//=========================== SmallYaw控制 ===========================//

void Gimbal_SmallYaw_Control(void)
{
    // 获取当前编码器和速度
    float current_angle = (float)Can2_M6020_MotorStatus[1].Angle;
    float current_speed = (float)Can2_M6020_MotorStatus[1].Speed;

    // ============ 1. 遥控器更新虚拟坐标 ============
    float delta = SMALLYAW_RC_SENS * global_rc_control.rc.ch[SMALLYAW_RC_CHANNEL];
    virtual_SmallYaw += delta;

    // 软限幅
    virtual_SmallYaw = limit_value(virtual_SmallYaw, -SMALLYAW_VIRTUAL_LIMIT, SMALLYAW_VIRTUAL_LIMIT);

    // ============ 2. 虚拟坐标 → 实际目标编码 ============
    // SmallYaw: 向右转编码减小，目标 = 中值 - 虚拟值
    float target_angle = (float)origin_SmallYaw_count - virtual_SmallYaw;

    // 归一化到0-8191
    while (target_angle < 0.0f) target_angle += 8192.0f;
    while (target_angle >= 8192.0f) target_angle -= 8192.0f;

    // ============ 3. 更新状态变量 ============
    now_SmallYaw_count = (int16_t)current_angle;
    error_SmallYaw_count = (int16_t)(target_angle - current_angle);

    // ============ 4. 位置环计算 ============
    PID_PositionSetNeedValue(&SmallYaw_PositionPID, target_angle);
    PID_PositionCalc_Encoder(&SmallYaw_PositionPID, current_angle);

    // ============ 5. 速度环计算 ============
    PID_PositionSetNeedValue(&SmallYaw_SpeedPID, SmallYaw_PositionPID.OUT);
    PID_PositionCalc(&SmallYaw_SpeedPID, current_speed);
    
    // 输出保存在SmallYaw_SpeedPID.OUT中，由Control_Loop统一发送
}

//=========================== BigYaw控制（跟随SmallYaw）===========================//

void Gimbal_BigYaw_Control(void)
{
    // 获取当前编码器和速度
    float current_angle = (float)Can2_M6020_MotorStatus[0].Angle;
    float current_speed = (float)Can2_M6020_MotorStatus[0].Speed;

    // ============ 目标计算：跟随SmallYaw虚拟坐标 ============
    // BigYaw: 向右转编码增加，目标 = 中值 + 虚拟值
    float target_angle = (float)origin_BigYaw_count + virtual_SmallYaw;

    // 归一化到0-8191
    while (target_angle < 0.0f) target_angle += 8192.0f;
    while (target_angle >= 8192.0f) target_angle -= 8192.0f;

    // ============ 更新状态变量 ============
    now_BigYaw_count = (int16_t)current_angle;
    error_BigYaw_count = (int16_t)(target_angle - current_angle);

    // ============ 位置环计算 ============
    PID_PositionSetNeedValue(&BigYaw_PositionPID, target_angle);
    PID_PositionCalc_Encoder(&BigYaw_PositionPID, current_angle);

    // ============ 速度环计算 ============
    PID_PositionSetNeedValue(&BigYaw_SpeedPID, BigYaw_PositionPID.OUT);
    PID_PositionCalc(&BigYaw_SpeedPID, current_speed);

    // ============ 保存输出 ============
    big_yaw_output = (int16_t)BigYaw_SpeedPID.OUT;
}

//=========================== 总控制循环 ===========================//

void Gimbal_Control_Loop(void)
{
    // Pitch控制（CAN1单独发送）
    Gimbal_Pitch_Control();
    
    // Yaw轴控制（先计算两个轴）
    Gimbal_SmallYaw_Control();
    Gimbal_BigYaw_Control();
    
    // 合并发送：BigYaw(ID 0x205, 第1路), SmallYaw(ID 0x206, 第2路)
    Motor_6020_Voltage1(big_yaw_output, (int16_t)SmallYaw_SpeedPID.OUT, 0, 0, &hcan2);
}

//=========================== PID调试接口 ===========================//

void Gimbal_Pitch_SetPosPID(float kp, float ki, float kd)
{
    PID_PositionSetParameter(&Pitch_PositionPID, kp, ki, kd);
}

void Gimbal_Pitch_SetSpeedPID(float kp, float ki, float kd)
{
    PID_PositionSetParameter(&Pitch_SpeedPID, kp, ki, kd);
}

void Gimbal_SmallYaw_SetPosPID(float kp, float ki, float kd)
{
    PID_PositionSetParameter(&SmallYaw_PositionPID, kp, ki, kd);
}

void Gimbal_SmallYaw_SetSpeedPID(float kp, float ki, float kd)
{
    PID_PositionSetParameter(&SmallYaw_SpeedPID, kp, ki, kd);
}

void Gimbal_BigYaw_SetPosPID(float kp, float ki, float kd)
{
    PID_PositionSetParameter(&BigYaw_PositionPID, kp, ki, kd);
}

void Gimbal_BigYaw_SetSpeedPID(float kp, float ki, float kd)
{
    PID_PositionSetParameter(&BigYaw_SpeedPID, kp, ki, kd);
}
