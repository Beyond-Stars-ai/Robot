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

//=========================== 全局变量 ===========================//

/**
 * @brief 底盘转动变化量（编码器值/20ms）
 * 
 * CalTask每20ms更新：
 *   g_chassis_delta = -delta_yaw * 22.756f
 * 
 * @note 使用消费式读取：CanTask读取后清零，确保每个delta只用一次
 */
float g_chassis_delta = 0.0f;
uint8_t g_chassis_delta_ready = 0;

//=========================== 初始化 ===========================//

void Gimbal_Control_Init(void)
{
    Virtual_Yaw_Init();

    // Pitch
    PID_PositionStructureInit(&Pitch_PositionPID, 4074.0f);
    PID_PositionSetParameter(&Pitch_PositionPID, 0.5f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&Pitch_PositionPID, -400.0f, 400.0f);
    PID_PositionSetEkRange(&Pitch_PositionPID, -3.0f, 3.0f);

    PID_PositionStructureInit(&Pitch_SpeedPID, 0.0f);
    PID_PositionSetParameter(&Pitch_SpeedPID, 50.0f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&Pitch_SpeedPID, -20000.0f, 20000.0f);
    PID_PositionSetEkRange(&Pitch_SpeedPID, -3.0f, 3.0f);

    // SmallYaw
    PID_PositionStructureInit(&SmallYaw_PositionPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_PositionPID, 0.5f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&SmallYaw_PositionPID, -4000.0f, 4000.0f);
    PID_PositionSetEkRange(&SmallYaw_PositionPID, -5.0f, 5.0f);

    PID_PositionStructureInit(&SmallYaw_SpeedPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_SpeedPID, 20.0f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&SmallYaw_SpeedPID, -10000.0f, 10000.0f);
    PID_PositionSetEkRange(&SmallYaw_SpeedPID, -3.0f, 3.0f);

    // BigYaw（保守参数，因为虚拟RC已经提供了前馈）
    PID_PositionStructureInit(&BigYaw_PositionPID, 0.0f);
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
    Pitch_PositionPID.Need_Value -= PITCH_RC_SENS * global_rc_control.rc.ch[PITCH_RC_CHANNEL];

    if (Pitch_PositionPID.Need_Value > PITCH_LIMIT_UP)
        Pitch_PositionPID.Need_Value = PITCH_LIMIT_UP;
    else if (Pitch_PositionPID.Need_Value < PITCH_LIMIT_DOWN)
        Pitch_PositionPID.Need_Value = PITCH_LIMIT_DOWN;

    PID_PositionCalc(&Pitch_PositionPID, Can1_M6020_MotorStatus[1].Position);

    PID_PositionSetNeedValue(&Pitch_SpeedPID, Pitch_PositionPID.OUT);
    PID_PositionCalc(&Pitch_SpeedPID, Can1_M6020_MotorStatus[1].Speed);

    Motor_6020_Voltage1(0, (int16_t)Pitch_SpeedPID.OUT, 0, 0, &hcan1);
}

//=========================== Yaw控制 ===========================//

void Gimbal_Yaw_Control(void)
{
    // 获取实际编码
    float real_small = (float)Can2_M6020_MotorStatus[1].Angle;
    float real_big = (float)Can2_M6020_MotorStatus[0].Angle;
    float real_small_speed = (float)Can2_M6020_MotorStatus[1].Speed;
    float real_big_speed = (float)Can2_M6020_MotorStatus[0].Speed;
    
    // 消费式读取：有新数据就用，用完清零避免重复计算
    float chassis_delta = 0.0f;
    if (g_chassis_delta_ready) {
        chassis_delta = g_chassis_delta;
        g_chassis_delta = 0.0f;        // 清零，确保20ms内只用一次
        g_chassis_delta_ready = 0;
    }
    
    // 统一更新（底盘delta传入，10ms内可能为0）
    Virtual_Yaw_Update(global_rc_control.rc.ch[2], 
                       chassis_delta,
                       real_small, 
                       real_big);
    
    // 获取目标值
    float target_small = Virtual_Yaw_GetTarget_Small();
    float target_big = Virtual_Yaw_GetTarget_Big();
    
    // SmallYaw控制
    PID_PositionSetNeedValue(&SmallYaw_PositionPID, target_small);
    PID_PositionCalc_Encoder(&SmallYaw_PositionPID, real_small);
    
    PID_PositionSetNeedValue(&SmallYaw_SpeedPID, SmallYaw_PositionPID.OUT);
    PID_PositionCalc(&SmallYaw_SpeedPID, real_small_speed);
    
    // BigYaw控制
    PID_PositionSetNeedValue(&BigYaw_PositionPID, target_big);
    PID_PositionCalc_Encoder(&BigYaw_PositionPID, real_big);
    
    PID_PositionSetNeedValue(&BigYaw_SpeedPID, BigYaw_PositionPID.OUT);
    PID_PositionCalc(&BigYaw_SpeedPID, real_big_speed);
    
    // 发送输出
    Motor_6020_Voltage1((int16_t)BigYaw_SpeedPID.OUT, 
                        (int16_t)SmallYaw_SpeedPID.OUT, 
                        0, 0, &hcan2);
}

//=========================== 总控制循环 ===========================//

void Gimbal_Control_Loop(void)
{
    Gimbal_Pitch_Control();
    Gimbal_Yaw_Control();
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
