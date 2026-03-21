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
 * @brief 底盘转动变化量（编码器值/1ms）
 * 
 * CalTask每1ms更新（插值后）
 */
float g_chassis_delta = 0.0f;

//=========================== 初始化 ===========================//

void Gimbal_Control_Init(void)
{
    Virtual_Yaw_Init();

    // Pitch - 位置环
    PID_PositionStructureInit(&Pitch_PositionPID, 4074.0f);
    PID_PositionSetParameter(&Pitch_PositionPID, 0.5f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&Pitch_PositionPID, -400.0f, 400.0f);
    PID_PositionSetEkRange(&Pitch_PositionPID, -3.0f, 3.0f);

    // Pitch - 速度环（1ms适配：降低Kp到25）
    PID_PositionStructureInit(&Pitch_SpeedPID, 0.0f);
    PID_PositionSetParameter(&Pitch_SpeedPID, 25.0f, 0.0f, 0.0f);  // 原来是50
    PID_PositionSetOUTRange(&Pitch_SpeedPID, -20000.0f, 20000.0f);
    PID_PositionSetEkRange(&Pitch_SpeedPID, -3.0f, 3.0f);

    // SmallYaw - 位置环
    PID_PositionStructureInit(&SmallYaw_PositionPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_PositionPID, 0.5f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&SmallYaw_PositionPID, -4000.0f, 4000.0f);
    PID_PositionSetEkRange(&SmallYaw_PositionPID, -5.0f, 5.0f);

    // SmallYaw - 速度环（1ms适配：降低Kp到10）
    PID_PositionStructureInit(&SmallYaw_SpeedPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_SpeedPID, 10.0f, 0.0f, 0.0f);  // 原来是20
    PID_PositionSetOUTRange(&SmallYaw_SpeedPID, -10000.0f, 10000.0f);
    PID_PositionSetEkRange(&SmallYaw_SpeedPID, -3.0f, 3.0f);

    // BigYaw - 位置环（可以适当加大，响应更快）
    PID_PositionStructureInit(&BigYaw_PositionPID, 0.0f);
    PID_PositionSetParameter(&BigYaw_PositionPID, 1.0f, 0.0f, 0.0f);  // 原来是0.8
    PID_PositionSetOUTRange(&BigYaw_PositionPID, -8000.0f, 8000.0f);
    PID_PositionSetEkRange(&BigYaw_PositionPID, -5.0f, 5.0f);

    // BigYaw - 速度环（1ms适配：降低Kp到15）
    PID_PositionStructureInit(&BigYaw_SpeedPID, 0.0f);
    PID_PositionSetParameter(&BigYaw_SpeedPID, 15.0f, 0.0f, 0.0f);  // 原来是30
    PID_PositionSetOUTRange(&BigYaw_SpeedPID, -25000.0f, 25000.0f);
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

//=========================== Yaw控制（1ms）===========================//

void Gimbal_Yaw_Control(void)
{
    // 获取实际编码
    float real_small = (float)Can2_M6020_MotorStatus[1].Angle;
    float real_big = (float)Can2_M6020_MotorStatus[0].Angle;
    float real_small_speed = (float)Can2_M6020_MotorStatus[1].Speed;
    float real_big_speed = (float)Can2_M6020_MotorStatus[0].Speed;
    
    // 1ms更新（直接使用g_chassis_delta，CalTask每1ms插值更新）
    Virtual_Yaw_Update(global_rc_control.rc.ch[2], 
                       g_chassis_delta,
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
