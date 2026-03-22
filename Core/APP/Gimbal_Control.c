#include "Gimbal_Control.h"
#include "can.h"
#include <math.h>

//=========================== PID定义 ===========================//

// Pitch
PID_PositionInitTypedef Pitch_PositionPID;
PID_PositionInitTypedef Pitch_SpeedPID;

// BigYaw：编码器位置环 + 速度环
PID_PositionInitTypedef BigYaw_PositionPID;
PID_PositionInitTypedef BigYaw_SpeedPID;

// SmallYaw：编码器位置环 + 速度环
PID_PositionInitTypedef SmallYaw_PositionPID;
PID_PositionInitTypedef SmallYaw_SpeedPID;

//=========================== 全局变量 ===========================//

float g_chassis_delta = 0.0f;        // 1ms底盘变化量（CalTask计算）
float g_chassis_delta_10ms = 0.0f;   // 10ms总变化量

//=========================== 初始化 ===========================//

void Gimbal_Control_Init(void)
{
    Virtual_Yaw_Init();

    // Pitch（保持不变）
    PID_PositionStructureInit(&Pitch_PositionPID, 4074.0f);
    PID_PositionSetParameter(&Pitch_PositionPID, 0.5f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&Pitch_PositionPID, -400.0f, 400.0f);
    PID_PositionSetEkRange(&Pitch_PositionPID, -3.0f, 3.0f);

    PID_PositionStructureInit(&Pitch_SpeedPID, 0.0f);
    PID_PositionSetParameter(&Pitch_SpeedPID, 25.0f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&Pitch_SpeedPID, -20000.0f, 20000.0f);

    // BigYaw：编码器双环（恢复原来架构）
    PID_PositionStructureInit(&BigYaw_PositionPID, 0.0f);
    PID_PositionSetParameter(&BigYaw_PositionPID, 0.8f, 0.0f, 0.0f);  // 位置环Kp=0.8
    PID_PositionSetOUTRange(&BigYaw_PositionPID, -6000.0f, 6000.0f);
    PID_PositionSetEkRange(&BigYaw_PositionPID, -5.0f, 5.0f);
    
    PID_PositionStructureInit(&BigYaw_SpeedPID, 0.0f);
    PID_PositionSetParameter(&BigYaw_SpeedPID, 30.0f, 0.0f, 0.0f);  // 速度环Kp=30
    PID_PositionSetOUTRange(&BigYaw_SpeedPID, -20000.0f, 20000.0f);
    
    // SmallYaw：编码器双环
    PID_PositionStructureInit(&SmallYaw_PositionPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_PositionPID, 0.5f, 0.0f, 0.0f);  // 位置环Kp=0.5
    PID_PositionSetOUTRange(&SmallYaw_PositionPID, -4000.0f, 4000.0f);
    PID_PositionSetEkRange(&SmallYaw_PositionPID, -5.0f, 5.0f);
    
    PID_PositionStructureInit(&SmallYaw_SpeedPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_SpeedPID, 20.0f, 0.0f, 0.0f);  // 速度环Kp=20
    PID_PositionSetOUTRange(&SmallYaw_SpeedPID, -10000.0f, 10000.0f);
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

//=========================== Yaw控制（编码器 + g_chassis_delta补偿）===========================//

void Gimbal_Yaw_Control(void)
{
    // 获取实际值
    float real_small = (float)Can2_M6020_MotorStatus[1].Angle;
    float real_big = (float)Can2_M6020_MotorStatus[0].Angle;
    float real_small_speed = (float)Can2_M6020_MotorStatus[1].Speed;
    float real_big_speed = (float)Can2_M6020_MotorStatus[0].Speed;
    
    // 更新目标（使用g_chassis_delta进行底盘补偿）
    Virtual_Yaw_Update(global_rc_control.rc.ch[2], 
                       g_chassis_delta,  // CalTask计算的底盘补偿
                       real_small, 
                       real_big);
    
    // 获取目标编码器值
    float target_small = Virtual_Yaw_GetTarget_Small();
    float target_big = Virtual_Yaw_GetTarget_Big();
    
    //---------- SmallYaw控制 ----------
    PID_PositionSetNeedValue(&SmallYaw_PositionPID, target_small);
    PID_PositionCalc_Encoder(&SmallYaw_PositionPID, real_small);
    
    PID_PositionSetNeedValue(&SmallYaw_SpeedPID, SmallYaw_PositionPID.OUT);
    PID_PositionCalc(&SmallYaw_SpeedPID, real_small_speed);
    
    //---------- BigYaw控制 ----------
    PID_PositionSetNeedValue(&BigYaw_PositionPID, target_big);
    PID_PositionCalc_Encoder(&BigYaw_PositionPID, real_big);
    
    PID_PositionSetNeedValue(&BigYaw_SpeedPID, BigYaw_PositionPID.OUT);
    PID_PositionCalc(&BigYaw_SpeedPID, real_big_speed);
    
    //---------- 发送输出 ----------
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
