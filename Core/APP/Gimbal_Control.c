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

//=========================== 工具函数 ===========================//

static float limit_value(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return value;
    return value;
}

//=========================== 初始化 ===========================//

void Gimbal_Control_Init(void)
{
    //---------- 虚拟坐标层初始化 ----------
    Virtual_Yaw_Init();

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
    PID_PositionStructureInit(&SmallYaw_PositionPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_PositionPID, 0.5f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&SmallYaw_PositionPID, -4000.0f, 4000.0f);
    PID_PositionSetEkRange(&SmallYaw_PositionPID, -5.0f, 5.0f);

    PID_PositionStructureInit(&SmallYaw_SpeedPID, 0.0f);
    PID_PositionSetParameter(&SmallYaw_SpeedPID, 20.0f, 0.0f, 0.0f);
    PID_PositionSetOUTRange(&SmallYaw_SpeedPID, -10000.0f, 10000.0f);
    PID_PositionSetEkRange(&SmallYaw_SpeedPID, -3.0f, 3.0f);

    //---------- BigYaw初始化 ----------
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

//=========================== Yaw控制（只从虚拟层取值）===========================//

void Gimbal_Yaw_Control(void)
{
    //---------- 1. 获取实际编码（给虚拟层）----------
    float real_small = (float)Can2_M6020_MotorStatus[1].Angle;
    float real_big = (float)Can2_M6020_MotorStatus[0].Angle;
    float real_small_speed = (float)Can2_M6020_MotorStatus[1].Speed;
    float real_big_speed = (float)Can2_M6020_MotorStatus[0].Speed;

    //---------- 1.5 获取陀螺仪Yaw角 ----------
    extern BMI088_Init_typedef Can_BMI088_Data;
    float gyro_yaw = Can_BMI088_Data.Yaw;
    
    //---------- 2. 更新虚拟坐标层 ----------
    // 遥控器值传递给虚拟层，虚拟层处理所有坐标转换
    Virtual_Yaw_Update(global_rc_control.rc.ch[2], real_small, real_big, gyro_yaw);
    
    //---------- 3. 从虚拟层获取目标（隔离！只取结果）----------
    float target_small = Virtual_Yaw_GetTarget_Small();
    float target_big = Virtual_Yaw_GetTarget_Big();
    
    //---------- 4. SmallYaw控制 ----------
    PID_PositionSetNeedValue(&SmallYaw_PositionPID, target_small);
    PID_PositionCalc_Encoder(&SmallYaw_PositionPID, real_small);
    
    PID_PositionSetNeedValue(&SmallYaw_SpeedPID, SmallYaw_PositionPID.OUT);
    PID_PositionCalc(&SmallYaw_SpeedPID, real_small_speed);
    
    //---------- 5. BigYaw控制 ----------
    PID_PositionSetNeedValue(&BigYaw_PositionPID, target_big);
    PID_PositionCalc_Encoder(&BigYaw_PositionPID, real_big);
    
    PID_PositionSetNeedValue(&BigYaw_SpeedPID, BigYaw_PositionPID.OUT);
    PID_PositionCalc(&BigYaw_SpeedPID, real_big_speed);
    
    //---------- 6. 发送输出 ----------
    // BigYaw(ID 0x205, 第1路), SmallYaw(ID 0x206, 第2路)
    Motor_6020_Voltage1((int16_t)BigYaw_SpeedPID.OUT, (int16_t)SmallYaw_SpeedPID.OUT, 0, 0, &hcan2);
}

//=========================== 总控制循环 ===========================//

void Gimbal_Control_Loop(void)
{
    // Pitch控制
    Gimbal_Pitch_Control();
    
    // Yaw控制（虚拟层隔离）
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
