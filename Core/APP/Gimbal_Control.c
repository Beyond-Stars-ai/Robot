#include "Gimbal_Control.h"
#include "can.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

//=========================== PID定义 ===========================//

PID_PositionInitTypedef Pitch_PositionPID;
PID_PositionInitTypedef Pitch_SpeedPID;
PID_PositionInitTypedef SmallYaw_PositionPID;
PID_PositionInitTypedef SmallYaw_SpeedPID;

//=========================== 内部变量 ===========================//

static float virtual_SmallYaw = 0.0f;       // SmallYaw虚拟坐标（0=机械中值，向右为正）

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

    // 位置环PID - 使用编码器循环计算
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

//=========================== Pitch控制 ===========================//

void Gimbal_Pitch_Control(void)
{
    // ============更新位置目标（遥控器模式）============
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
    // ch[2]已偏移（中位=0），向右打杆为正，向左为负
    float delta = SMALLYAW_RC_SENS * global_rc_control.rc.ch[SMALLYAW_RC_CHANNEL];
    virtual_SmallYaw += delta;

    // 软限幅
    virtual_SmallYaw = limit_value(virtual_SmallYaw, -SMALLYAW_VIRTUAL_LIMIT, SMALLYAW_VIRTUAL_LIMIT);

    // ============ 2. 虚拟坐标 → 实际目标编码 ============
    // SmallYaw机械向性：向右转编码减小，所以 目标 = 中值 - 虚拟值
    float target_angle = (float)origin_SmallYaw_count - virtual_SmallYaw;

    // 归一化到0-8191
    while (target_angle < 0.0f) target_angle += 8192.0f;
    while (target_angle >= 8192.0f) target_angle -= 8192.0f;

    // ============ 3. 更新状态变量（供调试）============
    now_SmallYaw_count = (int16_t)current_angle;
    error_SmallYaw_count = (int16_t)(target_angle - current_angle);

    // ============ 4. 位置环计算（编码器循环模式）============
    PID_PositionSetNeedValue(&SmallYaw_PositionPID, target_angle);
    PID_PositionCalc_Encoder(&SmallYaw_PositionPID, current_angle);

    // ============ 5. 速度环计算 ============
    PID_PositionSetNeedValue(&SmallYaw_SpeedPID, SmallYaw_PositionPID.OUT);
    PID_PositionCalc(&SmallYaw_SpeedPID, current_speed);

    // ============ 6. 发送输出 ============
    // SmallYaw是ID 0x206，对应Voltage1的第2路，CAN2
    Motor_6020_Voltage1(0, (int16_t)SmallYaw_SpeedPID.OUT, 0, 0, &hcan2);
}

//=========================== 总控制循环 ===========================//

void Gimbal_Control_Loop(void)
{
    Gimbal_Pitch_Control();
    Gimbal_SmallYaw_Control();
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

//=========================== 调试打印函数 ===========================//

// void Gimbal_GetDebugData(Gimbal_Debug_Data_t *data)
// {
//     if (data == NULL) return;
    
//     // 遥控器
//     data->rc_ch2 = global_rc_control.rc.ch[2];
//     data->rc_ch3 = global_rc_control.rc.ch[3];
    
//     // SmallYaw
//     data->virtual_s_yaw = virtual_SmallYaw;
//     data->s_yaw_target = (int16_t)SmallYaw_PositionPID.Need_Value;
//     data->s_yaw_current = now_SmallYaw_count;
//     data->s_yaw_error = error_SmallYaw_count;
//     data->s_yaw_pos_out = SmallYaw_PositionPID.OUT;
//     data->s_yaw_speed_out = (int16_t)SmallYaw_SpeedPID.OUT;
    
//     // Pitch
//     data->p_target = Pitch_PositionPID.Need_Value;
//     data->p_current = Can1_M6020_MotorStatus[1].Position;
//     data->p_error = (int16_t)(data->p_target - data->p_current);
//     data->p_pos_out = Pitch_PositionPID.OUT;
//     data->p_speed_out = (int16_t)Pitch_SpeedPID.OUT;
// }

// void Gimbal_PrintDebug(void)
// {
//     Gimbal_Debug_Data_t dbg;
//     Gimbal_GetDebugData(&dbg);
    
//     printf("=== Gimbal Debug ===\r\n");
//     printf("RC: ch2=%d ch3=%d\r\n", dbg.rc_ch2, dbg.rc_ch3);
//     printf("SmallYaw: v=%.1f tgt=%d cur=%d err=%d p_out=%.1f s_out=%d\r\n",
//            dbg.virtual_s_yaw, dbg.s_yaw_target, dbg.s_yaw_current, 
//            dbg.s_yaw_error, dbg.s_yaw_pos_out, dbg.s_yaw_speed_out);
//     printf("Pitch: tgt=%.1f cur=%lld err=%d p_out=%.1f s_out=%d\r\n",
//            dbg.p_target, dbg.p_current, dbg.p_error,
//            dbg.p_pos_out, dbg.p_speed_out);
// }

// void Gimbal_PrintDebug_SmallYaw(void)
// {
//     Gimbal_Debug_Data_t dbg;
//     Gimbal_GetDebugData(&dbg);
    
//     printf("[SmallYaw] RC=%d v=%.1f tgt=%d cur=%d err=%d p_out=%.1f s_out=%d\r\n",
//            dbg.rc_ch2, dbg.virtual_s_yaw, dbg.s_yaw_target, 
//            dbg.s_yaw_current, dbg.s_yaw_error, 
//            dbg.s_yaw_pos_out, dbg.s_yaw_speed_out);
// }

// void Gimbal_PrintDebug_Pitch(void)
// {
//     Gimbal_Debug_Data_t dbg;
//     Gimbal_GetDebugData(&dbg);
    
//     printf("[Pitch] RC=%d tgt=%.1f cur=%lld err=%d p_out=%.1f s_out=%d\r\n",
//            dbg.rc_ch3, dbg.p_target, dbg.p_current, 
//            dbg.p_error, dbg.p_pos_out, dbg.p_speed_out);
// }
