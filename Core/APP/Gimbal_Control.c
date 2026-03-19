#include "Gimbal_Control.h"
#include "can.h"
#include <math.h>
#include "Chassis_Follow.h"

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

//=========================== 工具函数：编码器归一化 ===========================//

static float normalize_encoder_diff(float angle)
{
    while (angle < -4096.0f) angle += 8192.0f;
    while (angle >= 4096.0f) angle -= 8192.0f;
    return angle;
}

//=========================== Yaw控制（反馈补偿方案）===========================//
//
// 【核心逻辑】
//   Virtual_Yaw：负责维持云台相对于地面的绝对朝向（闭环）
//   Chassis_Follow：负责抵消底盘转动的影响（前馈补偿反馈值）
//
// 【数学等价】
//   方法1（补偿目标）：error = (V + C) - R
//   方法2（补偿反馈）：error = V - (R - C)  <-- 本方案
//
// 【优势】
//   - Virtual_Yaw 保持单一职责：只需关注绝对朝向
//   - 底盘跟随与虚拟坐标真正解耦
//   - 便于单独开关底盘跟随功能

void Gimbal_Yaw_Control(void)
{
    //---------- 1. 获取原始编码（给虚拟层计算目标）----------
    float real_small = (float)Can2_M6020_MotorStatus[1].Angle;
    float real_big = (float)Can2_M6020_MotorStatus[0].Angle;
    float real_small_speed = (float)Can2_M6020_MotorStatus[1].Speed;
    float real_big_speed = (float)Can2_M6020_MotorStatus[0].Speed;
    
    //---------- 2. 更新虚拟坐标层 ----------
    // 虚拟层基于原始编码计算目标，保持对绝对朝向的闭环控制
    Virtual_Yaw_Update(global_rc_control.rc.ch[2], real_small, real_big);
    
    //---------- 3. 从虚拟层获取目标 ----------
    // SmallYaw：仅由virtual_position控制（不受底盘跟随影响）
    float target_small = Virtual_Yaw_GetTarget_Small();
    
    // BigYaw：仅由virtual_position控制绝对朝向
    float target_big = Virtual_Yaw_GetTarget_Big();
    
    //---------- 4. 底盘跟随补偿（反馈值修正）----------
    // 【关键】将底盘转动的影响从反馈中"扣除"
    // 这样PID看到的是"相对于地面的编码值"
    float chassis_compensation = Chassis_Follow_GetTarget_Big();
    float real_big_compensated = real_big - chassis_compensation;
    
    // 处理编码器环绕（补偿后可能越界）
    real_big_compensated = normalize_encoder_diff(real_big_compensated);
    
    //---------- 5. SmallYaw控制 ----------
    PID_PositionSetNeedValue(&SmallYaw_PositionPID, target_small);
    PID_PositionCalc_Encoder(&SmallYaw_PositionPID, real_small);
    
    PID_PositionSetNeedValue(&SmallYaw_SpeedPID, SmallYaw_PositionPID.OUT);
    PID_PositionCalc(&SmallYaw_SpeedPID, real_small_speed);
    
    //---------- 6. BigYaw控制（使用补偿后的反馈）----------
    PID_PositionSetNeedValue(&BigYaw_PositionPID, target_big);
    PID_PositionCalc_Encoder(&BigYaw_PositionPID, real_big_compensated);
    
    PID_PositionSetNeedValue(&BigYaw_SpeedPID, BigYaw_PositionPID.OUT);
    PID_PositionCalc(&BigYaw_SpeedPID, real_big_speed);
    
    //---------- 7. 发送输出 ----------
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
