#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#include <stdint.h>
#include "PID.h"
#include "Motor.h"
#include "remote_control.h"
#include "virtual_postion.h"

//=========================== 外部变量 ===========================//

extern RC_ctrl_t global_rc_control;
extern M6020_Motor Can1_M6020_MotorStatus[7];
extern M6020_Motor Can2_M6020_MotorStatus[7];

//=========================== 全局变量 ===========================//

/**
 * @brief 底盘补偿累积值（编码器值单位）
 * 
 * CalTask每20ms更新：
 *   g_chassis_compensation += (-delta_yaw * 22.756f)
 * 
 * 表示底盘相对于开机时转动了多少编码器单位。
 * Virtual_Yaw_Update会计算其变化率，转换为虚拟RC值。
 */
extern float g_chassis_compensation;

//=========================== PID变量 ===========================//

extern PID_PositionInitTypedef Pitch_PositionPID;
extern PID_PositionInitTypedef Pitch_SpeedPID;
extern PID_PositionInitTypedef SmallYaw_PositionPID;
extern PID_PositionInitTypedef SmallYaw_SpeedPID;
extern PID_PositionInitTypedef BigYaw_PositionPID;
extern PID_PositionInitTypedef BigYaw_SpeedPID;

//=========================== 配置参数 ===========================//

#define PITCH_RC_CHANNEL        3
#define PITCH_RC_SENS           0.03f
#define PITCH_LIMIT_UP          4500.0f
#define PITCH_LIMIT_DOWN        3300.0f

//=========================== 接口函数 ===========================//

void Gimbal_Control_Init(void);
void Gimbal_Control_Loop(void);

void Gimbal_Pitch_Control(void);
void Gimbal_Yaw_Control(void);

// PID调试
void Gimbal_Pitch_SetPosPID(float kp, float ki, float kd);
void Gimbal_Pitch_SetSpeedPID(float kp, float ki, float kd);
void Gimbal_SmallYaw_SetPosPID(float kp, float ki, float kd);
void Gimbal_SmallYaw_SetSpeedPID(float kp, float ki, float kd);
void Gimbal_BigYaw_SetPosPID(float kp, float ki, float kd);
void Gimbal_BigYaw_SetSpeedPID(float kp, float ki, float kd);

#endif // __GIMBAL_CONTROL_H
