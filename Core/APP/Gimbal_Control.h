#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#include <stdint.h>
#include "PID.h"
#include "Motor.h"
#include "remote_control.h"

//=========================== 外部变量 ===========================//

extern RC_ctrl_t global_rc_control;
extern M6020_Motor Can1_M6020_MotorStatus[7];  // CAN1 - Pitch
extern M6020_Motor Can2_M6020_MotorStatus[7];  // CAN2 - Yaw

// 机械中值和状态变量（定义在freertos.c）
extern int16_t origin_BigYaw_count;
extern int16_t origin_SmallYaw_count;
extern int16_t now_BigYaw_count;
extern int16_t now_SmallYaw_count;
extern int16_t error_BigYaw_count;
extern int16_t error_SmallYaw_count;

//=========================== PID变量 ===========================//

extern PID_PositionInitTypedef Pitch_PositionPID;
extern PID_PositionInitTypedef Pitch_SpeedPID;
extern PID_PositionInitTypedef SmallYaw_PositionPID;
extern PID_PositionInitTypedef SmallYaw_SpeedPID;

//=========================== 配置参数 ===========================//

// Pitch轴 - CAN1, ID 0x206, 索引1
#define PITCH_RC_CHANNEL        3
#define PITCH_RC_SENS           0.01f
#define PITCH_LIMIT_UP          4500.0f
#define PITCH_LIMIT_DOWN        3300.0f

// SmallYaw轴 - CAN2, ID 0x206, 索引1
#define SMALLYAW_RC_CHANNEL     2
#define SMALLYAW_RC_SENS        0.08f
#define SMALLYAW_VIRTUAL_LIMIT  1500.0f

//=========================== 接口函数 ===========================//

void Gimbal_Control_Init(void);
void Gimbal_Control_Loop(void);

// 单独控制函数
void Gimbal_Pitch_Control(void);
void Gimbal_SmallYaw_Control(void);

// PID调试
void Gimbal_Pitch_SetPosPID(float kp, float ki, float kd);
void Gimbal_Pitch_SetSpeedPID(float kp, float ki, float kd);
void Gimbal_SmallYaw_SetPosPID(float kp, float ki, float kd);
void Gimbal_SmallYaw_SetSpeedPID(float kp, float ki, float kd);

#endif // __GIMBAL_CONTROL_H
