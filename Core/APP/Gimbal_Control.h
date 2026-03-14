#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#include <stdint.h>
#include "PID.h"
#include "Motor.h"
#include "remote_control.h"

//=========================== 外部变量 ===========================//

// 机械中值编码值（定义在freertos.c）
extern int16_t origin_BigYaw_count;
extern int16_t origin_SmallYaw_count;
extern int16_t now_BigYaw_count;
extern int16_t now_SmallYaw_count;
extern int16_t error_BigYaw_count;
extern int16_t error_SmallYaw_count;

extern RC_ctrl_t global_rc_control;
extern M6020_Motor Can1_M6020_MotorStatus[7];  // CAN1 GM6020 - Pitch
extern M6020_Motor Can2_M6020_MotorStatus[7];  // CAN2 GM6020 - BigYaw, SmallYaw

//=========================== Pitch轴配置 ===========================//

// Pitch轴使用CAN1, ID 0x206, 数组索引1
#define PITCH_CAN_BUS       1
#define PITCH_MOTOR_IDX     1
#define PITCH_MID           4074        // 机械中值编码值
#define PITCH_LIMIT_UP      4500        // 上极限
#define PITCH_LIMIT_DOWN    3300        // 下极限
#define PITCH_RC_CHANNEL    3           // 遥控器通道ch[3]
#define PITCH_RC_SENS       0.01f       // 遥控器灵敏度

//=========================== Yaw轴配置（预留）===========================//

// BigYaw: CAN2, ID 0x205, 数组索引0, 顺时针编码增加
#define BIGYAW_MOTOR_IDX    0
#define BIGYAW_CW_INCREASE  1           // 顺时针编码增加

// SmallYaw: CAN2, ID 0x206, 数组索引1, 逆时针编码增加（顺时针减小）
#define SMALLYAW_MOTOR_IDX  1
#define SMALLYAW_CW_INCREASE 0          // 顺时针编码减小（逆时针增加）

#define YAW_RC_CHANNEL      2           // 遥控器通道ch[2]

//=========================== PID变量（供外部调试使用）===========================//

extern PID_PositionInitTypedef Pitch_PositionPID;
extern PID_PositionInitTypedef Pitch_SpeedPID;

//=========================== 接口函数 ===========================//

// 初始化
void Gimbal_Control_Init(void);
void Gimbal_Pitch_Init(void);

// 控制函数
void Gimbal_Control_Loop(void);     // 总控制循环
void Gimbal_Pitch_Control(void);

// PID调试
void Gimbal_Pitch_SetPosPID(float kp, float ki, float kd);
void Gimbal_Pitch_SetSpeedPID(float kp, float ki, float kd);

#endif // __GIMBAL_CONTROL_H
