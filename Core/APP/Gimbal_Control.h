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
extern PID_PositionInitTypedef BigYaw_PositionPID;
extern PID_PositionInitTypedef BigYaw_SpeedPID;

//=========================== 配置参数（可调）===========================//

// Pitch轴 - CAN1, ID 0x206, 索引1
#define PITCH_RC_CHANNEL        3
#define PITCH_RC_SENS           0.01f
#define PITCH_LIMIT_UP          4500.0f
#define PITCH_LIMIT_DOWN        3300.0f

// SmallYaw轴 - CAN2, ID 0x206, 索引1
#define SMALLYAW_RC_CHANNEL     2
#define SMALLYAW_RC_SENS        0.5f        // 灵敏度提高10倍（原0.05）
#define SMALLYAW_VIRTUAL_LIMIT  4000.0f     // 限幅放宽（原1500）

//=========================== 调试数据结构 ===========================//

typedef struct {
    // 遥控器
    int16_t rc_ch2;             // yaw通道原始值
    int16_t rc_ch3;             // pitch通道原始值
    
    // SmallYaw
    float virtual_s_yaw;        // 虚拟坐标
    int16_t s_yaw_target;       // 目标编码值
    int16_t s_yaw_current;      // 当前编码值
    int16_t s_yaw_error;        // 误差
    int16_t s_yaw_speed_out;    // 速度环输出
    float s_yaw_pos_out;        // 位置环输出
    
    // Pitch
    float p_target;             // 目标位置
    int64_t p_current;          // 当前位置
    int16_t p_error;            // 误差
    int16_t p_speed_out;        // 速度环输出
    float p_pos_out;            // 位置环输出
    
} Gimbal_Debug_Data_t;

//=========================== 接口函数 ===========================//

void Gimbal_Control_Init(void);
void Gimbal_Control_Loop(void);

// 单独控制函数
void Gimbal_Pitch_Control(void);
void Gimbal_SmallYaw_Control(void);
void Gimbal_BigYaw_Control(void);

// PID调试
void Gimbal_Pitch_SetPosPID(float kp, float ki, float kd);
void Gimbal_Pitch_SetSpeedPID(float kp, float ki, float kd);
void Gimbal_SmallYaw_SetPosPID(float kp, float ki, float kd);
void Gimbal_SmallYaw_SetSpeedPID(float kp, float ki, float kd);
void Gimbal_BigYaw_SetPosPID(float kp, float ki, float kd);
void Gimbal_BigYaw_SetSpeedPID(float kp, float ki, float kd);

// 调试打印
void Gimbal_GetDebugData(Gimbal_Debug_Data_t *data);
void Gimbal_PrintDebug(void);
void Gimbal_PrintDebug_SmallYaw(void);   // 只打印SmallYaw
void Gimbal_PrintDebug_Pitch(void);      // 只打印Pitch

#endif // __GIMBAL_CONTROL_H
