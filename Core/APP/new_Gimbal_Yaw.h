// #ifndef __NEW_GIMBAL_YAW_H
// #define __NEW_GIMBAL_YAW_H

// #include <stdint.h>
// #include "PID.h"
// #include "Motor.h"
// #include "remote_control.h"

// //=========================== 外部变量声明 (实际使用的) ===========================//

// extern PID_PositionInitTypedef SmallYaw_GyroscopePID;
// extern PID_PositionInitTypedef SmallYaw_PositionPID;
// extern PID_PositionInitTypedef SmallYaw_SpeedPID;

// extern M6020_Motor Can2_M6020_MotorStatus[7];     // GM6020电机状态数组 - Can2
// extern RC_ctrl_t global_rc_control;               // 全局遥控器数据

// //=========================== 配置参数 ===========================//

// // SmallYaw 配置 (Can2, ID 0x206, 数组索引1)
// #define SMALLYAW_MOTOR_IDX  1
// #define SMALLYAW_MID        4000            // 中位值(编码器原始值 0-8191)
// #define SMALLYAW_MAX_DELTA  3000            // 最大偏移量(左右软限位)

// // BigYaw 配置 (Can2, ID 0x205, 数组索引0)
// #define BIGYAW_MOTOR_IDX    0

// // 遥控器灵敏度
// #define SMALLYAW_REMOTE_SENS    0.05f

// //=========================== 控制模式 ===========================//

// typedef enum {
//     SMALLYAW_MODE_ENCODER = 0,      // 编码器模式：直接设置目标编码器值
//     SMALLYAW_MODE_REMOTE,           // 遥控器模式：遥控器增量控制
//     SMALLYAW_MODE_GYRO,             // 陀螺仪模式：预留
// } SmallYaw_Mode_e;

// //=========================== 数据结构 ===========================//

// typedef struct {
//     float target_position;          // 目标位置(编码器原始值 0-8191)
//     float current_position;         // 当前位置
//     float current_speed;            // 当前速度
//     SmallYaw_Mode_e mode;           // 当前控制模式
//     float remote_sensitivity;       // 遥控器灵敏度
//     float limit_min;                // 最小限位值
//     float limit_max;                // 最大限位值
//     int16_t output;                 // 电机输出值
// } SmallYaw_Control_t;

// typedef struct {
//     float initial_offset;           // BigYaw与SmallYaw的初始机械差值
//     int8_t  is_offset_calibrated;   // 差值校准标志
//     float target_position;          // 目标位置
//     float current_position;         // 当前位置
//     float current_speed;            // 当前速度
//     uint8_t follow_enable;          // 跟随使能标志
//     int16_t output;                 // 电机输出值
// } BigYaw_Control_t;

// //=========================== 对外接口 ===========================//

// void Gimbal_YawSmall_Init(void);
// void Gimbal_YawBig_Init(void);
// void Gimbal_YawSmall_Control(void);
// void Gimbal_YawBig_Control(void);

// void Gimbal_YawSmall_SetMode(SmallYaw_Mode_e mode);
// SmallYaw_Mode_e Gimbal_YawSmall_GetMode(void);

// void Gimbal_YawSmall_SetTargetPosition(float position);
// float Gimbal_YawSmall_GetTargetPosition(void);
// float Gimbal_YawSmall_GetCurrentPosition(void);

// void Gimbal_YawBig_EnableFollow(uint8_t enable);
// void Gimbal_YawBig_RecalibrateOffset(void);
// float Gimbal_YawBig_GetCurrentPosition(void);
// float Gimbal_YawBig_GetOffset(void);

// // PID调试
// void Gimbal_YawSmall_SetPosPID(float kp, float ki, float kd);
// void Gimbal_YawSmall_SetSpeedPID(float kp, float ki, float kd);
// void Gimbal_YawBig_SetPosPID(float kp, float ki, float kd);
// void Gimbal_YawBig_SetSpeedPID(float kp, float ki, float kd);

// // CAN输出 - 发送两个yaw轴的电压指令
// void Gimbal_Yaw_SendOutput(void);

// #endif // __NEW_GIMBAL_YAW_H
