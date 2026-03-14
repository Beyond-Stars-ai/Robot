// #include "new_Gimbal_Yaw.h"
// #include "can.h"          // 包含hcan2定义
// #include <math.h>

// //=========================== PID定义 (保留原有变量名供外部使用) ===========================//

// PID_PositionInitTypedef SmallYaw_GyroscopePID;    // 陀螺仪外环(预留)
// PID_PositionInitTypedef SmallYaw_PositionPID;     // 编码器位置环
// PID_PositionInitTypedef SmallYaw_SpeedPID;        // 速度环

// static PID_PositionInitTypedef BigYaw_PositionPID; // BigYaw位置环
// static PID_PositionInitTypedef BigYaw_SpeedPID;    // BigYaw速度环

// //=========================== 内部变量 ===========================//

// static SmallYaw_Control_t SmallYaw_Ctrl;
// static BigYaw_Control_t BigYaw_Ctrl;

// // 快捷访问宏
// #define SMALLYAW_MOTOR  (Can2_M6020_MotorStatus[SMALLYAW_MOTOR_IDX])
// #define BIGYAW_MOTOR    (Can2_M6020_MotorStatus[BIGYAW_MOTOR_IDX])

// //=========================== 工具函数 ===========================//

// static float normalize_encoder(float value)
// {
//     while (value < 0) value += 8192.0f;
//     while (value >= 8192.0f) value -= 8192.0f;
//     return value;
// }

// static float limit_value(float value, float min, float max)
// {
//     if (value < min) return min;
//     if (value > max) return max;
//     return value;
// }

// //=========================== SmallYaw初始化 ===========================//

// void Gimbal_YawSmall_Init(void)
// {
//     // 结构体初始化
//     SmallYaw_Ctrl.mode = SMALLYAW_MODE_ENCODER;
//     SmallYaw_Ctrl.target_position = SMALLYAW_MID;
//     SmallYaw_Ctrl.current_position = 0;
//     SmallYaw_Ctrl.current_speed = 0;
//     SmallYaw_Ctrl.remote_sensitivity = SMALLYAW_REMOTE_SENS;
//     SmallYaw_Ctrl.output = 0;
//     SmallYaw_Ctrl.limit_min = SMALLYAW_MID - SMALLYAW_MAX_DELTA;
//     SmallYaw_Ctrl.limit_max = SMALLYAW_MID + SMALLYAW_MAX_DELTA;
    
//     // 位置环PID - 使用PID_PositionCalc_Encoder处理0-8191循环
//     PID_PositionStructureInit(&SmallYaw_PositionPID, SmallYaw_Ctrl.target_position);
//     PID_PositionSetParameter(&SmallYaw_PositionPID, 0.5f, 0.0f, 0.0f);
//     PID_PositionSetOUTRange(&SmallYaw_PositionPID, -4000.0f, 4000.0f);
//     PID_PositionSetEkRange(&SmallYaw_PositionPID, -10.0f, 10.0f);
    
//     // 速度环PID
//     PID_PositionStructureInit(&SmallYaw_SpeedPID, 0);
//     PID_PositionSetParameter(&SmallYaw_SpeedPID, 20.0f, 0.0f, 0.0f);
//     PID_PositionSetOUTRange(&SmallYaw_SpeedPID, -10000.0f, 10000.0f);
//     PID_PositionSetEkRange(&SmallYaw_SpeedPID, -3.0f, 3.0f);
    
//     // 陀螺仪外环预留
//     PID_PositionStructureInit(&SmallYaw_GyroscopePID, 0);
//     PID_PositionSetParameter(&SmallYaw_GyroscopePID, 8.0f, 0.0f, 0.0f);
//     PID_PositionSetOUTRange(&SmallYaw_GyroscopePID, -4000.0f, 4000.0f);
// }

// //=========================== BigYaw初始化 ===========================//

// void Gimbal_YawBig_Init(void)
// {
//     BigYaw_Ctrl.initial_offset = 0;
//     BigYaw_Ctrl.is_offset_calibrated = 0;
//     BigYaw_Ctrl.target_position = 0;
//     BigYaw_Ctrl.current_position = 0;
//     BigYaw_Ctrl.current_speed = 0;
//     BigYaw_Ctrl.follow_enable = 1;
//     BigYaw_Ctrl.output = 0;
    
//     PID_PositionStructureInit(&BigYaw_PositionPID, 0);
//     PID_PositionSetParameter(&BigYaw_PositionPID, 0.8f, 0.0f, 0.0f);
//     PID_PositionSetOUTRange(&BigYaw_PositionPID, -4000.0f, 4000.0f);
//     PID_PositionSetEkRange(&BigYaw_PositionPID, -10.0f, 10.0f);
    
//     PID_PositionStructureInit(&BigYaw_SpeedPID, 0);
//     PID_PositionSetParameter(&BigYaw_SpeedPID, 25.0f, 0.0f, 0.0f);
//     PID_PositionSetOUTRange(&BigYaw_SpeedPID, -12000.0f, 12000.0f);
//     PID_PositionSetEkRange(&BigYaw_SpeedPID, -3.0f, 3.0f);
// }

// //=========================== SmallYaw控制 ===========================//

// void Gimbal_YawSmall_Control(void)
// {
//     // 获取当前编码器值和速度
//     SmallYaw_Ctrl.current_position = (float)SMALLYAW_MOTOR.Angle;
//     SmallYaw_Ctrl.current_speed = (float)SMALLYAW_MOTOR.Speed;
    
//     // 根据模式更新目标值
//     switch (SmallYaw_Ctrl.mode) {
//         case SMALLYAW_MODE_ENCODER:
//             // 编码器模式：目标值由SetTargetPosition设置
//             break;
            
//         case SMALLYAW_MODE_REMOTE:
//             // 遥控器模式：ch[2]控制yaw，中位1024
//             {
//                 int16_t rc_value = global_rc_control.rc.ch[2];
//                 float delta = (rc_value - 1024) * SmallYaw_Ctrl.remote_sensitivity;
//                 SmallYaw_Ctrl.target_position += delta;
//                 // 软限位
//                 SmallYaw_Ctrl.target_position = limit_value(
//                     SmallYaw_Ctrl.target_position, 
//                     SmallYaw_Ctrl.limit_min, 
//                     SmallYaw_Ctrl.limit_max);
//             }
//             break;
            
//         case SMALLYAW_MODE_GYRO:
//             // 陀螺仪模式：预留接口
//             break;
            
//         default:
//             break;
//     }
    
//     // 目标值归一化到0-8191
//     SmallYaw_Ctrl.target_position = normalize_encoder(SmallYaw_Ctrl.target_position);
    
//     // ============ 位置环(编码器环) ============
//     PID_PositionSetNeedValue(&SmallYaw_PositionPID, SmallYaw_Ctrl.target_position);
//     PID_PositionCalc_Encoder(&SmallYaw_PositionPID, SmallYaw_Ctrl.current_position);
    
//     // ============ 速度环 ============
//     PID_PositionSetNeedValue(&SmallYaw_SpeedPID, SmallYaw_PositionPID.OUT);
//     PID_PositionCalc(&SmallYaw_SpeedPID, SmallYaw_Ctrl.current_speed);
    
//     SmallYaw_Ctrl.output = (int16_t)SmallYaw_SpeedPID.OUT;
// }

// //=========================== BigYaw控制 ===========================//

// void Gimbal_YawBig_Control(void)
// {
//     // 获取当前编码器值和速度
//     BigYaw_Ctrl.current_position = (float)BIGYAW_MOTOR.Angle;
//     BigYaw_Ctrl.current_speed = (float)BIGYAW_MOTOR.Speed;
    
//     // 首次校准：记录初始机械差值
//     if (!BigYaw_Ctrl.is_offset_calibrated) {
//         // First_Flag在Motor.c中收到数据后设置
//         if (SMALLYAW_MOTOR.First_Flag && BIGYAW_MOTOR.First_Flag) {
//             // 初始差值 = BigYaw位置 - SmallYaw位置
//             BigYaw_Ctrl.initial_offset = BigYaw_Ctrl.current_position - SmallYaw_Ctrl.current_position;
//             BigYaw_Ctrl.is_offset_calibrated = 1;
//         } else {
//             // 数据未就绪，不输出
//             BigYaw_Ctrl.output = 0;
//             return;
//         }
//     }
    
//     if (!BigYaw_Ctrl.follow_enable) {
//         BigYaw_Ctrl.output = 0;
//         return;
//     }
    
//     // 目标位置 = SmallYaw当前位置 + 初始差值
//     // 这样BigYaw会跟随SmallYaw保持固定的机械差值
//     BigYaw_Ctrl.target_position = normalize_encoder(
//         SmallYaw_Ctrl.current_position + BigYaw_Ctrl.initial_offset);
    
//     // ============ 位置环 ============
//     PID_PositionSetNeedValue(&BigYaw_PositionPID, BigYaw_Ctrl.target_position);
//     PID_PositionCalc_Encoder(&BigYaw_PositionPID, BigYaw_Ctrl.current_position);
    
//     // ============ 速度环 ============
//     PID_PositionSetNeedValue(&BigYaw_SpeedPID, BigYaw_PositionPID.OUT);
//     PID_PositionCalc(&BigYaw_SpeedPID, BigYaw_Ctrl.current_speed);
    
//     BigYaw_Ctrl.output = (int16_t)BigYaw_SpeedPID.OUT;
// }

// //=========================== 接口函数 ===========================//

// void Gimbal_YawSmall_SetMode(SmallYaw_Mode_e mode)
// {
//     if (mode != SmallYaw_Ctrl.mode) {
//         SmallYaw_Ctrl.mode = mode;
//         PID_PositionClean(&SmallYaw_PositionPID);
//         PID_PositionClean(&SmallYaw_SpeedPID);
//     }
// }

// SmallYaw_Mode_e Gimbal_YawSmall_GetMode(void)
// {
//     return SmallYaw_Ctrl.mode;
// }

// void Gimbal_YawSmall_SetTargetPosition(float position)
// {
//     SmallYaw_Ctrl.target_position = limit_value(position, 
//         SmallYaw_Ctrl.limit_min, SmallYaw_Ctrl.limit_max);
// }

// float Gimbal_YawSmall_GetTargetPosition(void)
// {
//     return SmallYaw_Ctrl.target_position;
// }

// float Gimbal_YawSmall_GetCurrentPosition(void)
// {
//     return SmallYaw_Ctrl.current_position;
// }

// void Gimbal_YawBig_EnableFollow(uint8_t enable)
// {
//     BigYaw_Ctrl.follow_enable = enable;
// }

// void Gimbal_YawBig_RecalibrateOffset(void)
// {
//     BigYaw_Ctrl.is_offset_calibrated = 0;
// }

// float Gimbal_YawBig_GetCurrentPosition(void)
// {
//     return BigYaw_Ctrl.current_position;
// }

// float Gimbal_YawBig_GetOffset(void)
// {
//     return BigYaw_Ctrl.initial_offset;
// }

// void Gimbal_YawSmall_SetPosPID(float kp, float ki, float kd)
// {
//     PID_PositionSetParameter(&SmallYaw_PositionPID, kp, ki, kd);
// }

// void Gimbal_YawSmall_SetSpeedPID(float kp, float ki, float kd)
// {
//     PID_PositionSetParameter(&SmallYaw_SpeedPID, kp, ki, kd);
// }

// void Gimbal_YawBig_SetPosPID(float kp, float ki, float kd)
// {
//     PID_PositionSetParameter(&BigYaw_PositionPID, kp, ki, kd);
// }

// void Gimbal_YawBig_SetSpeedPID(float kp, float ki, float kd)
// {
//     PID_PositionSetParameter(&BigYaw_SpeedPID, kp, ki, kd);
// }

// //=========================== CAN输出 ===========================//
// /**
//  * @brief 发送Yaw轴电机电压指令
//  * @note  使用CAN2，0x1FF指令
//  *        BigYaw (ID 0x205) -> 第1路
//  *        SmallYaw (ID 0x206) -> 第2路
//  */
// void Gimbal_Yaw_SendOutput(void)
// {
//     // Motor_6020_Voltage1: CAN ID 0x1FF, 控制电机ID 1-4 (对应0x205-0x208)
//     // 参数顺序: device1, device2, device3, device4
//     // BigYaw是ID 0x205 (第1路), SmallYaw是ID 0x206 (第2路)
//     Motor_6020_Voltage1(BigYaw_Ctrl.output, SmallYaw_Ctrl.output, 0, 0, &hcan2);
// }
