#ifndef __GIMBAL_SMALLYAW_H
#define __GIMBAL_SMALLYAW_H

#include <stdint.h>
#include "PID.h"
#include "Motor.h"
#include "remote_control.h"

//=========================== 外部变量 ===========================//

extern RC_ctrl_t global_rc_control;
extern M6020_Motor Can2_M6020_MotorStatus[7];

// freertos.c中定义的机械中值和状态变量
extern int16_t origin_SmallYaw_count;
extern int16_t now_SmallYaw_count;
extern int16_t error_SmallYaw_count;

//=========================== PID变量（供外部使用）===========================//

extern PID_PositionInitTypedef SmallYaw_PositionPID;
extern PID_PositionInitTypedef SmallYaw_SpeedPID;

//=========================== 配置 ===========================//

#define SMALLYAW_MOTOR_IDX      1       // CAN2, ID 0x206
#define SMALLYAW_RC_CHANNEL     2       // 遥控器ch[2]
#define SMALLYAW_RC_SENS        0.05f   // 遥控器灵敏度
#define SMALLYAW_VIRTUAL_LIMIT  1500    // 虚拟坐标软限幅 ±1500

//=========================== 接口 ===========================//

void Gimbal_SmallYaw_Init(void);
void Gimbal_SmallYaw_Control(void);

#endif // __GIMBAL_SMALLYAW_H
