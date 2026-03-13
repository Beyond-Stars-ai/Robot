#ifndef __GIMBAL_YAW_SMALL_H
#define __GIMBAL_YAW_SMALL_H

#include "PID.h"
#include "Motor.h"
#include "remote_control.h"
#include "BSP_CAN.h"
#include "BMI088.h"
#include "Gimbal_PoseCalc.h"

void Gimbal_YawSmall_Init(void);
// void Gimbal_YawSmall_Control(void);

void Gimbal_YawBig_Init(void);
// void Gimbal_YawBig_Control(void);

#endif // __GIMBAL_YAW_SMALL_H
