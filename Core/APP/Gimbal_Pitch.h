#ifndef __GIMBAL_PITCH_H
#define __GIMBAL_PITCH_H

#include "PID.h"
#include "Motor.h"
#include "remote_control.h"
#include "BSP_CAN.h"

void Gimbal_Pitch_Init(void);
void Gimbal_Pitch_Control(void);
#endif // __GIMBAL_PITCH_H
