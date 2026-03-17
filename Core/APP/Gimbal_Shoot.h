#ifndef GIMBAL_SHOOT_H
#define	GIMBAL_SHOOT_H

#include "PID.h"
#include "Motor.h"
#include "remote_control.h"
#include "BSP_CAN.h"

void Gimbal_Shoot_Init(void);
void Gimbal_Shoot_Control(void);

#endif //GIMBAL_SHOOT_H
