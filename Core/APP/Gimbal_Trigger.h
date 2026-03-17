#ifndef GIMBAL_TRIGGER_H
#define GIMBAL_TRIGGER_H

#include "PID.h"
#include "Motor.h"
#include "remote_control.h"
#include "BSP_CAN.h"

void Gimbal_Trigger_Init(void);
void Gimbal_Trigger_Control(void);

#endif //GIMBAL_TRIGGER_H
