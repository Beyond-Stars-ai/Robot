#ifndef GIMBAL_CTOC_H
#define GIMBAL_CTOC_H
#include "BMI088.h"
#include "remote_control.h"

extern BMI088_Init_typedef Can_BMI088_Data;

void Gimbal_CtoC_Remote(void);
//void CToC_GyroProcess(uint32_t ID,uint8_t *Data);
//void CToC_AccelProcess(uint32_t ID,uint8_t *Data);
void CToC_AngleProcess(uint8_t *Data,BMI088_Init_typedef *data);



#endif //GIMBAL_CTOC_H

