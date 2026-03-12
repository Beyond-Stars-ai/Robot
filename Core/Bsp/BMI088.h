#ifndef BMI088_H
#define BMI088_H
#include "stdint.h"
#include "can.h"

typedef struct 
{
	float Accel[3];
	float Gyro[3]; 
	float Yaw,Pitch,Roll;
	float Mag[3];  
	float Temp;    
}BMI088_Init_typedef;

BMI088_Init_typedef BMI088_GetData(BMI088_Init_typedef *data);
void BMI088_Angle(int16_t data1, int16_t data2, 
									int16_t data3, int16_t data4,
									CAN_HandleTypeDef *hcan);
void BMI088_Gyro(	int16_t data1, int16_t data2, 
									int16_t data3, int16_t data4,
									CAN_HandleTypeDef *hcan);
void BMI088_Accel(int16_t data1, int16_t data2, 
									int16_t data3, int16_t data4,
									CAN_HandleTypeDef *hcan);
#endif // BMI088_H
