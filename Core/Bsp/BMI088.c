#include <math.h>
#include "can.h"
#include "BMI088.h"
#include "BMI088driver.h"
#include "Parameter.h"
//===============变量区

//===============公共函数
//简介：获取欧拉角数据(单位：°)
BMI088_Init_typedef BMI088_GetData(BMI088_Init_typedef *data)
{
	//上次读取时刻、本次读取时刻、读取时间间隔
  static uint32_t last_read_time = 0;
	static uint32_t current_time = 0;
	static uint32_t delta_time = 0;
	static double yaw_g,pitch_g,roll_g;//角速度计算三轴数据
	static double yaw_a,pitch_a,roll_a;//加速度计算三轴数据
	float alpha = 0.95238;//0.95238
	
	BMI088_read(data->Gyro, data->Accel, &data->Temp);
	if(data->Accel[0]>2 || data->Accel[1]>2 || data->Accel[2]>2){
	//计算时间变化
		last_read_time = current_time;
		current_time = HAL_GetTick();
		delta_time = current_time - last_read_time;
	//计算欧拉角
		yaw_g 	+=  (0.001f * delta_time) * data->Gyro[2]/ PI	* 180;   // 角度增量 = 角速度 * 时间
		pitch_g +=  (0.001f * delta_time) * data->Gyro[1]/ PI	* 180; 
		roll_g 	+=  (0.001f * delta_time) * data->Gyro[0]/ PI	* 180;  
	
		yaw_a 	= 0;																	// 角度增量 = 角速度 * 时间
		pitch_a = atan2(data->Accel[0],data->Accel[2]) / PI	* 180;  
		roll_a 	= atan2(data->Accel[1],data->Accel[2]) / PI	* 180; 
		
		data->Yaw 	= alpha*yaw_g + (1-alpha)*yaw_a;
		data->Pitch = alpha*pitch_g + (1-alpha)*pitch_a;
		data->Roll 	= alpha*roll_g + (1-alpha)*roll_a;
	}
	if(data->Yaw > 180)				data->Yaw -=360;
	else if(data->Yaw < -180)	data->Yaw +=360;
	if(data->Pitch > 180)				data->Pitch -=360;
	else if(data->Pitch < -180)	data->Pitch +=360;
	if(data->Roll > 180)				data->Roll -=360;
	else if(data->Roll < -180)	data->Roll +=360;
	
	return *data;
}

void BMI088_Angle(int16_t data1, int16_t data2, 
										 int16_t data3, int16_t data4,
										 CAN_HandleTypeDef *hcan)//CANID:0x200 电机ID:1-4 (-10000,10000)
{
		CAN_TxHeaderTypeDef  Tx_Message;
		uint8_t	Can_Send_Data[8];
    uint32_t send_mail_box;
	
    Tx_Message.StdId = 0x146;
    Tx_Message.IDE = CAN_ID_STD;
    Tx_Message.RTR = CAN_RTR_DATA;
    Tx_Message.DLC = 0x08;
    Can_Send_Data[0] = data1 >> 8;
    Can_Send_Data[1] = data1;
    Can_Send_Data[2] = data2 >> 8;
    Can_Send_Data[3] = data2;
    Can_Send_Data[4] = data3 >> 8;
    Can_Send_Data[5] = data3;
    Can_Send_Data[6] = data4 >> 8;
    Can_Send_Data[7] = data4;

    HAL_CAN_AddTxMessage(hcan, &Tx_Message, Can_Send_Data, &send_mail_box);
}


void BMI088_Gyro(int16_t data1, int16_t data2, 
										 int16_t data3, int16_t data4,
										 CAN_HandleTypeDef *hcan)//CANID:0x200 电机ID:1-4 (-10000,10000)
{
		CAN_TxHeaderTypeDef  Tx_Message;
		uint8_t	Can_Send_Data[8];
    uint32_t send_mail_box;
	
    Tx_Message.StdId = 0x147;
    Tx_Message.IDE = CAN_ID_STD;
    Tx_Message.RTR = CAN_RTR_DATA;
    Tx_Message.DLC = 0x08;
    Can_Send_Data[0] = data1 >> 8;
    Can_Send_Data[1] = data1;
    Can_Send_Data[2] = data2 >> 8;
    Can_Send_Data[3] = data2;
    Can_Send_Data[4] = data3 >> 8;
    Can_Send_Data[5] = data3;
    Can_Send_Data[6] = data4 >> 8;
    Can_Send_Data[7] = data4;

    HAL_CAN_AddTxMessage(hcan, &Tx_Message, Can_Send_Data, &send_mail_box);
}


void BMI088_Accel(int16_t data1, int16_t data2, 
											int16_t data3, int16_t data4,
											CAN_HandleTypeDef *hcan)//CANID:0x200 电机ID:1-4 (-10000,10000)
{
		CAN_TxHeaderTypeDef  Tx_Message;
		uint8_t	Can_Send_Data[8];
    uint32_t send_mail_box;
	
    Tx_Message.StdId = 0x148;
    Tx_Message.IDE = CAN_ID_STD;
    Tx_Message.RTR = CAN_RTR_DATA;
    Tx_Message.DLC = 0x08;
    Can_Send_Data[0] = data1 >> 8;
    Can_Send_Data[1] = data1;
    Can_Send_Data[2] = data2 >> 8;
    Can_Send_Data[3] = data2;
    Can_Send_Data[4] = data3 >> 8;
    Can_Send_Data[5] = data3;
    Can_Send_Data[6] = data4 >> 8;
    Can_Send_Data[7] = data4;

    HAL_CAN_AddTxMessage(hcan, &Tx_Message, Can_Send_Data, &send_mail_box);
}


