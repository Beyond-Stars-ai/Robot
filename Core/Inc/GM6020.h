#ifndef __GM6020_H
#define __GM6020_H

typedef enum
{
	GM6020_1=0x205,//ID1
	GM6020_2=0x206,//ID2
	GM6020_3=0x207,//ID3
	GM6020_4=0x208,//ID4
	GM6020_5=0x209,//ID5
	GM6020_6=0x20A,//ID6
	GM6020_7=0x20B,//ID7
}GM6020_ID;//GM6020电机ID号枚举

typedef struct
{
	uint16_t Angle;//GM6020电机机械角度
	uint8_t First_Flag;//GM6020电机首次接收标志位
	int64_t r;//GM6020电机转过圈数(默认圈数只会出现0,1)
	int64_t Position;//GM6020电机角度位置原始数据
	int16_t Speed;//GM6020电机转速
	int16_t Current;//GM6020电机实际转矩电流
	uint8_t Temperature;//GM6020电机电机温度
}GM6020_Motor;//GM6020电机状态结构体

extern GM6020_Motor GM6020_MotorStatus[];//GM6020电机状态数组

uint8_t GM6020_CAN1SetLIDVoltage(int16_t Voltage1,int16_t Voltage2,int16_t Voltage3,int16_t Voltage4);//CAN1总线设置GM6020低位ID电压
uint8_t GM6020_CAN1SetHIDVoltage(int16_t Voltage5,int16_t Voltage6,int16_t Voltage7);//CAN1总线设置GM6020高位ID电压
uint8_t GM6020_CAN2SetLIDVoltage(int16_t Voltage1,int16_t Voltage2,int16_t Voltage3,int16_t Voltage4);//CAN2总线设置GM6020低位ID电压
uint8_t GM6020_CAN2SetHIDVoltage(int16_t Voltage5,int16_t Voltage6,int16_t Voltage7);//CAN2总线设置GM6020高位ID电压
void GM6020_CANDataProcess(GM6020_ID ID,uint8_t *Data);//GM6020数据处理

#endif
