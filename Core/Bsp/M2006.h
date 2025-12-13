#ifndef __M2006_H
#define __M2006_H

typedef enum
{
	M2006_1=0x201,//ID1
	M2006_2=0x202,//ID2
	M2006_3=0x203,//ID3
	M2006_4=0x204,//ID4
	M2006_5=0x205,//ID5
	M2006_6=0x206,//ID6
	M2006_7=0x207,//ID7
	M2006_8=0x208,//ID8
}M2006_ID;//M2006电机ID号枚举

typedef struct
{
	uint8_t First_Flag;//M2006电机首次接收标志位
	int64_t Rotor_r;//M2006电机转子转过圈数
	uint16_t RawRotorAngle;//M2006电机转子机械角度原始数据(范围0~8191,对应0~360°,注意:8192对应360°)
	float RotorAngle;//M2006电机转子机械角度(单位°)
	int64_t RawRotorPosition;//M2006电机转子角度位置原始数据
	float RotorPosition;//M2006电机转子角度位置(单位°)
	int16_t RotorSpeed;//M2006电机转子转速(单位RPM)
	
	int64_t Shaft_r;//M2006电机转轴转过圈数
	float ShaftAngle;//M2006电机转轴机械角度(单位°)
	float ShaftPosition;//M2006电机转轴角度位置(单位°)
	float ShaftSpeed;//M2006电机转轴转速(单位RPM)
	
	int16_t RawCurrent;//M2006电机转矩电流原始数据(范围-10000~10000,对应-10A~10A,注意:10000对应10A)
	float Current;//M2006电机转矩电流(单位A)
	
	float Power;//M2006电机功率(单位W)
}M2006_Motor;//M2006电机状态结构体(减速比36:1,转矩系数0.18N·m/A)

extern M2006_Motor M2006_MotorStatus[];//M2006电机状态数组

uint8_t M2006_CANSetLIDCurrent(int16_t Current1,int16_t Current2,int16_t Current3,int16_t Current4);//CAN总线设置M2006低位ID电流
uint8_t M2006_CANSetHIDCurrent(int16_t Current5,int16_t Current6,int16_t Current7,int16_t Current8);//CAN总线设置M2006高位ID电流
void M2006_CANDataProcess(M2006_ID ID,uint8_t *Data);//M2006数据处理

#endif
