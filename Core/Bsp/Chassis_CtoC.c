#include "Chassis_CtoC.h"
#include "can.h"
#include "BMI088.h"
#include "BMI088driver.h"
Remote_Data Remote_RxData;//遥控器接收数据
extern BMI088_Init_typedef BMI088_Data;

void CToC_CANDataProcess(uint32_t ID,uint8_t *Data)
{
	if(ID==0x149)//接收遥控器拨杆数据
	{
		Remote_RxData.Remote_R_RL=(int16_t)((uint16_t)Data[0]<<8 | Data[1]);//右摇杆右左
		Remote_RxData.Remote_R_UD=(int16_t)((uint16_t)Data[2]<<8 | Data[3]);//右摇杆上下
		Remote_RxData.Remote_L_RL=(int16_t)((uint16_t)Data[4]<<8 | Data[5]);//左摇杆右左
		Remote_RxData.Remote_L_UD=(int16_t)((uint16_t)Data[6]<<8 | Data[7]);//左摇杆上下
	}
//	else if(ID==0x189)//接收遥控器控制数据
//	{
//		Remote_Status=Data[0];//遥控器连接状态
//		Remote_RxData.Remote_RS=Data[1];//遥控器右侧拨动开关
//		Remote_RxData.Remote_KeyPush_Ctrl=Data[2];//键盘Ctrl状态
//		Remote_RxData.Remote_KeyPush_Shift=Data[3];//键盘Shift状态
//		Remote_StartFlag=Data[4];//遥控器启动标志位
//		Remote_RxData.Remote_LS=Data[5];//遥控器左侧拨动开关
//	}
}

void Chassis_CtoC_BMI088(BMI088_Init_typedef *data)
{
	
	BMI088_GetData(data);
	
	//定义缩放系数
	float GYRO_SCALE  = 100.0f;   // rad/s → 0.01 rad/s per LSB
	float ACCEL_SCALE = 100.0f;  // m/s² → 0.001 m/s² per LSB
	float TEMP_SCALE  = 10.0f;    // °C → 0.1°C per LSB
	
	int16_t d1, d2, d3, d4;
	
	// 3. 根据 CAN ID 决定发送内容
	int16_t yaw		= (int16_t)(data->Yaw * 100);
	int16_t pitch = (int16_t)(data->Pitch * 100);
	int16_t roll 	= (int16_t)(data->Roll * 10);
	int16_t tmp 	= (int16_t)(data->Temp * TEMP_SCALE);
	
	BMI088_Can_Angle(yaw, pitch, roll, tmp, &hcan2);

}



