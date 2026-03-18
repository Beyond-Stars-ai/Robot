#include "Gimbal_CtoC.h"

BMI088_Init_typedef Can_BMI088_Data = {0};
BMI088_Init_typedef BigYaw_BMI088_Data = {0};
BMI088_Init_typedef SmallYaw_BMI088_Data = {0};
extern RC_ctrl_t global_rc_control; // 全局遥控器数据

static void CToC_MasterSendData(int16_t data1, int16_t data2,
                                int16_t data3, int16_t data4,
                                CAN_HandleTypeDef *hcan)
{
    CAN_TxHeaderTypeDef Tx_Message;
    uint8_t Can_Send_Data[8];
    uint32_t send_mail_box;

    Tx_Message.StdId = 0x149;
    Tx_Message.RTR = CAN_RTR_DATA; // 数据帧
    Tx_Message.IDE = CAN_ID_STD;   // 标准格式
    Tx_Message.DLC = 0x08;         // 8字节数据段
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

void Gimbal_CtoC_Remote(void)
{
    CToC_MasterSendData(global_rc_control.rc.ch[0], global_rc_control.rc.ch[1],
                        global_rc_control.rc.ch[2], global_rc_control.rc.ch[3], &hcan2);
}

void CToC_AngleProcess(uint32_t ID,uint8_t *Data,BMI088_Init_typedef *data)
{
	int16_t yaw = (int16_t)((Data[0] << 8) | Data[1]);
	int16_t pitch = (int16_t)((Data[2] << 8) | Data[3]);
	int16_t roll = (int16_t)((Data[4] << 8) | Data[5]);
	int16_t tmp = (int16_t)((Data[6] << 8) | Data[7]);

	data->Yaw = yaw / 100.0f;      // 转回 rad/s
	data->Pitch = pitch / 100.0f;
	data->Roll = roll / 100.0f;
	data->Temp = tmp / 10.0f;         // 转回 °C

}

// void CToC_GyroProcess(uint32_t ID,uint8_t *Data)
//{
//	int16_t gx = (int16_t)((Data[0] << 8) | Data[1]);
//	int16_t gy = (int16_t)((Data[2] << 8) | Data[3]);
//	int16_t gz = (int16_t)((Data[4] << 8) | Data[5]);
//	int16_t tmp = (int16_t)((Data[6] << 8) | Data[7]);
//
//	g_bmi088_rx_gyro[0] = gx / 100.0f;      // 转回 rad/s
//	g_bmi088_rx_gyro[1] = gy / 100.0f;
//	g_bmi088_rx_gyro[2] = gz / 100.0f;
//	g_bmi088_rx_temp = tmp / 10.0f;         // 转回 °C
// }

// void CToC_AccelProcess(uint32_t ID,uint8_t *Data)
//{
//	int16_t ax = (int16_t)((Data[0] << 8) | Data[1]);
//	int16_t ay = (int16_t)((Data[2] << 8) | Data[3]);
//	int16_t az = (int16_t)((Data[4] << 8) | Data[5]);
//	int16_t tmp = (int16_t)((Data[6] << 8) | Data[7]);
//
//	g_bmi088_rx_accel[0] = ax / 100.0f;     // 转回 m/s²
//	g_bmi088_rx_accel[1] = ay / 100.0f;
//	g_bmi088_rx_accel[2] = az / 100.0f;
//	g_bmi088_rx_temp = tmp / 10.0f;
// }