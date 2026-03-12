#ifndef CHASSIS_CTOC_H
#define CHASSIS_CTOC_H
#include <stdint.h>
#include "BMI088.h"
typedef struct
{
	int16_t Remote_R_RL;//通道0-右摇杆左右(右为大),范围364(最左端)~1684(最右端),默认值1024(中间)
	int16_t Remote_R_UD;//通道1-右摇杆上下(上为大),范围364(最下端)~1684(最上端),默认值1024(中间)
	int16_t Remote_L_RL;//通道2-左摇杆左右(右为大),范围364(最左端)~1684(最右端),默认值1024(中间)
	int16_t Remote_L_UD;//通道3-左摇杆上下(上为大),范围364(最下端)~1684(最上端),默认值1024(中间)
	
	uint8_t Remote_LS;//S1-左侧拨动开关,范围1~3,上为1,下为2,中间为3
	uint8_t Remote_RS;//S2-右侧拨动开关,范围1~3,上为1,下为2,中间为3
	
	int16_t Remote_Mouse_RL;//鼠标X轴-鼠标左右速度,范围-32768~32767,向右为正,向左为负,静止值为0
	int16_t Remote_Mouse_DU;//鼠标Y轴-鼠标前后速度,范围-32768~32767,向后为正,向前为负,静止值为0
	int16_t Remote_Mouse_Wheel;//鼠标Z轴-鼠标滚轮速度,范围-32768~32767,向前为正,向后为负,静止值为0
	uint8_t Remote_Mouse_KeyL;//鼠标左键,按下为1,未按下为0
	uint8_t Remote_Mouse_KeyR;//鼠标右键,按下为1,未按下为0
	
	uint8_t Remote_Key_W;//键盘W键,按下为1,未按下为0
	uint8_t Remote_Key_S;//键盘S键,按下为1,未按下为0
	uint8_t Remote_Key_A;//键盘A键,按下为1,未按下为0
	uint8_t Remote_Key_D;//键盘D键,按下为1,未按下为0
	uint8_t Remote_Key_Q;//键盘Q键,按下为1,未按下为0
	uint8_t Remote_Key_E;//键盘E键,按下为1,未按下为0
	uint8_t Remote_Key_Shift;//键盘Shift键,按下为1,未按下为0
	uint8_t Remote_Key_Ctrl;//键盘Ctrl键,按下为1,未按下为0
	uint8_t Remote_KeyPush_Ctrl;//按下键盘Ctrl键,按下时0,1切换
	uint8_t Remote_KeyPush_Shift;//按下键盘Shift键,按下时0,1切换
	
	int16_t Remote_ThumbWheel;//保留字段-遥控器拨轮,范围-3278(最上端)~1684(最下端),默认值1024
}Remote_Data;//遥控器接收结构体

void CToC_CANDataProcess(uint32_t ID,uint8_t *Data);
void Chassis_CtoC_BMI088(BMI088_Init_typedef *data);
#endif //CHASSIS_CTOC_H	
